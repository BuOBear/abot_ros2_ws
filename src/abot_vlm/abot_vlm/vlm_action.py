"""ROS 2 Action server for on-demand visual language requests."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import Image

from abot_vlm_interfaces.action import AnalyzeImage
from .worker import ArkBackend, Backend, BackendUnavailable, FrameCache, InvalidImage, SingleFlightWorker


class VlmAction(Node):
    def __init__(self, backend: Backend | None = None, parameter_overrides=None) -> None:
        super().__init__('vlm_action', parameter_overrides=parameter_overrides)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('default_max_frame_age_sec', 1.0)
        self.declare_parameter('default_timeout_sec', 15.0)
        self.declare_parameter('maximum_frame_age_sec', 5.0)
        self.declare_parameter('max_timeout_sec', 60.0)
        try:
            self._default_age = self._positive_parameter('default_max_frame_age_sec', 60.0)
            self._default_timeout = self._positive_parameter('default_timeout_sec', 120.0)
            self._maximum_age = self._positive_parameter('maximum_frame_age_sec', 60.0)
            self._maximum_timeout = self._positive_parameter('max_timeout_sec', 120.0)
            if self._default_age > self._maximum_age:
                raise ValueError('default_max_frame_age_sec exceeds maximum_frame_age_sec')
            if self._default_timeout > self._maximum_timeout:
                raise ValueError('default_timeout_sec exceeds max_timeout_sec')
        except Exception:
            super().destroy_node()
            raise
        self._frames = FrameCache()
        self._worker = SingleFlightWorker(backend or ArkBackend())
        group = ReentrantCallbackGroup()
        self._image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._frames.update, qos_profile_sensor_data, callback_group=group)
        self._server = ActionServer(
            self, AnalyzeImage, '/vlm/analyze',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=group,
        )
        self.get_logger().info('VLM Action ready; requests are explicit and single-flight.')

    def _positive_parameter(self, name: str, hard_max: float) -> float:
        raw = self.get_parameter(name).value
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f'{name} must be numeric')
        value = float(raw)
        if not math.isfinite(value) or value <= 0 or value > hard_max:
            raise ValueError(f'{name} must be finite and in (0, {hard_max}]')
        return value

    def _goal(self, request: AnalyzeImage.Goal) -> GoalResponse:
        if not request.prompt.strip() or len(request.prompt) > 4096:
            return GoalResponse.REJECT
        for value in (request.max_frame_age_sec, request.timeout_sec):
            if not math.isfinite(value) or value < 0:
                return GoalResponse.REJECT
        if request.max_frame_age_sec > self._maximum_age:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if self._worker.reserve() is not None else GoalResponse.REJECT

    def _cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def _result(code: int, text: str = '') -> AnalyzeImage.Result:
        result = AnalyzeImage.Result()
        result.code = code
        result.text = text
        return result

    async def _execute(self, goal_handle) -> AnalyzeImage.Result:
        token = self._worker.current_token()
        if token is None:
            goal_handle.abort()
            return self._result(AnalyzeImage.Result.BACKEND_ERROR)
        try:
            return await self._execute_reserved(goal_handle, token)
        except Exception:
            # Never release the reservation while an accepted goal has no
            # terminal Action outcome, even if an unexpected callback fails.
            goal_handle.abort()
            return self._result(AnalyzeImage.Result.BACKEND_ERROR)
        finally:
            # A finished backend alone cannot admit the next goal. Keep this
            # reservation until the matching Action has reached its terminal
            # state and this callback has handled its result.
            self._worker.finish(token)

    async def _execute_reserved(self, goal_handle, token: int) -> AnalyzeImage.Result:
        requested_age = float(goal_handle.request.max_frame_age_sec)
        requested_timeout = float(goal_handle.request.timeout_sec)
        max_age = requested_age or self._default_age
        timeout = min(requested_timeout or self._default_timeout, self._maximum_timeout)

        frame = self._frames.snapshot()
        if frame is None:
            goal_handle.abort()
            return self._result(AnalyzeImage.Result.NO_FRAME)
        if time.monotonic() - frame.received_at > max_age:
            goal_handle.abort()
            return self._result(AnalyzeImage.Result.STALE_FRAME)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(AnalyzeImage.Result.CANCELED)
        try:
            available = self._worker.backend.available()
        except Exception:
            goal_handle.abort()
            return self._result(AnalyzeImage.Result.BACKEND_ERROR)
        if not available:
            goal_handle.abort()
            return self._result(AnalyzeImage.Result.UNAVAILABLE)

        feedback = AnalyzeImage.Feedback()
        feedback.stage = 'analyzing'
        goal_handle.publish_feedback(feedback)
        try:
            future = self._worker.submit(token, goal_handle.request.prompt, frame.image, timeout)
        except Exception:
            goal_handle.abort()
            return self._result(AnalyzeImage.Result.BACKEND_ERROR)

        deadline = time.monotonic() + timeout
        while True:
            if goal_handle.is_cancel_requested:
                self._worker.abort_active(token)
                goal_handle.canceled()
                return self._result(AnalyzeImage.Result.CANCELED)
            if time.monotonic() >= deadline:
                self._worker.abort_active(token)
                goal_handle.abort()
                return self._result(AnalyzeImage.Result.TIMEOUT)
            if future.done():
                try:
                    answer = future.result()
                except BackendUnavailable:
                    goal_handle.abort()
                    return self._result(AnalyzeImage.Result.UNAVAILABLE)
                except InvalidImage:
                    goal_handle.abort()
                    return self._result(AnalyzeImage.Result.INVALID_IMAGE)
                except TimeoutError:
                    goal_handle.abort()
                    return self._result(AnalyzeImage.Result.TIMEOUT)
                except Exception:
                    # Backend exceptions are intentionally not returned to clients:
                    # SDK/HTTP messages can contain request metadata.
                    goal_handle.abort()
                    return self._result(AnalyzeImage.Result.BACKEND_ERROR)
                goal_handle.succeed()
                return self._result(AnalyzeImage.Result.OK, answer)
            # Humble's rclpy executor drives coroutine callbacks without an
            # asyncio event loop. A ROS timer wakes a native rclpy Future.
            wake = Future()
            timer = self.create_timer(
                0.02, lambda: wake.set_result(None) if not wake.done() else None)
            try:
                await wake
            finally:
                self.destroy_timer(timer)

    def destroy_node(self):
        self._server.destroy()
        self._worker.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmAction()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
