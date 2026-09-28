"""Explicit, asynchronous Nav2 mission runner; never publishes velocity."""

import json
import math
import time

import rclpy
from abot_control_interfaces.srv import AcquireNav, ReleaseNav
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
import yaml

from .state_machine import Mission, parse_route


class MissionNode(Node):
    def __init__(self):
        super().__init__('abot_mission')
        self.declare_parameter('route_file', '')
        self.declare_parameter('map_id', '')
        self.declare_parameter('localization_max_age_sec', 1.0)
        self.declare_parameter('preparing_timeout_sec', 10.0)
        path = str(self.get_parameter('route_file').value).strip()
        configured_map = str(self.get_parameter('map_id').value).strip()
        self.machine = None
        if path:
            with open(path, encoding='utf-8') as route_stream:
                route = parse_route(yaml.safe_load(route_stream))
            if not configured_map or route.map_id != configured_map:
                raise ValueError('map_id parameter must match the validated route map_id')
            self.machine = Mission(route)
        self.max_localization_age = float(self.get_parameter('localization_max_age_sec').value)
        if not math.isfinite(self.max_localization_age) or self.max_localization_age <= 0:
            raise ValueError('localization_max_age_sec must be positive')
        self.preparing_timeout = float(self.get_parameter('preparing_timeout_sec').value)
        if not math.isfinite(self.preparing_timeout) or self.preparing_timeout <= 0:
            raise ValueError('preparing_timeout_sec must be positive')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        mode_state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/control/mode_state', self._on_mode_state,
                                 mode_state_qos)
        self.create_subscription(UInt64, '/control/mode_epoch', self._on_mode_epoch,
                                 mode_state_qos)
        self.acquire_client = self.create_client(AcquireNav, '/control/acquire_nav')
        self.release_client = self.create_client(ReleaseNav, '/control/release_nav')
        self.status_publisher = self.create_publisher(String, '/mission/status', qos)
        self.start_service = self.create_service(Trigger, '/mission/start', self._start)
        self.cancel_service = self.create_service(Trigger, '/mission/cancel', self._cancel)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.lifecycle_active_at = None
        self.lifecycle_pending = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_handle = None
        self.cancel_future = None
        self.cancel_requested_at = None
        self.last_mode = None
        self.last_epoch = None
        self.mode_version = 0
        self.epoch_version = 0
        self.acquire_mode_version = 0
        self.acquire_epoch_version = 0
        self.acquire_future = None
        self.acquire_requested_at = None
        self.acquire_expired = False
        self.acquire_uncertain = False
        self.lease_id = None
        self.lease_epoch = None
        self.lease_confirmed = False
        self.lease_acquired_at = None
        self.release_future = None
        self.release_requested_at = None
        self.release_mode_version = 0
        self.release_epoch_version = 0
        self.release_expected_epoch = None
        self.release_response_success = False
        self.release_blocked = False
        self.release_failure_detail = None
        self.preparing_since = None
        self._last_phase = None
        self._last_lifecycle_request = 0.0
        self._status_timer = self.create_timer(0.1, self._tick)
        self._report()
        if self.machine is None:
            self.get_logger().warn('No route_file configured; /mission/start will reject requests')

    def _report(self):
        machine = self.machine
        status = {
            'state': machine.phase if machine else 'unconfigured',
            'step': machine.step.name if machine and machine.index < len(machine.route.steps) else '',
            'step_index': machine.index if machine else -1,
            'goal_token': machine.active_token if machine else None,
            'detail': machine.detail if machine else 'route_file_required',
            'map_id': machine.route.map_id if machine else '',
            'lease_pending': (self.acquire_future is not None or self.release_future is not None
                              or self.lease_id is not None),
            'lease_blocked': self.release_blocked,
            'acquire_uncertain': self.acquire_uncertain,
        }
        self.status_publisher.publish(String(data=json.dumps(status, sort_keys=True)))
        self._last_phase = status['state']

    def _on_mode_state(self, msg):
        self.mode_version += 1
        self.last_mode = msg.data
        if self.release_requested_at is not None:
            self._try_finish_release()
            return
        if (self.lease_id is not None and msg.data != 'nav' and
                (self.lease_confirmed or self.mode_version > self.acquire_mode_version)):
            self._lease_lost('mode_preempted')
        else:
            self._maybe_dispatch()

    def _on_mode_epoch(self, msg):
        self.epoch_version += 1
        self.last_epoch = msg.data
        if self.release_requested_at is not None:
            self._try_finish_release()
            return
        if self.lease_id is not None and self.lease_confirmed and msg.data != self.lease_epoch:
            self._lease_lost('mode_epoch_changed')
        elif self.lease_id is not None and msg.data > self.lease_epoch:
            self._lease_lost('mode_epoch_changed')
        else:
            self._maybe_dispatch()

    def _start(self, _request, response):
        if self.machine is None:
            response.message = 'configure route_file and matching map_id first'
        elif (self.acquire_future is not None or self.release_future is not None or
              self.lease_id is not None or self.release_blocked or self.acquire_uncertain):
            response.message = 'Nav lease pending, held, or release unconfirmed'
        elif self.machine.start():
            self.preparing_since = time.monotonic()
            response.success = True
            response.message = 'accepted; waiting for Nav2, localization, and Nav lease'
            self._report()
        else:
            response.message = 'mission busy or previous goal result unconfirmed'
        return response

    def _cancel(self, _request, response):
        response.success = self._request_cancel('operator')
        response.message = ('cancel requested; await goal result' if response.success else
                            'no active mission')
        return response

    def _request_cancel(self, reason):
        if not self.machine or not self.machine.request_cancel(reason):
            return False
        self._begin_release()
        if self.machine.active_token is not None:
            self.cancel_requested_at = time.monotonic()
            self._send_cancel_if_possible()
        self._report()
        return True

    def _lease_lost(self, reason):
        if self.machine and self.machine.phase in (
                'preparing', 'awaiting_goal', 'navigating', 'canceling'):
            self._request_cancel(reason)
        else:
            self._begin_release()

    def _begin_acquire(self, now):
        if not self.acquire_client.service_is_ready():
            return
        self.acquire_mode_version = self.mode_version
        self.acquire_epoch_version = self.epoch_version
        self.acquire_requested_at = now
        self.acquire_expired = False
        try:
            self.acquire_future = self.acquire_client.call_async(AcquireNav.Request())
        except Exception as exc:
            self.acquire_requested_at = None
            self.machine.phase = 'failed'
            self.machine.detail = f'acquire_request_error: {exc}'
            self._report()
            return
        self.acquire_future.add_done_callback(self._on_acquire)

    def _on_acquire(self, future):
        if future is not self.acquire_future:
            return
        self.acquire_future = None
        self.acquire_requested_at = None
        try:
            reply = future.result()
        except Exception as exc:
            # The service may have granted a lease before its reply failed.
            # Without the ID, there is no safe ReleaseNav request.
            self.acquire_uncertain = True
            self.machine.phase = 'failed'
            self.machine.detail = f'acquire_service_error: {exc}; lease outcome unknown'
            self._report()
            return
        if not reply.success:
            if self.machine.phase == 'preparing':
                self.machine.phase = 'failed'
                self.machine.detail = f'acquire_rejected: {reply.message}'
            self._report()
            return
        if reply.lease_id == 0:
            self.acquire_uncertain = True
            self.machine.phase = 'failed'
            self.machine.detail = 'acquire_invalid_lease_id; lease outcome unknown'
            self._report()
            return
        self.lease_id = reply.lease_id
        self.lease_epoch = reply.epoch
        self.lease_acquired_at = time.monotonic()
        self.lease_confirmed = False
        if self.acquire_expired or self.machine.phase != 'preparing':
            self._begin_release()
        elif (self.last_epoch is not None and self.last_epoch > self.lease_epoch) or (
                self.mode_version > self.acquire_mode_version and self.last_mode != 'nav'):
            self._lease_lost('mode_preempted_before_ack')
        else:
            self._maybe_dispatch()
        self._report()

    def _begin_release(self):
        if (self.lease_id is None or self.release_requested_at is not None or
                self.release_blocked):
            return
        request = ReleaseNav.Request()
        request.lease_id = self.lease_id
        self.release_requested_at = time.monotonic()
        self.release_mode_version = self.mode_version
        self.release_epoch_version = self.epoch_version
        self.release_expected_epoch = (self.lease_epoch + 1) & ((1 << 64) - 1)
        self.release_response_success = False
        try:
            self.release_future = self.release_client.call_async(request)
        except Exception as exc:
            self._release_failed(f'release_request_error: {exc}')
            return
        self.release_future.add_done_callback(self._on_release)

    def _release_failed(self, detail):
        self.release_blocked = True
        self.release_failure_detail = detail
        if self.machine.active_token is not None:
            self.machine.request_cancel('release_unconfirmed')
            self._send_cancel_if_possible()
        else:
            self.machine.phase = 'failed'
        self.machine.detail = detail
        self._report()

    def _on_release(self, future):
        if future is not self.release_future:
            return
        self.release_future = None
        try:
            reply = future.result()
        except Exception as exc:
            self._release_failed(f'release_service_error: {exc}')
            return
        if not reply.success:
            self._release_failed(f'release_rejected: {reply.message}')
            return
        self.release_response_success = True
        self._try_finish_release()

    def _try_finish_release(self):
        if not self.release_response_success or self.release_requested_at is None:
            return
        # These samples must follow this ReleaseNav request. ROS topics have
        # independent delivery order, so the Service response alone is not a
        # barrier for both transient-local state topics.
        mode_new = self.mode_version > self.release_mode_version
        epoch_new = self.epoch_version > self.release_epoch_version
        if ((mode_new and self.last_mode not in ('nav', 'disabled')) or
                (epoch_new and self.last_epoch != self.release_expected_epoch)):
            self._release_failed('release_state_mismatch; external mode change suspected')
            return
        if not (mode_new and epoch_new and self.last_mode == 'disabled' and
                self.last_epoch == self.release_expected_epoch):
            return
        self.release_requested_at = None
        self.lease_id = None
        self.lease_epoch = None
        self.lease_confirmed = False
        self.lease_acquired_at = None
        self.release_response_success = False
        self.release_expected_epoch = None
        self.release_blocked = False
        self.release_failure_detail = None
        if self.machine.phase == 'preparing':
            self.preparing_since = time.monotonic()
        if self.machine.phase == 'observing':
            self.machine.start_observation(time.monotonic())
        self._report()

    def _maybe_dispatch(self):
        if (not self.machine or self.machine.phase != 'preparing' or self.lease_id is None or
                self.acquire_future is not None or self.release_future is not None or
                self.release_blocked or self.lease_confirmed):
            return
        if (self.mode_version <= self.acquire_mode_version or
                self.epoch_version <= self.acquire_epoch_version or
                self.last_mode != 'nav' or self.last_epoch != self.lease_epoch):
            return
        now = time.monotonic()
        if not self._ready(now):
            self.machine.phase = 'failed'
            self.machine.detail = 'readiness_lost_before_goal'
            self._begin_release()
            self._report()
            return
        self.lease_confirmed = True
        self._dispatch(now)

    def _send_cancel_if_possible(self):
        if self.goal_handle is None or self.cancel_future is not None:
            return
        token = self.machine.active_token
        if token is None:
            return
        try:
            self.cancel_future = self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.machine.detail = f'cancel_request_error: {exc}; goal result unconfirmed'
            self.get_logger().error(self.machine.detail)
            self._report()
            return
        self.cancel_future.add_done_callback(lambda future: self._on_cancel_reply(token, future))

    def _on_cancel_reply(self, token, future):
        if not self.machine or token != self.machine.active_token:
            return
        try:
            reply = future.result()
            wanted = bytes(self.goal_handle.goal_id.uuid)
            matched = any(bytes(item.goal_id.uuid) == wanted for item in reply.goals_canceling)
            if not matched:
                self.machine.detail = 'cancel_not_acknowledged; awaiting goal result'
                self.get_logger().error(self.machine.detail)
        except Exception as exc:
            self.machine.detail = f'cancel_service_error: {exc}; awaiting goal result'
            self.get_logger().error(self.machine.detail)
        self._report()

    def _poll_lifecycle(self, now):
        if self.lifecycle_pending is not None or now - self._last_lifecycle_request < 1.0:
            return
        if not self.lifecycle.service_is_ready():
            return
        self._last_lifecycle_request = now
        try:
            self.lifecycle_pending = self.lifecycle.call_async(GetState.Request())
        except Exception as exc:
            self.lifecycle_active_at = None
            self.get_logger().warn(f'Nav2 lifecycle request failed: {exc}')
            return
        self.lifecycle_pending.add_done_callback(self._on_lifecycle)

    def _on_lifecycle(self, future):
        self.lifecycle_pending = None
        try:
            self.lifecycle_active_at = (time.monotonic() if
                                        future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                                        else None)
        except Exception as exc:
            self.lifecycle_active_at = None
            self.get_logger().warn(f'Nav2 lifecycle state unavailable: {exc}')

    def _localized(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(tf.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            return 0 <= age <= self.max_localization_age
        except (TransformException, ValueError):
            return False

    def _ready(self, now):
        return (self.nav.server_is_ready() and self.lifecycle_active_at is not None and
                now - self.lifecycle_active_at <= 2.0 and self._localized())

    def _tick(self):
        machine = self.machine
        if machine is None:
            return
        now = time.monotonic()
        if machine.phase == 'preparing':
            if (self.release_requested_at is None and self.preparing_since is not None and
                    now - self.preparing_since > self.preparing_timeout):
                machine.phase = 'failed'
                machine.detail = 'preparing_timeout'
                if self.acquire_future is not None:
                    self.acquire_expired = True
                self._begin_release()
                self._report()
                return
            self._poll_lifecycle(now)
            if (self.acquire_future is not None and self.acquire_requested_at is not None and
                    now - self.acquire_requested_at > 2.0 and not self.acquire_expired):
                self.acquire_expired = True
                machine.phase = 'failed'
                machine.detail = 'acquire_timeout; awaiting late reply before restart'
                self._report()
            elif (self.lease_id is not None and not self.lease_confirmed and
                  self.lease_acquired_at is not None and
                  now - self.lease_acquired_at > 2.0):
                machine.phase = 'failed'
                machine.detail = 'lease_state_ack_timeout'
                self._begin_release()
                self._report()
            elif (self.lease_id is None and self.acquire_future is None and
                  self.release_future is None and not self.release_blocked and
                  self._ready(now)):
                self._begin_acquire(now)
        elif machine.phase in ('awaiting_goal', 'navigating', 'observing'):
            signal = machine.tick(now)
            if signal == 'cancel_goal':
                self._begin_release()
                self.cancel_requested_at = now
                self._send_cancel_if_possible()
                self._report()
        elif machine.phase == 'canceling' and self.cancel_requested_at is not None:
            if now - self.cancel_requested_at > 5.0 and 'unconfirmed' not in machine.detail:
                machine.detail = 'cancel_result_unconfirmed; goal handle remains reserved'
                self.get_logger().error(machine.detail)
                self._report()
        if (self.release_requested_at is not None and
                now - self.release_requested_at > 2.0 and not self.release_blocked):
            reason = ('release_state_ack_timeout' if self.release_response_success else
                      'release_timeout; awaiting reply before restart')
            self._release_failed(reason)

    def _dispatch(self, now):
        token = self.machine.dispatch(now)
        if token is None:
            return
        step = self.machine.step
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.machine.route.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = step.x
        pose.pose.position.y = step.y
        pose.pose.orientation.z = math.sin(step.yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(step.yaw_rad / 2.0)
        goal.pose = pose
        try:
            future = self.nav.send_goal_async(goal)
        except Exception as exc:
            self.machine.goal_response(token, False)
            self.machine.detail = f'send_goal_error: {exc}'
            self._begin_release()
            self._report()
            return
        future.add_done_callback(lambda finished: self._on_goal_response(token, finished))
        self._report()

    def _on_goal_response(self, token, future):
        if not self.machine or token != self.machine.active_token:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.machine.goal_response(token, False)
            self.machine.detail = f'send_goal_error: {exc}'
            self._begin_release()
            self._report()
            return
        if not self.machine.goal_response(token, handle.accepted):
            return
        if not handle.accepted:
            self._begin_release()
            self._report()
            return
        self.goal_handle = handle
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            self.machine.request_cancel('result_subscription_error')
            self.machine.detail = f'get_result_error: {exc}; goal result unconfirmed'
            self._begin_release()
            self._send_cancel_if_possible()
            self._report()
            return
        result_future.add_done_callback(lambda finished: self._on_result(token, finished))
        if self.machine.cancel_requested:
            self._send_cancel_if_possible()
        self._report()

    def _on_result(self, token, future):
        if not self.machine or token != self.machine.active_token:
            return
        try:
            status = future.result().status
        except Exception as exc:
            # The goal may still be running; retain its handle and block restarts.
            self.machine.detail = f'goal_result_error: {exc}; result unconfirmed'
            self.machine.request_cancel('result_subscription_error')
            self._begin_release()
            self._send_cancel_if_possible()
            self._report()
            return
        self.machine.goal_result(token, status, self.get_clock().now().nanoseconds * 1e-9)
        self.goal_handle = None
        self.cancel_future = None
        self.cancel_requested_at = None
        self._begin_release()
        if self.release_blocked and self.release_failure_detail:
            self.machine.detail = self.release_failure_detail
        self._report()

    def _on_detections(self, topic, msg):
        machine = self.machine
        if not machine or machine.phase != 'observing':
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        now = time.monotonic()
        for detection in msg.detections:
            for result in detection.results:
                if machine.observation(topic, result.hypothesis.class_id,
                                       result.hypothesis.score, stamp, now_ros, now):
                    if machine.phase == 'preparing':
                        self.preparing_since = now
                    self._report()
                    return

    def install_observation_subscriptions(self):
        self.observation_subscriptions = []
        if self.machine:
            for topic in sorted({step.observe.topic for step in self.machine.route.steps
                                 if step.observe is not None}):
                self.observation_subscriptions.append(self.create_subscription(
                    Detection2DArray, topic,
                    lambda msg, topic=topic: self._on_detections(topic, msg),
                    qos_profile_sensor_data))


def main():
    rclpy.init()
    node = None
    try:
        node = MissionNode()
        node.install_observation_subscriptions()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
