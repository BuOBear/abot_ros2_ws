import copy
import math

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from abot_tracking.vision_follower import TrackingPolicy, stamp_seconds


def policy():
    return TrackingPolicy('red', kp_yaw=0.5, max_yaw_rate=0.25,
                          center_deadband=0.05, min_bbox_area_px=500,
                          camera_info_timeout=0.5, detection_timeout=0.3)


def set_stamp(header, value):
    header.stamp.sec = int(value)
    header.stamp.nanosec = round((value - int(value)) * 1e9)
    header.frame_id = 'camera_optical_frame'


def camera(stamp=100.0, width=640, height=480):
    info = CameraInfo()
    set_stamp(info.header, stamp)
    info.width, info.height = width, height
    return info


def detections(stamp=100.1, x=480, y=240, class_id='red', width=40,
               height=40, score=0.8):
    array = Detection2DArray()
    set_stamp(array.header, stamp)
    if x is not None:
        item = Detection2D()
        item.header = copy.deepcopy(array.header)
        item.bbox.center.position.x = float(x)
        item.bbox.center.position.y = float(y)
        item.bbox.size_x = float(width)
        item.bbox.size_y = float(height)
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = class_id
        result.hypothesis.score = score
        item.results.append(result)
        array.detections.append(item)
    return array


def test_selected_target_turns_only_while_camera_and_detection_are_fresh():
    follower = policy()
    follower.receive_detections(detections(), 100.1, 100.1)
    assert follower.yaw_rate(100.1, 100.1) == 0.0
    follower.receive_camera_info(camera(100.1), 100.1, 100.1)
    follower.receive_detections(detections(), 100.1, 100.1)
    assert follower.yaw_rate(100.1, 100.1) == -0.25
    assert follower.yaw_rate(100.41, 100.41) == 0.0  # Detection timeout.
    follower.receive_camera_info(camera(100.43), 100.43, 100.43)
    follower.receive_detections(detections(100.43, x=320), 100.43, 100.43)
    assert follower.yaw_rate(100.43, 100.43) == 0.0  # Centered target.
    follower.receive_camera_info(camera(100.44), 100.44, 100.44)
    follower.receive_detections(detections(100.44, x=None), 100.44, 100.44)
    assert follower.yaw_rate(100.44, 100.44) == 0.0  # Explicit loss.


def test_replay_wrong_class_and_bad_geometry_do_not_move():
    follower = policy()
    follower.receive_camera_info(camera(100.1), 100.1, 100.1)
    follower.receive_detections(detections(class_id='green'), 100.1, 100.1)
    assert follower.yaw_rate(100.1, 100.1) == 0.0
    follower.receive_camera_info(camera(100.2), 100.2, 100.2)
    follower.receive_detections(detections(100.2, x=480, width=math.nan), 100.2, 100.2)
    assert follower.yaw_rate(100.2, 100.2) == 0.0
    follower.receive_camera_info(camera(100.3), 100.3, 100.3)
    follower.receive_detections(detections(100.3, x=480), 100.3, 100.3)
    assert follower.yaw_rate(100.3, 100.3) == -0.25
    follower.receive_detections(detections(100.3, x=160), 100.4, 100.4)
    assert follower.yaw_rate(100.4, 100.4) == -0.25  # Same stamp cannot replace target.
    follower.receive_camera_info(camera(100.5, width=320), 100.5, 100.5)
    assert follower.yaw_rate(100.5, 100.5) == 0.0  # Mode change clears old error.
    follower.receive_detections(detections(100.6, x=160), 100.6, 100.6)
    assert follower.yaw_rate(100.6, 100.6) == 0.0


def test_future_stamps_and_stale_camera_info_fail_closed():
    follower = policy()
    follower.receive_camera_info(camera(stamp=101.0), 100.0, 100.0)
    follower.receive_detections(detections(), 100.1, 100.1)
    assert follower.yaw_rate(100.1, 100.1) == 0.0
    follower.receive_camera_info(camera(100.1), 100.1, 100.1)
    follower.receive_detections(detections(), 100.1, 100.1)
    assert follower.yaw_rate(100.51, 100.51) == 0.0


def test_detection_requires_same_camera_info_acquisition_stamp():
    follower = policy()
    follower.receive_camera_info(camera(100.0), 100.0, 100.0)
    follower.receive_detections(detections(100.1), 100.1, 100.1)
    assert follower.yaw_rate(100.1, 100.1) == 0.0
    follower.receive_camera_info(camera(100.2), 100.2, 100.2)
    follower.receive_detections(detections(100.2), 100.2, 100.2)
    assert follower.yaw_rate(100.2, 100.2) == -0.25


def test_delayed_detection_still_finds_camera_info_at_high_frame_rate():
    follower = policy()
    for frame in range(32):
        acquired_at = 100.0 + frame / 120.0
        info = camera(acquired_at)
        received_at = stamp_seconds(info.header.stamp)
        follower.receive_camera_info(info, received_at, received_at)
    # A detector can finish after many newer CameraInfo messages; the
    # acquisition stamp still identifies the right image dimensions.
    follower.receive_detections(detections(100.0), 100.26, 100.26)
    assert follower.yaw_rate(100.26, 100.26) == -0.25


def test_camera_info_history_is_bounded_and_expires_by_time():
    follower = policy()
    for frame in range(270):
        info = camera(100.0 + frame / 1000.0)
        received_at = stamp_seconds(info.header.stamp)
        follower.receive_camera_info(info, received_at, received_at)
    assert len(follower.camera_history) == 256
    assert (100, 0) not in follower.camera_history

    follower.receive_camera_info(camera(100.8), 100.8, 100.8)
    assert len(follower.camera_history) == 1


def test_selected_non_color_class_and_other_classes_do_not_compete():
    follower = TrackingPolicy('tag:apriltag_36h11:7', kp_yaw=0.5,
                              max_yaw_rate=0.25, center_deadband=0.05,
                              min_bbox_area_px=500,
                              camera_info_timeout=0.5, detection_timeout=0.3)
    follower.receive_camera_info(camera(100.0), 100.0, 100.0)
    follower.receive_detections(detections(100.0, class_id='person'), 100.0, 100.0)
    assert follower.yaw_rate(100.0, 100.0) == 0.0
    follower.receive_camera_info(camera(100.1), 100.1, 100.1)
    follower.receive_detections(
        detections(100.1, class_id='tag:apriltag_36h11:7'), 100.1, 100.1)
    assert follower.yaw_rate(100.1, 100.1) == -0.25


def test_mixed_frame_and_untrusted_hypothesis_cannot_drive():
    follower = policy()
    follower.receive_camera_info(camera(100.0), 100.0, 100.0)
    mixed_frame = detections(100.0)
    mixed_frame.detections[0].header.frame_id = 'other_optical_frame'
    follower.receive_detections(mixed_frame, 100.0, 100.0)
    assert follower.yaw_rate(100.0, 100.0) == 0.0

    follower.receive_camera_info(camera(100.1), 100.1, 100.1)
    mixed_stamp = detections(100.1)
    set_stamp(mixed_stamp.detections[0].header, 100.0)
    follower.receive_detections(mixed_stamp, 100.1, 100.1)
    assert follower.yaw_rate(100.1, 100.1) == 0.0

    follower.receive_camera_info(camera(100.2), 100.2, 100.2)
    negative_score = detections(100.2, score=-0.1)
    follower.receive_detections(negative_score, 100.2, 100.2)
    assert follower.yaw_rate(100.2, 100.2) == 0.0

    follower.receive_camera_info(camera(100.3), 100.3, 100.3)
    invalid_score = detections(100.3, score=math.nan)
    follower.receive_detections(invalid_score, 100.3, 100.3)
    assert follower.yaw_rate(100.3, 100.3) == 0.0

    follower.receive_camera_info(camera(100.4), 100.4, 100.4)
    truncated_box = detections(100.4, x=635, width=40)
    follower.receive_detections(truncated_box, 100.4, 100.4)
    assert follower.yaw_rate(100.4, 100.4) == 0.0

    # A zero HOG margin is valid and must not exclude a person detection.
    follower.receive_camera_info(camera(100.5), 100.5, 100.5)
    follower.receive_detections(detections(100.5, score=0.0), 100.5, 100.5)
    assert follower.yaw_rate(100.5, 100.5) == -0.25


def test_clock_rewind_stops_then_accepts_new_epoch():
    follower = policy()
    follower.receive_camera_info(camera(100.0), 100.0, 100.0)
    follower.receive_detections(detections(100.0), 100.0, 100.0)
    assert follower.yaw_rate(100.0, 100.0) == -0.25

    # The steady clock advances even when the ROS /clock restarts.
    assert follower.yaw_rate(100.1, 1.0) == 0.0
    follower.receive_camera_info(camera(1.1), 100.2, 1.1)
    follower.receive_detections(detections(1.1), 100.2, 1.1)
    assert follower.yaw_rate(100.2, 1.1) == -0.25


def test_reception_deadline_stops_even_if_ros_clock_is_paused():
    follower = policy()
    follower.receive_camera_info(camera(100.0), 100.0, 100.0)
    follower.receive_detections(detections(100.0), 100.0, 100.0)
    assert follower.yaw_rate(100.0, 100.0) == -0.25
    assert follower.yaw_rate(100.31, 100.0) == 0.0
