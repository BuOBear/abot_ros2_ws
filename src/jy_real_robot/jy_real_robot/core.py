"""Device-free perception math and temporal contracts; all lengths are metres."""
import math
from pathlib import Path
import threading

import cv2
import numpy as np


def local_weights(filename):
    path = Path(filename).expanduser()
    if not filename or not path.is_file() or path.suffix.lower() != '.pt':
        raise ValueError('A local YOLO11 .pt file is required; automatic downloads are disabled')
    return str(path.resolve())


def fresh(stamp_ns, now_ns, maximum_age, future_tolerance=0.05):
    age = (now_ns - stamp_ns) / 1e9
    return stamp_ns > 0 and -future_tolerance <= age <= maximum_age


class LatestFrame:
    """A bounded one-slot queue: inference never builds a backlog."""
    def __init__(self):
        self._lock = threading.Lock()
        self._value = None

    def put(self, value):
        with self._lock:
            self._value = value

    def take(self):
        with self._lock:
            result, self._value = self._value, None
            return result


def detections_from_result(result, width, height, minimum_confidence):
    """Return (label, score, x1, y1, x2, y2), without relabelling classes."""
    if result.boxes is None:
        return []
    boxes = result.boxes.cpu().numpy()
    output = []
    for rect, score, class_id in zip(boxes.xyxy, boxes.conf, boxes.cls):
        values = [*rect, score, class_id]
        if not all(math.isfinite(float(v)) for v in values):
            continue
        if not minimum_confidence <= float(score) <= 1.0:
            continue
        if float(class_id) != int(class_id) or int(class_id) < 0:
            continue
        try:
            label = str(result.names[int(class_id)])
        except (KeyError, IndexError):
            continue
        x1, y1, x2, y2 = map(float, rect)
        x1, x2 = max(0.0, x1), min(float(width), x2)
        y1, y2 = max(0.0, y1), min(float(height), y2)
        if x2 > x1 and y2 > y1:
            output.append((label, float(score), x1, y1, x2, y2))
    return output


def rotation_from_quaternion(xyzw):
    q = np.asarray(xyzw, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('invalid quaternion')
    norm = np.linalg.norm(q)
    if norm < 1e-8 or abs(norm - 1.0) > 0.02:
        raise ValueError('quaternion must be normalized')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def calibrated_ray(u, v, k, distortion, distortion_model):
    """Raw color-image pixel to an optical ray, with lens distortion removed."""
    matrix = np.asarray(k, dtype=float).reshape(3, 3)
    d = np.asarray(distortion, dtype=float)
    if not (np.all(np.isfinite(matrix)) and np.all(np.isfinite(d))
            and math.isfinite(u) and math.isfinite(v)):
        raise ValueError('non-finite camera calibration/pixel')
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError('camera is not calibrated')
    # OpenCV's undistortPoints uses fx/fy/cx/cy and ignores the other K entries.
    # Reject a nonstandard matrix rather than silently using different geometry.
    if not np.allclose(matrix[[0, 1, 2, 2, 2], [1, 0, 0, 1, 2]],
                       [0, 0, 0, 0, 1], rtol=0, atol=1e-9):
        raise ValueError('unsupported camera intrinsic matrix')
    if distortion_model == 'plumb_bob' and d.size != 5:
        raise ValueError('plumb_bob requires five distortion coefficients')
    if distortion_model == 'rational_polynomial' and d.size != 8:
        raise ValueError('rational_polynomial requires eight distortion coefficients')
    pixel = np.array([[[u, v]]], dtype=float)
    if distortion_model in ('plumb_bob', 'rational_polynomial'):
        point = cv2.undistortPoints(pixel, matrix, d)
    elif distortion_model == 'equidistant' and d.size == 4:
        point = cv2.fisheye.undistortPoints(pixel, matrix, d)
    else:
        raise ValueError('unsupported distortion model')
    x, y = point.reshape(2)
    return np.array([x, y, 1.0])


def intersect_plane(ray, camera_origin, rotation, normal, offset,
                    minimum_range=0.05, maximum_range=2.0):
    """Intersect optical ray with calibrated n.dot(point) + offset = 0.

    This is a known-plane estimate, NOT a depth measurement or grasp pose.
    All geometry is expressed in the chosen target frame.
    """
    ray, origin, normal = [np.asarray(v, dtype=float) for v in
                           (ray, camera_origin, normal)]
    rotation = np.asarray(rotation, dtype=float)
    if any(v.shape != (3,) for v in (ray, origin, normal)) or rotation.shape != (3, 3):
        raise ValueError('invalid geometry dimensions')
    if not all(np.all(np.isfinite(v)) for v in (ray, origin, normal, rotation)):
        raise ValueError('non-finite geometry')
    if not math.isfinite(offset) or np.linalg.norm(normal) < 1e-8 or np.linalg.norm(ray) < 1e-8:
        raise ValueError('invalid plane/ray')
    if not 0 < minimum_range < maximum_range:
        raise ValueError('invalid range limits')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) < 0:
        raise ValueError('invalid rotation')
    direction = rotation @ (ray / np.linalg.norm(ray))
    denominator = normal @ direction
    if abs(denominator) / np.linalg.norm(normal) < 1e-3:
        raise ValueError('ray is parallel to the plane')
    distance = -(normal @ origin + offset) / denominator
    if not minimum_range <= distance <= maximum_range:
        raise ValueError('plane intersection behind camera or outside calibrated range')
    return origin + distance * direction
