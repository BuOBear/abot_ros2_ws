"""Generic Haar-cascade face boxes for the camera image stream."""

import math
from numbers import Integral, Real
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

from .detection_utils import ImageBox, ImageDetector, spin_detector


def _validate_face_configuration(cascade, scale_factor, min_neighbors, min_size_px):
    if (cascade is None or not callable(getattr(cascade, 'detectMultiScale', None)) or
            isinstance(scale_factor, bool) or not isinstance(scale_factor, Real) or
            not math.isfinite(scale_factor) or scale_factor <= 1.0 or
            not isinstance(min_neighbors, Integral) or isinstance(min_neighbors, bool) or
            min_neighbors < 0 or not isinstance(min_size_px, Integral) or
            isinstance(min_size_px, bool) or min_size_px <= 0):
        raise ValueError('Invalid face detector configuration')


def detect_faces(image_bgr, cascade, scale_factor=1.1, min_neighbors=5,
                 min_size_px=30):
    """Return face boxes from a BGR frame using an OpenCV Haar cascade.

    Cascade detections do not include a calibrated confidence. The score 1.0
    means only that the cascade returned a face candidate.
    """
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8 or
            image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0):
        raise ValueError('Expected a nonempty BGR image')
    _validate_face_configuration(cascade, scale_factor, min_neighbors, min_size_px)

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    rectangles = cascade.detectMultiScale(
        gray, scaleFactor=float(scale_factor), minNeighbors=int(min_neighbors),
        minSize=(int(min_size_px), int(min_size_px)))
    height, width = gray.shape
    boxes = []
    for rectangle in rectangles:
        if len(rectangle) != 4:
            continue
        if not all(math.isfinite(float(value)) for value in rectangle):
            continue
        x, y, box_width, box_height = (int(value) for value in rectangle)
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(width, x + box_width), min(height, y + box_height)
        if x1 <= x0 or y1 <= y0:
            continue
        boxes.append(ImageBox(
            'face', (x0 + x1) / 2.0, (y0 + y1) / 2.0,
            float(x1 - x0), float(y1 - y0), 1.0))
    return boxes


class FaceDetector(ImageDetector):
    """ROS 2 source publishing 2-D face boxes from camera images."""

    def __init__(self):
        super().__init__('face_detector', '/perception/faces', 3.0)
        default_path = (Path(get_package_share_directory('abot_perception')) /
                        'cascades' / 'haarcascade_frontalface_default.xml')
        self.declare_parameter('cascade_path', str(default_path))
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('min_size_px', 30)

        cascade_path = str(self.get_parameter('cascade_path').value).strip()
        if not cascade_path:
            raise ValueError('cascade_path must not be empty')
        try:
            self.scale_factor = float(self.get_parameter('scale_factor').value)
        except (TypeError, ValueError) as exc:
            raise ValueError('scale_factor must be finite and greater than 1') from exc
        self.min_neighbors = self.get_parameter('min_neighbors').value
        self.min_size_px = self.get_parameter('min_size_px').value

        self.cascade = cv2.CascadeClassifier(cascade_path)
        if self.cascade.empty():
            raise ValueError(f'Cannot load face cascade: {cascade_path}')
        _validate_face_configuration(self.cascade, self.scale_factor,
                                     self.min_neighbors, self.min_size_px)

    def find_boxes(self, image_bgr):
        return detect_faces(image_bgr, self.cascade, self.scale_factor,
                            self.min_neighbors, self.min_size_px)


def main():
    spin_detector(FaceDetector)
