"""Configurable OpenCV AprilTag/ArUco 2-D marker detection, without pose claims."""

import math

import cv2
import numpy as np

from .detection_utils import ImageBox, ImageDetector, spin_detector


def tag_dictionary(name):
    if (not isinstance(name, str) or not name.startswith('DICT_') or
            not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, name)):
        raise ValueError(f'OpenCV marker dictionary unavailable: {name}')
    value = getattr(cv2.aruco, name)
    if not isinstance(value, int):
        raise ValueError(f'Invalid OpenCV marker dictionary: {name}')
    return cv2.aruco.Dictionary_get(value)


def tag_class(dictionary_name, marker_id):
    return f'tag:{dictionary_name.removeprefix("DICT_").lower()}:{int(marker_id)}'


def detect_tags(image_bgr, dictionary_name, dictionary, min_side_px=16.0):
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8 or
            image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0):
        raise ValueError('Expected a nonempty BGR image')
    if (isinstance(min_side_px, bool) or not isinstance(min_side_px, (int, float)) or
            not math.isfinite(min_side_px) or min_side_px <= 0):
        raise ValueError('min_side_px must be positive')
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None:
        return []
    height, width = gray.shape
    boxes = []
    for quadrilateral, marker_id in zip(corners, ids.flatten()):
        points = quadrilateral.reshape(-1, 2)
        x0, y0 = points.min(axis=0)
        x1, y1 = points.max(axis=0)
        side_x, side_y = float(x1 - x0), float(y1 - y0)
        if (not all(math.isfinite(float(v)) for v in (x0, y0, x1, y1)) or
                min(side_x, side_y) < min_side_px or x0 < 0 or y0 < 0 or
                x1 >= width or y1 >= height):
            continue
        # A decoded dictionary/ID is categorical. 1.0 is not a calibrated confidence.
        boxes.append(ImageBox(tag_class(dictionary_name, marker_id),
                              float((x0 + x1) / 2), float((y0 + y1) / 2),
                              side_x, side_y, 1.0))
    return boxes


class TagDetector(ImageDetector):
    def __init__(self):
        super().__init__('tag_detector', '/perception/tags', 10.0)
        self.declare_parameter('dictionary', 'DICT_APRILTAG_36h11')
        self.declare_parameter('min_side_px', 16.0)
        self.dictionary_name = str(self.get_parameter('dictionary').value)
        self.dictionary = tag_dictionary(self.dictionary_name)
        self.min_side_px = float(self.get_parameter('min_side_px').value)
        if not math.isfinite(self.min_side_px) or self.min_side_px <= 0:
            raise ValueError('min_side_px must be finite and positive')

    def find_boxes(self, image_bgr):
        return detect_tags(image_bgr, self.dictionary_name, self.dictionary,
                           self.min_side_px)


def main():
    spin_detector(TagDetector)
