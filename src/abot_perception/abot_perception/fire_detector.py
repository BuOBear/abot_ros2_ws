"""HSV warm-color regions exposed as provisional fire candidates."""

import math

import cv2
import numpy as np

from .color_detector import hsv_bounds
from .detection_utils import ImageBox, ImageDetector, spin_detector


def detect_fire_candidate(image_bgr, color_ranges, min_contours,
                          min_total_bbox_area_px):
    """Apply the legacy red/yellow contour gate and return one union box.

    A result requires strictly more than ``min_contours`` contours and
    strictly more than ``min_total_bbox_area_px`` summed bounding-box area.
    Warm color alone is not evidence of fire; this remains an unvalidated
    candidate heuristic.
    """
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8 or
            image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0):
        raise ValueError('Expected a nonempty BGR image')
    if (isinstance(min_contours, (bool, np.bool_)) or
            not isinstance(min_contours, (int, np.integer)) or min_contours < 0):
        raise ValueError('min_contours must be a nonnegative integer')
    if (isinstance(min_total_bbox_area_px, (bool, np.bool_)) or
            not isinstance(min_total_bbox_area_px, (int, float, np.number)) or
            not math.isfinite(min_total_bbox_area_px) or min_total_bbox_area_px <= 0):
        raise ValueError('min_total_bbox_area_px must be positive')

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for intervals in color_ranges.values():
        for lower, upper in intervals:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

    mask = cv2.blur(mask, (5, 5))
    _, mask = cv2.threshold(mask, 10, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    image_area = float(image_bgr.shape[0] * image_bgr.shape[1])
    if len(contours) <= min_contours:
        return []

    rectangles = []
    total_bbox_area_px = 0
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        rectangles.append((x, y, width, height))
        total_bbox_area_px += width * height
    if total_bbox_area_px <= min_total_bbox_area_px:
        return []

    left = min(x for x, _, _, _ in rectangles)
    top = min(y for _, y, _, _ in rectangles)
    right = max(x + width for x, _, width, _ in rectangles)
    bottom = max(y + height for _, y, _, height in rectangles)
    union_width = right - left
    union_height = bottom - top
    return [ImageBox(
        'fire_candidate', left + union_width / 2.0,
        top + union_height / 2.0, float(union_width), float(union_height),
        min(1.0, total_bbox_area_px / image_area))]


class FireDetector(ImageDetector):
    """Publish provisional fire-color candidates from one stamped image stream."""

    def __init__(self):
        super().__init__('fire_detector', '/perception/fire', 10.0)
        self.declare_parameter('min_contours', 10)
        self.declare_parameter('min_total_bbox_area_px', 40.0)
        for name, defaults in {
            'red_lower': [0, 128, 46],
            'red_upper': [5, 255, 255],
            'red_wrap_lower': [156, 128, 46],
            'red_wrap_upper': [179, 255, 255],
            'yellow_lower': [15, 128, 46],
            'yellow_upper': [50, 255, 255],
        }.items():
            self.declare_parameter(name, defaults)

        self.min_contours = self.get_parameter('min_contours').value
        if (isinstance(self.min_contours, bool) or
                not isinstance(self.min_contours, int) or self.min_contours < 0):
            raise ValueError('min_contours must be a nonnegative integer')
        try:
            self.min_total_bbox_area_px = float(
                self.get_parameter('min_total_bbox_area_px').value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                'min_total_bbox_area_px must be finite and positive') from exc
        if (not math.isfinite(self.min_total_bbox_area_px) or
                self.min_total_bbox_area_px <= 0):
            raise ValueError('min_total_bbox_area_px must be finite and positive')

        bounds = lambda name: hsv_bounds(
            self.get_parameter(name + '_lower').value,
            self.get_parameter(name + '_upper').value)
        self.color_ranges = {
            'red': (bounds('red'), bounds('red_wrap')),
            'yellow': (bounds('yellow'),),
        }

    def find_boxes(self, image_bgr):
        return detect_fire_candidate(
            image_bgr, self.color_ranges, self.min_contours,
            self.min_total_bbox_area_px)


def main():
    spin_detector(FireDetector)
