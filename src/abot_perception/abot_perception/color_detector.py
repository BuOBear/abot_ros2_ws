"""ROS 2 HSV color detector migrated from the legacy abot_object_detect idea.

One largest blob per configured color is reported in image pixel coordinates.
The image header is preserved so consumers can reject old observations.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .detection_utils import ImageBox, ImageDetector, spin_detector


@dataclass(frozen=True)
class Blob:
    class_id: str
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    area_px: float


def hsv_bounds(lower, upper):
    """Validate one OpenCV HSV interval without silently clipping bad input."""
    try:
        if len(lower) != 3 or len(upper) != 3:
            raise ValueError('HSV bounds need exactly three channels')
    except TypeError as exc:
        raise ValueError('HSV bounds need exactly three channels') from exc
    limits = (179, 255, 255)
    if any(not isinstance(value, (int, np.integer)) or isinstance(value, (bool, np.bool_))
           for value in (*lower, *upper)):
        raise ValueError('HSV bounds must be integer channels')
    if any(not 0 <= low <= high <= limit
           for low, high, limit in zip(lower, upper, limits)):
        raise ValueError('Invalid HSV channel range')
    return np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)


def detect_blobs(image_bgr, color_ranges, min_area_px):
    """Return one largest valid contour for each named color."""
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8 or
            image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0):
        raise ValueError('Expected a nonempty BGR image')
    if (isinstance(min_area_px, (bool, np.bool_)) or
            not isinstance(min_area_px, (int, float, np.number)) or
            not math.isfinite(min_area_px) or min_area_px <= 0):
        raise ValueError('min_area_px must be positive')
    hsv = cv2.cvtColor(cv2.GaussianBlur(image_bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), dtype=np.uint8)
    blobs = []
    for class_id, intervals in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in intervals:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < min_area_px:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        blobs.append(Blob(class_id, x + width / 2.0, y + height / 2.0,
                          float(width), float(height), area))
    return blobs


class ColorDetector(ImageDetector):
    def __init__(self):
        super().__init__('color_detector', '/perception/colors', 10.0)
        self.declare_parameter('min_area_px', 500.0)
        for name, defaults in {
            'red_lower': [0, 112, 41], 'red_upper': [15, 255, 255],
            'red_wrap_lower': [170, 112, 41], 'red_wrap_upper': [179, 255, 255],
            'yellow_lower': [20, 100, 100], 'yellow_upper': [34, 255, 255],
            'green_lower': [35, 78, 71], 'green_upper': [85, 255, 255],
        }.items():
            self.declare_parameter(name, defaults)
        try:
            self.min_area_px = float(self.get_parameter('min_area_px').value)
        except (TypeError, ValueError) as exc:
            raise ValueError('min_area_px must be finite and positive') from exc
        if not math.isfinite(self.min_area_px) or self.min_area_px <= 0:
            raise ValueError('min_area_px must be finite and positive')
        bounds = lambda name: hsv_bounds(
            self.get_parameter(name + '_lower').value,
            self.get_parameter(name + '_upper').value)
        self.color_ranges = {
            'red': (bounds('red'), bounds('red_wrap')),
            'yellow': (bounds('yellow'),),
            'green': (bounds('green'),),
        }
    def find_boxes(self, image_bgr):
        blobs = detect_blobs(image_bgr, self.color_ranges, self.min_area_px)
        image_area = float(image_bgr.shape[0] * image_bgr.shape[1])
        # Occupied image fraction is a segmentation proxy, not calibrated confidence.
        return [ImageBox(blob.class_id, blob.center_x, blob.center_y,
                         blob.size_x, blob.size_y,
                         min(1.0, blob.area_px / image_area)) for blob in blobs]


def main():
    spin_detector(ColorDetector)
