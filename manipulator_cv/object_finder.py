"""Color-threshold object detector used by the manipulator camera."""

import cv2
import numpy as np


class ObjectFinder:
    def __init__(
        self,
        thresholds,
        min_area=100.0,
        max_area_ratio=0.25,
        border_margin=2,
    ):
        low_threshold, high_threshold = thresholds
        self.lower_limit = np.array(low_threshold)
        self.higher_limit = np.array(high_threshold)
        self.min_area = min_area
        self.max_area_ratio = max_area_ratio
        self.border_margin = border_margin
        self.last_area = 0.0

    def _is_valid_contour(self, contour, image_shape):
        height, width = image_shape[:2]
        area = cv2.contourArea(contour)
        if area < self.min_area or area > width * height * self.max_area_ratio:
            return False

        x, y, contour_width, contour_height = cv2.boundingRect(contour)
        # Il pacco e' interno all'immagine: un contorno sul bordo e' lo sfondo.
        return (
            x > self.border_margin
            and y > self.border_margin
            and x + contour_width < width - self.border_margin
            and y + contour_height < height - self.border_margin
        )

    def find(self, image):
        binary_image = cv2.inRange(
            image, self.lower_limit, self.higher_limit
        )
        contours, _ = cv2.findContours(
            binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        cx = -1
        cy = -1
        self.last_area = 0.0
        valid_contours = [
            contour
            for contour in contours
            if self._is_valid_contour(contour, image.shape)
        ]
        if valid_contours:
            contour = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            self.last_area = area
            cv2.drawContours(image, [contour], -1, (255, 255, 255))
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                cv2.circle(image, (cx, cy), 3, (0, 0, 0), -1)

        return cx, cy, binary_image
