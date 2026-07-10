"""Color-threshold object detector used by the manipulator camera."""

import cv2
import numpy as np


class ObjectFinder:
    def __init__(self, thresholds, min_area=100.0):
        low_threshold, high_threshold = thresholds
        self.lower_limit = np.array(low_threshold)
        self.higher_limit = np.array(high_threshold)
        self.min_area = min_area

    def find(self, image):
        binary_image = cv2.inRange(
            image, self.lower_limit, self.higher_limit
        )
        contours, _ = cv2.findContours(
            binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        cx = -1
        cy = -1
        if contours:
            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) >= self.min_area:
                cv2.drawContours(image, [contour], -1, (255, 255, 255))
                moments = cv2.moments(contour)
                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                    cv2.circle(image, (cx, cy), 3, (0, 0, 0), -1)

        return cx, cy, binary_image
