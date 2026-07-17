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

    def find(self, image):
        binary_image = cv2.inRange(
            image, self.lower_limit, self.higher_limit
        )
        contours, _ = cv2.findContours(
            binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        cx = -1
        cy = -1
        
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea) #altrimenti può prendere anche una linea e provare a calcolare il centroide, a noi serve il centroide del pacco
            cv2.drawContours(image, [cnt], -1,  (255, 255, 255))
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                # get the centroid of the contour
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.circle(image, (cx, cy), 3, (0,0,0), -1)
        else:
            print("nessun contorno trovato")
            cx = -1
            cy = -1

        return cx, cy, binary_image
