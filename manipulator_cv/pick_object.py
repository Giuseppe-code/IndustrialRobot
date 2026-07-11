"""Controllo visuale per centrare il manipolatore sopra al pacco."""

from dataclasses import dataclass

import cv2

from controller.mission_types import CargoColorCode
from lib.system.controllers import PID_Controller
from manipulator_cv.object_finder import ObjectFinder


PACKAGE_THRESHOLDS = {
    CargoColorCode.BLUE: ([140, 0, 0], [255, 150, 140]),
    CargoColorCode.RED: ([0, 0, 160], [100, 120, 255]),
}


@dataclass
class VisualPickResult:
    found: bool
    locked: bool
    color: CargoColorCode
    centroid: tuple[int, int]
    error: tuple[int, int]
    target: tuple[float, float, float]
    binary_image: object


class VisualPickController:
    def __init__(
        self,
        width=512,
        height=512,
        target_height=0.75,
        tolerance_px=6,
        lock_frames=8,
    ):
        self.width = width
        self.height = height
        self.target_height = target_height
        self.tolerance_px = tolerance_px
        self.lock_frames = lock_frames
        self.locked_count = 0
        self.locked_color = CargoColorCode.UNKNOWN
        self.object_finders = {
            color: ObjectFinder(thresholds)
            for color, thresholds in PACKAGE_THRESHOLDS.items()
        }

        # I guadagni derivano dalla camera montata sul polso nella scena TestWorld.
        self.x_controller = PID_Controller(1.0 / 850.0, 0.0, 0.0, 0.03)
        self.y_controller = PID_Controller(1.0 / 760.0, 0.0, 0.0, 0.03)

    def reset(self):
        self.locked_count = 0
        self.locked_color = CargoColorCode.UNKNOWN

    def evaluate(self, image, current_pose, delta_t):
        detections = []
        binary_image = None
        for color, finder in self.object_finders.items():
            cx, cy, color_mask = finder.find(image)
            binary_image = (
                color_mask
                if binary_image is None
                else cv2.bitwise_or(binary_image, color_mask)
            )
            if cx >= 0 and cy >= 0:
                detections.append((finder.last_area, color, cx, cy))

        x, y, _, _ = current_pose

        if not detections:
            self.locked_count = 0
            self.locked_color = CargoColorCode.UNKNOWN
            return VisualPickResult(
                False,
                False,
                CargoColorCode.UNKNOWN,
                (-1, -1),
                (0, 0),
                (x, y, self.target_height),
                binary_image,
            )

        _, color, cx, cy = max(detections, key=lambda detection: detection[0])

        error_x = self.width // 2 - cx
        error_y = self.height // 2 - cy
        centered = (
            abs(error_x) <= self.tolerance_px
            and abs(error_y) <= self.tolerance_px
        )
        if centered and color == self.locked_color:
            self.locked_count += 1
        elif centered:
            self.locked_color = color
            self.locked_count = 1
        else:
            self.locked_color = color
            self.locked_count = 0

        correction_x = self.x_controller.evaluate(delta_t, error_x)
        correction_y = self.y_controller.evaluate(delta_t, error_y)

        # La camera e' ruotata rispetto agli assi del manipolatore.
        target_x = x - correction_x
        target_y = y + correction_y
        return VisualPickResult(
            True,
            self.locked_count >= self.lock_frames,
            color,
            (cx, cy),
            (error_x, error_y),
            (target_x, target_y, self.target_height),
            binary_image,
        )
