"""Controllo visuale per centrare il manipolatore sopra al pacco."""

from dataclasses import dataclass

from lib.system.controllers import PID_Controller
from manipulator_cv.object_finder import ObjectFinder


RED_PACKAGE_THRESHOLDS = ([0, 0, 160], [100, 120, 255])


@dataclass
class VisualPickResult:
    found: bool
    locked: bool
    centroid: tuple[int, int]
    error: tuple[int, int]
    target: tuple[float, float, float]
    binary_image: object


class VisualPickController:
    def __init__(
        self,
        width=512,
        height=512,
        target_height=0.5,
        tolerance_px=6,
        lock_frames=8,
    ):
        self.width = width
        self.height = height
        self.target_height = target_height
        self.tolerance_px = tolerance_px
        self.lock_frames = lock_frames
        self.locked_count = 0
        self.object_finder = ObjectFinder(RED_PACKAGE_THRESHOLDS)

        # I guadagni derivano dalla camera montata sul polso nella scena TestWorld.
        self.x_controller = PID_Controller(1.0 / 850.0, 0.0, 0.0, 0.03)
        self.y_controller = PID_Controller(1.0 / 760.0, 0.0, 0.0, 0.03)

    def reset(self):
        self.locked_count = 0

    def evaluate(self, image, current_pose, delta_t):
        cx, cy, binary_image = self.object_finder.find(image)
        x, y, _, _ = current_pose

        if cx < 0 or cy < 0:
            self.locked_count = 0
            return VisualPickResult(
                False,
                False,
                (cx, cy),
                (0, 0),
                (x, y, self.target_height),
                binary_image,
            )

        error_x = self.width // 2 - cx
        error_y = self.height // 2 - cy
        centered = (
            abs(error_x) <= self.tolerance_px
            and abs(error_y) <= self.tolerance_px
        )
        self.locked_count = self.locked_count + 1 if centered else 0

        correction_x = self.x_controller.evaluate(delta_t, error_x)
        correction_y = self.y_controller.evaluate(delta_t, error_y)

        # La camera e' ruotata rispetto agli assi del manipolatore.
        target_x = x - correction_x
        target_y = y + correction_y
        return VisualPickResult(
            True,
            self.locked_count >= self.lock_frames,
            (cx, cy),
            (error_x, error_y),
            (target_x, target_y, self.target_height),
            binary_image,
        )
