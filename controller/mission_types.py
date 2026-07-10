from enum import IntEnum


class MissionStateId(IntEnum):
    DRIVE_TO_B = 0
    PICK_APPROACH = 1
    VISION_PICK = 2
    PICK_DOWN = 3
    PICK_LIFT_SAFE = 4
    DRIVE_TO_UNLOAD = 5
    DROP_APPROACH = 6
    DROP_DOWN = 7
    DROP_LIFT_SAFE = 8
    DONE = 9
