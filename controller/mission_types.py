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


class CargoColorCode(IntEnum):
    UNKNOWN = 0
    BLUE = 1
    RED = 2


class UnloadZoneCode(IntEnum):
    UNKNOWN = 0
    A = 1
    C = 3


UNLOAD_ZONE_BY_COLOR = {
    CargoColorCode.BLUE: UnloadZoneCode.A,
    CargoColorCode.RED: UnloadZoneCode.C,
}
