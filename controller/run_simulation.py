import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))
sys.path.append(str(ROOT / "path_planning"))

import cv2

from controller.AckermannController import AckermannController
from controller.mission_types import (
    CargoColorCode,
    MissionStateId,
    UNLOAD_ZONE_BY_COLOR,
    UnloadZoneCode,
)
from lib.dds.dds import DDS
from lib.system.manipulator_control import FourJointsManipulatorControl
from manipulator_cv.camera import Camera
from manipulator_cv.pick_object import VisualPickController
from path_planning.nf1 import NF1Planner
from path_planning.world import World


ARM_ORIENTATION = math.radians(-90)
ARM_APPROACH_TARGET = (-0.8, 0.0, 0.75)
# visual scan  multiposa temporaneamente disabilitata.
# VISION_SEARCH_TARGETS = (
#     ARM_APPROACH_TARGET,
#     (-0.6, 0.0, 0.75),
#     (-0.7, -0.2, 0.75),
#     (-0.7, 0.2, 0.75),
# )
PICK_HEIGHT = 0.08
DROP_HEIGHT = 0.08
PACKAGE_MASS_KG = 2.0
WAYPOINT_THRESHOLD = 1.3
FINAL_THRESHOLD = 0.18
UNLOAD_FINAL_THRESHOLD = 1

PICK_START_MAP = (50, 50)
PICK_TARGET_MAP = (175, 1375)
UNLOAD_TARGET_MAP = {
    UnloadZoneCode.A: (50, 50),
    UnloadZoneCode.C: (925, 2025),#925, 2025 #900, 2050
}


def has_reached(x, y, target, threshold):
    target_x, target_y = target
    return math.dist((x,y), (target_x,target_y)) < threshold


def arm_has_reached(
    arm : FourJointsManipulatorControl,
    target: tuple[float, float, float],
    position_threshold=0.025,
    angle_threshold=math.radians(3),
):
    x, y, z, angle = arm.get_pose()
    target_x, target_y, target_z = target
    distance = math.dist((x,y,z), (target_x, target_y, target_z))

    angle_error = math.atan2(
        math.sin(angle - ARM_ORIENTATION),
        math.cos(angle - ARM_ORIENTATION),
    )
    return distance < position_threshold and abs(angle_error) < angle_threshold


def publish_state(
    dds: DDS,
    ackermann : AckermannController,
    arm : FourJointsManipulatorControl,
    mission: MissionStateId,
    payload_mass: float,
    cargo_color: CargoColorCode,
    unload_zone: UnloadZoneCode,
):
    vehicle_values = zip(("X", "Y", "Theta"), ackermann.pose)
    arm_values = zip(
        ("theta0", "theta1", "theta2", "theta3"),
        arm.get_joint_angles(),
    )
    pose_values = zip(("x", "y", "z", "a"), arm.get_pose())

    for name, value in (*vehicle_values, *arm_values, *pose_values):
        dds.publish(name, value, DDS.DDS_TYPE_FLOAT)

    dds.publish("PayloadMass", payload_mass, DDS.DDS_TYPE_FLOAT)
    dds.publish("MissionState", float(mission.value), DDS.DDS_TYPE_FLOAT)
    dds.publish("CargoColorCode", float(cargo_color.value), DDS.DDS_TYPE_FLOAT)
    dds.publish("UnloadZoneCode", float(unload_zone.value), DDS.DDS_TYPE_FLOAT)


def build_world():
    world = World(2500, 2500, 50)
    world.add_rectangle_obstacle(525, 1500, 675, 1900) #rettangolo 575, 1550, 625, 1850 (12,34) larghezza (1,5.488), +  a l for margin
    world.add_rectangle_obstacle(37.5, 1362.5, 62.5, 1387.5) #pacco pos (1, 27,5), larghezza 0,5
    return world

def build_path(start_map, target_map):
    world = build_world()
    path_map = NF1Planner(world)
    path_map=path_map.plan(start_map, target_map)
    #img = path_map.world_to_image()
    #cv2.imshow("world", img)
    #cv2.waitKey(0)
    path = []
    for x, y in path_map:
        world_x, world_y = world.to_world(x, y)
        path.append((world_x / world.scale, -world_y / world.scale)) ##offset to angle
    return path


def build_path_to_pick_zone():
    # Il punto resta allineato a B, ma il carrello si ferma accanto al pacco.
    return build_path(PICK_START_MAP, PICK_TARGET_MAP)


def build_path_to_unload_zone(unload_zone):
    return build_path(PICK_TARGET_MAP, UNLOAD_TARGET_MAP[unload_zone])


def follow_path(
    ackermann : AckermannController,
    delta_t : float,
    path : list[tuple[int, int]],
    index : int,
    target: tuple[int,int],
    step: int,
    label: str,
    final_threshold=FINAL_THRESHOLD,##final_threshold o unload_final_threshold
):
    ackermann.evaluate(delta_t)
    is_last = index == len(path) - 1
    threshold = final_threshold if is_last else WAYPOINT_THRESHOLD
    
    if step % 60 == 0:
        distance = math.dist(ackermann.pose[:2], target)
        print(f"{label}: punto {index + 1}/{len(path)}, distanza {distance:.2f} m")

    if not has_reached(*ackermann.pose[:2], target, threshold):
        return index, target, False

    if is_last:
        return index, target, True

    #ha raggiunto la cella, ora deve puntare alla prossima 
    index += 1
    target = path[index]
    ackermann.start_motion(target)
    return index, target, False


def parse_args():
    parser = argparse.ArgumentParser(description="Missione di presa e scarico")
    parser.add_argument(
        "--max-time",
        type=float,
        default=240.0,
        help="durata massima della simulazione in secondi",
    )
    parser.add_argument(
        "--show-camera",
        action="store_true",
        help="mostra immagine e maschera del riconoscimento",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    dds = DDS()
    camera = Camera()
    camera_connected = False
    completed = False

    try:
        dds.start()
        dds.subscribe(["tick"])
        # La connessione resta pronta mentre il braccio raggiunge la posa visuale.
        camera.connect()
        camera_connected = True

        path = build_path_to_pick_zone()
        print(f"Percorso verso B: {len(path)} punti, arrivo {path[-1]}")

        ackermann = AckermannController()
        ackermann.set_pose(1.0, -1.0, 0.0)
        arm = FourJointsManipulatorControl()
        visual_picker = VisualPickController(camera.width, camera.height)

        mission = MissionStateId.DRIVE_TO_B
        cargo_color = CargoColorCode.UNKNOWN
        unload_zone = UnloadZoneCode.UNKNOWN #A o C
        payload_mass = 0.0
        waypoint_index = 0
        target = path[waypoint_index]
        pick_xy = None
        missing_frames = 0
        arm_settle_frames = 0
        vision_target = ARM_APPROACH_TARGET
        step = 0
        elapsed_time = 0.0

        ackermann.start_motion(target)

        publish_state(
            dds,
            ackermann,
            arm,
            mission,
            payload_mass,
            cargo_color,
            unload_zone,
        )
        while elapsed_time < args.max_time:
            delta_t = dds.wait("tick")
            if delta_t is None or delta_t <= 0:
                continue

            elapsed_time += delta_t
            arm.evaluate(delta_t)

            if mission is MissionStateId.DRIVE_TO_B:
                waypoint_index, target, arrived = follow_path(
                    ackermann,
                    delta_t,
                    path,
                    waypoint_index,
                    target,
                    step,
                    "Guida verso B",
                )
                if arrived:
                    if not arm.set_target(*vision_target, ARM_ORIENTATION):
                        raise RuntimeError(
                            "La posa di approccio del braccio non e' raggiungibile"
                        )
                    mission = MissionStateId.PICK_APPROACH
                    print("PICK_APPROACH: veicolo fermo davanti al pacco")

            elif mission is MissionStateId.PICK_APPROACH:
                if arm_has_reached(
                    arm,
                    vision_target,
                    position_threshold=0.003,
                    angle_threshold=math.radians(0.25),
                ):
                    visual_picker.reset()
                    pose = tuple(round(value, 3) for value in arm.get_pose())
                    print("VISION_PICK: ricerca e centraggio, posa braccio", pose)
                    dds.publish("read_image", 1, DDS.DDS_TYPE_INT)
                    image = camera.read()
                    color_finded = visual_picker.check_color(image)
                    if color_finded is not None:
                        mission= MissionStateId.VISION_PICK

            elif mission is MissionStateId.VISION_PICK:
                dds.publish("read_image", 1, DDS.DDS_TYPE_INT)
                image = camera.read()
                result = visual_picker.evaluate(image, arm.get_pose(), delta_t, color_finded)
                if result.found:
                    missing_frames = 0
                    if not arm.set_target(*result.target, ARM_ORIENTATION):
                        if step % 30 == 0:
                            print(
                                "VISION_PICK: correzione scartata, "
                                "posa non raggiungibile"
                            )
                    elif step % 15 == 0:
                        print(
                            f"VISION_PICK: {result.color.name}, "
                            f"centro {result.centroid}, errore {result.error}"
                        )

                    if result.locked:
                        cargo_color = result.color
                        unload_zone = UNLOAD_ZONE_BY_COLOR[cargo_color]
                        current_x, current_y, _, _ = arm.get_pose()
                        pick_xy = (current_x, current_y)
                        down_target = (*pick_xy, PICK_HEIGHT)
                        if not arm.set_target(*down_target, ARM_ORIENTATION):
                            raise RuntimeError(
                                "La quota di presa non e' raggiungibile"
                            )
                        mission = MissionStateId.PICK_DOWN
                        print(
                            f"PICK_DOWN: pacco {cargo_color.name}, "
                            f"destinazione {unload_zone.name}"
                        )
                else:
                    missing_frames += 1
                    if step % 30 == 0:
                        print("")
                        print("VISION_PICK: pacco non visibile")


                if args.show_camera:
                    cv2.imshow("camera", image)
                    cv2.imshow("binary_image", result.binary_image)
                    cv2.waitKey(1)

            elif mission is MissionStateId.PICK_DOWN:
                #dds.publish("read_image", 1, DDS.DDS_TYPE_INT)
                #image = camera.read()
                #cv2.imshow('image', image)
                k = cv2.waitKey(1)
                if k == ord('q'):
                    exit()
                down_target = (*pick_xy, PICK_HEIGHT)
                if arm_has_reached(arm, down_target):
                    payload_mass = PACKAGE_MASS_KG
                    ackermann.set_payload_mass(payload_mass)
                    camera.close()
                    camera_connected = False
                    lift_target = (*pick_xy, ARM_APPROACH_TARGET[2])
                    arm.set_target(*lift_target, ARM_ORIENTATION)
                    mission = MissionStateId.PICK_LIFT_SAFE
                    print(
                        f"PICK_LIFT_SAFE: pacco preso, carico {payload_mass:.1f} kg"
                    )

            elif mission is MissionStateId.PICK_LIFT_SAFE:
                lift_target = (*pick_xy, ARM_APPROACH_TARGET[2])
                if arm_has_reached(arm, lift_target):
                    print("unload_zone", unload_zone)
                    path = build_path_to_unload_zone(unload_zone)
                    waypoint_index = 0
                    target = path[waypoint_index]
                    ackermann.start_motion(target)
                    mission = MissionStateId.DRIVE_TO_UNLOAD
                    print(
                        f"DRIVE_TO_UNLOAD: {len(path)} punti verso "
                        f"la zona {unload_zone.name}"
                    )

            elif mission is MissionStateId.DRIVE_TO_UNLOAD:
                waypoint_index, target, arrived = follow_path(
                    ackermann,
                    delta_t,
                    path,
                    waypoint_index,
                    target,
                    step,
                    f"Guida verso {unload_zone.name}",
                    UNLOAD_FINAL_THRESHOLD,
                )
                if arrived:
                    arm.set_target(*ARM_APPROACH_TARGET, ARM_ORIENTATION)
                    mission = MissionStateId.DROP_APPROACH
                    print(f"DROP_APPROACH: arrivo in zona {unload_zone.name}")

            elif mission is MissionStateId.DROP_APPROACH:
                if arm_has_reached(arm, ARM_APPROACH_TARGET):
                    drop_target = (
                        ARM_APPROACH_TARGET[0],
                        ARM_APPROACH_TARGET[1],
                        DROP_HEIGHT,
                    )
                    arm.set_target(*drop_target, ARM_ORIENTATION)
                    mission = MissionStateId.DROP_DOWN
                    print("DROP_DOWN: inizio deposito del pacco")

            elif mission is MissionStateId.DROP_DOWN:
                drop_target = (
                    ARM_APPROACH_TARGET[0],
                    ARM_APPROACH_TARGET[1],
                    DROP_HEIGHT,
                )
                if arm_has_reached(arm, drop_target):
                    payload_mass = 0.0
                    ackermann.set_payload_mass(payload_mass)
                    arm.set_target(*ARM_APPROACH_TARGET, ARM_ORIENTATION)
                    mission = MissionStateId.DROP_LIFT_SAFE
                    print("DROP_LIFT_SAFE: pacco rilasciato, massa rimossa")

            elif mission is MissionStateId.DROP_LIFT_SAFE:
                if arm_has_reached(arm, ARM_APPROACH_TARGET):
                    mission = MissionStateId.DONE
                    completed = True
                    print(
                        f"DONE: pacco {cargo_color.name} depositato "
                        f"in zona {unload_zone.name}"
                    )

            publish_state(
                dds,
                ackermann,
                arm,
                mission,
                payload_mass,
                cargo_color,
                unload_zone,
            )
            step += 1

            if completed:
                break

        if not completed:
            print(
                f"Missione interrotta dopo {elapsed_time:.1f} s nello stato "
                f"{mission.name}"
            )
            return 1
        return 0
    finally:
        if camera_connected:
            camera.close()
        dds.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
