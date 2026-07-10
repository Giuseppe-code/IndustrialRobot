import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))
sys.path.append(str(ROOT / "path_planning"))

import cv2

from AckermannController import AckermannController
from lib.dds.dds import DDS
from lib.system.manipulator_control import FourJointsManipulatorControl
from manipulator_cv.camera import Camera
from manipulator_cv.pick_object import VisualPickController
from mission_types import MissionStateId
from path_planning.nf1 import NF1Planner
from path_planning.world import World


ARM_ORIENTATION = math.radians(-90)
ARM_APPROACH_TARGET = (-0.8, 0.0, 0.5)
PICK_HEIGHT = 0.08
PACKAGE_MASS_KG = 2.0
WAYPOINT_THRESHOLD = 1.5
FINAL_THRESHOLD = 0.18


def has_reached(x, y, target, threshold):
    target_x, target_y = target
    return math.hypot(x - target_x, y - target_y) < threshold


def arm_has_reached(
    arm,
    target,
    position_threshold=0.025,
    angle_threshold=math.radians(3),
):
    x, y, z, angle = arm.get_pose()
    target_x, target_y, target_z = target
    distance = math.sqrt(
        (x - target_x) ** 2
        + (y - target_y) ** 2
        + (z - target_z) ** 2
    )
    angle_error = math.atan2(
        math.sin(angle - ARM_ORIENTATION),
        math.cos(angle - ARM_ORIENTATION),
    )
    return distance < position_threshold and abs(angle_error) < angle_threshold


def publish_state(dds, ackermann, arm, mission, payload_mass):
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
    dds.publish("CargoColorCode", 2.0, DDS.DDS_TYPE_FLOAT)
    dds.publish("UnloadZoneCode", 3.0, DDS.DDS_TYPE_FLOAT)


def build_path_to_pick_zone():
    world = World(2000, 2000, 50)
    world.add_rectangle_obstacle(575, 1550, 625, 1850)
    world.add_rectangle_obstacle(0, 1362.5, 12.5, 1387.5)
    planner = NF1Planner(world)

    # Il punto resta allineato a B (27.5 m), ma il carrello si ferma davanti al pacco.
    path_map = planner.plan((50, 50), (175, 1375))
    path = []
    for x, y in path_map:
        world_x, world_y = world.to_world(x, y)
        path.append((world_x / world.scale, -world_y / world.scale))
    return path


def parse_args():
    parser = argparse.ArgumentParser(description="Missione di presa in zona B")
    parser.add_argument(
        "--max-time",
        type=float,
        default=120.0,
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

        path_to_b = build_path_to_pick_zone()
        print("Percorso verso B:", path_to_b)

        ackermann = AckermannController()
        ackermann.set_pose(1.0, -1.0, 0.0)
        arm = FourJointsManipulatorControl()
        visual_picker = VisualPickController(camera.width, camera.height)

        mission = MissionStateId.DRIVE_TO_B
        payload_mass = 0.0
        waypoint_index = 0
        target = path_to_b[waypoint_index]
        pick_xy = None
        missing_frames = 0
        camera_warmup_frames = 0
        camera_reconnects = 0
        step = 0
        elapsed_time = 0.0

        ackermann.start_motion(target)
        if not arm.set_target(*ARM_APPROACH_TARGET, ARM_ORIENTATION):
            raise RuntimeError("La posa di approccio del braccio non e' raggiungibile")

        publish_state(dds, ackermann, arm, mission, payload_mass)

        while elapsed_time < args.max_time:
            delta_t = dds.wait("tick")
            if delta_t is None or delta_t <= 0:
                continue

            elapsed_time += delta_t
            arm.evaluate(delta_t)

            if mission is MissionStateId.DRIVE_TO_B:
                ackermann.evaluate(delta_t)
                is_last = waypoint_index == len(path_to_b) - 1
                threshold = FINAL_THRESHOLD if is_last else WAYPOINT_THRESHOLD

                if step % 60 == 0:
                    distance = math.dist(ackermann.pose[:2], target)
                    print(
                        f"Guida: punto {waypoint_index + 1}/{len(path_to_b)}, "
                        f"distanza {distance:.2f} m"
                    )

                if has_reached(*ackermann.pose[:2], target, threshold):
                    if is_last:
                        mission = MissionStateId.PICK_APPROACH
                        print("PICK_APPROACH: veicolo fermo davanti al pacco")
                    else:
                        waypoint_index += 1
                        target = path_to_b[waypoint_index]
                        ackermann.start_motion(target)

            elif mission is MissionStateId.PICK_APPROACH:
                if arm_has_reached(arm, ARM_APPROACH_TARGET):
                    camera.connect()
                    camera_connected = True
                    camera_warmup_frames = 3
                    visual_picker.reset()
                    mission = MissionStateId.VISION_PICK
                    pose = tuple(round(value, 3) for value in arm.get_pose())
                    print("VISION_PICK: ricerca e centraggio, posa braccio", pose)

            elif mission is MissionStateId.VISION_PICK:
                if camera_warmup_frames > 0:
                    camera_warmup_frames -= 1
                else:
                    dds.publish("read_image", 1, DDS.DDS_TYPE_INT)
                    image = camera.read()
                    result = visual_picker.evaluate(image, arm.get_pose(), delta_t)

                    if result.found:
                        missing_frames = 0
                        if not arm.set_target(*result.target, ARM_ORIENTATION):
                            print(
                                "VISION_PICK: correzione scartata, "
                                "posa non raggiungibile"
                            )
                        elif step % 15 == 0:
                            print(
                                "VISION_PICK: centro",
                                result.centroid,
                                "errore",
                                result.error,
                            )

                        if result.locked:
                            current_x, current_y, _, _ = arm.get_pose()
                            pick_xy = (current_x, current_y)
                            down_target = (*pick_xy, PICK_HEIGHT)
                            if not arm.set_target(*down_target, ARM_ORIENTATION):
                                raise RuntimeError(
                                    "La quota di presa non e' raggiungibile"
                                )
                            mission = MissionStateId.PICK_DOWN
                            print("PICK_DOWN: pacco centrato, inizio discesa")
                    else:
                        missing_frames += 1
                        if step % 30 == 0:
                            print("VISION_PICK: pacco non visibile")
                        if missing_frames >= 30 and camera_reconnects == 0:
                            camera.reconnect()
                            camera_reconnects += 1
                            camera_warmup_frames = 3
                            missing_frames = 0
                            print("VISION_PICK: stream camera riconnesso")

                    if args.show_camera:
                        cv2.imshow("camera", image)
                        cv2.imshow("binary_image", result.binary_image)
                        cv2.waitKey(1)

            elif mission is MissionStateId.PICK_DOWN:
                down_target = (*pick_xy, PICK_HEIGHT)
                if arm_has_reached(arm, down_target):
                    payload_mass = PACKAGE_MASS_KG
                    ackermann.set_payload_mass(payload_mass)

                    lift_target = (*pick_xy, ARM_APPROACH_TARGET[2])
                    arm.set_target(*lift_target, ARM_ORIENTATION)
                    mission = MissionStateId.PICK_LIFT_SAFE
                    print(
                        f"PICK_LIFT_SAFE: pacco preso, carico {payload_mass:.1f} kg"
                    )

            elif mission is MissionStateId.PICK_LIFT_SAFE:
                lift_target = (*pick_xy, ARM_APPROACH_TARGET[2])
                if arm_has_reached(arm, lift_target):
                    mission = MissionStateId.DRIVE_TO_UNLOAD
                    completed = True
                    print("Presa completata: carrello pronto per la zona di scarico")

            publish_state(dds, ackermann, arm, mission, payload_mass)
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
