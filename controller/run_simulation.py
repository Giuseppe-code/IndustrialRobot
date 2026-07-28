import argparse
import math
import sys
from pathlib import Path
from matplotlib import pylab

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))
sys.path.append(str(ROOT / "path_planning"))

import cv2
from controller.Path_Planning import *
from controller.AckermannController import AckermannController
from controller.mission_types import (
    CargoColorCode,
    MissionStateId,
    UNLOAD_ZONE_BY_COLOR,
    UnloadZoneCode,
)
from lib.data.dataplot import DataPlotter, plot_multiple
from lib.dds.dds import DDS
from lib.system.manipulator_control import FourJointsManipulatorControl
from manipulator_cv.camera import Camera
from manipulator_cv.pick_object import VisualPickController
from path_planning.nf1 import NF1Planner
from path_planning.world import World


ARM_ORIENTATION = math.radians(-105)
ARM_MOVEMENT_TARGET = (0.20, 0.00, 0.55)
ARM_APPROACH_TARGET_0 = (-0.8, 0.0, 0.75) # Zone_B
ARM_APPROACH_TARGET_1 = (-0.32, -0.23, 0.75) # Zone_C
ARM_APPROACH_TARGET_2 = (+0.42, -0.10, 0.75) # Zone_A

PICK_HEIGHT = 0.15
DROP_HEIGHT = 0.15
PACKAGE_MASS_KG = 2.0

UNLOAD_FINAL_THRESHOLD = 1


# calcolo della distnza e orientamento tra posa braccio e posa desiderata. restituisce true se le treshold sono soddisfatte.
def arm_has_reached(
    arm : FourJointsManipulatorControl,
    target: tuple[float, float, float],
    position_threshold=0.5,
    angle_threshold=math.radians(3),
):
    x, y, z, angle = arm.get_pose()
    target_x, target_y, target_z = target
    xy_error = math.dist((x,y), (target_x, target_y))
    z_error = abs(z - target_z)
# normalizzazione errore di handling.
    angle_error = math.atan2(
        math.sin(angle - ARM_ORIENTATION),
        math.cos(angle - ARM_ORIENTATION),
    )
    return abs(xy_error) <= position_threshold and z_error < position_threshold and abs(angle_error) <= angle_threshold

# connessioni e update con dds
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

# istanziazione dei grafici.
def build_plotters():
    speed_plot = DataPlotter()
    speed_plot.set_x("Tempo (s)")
    speed_plot.add_y("Speed", "Velocita carrello (m/s)")
    speed_plot.add_y("Target_Speed", "Velocita Target carrello (m/s)")

    position_plot = DataPlotter()
    position_plot.set_x("Tempo (s)")
    position_plot.add_y("x", "Posizione X carrello (m)")
    position_plot.add_y("y", "Posizione Y carrello (m)")

    orientation_plot = DataPlotter()
    orientation_plot.set_x("Tempo (s)")
    orientation_plot.add_y("vehicle_angle", "Orientamento carrello (gradi)")

    arm_plot = DataPlotter()
    arm_plot.set_x("Tempo (s)")
    arm_plot.add_y("theta0", "Theta 0 (gradi)")
    arm_plot.add_y("theta1", "Theta 1 (gradi)")
    arm_plot.add_y("theta2", "Theta 2 (gradi)")
    arm_plot.add_y("theta3", "Theta 3 (gradi)")

    return speed_plot, position_plot, orientation_plot, arm_plot


def append_plot_data(plotters, elapsed_time, ackermann: AckermannController, arm, mission: MissionStateId):
    speed_plot, position_plot, orientation_plot, arm_plot = plotters
    vehicle_x, vehicle_y, vehicle_theta = ackermann.pose
    theta0, theta1, theta2, theta3 = arm.get_joint_angles()
    _, _, _, arm_alpha = arm.get_pose()
    is_driving = mission in (
        MissionStateId.DRIVE_TO_B,
        MissionStateId.DRIVE_TO_UNLOAD,
    )

    vision_or_picker = mission in (
        MissionStateId.VISION_PICK,
        MissionStateId.PICK_DOWN
    )

    if is_driving:
        speed_plot.append_x(elapsed_time)
        speed_plot.append_y("Speed", ackermann.v)
        speed_plot.append_y("Target_Speed", ackermann.vt)
        
        orientation_plot.append_x(elapsed_time)
        orientation_plot.append_y("vehicle_angle", math.degrees(ackermann.alpha_c))

        position_plot.append_x(elapsed_time)
        position_plot.append_y("x", vehicle_x)
        position_plot.append_y("y", vehicle_y)
    
    if vision_or_picker : 
        arm_plot.append_x(elapsed_time)
        arm_plot.append_y("theta0", math.degrees(theta0))
        arm_plot.append_y("theta1", math.degrees(theta1))
        arm_plot.append_y("theta2", math.degrees(theta2))
        arm_plot.append_y("theta3", math.degrees(theta3))

# utility utili per mostrare camera e limitare tempo di simulazione.
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
    ackermann = AckermannController()
    planning = PathPlanning(ackermann)

    try:
        dds.start()
        dds.subscribe(["tick"])
        # La connessione resta pronta mentre il braccio raggiunge la posa visuale.
        camera.connect()
        camera_connected = True
        path = planning.build_path_to_pick_zone()
        planning.build_angle_search(path)

        print(f"Percorso verso B: {len(path)} punti, arrivo {path[-1]}")

        ackermann.set_pose(1.0, -1, 0.0) #1 -1 #-3 20
        arm = FourJointsManipulatorControl()
        visual_picker = VisualPickController(camera.width, camera.height)
        plotters = build_plotters()

        mission = MissionStateId.DRIVE_TO_B
        cargo_color = CargoColorCode.UNKNOWN
        unload_zone = UnloadZoneCode.UNKNOWN #A o C
        payload_mass = 0.0
        target= planning.reset()
        pick_xy = None
        missing_frames = 0
        arm_settle_frames = 0
        vision_target = ARM_APPROACH_TARGET_0
        step = 0
        elapsed_time = 0.0

        ackermann.start_motion(target)
        arm.set_target(*ARM_MOVEMENT_TARGET, ARM_ORIENTATION)

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
                arrived = planning.follow_path(
                    delta_t,
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
                    position_threshold=1,
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
                        print("Unload_zone settato")
                        height= arm.get_pose()[2]
                        mission = MissionStateId.PICK_DOWN
                        visual_picker.locked_count=0
                        down_target=(result.target[0], result.target[1], height)
                        if not arm.set_target(*down_target, ARM_ORIENTATION):
                            raise RuntimeError("La quota di presa non e' raggiungibile")
                        print(
                            f"PICK_DOWN: pacco {cargo_color.name}, "
                            f"destinazione {unload_zone.name}"
                        )
                else:
                    missing_frames += 1
                    if step % 30 == 0:
                        print("VISION_PICK: pacco non visibile")

            elif mission is MissionStateId.PICK_DOWN:

                dds.publish("read_image", 1, DDS.DDS_TYPE_INT)
                image = camera.read()
                result = visual_picker.evaluate(image, arm.get_pose(), delta_t, color_finded)
                
                if args.show_camera:
                    cv2.imshow("camera", image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                if (abs(result.error[0]) < 8)and(abs(result.error[1]) < 8) and height > PICK_HEIGHT:
                    height -= 0.15
                    down_target=(result.target[0], result.target[1], height)
                    if not arm.set_target(*down_target, ARM_ORIENTATION):
                        raise RuntimeError("La quota di presa non e' raggiungibile")

                actual = arm.get_pose()
                print(f"target=({down_target[0]:.3f}, {down_target[1]:.3f}, {down_target[2]:.3f}) actual=({actual[0]:.3f}, {actual[1]:.3f}, {actual[2]:.3f})")

                print("valore height", height, "valore da raggiungere", PICK_HEIGHT)
                if arm_has_reached(arm, down_target, position_threshold=0.5) and height <= PICK_HEIGHT:
                    payload_mass = PACKAGE_MASS_KG
                    ackermann.set_payload_mass(payload_mass)
                    current_x, current_y, _, _ = arm.get_pose()
                    pick_xy = (current_x, current_y)
                    camera.close()
                    camera_connected = False
                    lift_target = (*pick_xy, ARM_APPROACH_TARGET_0[2])
                    arm.set_target(*lift_target, ARM_ORIENTATION)
                    mission = MissionStateId.PICK_LIFT_SAFE

                    print(
                        f"PICK_LIFT_SAFE: pacco preso, carico {payload_mass:.1f} kg"
                    )

            elif mission is MissionStateId.PICK_LIFT_SAFE:
                if arm_has_reached(arm, lift_target):
                    print("unload_zone", unload_zone)
                    path=planning.build_path_to_unload_zone(unload_zone)
                    planning.build_angle_search(path)
                    target = planning.reset()
                    ackermann.start_motion(target)
                    mission = MissionStateId.DRIVE_TO_UNLOAD
                    print(
                        f"DRIVE_TO_UNLOAD: {len(path)} punti verso "
                        f"la zona {unload_zone.name}"
                    )

            elif mission is MissionStateId.DRIVE_TO_UNLOAD:
                arrived = planning.follow_path(
                    delta_t,
                    f"Guida verso {unload_zone.name}",
                    UNLOAD_FINAL_THRESHOLD,
                )
                if arrived:
                    print("Target_1")
                    arm_approach=ARM_APPROACH_TARGET_1
                    if unload_zone == UnloadZoneCode.A:
                        print("Target_2_ Setted")
                        arm_approach= ARM_APPROACH_TARGET_2
                    arm.set_target(*arm_approach, ARM_ORIENTATION)
                    mission = MissionStateId.DROP_APPROACH
                    print(f"DROP_APPROACH: arrivo in zona {unload_zone.name}")

            elif mission is MissionStateId.DROP_APPROACH:
                if arm_has_reached(arm, arm_approach):
                    drop_target = (
                        arm_approach[0],
                        arm_approach[1],
                        DROP_HEIGHT,
                    )
                    arm.set_target(*drop_target, ARM_ORIENTATION)
                    mission = MissionStateId.DROP_DOWN
                    print("DROP_DOWN: inizio deposito del pacco")

            elif mission is MissionStateId.DROP_DOWN:
                if arm_has_reached(arm, drop_target, position_threshold=0.1):
                    payload_mass = 0.0
                    ackermann.set_payload_mass(payload_mass)
                    arm.set_target(*ARM_MOVEMENT_TARGET, ARM_ORIENTATION)
                    mission = MissionStateId.DROP_LIFT_SAFE
                    print("DROP_LIFT_SAFE: pacco rilasciato, massa rimossa")

            elif mission is MissionStateId.DROP_LIFT_SAFE:
                if arm_has_reached(arm, ARM_MOVEMENT_TARGET):
                    mission = MissionStateId.DONE
                    completed = True
                    print(
                        f"DONE: pacco {cargo_color.name} depositato "
                        f"in zona {unload_zone.name}"
                    )
                mission = MissionStateId.DONE
                completed = False
            

            publish_state(
                dds,
                ackermann,
                arm,
                mission,
                payload_mass,
                cargo_color,
                unload_zone,
            )
            append_plot_data(plotters, elapsed_time, ackermann, arm, mission)
            step += 1

            if mission is MissionStateId.DONE:
                break


        for plotter in plotters:
            if plotter.x_data:  # solo se ha dati
                plotter.plot()
            else:
                print(f"[plot] nessun dato per {list(plotter.y_label.keys())}")
        pylab.show()

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
