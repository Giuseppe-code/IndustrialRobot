import argparse
import math
import sys
from pathlib import Path
from matplotlib import pylab

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
from lib.data.dataplot import DataPlotter, plot_multiple
from lib.dds.dds import DDS
from lib.system.manipulator_control import FourJointsManipulatorControl
from manipulator_cv.camera import Camera
from manipulator_cv.pick_object import VisualPickController
from path_planning.nf1 import NF1Planner
from path_planning.world import World


ARM_ORIENTATION = math.radians(-105)
ARM_APPROACH_TARGET_1 = (-0.32, -0.25, 0.75)
ARM_APPROACH_TARGET_2 = (+0.8, 0.0, 0.75)
ARM_MOVEMENT_TARGET = (0.20, 0.00, 0.55)
# visual scan  multiposa temporaneamente disabilitata.
# VISION_SEARCH_TARGETS = (
#     ARM_APPROACH_TARGET,
#     (-0.6, 0.0, 0.75),
#     (-0.7, -0.2, 0.75),
#     (-0.7, 0.2, 0.75),
# )
PICK_HEIGHT = 0.15
DROP_HEIGHT = 0.15
PACKAGE_MASS_KG = 2.0
# soglie di raggiunta della mappa
WAYPOINT_THRESHOLD = 2
FINAL_THRESHOLD = 0.10
UNLOAD_FINAL_THRESHOLD = 1
# ogni punto va downscalato per la grandezza della scala per avere il riferimento godot.
# punto a
PICK_START_MAP = (50, 50)
# punto b
PICK_TARGET_MAP = (175, 1375)
# zone di drop associate al colore su mission_types
UNLOAD_TARGET_MAP = {
    UnloadZoneCode.A: (50, 50),
    UnloadZoneCode.C: (900, 1975),#925, 2025 #900, 2050
}


# calcolo distanza tra posa robot e waypoint , se minore di soglia true.
def has_reached(x, y, target, threshold):
    target_x, target_y = target
    return math.dist((x,y), (target_x,target_y)) < threshold

# calcolo della distnza e orientamento tra posa braccio e posa desiderata. restituisce true se le treshold sono soddisfatte.
def arm_has_reached(
    arm : FourJointsManipulatorControl,
    target: tuple[float, float, float],
    position_threshold=0.005,
    angle_threshold=math.radians(3),
):
    x, y, z, angle = arm.get_pose()
    target_x, target_y, target_z = target
    distance = math.dist((x,y,z), (target_x, target_y, target_z))
# normalizzazione errore di handling.
    angle_error = math.atan2(
        math.sin(angle - ARM_ORIENTATION),
        math.cos(angle - ARM_ORIENTATION),
    )
    return distance < position_threshold and abs(angle_error) < angle_threshold

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

# discretizzazione dell'ambiente, aggiunta di ostacolo fisso e pacco.
def build_world():
    world = World(2500, 2500, 50)
    world.add_rectangle_obstacle(525, 1500, 675, 1900) #rettangolo 575, 1550, 625, 1850 (12,34) larghezza (1,5.488), +  a l for margin
    world.add_rectangle_obstacle(37.5, 1362.5, 62.5, 1387.5) #pacco pos (1, 27,5), larghezza 0,5
    return world
# calcolo generica traiettoria
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

# se un insieme di punti è attraversato da una linea retta prende solo l'inizio e la fine, 
#tralasciando i punti intermedi
def build_angle_search(path):
    new_path= []
    index =0
    print("path. prec", path)
    new_path.append(path[index])
    print("len_path", len(path)-1)
    while index <= len(path)-1:
        distanceX= path[index][0] - path[index+1][0]
        distanceY= path[index][1] - path[index+1][1]
        while index < len(path)-1 and distanceX == (path[index][0] - path[index+1][0]) and distanceY == (path[index][1] - path[index+1][1]):
            print("dentro dentro")
            index+=1
        new_path.append(path[index])
        index+=1
    print("path actual", new_path)
    return new_path


# calcolo con _path il tragitto per il punto b di carico.
def build_path_to_pick_zone():
    # Il punto resta allineato a B, ma il carrello si ferma accanto al pacco.
    return build_path(PICK_START_MAP, PICK_TARGET_MAP)

# calcola tragitto in base alla zona di scarico.
def build_path_to_unload_zone(unload_zone):
    return build_path(PICK_TARGET_MAP, UNLOAD_TARGET_MAP[unload_zone])

# ciclo di inseguimento dei waypoint con le relative soglie in base al waypoint intermedio / finale e print di debug.
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
        print("Target", target)


    if not has_reached(*ackermann.pose[:2], target, threshold):
        return index, target, False

    if is_last:
        return index, target, True

    #ha raggiunto la cella, ora deve puntare alla prossima 
    index += 1
    target = path[index]
    ackermann.start_motion(target)
    return index, target, False

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
    #orientation_plot.add_y("arm_alpha", "Orientamento braccio (gradi)")

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
        print("dnetro")
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

    try:
        dds.start()
        dds.subscribe(["tick"])
        # La connessione resta pronta mentre il braccio raggiunge la posa visuale.
        camera.connect()
        camera_connected = True

        path = build_angle_search(build_path_to_pick_zone())
        print(f"Percorso verso B: {len(path)} punti, arrivo {path[-1]}")

        ackermann = AckermannController()
        ackermann.set_pose(1.0, -1, 0.0) #1 -1 #-3 20
        arm = FourJointsManipulatorControl()
        print(f"L1={arm.arm.element_1.L} L2={arm.arm.element_2.L} L3={arm.arm.element_3.L}")
        visual_picker = VisualPickController(camera.width, camera.height)
        plotters = build_plotters()

        mission = MissionStateId.DRIVE_TO_B
        cargo_color = CargoColorCode.UNKNOWN
        unload_zone = UnloadZoneCode.UNKNOWN #A o C
        payload_mass = 0.0
        waypoint_index = 0
        target = path[waypoint_index]
        pick_xy = None
        missing_frames = 0
        arm_settle_frames = 0
        vision_target = ARM_APPROACH_TARGET_1
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
                    mission= MissionStateId.DRIVE_TO_UNLOAD
                    payload_mass = PACKAGE_MASS_KG
                    ackermann.set_payload_mass(payload_mass)
                    cargo_color =  CargoColorCode.RED
                    unload_zone = UNLOAD_ZONE_BY_COLOR[cargo_color]
                    #mission = MissionStateId.DONE
                    #completed = True
                    path = build_path_to_unload_zone(unload_zone)
                    path = build_angle_search(path)
                    waypoint_index = 0
                    target = path[waypoint_index]
                    ackermann.start_motion(target)
                    print("Pacco preso")

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
                    if unload_zone == UnloadZoneCode.C:
                        arm_approach= ARM_APPROACH_TARGET_2
                    else:
                        arm_approach=ARM_APPROACH_TARGET_1
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
                if arm_has_reached(arm, drop_target, position_threshold=0.2):
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
