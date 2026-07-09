import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))
sys.path.append(str(ROOT / "path_planning"))

import cv2
from matplotlib import pylab

from lib.dds.dds import DDS
from lib.dds.image_reader import ImageReader
from lib.data.dataplot import DataPlotter
from lib.system.cart import AckermannSteering
from lib.system.trajectory import StraightLine2DMotion
from lib.system.polar import Polar2DController
from lib.system.controllers import PID_Controller
from AckermannController import AckermannController
#from lib.system.cart import AckermannSlopeLoad, MeasuredTerrainProfile
#from lib.system.object_finder import ObjectFinder
from lib.utils.time import Time
from lib.system.manipulator_control import FourJointsManipulatorControl
from path_planning.world import World
from path_planning.nf1 import NF1Planner
from mission_types import MissionStateId 



def has_reached(xa,ya, target, soglia):
    x, y = target
    return soglia > math.sqrt((xa-x)**2 +(ya-y)**2)
    


def publish_state(dds, ackermann: AckermannController, robot: FourJointsManipulatorControl):
    #ackerman
    xA, yA, thetaA = ackermann.pose
    dds.publish("X", xA, DDS.DDS_TYPE_FLOAT)
    dds.publish("Y", yA, DDS.DDS_TYPE_FLOAT)
    dds.publish("Theta", thetaA, DDS.DDS_TYPE_FLOAT)

    #arm
    (t0, t1, t2, t3) = robot.get_joint_angles()
    (xR, yR, zR, aR) = robot.get_pose()

    dds.publish('theta0', t0, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta1', t1, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta2', t2, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta3', t3, DDS.DDS_TYPE_FLOAT)
    dds.publish('x', xR, DDS.DDS_TYPE_FLOAT)
    dds.publish('y', yR, DDS.DDS_TYPE_FLOAT)
    dds.publish('z', zR, DDS.DDS_TYPE_FLOAT)
    dds.publish('a', aR, DDS.DDS_TYPE_FLOAT)
    #dds.publish("SteeringAngle", controller.ackermann.alpha_c, DDS.DDS_TYPE_FLOAT)

def main():
    dds = DDS()
    dds.start()
    dds.subscribe(["tick", "TerrainHeight", "TerrainSlope"])

    w = World(2000, 2000, 50) #100, se parto da 1000 ho 50 posizioni
    w.add_rectangle_obstacle(575, 1550, 625, 1850) # ostacolo Godot x=31..37, z=-11.5..-12.5
    w.add_rectangle_obstacle(0, 1362.5, 12.5, 1387.5) # pacco in zoneB, ingombro Godot x=27.25..27.75, z=-0.25..0.25
    nf1 = NF1Planner(w)
    path_BA_map = nf1.plan((50,50),(50, 1375)) ##1 1 ]]50 ##1375
    path_BA = []
    for x, y in path_BA_map:
        wx, wy = w.to_world(x, y)
        path_BA.append((wx / w.scale, -wy / w.scale))
    
    #img = nf1.world_to_image()
    #cv2.imshow("world", img)
    #cv2.waitKey(0)
    print("path_BA:", path_BA)
    threashold = 1.50
    index = 0
    step = 0
    elapsed_time = 0.0

    mission = MissionStateId.DRIVE_TO_B
    
    ackermann = AckermannController()
    arm = FourJointsManipulatorControl()

    ackermann.pose=(1,1,0)
    
    target = path_BA[index]
    ackermann.start_motion(target)

    x_target_arm = 0.15
    y_target_arm = 0.15
    z_target_arm = 0.5
    arm.set_target(x_target_arm, y_target_arm, z_target_arm, math.radians(-90))


    while elapsed_time < 50:
        # usiamo wait per evitare letture none
        delta_t = dds.wait("tick")
        elapsed_time += delta_t
        
        if mission is MissionStateId.DRIVE_TO_B:
            arm.evaluate(delta_t)
            ##mettere che lo start è la pose dove mi trovo, quando arrivo alla soglia devo aumentare
            ackermann.evaluate(delta_t)
            distance = math.sqrt((ackermann.pose[0]-target[0])**2 +(ackermann.pose[1]-target[1])**2)
            if step % 30 == 0:
                print("step", step, "target", index, target, "pose", ackermann.pose, "distance", distance)
            ##distanza
            if has_reached(ackermann.pose[0], ackermann.pose[1], target, threashold):
                index= index+1
                print("ha raggiunto un obiettivo")
                if index >= len(path_BA):
                    mission = MissionStateId.PICK_APPROACH
                    print("fase di Pick Iniziata")
                    break
                target = path_BA[index]
                ackermann.start_motion(target)

        if mission is MissionStateId.PICK_APPROACH:
            print("Fase pick iniziata")
            arm.set_target(x_target_arm, y_target_arm, z_target_arm, math.radians(-90))


        ##else:
        #    return
        publish_state(dds, ackermann, arm)
        step += 1




if __name__ == "__main__":
    raise SystemExit(main())
