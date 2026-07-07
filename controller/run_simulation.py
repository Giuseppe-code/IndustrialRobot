import argparse
import json
import math
import sys
from pathlib import Path
import sys

sys.path.append("../")

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
#from manipulator_control import FourJointsManipulatorControl
#from ackermann_mission import AckermannMissionConfig, AckermannMissionController
from path_planning.world import World
from path_planning.nf1 import NF1Planner
from mission_types import MissionStateId 

def has_reached(xa,ya, target, soglia):
    x, y = target
    threshold =math.sqrt((xa-x)**2 +(ya-y)**2)
    print("threshold attuale", threshold)
    return soglia > threshold
    


def publish_state(dds, controller):
    x, y, theta = controller.pose
    dds.publish("X", x, DDS.DDS_TYPE_FLOAT)
    dds.publish("Y", y, DDS.DDS_TYPE_FLOAT)
    dds.publish("Theta", theta, DDS.DDS_TYPE_FLOAT)
    #dds.publish("SteeringAngle", controller.ackermann.alpha_c, DDS.DDS_TYPE_FLOAT)

def main():
    dds = DDS()
    dds.start()
    dds.subscribe(["tick", "TerrainHeight", "TerrainSlope"])

    w = World(1500, 1500, 50) #100, se parto da 1000 ho 50 posizioni
    #w.add_rectangle_obstacle(31, -11.5, 37, 12.5) #punti :31 -11.5, 37 -12.5
    nf1 = NF1Planner(w)
    path_BA= nf1.plan((50,50),(50, 1375)) ##0 0 
    
    #img = nf1.world_to_image()
    #cv2.imshow("world", img)
    #cv2.waitKey(0)
    print(path_BA)
    threashold = 1.50
    

    mission = MissionStateId.DRIVE_TO_B
    
    ackermann = AckermannController()
    t = Time()
    t.start()
    index = 5
    step = 0
    while t.get() < 50:  
        delta_t =  dds.read("tick") 
        print(f"delta_t ricevuto: {delta_t}")

        print(ackermann.pose[0], ackermann.pose[1], path_BA[index])
        path_BA[index]= (0,-20)
        if mission is MissionStateId.DRIVE_TO_B:
            ##mettere che lo start è la pose dove mi trovo, quando arrivo alla soglia devo aumentare
            
            ackermann.start_motion(path_BA[index])
            ackermann.evaluate(delta_t)
            ##distanza
            if has_reached(ackermann.pose[0], ackermann.pose[1], path_BA[index], threashold):
                index= index+1
                print("ha raggiunto l'obbiettivo")


        ##else:
        #    return
        publish_state(dds, ackermann)




if __name__ == "__main__":
    raise SystemExit(main())
