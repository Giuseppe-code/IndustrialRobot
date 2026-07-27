from path_planning.nf1 import NF1Planner
from path_planning.world import World
import math
from controller.mission_types import (
    CargoColorCode,
    MissionStateId,
    UNLOAD_ZONE_BY_COLOR,
    UnloadZoneCode,
)

from controller.AckermannController import AckermannController

# calcolo distanza tra posa robot e waypoint , se minore di soglia true.
def has_reached(x, y, target, threshold):
    target_x, target_y = target
    return math.dist((x,y), (target_x,target_y)) < threshold

WAYPOINT_THRESHOLD = 1.3
FINAL_THRESHOLD = 0.10
UNLOAD_FINAL_THRESHOLD = 0.10
# ogni punto va downscalato per la grandezza della scala per avere il riferimento godot.
# punto a
PICK_START_MAP = (50, 50)
# punto b
PICK_TARGET_MAP = (175, 1375)
# zone di drop associate al colore su mission_types
UNLOAD_TARGET_MAP = {
    UnloadZoneCode.A: (100, 50),
    UnloadZoneCode.C: (900, 1975),#925, 2025 #900, 2050
}


class PathPlanning:
    def __init__(self, ackermann : AckermannController):
        self.world = World(2500, 2500, 50)
        self.add_obstacle()
        self.path= []
        self.ackermann = ackermann
        self.waypoint_index = 0
        self.index = 0
        self.target = []
        self.step = 0

    def add_obstacle(self):
        self.world.add_rectangle_obstacle(525, 1550, 675, 1850) #rettangolo 575, 1550, 625, 1850 (12,34) larghezza (1,5.488), +  a l for margin
        self.world.add_rectangle_obstacle(37.5, 1350, 62.5, 1450) #pacco pos (1, 27,5), larghezza 0,5 #37.5, 1362.5, 62.5, 1387.5
        return 

    def set_target(self):
        self.target= self.path[self.waypoint_index]

    # calcolo generica traiettoria
    def build_path(self, start_map, target_map):
        path_map = NF1Planner(self.world)
        path_map=path_map.plan(start_map, target_map)
        #img = path_map.world_to_image()
        #cv2.imshow("world", img)
        #cv2.waitKey(0)
        path = []
        for x, y in path_map:
            world_x, world_y = self.world.to_world(x, y)
            path.append((world_x / self.world.scale, -world_y / self.world.scale)) ##offset to angle
        return path
    
    #dato un path formato da più sequenze di punti in linea retta crea un nuovo path 
    # con i solo punti iniziali e finali
    def build_angle_search(self, path):
        new_path= []
        index =0
        print("path. prec", path)
        new_path.append(path[index])
        print("len_path", len(path)-1)
        while index < len(path)-1:
            distanceX= path[index][0] - path[index+1][0]
            distanceY= path[index][1] - path[index+1][1]
            while index < len(path)-1 and distanceX == (path[index][0] - path[index+1][0]) and distanceY == (path[index][1] - path[index+1][1]):
                index+=1
            new_path.append(path[index])
            index+=1
        print("path actual", new_path)
        self.path = new_path
        self.set_target()
        return
    
    # calcolo con _path il tragitto per il punto b di carico.
    def build_path_to_pick_zone(self):
        # Il punto resta allineato a B, ma il carrello si ferma accanto al pacco.
        return self.build_path(PICK_START_MAP, PICK_TARGET_MAP)

    # calcola tragitto in base alla zona di scarico.
    def build_path_to_unload_zone(self, unload_zone):
        return self.build_path(PICK_TARGET_MAP, UNLOAD_TARGET_MAP[unload_zone])
    

    # ciclo di inseguimento dei waypoint con le relative soglie in base al waypoint intermedio / finale e print di debug.
    def follow_path(self,
        delta_t : float,
        label: str,
        final_threshold=FINAL_THRESHOLD,##final_threshold o unload_final_threshold
    ):
        self.ackermann.evaluate(delta_t)
        is_last = self.index == len(self.path) - 1
        threshold = final_threshold if is_last else WAYPOINT_THRESHOLD
        
        if self.step % 60 == 0:
            distance = math.dist(self.ackermann.pose[:2], self.target)
            print(f"{label}: punto {self.index + 1}/{len(self.path)}, distanza {distance:.2f} m")
            print("Target", self.target)


        
        if not has_reached(*self.ackermann.pose[:2], self.target, threshold):
            return False

        if is_last:
            return True

        #ha raggiunto la cella, ora deve puntare alla prossima
        self.step=+1 
        self.index += 1
        self.target = self.path[self.index]
        self.ackermann.start_motion(self.target)
        return False
    
    def reset(self):
        self.waypoint_index = 0
        self.index = 0
        self.set_target()
        self.step = 0
        return self.target