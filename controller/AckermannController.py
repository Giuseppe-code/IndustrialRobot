from lib.system.cart import AckermannSteering
from lib.system.trajectory import StraightLine2DMotion
from lib.system.polar import Polar2DController
from lib.system.controllers import PID_Controller

import math

class AckermannController:
    def __init__(self):
        self.base_mass = 5.0
        self.payload_mass = 0.0
        self.ackermann = AckermannSteering(
            _mass = self.base_mass,
            _lin_friction = 0.8,
            _r_traction = 0.5,
            _lateral_wheelbase= 1)
        self.trajectory =StraightLine2DMotion( 
                _vmax = 4, 
                _acc = 4.5,
                _dec= 4.5)
    
        self.polar2D = Polar2DController(
                KP_linear=2,
                v_max=4,
                KP_heading=1.5,
                steering_max=math.radians(40)
            )

        self.linearSpeedController = PID_Controller(20, 8, 0, 100)
        self.pose=[0,0,0]
        self.v=0
        self.vt=0
        self.alpha_c=math.radians(0)

    def set_pose(self, x, y, theta):
        self.pose = [x, y, theta]
        self.ackermann.x = x
        self.ackermann.y = y
        self.ackermann.theta = theta

    def set_payload_mass(self, payload_mass):
        self.payload_mass = max(0.0, payload_mass)
        self.ackermann.M = self.base_mass + self.payload_mass
    
    def start_motion(self, end):
        self.trajectory.start_motion((self.pose[0], self.pose[1]),end)

    def evaluate(self, delta_t):
        (self.v, w)=self.ackermann.get_speed()
        self.pose=self.ackermann.get_pose()

        (xt, yt) = self.trajectory.evaluate(delta_t)
        (self.vt, self.alpha_c) = self.polar2D.evaluate(delta_t=delta_t, xt=xt, yt=yt, current_pose=self.pose)
        error_v= self.vt-self.v
        # Stampe sostituite dal log generale su run_simulation.py.
        # print("Error_v", error_v)
        T= self.linearSpeedController.evaluate(delta_t=delta_t, _error= error_v)
        # print("Forza applicata T", T)
        #ackermannVehicle
        self.ackermann.evaluate(delta_t=delta_t, torque=T, steering_angle=self.alpha_c)
        
       
