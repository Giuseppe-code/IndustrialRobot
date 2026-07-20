
                    # Cambio posa di visual scan dopo 30 frame, 4 pose circolari  temporaneamente disabilitato.
                    # if missing_frames >= 30:
                    #     vision_pose_index = (
                    #         vision_pose_index + 1
                    #     ) % len(VISION_SEARCH_TARGETS)
                    #     vision_target = VISION_SEARCH_TARGETS[vision_pose_index]
                    #     if not arm.set_target(
                    #         *vision_target,
                    #         ARM_ORIENTATION,
                    #     ):
                    #         raise RuntimeError(
                    #             "La posa di ricerca non e' raggiungibile"
                    #         )
                    #     arm_settle_frames = 0
                    #     missing_frames = 0
                    #     mission = MissionStateId.PICK_APPROACH
                    #     print(
                    #         "VISION_PICK: nuova posa di ricerca "
                    #         f"{vision_pose_index + 1}/"
                    #         f"{len(VISION_SEARCH_TARGETS)}"
                    #     )
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
                _vmax = 1, 
                _acc = 0.55,
                _dec= 0.75)
    
        self.polar2D = Polar2DController(
                KP_linear=1,
                v_max=3,
                KP_heading=20,
                steering_max=math.radians(40)
            )