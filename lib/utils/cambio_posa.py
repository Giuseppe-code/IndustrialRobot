
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