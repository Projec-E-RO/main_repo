
gripper_reaction = self.PICK_POS if msg.gripper_open else self.OPEN_POS  # 먼저 그리퍼를 제어하고, 완료 후 팔 이동

self._control_gripper(
    gripper_position,
    gripper_action, 
    lambda: self._start_arm_movement(target_pose)
)