from neuromeka import IndyClient3, IndyClient3CommonMsgs

indy = IndyClient3("192.168.0.122")

sys_info = indy.get_system_info()
print("System Information (Model name): ", sys_info.model_name)
print("System Information (Serial Number): ", sys_info.robot_sn)

control_data = indy.get_control_data()
print("Joint Rotations: ", control_data.q)
print("Task Position: ", control_data.p)

print("Moving to the new position")
movej_result = indy.movej(jpos=[74.57, 9.54, 99.05, 0.23, 71.33, 0.0], blending_type=IndyClient3CommonMsgs.BLENDING_TYPE_NONE)
movej_result = indy.movej(jpos=[0.0, 0.0, 0.0, 0.0, 0.0, -45.0], base_type=IndyClient3CommonMsgs.JOINT_BASE_TYPE_RELATIVE)

print("Moving to the original position")
movej_result = indy.movej(jpos=control_data.q)