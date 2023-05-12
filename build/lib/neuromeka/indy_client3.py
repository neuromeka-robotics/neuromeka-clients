import sys
from operator import add
import json
import grpc
from google.protobuf.json_format import ParseDict
import math
import time

from .proto.control_pb2_grpc import ControlStub
from .proto.control_pb2 import *
from .proto import control_msgs_pb2 as control_msgs
from .proto import shared_msgs_pb2 as IndyClient3CommonMsgs


JogVelRatioDefault = 15  # %
JogAccRatioDefault = 100  # %

def grpc_mapping_call(control_stub, func_name, arg_dict=None):
    if arg_dict is None:
        arg_dict = {}
    grpc_request = getattr(control_msgs, "{}Req".format(func_name))()
    ParseDict(arg_dict, grpc_request)
    try:
        grpc_response = getattr(control_stub, func_name)(grpc_request)

    except grpc.RpcError as rpc_error:
        print('Robot gRPC: ' + func_name + ': Connection unavailable')
        grpc_response = None

    except Exception as ex:
        print(func_name + 'Robot gRPC Exception: ' + str(ex))
        grpc_response = None

    return grpc_response

class IndyClient3:
    def __init__(self, step_ip='127.0.0.1'):
        # initialize RPC        
        self.channel = grpc.insecure_channel('%s:20132'%step_ip)
        self.stub = ControlStub(self.channel)
        self._target_pos = [2, [], []]

    def __del__(self):
        if self.channel is not None:
            self.channel.close()

    ############################
    # Motion Control
    ############################

    # Default Motion
    def movej(self, jpos,
              blending_type=IndyClient3CommonMsgs.BLENDING_TYPE_NONE,
              base_type=IndyClient3CommonMsgs.JOINT_BASE_TYPE_ABSOLUTE,
              blending_radius=0.0,
              vel_ratio=JogVelRatioDefault,
              acc_ratio=JogAccRatioDefault):
        self._target_pos[0] = 1
        if base_type == IndyClient3CommonMsgs.JOINT_BASE_TYPE_ABSOLUTE:
            self._target_pos[1] = jpos
        else:
            self._target_pos[1] = self.get_control_data().q
            self._target_pos[1] = list(map(add, self._target_pos[1], jpos))

        movejv_dict = dict(jpos=list(jpos),
                           blending_type=blending_type,
                           base_type=base_type,
                           blending_radius=blending_radius,
                           vel=vel_ratio,
                           acc=acc_ratio)
        result = grpc_mapping_call(self.stub, "AMoveJ", movejv_dict)
        return result

    def movej_time(self, jpos, move_time=2.0,
                   blending_type=IndyClient3CommonMsgs.BLENDING_TYPE_NONE,
                   base_type=IndyClient3CommonMsgs.JOINT_BASE_TYPE_ABSOLUTE,
                   blending_radius=0.0):
        """
        jpos = [deg, deg, deg, deg, deg, deg]
        move_time = seconds
        """

        self._target_pos[0] = 1
        if base_type == IndyClient3CommonMsgs.JOINT_BASE_TYPE_ABSOLUTE:
            self._target_pos[1] = jpos
        else:
            self._target_pos[1] = self.get_control_data().q
            self._target_pos[1] = list(map(add, self._target_pos[1], jpos))

        movejt_dict = dict(jpos=list(jpos),
                           time=move_time,
                           blending_type=blending_type,
                           base_type=base_type,
                           blending_radius=blending_radius)
        result = grpc_mapping_call(self.stub, "AMoveJT", movejt_dict)
        return result

    def movel(self, tpos,
              blending_type=IndyClient3CommonMsgs.BLENDING_TYPE_NONE,
              base_type=IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE,
              blending_radius=0.0,
              vel_ratio=JogVelRatioDefault,
              acc_ratio=JogAccRatioDefault):
        """
        tpos = [mm, mm, mm, deg, deg, deg]
        vel_level : 1 ~ 9
        """
        self._target_pos[0] = 0
        if base_type == IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE:
            self._target_pos[2] = tpos
        else:
            self._target_pos[2] = self.get_control_data().p
            self._target_pos[2] = self._task_relative_rotation(tpos, self._target_pos[2])

        movelv_dict = dict(tpos=list(tpos),
                           blending_type=blending_type,
                           base_type=base_type,
                           blending_radius=blending_radius,
                           vel=vel_ratio,
                           acc=acc_ratio)
        result = grpc_mapping_call(self.stub, "AMoveL", movelv_dict)
        return result

    def movel_time(self, tpos, move_time=2.0,
                   blending_type=IndyClient3CommonMsgs.BLENDING_TYPE_NONE,
                   base_type=IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE,
                   blending_radius=0.0):
        """
        tpos = [mm, mm, mm, deg, deg, deg]
        move_time = seconds
        """
        self._target_pos[0] = 0
        if base_type == IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE:
            self._target_pos[2] = tpos
        else:
            self._target_pos[2] = self.get_control_data().p
            self._target_pos[2] = self._task_relative_rotation(tpos, self._target_pos[2])

        movelt_dict = dict(tpos=list(tpos),
                           blending_type=blending_type,
                           base_type=base_type,
                           blending_radius=blending_radius,
                           time=move_time)
        result = grpc_mapping_call(self.stub, "AMoveLT", movelt_dict)
        return result

    def movec(self, tpos0, tpos1, angle=90.0,
              blending_type=IndyClient3CommonMsgs.BLENDING_TYPE_NONE,
              base_type=IndyClient3CommonMsgs.JOINT_BASE_TYPE_ABSOLUTE,
              blending_radius=0.0,
              setting_type=IndyClient3CommonMsgs.POINT_SET,
              move_type=IndyClient3CommonMsgs.CONSTANT,
              vel_ratio=JogVelRatioDefault,
              acc_ratio=JogAccRatioDefault):

        self._target_pos[0] = 0
        if base_type == IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE:
            self._target_pos[2] = tpos1
        else:
            self._target_pos[2] = self.get_control_data().p
            self._target_pos[2] = self._task_relative_rotation(tpos1, self._target_pos[2])

        movecv_dict = dict(tpos0=list(tpos0),
                           tpos1=list(tpos1),
                           angle=angle,
                           blending_type=blending_type,
                           base_type=base_type,
                           blending_radius=blending_radius,
                           setting_type=setting_type,
                           move_type=move_type,
                           vel=vel_ratio,
                           acc=acc_ratio)
        result = grpc_mapping_call(self.stub, "AMoveC", movecv_dict)
        return result

    def movec_time(self, tpos0, tpos1, angle=90.0,
                   blending_type=IndyClient3CommonMsgs.BLENDING_TYPE_NONE,
                   base_type=IndyClient3CommonMsgs.JOINT_BASE_TYPE_ABSOLUTE,
                   blending_radius=0.0,
                   setting_type=IndyClient3CommonMsgs.POINT_SET,
                   move_type=IndyClient3CommonMsgs.CONSTANT,
                   move_time=2.0):

        self._target_pos[0] = 0
        if base_type == IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE:
            self._target_pos[2] = tpos1
        else:
            self._target_pos[2] = self._task_relative_rotation(tpos1, self._target_pos[2])

        movect_dict = dict(tpos0=list(tpos0),
                           tpos1=list(tpos1),
                           angle=angle,
                           blending_type=blending_type,
                           base_type=base_type,
                           blending_radius=blending_radius,
                           setting_type=setting_type,
                           move_type=move_type,
                           time=move_time)
        result = grpc_mapping_call(self.stub, "AMoveCT", movect_dict)
        return result

    def wait_io(self, di_list, do_list, end_di_list, end_do_list, conjunction=0,
                set_do_list=None, set_end_do_list=None, set_ao_list=None, set_end_ao_list=None):
        """
        do_list = List[Tuple[int, bool]]
        di_list = List[Tuple[int, bool]]
        """
        io_list_dict = dict(
            do_list=[{'addr': item.addr, 'on': item.on} for item in do_list],
            di_list=[{'addr': item.addr, 'on': item.on} for item in di_list],
            end_di_list=[{'addr': item.addr, 'on': item.on} for item in end_di_list],
            end_do_list=[{'addr': item.addr, 'on': item.on} for item in end_do_list],
            conjunction=conjunction,
            set_do_list=[{'addr': item.addr, 'on': item.on} for item in set_do_list],
            set_end_do_list=[{'addr': item.addr, 'on': item.on} for item in set_end_do_list],
            set_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in set_ao_list],
            set_end_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in set_end_ao_list]
        )
        result = grpc_mapping_call(self.stub, "AWaitIO", io_list_dict)
        return result

    def wait_time(self, wait_sec=0,
                  set_do_list=None, set_end_do_list=None, set_ao_list=None, set_end_ao_list=None):
        """
        wait_sec = seconds
        """
        waittime_dict = dict(time=wait_sec,
                             set_do_list=[{'addr': item.addr, 'on': item.on} for item in set_do_list],
                             set_end_do_list=[{'addr': item.addr, 'on': item.on} for item in set_end_do_list],
                             set_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in set_ao_list],
                             set_end_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in set_end_ao_list])
        result = grpc_mapping_call(self.stub, "AWaitTime", waittime_dict)
        return result

    def wait_progress(self, progress=100,
                      set_do_list=None, set_end_do_list=None, set_ao_list=None, set_end_ao_list=None):
        """
        progress = 0 ~ 100%
        """
        waitprogress_dict = dict(progress=progress,
                                 set_do_list=[{'addr': item.addr, 'on': item.on} for item in set_do_list],
                                 set_end_do_list=[{'addr': item.addr, 'on': item.on} for item in set_end_do_list],
                                 set_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in set_ao_list],
                                 set_end_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in
                                                  set_end_ao_list])
        result = grpc_mapping_call(self.stub, "AWaitProgress", waitprogress_dict)
        return result

    def wait_traj(self, traj_condition,
                  set_do_list=None, set_end_do_list=None, set_ao_list=None, set_end_ao_list=None):
        result = grpc_mapping_call(self.stub, "AWaitTraj",
                                   dict(traj_condition=traj_condition,
                                        set_do_list=[{'addr': item.addr, 'on': item.on} for item in set_do_list],
                                        set_end_do_list=[{'addr': item.addr, 'on': item.on} for item in
                                                         set_end_do_list],
                                        set_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in
                                                     set_ao_list],
                                        set_end_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in
                                                         set_end_ao_list]))
        return result

    def wait_radius(self, radius=100,
                    set_do_list=None, set_end_do_list=None, set_ao_list=None, set_end_ao_list=None):
        """
        radius in mm
        """
        waitradius_dict = dict(radius=radius,
                               set_do_list=[{'addr': item.addr, 'on': item.on} for item in set_do_list],
                               set_end_do_list=[{'addr': item.addr, 'on': item.on} for item in set_end_do_list],
                               set_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in set_ao_list],
                               set_end_ao_list=[{'addr': item.addr, 'voltage': item.voltage} for item in
                                                set_end_ao_list])
        result = grpc_mapping_call(self.stub, "AWaitRadius", waitradius_dict)
        return result

    def move_stop(self):
        """
        stop move element
        """
        result = grpc_mapping_call(self.stub, "AMotionStop")
        return result

    # Motion Config
    def set_ref_frame(self, ref_frame):
        result = grpc_mapping_call(self.stub, "SetRefFrame", dict(ref_frame=list(ref_frame)))
        return result

    def set_ref_frame_planar(self, tpos0, tpos1, tpos2):
        result = grpc_mapping_call(self.stub, "SetRefFramePlanar", dict(tpos0=list(tpos0),
                                                                                  tpos1=list(tpos1),
                                                                                  tpos2=list(tpos2)))
        return result

    def set_tcp_frame(self, tcp_frame):
        result = grpc_mapping_call(self.stub, "SetToolFrame", dict(tool_frame=list(tcp_frame)))
        return result

    def set_speed_ratio(self, speed_ratio=50):
        """
        speed_ratio = 0 ~ 100
        """
        # print("set_speed_ratio: " + str(speed_ratio))
        result = grpc_mapping_call(self.stub, "SetSpeedRatio", dict(ratio=speed_ratio))
        return result

    def set_command_speed_ratio(self, command_speed_ratio=50):
        """
        command_speed_ratio = 0 ~ 100
        """
        # print("set_command_speed_ratio: " + str(command_speed_ratio))
        result = grpc_mapping_call(self.stub, "SetCommandSpeedRatio", dict(ratio=command_speed_ratio))
        return result

    ############################
    # Command
    ############################

    def reboot(self):
        result = grpc_mapping_call(self.stub, "Reboot")
        return result

    def recover(self):
        """
        Recover from violation
        """
        result = grpc_mapping_call(self.stub, "Recover")
        return result

    def stop_motion(self, stop_category=IndyClient3CommonMsgs.STOP_CAT_REDUCE_SPEED):
        """
            STOP_CAT_IMMEDIATE_BRAKE=0;
            STOP_CAT_REDUCE_SPEED_AND_BRAKE =1;
            STOP_CAT_REDUCE_SPEED=2;
        """
        result = grpc_mapping_call(self.stub, "Stop", dict(stop_category=stop_category))
        return result

    def pause_motion(self, pause_category=IndyClient3CommonMsgs.PAUSE_CAT_REDUCE_SPEED):
        """
            PAUSE_CAT_IMMEDIATE=0;
            PAUSE_CAT_REDUCE_SPEED=1;
        """
        result = grpc_mapping_call(self.stub, "Pause", dict(pause_category=pause_category))
        return result

    def resume_motion(self):
        result = grpc_mapping_call(self.stub, "Resume")
        return result

    def set_brake(self, on_list: list):
        result = grpc_mapping_call(self.stub, "Brake", dict(on_list=list(on_list)))
        return result

    def set_servo(self, on: bool):
        result = grpc_mapping_call(self.stub, "Servo", dict(on=on))
        return result

    def set_auto_servooff(self, enable=True, time=20.0):
        result = grpc_mapping_call(self.stub, "SetAutoServoOff", dict(enable=enable, time=time))
        return result

    def get_auto_servooff(self):
        result = grpc_mapping_call(self.stub, "GetAutoServoOff")
        return result

    def set_simulation_mode(self, on: bool):
        result = grpc_mapping_call(self.stub, "SimulationMode", dict(on=on))
        return result
    
    def set_direct_teaching(self, enable=True):
        """
        enable = True | False
        """
        result = grpc_mapping_call(self.stub, "DirectTeachingMode", dict(on=enable))
        return result

    ############################
    # Data
    ############################

    def get_control_data(self):
        """
        Control Data:
            q           -> float[]
            qdot        -> float[]
            qddot       -> float[]
            p           -> float[]
            pdot        -> float[]
            pddot       -> float[]
            ref_frame   -> float[]
            tool_frame  -> float[]
        """
        result = grpc_mapping_call(self.stub, "GetRTControlData")
        return result

    def get_io_data(self):
        """
        IO Data:
            di      -> bool[]
            do      -> bool[]
            ai      -> int32[]
            ao      -> int32[]
            end_di  -> bool[]
            end_do  -> bool[]
            end_ai  -> bool[]
            end_ao  -> bool[]
        """
        result = grpc_mapping_call(self.stub, "GetIOData")
        return result

    def get_core_data(self):
        """
        CORE Data:
            temperatures    -> float[]
            voltages        -> float[]
            currents        -> float[]
            states          -> string[]
            state_codes     -> string[]
            brake_states    -> bool[]
            servo_state     -> bool
        """
        result = grpc_mapping_call(self.stub, "GetCoreData")
        return result
    
    def get_system_info(self):
        """
        System Info:
            control_task_ver        -> string
            dof                     -> int32
            model_name              -> string
            robot_sn                -> string
            io_board_fw_ver         -> string
            core_board_fw_vers[]    -> string[]
            endtool_board_fw_ver    -> string
            endtool_port_type
                                    END_TOOL_PORT_TYPE_A = 0
                                    END_TOOL_PORT_TYPE_B = 1
                                    END_TOOL_PORT_TYPE_AB = 2
        """
        result = grpc_mapping_call(self.stub, "GetSystemInfoData")
        return result

    def get_motion_data(self):
        """
        Motion Info:
            traj_progress     -> int32
            is_in_motion      -> bool
            is_motion_done    -> bool
            is_pausing        -> bool
            is_stopping        -> bool
            has_motion        -> bool
            speed_ratio       -> int32
            motion_queue_size -> int32
            msg               -> string
            traj_state
                            TRAJSTATE_NONE = 0;
                            TRAJSTATE_INIT = 1;
                            TRAJSTATE_CALC = 2;
                            TRAJSTATE_STBY = 3;
                            TRAJSTATE_ACC = 4;
                            TRAJSTATE_CRZ = 5;
                            TRAJSTATE_DEC = 6;
                            TRAJSTATE_FIN = 7;
                            TRAJSTATE_CANC = 8;
                            TRAJSTATE_ERR = 9;
        """
        result = grpc_mapping_call(self.stub, "GetMotionData")
        return result

    def get_control_state_data(self):
        """
        Control State:
            violation           -> string
            is_simulation_mode  -> bool
            state               -> OpState
                OP_SYSTEM_OFF=0;
                OP_SYSTEM_ON=1;
                OP_VIOLATE=2;
                OP_RECOVER_HARD=3;
                OP_RECOVER_SOFT=4;
                OP_IDLE=5;
                OP_MOVING=6;
                OP_TEACHING=7;
                OP_COLLISION=8;
                OP_STOP_AND_OFF=9;
        """
        result = grpc_mapping_call(self.stub, "GetStateData")
        return result

    def get_violation_data(self):
        result = grpc_mapping_call(self.stub, "GetViolationData")
        return result

    def set_program_state(self, program_state):
        result = grpc_mapping_call(self.stub, "SetProgramState", dict(program_state=program_state))
        return result

    ############################
    # Utility
    ############################
    
    def calculate_joint_pos(self, tpos, init_jpos):
        result = grpc_mapping_call(self.stub, "InverseKinematics", dict(tpos=list(tpos),
                                                                                  init_jpos=list(init_jpos)))
        return result

    def calculate_relative_pos(self, start_pos, end_pos,
                               base_type=IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE):
        result = grpc_mapping_call(self.stub, "CalculateRelativePose", dict(start_pos=list(start_pos),
                                                                                      end_pos=list(end_pos),
                                                                                      base_type=base_type))
        return result

    def calculate_current_pos_rel(self, current_pos, relative_pos,
                                  base_type=IndyClient3CommonMsgs.TASK_BASE_TYPE_ABSOLUTE):
        result = grpc_mapping_call(self.stub, "CalculateCurrentPoseRel",
                                   dict(current_pos=list(current_pos),
                                        relative_pos=list(relative_pos),
                                        base_type=base_type))
        return result

    def check_approach_retract_valid(self, tpos, init_jpos, pre_tpos, post_tpos):
        for i in range(0, len(pre_tpos)):
            pre_tpos[i] = pre_tpos[i] * -1
        result = grpc_mapping_call(self.stub, "CheckAproachRetractValid", dict(tpos=list(tpos),
                                                                                         init_jpos=list(init_jpos),
                                                                                         pre_tpos=list(pre_tpos),
                                                                                         post_tpos=list(post_tpos)))
        return result

    def get_pallet_point_list(self, tpos, jpos, pre_tpos, post_tpos, pallet_pattern, width, height):
        result = grpc_mapping_call(self.stub, "GetPalletPointList", dict(tpos=list(tpos),
                                                                                   jpos=list(jpos),
                                                                                   pre_tpos=list(pre_tpos),
                                                                                   post_tpos=list(post_tpos),
                                                                                   pallet_pattern=pallet_pattern,
                                                                                   width=width,
                                                                                   height=height))
        return result

    ############################
    # DIO
    ############################
    
    def set_do(self, do_list: list):
        """
        do_list = [(int_addr1, True/False), (int_addr1, True/False), ...]
        """
        do_dict = dict(do_list=[{'addr': item[0], 'on': item[1]} for item in do_list])
        result = grpc_mapping_call(self.stub, "SetDO", do_dict)
        return result

    def set_ao(self, ao_list: list):
        """
        ao_list = [(int_addr1, int), (int_addr1, int), ...]
        """
        ao_dict = dict(ao_list=[{'addr': item[0], 'voltage': item[1]} for item in ao_list])
        result = grpc_mapping_call(self.stub, "SetAO", ao_dict)
        return result

    def set_end_do(self, end_do_list: list):
        """
        end_do_list = [(int_addr1, True/False), (int_addr1, True/False), ...]
        // addr 0 --> 1	//NPN Gripper On
        // addr 1 --> 2	//PNP Gripper On
        // addr 2 --> 4	//None
        // addr 3 --> 8	//EModi Gripper On
        """
        do_dict = dict(do_list=[{'addr': item[0], 'on': item[1]} for item in end_do_list])
        result = grpc_mapping_call(self.stub, "SetEndDO", do_dict)
        return result

    def set_end_ao(self, ao_list: list):
        """
        ao_list = [(int_addr1, True/False), (int_addr1, True/False), ...]
        """
        ao_dict = dict(ao_list=[{'addr': item[0], 'voltage': item[1]} for item in ao_list])
        result = grpc_mapping_call(self.stub, "SetEndAO", ao_dict)
        return result

    ############################
    # Config Setting
    ############################

    def set_joint_gains(self, kp, kv, kl2):
        """
        Joint Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
        """
        joint_gains_dict = dict(kp=list(kp), kv=list(kv), kl2=list(kl2))
        result = grpc_mapping_call(self.stub, "SetJointControlGain", joint_gains_dict)
        return result

    def get_joint_gains(self):
        """
        Joint Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
        """
        result = grpc_mapping_call(self.stub, "GetJointControlGain")
        return result

    def set_task_gains(self, kp, kv, kl2):
        """
        Task Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
        """
        task_gains_dict = dict(kp=list(kp), kv=list(kv), kl2=list(kl2))
        result = grpc_mapping_call(self.stub, "SetTaskControlGain", task_gains_dict)
        return result

    def get_task_gains(self):
        """
        Task Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
        """
        result = grpc_mapping_call(self.stub, "GetTaskControlGain")
        return result

    def set_impedance_gains(self, mass, damping, stiffness, kl2):
        """
        Impedance Control Gains:
            mass   -> float[6]
            damping   -> float[6]
            stiffness   -> float[6]
            kl2  -> float[6]
        """
        impedance_gains_dict = dict(mass=list(mass), damping=list(damping), stiffness=list(stiffness), kl2=list(kl2))
        result = grpc_mapping_call(self.stub, "SetImpedanceControlGain", impedance_gains_dict)
        return result

    def get_impedance_gains(self):
        """
        Impedance Control Gains:
            mass   -> float[6]
            damping   -> float[6]
            stiffness   -> float[6]
            kl2  -> float[6]
        """
        result = grpc_mapping_call(self.stub, "GetImpedanceControlGain")
        return result

    def set_mounting_angles(self, rotY, rotZ):
        """
        Mounting Angles:
            rotY                -> double
            rotZ                -> double
        """
        result = grpc_mapping_call(self.stub, "SetMountPos", dict(ry=rotY, rz=rotZ))
        return result

    def get_mounting_angles(self):
        """
        Mounting Angles:
            rotY                -> double
            rotZ                -> double
        """
        result = grpc_mapping_call(self.stub, "GetMountPos")
        return result

    def set_tool_property(self, mass, center_of_mass, inertia):
        """
        Tool Property:
            mass                -> double
            center_of_mass      -> double[3]
            inertia             -> double[6]
        """
        result = grpc_mapping_call(self.stub, "SetToolProperty", dict(mass=mass,
                                                                                center_of_mass=list(center_of_mass),
                                                                                inertia=list(inertia)))
        return result

    def get_tool_property(self):
        """
        Tool Property:
            mass                -> double
            center_of_mass      -> double[3]
            inertia             -> double[6]
        """
        result = grpc_mapping_call(self.stub, "GetToolProperty")
        return result

    def set_collision_level(self, level):
        """
        Collision Level: 1 ~ 5
        """
        result = grpc_mapping_call(self.stub, "SetCollSensLevel", dict(level=level))
        return result

    def get_collision_level(self):
        """
        Collision Level: 1 ~ 5
        """
        result = grpc_mapping_call(self.stub, "GetCollSensLevel")
        return result

    def set_collision_params(self, j_torque_bases, j_torque_tangents,
                             t_torque_bases, t_torque_tangents,
                             t_constvel_torque_bases, t_constvel_torque_tangents,
                             t_conveyor_torque_bases, t_conveyor_torque_tangents,
                             error_bases, error_tangents):
        """
        Collision Params:
            j_torque_bases                  -> double[6]
            j_torque_tangents               -> double[6]
            t_torque_bases                  -> double[6]
            t_torque_tangents               -> double[6]
            t_constvel_torque_bases         -> double[6]
            t_constvel_torque_tangents      -> double[6]
            t_conveyor_torque_bases         -> double[6]
            t_conveyor_torque_tangents      -> double[6]
            error_bases                     -> double[6]
            error_tangents                  -> double[6]
        """
        collision_params_dict = dict(j_torque_bases=list(j_torque_bases), j_torque_tangents=list(j_torque_tangents),
                                     t_torque_bases=list(t_torque_bases), t_torque_tangents=list(t_torque_tangents),
                                     t_constvel_torque_bases=list(t_constvel_torque_bases),
                                     t_constvel_torque_tangents=list(t_constvel_torque_tangents),
                                     t_conveyor_torque_bases=list(t_conveyor_torque_bases),
                                     t_conveyor_torque_tangents=list(t_conveyor_torque_tangents),
                                     error_bases=list(error_bases), error_tangents=list(error_tangents))
        result = grpc_mapping_call(self.stub, "SetCollSensParam", collision_params_dict)
        return result

    def get_collision_params(self):
        """
        Collision Params:
            j_torque_bases                  -> double[6]
            j_torque_tangents               -> double[6]
            t_torque_bases                  -> double[6]
            t_torque_tangents               -> double[6]
            t_constvel_torque_bases         -> double[6]
            t_constvel_torque_tangents      -> double[6]
            t_conveyor_torque_bases         -> double[6]
            t_conveyor_torque_tangents      -> double[6]
            error_bases                     -> double[6]
            error_tangents                  -> double[6]
        """
        result = grpc_mapping_call(self.stub, "GetCollSensParam")
        for i in range(0, 6):
            result.j_torque_bases[i] = round(result.j_torque_bases[i], 4)
            result.j_torque_tangents[i] = round(result.j_torque_tangents[i], 4)
            result.t_torque_bases[i] = round(result.t_torque_bases[i], 4)
            result.t_torque_tangents[i] = round(result.t_torque_tangents[i], 4)
            result.t_constvel_torque_bases[i] = round(result.t_constvel_torque_bases[i], 4)
            result.t_constvel_torque_tangents[i] = round(result.t_constvel_torque_tangents[i], 4)
            result.t_conveyor_torque_bases[i] = round(result.t_conveyor_torque_bases[i], 4)
            result.t_conveyor_torque_tangents[i] = round(result.t_conveyor_torque_tangents[i], 4)
            result.error_bases[i] = round(result.error_bases[i], 4)
            result.error_tangents[i] = round(result.error_tangents[i], 4)
        return result

    def set_collision_policy(self, policy=IndyClient3CommonMsgs.COLLISION_POLICY_STOP, sleep_time=0, gravity_time=0.1):
        """
        Collision Policy:
           policy      -> NO_DETECT | PAUSE | RESUME_AFTER_SLEEP | STOP
           sleep_time  -> float (sec)
        """
        result = grpc_mapping_call(self.stub, "SetCollPolicy",
                                   dict(policy=policy, sleep_time=sleep_time, gravity_time=gravity_time))
        return result

    def get_collision_policy(self):
        """
        Collision Policy:
           policy      -> NO_DETECT | PAUSE | RESUME_AFTER_SLEEP | STOP
           sleep_time  -> float (sec)
           gravity_time  -> float (sec)
        """
        result = grpc_mapping_call(self.stub, "GetCollPolicy")
        return result

    def get_collision_tuning_params(self):
        """
        Collision Params:
            j_torque_bases                  -> double[6]
            j_torque_tangents               -> double[6]
            t_torque_bases                  -> double[6]
            t_torque_tangents               -> double[6]
            t_constvel_torque_bases         -> double[6]
            t_constvel_torque_tangents      -> double[6]
            t_conveyor_torque_bases         -> double[6]
            t_conveyor_torque_tangents      -> double[6]
            error_bases                     -> double[6]
            error_tangents                  -> double[6]
        """
        result = grpc_mapping_call(self.stub, "GetCollTuningParam")
        for i in range(0, 6):
            result.j_torque_bases[i] = round(result.j_torque_bases[i], 4)
            result.j_torque_tangents[i] = round(result.j_torque_tangents[i], 4)
            result.t_torque_bases[i] = round(result.t_torque_bases[i], 4)
            result.t_torque_tangents[i] = round(result.t_torque_tangents[i], 4)
            result.t_constvel_torque_bases[i] = round(result.t_constvel_torque_bases[i], 4)
            result.t_constvel_torque_tangents[i] = round(result.t_constvel_torque_tangents[i], 4)
            result.t_conveyor_torque_bases[i] = round(result.t_conveyor_torque_bases[i], 4)
            result.t_conveyor_torque_tangents[i] = round(result.t_conveyor_torque_tangents[i], 4)
            result.error_bases[i] = round(result.error_bases[i], 4)
            result.error_tangents[i] = round(result.error_tangents[i], 4)
        return result

    def set_friction_comp_level(self, control_comp: bool, control_comp_levels: list,
                                dt_comp: bool, dt_comp_levels: list,
                                id_joint=-1):
        friction_comp_level_dict = dict(control_comp=control_comp, control_comp_levels=list(control_comp_levels),
                                        dt_comp=dt_comp, dt_comp_levels=list(dt_comp_levels),
                                        id_joint=id_joint)
        result = grpc_mapping_call(self.stub, "SetFricComp", friction_comp_level_dict)
        return result

    def get_friction_comp_level(self):
        result = grpc_mapping_call(self.stub, "GetFricComp")
        return result

    def set_safety_limit(self, power_limit: float, power_limit_ratio: float,
                         tcp_force_limit: float, tcp_force_limit_ratio: float,
                         tcp_speed_limit: float, tcp_speed_limit_ratio: float, ):
        """
        Safety Limits:
            power_limit             -> float
            power_limit_ratio       -> float
            tcp_force_limit         -> float
            tcp_force_limit_ratio   -> float
            tcp_speed_limit         -> float
            tcp_speed_limit_ratio   -> float
            joint_limits   -> float[]
        """
        safety_limit_dict = dict(power_limit=power_limit,
                                 power_limit_ratio=power_limit_ratio,
                                 tcp_force_limit=tcp_force_limit,
                                 tcp_force_limit_ratio=tcp_force_limit_ratio,
                                 tcp_speed_limit=tcp_speed_limit,
                                 tcp_speed_limit_ratio=tcp_speed_limit_ratio,
                                 joint_limits=[179, 179, 179, 179, 179, 179])  # Joint Limits is not applied yet
        result = grpc_mapping_call(self.stub, "SetSafetyLimitConfig", safety_limit_dict)
        return result

    def get_safety_limit(self):
        """
        Safety Limits:
            power_limit             -> float
            power_limit_ratio       -> float
            tcp_force_limit         -> float
            tcp_force_limit_ratio   -> float
            tcp_speed_limit         -> float
            tcp_speed_limit_ratio   -> float
            joint_limits   -> float[]
        """
        result = grpc_mapping_call(self.stub, "GetSafetyLimitConfig")
        return result

    def set_safety_stop_cat(self, jpos_limit_stop=IndyClient3CommonMsgs.STOP_CAT_IMMEDIATE_BRAKE,
                            jvel_limit_stop=IndyClient3CommonMsgs.STOP_CAT_IMMEDIATE_BRAKE,
                            jtau_limit_stop=IndyClient3CommonMsgs.STOP_CAT_IMMEDIATE_BRAKE,
                            tvel_limit_stop=IndyClient3CommonMsgs.STOP_CAT_IMMEDIATE_BRAKE,
                            tforce_limit_stop=IndyClient3CommonMsgs.STOP_CAT_IMMEDIATE_BRAKE,
                            power_limit_stop=IndyClient3CommonMsgs.STOP_CAT_IMMEDIATE_BRAKE):
        """
        Safety Stop Category:
            jpos_limit_stop = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            jvel_limit_stop = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            jtau_limit_stop = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            tvel_limit_stop = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            tforce_limit_stop = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            power_limit_stop = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
        """
        safety_stop_dict = dict(joint_position_limit_stop_cat=jpos_limit_stop,
                                joint_speed_limit_stop_cat=jvel_limit_stop,
                                joint_torque_limit_stop_cat=jtau_limit_stop,
                                tcp_speed_limit_stop_cat=tvel_limit_stop,
                                tcp_force_limit_stop_cat=tforce_limit_stop,
                                power_limit_stop_cat=power_limit_stop)
        result = grpc_mapping_call(self.stub, "SetSafetyStopConfig", safety_stop_dict)
        return result

    def get_safety_stop_cat(self):
        """
        Safety Stop Category:
            joint_position_limit_stop_cat = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            joint_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            joint_torque_limit_stop_cat = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            tcp_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            tcp_force_limit_stop_cat = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
            power_limit_stop_cat = IMMEDIATE_BRAKE(0) | REDUCE_SPEED_AND_BRAKE(1) | REDUCE_SPEED(2)
        """
        result = grpc_mapping_call(self.stub, "GetSafetyStopConfig")
        return result

    @staticmethod
    def _rotation_matrix(rpy):
        roll_rad = rpy[0] * math.pi / 180
        pitch_rad = rpy[1] * math.pi / 180
        yaw_rad = rpy[2] * math.pi / 180
        a = math.cos(pitch_rad) * math.cos(yaw_rad)
        b = math.sin(roll_rad) * math.sin(pitch_rad) * math.cos(yaw_rad) - math.cos(roll_rad) * math.sin(yaw_rad)
        c = math.cos(roll_rad) * math.sin(pitch_rad) * math.cos(yaw_rad) + math.sin(roll_rad) * math.sin(yaw_rad)
        e = math.cos(pitch_rad) * math.sin(yaw_rad)
        f = math.sin(roll_rad) * math.sin(pitch_rad) * math.sin(yaw_rad) + math.cos(roll_rad) * math.cos(yaw_rad)
        g = math.cos(roll_rad) * math.sin(pitch_rad) * math.sin(yaw_rad) - math.sin(roll_rad) * math.cos(yaw_rad)
        i = -math.sin(pitch_rad)
        j = math.sin(roll_rad) * math.cos(pitch_rad)
        k = math.cos(roll_rad) * math.cos(pitch_rad)
        return [[a, b, c], [e, f, g], [i, j, k]]

    def _task_relative_rotation(self, tpos, current):
        _rot_c = self._rotation_matrix(current[3:])
        _rot_t = self._rotation_matrix(tpos[3:])
        e = [_rot_t[0][0] * _rot_c[0][0] + _rot_t[0][1] * _rot_c[1][0] + _rot_t[0][2] * _rot_c[2][0],
             _rot_t[1][0] * _rot_c[0][0] + _rot_t[1][1] * _rot_c[1][0] + _rot_t[1][2] * _rot_c[2][0],
             _rot_t[2][0] * _rot_c[0][0] + _rot_t[2][1] * _rot_c[1][0] + _rot_t[2][2] * _rot_c[2][0],
             _rot_t[2][0] * _rot_c[0][1] + _rot_t[2][1] * _rot_c[1][1] + _rot_t[2][2] * _rot_c[2][1],
             _rot_t[2][0] * _rot_c[0][2] + _rot_t[2][1] * _rot_c[1][2] + _rot_t[2][2] * _rot_c[2][2]]
        x = current[0] + tpos[0]
        y = current[1] + tpos[1]
        z = current[2] + tpos[2]
        u = math.atan2(e[3], e[4]) * 180 / math.pi
        v = -math.asin(e[2]) * 180 / math.pi
        w = math.atan2(e[1], e[0]) * 180 / math.pi
        return [x, y, z, u, v, w]
    