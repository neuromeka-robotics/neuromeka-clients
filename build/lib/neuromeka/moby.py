import sys
import json
import grpc
import math

from .proto.MobygRPCServer_pb2_grpc import GRPCMobyTaskStub
from .proto.MobygRPCServer_pb2 import *

class MobyClient:
    def __init__(self, step_ip='192.168.214.20'):
        # initialize RPC
        
        self.channel = grpc.insecure_channel('%s:50051'%step_ip)
        self.stub = GRPCMobyTaskStub(self.channel)

    ###### Moby gRPC protocol
    ## Moby EtherCAT data
    def get_motor_dirver_tx(self, slave_num):
        ## Get the Moby's motor driver TxPDO data
        # Rotation motor: 0, 2, 4, 6 (fr, br, bl, fl)
        # Driving motor: 1, 3, 5, 7 (fr, br, bl, fl)
        # Indy motor: 8, 9, 10, 11, 12, 13 (14 for 7-DOF manipulator)
        tx = self.stub.GetMobyTxData(IntVal(val=slave_num))
        return {'statusWord': tx.statusWord, 'modeOpDisp': tx.modeOpDisp, 'actualPosition': tx.actualPosition,
                'actualVelocity': tx.actualVelocity, 'actualTorque': tx.actualTorque}

    def get_motor_dirver_rx(self, slave_num):
        # Get the Moby's motor driver RxPDO data
        # Rotation motor index: 0, 2, 4, 6 (fr, br, bl, fl)
        # Driving motor index: 1, 3, 5, 7 (fr, br, bl, fl)
        # Indy motor index: 8, 9, 10, 11, 12, 13 (14 for 7-DOF manipulator)
        rx = self.stub.GetMobyRxData(IntVal(val=slave_num))
        return {'controlWord': rx.controlWord, 'modeOp': rx.modeOp, 'targetPosition': rx.targetPosition,
                'targetVelocity': rx.targetVelocity, 'targetTorque': rx.targetTorque}

    def get_moby_txdata(self, slave_num):
        return self.stub.GetMobyTxData(IntVal(val=slave_num))

    def get_moby_rxdata(self, slave_num):
        return self.stub.GetMobyRxData(IntVal(val=slave_num))

    ## Moby State
    def get_moby_state(self):
        state = self.stub.GetMobyState(Empty())
        return {'is_ready': state.isReady, 'is_moving': state.isMoving, 'is_move_finished': state.isMoveFinished,
                'is_emg_pushed': state.isEmgPushed, 'is_error_state': state.isErrorState,
                'is_home_pose': state.isHomePose, 'is_resetting': state.isResetting,
                'is_imu_avail': state.isIMUAvailable, 'is_program_running': state.isProgramRunning,
                'is_program_pause': state.isProgramPause, 'is_rotation_zero': state.isRotationZero}

    def get_moby_error_state(self):
        error = self.stub.GetMobyErrorState(Empty())
        error_code = error.errorState
        error_dict = {
            0: "NONE",
            1: "HW_EMG_PUSH",
            2: "HW_CONNECTION_LOST",
            3: "ECAT_MASTER_NOT_OP",
            4: "ECAT_SLAVE_NOT_OP",
            5: "ECAT_MOTOR_NOT_SERVO_ON",
            6: "ECAT_MOTOR_ERROR",
            7: "ECAT_SYSTEM_NOT_READY",
            8: "ECAT_ENCODER1_ERROR",
            9: "ECAT_ENCODER2_ERROR",
            10: "ECAT_ENCODER3_ERROR",
            11: "ECAT_ENCODER4_ERROR",
            12: "SW_POSITION_LIMIT",
            13: "SW_TORQUE_LIMIT",
        }
        error_index1 = error.errorIndex1
        error_index2 = error.errorIndex2
        error_index3 = error.errorIndex3

        return error_dict.get(error_code, "Normal (UNKNOWN ERROR CODE)"), error_index1, error_index2, error_index3

    def recover(self):
        return self.stub.Recover()

    ## Get Moby's odometry and physical data
    def get_moby_pose(self):
        pose = self.stub.GetMobyPose(Empty())
        return [pose.px, pose.py, pose.pw]

    def get_moby_vel(self):
        vel = self.stub.GetMobyVel(Empty())
        return [vel.vx, vel.vy, vel.vw]

    def reset_moby_pose(self):
        return self.stub.ResetMobyPose(Empty())

    def get_target_vel(self):
        target = self.stub.GetTargetVel(Empty())
        return [target.vx, target.vy, target.vw]

    ## Swerve drive
    def get_rotation_angle(self):
        val = self.stub.GetRotationAngleDeg(Empty())
        return {'fl': val.fl, 'fr': val.fr, 'bl': val.bl, 'br': val.br}

    def get_drive_speed(self):
        val = self.stub.GetDriveSpeed(Empty())
        return {'fl': val.fl, 'fr': val.fr, 'bl': val.bl, 'br': val.br}

    def get_zero(self):
        val = self.stub.GetRotationZeroCount(Empty())
        return {'fl': val.fl, 'fr': val.fr, 'bl': val.bl, 'br': val.br}

    ## Differential drive
    def get_cmode(self):
        return self.stub.GetCMode(Empty()).val

        ## Get sensor data

    def get_gyro_data(self):
        return self.stub.GetGyroData(Empty()).val

    def reset_gyro(self):
        return self.stub.ResetGyroSensor(Empty())

    def get_imu_data(self):
        data = self.stub.GetGyroFullData(Empty())
        angle = [data.angleX, data.angleY, data.angleZ]
        vel = [data.angleVelX, data.angleVelY, data.angleVelZ]
        acc = [data.linAccX, data.linAccY, data.linAccZ]
        return angle, vel, acc

    def use_gyro_for_odom(self, use_gyro):
        return self.stub.UseGyroForOdom(BoolVal(val=use_gyro))

    def get_ir_data(self):
        value = self.stub.GetIRSensorData(Empty())
        return {'ir_front1': value.ir_front1, 'ir_front2': value.ir_front2, 'ir_left1': value.ir_left1,
                'ir_left2': value.ir_left2, 'ir_left3': value.ir_left3, 'ir_rear': value.ir_rear,
                'ir_right1': value.ir_right1, 'ir_right2': value.ir_right2, 'ir_right3': value.ir_right3}

    def get_us_data(self):
        value = self.stub.GetUSSensorData(Empty())
        return {'front_left1': value.us_front_left1, 'front_left2': value.us_front_left2,
                'front_left3': value.us_front_left3, 'front_ground': value.us_front_ground,
                'front_right1': value.us_front_right1, 'front_right2': value.us_front_right2,
                'front_right3': value.us_front_right3, 'front_right4': value.us_front_right4,
                'back_right1': value.us_back_right1, 'back_right2': value.us_back_right2,
                'back_right3': value.us_back_right3, 'back_ground': value.us_back_ground,
                'back_left1': value.us_back_left1, 'back_left2': value.us_back_left2, 'back_left3': value.us_back_left3,
                'back_left4': value.us_back_left4}

    ## BMS data
    def get_bms(self):
        value = self.stub.GetBMSData(Empty())
        return {'BMS status-1': value.bms_status[0] / 10, 'BMS status-2': value.bms_status[1] / 10,
                'Pack voltage-1': value.pack_volt[0] / 100, 'Pack voltage-2': value.pack_volt[1] / 100,
                'Battery Voltage-1': value.battery_volt[0] / 100, 'Battery Voltage-2': value.battery_volt[1] / 100,
                'Pack current1-1': value.pack_current1[0] / 100, 'Pack current1-2': value.pack_current1[1] / 100,
                'Pack current2-1': value.pack_current2[0] / 100, 'Pack current2-2': value.pack_current2[1] / 100,
                'Is Charge': value.isCharge, 'Is Cell OverVoltage': value.isCellOverVolt,
                'Is Cell UnderVoltage': value.isCellUnderVolt, 'Is OverCurrent Charge': value.isOverCurCharge,
                'Is OverCurrent Discharge': value.isOverCurDischrg, 'Is Short Circuit': value.isShortCircuit,
                'Is OverTemperature': value.isOverTemperature, 'Is Pack OverVoltage': value.isPackOverVolt,
                'SOC': value.SOC * 0.1, 'SOH': value.SOH, 'Time for Charge': value.time_charge,
                'time for Discharge': value.time_dcharge, 'Remain Capacity Ah': value.rem_capAh / 100,
                'Remain Capacity Wh': value.rem_capWh, 'Temperature-1': value.bms_temperature[0] * 0.1,
                'Temperature-2': value.bms_temperature[1] * 0.1, 'Temperature-3': value.bms_temperature[2] * 0.1,
                'Temperature-4': value.bms_temperature[3] * 0.1, 'Cell Voltage-1': value.cell_volt[0] * 0.001,
                'Cell Voltage-2': value.cell_volt[1] * 0.001, 'Cell Voltage-3': value.cell_volt[2] * 0.001,
                'Cell Voltage-4': value.cell_volt[3] * 0.001, 'Cell Voltage-5': value.cell_volt[4] * 0.001,
                'Cell Voltage-6': value.cell_volt[5] * 0.001, 'Cell Voltage-7': value.cell_volt[6] * 0.001,
                'Cell Voltage-8': value.cell_volt[7] * 0.001, 'Cell Voltage-9': value.cell_volt[8] * 0.001,
                'Cell Voltage-10': value.cell_volt[9] * 0.001, 'Cell Voltage-11': value.cell_volt[10] * 0.001,
                'Cell Voltage-12': value.cell_volt[11] * 0.001, 'Cell Voltage-13': value.cell_volt[12] * 0.001}

    ## Moby motion command
    def stop_motion(self):
        return self.stub.StopMotion(Empty())

    def set_target_velocity(self, vx, vy, vw):
        return self.stub.SetStepControl(TargetVel(vx=vx, vy=vy, vw=vw))

    def go_straight(self):
        return self.stub.SetRotationAngleDeg(SwerveDoubles(fl=0, fr=0, bl=0, br=0))

    def move_rotation_deg(self, fr, br, bl, fl):
        return self.stub.SetRotationAngleDeg(SwerveDoubles(fr=fr, br=br, bl=bl, fl=fl))

    def move_driving_mps(self, fr, br, bl, fl):
        return self.stub.DriveWheel(SwerveDoubles(fr=fr, br=br, bl=bl, fl=fl))

    ## Set Moby parameters
    def set_zero_as_current(self):
        return self.stub.SetZeroPosAsCurrentPos(Empty())

    def set_rotation_interpolator_vel_acc(self, vel, acc):
        return self.stub.SetRotationVelAcc(DoubleVals(val=[vel, acc]))

    def set_rotation_interpolator(self, traj_type):
        return self.stub.SetRotationInterpolator(IntVal(val=traj_type))

    def set_drive_interpolator_acc_dec(self, acc, dec):
        return self.stub.SetDriveAccDec(DoubleVals(val=[acc, dec]))

    def set_drive_interpolator_onoff(self, onoff):
        return self.stub.SetDriveInterpolatorOnOff(BoolVal(val=onoff))

    def set_rotation_gain_fl(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=0, k=k, kv=kv, kp=kp))

    def set_rotation_gain_fr(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=1, k=k, kv=kv, kp=kp))

    def set_rotation_gain_bl(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=2, k=k, kv=kv, kp=kp))

    def set_rotation_gain_br(self, k, kv, kp):
        return self.stub.SetControlParam(RotationGain(idx=3, k=k, kv=kv, kp=kp))

    def set_torque_mode(self, isOn):
        return self.stub.SetRotationTorqueMode(BoolVal(val=isOn))

        ## Set Moby-Agri parameters

    def get_robot_zero_count(self):
        data = self.stub.GetRobotZeroCount(Empty())
        return [data.rotation, data.elevation]

    def set_robot_zero_as_current(self):
        return self.stub.SetRobotZeroAsCurrent(Empty())

    def turn_led(self, on):
        return self.stub.TurnLEDOnOff(BoolVal(val=on))

    ## Data logger
    def start_logging(self):
        return self.stub.StartRTLogging(Empty())

    def end_logging(self):
        return self.stub.EndRTLogging(Empty())

    def set_logger(self):
        return self.stub.SetLoggerBuffer(IntVal())

    def save_logger(self):
        return self.stub.RTLoggerSave(Empty())






