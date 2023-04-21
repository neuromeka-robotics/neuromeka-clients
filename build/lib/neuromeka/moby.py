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
        return {'statusWord': tx.statusWord, 'modeOpDisp': tx.modeOpDisp, 'actualPosition':tx.actualPosition, 'actualVelocity':tx.actualVelocity, 'actualTorque':tx.actualTorque}
    
    def get_motor_dirver_rx(self, slave_num):
        # Get the Moby's motor driver RxPDO data
        # Rotation motor index: 0, 2, 4, 6 (fr, br, bl, fl)
        # Driving motor index: 1, 3, 5, 7 (fr, br, bl, fl)
        # Indy motor index: 8, 9, 10, 11, 12, 13 (14 for 7-DOF manipulator)
        rx = self.stub.GetMobyRxData(IntVal(val=slave_num))
        return {'controlWord': rx.controlWord, 'modeOp': rx.modeOp, 'targetPosition':rx.targetPosition, 'targetVelocity':rx.targetVelocity, 'targetTorque':rx.targetTorque}
    
    ## Moby State
    def get_moby_state(self):
        state = self.stub.GetMobyState(Empty())
        return {'is_ready': state.isReady, 'is_moving': state.isMoving, 'is_move_finished':state.isMoveFinished, 'is_emg_pushed':state.isEmgPushed, 'is_error_state':state.isErrorState, 'is_home_pose':state.isHomePose, 'is_resetting':state.isResetting, 'is_imu_avail':state.isIMUAvailable, 'is_program_running':state.isProgramRunning, 'is_program_pause':state.isProgramPause, 'is_rotation_zero':state.isRotationZero}
    
    def get_moby_error_state(self):
        error_code = self.stub.GetMobyErrorState(Empty()).errorState
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
        
        return error_dict.get(error_code, "Normal (UNKNOWN ERROR CODE)")

    def recover(self):
        return self.stub.Recover()
    
    ## Get Moby's odometry data
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
        return {'fl': val.fl, 'fr': val.fr, 'bl':val.bl, 'br':val.br}
    
    def get_drive_speed(self):
        val = self.stub.GetDriveSpeed(Empty())
        return {'fl': val.fl, 'fr': val.fr, 'bl':val.bl, 'br':val.br}
    
    def get_zero(self):
        val = self.stub.GetRotationZeroCount(Empty())
        return {'fl': val.fl, 'fr': val.fr, 'bl':val.bl, 'br':val.br}
    
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
        return {'ir_front1': value.ir_front1, 'ir_front2': value.ir_front2, 'ir_left1':value.ir_left1, 'ir_left2':value.ir_left2, 'ir_left3':value.ir_left3, 'ir_rear':value.ir_rear, 'ir_right1':value.ir_right1, 'ir_right2':value.ir_right2, 'ir_right3':value.ir_right3}
    
    ## BMS data
    def get_bms(self):
        value = self.stub.GetBMSData(Empty())
        return {'BMS status-1': value.bms_status[0]/10, 'BMS status-2': value.bms_status[1]/10, 'Pack voltage-1':value.pack_volt[0]/100, 'Pack voltage-2':value.pack_volt[1]/100, 'Battery Voltage-1':value.battery_volt[0]/100, 'Battery Voltage-2':value.battery_volt[1]/100, 'Pack current1-1':value.pack_current1[0]/10, 'Pack current1-2':value.pack_current1[1]/10, 'Pack current2-1':value.pack_current2[0]/10, 'Pack current2-2':value.pack_current2[1]/10}
    
    ## Moby motion command
    def stop_motion(self):
        return self.stub.StopMotion(Empty())

    def set_target_velocity(self, vx, vy, vw):
        return self.stub.SetStepControl(TargetVel(vx=vx, vy=vy, vw=vw))
    
    def move_rotation_deg(self, fr, br, bl, fl):
        return self.stub.SetRotationAngleDeg(SwerveDoubles(fr=fr, br=br, bl=bl, fl=fl))
    
    def move_driving_mps(self, fr, br, bl, fl):
        return self.stub.DriveWheel(SwerveDoubles(fr=fr, br=br, bl=bl, fl=fl))    
    
    ## Set Moby parameters
    def set_zero_as_current(self):
        return self.stub.SetZeroPosAsCurrentPos(Empty())
    
    def set_rotation_interpolator_vel_acc(self, vel, acc):
        return self.stub.SetRotationVelAcc(DoubleVals(val=[vel, acc]))
    
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
    
    ## Data logger
    def start_logging(self):
        return self.stub.StartRTLogging(Empty())
    
    def end_logging(self):
        return self.stub.EndRTLogging(Empty())
    
    def set_logger(self):
        return self.stub.SetLoggerBuffer(IntVal())
    
    def save_logger(self):
        return self.stub.RTLoggerSave(Empty())
    
    

    
    
    
