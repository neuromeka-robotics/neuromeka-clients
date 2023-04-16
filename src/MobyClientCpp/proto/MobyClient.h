//
// Created by heoyoungjin on 2023-04-16.
//

#ifndef MOBYFRAMEWORK0_2_MOBYCLIENT_H
#define MOBYFRAMEWORK0_2_MOBYCLIENT_H



#include "MobygRPCServer.grpc.pb.h"

#include <grpcpp/grpcpp.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using GRPCMoby::GRPCMobyTask;
using GRPCMoby::Empty;
using GRPCMoby::IntVal;
using GRPCMoby::BoolVal;
using GRPCMoby::DoubleVals;
using GRPCMoby::MotorDriverTx;
using GRPCMoby::MotorDriverRx;
using GRPCMoby::MobyState;
using GRPCMoby::MobyErrorState;
using GRPCMoby::MobyPose;
using GRPCMoby::MobyVel;
using GRPCMoby::SwerveDoubles;
using GRPCMoby::TargetVel;
using GRPCMoby::ZeroCount;
using GRPCMoby::IMUData;
using GRPCMoby::IRData;
using GRPCMoby::BMSData;
using GRPCMoby::RotationGain;

class MobyClient{
public:
    MobyClient(std::shared_ptr<grpc::Channel> channel);

    // RPC calls
    MotorDriverTx GetMobyTxData(int32_t val);
    void GetMobyRxData(int32_t val, int32_t &tarPos, int32_t &tarVel, int32_t &tarTor, int &modeOp, int &ctrlWord);
    void GetMobyState(std::map<std::string, bool> &state);
    std::string GetMobyErrorState();
    void Recover();
    void GetMobyPose(double &px, double &py, double &ptheta);
    void GetMobyVel(double &vx, double &vy, double &vtheta);
    void ResetMobyPose();
    void GetRotationAngleDeg(double &fl, double &fr, double &bl, double &br);
    void GetDriveSpeed(double &fl, double &fr, double &bl, double &br);
    void GetTargetVel(double &vx, double &vy, double &vtheta);
    void GetRotationZeroCount(int64_t &fl, int64_t &fr, int64_t &bl, int64_t &br);
    void GetGyroData(double &heading, double &heading_rate);
    void ResetGyroSensor();
    void UseGyroForOdom(bool val);
    IMUData GetGyroFullData();
    IRData GetIRSensorData();
    std::map<std::string, double> GetBMSData();
    void SetTargetVelocity(double vx, double vy, double vtheta);
    void StopMotion();
    void SetRotationAngleDeg(double fl, double fr, double bl, double br);
    void DriveWheel(double fl, double fr, double bl, double br);
    void SetZeroPosAsCurrentPos();
    Empty SetRotationVelAcc(DoubleVals rotation_vel_acc);
    Empty SetDriveAccDec(DoubleVals drive_acc_dec);
    Empty SetDriveInterpolatorOnOff(bool val);

private:
    std::unique_ptr<GRPCMobyTask::Stub> stub_;
};

#endif //MOBYFRAMEWORK0_2_MOBYCLIENT_H
