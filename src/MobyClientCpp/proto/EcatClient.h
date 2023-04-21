//
// Created by heoyoungjin on 2023-04-16.
//

#ifndef MOBYFRAMEWORK0_2_ECATCLIENT_H
#define MOBYFRAMEWORK0_2_ECATCLIENT_H


#include <grpcpp/grpcpp.h>
#include "EtherCATCommgRPCServer.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using GRPCECat::GRPCECatTask;
//using GRPCECat::Empty;
//using GRPCECat::IntVal;
using GRPCECat::IntVals;
using GRPCECat::ServoIndex;
//using GRPCECat::MotorDriverRx;
using GRPCECat::MotorDriverRxIndex;
//using GRPCECat::MotorDriverTx;
using GRPCECat::ServoParam;
using GRPCECat::IOBoardRx;
using GRPCECat::IOBoardTx;
using GRPCECat::EndtoolRx;
using GRPCECat::EndtoolTx;
using GRPCECat::EndtoolRevCDTTx;
using GRPCECat::DIList;
using GRPCECat::DOList;
using GRPCECat::RobotusFTTx;
using GRPCECat::FloatVal;
using GRPCECat::StringVal;

class EcatClient {
public:
    EcatClient(std::shared_ptr<Channel> channel) : stub_(GRPCECatTask::NewStub(channel)) {}

    int GetMasterStatus();
    void GetSlaveStatus(int *res);
    int GetRxDomainStatus();
    int GetTxDomainStatus();
    void IsSystemReady(int *res);
    void SetServoOnOff(const ServoIndex& index);
    void SetRxPDOMotorDriver(const MotorDriverRxIndex& index);
    GRPCECat::MotorDriverRx GetRxPDOMotorDriver(int index);
    GRPCECat::MotorDriverTx GetTxPDOMotorDriver(int index);
//    int GetMotorDriverDIs(int index);
//    int GetErrorCode(int index);
//    int GetMaxTorque(int index);
//    void SetMaxTorque(const ServoParam& param);
//    int GetMaxMotorSpeed(int index);
//    void SetMaxMotorSpeed(const ServoParam& param);
//    void SetNRMKIOBoardOutput(const IOBoardRx& output);
//    IOBoardTx GetNRMKIOBoardInput();
//    IOBoardRx GetNRMKIOBoardOutput();
//    void SetNRMKEndtoolOutput(const EndtoolRx& output);
//    EndtoolTx GetNRMKEndtoolInput();
//    EndtoolRevCDTTx GetNRMKEndtoolRevCDTInput();
//    void SetDO(const DOList& do_list);
//    DIList GetDI(int index);
//    DOList GetDO(int index);
//    RobotusFTTx GetRobotusFTSensor(int index);
//    void ResetWelconDriver(int index);
//    int GetCOREErrorCodeSDO(int index);
//    float GetCORETemperature1SDO(int index);
//    float GetCORETemperature2SDO(int index);
//    float GetCORETemperature3SDO(int index);
//    std::string GetNRMKFWVersionSDO(int index);
//    int GetMaxTorqueSDO(int index);
//    int GetProfileVelocitySDO(int index);
//    int GetProfileAccSDO(int index);
//    int GetProfileDecSDO(int index);
//    void SetMaxTorqueSDO(const ServoParam& param);
//    void SetProfileVelocitySDO(const ServoParam& param);
//    void SetProfileAccSDO(const ServoParam& param);
//    void SetProfileDecSDO(const ServoParam& param);

private:
    std::unique_ptr<GRPCECatTask::Stub> stub_;
};


#endif //MOBYFRAMEWORK0_2_ECATCLIENT_H
