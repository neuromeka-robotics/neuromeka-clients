//
// Created by heoyoungjin on 2023-04-16.
//

#include "EcatClient.h"


int EcatClient::GetMasterStatus()
{
    GRPCECat::Empty request;
    GRPCECat::IntVal response;
    ClientContext context;

    Status status = stub_->GetMasterStatus(&context, request, &response);

    if (status.ok()) {
        return response.val();
    } else {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
        return -1;
    }
}

void EcatClient::GetSlaveStatus(int *res)
{
    GRPCECat::Empty request;
    IntVals response;
    ClientContext context;

    Status status = stub_->GetSlaveStatus(&context, request, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
    else{
        res = new int[response.val_size()];
        for (int i=0; i<response.val_size(); i++)
            res[i] = response.val(i);

        return;
    }
}

int EcatClient::GetRxDomainStatus() {
    GRPCECat::Empty request;
    GRPCECat::IntVal response;
    ClientContext context;

    Status status = stub_->GetRxDomainStatus(&context, request, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
    else{
        return response.val();
    }
}

int EcatClient::GetTxDomainStatus() {
    GRPCECat::Empty request;
    GRPCECat::IntVal response;
    ClientContext context;

    Status status = stub_->GetTxDomainStatus(&context, request, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
    else{
        return response.val();
    }
}

void EcatClient::IsSystemReady(int *res) {
    GRPCECat::Empty request;
    IntVals response;
    ClientContext context;

    Status status = stub_->IsSystemReady(&context, request, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
    else
    {
        res = new int[response.val_size()];
        for (int i=0; i<response.val_size(); i++)
            res[i] = response.val(i);

        return;
    }
}

void EcatClient::SetServoOnOff(const ServoIndex& servo_index) {
    GRPCECat::Empty response;
    ClientContext context;

    Status status = stub_->SetServoOnOff(&context, servo_index, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
}

void EcatClient::SetRxPDOMotorDriver(const MotorDriverRxIndex& motor_driver_rx_index) {
    GRPCECat::Empty response;
    ClientContext context;

    Status status = stub_->SetRxPDOMotorDriver(&context, motor_driver_rx_index, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
}

GRPCECat::MotorDriverRx EcatClient::GetRxPDOMotorDriver(int index) {
    GRPCECat::IntVal request;
    GRPCECat::MotorDriverRx response;
    ClientContext context;
    request.set_val(index);

    Status status = stub_->GetRxPDOMotorDriver(&context, request, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
    return response;
}

GRPCECat::MotorDriverTx EcatClient::GetTxPDOMotorDriver(int index) {
    GRPCECat::IntVal request;
    GRPCECat::MotorDriverTx response;
    ClientContext context;
    request.set_val(index);


    Status status = stub_->GetTxPDOMotorDriver(&context, request, &response);

    if (!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
    }
    return response;
}




