//
// Created by heoyoungjin on 2023-04-16.
//

#include "MobyClient.h"
#include <iostream>
#include <memory>

MobyClient::MobyClient(std::shared_ptr<Channel> channel)
: stub_(GRPCMobyTask::NewStub(channel))
{

}

MotorDriverTx MobyClient::GetMobyTxData(int32_t val)
{
    IntVal request;
    request.set_val(val);

    ClientContext context;
    MotorDriverTx response;

    Status status = stub_->GetMobyTxData(&context, request, &response);

    if (!status.ok()) {
        std::cout << "GetMobyTxData rpc failed." << std::endl;
    } else{

        return response;
    }
}

void MobyClient::GetMobyRxData(int32_t val,
                                        int32_t &tarPos, int32_t &tarVel, int32_t &tarTor, int &modeOp, int &ctrlWord) {
    IntVal request;
    request.set_val(val);

    ClientContext context;
    MotorDriverRx response;

    Status status = stub_->GetMobyRxData(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetMobyRxData rpc failed." << std::endl;
    }
    else{
        tarPos = response.targetposition();
        tarVel = response.targetvelocity();
        tarTor = response.targettorque();
        modeOp = response.modeop();
        ctrlWord = response.controlword();

        return;
    }
}

void MobyClient::GetMobyState(std::map<std::string, bool> &state)
{
    Empty request;

    ClientContext context;
    MobyState response;

    Status status = stub_->GetMobyState(&context, request, &response);



    // Print an error message if the RPC call failed
    if (!status.ok()) {
        std::cerr << "GetMobyState RPC failed: " << status.error_message() << std::endl;
    } else{
        state["is_ready"] = response.isready();
        state["is_moving"] = response.ismoving();
        state["is_move_finished"] = response.ismovefinished();
        state["is_emg_pushed"] = response.isemgpushed();
        state["is_error_state"] = response.iserrorstate();
        state["is_home_pose"] = response.ishomepose();
        state["is_resetting"] = response.isresetting();
        state["is_imu_avail"] = response.isimuavailable();
        state["is_program_running"] = response.isprogramrunning();
        state["is_program_pause"] = response.isprogrampause();
        state["is_rotation_zero"] = response.isrotationzero();

        return;
    }
}

std::string MobyClient::GetMobyErrorState() {
    Empty request;

    ClientContext context;
    MobyErrorState response;

    Status status = stub_->GetMobyErrorState(&context, request, &response);

    // Map the error state code to a human-readable error message
    std::map<int, std::string> error_dict = {
            {0, "NONE"},
            {1, "HW_EMG_PUSH"},
            {2, "HW_CONNECTION_LOST"},
            {3, "ECAT_MASTER_NOT_OP"},
            {4, "ECAT_SLAVE_NOT_OP"},
            {5, "ECAT_MOTOR_NOT_SERVO_ON"},
            {6, "ECAT_MOTOR_ERROR"},
            {7, "ECAT_SYSTEM_NOT_READY"},
            {8, "ECAT_ENCODER1_ERROR"},
            {9, "ECAT_ENCODER2_ERROR"},
            {10, "ECAT_ENCODER3_ERROR"},
            {11, "ECAT_ENCODER4_ERROR"},
            {12, "SW_POSITION_LIMIT"},
            {13, "SW_TORQUE_LIMIT"}
    };
    int error_code = response.errorstate();
    std::string error_message = error_dict[error_code];

    // Print an error message if the RPC call failed
    if (!status.ok()) {
        std::cerr << "GetMobyErrorState RPC failed: " << status.error_message() << std::endl;
    }

    return error_message;
}

void MobyClient::Recover() {
    Empty request;

    ClientContext context;
    Empty response;

    Status status = stub_->Recover(&context, request, &response);
    if (!status.ok()) {
        std::cout << "Recover rpc failed." << std::endl;
    }
    else{
        return;
    }
}

void MobyClient::GetMobyPose(double &px, double &py, double &ptheta) {
    Empty request;

    ClientContext context;
    MobyPose response;

    Status status = stub_->GetMobyPose(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetMobyPose rpc failed." << std::endl;
    } else{
        px = response.px();
        py = response.py();
        ptheta = response.pw();
        return;
    }

}

void MobyClient::GetMobyVel(double &vx, double &vy, double &vtheta) {
    Empty request;

    ClientContext context;
    MobyVel response;
    Status status = stub_->GetMobyVel(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetMobyVel rpc failed." << std::endl;
    }
    else
    {
        vx = response.vx();
        vy = response.vy();
        vtheta = response.vw();
        return;
    }
}

void MobyClient::ResetMobyPose() {
    Empty request;

    ClientContext context;
    Empty response;

    Status status = stub_->ResetMobyPose(&context, request, &response);
    if (!status.ok()) {
        std::cout << "ResetMobyPose rpc failed." << std::endl;
    }
    else{
        return;
    }
}

void MobyClient::GetRotationAngleDeg(double &fl, double &fr, double &bl, double &br) {
    Empty request;

    ClientContext context;
    SwerveDoubles response;

    Status status = stub_->GetRotationAngleDeg(&context, request, &response);

    if (!status.ok()) {
        std::cout << "GetRotationAngleDeg rpc failed." << std::endl;
    }
    else{
        fl = response.fl();
        fr = response.fr();
        bl = response.bl();
        br = response.br();
        return;
    }
}

void MobyClient::GetDriveSpeed(double &fl, double &fr, double &bl, double &br) {
    Empty request;
    ClientContext context;
    SwerveDoubles response;

    Status status = stub_->GetDriveSpeed(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetDriveSpeed rpc failed." << std::endl;
    }
    else{
        fl = response.fl();
        fr = response.fr();
        bl = response.bl();
        br = response.br();
        return;
    }

}

void MobyClient::GetTargetVel(double &vx, double &vy, double &vtheta) {
    Empty request;
    ClientContext context;
    TargetVel response;

    Status status = stub_->GetTargetVel(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetTargetVel rpc failed." << std::endl;
    }
    else
    {
        vx = response.vx();
        vy = response.vy();
        vtheta = response.vw();
        return;
    }
}

void MobyClient::GetRotationZeroCount(int64_t &fl, int64_t &fr, int64_t &bl, int64_t &br) {
    Empty request;
    ClientContext context;
    ZeroCount response;

    Status status = stub_->GetRotationZeroCount(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetRotationZeroCount rpc failed." << std::endl;
    }
    else{
        fl = response.fl();
        fr = response.fr();
        bl = response.bl();
        br = response.br();
        return;
    }

}

void MobyClient::GetGyroData(double &heading, double &heading_rate)
{
    Empty request;
    ClientContext context;
    DoubleVals response;

    Status status = stub_->GetGyroData(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetGyroData rpc failed." << std::endl;
    }
    else{
        heading = response.val(0);
        heading_rate = response.val(1);
        return;
    }
}

void MobyClient::ResetGyroSensor() {
    Empty request;
    ClientContext context;
    Empty response;

    Status status = stub_->ResetGyroSensor(&context, request, &response);
    if (!status.ok()) {
        std::cout << "ResetGyroSensor rpc failed." << std::endl;
    }
    else{
        return;
    }

}

void MobyClient::UseGyroForOdom(bool val) {
    BoolVal request;
    request.set_val(val);
    ClientContext context;
    Empty response;

    Status status = stub_->UseGyroForOdom(&context, request, &response);
    if (!status.ok()) {
        std::cout << "UseGyroForOdom rpc failed." << std::endl;
    }
    else{
        return;
    }

}

IMUData MobyClient::GetGyroFullData() {
    Empty request;
    ClientContext context;
    IMUData response;

    Status status = stub_->GetGyroFullData(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetGyroFullData rpc failed." << std::endl;
    }
    return response;
}

IRData MobyClient::GetIRSensorData()
{
    Empty request;
    ClientContext context;
    IRData response;

    Status status = stub_->GetIRSensorData(&context, request, &response);
    if (!status.ok()) {
        std::cout << "GetIRSensorData rpc failed." << std::endl;
    }
    return response;

}

std::map<std::string, double> MobyClient::GetBMSData() {
    Empty request;
    ClientContext context;
    BMSData response;

    Status status = stub_->GetBMSData(&context, request, &response);



    if (!status.ok()) {
        std::cout << "GetBMSData rpc failed." << std::endl;
    }
    else{

        return {
                {"BMS status-1", response.bms_status(0)/10.0},
                {"BMS status-2", response.bms_status(1)/10.0},
                {"Pack voltage-1", response.pack_volt(0)/100.0},
                {"Pack voltage-2", response.pack_volt(1)/100.0},
                {"Battery Voltage-1", response.battery_volt(0)/100.0},
                {"Battery Voltage-2", response.battery_volt(1)/100.0},
                {"Pack current1-1", response.pack_current1(0)/10.0},
                {"Pack current1-2", response.pack_current1(1)/10.0},
                {"Pack current2-1", response.pack_current2(0)/10.0},
                {"Pack current2-2", response.pack_current2(1)/10.0}
        };
    }
}

void MobyClient::SetTargetVelocity(double vx, double vy, double vtheta) {
    ClientContext context;
    TargetVel target_vel;
    target_vel.set_vx(vx);
    target_vel.set_vy(vy);
    target_vel.set_vw(vtheta);

    Empty response;
    Status status = stub_->SetStepControl(&context, target_vel, &response);
    if (!status.ok()) {
        std::cout << "SetStepControl rpc failed." << std::endl;
    }
    else{
        return;
    }

}

void MobyClient::StopMotion() {
    Empty request;
    ClientContext context;
    Empty response;

    Status status = stub_->StopMotion(&context, request, &response);
    if (!status.ok()) {
        std::cout << "StopMotion rpc failed." << std::endl;
    }
    else{
        return;
    }

}

void MobyClient::SetRotationAngleDeg(double fl, double fr, double bl, double br) {
    ClientContext context;
    SwerveDoubles rotation_angles;
    Empty response;

    rotation_angles.set_fr(fr);
    rotation_angles.set_fl(fl);
    rotation_angles.set_br(br);
    rotation_angles.set_bl(bl);

    Status status = stub_->SetRotationAngleDeg(&context, rotation_angles, &response);
    if (!status.ok()) {
        std::cout << "SetRotationAngleDeg rpc failed." << std::endl;
    }
    else{
        return;
    }

}

void MobyClient::DriveWheel(double fl, double fr, double bl, double br) {
    ClientContext context;
    SwerveDoubles drive_speeds;
    Empty response;

    drive_speeds.set_fr(fr);
    drive_speeds.set_fl(fl);
    drive_speeds.set_br(br);
    drive_speeds.set_bl(bl);

    Status status = stub_->DriveWheel(&context, drive_speeds, &response);
    if (!status.ok()) {
        std::cout << "DriveWheel rpc failed." << std::endl;
    }
    else{
        return;
    }

}

void MobyClient::SetZeroPosAsCurrentPos() {
    Empty request;
    ClientContext context;
    Empty response;

    Status status = stub_->SetZeroPosAsCurrentPos(&context, request, &response);
    if (!status.ok()) {
        std::cout << "SetZeroPosAsCurrentPos rpc failed." << std::endl;
    }
    else{
        return;
    }

}

Empty MobyClient::SetRotationVelAcc(DoubleVals rotation_vel_acc) {
    ClientContext context;
    Empty response;
    Status status = stub_->SetRotationVelAcc(&context, rotation_vel_acc, &response);
    if (!status.ok()) {
        std::cout << "SetRotationVelAcc rpc failed." << std::endl;
    }
    return response;

}

Empty MobyClient::SetDriveAccDec(DoubleVals drive_acc_dec)
{
    ClientContext context;
    Empty response;
    Status status = stub_->SetDriveAccDec(&context, drive_acc_dec, &response);
    if (!status.ok()) {
        std::cout << "SetDriveAccDec rpc failed." << std::endl;
    }
    return response;

}

Empty MobyClient::SetDriveInterpolatorOnOff(bool val)
{
    BoolVal request;
    request.set_val(val);
    ClientContext context;
    Empty response;

    Status status = stub_->SetDriveInterpolatorOnOff(&context, request, &response);

    if (!status.ok()) {
        std::cout << "SetDriveInterpolatorOnOff rpc failed." << std::endl;
    }
    return response;

}
