/*
 * MobyClientCpp.cpp
 *
 *  Created on: 2023. 04. 15.
 *      Author: Young Jin Heo
 */

#include <unistd.h>

#include <iostream>
#include <memory>
#include <json/json.h>


#include "proto/EcatClient.h"
#include "proto/MobyClient.h"

#define MOBY_TYPE_SWERVE 0
#define MOBY_TYPE_DD     1

int main(int argc, char** argv)
{

    // Set Moby Type:
    // 0 for Swerve dirve type
    // 1 for Differential drive type
    int moby_type = MOBY_TYPE_SWERVE;
//    int moby_type = MOBY_TYPE_DD;

    // Create a gRPC channel to the server
    std::shared_ptr<Channel> ecat_channel = grpc::CreateChannel("192.168.214.20:50050", grpc::InsecureChannelCredentials());
    std::shared_ptr<Channel> moby_channel = grpc::CreateChannel("192.168.214.20:50051", grpc::InsecureChannelCredentials());

    // Instantiate the GRPCMobyClient with the channel
    EcatClient ecat_client(ecat_channel);
    MobyClient moby_client(moby_channel);

    // Example: Call the GetMobyState function
    std::map<std::string, bool> moby_state;
    moby_client.GetMobyState(moby_state);
    std::cout << "MobyState: isReady=" << moby_state["is_ready"] << std::endl;
    std::cout << "MobyState: isMoving=" << moby_state["is_moving"] << std::endl;
    std::cout << "MobyState: isMoveFinished=" << moby_state["is_move_finished"] << std::endl;
    std::cout << "MobyState: isEmgPushed=" << moby_state["is_emg_pushed"] << std::endl;
    std::cout << "MobyState: isErrorState=" << moby_state["is_error_state"] << std::endl;
    std::cout << "MobyState: isHomePose=" << moby_state["is_home_pose"] << std::endl;
    std::cout << "MobyState: isResetting=" << moby_state["is_resetting"] << std::endl;
    std::cout << "MobyState: isIMUAvailable=" << moby_state["is_imu_avail"] << std::endl;
    std::cout << "MobyState: isProgramRunning=" << moby_state["is_program_running"] << std::endl;
    std::cout << "MobyState: isProgramPause=" << moby_state["is_program_pause"] << std::endl;
    std::cout << "MobyState: isRotationZero=" << moby_state["is_rotation_zero"] << std::endl;

    ///*** Get Motor status
    int swerve_rotation_motor_status[4]; // fl, fr, bl, br
    int swerve_drive_motor_status[4]; // fl, fr, bl, br
    int dd_motor_status[2];
    MotorDriverTx rotation_fl, rotation_fr, rotation_bl, rotation_br;
    MotorDriverTx drive_fl, drive_fr, drive_bl, drive_br;
    MotorDriverTx diff_left, diff_right;

    if (moby_type == MOBY_TYPE_SWERVE)
    {
        rotation_fr = moby_client.GetMobyTxData(0);
        rotation_br = moby_client.GetMobyTxData(2);
        rotation_bl = moby_client.GetMobyTxData(4);
        rotation_fl = moby_client.GetMobyTxData(6);

        drive_fr = moby_client.GetMobyTxData(1);
        drive_br = moby_client.GetMobyTxData(3);
        drive_bl = moby_client.GetMobyTxData(5);
        drive_fl = moby_client.GetMobyTxData(7);

        printf("Encoder Position (Rotation): [fr, br, bl, fl]=[%d, %d, %d, %d]\n",
               rotation_fr.actualposition(), rotation_br.actualposition(),
               rotation_bl.actualposition(), rotation_fl.actualposition());

        printf("Encoder Position (Driving): [fr, br, bl, fl]=[%d, %d, %d, %d]\n",
               drive_fr.actualposition(), drive_br.actualposition(),
               drive_bl.actualposition(), drive_fl.actualposition());

        printf("Motor State (Rotation): [fr, br, bl, fl]=[%d, %d, %d, %d]\n",
               rotation_fr.statusword(), rotation_br.statusword(),
               rotation_bl.statusword(), rotation_fl.statusword());

        printf("Motor State (Driving): [fr, br, bl, fl]=[%d, %d, %d, %d]\n",
               drive_fr.statusword(), drive_br.statusword(),
               drive_bl.statusword(), drive_fl.statusword());

        // Each motor's velocity
        double vel_fl, vel_fr, vel_bl, vel_br;
        moby_client.GetDriveSpeed(vel_fl, vel_fr, vel_bl, vel_br);
        printf("GetDriveSpeed: [fl, fr, bl, br]=[%f, %f, %f, %f] m/s\n", vel_fl, vel_fr, vel_bl, vel_br);

    }
    else if (moby_type == MOBY_TYPE_DD)
    {
        diff_left = moby_client.GetMobyTxData(0);
        diff_right = moby_client.GetMobyTxData(1);
        printf("Encoder Position: [left, right]=[%d, %d]\n",
               diff_left.actualposition(), diff_right.actualposition());

        printf("Motor State (Driving): [left, right]=[%d, %d]\n",
               diff_left.statusword(), diff_right.statusword());

        double vel_fl, vel_fr, vel_bl, vel_br;
        moby_client.GetDriveSpeed(vel_fl, vel_fr, vel_bl, vel_br);
        printf("GetDriveSpeed: [left, right]=[%f, %f] m/s\n", vel_fl, vel_fr);
    }
    else{
        //...
    }

    ///*** Moby pose and velocity
    double px, py, ptheta;
    double vx, vy, vtheta;
    moby_client.GetMobyPose(px, py, ptheta);
    std::cout << "Moby Pose: px=" << px << ", py=" << py << ", ptheta=" << ptheta << std::endl;

    moby_client.GetMobyVel(vx, vy, vtheta);
    std::cout << "Moby Velocity: vx=" << vx << ", vy=" << vy << ", vtheta=" << vtheta << std::endl;


    ///*** Command
    if (moby_type == MOBY_TYPE_SWERVE)
    {
        // Move command for target Vx, Vy, Vtheta
        double target_vx = 0;
        double target_vy = 0;
        double target_vtheta = 0;

        moby_client.SetTargetVelocity(target_vx, target_vy, target_vtheta);

        // Move command rotation motor position (fl, fr, bl, br)
        double target_rot_fl = 0;
        double target_rot_fr = 0;
        double target_rot_bl = 0;
        double target_rot_br = 0;
        moby_client.SetRotationAngleDeg(target_rot_fl, target_rot_fr, target_rot_bl, target_rot_br);

        // Move command driving motor velocity (fl, fr, bl, br)
        double target_drive_fl = 0;
        double target_drive_fr = 0;
        double target_drive_bl = 0;
        double target_drive_br = 0;
        moby_client.SetRotationAngleDeg(target_drive_fl, target_drive_fr, target_drive_bl, target_drive_br);



    }
    else if (moby_type == MOBY_TYPE_DD)
    {
        // Command for target Vx, Vy, Vtheta
        double target_vx = 0;
        double target_vy = 0;
        double target_vtheta = 0;

        moby_client.SetTargetVelocity(target_vx, target_vy, target_vtheta);

        // Move command driving motor velocity (left, right)
        double target_drive_left = 0;
        double target_drive_right = 0;
        moby_client.SetRotationAngleDeg(target_drive_left, target_drive_right, 0, 0);

    }
    moby_client.StopMotion();


    return 0;
}

