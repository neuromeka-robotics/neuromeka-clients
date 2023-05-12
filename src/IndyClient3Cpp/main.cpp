#include <unistd.h>

#include <iostream>
#include <memory>

#include "proto/IndyClient3.h"

int main(int argc, char** argv)
{
    // Create a gRPC channel to the server
    std::shared_ptr<Channel> indy_channel = grpc::CreateChannel("192.168.0.122:20132", grpc::InsecureChannelCredentials());

    // Instantiate the IndyClient3 with the channel
    IndyClient3 indy_client(indy_channel);

    // Example: Call the GetSystemInfoData function
    std::string control_task_ver;
    int32_t dof;
    std::string model_name;
    EndToolPortType endtool_port_type;
    std::string io_board_fw_ver;
    std::vector<std::string> core_board_fw_vers;
    std::string endtool_board_fw_ver;
    std::string robot_sn;

    indy_client.GetSystemInfoData(control_task_ver, dof, model_name, endtool_port_type, io_board_fw_ver, core_board_fw_vers, endtool_board_fw_ver, robot_sn);
    std::cout << "System Information (Model name): " << model_name << std::endl;
    std::cout << "System Information (Serial Number): " << robot_sn << std::endl;

    std::cout << std::endl;

    std::vector<float> q;
    std::vector<float> qdot;
    std::vector<float> qddot;
    std::vector<float> p;
    std::vector<float> pdot;
    std::vector<float> pddot;
    std::vector<float> ref_frame;
    std::vector<float> tool_frame;
    std::string running_time;
    indy_client.GetRTControlData(q, qdot, qddot, p, pdot, pddot, ref_frame, tool_frame, running_time);

    for (int i = 0; i < q.size(); i++)
    {
        std::cout << "RT Control Data: Joint Rotations (q[ " << i << "]): " << q[i] << std::endl;
    }

    for (int i = 0; i < p.size(); i++)
    {
        std::cout << "RT Control Data: Task Position (p[ " << i << "]): " << p[i] << std::endl;
    }

    std::vector<float> q_new_abs = {74.57f, 9.54f, 99.05f, 0.23f, 71.33f, 0.0f};
    std::vector<float> q_new_rel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -45.0f};

    std::cout << "Moving to the new position" << std::endl;

    indy_client.AMoveJ(q_new_abs, IndyFramework::Protobuf::Shared::BlendingType::BLENDING_TYPE_NONE, IndyFramework::Protobuf::Shared::JointBaseType::JOINT_BASE_TYPE_ABSOLUTE, 0.0f, 15.0f, 100.0f);
    indy_client.AMoveJ(q_new_rel, IndyFramework::Protobuf::Shared::BlendingType::BLENDING_TYPE_NONE, IndyFramework::Protobuf::Shared::JointBaseType::JOINT_BASE_TYPE_RELATIVE, 0.0f, 15.0f, 100.0f);

    std::cout << "Moving to the original position" << std::endl;
    indy_client.AMoveJ(q, IndyFramework::Protobuf::Shared::BlendingType::BLENDING_TYPE_NONE, IndyFramework::Protobuf::Shared::JointBaseType::JOINT_BASE_TYPE_ABSOLUTE, 0.0f, 15.0f, 100.0f);

    return 0;
}

