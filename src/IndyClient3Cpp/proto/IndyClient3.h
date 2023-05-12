#include "control.pb.h"
#include "control.grpc.pb.h"
#include "control_msgs.pb.h"

#include <grpcpp/grpcpp.h>
#include <vector>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using namespace IndyFramework::Protobuf::Control;

using IndyFramework::Protobuf::Shared::AnalogSignal;
using IndyFramework::Protobuf::Shared::BlendingType;
using IndyFramework::Protobuf::Shared::CircularMovingType;
using IndyFramework::Protobuf::Shared::CircularSettingType;
using IndyFramework::Protobuf::Shared::CollisionPolicy;
using IndyFramework::Protobuf::Shared::DigitalSignal;
using IndyFramework::Protobuf::Shared::EndToolPortType;
using IndyFramework::Protobuf::Shared::JointBaseType;
using IndyFramework::Protobuf::Shared::MoveSJPoint;
using IndyFramework::Protobuf::Shared::MoveSJTPoint;
using IndyFramework::Protobuf::Shared::MoveSLPoint;
using IndyFramework::Protobuf::Shared::MoveSLTPoint;
using IndyFramework::Protobuf::Shared::OpState;
using IndyFramework::Protobuf::Shared::PalletPoint;
using IndyFramework::Protobuf::Shared::PauseCategory;
using IndyFramework::Protobuf::Shared::ProgramState;
using IndyFramework::Protobuf::Shared::StopCategory;
using IndyFramework::Protobuf::Shared::TaskBaseType;
using IndyFramework::Protobuf::Shared::TrajCondition;
using IndyFramework::Protobuf::Shared::TrajState;

class IndyClient3{
public:
    IndyClient3(std::shared_ptr<grpc::Channel> channel);

    // Default Motion
    void AMoveJ(std::vector<float> jpos, BlendingType blending_type, JointBaseType base_type, float blending_radius, float vel, float acc);
    void AMoveJT(std::vector<float> jpos, BlendingType blending_type, JointBaseType base_type, float blending_radius, float time);
    void AMoveL(std::vector<float> tpos, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float vel, float acc);
    void AMoveLT(std::vector<float> tpos, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float time);
    void AMoveC(std::vector<float> tpos0, std::vector<float> tpos1, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float angle, CircularSettingType setting_type, CircularMovingType move_type, float vel, float acc);
    void AMoveCT(std::vector<float> tpos0, std::vector<float> tpos1, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float angle, CircularSettingType setting_type, CircularMovingType move_type, float time);
    void AWaitIO(std::vector<DigitalSignal> di_list, std::vector<DigitalSignal> do_list, std::vector<DigitalSignal> end_di_list, std::vector<DigitalSignal> end_do_list, int32_t conjunction, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list);
    void AWaitTime(float time, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list);
    void AWaitProgress(int32_t progress, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list);
    void AWaitTraj(TrajCondition traj_condition, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list);
    void AWaitRadius(int32_t radius, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list);
    void AMotionStop();
    
    // Advanced Motion
    void MoveSJ(std::vector<MoveSJPoint> points, BlendingType blending_type, JointBaseType base_type, float blending_radius, float vel, float acc);
    void MoveSJT(std::vector<MoveSJTPoint> points, JointBaseType base_type);
    void MoveSL(std::vector<MoveSLPoint> points, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float disp_vel, float disp_acc, float rot_vel, float rot_acc);
    void MoveSLT(std::vector<MoveSLTPoint> points, BlendingType blending_type, TaskBaseType base_type, float blending_radius);
    void MoveSpiral();
    // Motion Config
    void SetRefFrame(std::vector<float> ref_frame);
    void SetRefFramePlanar(std::vector<float> tpos0, std::vector<float> tpos1, std::vector<float> tpos2, std::vector<float> &ref_frame);
    void SetToolFrame(std::vector<float> tool_frame);
    void SetSpeedRatio(int32_t ratio);
    void SetCommandSpeedRatio(int32_t ratio);
    
    // ---------------------------------------------------------------- //
    // Command
    // ---------------------------------------------------------------- //
    void Reboot();
    void Recover();
    void Stop(StopCategory stop_category);
    void Pause(PauseCategory pause_category);
    void Resume();
    void Brake(std::vector<bool> on_list);
    void Servo(bool on);
    void SetAutoServoOff(bool enable, float time);
    void GetAutoServoOff(bool &enable, float &time);
    void SimulationMode(bool on);
    void DirectTeachingMode(bool on);
    
    // ---------------------------------------------------------------- //
    // Data
    // ---------------------------------------------------------------- //
    void GetRTControlData(std::vector<float> &q, std::vector<float> &qdot, std::vector<float> &qddot, std::vector<float> &p, std::vector<float> &pdot, std::vector<float> &pddot, std::vector<float> &ref_frame, std::vector<float> &tool_frame, std::string &running_time);
    void GetIOData(std::vector<bool> &di, std::vector<bool> &do_, std::vector<int32_t> &ai, std::vector<int32_t> &ao, std::vector<bool> &end_di, std::vector<bool> &end_do, std::vector<int32_t> &end_ai, std::vector<int32_t> &end_ao);
    void GetCoreData(std::vector<float> &temperatures, std::vector<float> &voltages, std::vector<float> &currents, std::vector<std::string> &states, std::vector<std::string> &state_codes, std::vector<bool> &brake_states, bool &servo_state);
    void GetSystemInfoData(std::string &control_task_ver, int32_t &dof, std::string &model_name, EndToolPortType &endtool_port_type, std::string &io_board_fw_ver, std::vector<std::string> &core_board_fw_vers, std::string &endtool_board_fw_ver, std::string &robot_sn);
    void GetMotionData(TrajState &traj_state, int32_t &traj_progress, bool &is_in_motion, bool &is_motion_done, bool &is_pausing, bool &is_stopping, bool &has_motion, int32_t &speed_ratio, int32_t &motion_id, float &remain_distance, uint32_t &motion_queue_size, int32_t &cur_traj_progress);
    void GetStateData(bool &is_simulation_mode, OpState &state, std::string &violation);
    void GetViolationData(uint64_t &violation_code, uint32_t &j_index, std::vector<int32_t> &i_args, std::vector<float> &f_args);
    void SetProgramState(ProgramState program_state);
    
    // ---------------------------------------------------------------- //
    // Utility
    // ---------------------------------------------------------------- //
    void InverseKinematics(std::vector<float> tpos, std::vector<float> init_jpos, std::vector<float> &jpos);
    void CheckAproachRetractValid(std::vector<float> tpos, std::vector<float> init_jpos, std::vector<float> pre_tpos, std::vector<float> post_tpos, bool &is_valid, std::vector<float> &tar_pos, std::vector<float> &approach_pos, std::vector<float> &retract_pos);
    void GetPalletPointList(std::vector<float> tpos, std::vector<float> jpos, std::vector<float> pre_tpos, std::vector<float> post_tpos, int32_t pallet_pattern, int32_t width, int32_t height, std::vector<PalletPoint> &pallet_points);
    void CalculateRelativePose(std::vector<float> start_pos, std::vector<float> end_pos, TaskBaseType base_type, std::vector<float> &relative_pos);
    void CalculateCurrentPoseRel(std::vector<float> current_pos, std::vector<float> relative_pos, TaskBaseType base_type, std::vector<float> &calculated_pos);
    
    // ---------------------------------------------------------------- //
    // DIO
    // ---------------------------------------------------------------- //
    void GetDI(std::vector<bool> &di);
    void SetDI(std::vector<DigitalSignal> di_list);
    void SetDO(std::vector<DigitalSignal> do_list);
    void GetDO(std::vector<bool> &do_);
    void GetAI(std::vector<int32_t> &ai);
    void SetAI(std::vector<AnalogSignal> ai_list);
    void GetAO(std::vector<int32_t> &ao);
    void SetAO(std::vector<AnalogSignal> ao_list);
    void GetEndDI(std::vector<bool> &di);
    void SetEndDI(std::vector<DigitalSignal> di_list);
    void SetEndDO(std::vector<DigitalSignal> do_list);
    void GetEndDO(std::vector<bool> &do_);
    void GetEndAI(std::vector<int32_t> &ai);
    void SetEndAI(std::vector<AnalogSignal> ai_list);
    void GetEndAO(std::vector<int32_t> &ao);
    void SetEndAO(std::vector<AnalogSignal> ao_list);
    
    // ---------------------------------------------------------------- //
    // Config Setting
    // ---------------------------------------------------------------- //
    void SetJointControlGain(std::vector<float> kp, std::vector<float> kv, std::vector<float> kl2);
    void GetJointControlGain(std::vector<float> &kp, std::vector<float> &kv, std::vector<float> &kl2);
    void SetTaskControlGain(std::vector<float> kp, std::vector<float> kv, std::vector<float> kl2, std::string msg);
    void GetTaskControlGain(std::vector<float> &kp, std::vector<float> &kv, std::vector<float> &kl2);
    void SetImpedanceControlGain(std::vector<float> mass, std::vector<float> damping, std::vector<float> stiffness, std::vector<float> kl2, std::string msg);
    void GetImpedanceControlGain(std::vector<float> &mass, std::vector<float> &damping, std::vector<float> &stiffness, std::vector<float> &kl2);
    void SetFricComp(bool control_comp, std::vector<int32_t> control_comp_levels, bool dt_comp, std::vector<int32_t> dt_comp_levels, int32_t id_joint);
    void GetFricComp(bool &control_comp, std::vector<int32_t> &control_comp_levels, bool &dt_comp, std::vector<int32_t> &dt_comp_levels);
    void SetMountPos(float ry, float rz);
    void GetMountPos(float &ry, float &rz);
    void SetToolProperty(float mass, std::vector<float> center_of_mass, std::vector<float> inertia);
    void GetToolProperty(float &mass, std::vector<float> &center_of_mass, std::vector<float> &inertia);
    void SetCollSensLevel(int32_t level);
    void GetCollSensLevel(int32_t &level);
    void SetCollSensParam(std::vector<float> j_torque_bases, std::vector<float> j_torque_tangents, std::vector<float> t_torque_bases, std::vector<float> t_torque_tangents, std::vector<float> error_bases, std::vector<float> error_tangents, std::vector<float> t_constvel_torque_bases, std::vector<float> t_constvel_torque_tangents, std::vector<float> t_conveyor_torque_bases, std::vector<float> t_conveyor_torque_tangents);
    void GetCollSensParam(std::vector<float> &j_torque_bases, std::vector<float> &j_torque_tangents, std::vector<float> &t_torque_bases, std::vector<float> &t_torque_tangents, std::vector<float> &error_bases, std::vector<float> &error_tangents, std::vector<float> &t_constvel_torque_bases, std::vector<float> &t_constvel_torque_tangents, std::vector<float> &t_conveyor_torque_bases, std::vector<float> &t_conveyor_torque_tangents);
    void SetCollPolicy(CollisionPolicy policy, float sleep_time, float gravity_time);
    void GetCollPolicy(CollisionPolicy &policy, float &sleep_time, float &gravity_time);
    void GetCollTuningParam(std::vector<float> &j_torque_bases, std::vector<float> &j_torque_tangents, std::vector<float> &t_torque_bases, std::vector<float> &t_torque_tangents, std::vector<float> &error_bases, std::vector<float> &error_tangents, std::vector<float> &t_constvel_torque_bases, std::vector<float> &t_constvel_torque_tangents, std::vector<float> &t_conveyor_torque_bases, std::vector<float> &t_conveyor_torque_tangents, bool &is_calc_done);
    void GetSafetyLimitConfig(float &power_limit, float &power_limit_ratio, float &tcp_force_limit, float &tcp_force_limit_ratio, float &tcp_speed_limit, float &tcp_speed_limit_ratio, std::vector<float> &joint_limits);
    void SetSafetyLimitConfig(float power_limit, float power_limit_ratio, float tcp_force_limit, float tcp_force_limit_ratio, float tcp_speed_limit, float tcp_speed_limit_ratio, std::vector<float> joint_limits);
    void GetSafetyStopConfig(StopCategory &joint_position_limit_stop_cat, StopCategory &joint_speed_limit_stop_cat, StopCategory &joint_torque_limit_stop_cat, StopCategory &tcp_speed_limit_stop_cat, StopCategory &tcp_force_limit_stop_cat, StopCategory &power_limit_stop_cat);
    void SetSafetyStopConfig(StopCategory joint_position_limit_stop_cat, StopCategory joint_speed_limit_stop_cat, StopCategory joint_torque_limit_stop_cat, StopCategory tcp_speed_limit_stop_cat, StopCategory tcp_force_limit_stop_cat, StopCategory power_limit_stop_cat);
    // beckhoff_conv
    void GetEL5001(int32_t &status, int32_t &value, int32_t &delta, float &average);
    void GetEL5101(int32_t &status, int32_t &value, int32_t &latch, int32_t &delta, float &average);

private:
    std::unique_ptr<IndyFramework::Protobuf::Control::Control::Stub> stub_;
};