#include "IndyClient3.h"

IndyClient3::IndyClient3(std::shared_ptr<grpc::Channel> channel)
: stub_(IndyFramework::Protobuf::Control::Control::NewStub(channel))
{}

void IndyClient3::AMoveJ(std::vector<float> jpos, BlendingType blending_type, JointBaseType base_type, float blending_radius, float vel, float acc)
{
    AMoveJReq request;
    for (int i = 0; i < jpos.size(); i++)
    {
        request.add_jpos(jpos[i]);
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_vel(vel);
    request.set_acc(acc);

    ClientContext context;
    AMoveJRes response;

    Status resultStatus = stub_->AMoveJ(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AMoveJ failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AMoveJT(std::vector<float> jpos, BlendingType blending_type, JointBaseType base_type, float blending_radius, float time)
{
    AMoveJTReq request;
    for (int i = 0; i < jpos.size(); i++)
    {
        request.add_jpos(jpos[i]);
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_time(time);

    ClientContext context;
    AMoveJTRes response;

    Status resultStatus = stub_->AMoveJT(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AMoveJT failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AMoveL(std::vector<float> tpos, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float vel, float acc)
{
    AMoveLReq request;
    for (int i = 0; i < tpos.size(); i++)
    {
        request.add_tpos(tpos[i]);
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_vel(vel);
    request.set_acc(acc);

    ClientContext context;
    AMoveLRes response;

    Status resultStatus = stub_->AMoveL(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AMoveL failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AMoveLT(std::vector<float> tpos, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float time)
{
    AMoveLTReq request;
    for (int i = 0; i < tpos.size(); i++)
    {
        request.add_tpos(tpos[i]);
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_time(time);

    ClientContext context;
    AMoveLTRes response;

    Status resultStatus = stub_->AMoveLT(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AMoveLT failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AMoveC(std::vector<float> tpos0, std::vector<float> tpos1, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float angle, CircularSettingType setting_type, CircularMovingType move_type, float vel, float acc)
{
    AMoveCReq request;
    for (int i = 0; i < tpos0.size(); i++)
    {
        request.add_tpos0(tpos0[i]);
    }
    for (int i = 0; i < tpos1.size(); i++)
    {
        request.add_tpos1(tpos1[i]);
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_angle(angle);
    request.set_setting_type(setting_type);
    request.set_move_type(move_type);
    request.set_vel(vel);
    request.set_acc(acc);

    ClientContext context;
    AMoveCRes response;

    Status resultStatus = stub_->AMoveC(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AMoveC failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AMoveCT(std::vector<float> tpos0, std::vector<float> tpos1, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float angle, CircularSettingType setting_type, CircularMovingType move_type, float time)
{
    AMoveCTReq request;
    for (int i = 0; i < tpos0.size(); i++)
    {
        request.add_tpos0(tpos0[i]);
    }
    for (int i = 0; i < tpos1.size(); i++)
    {
        request.add_tpos1(tpos1[i]);
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_angle(angle);
    request.set_setting_type(setting_type);
    request.set_move_type(move_type);
    request.set_time(time);

    ClientContext context;
    AMoveCTRes response;

    Status resultStatus = stub_->AMoveCT(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AMoveCT failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AWaitIO(std::vector<DigitalSignal> di_list, std::vector<DigitalSignal> do_list, std::vector<DigitalSignal> end_di_list, std::vector<DigitalSignal> end_do_list, int32_t conjunction, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list)
{
    AWaitIOReq request;
    for (int i = 0; i < di_list.size(); i++)
    {
        DigitalSignal* di_list_pt = request.add_di_list();
        di_list_pt->set_addr(di_list[i].addr());
        di_list_pt->set_on(di_list[i].on());
    }
    for (int i = 0; i < do_list.size(); i++)
    {
        DigitalSignal* do_list_pt = request.add_do_list();
        do_list_pt->set_addr(do_list[i].addr());
        do_list_pt->set_on(do_list[i].on());
    }
    for (int i = 0; i < end_di_list.size(); i++)
    {
        DigitalSignal* end_di_list_pt = request.add_end_di_list();
        end_di_list_pt->set_addr(end_di_list[i].addr());
        end_di_list_pt->set_on(end_di_list[i].on());
    }
    for (int i = 0; i < end_do_list.size(); i++)
    {
        DigitalSignal* end_do_list_pt = request.add_end_do_list();
        end_do_list_pt->set_addr(end_do_list[i].addr());
        end_do_list_pt->set_on(end_do_list[i].on());
    }
    request.set_conjunction(conjunction);
    for (int i = 0; i < set_do_list.size(); i++)
    {
        DigitalSignal* set_do_list_pt = request.add_set_do_list();
        set_do_list_pt->set_addr(set_do_list[i].addr());
        set_do_list_pt->set_on(set_do_list[i].on());
    }
    for (int i = 0; i < set_end_do_list.size(); i++)
    {
        DigitalSignal* set_end_do_list_pt = request.add_set_end_do_list();
        set_end_do_list_pt->set_addr(set_end_do_list[i].addr());
        set_end_do_list_pt->set_on(set_end_do_list[i].on());
    }
    for (int i = 0; i < set_ao_list.size(); i++)
    {
        AnalogSignal* set_ao_list_pt = request.add_set_ao_list();
        set_ao_list_pt->set_addr(set_ao_list[i].addr());
        set_ao_list_pt->set_voltage(set_ao_list[i].voltage());
    }
    for (int i = 0; i < set_end_ao_list.size(); i++)
    {
        AnalogSignal* set_end_ao_list_pt = request.add_set_end_ao_list();
        set_end_ao_list_pt->set_addr(set_end_ao_list[i].addr());
        set_end_ao_list_pt->set_voltage(set_end_ao_list[i].voltage());
    }

    ClientContext context;
    AWaitIORes response;

    Status resultStatus = stub_->AWaitIO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AWaitIO failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AWaitTime(float time, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list)
{
    AWaitTimeReq request;
    request.set_time(time);
    for (int i = 0; i < set_do_list.size(); i++)
    {
        DigitalSignal* set_do_list_pt = request.add_set_do_list();
        set_do_list_pt->set_addr(set_do_list[i].addr());
        set_do_list_pt->set_on(set_do_list[i].on());
    }
    for (int i = 0; i < set_end_do_list.size(); i++)
    {
        DigitalSignal* set_end_do_list_pt = request.add_set_end_do_list();
        set_end_do_list_pt->set_addr(set_end_do_list[i].addr());
        set_end_do_list_pt->set_on(set_end_do_list[i].on());
    }
    for (int i = 0; i < set_ao_list.size(); i++)
    {
        AnalogSignal* set_ao_list_pt = request.add_set_ao_list();
        set_ao_list_pt->set_addr(set_ao_list[i].addr());
        set_ao_list_pt->set_voltage(set_ao_list[i].voltage());
    }
    for (int i = 0; i < set_end_ao_list.size(); i++)
    {
        AnalogSignal* set_end_ao_list_pt = request.add_set_end_ao_list();
        set_end_ao_list_pt->set_addr(set_end_ao_list[i].addr());
        set_end_ao_list_pt->set_voltage(set_end_ao_list[i].voltage());
    }

    ClientContext context;
    AWaitTimeRes response;

    Status resultStatus = stub_->AWaitTime(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AWaitTime failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AWaitProgress(int32_t progress, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list)
{
    AWaitProgressReq request;
    request.set_progress(progress);
    for (int i = 0; i < set_do_list.size(); i++)
    {
        DigitalSignal* set_do_list_pt = request.add_set_do_list();
        set_do_list_pt->set_addr(set_do_list[i].addr());
        set_do_list_pt->set_on(set_do_list[i].on());
    }
    for (int i = 0; i < set_end_do_list.size(); i++)
    {
        DigitalSignal* set_end_do_list_pt = request.add_set_end_do_list();
        set_end_do_list_pt->set_addr(set_end_do_list[i].addr());
        set_end_do_list_pt->set_on(set_end_do_list[i].on());
    }
    for (int i = 0; i < set_ao_list.size(); i++)
    {
        AnalogSignal* set_ao_list_pt = request.add_set_ao_list();
        set_ao_list_pt->set_addr(set_ao_list[i].addr());
        set_ao_list_pt->set_voltage(set_ao_list[i].voltage());
    }
    for (int i = 0; i < set_end_ao_list.size(); i++)
    {
        AnalogSignal* set_end_ao_list_pt = request.add_set_end_ao_list();
        set_end_ao_list_pt->set_addr(set_end_ao_list[i].addr());
        set_end_ao_list_pt->set_voltage(set_end_ao_list[i].voltage());
    }

    ClientContext context;
    AWaitProgressRes response;

    Status resultStatus = stub_->AWaitProgress(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AWaitProgress failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AWaitTraj(TrajCondition traj_condition, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list)
{
    AWaitTrajReq request;
    request.set_traj_condition(traj_condition);
    for (int i = 0; i < set_do_list.size(); i++)
    {
        DigitalSignal* set_do_list_pt = request.add_set_do_list();
        set_do_list_pt->set_addr(set_do_list[i].addr());
        set_do_list_pt->set_on(set_do_list[i].on());
    }
    for (int i = 0; i < set_end_do_list.size(); i++)
    {
        DigitalSignal* set_end_do_list_pt = request.add_set_end_do_list();
        set_end_do_list_pt->set_addr(set_end_do_list[i].addr());
        set_end_do_list_pt->set_on(set_end_do_list[i].on());
    }
    for (int i = 0; i < set_ao_list.size(); i++)
    {
        AnalogSignal* set_ao_list_pt = request.add_set_ao_list();
        set_ao_list_pt->set_addr(set_ao_list[i].addr());
        set_ao_list_pt->set_voltage(set_ao_list[i].voltage());
    }
    for (int i = 0; i < set_end_ao_list.size(); i++)
    {
        AnalogSignal* set_end_ao_list_pt = request.add_set_end_ao_list();
        set_end_ao_list_pt->set_addr(set_end_ao_list[i].addr());
        set_end_ao_list_pt->set_voltage(set_end_ao_list[i].voltage());
    }

    ClientContext context;
    AWaitTrajRes response;

    Status resultStatus = stub_->AWaitTraj(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AWaitTraj failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::AWaitRadius(int32_t radius, std::vector<DigitalSignal> set_do_list, std::vector<DigitalSignal> set_end_do_list, std::vector<AnalogSignal> set_ao_list, std::vector<AnalogSignal> set_end_ao_list)
{
    AWaitRadiusReq request;
    request.set_radius(radius);
    for (int i = 0; i < set_do_list.size(); i++)
    {
        DigitalSignal* set_do_list_pt = request.add_set_do_list();
        set_do_list_pt->set_addr(set_do_list[i].addr());
        set_do_list_pt->set_on(set_do_list[i].on());
    }
    for (int i = 0; i < set_end_do_list.size(); i++)
    {
        DigitalSignal* set_end_do_list_pt = request.add_set_end_do_list();
        set_end_do_list_pt->set_addr(set_end_do_list[i].addr());
        set_end_do_list_pt->set_on(set_end_do_list[i].on());
    }
    for (int i = 0; i < set_ao_list.size(); i++)
    {
        AnalogSignal* set_ao_list_pt = request.add_set_ao_list();
        set_ao_list_pt->set_addr(set_ao_list[i].addr());
        set_ao_list_pt->set_voltage(set_ao_list[i].voltage());
    }
    for (int i = 0; i < set_end_ao_list.size(); i++)
    {
        AnalogSignal* set_end_ao_list_pt = request.add_set_end_ao_list();
        set_end_ao_list_pt->set_addr(set_end_ao_list[i].addr());
        set_end_ao_list_pt->set_voltage(set_end_ao_list[i].voltage());
    }

    ClientContext context;
    AWaitRadiusRes response;

    Status resultStatus = stub_->AWaitRadius(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "AWaitRadius failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::MoveSJ(std::vector<MoveSJPoint> points, BlendingType blending_type, JointBaseType base_type, float blending_radius, float vel, float acc)
{
    MoveSJReq request;
    for (int i = 0; i < points.size(); i++)
    {
        MoveSJPoint* points_pt = request.add_points();
        for (int j = 0; j < points[i].jpos_size(); j++)
        {
            points_pt->add_jpos(points[i].jpos(j));
        }
        points_pt->set_blending_radius(points[i].blending_radius());
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_vel(vel);
    request.set_acc(acc);

    ClientContext context;
    MoveSJRes response;

    Status resultStatus = stub_->MoveSJ(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "MoveSJ failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::MoveSJT(std::vector<MoveSJTPoint> points, JointBaseType base_type)
{
    MoveSJTReq request;
    for (int i = 0; i < points.size(); i++)
    {
        MoveSJTPoint* points_pt = request.add_points();
        for (int j = 0; j < points[i].jpos_size(); j++)
        {
            points_pt->add_jpos(points[i].jpos(j));
        }
        points_pt->set_blending_radius(points[i].blending_radius());
        points_pt->set_time(points[i].time());
    }
    request.set_base_type(base_type);

    ClientContext context;
    MoveSJTRes response;

    Status resultStatus = stub_->MoveSJT(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "MoveSJT failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::MoveSL(std::vector<MoveSLPoint> points, BlendingType blending_type, TaskBaseType base_type, float blending_radius, float disp_vel, float disp_acc, float rot_vel, float rot_acc)
{
    MoveSLReq request;
    for (int i = 0; i < points.size(); i++)
    {
        MoveSLPoint* points_pt = request.add_points();
        for (int j = 0; j < points[i].tpos_size(); j++)
        {
            points_pt->add_tpos(points[i].tpos(j));
        }
        points_pt->set_blending_radius(points[i].blending_radius());
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);
    request.set_disp_vel(disp_vel);
    request.set_disp_acc(disp_acc);
    request.set_rot_vel(rot_vel);
    request.set_rot_acc(rot_acc);

    ClientContext context;
    MoveSLRes response;

    Status resultStatus = stub_->MoveSL(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "MoveSL failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::MoveSLT(std::vector<MoveSLTPoint> points, BlendingType blending_type, TaskBaseType base_type, float blending_radius)
{
    MoveSLTReq request;
    for (int i = 0; i < points.size(); i++)
    {
        MoveSLTPoint* points_pt = request.add_points();
        for (int j = 0; j < points[i].tpos_size(); j++)
        {
            points_pt->add_tpos(points[i].tpos(j));
        }
        points_pt->set_blending_radius(points[i].blending_radius());
        points_pt->set_time(points[i].time());
    }
    request.set_blending_type(blending_type);
    request.set_base_type(base_type);
    request.set_blending_radius(blending_radius);

    ClientContext context;
    MoveSLTRes response;

    Status resultStatus = stub_->MoveSLT(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "MoveSLT failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetRefFrame(std::vector<float> ref_frame)
{
    SetRefFrameReq request;
    for (int i = 0; i < ref_frame.size(); i++)
    {
        request.add_ref_frame(ref_frame[i]);
    }

    ClientContext context;
    SetRefFrameRes response;

    Status resultStatus = stub_->SetRefFrame(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetRefFrame failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetRefFramePlanar(std::vector<float> tpos0, std::vector<float> tpos1, std::vector<float> tpos2, std::vector<float> &ref_frame)
{
    SetRefFramePlanarReq request;
    for (int i = 0; i < tpos0.size(); i++)
    {
        request.add_tpos0(tpos0[i]);
    }
    for (int i = 0; i < tpos1.size(); i++)
    {
        request.add_tpos1(tpos1[i]);
    }
    for (int i = 0; i < tpos2.size(); i++)
    {
        request.add_tpos2(tpos2[i]);
    }

    ClientContext context;
    SetRefFramePlanarRes response;

    Status resultStatus = stub_->SetRefFramePlanar(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetRefFramePlanar failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.ref_frame_size(); i++)
        {
            ref_frame.push_back(response.ref_frame(i));
        }
        return;
    }
}

void IndyClient3::SetToolFrame(std::vector<float> tool_frame)
{
    SetToolFrameReq request;
    for (int i = 0; i < tool_frame.size(); i++)
    {
        request.add_tool_frame(tool_frame[i]);
    }

    ClientContext context;
    SetToolFrameRes response;

    Status resultStatus = stub_->SetToolFrame(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetToolFrame failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetSpeedRatio(int32_t ratio)
{
    SetSpeedRatioReq request;
    request.set_ratio(ratio);

    ClientContext context;
    SetSpeedRatioRes response;

    Status resultStatus = stub_->SetSpeedRatio(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetSpeedRatio failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetCommandSpeedRatio(int32_t ratio)
{
    SetCommandSpeedRatioReq request;
    request.set_ratio(ratio);

    ClientContext context;
    SetCommandSpeedRatioRes response;

    Status resultStatus = stub_->SetCommandSpeedRatio(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetCommandSpeedRatio failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::Stop(StopCategory stop_category)
{
    StopReq request;
    request.set_stop_category(stop_category);

    ClientContext context;
    StopRes response;

    Status resultStatus = stub_->Stop(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "Stop failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::Pause(PauseCategory pause_category)
{
    PauseReq request;
    request.set_pause_category(pause_category);

    ClientContext context;
    PauseRes response;

    Status resultStatus = stub_->Pause(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "Pause failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::Brake(std::vector<bool> on_list)
{
    BrakeReq request;
    for (int i = 0; i < on_list.size(); i++)
    {
        request.add_on_list(on_list[i]);
    }

    ClientContext context;
    BrakeRes response;

    Status resultStatus = stub_->Brake(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "Brake failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::Servo(bool on)
{
    ServoReq request;
    request.set_on(on);

    ClientContext context;
    ServoRes response;

    Status resultStatus = stub_->Servo(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "Servo failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetAutoServoOff(bool enable, float time)
{
    SetAutoServoOffReq request;
    request.set_enable(enable);
    request.set_time(time);

    ClientContext context;
    SetAutoServoOffRes response;

    Status resultStatus = stub_->SetAutoServoOff(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetAutoServoOff failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetAutoServoOff(bool &enable, float &time)
{
    GetAutoServoOffReq request;

    ClientContext context;
    GetAutoServoOffRes response;

    Status resultStatus = stub_->GetAutoServoOff(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetAutoServoOff failed." << std::endl;
    }
    else{
        enable = response.enable();
        time = response.time();
        return;
    }
}

void IndyClient3::SimulationMode(bool on)
{
    SimulationModeReq request;
    request.set_on(on);

    ClientContext context;
    SimulationModeRes response;

    Status resultStatus = stub_->SimulationMode(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SimulationMode failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::DirectTeachingMode(bool on)
{
    DirectTeachingModeReq request;
    request.set_on(on);

    ClientContext context;
    DirectTeachingModeRes response;

    Status resultStatus = stub_->DirectTeachingMode(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "DirectTeachingMode failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetRTControlData(std::vector<float> &q, std::vector<float> &qdot, std::vector<float> &qddot, std::vector<float> &p, std::vector<float> &pdot, std::vector<float> &pddot, std::vector<float> &ref_frame, std::vector<float> &tool_frame, std::string &running_time)
{
    GetRTControlDataReq request;

    ClientContext context;
    GetRTControlDataRes response;

    Status resultStatus = stub_->GetRTControlData(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetRTControlData failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.q_size(); i++)
        {
            q.push_back(response.q(i));
        }
        for (int i = 0; i < response.qdot_size(); i++)
        {
            qdot.push_back(response.qdot(i));
        }
        for (int i = 0; i < response.qddot_size(); i++)
        {
            qddot.push_back(response.qddot(i));
        }
        for (int i = 0; i < response.p_size(); i++)
        {
            p.push_back(response.p(i));
        }
        for (int i = 0; i < response.pdot_size(); i++)
        {
            pdot.push_back(response.pdot(i));
        }
        for (int i = 0; i < response.pddot_size(); i++)
        {
            pddot.push_back(response.pddot(i));
        }
        for (int i = 0; i < response.ref_frame_size(); i++)
        {
            ref_frame.push_back(response.ref_frame(i));
        }
        for (int i = 0; i < response.tool_frame_size(); i++)
        {
            tool_frame.push_back(response.tool_frame(i));
        }
        running_time = response.running_time();
        return;
    }
}

void IndyClient3::GetIOData(std::vector<bool> &di, std::vector<bool> &do_, std::vector<int32_t> &ai, std::vector<int32_t> &ao, std::vector<bool> &end_di, std::vector<bool> &end_do, std::vector<int32_t> &end_ai, std::vector<int32_t> &end_ao)
{
    GetIODataReq request;

    ClientContext context;
    GetIODataRes response;

    Status resultStatus = stub_->GetIOData(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetIOData failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.di_size(); i++)
        {
            di.push_back(response.di(i));
        }
        for (int i = 0; i < response.do__size(); i++)
        {
            do_.push_back(response.do_(i));
        }
        for (int i = 0; i < response.ai_size(); i++)
        {
            ai.push_back(response.ai(i));
        }
        for (int i = 0; i < response.ao_size(); i++)
        {
            ao.push_back(response.ao(i));
        }
        for (int i = 0; i < response.end_di_size(); i++)
        {
            end_di.push_back(response.end_di(i));
        }
        for (int i = 0; i < response.end_do_size(); i++)
        {
            end_do.push_back(response.end_do(i));
        }
        for (int i = 0; i < response.end_ai_size(); i++)
        {
            end_ai.push_back(response.end_ai(i));
        }
        for (int i = 0; i < response.end_ao_size(); i++)
        {
            end_ao.push_back(response.end_ao(i));
        }
        return;
    }
}

void IndyClient3::GetCoreData(std::vector<float> &temperatures, std::vector<float> &voltages, std::vector<float> &currents, std::vector<std::string> &states, std::vector<std::string> &state_codes, std::vector<bool> &brake_states, bool &servo_state)
{
    GetCoreDataReq request;

    ClientContext context;
    GetCoreDataRes response;

    Status resultStatus = stub_->GetCoreData(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetCoreData failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.temperatures_size(); i++)
        {
            temperatures.push_back(response.temperatures(i));
        }
        for (int i = 0; i < response.voltages_size(); i++)
        {
            voltages.push_back(response.voltages(i));
        }
        for (int i = 0; i < response.currents_size(); i++)
        {
            currents.push_back(response.currents(i));
        }
        for (int i = 0; i < response.states_size(); i++)
        {
            states.push_back(response.states(i));
        }
        for (int i = 0; i < response.state_codes_size(); i++)
        {
            state_codes.push_back(response.state_codes(i));
        }
        for (int i = 0; i < response.brake_states_size(); i++)
        {
            brake_states.push_back(response.brake_states(i));
        }
        servo_state = response.servo_state();
        return;
    }
}

void IndyClient3::GetSystemInfoData(std::string &control_task_ver, int32_t &dof, std::string &model_name, EndToolPortType &endtool_port_type, std::string &io_board_fw_ver, std::vector<std::string> &core_board_fw_vers, std::string &endtool_board_fw_ver, std::string &robot_sn)
{
    GetSystemInfoDataReq request;

    ClientContext context;
    GetSystemInfoDataRes response;

    Status resultStatus = stub_->GetSystemInfoData(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetSystemInfoData failed." << std::endl;
    }
    else{
        control_task_ver = response.control_task_ver();
        dof = response.dof();
        model_name = response.model_name();
        endtool_port_type = response.endtool_port_type();
        io_board_fw_ver = response.io_board_fw_ver();
        for (int i = 0; i < response.core_board_fw_vers_size(); i++)
        {
            core_board_fw_vers.push_back(response.core_board_fw_vers(i));
        }
        endtool_board_fw_ver = response.endtool_board_fw_ver();
        robot_sn = response.robot_sn();
        return;
    }
}

void IndyClient3::GetMotionData(TrajState &traj_state, int32_t &traj_progress, bool &is_in_motion, bool &is_motion_done, bool &is_pausing, bool &is_stopping, bool &has_motion, int32_t &speed_ratio, int32_t &motion_id, float &remain_distance, uint32_t &motion_queue_size, int32_t &cur_traj_progress)
{
    GetMotionDataReq request;

    ClientContext context;
    GetMotionDataRes response;

    Status resultStatus = stub_->GetMotionData(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetMotionData failed." << std::endl;
    }
    else{
        traj_state = response.traj_state();
        traj_progress = response.traj_progress();
        is_in_motion = response.is_in_motion();
        is_motion_done = response.is_motion_done();
        is_pausing = response.is_pausing();
        is_stopping = response.is_stopping();
        has_motion = response.has_motion();
        speed_ratio = response.speed_ratio();
        motion_id = response.motion_id();
        remain_distance = response.remain_distance();
        motion_queue_size = response.motion_queue_size();
        cur_traj_progress = response.cur_traj_progress();
        return;
    }
}

void IndyClient3::GetStateData(bool &is_simulation_mode, OpState &state, std::string &violation)
{
    GetStateDataReq request;

    ClientContext context;
    GetStateDataRes response;

    Status resultStatus = stub_->GetStateData(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetStateData failed." << std::endl;
    }
    else{
        is_simulation_mode = response.is_simulation_mode();
        state = response.state();
        violation = response.violation();
        return;
    }
}

void IndyClient3::GetViolationData(uint64_t &violation_code, uint32_t &j_index, std::vector<int32_t> &i_args, std::vector<float> &f_args)
{
    GetViolationDataReq request;

    ClientContext context;
    GetViolationDataRes response;

    Status resultStatus = stub_->GetViolationData(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetViolationData failed." << std::endl;
    }
    else{
        violation_code = response.violation_code();
        j_index = response.j_index();
        for (int i = 0; i < response.i_args_size(); i++)
        {
            i_args.push_back(response.i_args(i));
        }
        for (int i = 0; i < response.f_args_size(); i++)
        {
            f_args.push_back(response.f_args(i));
        }
        return;
    }
}

void IndyClient3::SetProgramState(ProgramState program_state)
{
    SetProgramStateReq request;
    request.set_program_state(program_state);

    ClientContext context;
    SetProgramStateRes response;

    Status resultStatus = stub_->SetProgramState(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetProgramState failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::InverseKinematics(std::vector<float> tpos, std::vector<float> init_jpos, std::vector<float> &jpos)
{
    InverseKinematicsReq request;
    for (int i = 0; i < tpos.size(); i++)
    {
        request.add_tpos(tpos[i]);
    }
    for (int i = 0; i < init_jpos.size(); i++)
    {
        request.add_init_jpos(init_jpos[i]);
    }

    ClientContext context;
    InverseKinematicsRes response;

    Status resultStatus = stub_->InverseKinematics(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "InverseKinematics failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.jpos_size(); i++)
        {
            jpos.push_back(response.jpos(i));
        }
        return;
    }
}

void IndyClient3::CheckAproachRetractValid(std::vector<float> tpos, std::vector<float> init_jpos, std::vector<float> pre_tpos, std::vector<float> post_tpos, bool &is_valid, std::vector<float> &tar_pos, std::vector<float> &approach_pos, std::vector<float> &retract_pos)
{
    CheckAproachRetractValidReq request;
    for (int i = 0; i < tpos.size(); i++)
    {
        request.add_tpos(tpos[i]);
    }
    for (int i = 0; i < init_jpos.size(); i++)
    {
        request.add_init_jpos(init_jpos[i]);
    }
    for (int i = 0; i < pre_tpos.size(); i++)
    {
        request.add_pre_tpos(pre_tpos[i]);
    }
    for (int i = 0; i < post_tpos.size(); i++)
    {
        request.add_post_tpos(post_tpos[i]);
    }

    ClientContext context;
    CheckAproachRetractValidRes response;

    Status resultStatus = stub_->CheckAproachRetractValid(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "CheckAproachRetractValid failed." << std::endl;
    }
    else{
        is_valid = response.is_valid();
        for (int i = 0; i < response.tar_pos_size(); i++)
        {
            tar_pos.push_back(response.tar_pos(i));
        }
        for (int i = 0; i < response.approach_pos_size(); i++)
        {
            approach_pos.push_back(response.approach_pos(i));
        }
        for (int i = 0; i < response.retract_pos_size(); i++)
        {
            retract_pos.push_back(response.retract_pos(i));
        }
        return;
    }
}

void IndyClient3::GetPalletPointList(std::vector<float> tpos, std::vector<float> jpos, std::vector<float> pre_tpos, std::vector<float> post_tpos, int32_t pallet_pattern, int32_t width, int32_t height, std::vector<PalletPoint> &pallet_points)
{
    GetPalletPointListReq request;
    for (int i = 0; i < tpos.size(); i++)
    {
        request.add_tpos(tpos[i]);
    }
    for (int i = 0; i < jpos.size(); i++)
    {
        request.add_jpos(jpos[i]);
    }
    for (int i = 0; i < pre_tpos.size(); i++)
    {
        request.add_pre_tpos(pre_tpos[i]);
    }
    for (int i = 0; i < post_tpos.size(); i++)
    {
        request.add_post_tpos(post_tpos[i]);
    }
    request.set_pallet_pattern(pallet_pattern);
    request.set_width(width);
    request.set_height(height);

    ClientContext context;
    GetPalletPointListRes response;

    Status resultStatus = stub_->GetPalletPointList(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetPalletPointList failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.pallet_points_size(); i++)
        {
            pallet_points.push_back(response.pallet_points(i));
        }
        return;
    }
}

void IndyClient3::CalculateRelativePose(std::vector<float> start_pos, std::vector<float> end_pos, TaskBaseType base_type, std::vector<float> &relative_pos)
{
    CalculateRelativePoseReq request;
    for (int i = 0; i < start_pos.size(); i++)
    {
        request.add_start_pos(start_pos[i]);
    }
    for (int i = 0; i < end_pos.size(); i++)
    {
        request.add_end_pos(end_pos[i]);
    }
    request.set_base_type(base_type);

    ClientContext context;
    CalculateRelativePoseRes response;

    Status resultStatus = stub_->CalculateRelativePose(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "CalculateRelativePose failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.relative_pos_size(); i++)
        {
            relative_pos.push_back(response.relative_pos(i));
        }
        return;
    }
}

void IndyClient3::CalculateCurrentPoseRel(std::vector<float> current_pos, std::vector<float> relative_pos, TaskBaseType base_type, std::vector<float> &calculated_pos)
{
    CalculateCurrentPoseRelReq request;
    for (int i = 0; i < current_pos.size(); i++)
    {
        request.add_current_pos(current_pos[i]);
    }
    for (int i = 0; i < relative_pos.size(); i++)
    {
        request.add_relative_pos(relative_pos[i]);
    }
    request.set_base_type(base_type);

    ClientContext context;
    CalculateCurrentPoseRelRes response;

    Status resultStatus = stub_->CalculateCurrentPoseRel(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "CalculateCurrentPoseRel failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.calculated_pos_size(); i++)
        {
            calculated_pos.push_back(response.calculated_pos(i));
        }
        return;
    }
}

void IndyClient3::GetDI(std::vector<bool> &di)
{
    GetDIReq request;

    ClientContext context;
    GetDIRes response;

    Status resultStatus = stub_->GetDI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetDI failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.di_size(); i++)
        {
            di.push_back(response.di(i));
        }
        return;
    }
}

void IndyClient3::SetDI(std::vector<DigitalSignal> di_list)
{
    SetDIReq request;
    for (int i = 0; i < di_list.size(); i++)
    {
        DigitalSignal* di_list_pt = request.add_di_list();
        di_list_pt->set_addr(di_list[i].addr());
        di_list_pt->set_on(di_list[i].on());
    }

    ClientContext context;
    SetDIRes response;

    Status resultStatus = stub_->SetDI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetDI failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetDO(std::vector<DigitalSignal> do_list)
{
    SetDOReq request;
    for (int i = 0; i < do_list.size(); i++)
    {
        DigitalSignal* do_list_pt = request.add_do_list();
        do_list_pt->set_addr(do_list[i].addr());
        do_list_pt->set_on(do_list[i].on());
    }

    ClientContext context;
    SetDORes response;

    Status resultStatus = stub_->SetDO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetDO failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetDO(std::vector<bool> &do_)
{
    GetDOReq request;

    ClientContext context;
    GetDORes response;

    Status resultStatus = stub_->GetDO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetDO failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.do__size(); i++)
        {
            do_.push_back(response.do_(i));
        }
        return;
    }
}

void IndyClient3::GetAI(std::vector<int32_t> &ai)
{
    GetAIReq request;

    ClientContext context;
    GetAIRes response;

    Status resultStatus = stub_->GetAI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetAI failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.ai_size(); i++)
        {
            ai.push_back(response.ai(i));
        }
        return;
    }
}

void IndyClient3::SetAI(std::vector<AnalogSignal> ai_list)
{
    SetAIReq request;
    for (int i = 0; i < ai_list.size(); i++)
    {
        AnalogSignal* ai_list_pt = request.add_ai_list();
        ai_list_pt->set_addr(ai_list[i].addr());
        ai_list_pt->set_voltage(ai_list[i].voltage());
    }

    ClientContext context;
    SetAIRes response;

    Status resultStatus = stub_->SetAI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetAI failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetAO(std::vector<int32_t> &ao)
{
    GetAOReq request;

    ClientContext context;
    GetAORes response;

    Status resultStatus = stub_->GetAO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetAO failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.ao_size(); i++)
        {
            ao.push_back(response.ao(i));
        }
        return;
    }
}

void IndyClient3::SetAO(std::vector<AnalogSignal> ao_list)
{
    SetAOReq request;
    for (int i = 0; i < ao_list.size(); i++)
    {
        AnalogSignal* ao_list_pt = request.add_ao_list();
        ao_list_pt->set_addr(ao_list[i].addr());
        ao_list_pt->set_voltage(ao_list[i].voltage());
    }

    ClientContext context;
    SetAORes response;

    Status resultStatus = stub_->SetAO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetAO failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetEndDI(std::vector<bool> &di)
{
    GetEndDIReq request;

    ClientContext context;
    GetEndDIRes response;

    Status resultStatus = stub_->GetEndDI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetEndDI failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.di_size(); i++)
        {
            di.push_back(response.di(i));
        }
        return;
    }
}

void IndyClient3::SetEndDI(std::vector<DigitalSignal> di_list)
{
    SetEndDIReq request;
    for (int i = 0; i < di_list.size(); i++)
    {
        DigitalSignal* di_list_pt = request.add_di_list();
        di_list_pt->set_addr(di_list[i].addr());
        di_list_pt->set_on(di_list[i].on());
    }

    ClientContext context;
    SetEndDIRes response;

    Status resultStatus = stub_->SetEndDI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetEndDI failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetEndDO(std::vector<DigitalSignal> do_list)
{
    SetEndDOReq request;
    for (int i = 0; i < do_list.size(); i++)
    {
        DigitalSignal* do_list_pt = request.add_do_list();
        do_list_pt->set_addr(do_list[i].addr());
        do_list_pt->set_on(do_list[i].on());
    }

    ClientContext context;
    SetEndDORes response;

    Status resultStatus = stub_->SetEndDO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetEndDO failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetEndDO(std::vector<bool> &do_)
{
    GetEndDOReq request;

    ClientContext context;
    GetEndDORes response;

    Status resultStatus = stub_->GetEndDO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetEndDO failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.do__size(); i++)
        {
            do_.push_back(response.do_(i));
        }
        return;
    }
}

void IndyClient3::GetEndAI(std::vector<int32_t> &ai)
{
    GetEndAIReq request;

    ClientContext context;
    GetEndAIRes response;

    Status resultStatus = stub_->GetEndAI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetEndAI failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.ai_size(); i++)
        {
            ai.push_back(response.ai(i));
        }
        return;
    }
}

void IndyClient3::SetEndAI(std::vector<AnalogSignal> ai_list)
{
    SetEndAIReq request;
    for (int i = 0; i < ai_list.size(); i++)
    {
        AnalogSignal* ai_list_pt = request.add_ai_list();
        ai_list_pt->set_addr(ai_list[i].addr());
        ai_list_pt->set_voltage(ai_list[i].voltage());
    }

    ClientContext context;
    SetEndAIRes response;

    Status resultStatus = stub_->SetEndAI(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetEndAI failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetEndAO(std::vector<int32_t> &ao)
{
    GetEndAOReq request;

    ClientContext context;
    GetEndAORes response;

    Status resultStatus = stub_->GetEndAO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetEndAO failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.ao_size(); i++)
        {
            ao.push_back(response.ao(i));
        }
        return;
    }
}

void IndyClient3::SetEndAO(std::vector<AnalogSignal> ao_list)
{
    SetEndAOReq request;
    for (int i = 0; i < ao_list.size(); i++)
    {
        AnalogSignal* ao_list_pt = request.add_ao_list();
        ao_list_pt->set_addr(ao_list[i].addr());
        ao_list_pt->set_voltage(ao_list[i].voltage());
    }

    ClientContext context;
    SetEndAORes response;

    Status resultStatus = stub_->SetEndAO(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetEndAO failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::SetJointControlGain(std::vector<float> kp, std::vector<float> kv, std::vector<float> kl2)
{
    SetJointControlGainReq request;
    for (int i = 0; i < kp.size(); i++)
    {
        request.add_kp(kp[i]);
    }
    for (int i = 0; i < kv.size(); i++)
    {
        request.add_kv(kv[i]);
    }
    for (int i = 0; i < kl2.size(); i++)
    {
        request.add_kl2(kl2[i]);
    }

    ClientContext context;
    SetJointControlGainRes response;

    Status resultStatus = stub_->SetJointControlGain(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetJointControlGain failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetJointControlGain(std::vector<float> &kp, std::vector<float> &kv, std::vector<float> &kl2)
{
    GetJointControlGainReq request;

    ClientContext context;
    GetJointControlGainRes response;

    Status resultStatus = stub_->GetJointControlGain(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetJointControlGain failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.kp_size(); i++)
        {
            kp.push_back(response.kp(i));
        }
        for (int i = 0; i < response.kv_size(); i++)
        {
            kv.push_back(response.kv(i));
        }
        for (int i = 0; i < response.kl2_size(); i++)
        {
            kl2.push_back(response.kl2(i));
        }
        return;
    }
}

void IndyClient3::SetTaskControlGain(std::vector<float> kp, std::vector<float> kv, std::vector<float> kl2, std::string msg)
{
    SetTaskControlGainReq request;
    for (int i = 0; i < kp.size(); i++)
    {
        request.add_kp(kp[i]);
    }
    for (int i = 0; i < kv.size(); i++)
    {
        request.add_kv(kv[i]);
    }
    for (int i = 0; i < kl2.size(); i++)
    {
        request.add_kl2(kl2[i]);
    }
    request.set_msg(msg);

    ClientContext context;
    SetTaskControlGainRes response;

    Status resultStatus = stub_->SetTaskControlGain(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetTaskControlGain failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetTaskControlGain(std::vector<float> &kp, std::vector<float> &kv, std::vector<float> &kl2)
{
    GetTaskControlGainReq request;

    ClientContext context;
    GetTaskControlGainRes response;

    Status resultStatus = stub_->GetTaskControlGain(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetTaskControlGain failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.kp_size(); i++)
        {
            kp.push_back(response.kp(i));
        }
        for (int i = 0; i < response.kv_size(); i++)
        {
            kv.push_back(response.kv(i));
        }
        for (int i = 0; i < response.kl2_size(); i++)
        {
            kl2.push_back(response.kl2(i));
        }
        return;
    }
}

void IndyClient3::SetImpedanceControlGain(std::vector<float> mass, std::vector<float> damping, std::vector<float> stiffness, std::vector<float> kl2, std::string msg)
{
    SetImpedanceControlGainReq request;
    for (int i = 0; i < mass.size(); i++)
    {
        request.add_mass(mass[i]);
    }
    for (int i = 0; i < damping.size(); i++)
    {
        request.add_damping(damping[i]);
    }
    for (int i = 0; i < stiffness.size(); i++)
    {
        request.add_stiffness(stiffness[i]);
    }
    for (int i = 0; i < kl2.size(); i++)
    {
        request.add_kl2(kl2[i]);
    }
    request.set_msg(msg);

    ClientContext context;
    SetImpedanceControlGainRes response;

    Status resultStatus = stub_->SetImpedanceControlGain(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetImpedanceControlGain failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetImpedanceControlGain(std::vector<float> &mass, std::vector<float> &damping, std::vector<float> &stiffness, std::vector<float> &kl2)
{
    GetImpedanceControlGainReq request;

    ClientContext context;
    GetImpedanceControlGainRes response;

    Status resultStatus = stub_->GetImpedanceControlGain(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetImpedanceControlGain failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.mass_size(); i++)
        {
            mass.push_back(response.mass(i));
        }
        for (int i = 0; i < response.damping_size(); i++)
        {
            damping.push_back(response.damping(i));
        }
        for (int i = 0; i < response.stiffness_size(); i++)
        {
            stiffness.push_back(response.stiffness(i));
        }
        for (int i = 0; i < response.kl2_size(); i++)
        {
            kl2.push_back(response.kl2(i));
        }
        return;
    }
}

void IndyClient3::SetFricComp(bool control_comp, std::vector<int32_t> control_comp_levels, bool dt_comp, std::vector<int32_t> dt_comp_levels, int32_t id_joint)
{
    SetFricCompReq request;
    request.set_control_comp(control_comp);
    for (int i = 0; i < control_comp_levels.size(); i++)
    {
        request.add_control_comp_levels(control_comp_levels[i]);
    }
    request.set_dt_comp(dt_comp);
    for (int i = 0; i < dt_comp_levels.size(); i++)
    {
        request.add_dt_comp_levels(dt_comp_levels[i]);
    }
    request.set_id_joint(id_joint);

    ClientContext context;
    SetFricCompRes response;

    Status resultStatus = stub_->SetFricComp(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetFricComp failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetFricComp(bool &control_comp, std::vector<int32_t> &control_comp_levels, bool &dt_comp, std::vector<int32_t> &dt_comp_levels)
{
    GetFricCompReq request;

    ClientContext context;
    GetFricCompRes response;

    Status resultStatus = stub_->GetFricComp(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetFricComp failed." << std::endl;
    }
    else{
        control_comp = response.control_comp();
        for (int i = 0; i < response.control_comp_levels_size(); i++)
        {
            control_comp_levels.push_back(response.control_comp_levels(i));
        }
        dt_comp = response.dt_comp();
        for (int i = 0; i < response.dt_comp_levels_size(); i++)
        {
            dt_comp_levels.push_back(response.dt_comp_levels(i));
        }
        return;
    }
}

void IndyClient3::SetMountPos(float ry, float rz)
{
    SetMountPosReq request;
    request.set_ry(ry);
    request.set_rz(rz);

    ClientContext context;
    SetMountPosRes response;

    Status resultStatus = stub_->SetMountPos(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetMountPos failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetMountPos(float &ry, float &rz)
{
    GetMountPosReq request;

    ClientContext context;
    GetMountPosRes response;

    Status resultStatus = stub_->GetMountPos(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetMountPos failed." << std::endl;
    }
    else{
        ry = response.ry();
        rz = response.rz();
        return;
    }
}

void IndyClient3::SetToolProperty(float mass, std::vector<float> center_of_mass, std::vector<float> inertia)
{
    SetToolPropertyReq request;
    request.set_mass(mass);
    for (int i = 0; i < center_of_mass.size(); i++)
    {
        request.add_center_of_mass(center_of_mass[i]);
    }
    for (int i = 0; i < inertia.size(); i++)
    {
        request.add_inertia(inertia[i]);
    }

    ClientContext context;
    SetToolPropertyRes response;

    Status resultStatus = stub_->SetToolProperty(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetToolProperty failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetToolProperty(float &mass, std::vector<float> &center_of_mass, std::vector<float> &inertia)
{
    GetToolPropertyReq request;

    ClientContext context;
    GetToolPropertyRes response;

    Status resultStatus = stub_->GetToolProperty(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetToolProperty failed." << std::endl;
    }
    else{
        mass = response.mass();
        for (int i = 0; i < response.center_of_mass_size(); i++)
        {
            center_of_mass.push_back(response.center_of_mass(i));
        }
        for (int i = 0; i < response.inertia_size(); i++)
        {
            inertia.push_back(response.inertia(i));
        }
        return;
    }
}

void IndyClient3::SetCollSensLevel(int32_t level)
{
    SetCollSensLevelReq request;
    request.set_level(level);

    ClientContext context;
    SetCollSensLevelRes response;

    Status resultStatus = stub_->SetCollSensLevel(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetCollSensLevel failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetCollSensLevel(int32_t &level)
{
    GetCollSensLevelReq request;

    ClientContext context;
    GetCollSensLevelRes response;

    Status resultStatus = stub_->GetCollSensLevel(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetCollSensLevel failed." << std::endl;
    }
    else{
        level = response.level();
        return;
    }
}

void IndyClient3::SetCollSensParam(std::vector<float> j_torque_bases, std::vector<float> j_torque_tangents, std::vector<float> t_torque_bases, std::vector<float> t_torque_tangents, std::vector<float> error_bases, std::vector<float> error_tangents, std::vector<float> t_constvel_torque_bases, std::vector<float> t_constvel_torque_tangents, std::vector<float> t_conveyor_torque_bases, std::vector<float> t_conveyor_torque_tangents)
{
    SetCollSensParamReq request;
    for (int i = 0; i < j_torque_bases.size(); i++)
    {
        request.add_j_torque_bases(j_torque_bases[i]);
    }
    for (int i = 0; i < j_torque_tangents.size(); i++)
    {
        request.add_j_torque_tangents(j_torque_tangents[i]);
    }
    for (int i = 0; i < t_torque_bases.size(); i++)
    {
        request.add_t_torque_bases(t_torque_bases[i]);
    }
    for (int i = 0; i < t_torque_tangents.size(); i++)
    {
        request.add_t_torque_tangents(t_torque_tangents[i]);
    }
    for (int i = 0; i < error_bases.size(); i++)
    {
        request.add_error_bases(error_bases[i]);
    }
    for (int i = 0; i < error_tangents.size(); i++)
    {
        request.add_error_tangents(error_tangents[i]);
    }
    for (int i = 0; i < t_constvel_torque_bases.size(); i++)
    {
        request.add_t_constvel_torque_bases(t_constvel_torque_bases[i]);
    }
    for (int i = 0; i < t_constvel_torque_tangents.size(); i++)
    {
        request.add_t_constvel_torque_tangents(t_constvel_torque_tangents[i]);
    }
    for (int i = 0; i < t_conveyor_torque_bases.size(); i++)
    {
        request.add_t_conveyor_torque_bases(t_conveyor_torque_bases[i]);
    }
    for (int i = 0; i < t_conveyor_torque_tangents.size(); i++)
    {
        request.add_t_conveyor_torque_tangents(t_conveyor_torque_tangents[i]);
    }

    ClientContext context;
    SetCollSensParamRes response;

    Status resultStatus = stub_->SetCollSensParam(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetCollSensParam failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetCollSensParam(std::vector<float> &j_torque_bases, std::vector<float> &j_torque_tangents, std::vector<float> &t_torque_bases, std::vector<float> &t_torque_tangents, std::vector<float> &error_bases, std::vector<float> &error_tangents, std::vector<float> &t_constvel_torque_bases, std::vector<float> &t_constvel_torque_tangents, std::vector<float> &t_conveyor_torque_bases, std::vector<float> &t_conveyor_torque_tangents)
{
    GetCollSensParamReq request;

    ClientContext context;
    GetCollSensParamRes response;

    Status resultStatus = stub_->GetCollSensParam(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetCollSensParam failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.j_torque_bases_size(); i++)
        {
            j_torque_bases.push_back(response.j_torque_bases(i));
        }
        for (int i = 0; i < response.j_torque_tangents_size(); i++)
        {
            j_torque_tangents.push_back(response.j_torque_tangents(i));
        }
        for (int i = 0; i < response.t_torque_bases_size(); i++)
        {
            t_torque_bases.push_back(response.t_torque_bases(i));
        }
        for (int i = 0; i < response.t_torque_tangents_size(); i++)
        {
            t_torque_tangents.push_back(response.t_torque_tangents(i));
        }
        for (int i = 0; i < response.error_bases_size(); i++)
        {
            error_bases.push_back(response.error_bases(i));
        }
        for (int i = 0; i < response.error_tangents_size(); i++)
        {
            error_tangents.push_back(response.error_tangents(i));
        }
        for (int i = 0; i < response.t_constvel_torque_bases_size(); i++)
        {
            t_constvel_torque_bases.push_back(response.t_constvel_torque_bases(i));
        }
        for (int i = 0; i < response.t_constvel_torque_tangents_size(); i++)
        {
            t_constvel_torque_tangents.push_back(response.t_constvel_torque_tangents(i));
        }
        for (int i = 0; i < response.t_conveyor_torque_bases_size(); i++)
        {
            t_conveyor_torque_bases.push_back(response.t_conveyor_torque_bases(i));
        }
        for (int i = 0; i < response.t_conveyor_torque_tangents_size(); i++)
        {
            t_conveyor_torque_tangents.push_back(response.t_conveyor_torque_tangents(i));
        }
        return;
    }
}

void IndyClient3::SetCollPolicy(CollisionPolicy policy, float sleep_time, float gravity_time)
{
    SetCollPolicyReq request;
    request.set_policy(policy);
    request.set_sleep_time(sleep_time);
    request.set_gravity_time(gravity_time);

    ClientContext context;
    SetCollPolicyRes response;

    Status resultStatus = stub_->SetCollPolicy(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetCollPolicy failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetCollPolicy(CollisionPolicy &policy, float &sleep_time, float &gravity_time)
{
    GetCollPolicyReq request;

    ClientContext context;
    GetCollPolicyRes response;

    Status resultStatus = stub_->GetCollPolicy(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetCollPolicy failed." << std::endl;
    }
    else{
        policy = response.policy();
        sleep_time = response.sleep_time();
        gravity_time = response.gravity_time();
        return;
    }
}

void IndyClient3::GetCollTuningParam(std::vector<float> &j_torque_bases, std::vector<float> &j_torque_tangents, std::vector<float> &t_torque_bases, std::vector<float> &t_torque_tangents, std::vector<float> &error_bases, std::vector<float> &error_tangents, std::vector<float> &t_constvel_torque_bases, std::vector<float> &t_constvel_torque_tangents, std::vector<float> &t_conveyor_torque_bases, std::vector<float> &t_conveyor_torque_tangents, bool &is_calc_done)
{
    GetCollTuningParamReq request;

    ClientContext context;
    GetCollTuningParamRes response;

    Status resultStatus = stub_->GetCollTuningParam(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetCollTuningParam failed." << std::endl;
    }
    else{
        for (int i = 0; i < response.j_torque_bases_size(); i++)
        {
            j_torque_bases.push_back(response.j_torque_bases(i));
        }
        for (int i = 0; i < response.j_torque_tangents_size(); i++)
        {
            j_torque_tangents.push_back(response.j_torque_tangents(i));
        }
        for (int i = 0; i < response.t_torque_bases_size(); i++)
        {
            t_torque_bases.push_back(response.t_torque_bases(i));
        }
        for (int i = 0; i < response.t_torque_tangents_size(); i++)
        {
            t_torque_tangents.push_back(response.t_torque_tangents(i));
        }
        for (int i = 0; i < response.error_bases_size(); i++)
        {
            error_bases.push_back(response.error_bases(i));
        }
        for (int i = 0; i < response.error_tangents_size(); i++)
        {
            error_tangents.push_back(response.error_tangents(i));
        }
        for (int i = 0; i < response.t_constvel_torque_bases_size(); i++)
        {
            t_constvel_torque_bases.push_back(response.t_constvel_torque_bases(i));
        }
        for (int i = 0; i < response.t_constvel_torque_tangents_size(); i++)
        {
            t_constvel_torque_tangents.push_back(response.t_constvel_torque_tangents(i));
        }
        for (int i = 0; i < response.t_conveyor_torque_bases_size(); i++)
        {
            t_conveyor_torque_bases.push_back(response.t_conveyor_torque_bases(i));
        }
        for (int i = 0; i < response.t_conveyor_torque_tangents_size(); i++)
        {
            t_conveyor_torque_tangents.push_back(response.t_conveyor_torque_tangents(i));
        }
        is_calc_done = response.is_calc_done();
        return;
    }
}

void IndyClient3::GetSafetyLimitConfig(float &power_limit, float &power_limit_ratio, float &tcp_force_limit, float &tcp_force_limit_ratio, float &tcp_speed_limit, float &tcp_speed_limit_ratio, std::vector<float> &joint_limits)
{
    GetSafetyLimitConfigReq request;

    ClientContext context;
    GetSafetyLimitConfigRes response;

    Status resultStatus = stub_->GetSafetyLimitConfig(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetSafetyLimitConfig failed." << std::endl;
    }
    else{
        power_limit = response.power_limit();
        power_limit_ratio = response.power_limit_ratio();
        tcp_force_limit = response.tcp_force_limit();
        tcp_force_limit_ratio = response.tcp_force_limit_ratio();
        tcp_speed_limit = response.tcp_speed_limit();
        tcp_speed_limit_ratio = response.tcp_speed_limit_ratio();
        for (int i = 0; i < response.joint_limits_size(); i++)
        {
            joint_limits.push_back(response.joint_limits(i));
        }
        return;
    }
}

void IndyClient3::SetSafetyLimitConfig(float power_limit, float power_limit_ratio, float tcp_force_limit, float tcp_force_limit_ratio, float tcp_speed_limit, float tcp_speed_limit_ratio, std::vector<float> joint_limits)
{
    SetSafetyLimitConfigReq request;
    request.set_power_limit(power_limit);
    request.set_power_limit_ratio(power_limit_ratio);
    request.set_tcp_force_limit(tcp_force_limit);
    request.set_tcp_force_limit_ratio(tcp_force_limit_ratio);
    request.set_tcp_speed_limit(tcp_speed_limit);
    request.set_tcp_speed_limit_ratio(tcp_speed_limit_ratio);
    for (int i = 0; i < joint_limits.size(); i++)
    {
        request.add_joint_limits(joint_limits[i]);
    }

    ClientContext context;
    SetSafetyLimitConfigRes response;

    Status resultStatus = stub_->SetSafetyLimitConfig(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetSafetyLimitConfig failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetSafetyStopConfig(StopCategory &joint_position_limit_stop_cat, StopCategory &joint_speed_limit_stop_cat, StopCategory &joint_torque_limit_stop_cat, StopCategory &tcp_speed_limit_stop_cat, StopCategory &tcp_force_limit_stop_cat, StopCategory &power_limit_stop_cat)
{
    GetSafetyStopConfigReq request;

    ClientContext context;
    GetSafetyStopConfigRes response;

    Status resultStatus = stub_->GetSafetyStopConfig(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetSafetyStopConfig failed." << std::endl;
    }
    else{
        joint_position_limit_stop_cat = response.joint_position_limit_stop_cat();
        joint_speed_limit_stop_cat = response.joint_speed_limit_stop_cat();
        joint_torque_limit_stop_cat = response.joint_torque_limit_stop_cat();
        tcp_speed_limit_stop_cat = response.tcp_speed_limit_stop_cat();
        tcp_force_limit_stop_cat = response.tcp_force_limit_stop_cat();
        power_limit_stop_cat = response.power_limit_stop_cat();
        return;
    }
}

void IndyClient3::SetSafetyStopConfig(StopCategory joint_position_limit_stop_cat, StopCategory joint_speed_limit_stop_cat, StopCategory joint_torque_limit_stop_cat, StopCategory tcp_speed_limit_stop_cat, StopCategory tcp_force_limit_stop_cat, StopCategory power_limit_stop_cat)
{
    SetSafetyStopConfigReq request;
    request.set_joint_position_limit_stop_cat(joint_position_limit_stop_cat);
    request.set_joint_speed_limit_stop_cat(joint_speed_limit_stop_cat);
    request.set_joint_torque_limit_stop_cat(joint_torque_limit_stop_cat);
    request.set_tcp_speed_limit_stop_cat(tcp_speed_limit_stop_cat);
    request.set_tcp_force_limit_stop_cat(tcp_force_limit_stop_cat);
    request.set_power_limit_stop_cat(power_limit_stop_cat);

    ClientContext context;
    SetSafetyStopConfigRes response;

    Status resultStatus = stub_->SetSafetyStopConfig(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "SetSafetyStopConfig failed." << std::endl;
    }
    else{
        return;
    }
}

void IndyClient3::GetEL5001(int32_t &status, int32_t &value, int32_t &delta, float &average)
{
    GetEL5001Req request;

    ClientContext context;
    GetEL5001Res response;

    Status resultStatus = stub_->GetEL5001(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetEL5001 failed." << std::endl;
    }
    else{
        status = response.status();
        value = response.value();
        delta = response.delta();
        average = response.average();
        return;
    }
}

void IndyClient3::GetEL5101(int32_t &status, int32_t &value, int32_t &latch, int32_t &delta, float &average)
{
    GetEL5101Req request;

    ClientContext context;
    GetEL5101Res response;

    Status resultStatus = stub_->GetEL5101(&context, request, &response);
    if (!resultStatus.ok()) {
        std::cout << "GetEL5101 failed." << std::endl;
    }
    else{
        status = response.status();
        value = response.value();
        latch = response.latch();
        delta = response.delta();
        average = response.average();
        return;
    }
}