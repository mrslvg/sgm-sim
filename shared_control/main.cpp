#include <iostream>
#include <queue>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <array>
#include <time.h>
#include <fstream>
#include <ctime>

// VINE ROBOT
#include "vine_robot.h"

// INPUT DEVICE
#include "ncurses.h"
#include "HapticDevice.h"

// COPPELIA ROBOTICS
#include "extApi.h"
#include "extApiPlatform.h"
#include "CoppeliaWrapper.h"

// SHARED AUTONOMY
#include "SmartAssFS.h"
#include "GoalSelection.h"

// TELEOP
#include "TeleopCtrl.h"

// UTILS
#include "utils.h"
#define BILLION 1e9

// FIXED ASSISTANCE
template<typename T, typename Container = std::deque<T>>
class iterable_queue : public std::queue<T, Container>
{
public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    iterator begin() { return this->c.begin(); }
    iterator end() { return this->c.end(); }
    const_iterator begin() const { return this->c.begin(); }
    const_iterator end() const { return this->c.end(); }
};

iterable_queue<double> qForce;

void initFilter(int f = 50)
{
    for (int i = 0; i < f; i++)
        qForce.push(0.0);
}

double filterForce(double value, int f = 50)
{
    qForce.push(value);
    if (qForce.size() > f)
        qForce.pop();
    double result = 0.0;
    for (auto it = qForce.begin(); it != qForce.end(); ++it)
        result += *it;
    result /= qForce.size();
    return result;
}

double runFixedAssistance(double robotEE_x, double robotEE_y, double robotEE_z, double currentGoal_x, double currentGoal_y, double currentGoal_z)
{
    double a = (currentGoal_x - robotEE_x);
    double b = (currentGoal_y - robotEE_y);
    double c = (currentGoal_z - robotEE_z);
    double d = sqrt((a * a) + (b * b) + (c * c));		//Distance End Effector - Current Goal
    double k = 10.0 / 1000.0;							//Stiffness fixed at 10N/mm - valid for 100Hz refresh
    if (d < 30.0)
        k = 0.0;										//If distance is below 30mm put stiffness = 0
    double f = min(d * k, 7.0);							//Max force = 7N
    f = filterForce(f, 50);								//Filter Over 50 samples
    return f;
}

int main(int argc, char** argv)
{
    // PARAMETERS
    bool use_haptic_device;
    float hd_deadzone;
    float hd_rate_gain;
    float hd_pos_gain;
    float kb_rate_gain;

    float cycle_time;              // milliseconds
    float deltaStiffnessPerSecond; // N/m
    int reactTimeCloser_seconds;   // s --> I used this just to express them in seconds,
    int reactTimeAway_seconds;     // s --> but we can actually use the iteration value
    int reactTimeSteady_seconds;   // s --> once we agree on the sample rate.
    float slopeTh;                 // this should work when running at 6.6Hz!
    float nearbyTh;                // millimeters
    float maxStiff;                // N/m
    float maxForce;                // N

    bool debug;
    bool save_data;

    bool teleoperation;
    bool targeting_assistance;
    bool targeting_fixed_assistance;
    bool auto_steering_manual_eversion;
    bool manual_steering_auto_eversion;
    bool full_autonomy;

    float sim_time = 0.0;

    read_params("./../src/config/params.txt",
                use_haptic_device,
                hd_deadzone,
                hd_rate_gain,
                hd_pos_gain,
                kb_rate_gain,
                cycle_time,
                deltaStiffnessPerSecond,
                reactTimeCloser_seconds,
                reactTimeAway_seconds,
                reactTimeSteady_seconds,
                slopeTh,
                nearbyTh,
                maxStiff,
                maxForce,
                teleoperation,
                targeting_assistance,
                targeting_fixed_assistance,
                auto_steering_manual_eversion,
                manual_steering_auto_eversion,
                full_autonomy,
                debug,
                save_data);

    float sampleRate_ms = cycle_time*1000;
    int reactTimeCloser = reactTimeCloser_seconds * 1000 / (int)sampleRate_ms;
    int reactTimeAway = reactTimeAway_seconds * 1000 / (int)sampleRate_ms;
    int reactTimeSteady = reactTimeSteady_seconds * 1000 / (int)sampleRate_ms;
    float deltaStiffness = (deltaStiffnessPerSecond / 1000.0f) / (1000.0f/ sampleRate_ms);

    std::ofstream file;

    if(save_data)
    {
        int rep_n;
        std::cout << "Enter repetition number: ";
        std::cin >> rep_n;

        int cond_n;
        std::cout << "Enter condition number: ";
        std::cin >> cond_n;

//        time_t rawtime;
//        struct tm * timeinfo;
//        char buffer[80];
//        time(&rawtime);
//        timeinfo = localtime(&rawtime);
//        strftime(buffer,sizeof(buffer),"%d-%m-%Y-%H:%M:%S",timeinfo);
//        std::string time_str(buffer);

        std::string cond_str = to_string(int(cond_n));
        std::string rep_str = to_string(int(rep_n));

        std::string file_name;
        if(teleoperation)
            file_name = rep_str + "/teleoperation_cond" + cond_str + ".txt";
        else if(targeting_assistance)
            file_name = rep_str + "/targeting_assistance_cond" + cond_str + ".txt";
        else if(targeting_fixed_assistance)
            file_name = rep_str + "/targeting_fixed_assistance_cond" + cond_str + ".txt";
        else if(auto_steering_manual_eversion)
            file_name = rep_str + "/auto_steering_manual_eversion_cond" + cond_str + ".txt";
        else if(manual_steering_auto_eversion)
            file_name = rep_str + "/manual_steering_auto_eversion_cond" + cond_str + ".txt";
        else if(full_autonomy)
            file_name = rep_str + "/full_autonomy_cond" + cond_str + ".txt";

        file = std::ofstream(file_name);
        file << "t eex eey eez "
                "hpx hpy hpz "
                "hfx hfy hfz "
                "obj1x obj1y obj1z "
                "obj2x obj2y obj2z "
                "tar1x tar1y tar1z "
                "tar2x tar2y tar2z "
                "obj_grip "
                "goalx goaly goalz "
                "\n";
    }

    // HAPTIC DEVICE
    HapticDevice falcon;
    double hd_pos[3] = {0,0,0}, hd_vel[3] = {0,0,0};
    std::array<double,3> hd_gui;
    unsigned int button = 0;

    if(use_haptic_device)
    {
        if(falcon.initialize() == 0)
        {
            printf("Haptic device initialised\n");
        }
        else
        {
            printf("Impossible to initialise haptic device\n");
            return 0;
        }
        falcon.update();
        falcon.getDevicePosition(hd_pos, hd_deadzone);
        falcon.getDeviceVelocity(hd_vel);
    }

    // KEYBOARD FUNCTIONS
    //initscr(); cbreak(); noecho(); scrollok(stdscr, TRUE); nodelay(stdscr, TRUE);

    // CONNECT TO COPPELIA
    CoppeliaWrapper coppelia;
    coppelia.connect(true);

    // RETRIEVE HANDLES
    simxInt item1_handle    = coppelia.getObjectHandle("item_1");
    simxInt item2_handle    = coppelia.getObjectHandle("item_2");
    simxInt target1_handle  = coppelia.getObjectHandle("target_1");
    simxInt target2_handle  = coppelia.getObjectHandle("target_2");
    simxInt rb_handle       = coppelia.getObjectHandle("rb_frame");
    simxInt ee_handle       = coppelia.getObjectHandle("end_effector");
    simxInt h_proxy_handle  = coppelia.getObjectHandle("h_proxy");

    // RETRIEVE POSITIONS
    float w_p_rb[3], rb_p_item1[3], rb_p_item2[3], rb_p_target1[3], rb_p_target2[3], w_p_ee[3], rb_p_ee[3];
    coppelia.getObjectPosition(rb_handle, -1, w_p_rb);
    coppelia.getObjectPosition(ee_handle, -1, w_p_ee);
    coppelia.getObjectPosition(ee_handle, rb_handle, rb_p_ee);
    coppelia.getObjectPosition(item1_handle, rb_handle, rb_p_item1);
    coppelia.getObjectPosition(item2_handle, rb_handle, rb_p_item2);
    coppelia.getObjectPosition(target1_handle, rb_handle, rb_p_target1);
    coppelia.getObjectPosition(target2_handle, rb_handle, rb_p_target2);

    float pos_item1[3] ={0,0,0}, pos_item2[3]={0,0,0};
    simxGetObjectPosition(coppelia.clientID(), item1_handle, rb_handle, pos_item1, simx_opmode_streaming);
    simxGetObjectPosition(coppelia.clientID(), item2_handle, rb_handle, pos_item2, simx_opmode_streaming);

    // VINE ROBOT
    Eigen::Matrix<double,4,4> w_T_rb = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix3d w_R_rb;
    w_R_rb = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // TODO: fix this reading from simulator
    w_T_rb.block(0,0,3,3) = w_R_rb;
    w_T_rb.block(0,3,3,1) = Eigen::Vector3d(w_p_rb[0], w_p_rb[1], w_p_rb[2]);
    vine_robot vr(w_p_ee, w_T_rb);
    vr.get_w_p_ee(w_p_ee);
    Eigen::Matrix<double,4,4> w_T_ee_cmd = w_T_rb;


    // INITIALIZE CONTROLLERS
    // Teleop Control
    Eigen::Matrix<double,4,4> rb_T_ee = Eigen::Matrix<double,4,4>::Identity();
    rb_T_ee.block(0,3,3,1) = Eigen::Vector3d(rb_p_ee[0], rb_p_ee[1], rb_p_ee[2]);
    TeleopCtrl teleop(rb_T_ee);
    double cmd_rate[3] = {0,0,0};

    // Goal Selection Algorithm
    float z_offset = 150.0f, bending_z_offset = 0; // TODO: Move to params file
    GoalSelection2 gsAlg = GoalSelection2();
    gsAlg.LogOn();
    gsAlg.Reset(rb_p_item1[0]*1e3, rb_p_item1[1]*1e3, rb_p_item1[2]*1e3,
                rb_p_item2[0]*1e3, rb_p_item2[1]*1e3, rb_p_item2[2]*1e3,
                rb_p_target1[0]*1e3, rb_p_target1[1]*1e3, rb_p_target1[2]*1e3,
                rb_p_target2[0]*1e3, rb_p_target2[1]*1e3, rb_p_target2[2]*1e3);
    gsAlg.SetZOffset(z_offset);
    gsAlg.SetBendingZOffset(bending_z_offset);
    float goal[3] = {0,0,0};

    // Targeting Assistance Algorithm
    SmartAssFS smAss(slopeTh, nearbyTh, reactTimeCloser, reactTimeAway,
                     reactTimeSteady, deltaStiffness, maxStiff/1000.0f, maxForce, false);
    smAss.LogOn();

    char key = getch();
    timespec start, stop, elapsed;

    int object_in_gripper = 0;
    initFilter(50);	//Filter over 50 samples

    // CONTROL LOOP
    while(key != 'q')
    {
        clock_gettime(CLOCK_REALTIME, &start);
        key = getch();
        if(use_haptic_device) // USE HAPTIC DEVICE
        {
            falcon.update();
            falcon.getDevicePosition(hd_pos, hd_deadzone);
            falcon.getDeviceButton(button);
            hd_pos[1] = 1.5*hd_pos[1]; //TO DO: move to params file
        }
        else // KEYBOARD CONTROL
        {
            hd_pos[0] = 0.0; hd_pos[1] = 0.0; hd_pos[2] = 0.0;
            switch (key) {
            case('a'):
                printf("x+\n");
                hd_pos[0] -= 1*kb_rate_gain;
                break;
            case('d'):
                printf("x-\n");
                hd_pos[0] += 1*kb_rate_gain;
                break;
            case('x'):
                printf("z+\n");
                hd_pos[2] += 1*kb_rate_gain;
                break;
            case('z'):
                printf("z-\n");
                hd_pos[2] -= 1*kb_rate_gain;
                break;
            case('w'):
                printf("y+\n");
                hd_pos[1] += 1*kb_rate_gain;
                break;
            case('s'):
                printf("y-\n");
                hd_pos[1] -= 1*kb_rate_gain;
                break;
            case('c'):
                printf("close gripper");
                button = 1;
                break;
            case('o'):
                printf("open gripper");
                button = 0;
                break;
            }
        }

        vr.get_w_p_ee(w_p_ee);
        vr.get_rb_p_ee(rb_p_ee);

        std::cout << w_p_ee[0] << " " << w_p_ee[1] << " " << w_p_ee[2] << " " << std::endl;
        std::cout << rb_p_ee[0] << " " << rb_p_ee[1] << " " << rb_p_ee[2] << " " << std::endl;

        if(teleoperation || targeting_assistance || targeting_fixed_assistance || auto_steering_manual_eversion)
        {
            gsAlg.Run(rb_p_ee[0]*1e3, rb_p_ee[1]*1e3, rb_p_ee[2]*1e3, (button>0 && object_in_gripper>0), false);
        }
        else
        {
            gsAlg.Run(rb_p_ee[0]*1e3, rb_p_ee[1]*1e3, rb_p_ee[2]*1e3, (button>0 && object_in_gripper>0), true);
        }

        gsAlg.GetGoal(goal[0], goal[1], goal[2]);
        // std::cout << "button:" << button << std::endl;

        if(teleoperation) // TELEOPERATION
        {
            cmd_rate[0] = hd_pos[0]; cmd_rate[1] = hd_pos[1]; cmd_rate[2] = hd_pos[2];
            // RATE CONTROL
            //teleop.rateCtrl(cmd_rate, hd_rate_gain, rb_T_ee);

            // POSITION CONTROL
            teleop.posCtrl(hd_pos, hd_pos_gain, rb_T_ee);
            w_T_ee_cmd = w_T_rb*rb_T_ee;
        }
        else if(targeting_assistance) // TARGETING ASSISTANCE
        {
            cmd_rate[0] = hd_pos[0]; cmd_rate[1] = hd_pos[1]; cmd_rate[2] = hd_pos[2];

            float hd_gui_int = smAss.Run_Magnitude(goal[0], goal[1], goal[2], rb_p_ee[0]*1e3, rb_p_ee[1]*1e3, rb_p_ee[2]*1e3);
            if(use_haptic_device)
            {
                Eigen::Vector3f hd_gui_dir(goal[0]*1e-3-rb_p_ee[0], goal[1]*1e-3-rb_p_ee[1], goal[2]*1e-3-rb_p_ee[2]);
                // hd_gui_dir.normalize();
                teleop.hapticGuidance(hd_gui_dir, hd_gui_int, hd_gui);
                falcon.setDeviceForce(hd_gui);
                //            hd_gui[0] = -hd_gui_int*hd_gui_dir[1];
                //            hd_gui[1] = -hd_gui_int*hd_gui_dir[2];
                //            hd_gui[2] = hd_gui_int*hd_gui_dir[0];
            }
            // DRAW FORCE
            float w_p_proxy[3];
            Eigen::Vector3d w_p_goal = w_R_rb*Eigen::Vector3d(goal[0]*1e-3, goal[1]*1e-3, goal[2]*1e-3) + Eigen::Vector3d(w_p_rb[0], w_p_rb[1], w_p_rb[2]);

//            std::cout << "w_p_goal: " << w_p_goal[0] << " " << w_p_goal[1] << " " << w_p_goal[2] << " " << std::endl;
//            std::cout << "w_p_rb: " << w_p_rb[0] << " " << w_p_rb[1] << " " << w_p_rb[2] << " " << std::endl;
//            std::cout << "goal: " << goal[0] << " " << goal[1] << " " << goal[2] << " " << std::endl;
//            std::cout << "w_T_ee: " << w_T_ee(0,3) << " " << w_T_ee(1,3) << " " << w_T_ee(2,3) << " " << std::endl;
            // simxInt returnCode = simxGetObjectPosition(coppelia.clientID(), target_handle, -1, w_p_obj, simx_opmode_oneshot);

            for (unsigned int i = 0; i < 3; i++)
                w_p_proxy[i] = w_p_ee[i] + 0.3*hd_gui_int*(w_p_goal[i] - w_p_ee[i]);

            simxSetObjectPosition(coppelia.clientID(), h_proxy_handle, -1, w_p_proxy, simx_opmode_oneshot);
            // RATE CONTROL
            //teleop.rateCtrl(cmd_rate, hd_rate_gain, rb_T_ee);

            // POSITION CONTROL
            teleop.posCtrl(hd_pos, hd_pos_gain, rb_T_ee);
            w_T_ee_cmd = w_T_rb*rb_T_ee;

        }
        else if(targeting_fixed_assistance) // TARGETING FIXED ASSISTANCE
        {
            cmd_rate[0] = hd_pos[0]; cmd_rate[1] = hd_pos[1]; cmd_rate[2] = hd_pos[2];

            double hd_gui_int = runFixedAssistance(rb_p_ee[0]*1e3, rb_p_ee[1]*1e3, rb_p_ee[2]*1e3, goal[0], goal[1], goal[2]);
            if(use_haptic_device)
            {
                Eigen::Vector3f hd_gui_dir(goal[0]*1e-3-rb_p_ee[0], goal[1]*1e-3-rb_p_ee[1], goal[2]*1e-3-rb_p_ee[2]);
                //hd_gui_dir.normalize();
                teleop.hapticGuidance(hd_gui_dir, hd_gui_int, hd_gui);
                falcon.setDeviceForce(hd_gui);

            }
            // DRAW FORCE
            float w_p_proxy[3];
            Eigen::Vector3d w_p_goal = w_R_rb*Eigen::Vector3d(goal[0]*1e-3, goal[1]*1e-3, goal[2]*1e-3) + Eigen::Vector3d(w_p_rb[0], w_p_rb[1], w_p_rb[2]);

            for (unsigned int i = 0; i < 3; i++)
                w_p_proxy[i] = w_p_ee[i] + 0.3*hd_gui_int*(w_p_goal[i] - w_p_ee[i]);

            simxSetObjectPosition(coppelia.clientID(), h_proxy_handle, -1, w_p_proxy, simx_opmode_oneshot);

            // RATE CONTROL
            //teleop.rateCtrl(cmd_rate, hd_rate_gain, rb_T_ee);

            // POSITION CONTROL
            teleop.posCtrl(hd_pos, hd_pos_gain, rb_T_ee);
            w_T_ee_cmd = w_T_rb*rb_T_ee;

        }
        else if(auto_steering_manual_eversion) // AUTO STEERING MANUAL EVERSION
        {
            cmd_rate[0] = -(goal[1]*1e-3-rb_T_ee(1,3)); cmd_rate[1] = hd_pos[1]; cmd_rate[2] = goal[0]*1e-3-rb_T_ee(0,3);
            // RATE CONTROL
            //teleop.rateCtrl(cmd_rate, hd_rate_gain, rb_T_ee);

            // POSITION CONTROL
            teleop.posCtrlASME(cmd_rate, hd_pos_gain, hd_rate_gain, rb_T_ee);
            w_T_ee_cmd = w_T_rb*rb_T_ee;
        }
        else if(manual_steering_auto_eversion) // MANUAL STEERING AUTO EVERSION
        {
            cmd_rate[0] = hd_pos[0]; cmd_rate[1] = -(goal[2]*1e-3-rb_T_ee(2,3)); cmd_rate[2] = hd_pos[2];
            // RATE CONTROL
            //teleop.rateCtrl(cmd_rate, hd_rate_gain, rb_T_ee);

            // POSITION CONTROL
            teleop.posCtrlMSAE(cmd_rate, hd_pos_gain, hd_rate_gain, rb_T_ee);
            w_T_ee_cmd = w_T_rb*rb_T_ee;
        }
        else if(full_autonomy) // FULL AUTONOMY
        {
            cmd_rate[0] = -(goal[1]*1e-3-rb_T_ee(1,3)); cmd_rate[1] = -(goal[2]*1e-3-rb_T_ee(2,3)); cmd_rate[2] = goal[0]*1e-3-rb_T_ee(0,3);
            // RATE CONTROL
            teleop.rateCtrl(cmd_rate, hd_rate_gain, rb_T_ee);
            w_T_ee_cmd = w_T_rb*rb_T_ee;
        }

        // TRANSFORM & SEND COMMANDS
        float w_p_ee_cmd[3] = {(float)w_T_ee_cmd(0,3), (float)w_T_ee_cmd(1,3), (float)w_T_ee_cmd(2,3)};
        vr.control(w_p_ee_cmd, w_p_ee);
        simxSetObjectPosition(coppelia.clientID(), ee_handle, -1, w_p_ee, simx_opmode_oneshot);

        // VINE ROBOT
        //        vr.ik(rb_T_ee, length, angle_x, angle_y);
        //        // rb_ee = vr_ctrl.run(length);
        //        Eigen::Matrix3d Ra;
        //        //Ra = w_R_rb*Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX());
        //        //w_rb.block(0,0,3,3) = Ra;
        //        w_T_ee = w_T_rb*rb_T_ee;
        //        float w_p_ee[3];
        //        w_p_ee[0] = w_T_ee(0,3);
        //        w_p_ee[1] = w_T_ee(1,3);
        //        w_p_ee[2] = w_T_ee(2,3);
        //        Eigen::Matrix<double,3,3> R = w_T_ee.block(0,0,3,3);
        //        Eigen::Matrix<double,3,3> m;
        //        m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
        //                * Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitY())
        //                * Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ());
        //        R = R*m;
        //        Eigen::Matrix<double,3,1> ea = R.eulerAngles(0,1,2);
        //        float ori[3];
        //        ori[0] = ea[0];
        //        ori[1] = ea[1];
        //        ori[2] = ea[2];

        // GRIPPER
        // simxFloat p1, p2;
        // simxGetJointPosition(coppelia.clientID(), grp1_handle, &p1, simx_opmode_oneshot);
        // simxGetJointPosition(coppelia.clientID(), grp2_handle, &p2, simx_opmode_oneshot);

        if (button > 0)
        {
//            if (p1 < p2-0.008)
//            {
//                simxSetJointTargetVelocity(coppelia.clientID(), grp1_handle, -0.02, simx_opmode_oneshot);
//                simxSetJointTargetVelocity(coppelia.clientID(), grp2_handle, -0.08, simx_opmode_oneshot);
//            }
//            else
//            {
//                simxSetJointTargetVelocity(coppelia.clientID(), grp1_handle, -0.08, simx_opmode_oneshot);
//                simxSetJointTargetVelocity(coppelia.clientID(), grp2_handle, -0.08, simx_opmode_oneshot);
//            }

            // coppelia.getObjectPosition(ee_handle, rb_handle, rb_p_ee);
            //float dist1 = gsAlg.EDist(rb_p_item1, rb_p_ee);
            if(object_in_gripper == 0)
            {
                rb_p_ee[0] = rb_T_ee(0,3); rb_p_ee[1] = rb_T_ee(1,3); rb_p_ee[2] = rb_T_ee(2,3);
                if (dist(rb_p_item1, rb_p_ee) < 0.05)
                {
                    simxSetObjectParent(coppelia.clientID(), item1_handle, ee_handle, true, simx_opmode_oneshot);
                    object_in_gripper = 1;
                }
                if (dist(rb_p_item2, rb_p_ee) < 0.05)
                {
                    simxSetObjectParent(coppelia.clientID(), item2_handle, ee_handle, true, simx_opmode_oneshot);
                    object_in_gripper = 2;
                }
            }
        }
        else
        {
//            if (p1 < p2)
//            {
//                simxSetJointTargetVelocity(coppelia.clientID(), grp1_handle, 0.08, simx_opmode_oneshot);
//                simxSetJointTargetVelocity(coppelia.clientID(), grp2_handle, 0.04, simx_opmode_oneshot);
//            }
//            else
//            {
//                simxSetJointTargetVelocity(coppelia.clientID(), grp1_handle, 0.04, simx_opmode_oneshot);
//                simxSetJointTargetVelocity(coppelia.clientID(), grp2_handle, 0.08, simx_opmode_oneshot);
//            }

            if(object_in_gripper != 0)
            {
                if (object_in_gripper == 1)
                {
                    simxSetObjectParent(coppelia.clientID(), item1_handle, -1, true, simx_opmode_oneshot);
                    object_in_gripper = 0;
                }
                if (object_in_gripper == 2)
                {
                    simxSetObjectParent(coppelia.clientID(), item2_handle, -1, true, simx_opmode_oneshot);
                    object_in_gripper = 0;
                }
            }
        }

        if(save_data)
        {
            simxGetObjectPosition(coppelia.clientID(), item1_handle, rb_handle, pos_item1, simx_opmode_oneshot);
            simxGetObjectPosition(coppelia.clientID(), item2_handle, rb_handle, pos_item2, simx_opmode_oneshot);
            // coppelia.getObjectPosition(item1_handle, rb_handle, pos_item1);
            // coppelia.getObjectPosition(item2_handle, rb_handle, pos_item2);

            file << sim_time << " " << w_p_ee[0] << " " << w_p_ee[1] << " " << w_p_ee[2] << " " <<
                hd_pos[0] << " " << hd_pos[1] << " " << hd_pos[2] << " " <<
                hd_gui[0] << " " << hd_gui[1] << " " << hd_gui[2] << " " <<
                pos_item1[0] << " " << pos_item1[1] << " " << pos_item1[2] << " " <<
                pos_item2[0] << " " << pos_item2[1] << " " << pos_item2[2] << " " <<
                rb_p_target1[0] << " " << rb_p_target1[1] << " " << rb_p_target1[2] << " " <<
                rb_p_target2[0] << " " << rb_p_target2[1] << " " << rb_p_target2[2] << " " <<
                object_in_gripper << " " <<
                goal[0] << " " << goal[1] << " " << goal[2] << " " <<
                "\n";
        }
        // std:: cout << "NOT FREEZED" << endl;
        clock_gettime(CLOCK_REALTIME, &stop);
        elapsed = diff_time(&start,&stop);

        if (((cycle_time*BILLION) - elapsed.tv_nsec) > 0)
            usleep((cycle_time*BILLION - (elapsed).tv_nsec)/1000);

        sim_time += cycle_time;

        if(debug)
        {
            cout << "\033[2J\033[1;1H";
            printf( "sleep: %lld.%.9f\n",0 - (long long)elapsed.tv_sec, ((cycle_time*BILLION) - elapsed.tv_nsec));
            cout << "Haptic device position = " << hd_pos[0] <<" " << hd_pos[1] << " " << hd_pos[2] <<  endl;
            cout << "End effector position = " << rb_T_ee(0,3) << " " << rb_T_ee(1,3) << " " << rb_T_ee(2,3) <<  endl;
            cout << "Goal position = " << goal[0] << " " << goal[1] << " " << goal[2] <<  endl;
            cout << "Force = " << hd_gui[0] << " " << hd_gui[1] << " " << hd_gui[2] << endl;
        }
        //break;
    }

    // nocbreak();
    fprintf(stdout, "Stopping the haptic device...\n");
    if (falcon.close() == 0)
    {
        printf("Haptic device successfully stopped \n");
    }
    else
    {
        fprintf(stderr, "An error occurred during stopping the haptic device...\n");
    }
    simxStopSimulation(coppelia.clientID(), simx_opmode_blocking);
    simxFinish(-1);
    printf("Connection to Coppelia Sim closed\n");

    return 0;
}
