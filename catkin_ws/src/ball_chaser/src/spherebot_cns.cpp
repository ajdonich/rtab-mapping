#include "ros/ros.h"
#include "ball_chaser/CommandVel.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ContactsState.h>
#include <std_msgs/Float64.h>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>

// Constants
#define CMD_PATIENCE 3.0      // max nsec to wait
#define RTD 57.29577951308232 // rad_to_deg == 180/pi
#define TAU 6.283185307179586 // == 2*pi
#define JT_ERROR 0.05         // 2.8 degrees
#define JT_WIDE_ERROR 0.1     // 5.7 degrees
#define WRIST_SUPINE 3.1416   // 180 degrees
#define HEAD_FLEX 0.2618      // 15 degrees

// Singleton accessor macro
#define SPHEREBOT SphereBotCNS::getInstance()

enum RobotMode { 
    SEARCH_BALL, 
    APPROACH_BALL, 
    DOCK_BALL,
    PICKUP,
    SEARCH_BUCKET,
    APPROACH_BUCKET,
    DOCK_BUCKET,
    DROPOFF
};

enum HeadState { 
    UNDEFINED_HS,
    NEUTRAL,
    FLEXED
};

enum ArmState { 
    UNDEFINED_AS,
    STOWED_SUPINE,
    STOWED_PRONE, 
    CARRY_SUPINE
};

enum TargColor {
    WHITE,
    RED
};

// ----------------- ARM POSE LIBRARY -----------------

std::vector<std::string> ARM_JOINTS{
    "arm_wheel_joint", "arm_forearm_joint", "arm_wrist_joint",
    "arm_hand_left_joint", "arm_hand_right_joint", 
    "arm_finger_left_joint", "arm_finger_right_joint" };

// Order of pose angles match those of ARM_JOINTS above
std::unordered_map<int, std::vector<double>> ARM_POSES({
    { ArmState::STOWED_SUPINE, {0.0, 0.0, 3.1416, 0.0, 0.0, 0.0, 0.0} },
    { ArmState::STOWED_PRONE, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} },
    { ArmState::CARRY_SUPINE, {-0.3491, 0.0, 3.1416, 0.6109, -0.6109, -0.7500, 0.7500} }});


class TargRect
{
public:
    int x, y, w, h;
    TargRect() : x(-1), y(-1), w(-1), h(-1) {}
    TargRect(int xa, int ya) : x(xa), y(ya), w(-1), h(-1) {}
    
    bool valid() { return x >= 0 && y >= 0 && w >= 0 && h >= 0; }
    void reset() { x=-1, y=-1, w=-1, h=-1; }
    
    std::string str(float zdepth=0.0) {
        return std::string("x,y=(" + std::to_string(x) + "," + std::to_string(y) + 
            "), cen=(" + std::to_string(x+(w/2)) + "," + std::to_string(y+(h/2)) +
            "), w=" + std::to_string(w) + ", h=" + std::to_string(h) + 
            ", z=" + std::to_string(zdepth));
    }
};

class SphereBotCNS
{
public:

    // SphereBotCNS singleton accessor
    static SphereBotCNS& getInstance() {
        static SphereBotCNS single_instance; 
        return single_instance;
    }
    
    // Camera image callback (updates member: TargRect target)
    static void process_image_callback(const sensor_msgs::Image &img) {     
        TargRect targ = SPHEREBOT.findtarget(img);
        std::lock_guard<std::mutex> guard(SPHEREBOT.targetlock);
        SPHEREBOT.target = targ;
    }
    
    // Lidar laser callback (updates member: double targdistz)
    static void process_lidar_callback(const sensor_msgs::LaserScan &scan) {
        float depth = 0.0; int count = 0;
        for (int i=0; i<32; ++i) if (scan.ranges[i] < 1.25) 
            depth += scan.ranges[i], count += 1;
        
        SPHEREBOT.targdistz = count>0 ? depth/count : -1.0;
    }

    // Joint states callback (updates member: unordered_map<string,float> jtstates)
    static void process_jtstates_callback(const sensor_msgs::JointState &state) {
        for (int i=0; i<9; i++) SPHEREBOT.jtstates[state.name[i]] = state.position[i];
    }

    // Send request to command_robot service
    void drive_robot(float lin_x, float ang_z) 
    {
        // Don't send back-to-back duplicate drive commands
        static std::vector<float> prev_drivecmd(2, 0.0);
        if (prev_drivecmd[0] != lin_x || prev_drivecmd[1] != ang_z)
        {
            ball_chaser::CommandVel srv;
            srv.request.linear_x = lin_x;
            srv.request.angular_z = ang_z;

            // Call the /ball_chaser/command_robot service w/requested velocities
            if (!cmd_client.call(srv)) ROS_ERROR("Failed to call service command_robot");
            else { 
                prev_drivecmd[0] = lin_x;
                prev_drivecmd[1] = ang_z;

                // Debug/info logging
                // std::string xdstr = "STOP ";
                // if (lin_x > 0.0) xdstr = "FORWARD ";
                // else if (lin_x < 0.0) xdstr = "REVERSE ";

                // std::string ztstr = "STOP";
                // if (ang_z > 0.0) ztstr = "LEFT";
                // else if (ang_z < 0.0) ztstr = "RIGHT";

                // ROS_INFO(("Drive cmd: " + xdstr + ztstr + 
                // " (%1.2f, %1.2f)").c_str(), lin_x, ang_z);
            }
        }
    }

    // Default is to test for WHITE pixel
    bool is_coloredpixel(TargColor col, const sensor_msgs::Image &img, int i) {
        if (col == TargColor::RED) 
            return img.data[i] == 255 && img.data[i+1] == 0 && img.data[i+2] == 0;
        return img.data[i] == 255 && img.data[i+1] == 255 && img.data[i+2] == 255;
    }

    TargColor targcolor() {
        switch(mode)
        {
            case RobotMode::SEARCH_BUCKET:
            case RobotMode::APPROACH_BUCKET:
            case RobotMode::DOCK_BUCKET:
            case RobotMode::DROPOFF:
                return TargColor::RED;

            case RobotMode::SEARCH_BALL:
            case RobotMode::APPROACH_BALL:
            case RobotMode::DOCK_BALL:
            case RobotMode::PICKUP:
            default: return TargColor::WHITE;
        }
    }

    // Returns rect around target (expects img.encoding == RGB8). 
    TargRect findtarget(const sensor_msgs::Image img)
    {
        int i = 0;
        TargColor c = targcolor();
        while (true) {
            // Find first white pixel in img (if exits)
            if (i >= img.height * img.step) return TargRect();
            else if (is_coloredpixel(c, img, i)) break;
            i += 3;
        }

        // Derive target frame values
        int row = (int)(i/img.step);
        int col = (i/3) % img.width;        
        TargRect bframe(col, row);
        
        while (row < img.height && bframe.h == -1)
        {
            int rstart = row*img.step;
            int rend = rstart + img.step;
            int il = rstart + col*3, ir = il;
            
            while (il >= rstart && is_coloredpixel(c, img, il)) il -= 3;
            while (ir < rend && is_coloredpixel(c, img, ir)) ir += 3;

            if ((il-rstart)/3 < bframe.x) bframe.x = (il-rstart)/3;
            if ((ir-il-1)/3 > bframe.w) bframe.w = (ir-il-1)/3;
            if (ir == il) bframe.h = row-bframe.y-1;
            row += 1;
        }
        
        // _publish_frame(img, bframe);
        ROS_INFO_STREAM(bframe.str(targdistz));
        return bframe;
    }

    /* 
    Attempts to keep target centered horizontally in 800x800 camera visual.
    Approaches at top speed to ~5m, then at exponentially decreasing speed 
    to 1m. Uses lidar and empirically derived table of ball target dims:
    Depth:  Target dims:                    Xdrive-vel:
     1m:     x,y=(349,637), area=103x119     6.64e-06
     2m:     x,y=(374,511), area=49x49       0.13
     3m:     x,y=(383,469), area=32x32       0.50
     4m:     x,y=(387,449), area=24x23       0.91
     5m:     x,y=(390,437), area=20x17       1.00 
    */
    bool approach_target() 
    {
        static float MAXTURN = 1.0;
        static float MAXDRIVE = 1.0;
        static int hcenter = 400;
        
        TargRect tframe = safe_target();
        if ( !tframe.valid() ) return true;
        else
        {
            // Put bcenter at 400px 
            float xdrive = 0.0, zturn = 0.0;
            float bcenter = tframe.x + (tframe.w/2);
            float zfactor = (hcenter - bcenter)/hcenter;
            zturn = zfactor*MAXTURN; // Linear scaled turn
            
            float targarea = tframe.w*tframe.h;
            bool done = 0.0 < targdistz && targdistz < 1.0;
            if (done) xdrive = 0.0, zturn = 0.0; // Stop
            else if (targarea < 340) xdrive = MAXDRIVE; // Fast speed
            else {
                float xfactor = std::pow(0.4, std::log(targarea/340));
                xdrive = xfactor*MAXDRIVE; // Scaled exponential slow down
            }

            // Drive control can't handle simultaneous high speed linear
            // and angular movement, thus prioritize turn and clip drive.
            if (fabs(zturn) > 0.3) xdrive = 0.0;
            drive_robot(xdrive, zturn);
            return done;
        }
    }

    // Similar to approach but uses lidar exclusively for range.
    bool dock_target() 
    {
        static float DOCKTURN = 1.0;
        static float DOCKDRIVE = 0.1;
        static int hcenter = 400;
        
        TargRect tframe = safe_target();
        if ( !tframe.valid() ) return true;
        else
        {
            // Put bcenter at 400px 
            float xdrive = 0.0, zturn = 0.0;
            float bxcenter = tframe.x + (tframe.w/2);
            float zfactor = (hcenter - bxcenter)/hcenter;
            zturn = zfactor*DOCKTURN; // Linear scaled turn
            
            bool done = targdistz < 0.4;
            if (done) xdrive = 0.0, zturn = 0.0; // Stop
            else xdrive = targdistz*DOCKDRIVE;

            drive_robot(xdrive, zturn);
            return done;
        }
    }

    // Reads from the joint states hash table (maintained by 
    // ros::spin thread) and returns the current HeadState.
    HeadState headstate(int verbose=0) {
        HeadState hstate = HeadState::UNDEFINED_HS;
        double sjt = jtstates["spherebot_isocket_joint"];
        double hjt = jtstates["spherebot_head_joint"];

        if (std::fabs(sjt) < JT_ERROR && std::fabs(hjt) < JT_ERROR) 
            hstate = HeadState::NEUTRAL;

        else if (std::fabs(std::fabs(sjt)-HEAD_FLEX) < JT_ERROR && 
                 std::fabs(std::fabs(hjt)-HEAD_FLEX) < JT_ERROR) 
            hstate = HeadState::FLEXED;
        
        if (verbose)
            ROS_INFO("Head joints: %0.1f, %0.1f, %d", sjt*RTD, hjt*RTD, hstate);

        return hstate;
    }

    // Reads from the joint states hash table (maintained by 
    // ros::spin thread) and returns the current ArmState.
    // Evaluates state relative to ARM POSE LIBRARY at top.
    ArmState armstate(int verbose=0) 
    {
        ArmState astate = ArmState::UNDEFINED_AS;
        for (const auto &pose : ARM_POSES)
        {
            bool pmatch = true;
            for(int i=0; i<ARM_JOINTS.size() && pmatch; ++i) {
                double jtval = jtstates[ARM_JOINTS[i]];
                if (ARM_JOINTS[i] == "arm_wrist_joint") 
                    jtval = std::fabs(std::fmod(jtval,TAU));

                pmatch &= std::fabs(jtval - pose.second[i]) < JT_ERROR;
            }

            if (pmatch) { astate = static_cast<ArmState>(pose.first); break; }
        }

        static int log_throttle = 0;
        if (verbose && (log_throttle%10) == 0) {
            double awj = jtstates["arm_wheel_joint"];
            double afj = jtstates["arm_forearm_joint"];
            double awrj = jtstates["arm_wrist_joint"];
            double ahlj = jtstates["arm_hand_left_joint"];
            double ahrj = jtstates["arm_hand_right_joint"];
            double aflj = jtstates["arm_finger_left_joint"];
            double afrj = jtstates["arm_finger_right_joint"];
            ROS_INFO("Arm joints: %0.1f, %0.2f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %d",
                awj*RTD, afj, awrj*RTD, ahlj*RTD, ahrj*RTD, aflj*RTD, afrj*RTD, astate);
        }
        log_throttle += 1;
        return astate;
    }

    // Publish head/isocket joint angle cmds to place in position: hstate, if
    // not already there. Waits for cmd to complete, up to CMD_PATIENCE secs.
    bool position_head(ros::Rate &loop_rate, const HeadState &hstate)
    {
        if (headstate() == hstate) return true;
        drive_robot(0.0, 0.0); // Pause wheels
        bool success = false;

        // Start command completion time window
        double initial = ros::Time::now().toSec();
        while (not initial) initial = ros::Time::now().toSec();

        // Instantiate and populate messages
        std_msgs::Float64 spine_ang, head_ang;
        if (hstate == HeadState::NEUTRAL) spine_ang.data = 0.0, head_ang.data = 0.0;
        else if (hstate == HeadState::FLEXED) spine_ang.data = HEAD_FLEX, head_ang.data = HEAD_FLEX;
        
        // Publish commands
        headpos_publisher.publish(spine_ang);
        spinepos_publisher.publish(head_ang);
        
        // Allow max of CMD_PATIENCE sec for command to complete
        while (ros::Time::now().toSec()-initial < CMD_PATIENCE)
            if (success = (headstate() == hstate)) break;
            loop_rate.sleep();

        ROS_INFO("Published POS head commands, comp time: %f sec, success: %d",
                  ros::Time::now().toSec()-initial, success);

        return success;
    }

    // Publish robot arm joint angle cmds to place in position: astate, if
    // not already there. Waits for cmd to complete, up to CMD_PATIENCE secs.
    bool position_arm(ros::Rate &loop_rate, const ArmState &astate) 
    {
        if (armstate() == astate) return true;
        bool success = false;

        // Start command completion time window
        double initial = ros::Time::now().toSec();
        while (not initial) initial = ros::Time::now().toSec();

        // Instantiate and populate messages
        std_msgs::Float64 wheel_ang, forearm_dist, wrist_ang;
        std_msgs::Float64 hand_left_ang, hand_right_ang;
        std_msgs::Float64 finger_left_ang, finger_right_ang;
        wheel_ang.data = ARM_POSES[astate][0];
        forearm_dist.data = ARM_POSES[astate][1];
        wrist_ang.data = ARM_POSES[astate][2];
        hand_left_ang.data = ARM_POSES[astate][3];
        hand_right_ang.data = ARM_POSES[astate][4];
        finger_left_ang.data = ARM_POSES[astate][5];
        finger_right_ang.data = ARM_POSES[astate][6];

        // Publish commands
        armwheel_publisher.publish(wheel_ang);
        forearm_publisher.publish(forearm_dist);
        wrist_publisher.publish(wrist_ang);
        finger_left_publisher.publish(finger_left_ang);
        finger_right_publisher.publish(finger_right_ang);
        hand_left_publisher.publish(hand_left_ang);
        hand_right_publisher.publish(hand_right_ang);

        // Allow max of CMD_PATIENCE sec for command to complete
        while (ros::Time::now().toSec()-initial < CMD_PATIENCE)
            if (success = (armstate() == astate)) break;
            loop_rate.sleep();

        ROS_INFO("Published POS arm commands, comp time: %f sec, success: %d",
                  ros::Time::now().toSec()-initial, success);

        return success;
    }

    // Sends an empirically derived sequence of arm joint commands 
    // timed over a number of seconds to (ideally) pick up the ball.
    bool pickup_ball() 
    {
        static double DOFFSET = 0.45;
        drive_robot(0.0, 0.0); // Pause wheels

        // Open hand
        std_msgs::Float64 wrist_ang, hand_left_ang, hand_right_ang;
        std_msgs::Float64 finger_left_ang, finger_right_ang;
        hand_left_ang.data = 0.7854, hand_right_ang.data = -0.7854;
        finger_left_ang.data = -0.7854, finger_right_ang.data = 0.7854;
        wrist_ang.data = 0.0;

        wrist_publisher.publish(wrist_ang);
        hand_left_publisher.publish(hand_left_ang);
        hand_right_publisher.publish(hand_right_ang);
        finger_left_publisher.publish(finger_left_ang);
        finger_right_publisher.publish(finger_right_ang);
        ROS_INFO("Published open hand angles: %f, %f", 
                 hand_left_ang.data, hand_right_ang.data);
        ros::Duration(1).sleep();

        // Reach out hand 
        double x = targdistz + 0.1;
        double theta = atan(0.25/x);
        double d = x/cos(theta) - DOFFSET;

        std_msgs::Float64 wheel_ang, forearm_dist;
        wheel_ang.data = theta, forearm_dist.data = d, 

        armwheel_publisher.publish(wheel_ang);
        forearm_publisher.publish(forearm_dist);
        ROS_INFO("Published reach out arm position ang/dist: %f, %f, %f", 
                 wheel_ang.data, forearm_dist.data, wrist_ang.data);
        ros::Duration(1).sleep();

        // Close hand
        hand_left_ang.data = 0.0, hand_right_ang.data = 0.0;
        finger_left_ang.data = -0.5236, finger_right_ang.data = 0.5236;

        finger_left_publisher.publish(finger_left_ang);
        finger_right_publisher.publish(finger_right_ang);
        hand_left_publisher.publish(hand_left_ang);
        hand_right_publisher.publish(hand_right_ang);
        ROS_INFO("Published close hand angles: %f, %f", 
                 hand_left_ang.data, hand_right_ang.data);
        ros::Duration(2).sleep();

        // Pickup ball 
        wheel_ang.data = -0.3491, forearm_dist.data = 0.0, 
        wrist_ang.data = WRIST_SUPINE;

        forearm_publisher.publish(forearm_dist);
        armwheel_publisher.publish(wheel_ang);
        wrist_publisher.publish(wrist_ang);
        ROS_INFO("Published lift ball arm position ang/dist: %f, %f, %f", 
                 wheel_ang.data, forearm_dist.data, wrist_ang.data);
        ros::Duration(2).sleep();

        // Carry ball
        hand_left_ang.data = 0.6109, hand_right_ang.data = -0.6109;
        finger_left_ang.data = -0.7500, finger_right_ang.data = 0.7500;

        finger_left_publisher.publish(finger_left_ang);
        finger_right_publisher.publish(finger_right_ang);
        hand_left_publisher.publish(hand_left_ang);
        hand_right_publisher.publish(hand_right_ang);
        ROS_INFO("Published carry ball angles: %f, %f", 
                 hand_left_ang.data, hand_right_ang.data);
        ros::Duration(2).sleep();
        return true;
    }

    // Similar to pickup_ball, but simpler. Expects the bucket to be 
    // positioned with its RED sign to the back/away from the spherebot.
    bool dropoff_ball() 
    {
        drive_robot(0.0, 0.0); // Pause wheels

        // Extend and pronate hand 
        std_msgs::Float64 wheel_ang, forearm_dist, wrist_ang;
        wheel_ang.data = -.3491, forearm_dist.data = .2, 
        wrist_ang.data = 0.0;

        armwheel_publisher.publish(wheel_ang);
        forearm_publisher.publish(forearm_dist);
        wrist_publisher.publish(wrist_ang);
        ROS_INFO("Published reach out arm position ang/dist: %f, %f, %f", 
                 wheel_ang.data, forearm_dist.data, wrist_ang.data);
        ros::Duration(0.5).sleep();

        // Open hand
        std_msgs::Float64 hand_left_ang, hand_right_ang;
        std_msgs::Float64 finger_left_ang, finger_right_ang;
        hand_left_ang.data = 0.7854, hand_right_ang.data = -0.7854;
        finger_left_ang.data = -0.7854, finger_right_ang.data = 0.7854;

        finger_left_publisher.publish(finger_left_ang);
        finger_right_publisher.publish(finger_right_ang);
        hand_left_publisher.publish(hand_left_ang);
        hand_right_publisher.publish(hand_right_ang);
        ROS_INFO("Published open hand angles: %f, %f", 
                 hand_left_ang.data, hand_right_ang.data);
        ros::Duration(1).sleep();

        return true;
    }

    // Convenience method.
    void log_robot_state() {
        static std::string rstatestr[8] = { 
            "SEARCH_BALL", "APPROACH_BALL", "DOCK_BALL", "PICKUP", 
            "SEARCH_BUCKET", "APPROACH_BUCKET", "DOCK_BUCKET", "DROPOFF" };
        static std::string hstatestr[3] = { "UNDEFINED_HS", "NEUTRAL", "FLEXED" };
        static std::string astatestr[3] = { "UNDEFINED_AS", "STOWED_SUPINE", "STOWED_PRONE" };
        
        static int laststates[3] = {-1,-1,-1};
        int currstate[3] = {mode, headstate(), armstate()};

        bool same = true;
        for (int i=0; i<3; i++) {
            same &= (laststates[i] == currstate[i]);
            laststates[i] = currstate[i];
        }

        if (!same) ROS_INFO_STREAM("Robot State: " + rstatestr[currstate[0]] + 
            ", Head: " + hstatestr[currstate[1]] + ", Arm: " + astatestr[currstate[2]]);
    }

    // Spherebot brain state machine event loop. Executed on its own thread and 
    // communicates with ros::spin thread ONLY through specified shared resources.
    void state_machine(ros::Rate loop_rate)
    {
        while (ros::ok()) 
        {
            log_robot_state();
            switch(mode)
            {
                case RobotMode::SEARCH_BALL:
                    while (!position_arm(loop_rate, ArmState::STOWED_SUPINE) || 
                           !position_head(loop_rate, HeadState::NEUTRAL));

                    if (!safe_target().valid()) drive_robot(0.0, 0.5);
                    else {
                        drive_robot(0.0, 0.0);
                        mode = RobotMode::APPROACH_BALL;
                    }
                    break;
                
                case RobotMode::APPROACH_BALL:
                    while (!position_arm(loop_rate, ArmState::STOWED_SUPINE) || 
                           !position_head(loop_rate, HeadState::NEUTRAL));

                    if (!safe_target().valid()) mode = RobotMode::SEARCH_BALL;
                    else if (approach_target()) mode = RobotMode::DOCK_BALL;
                    break;
                
                case RobotMode::DOCK_BALL:
                    while (!position_arm(loop_rate, ArmState::STOWED_PRONE) || 
                           !position_head(loop_rate, HeadState::FLEXED));

                    if (targdistz < 0) mode = RobotMode::SEARCH_BALL;
                    else if (dock_target()) mode = RobotMode::PICKUP;
                    break;

                case RobotMode::PICKUP:
                    if (targdistz < 0) mode = RobotMode::SEARCH_BALL;
                    else if (pickup_ball()) mode = RobotMode::SEARCH_BUCKET;
                    break;

                case RobotMode::SEARCH_BUCKET: 
                    while (!position_head(loop_rate, HeadState::NEUTRAL));
                    if (!target.valid()) drive_robot(0.0, 0.5);
                    else {
                        drive_robot(0.0, 0.0);
                        mode = RobotMode::APPROACH_BUCKET;
                    }
                    break;

                case RobotMode::APPROACH_BUCKET: 
                    while (!position_head(loop_rate, HeadState::NEUTRAL));
                    if (!safe_target().valid()) mode = RobotMode::SEARCH_BUCKET;
                    else if (approach_target()) mode = RobotMode::DOCK_BUCKET;
                    break;

                case RobotMode::DOCK_BUCKET: 
                    if (targdistz < 0) mode = RobotMode::SEARCH_BUCKET;
                    else if (dock_target()) mode = RobotMode::DROPOFF;
                    break;

                case RobotMode::DROPOFF:
                    if (dropoff_ball()) {
                        mode = RobotMode::SEARCH_BALL;

                        // Back away from bucket.
                        drive_robot(-2.0, 0.0);
                        ros::Duration(2).sleep();
                    }
                    break;

                default: ROS_ERROR("Unrecogized robot mode");
            }

            loop_rate.sleep();
        }
    }

    // Cb initialize
    void initialize()
    {
        // Spherebot head/isocket joint angle control publishers
        headpos_publisher = _n.advertise<std_msgs::Float64>("/head_position_controller/command", 10);
        spinepos_publisher = _n.advertise<std_msgs::Float64>("/isocket_position_controller/command", 10);

        // Arm/hand joint angle control publishers
        armwheel_publisher = _n.advertise<std_msgs::Float64>("/arm_wheel_position_controller/command", 10);
        forearm_publisher = _n.advertise<std_msgs::Float64>("/arm_forearm_position_controller/command", 10);
        wrist_publisher = _n.advertise<std_msgs::Float64>("/arm_wrist_position_controller/command", 10);
        hand_left_publisher = _n.advertise<std_msgs::Float64>("/arm_hand_left_position_controller/command", 10);
        hand_right_publisher = _n.advertise<std_msgs::Float64>("/arm_hand_right_position_controller/command", 10);
        finger_left_publisher = _n.advertise<std_msgs::Float64>("/arm_finger_left_position_controller/command", 10);
        finger_right_publisher = _n.advertise<std_msgs::Float64>("/arm_finger_right_position_controller/command", 10);
        
        // Uses camera_subscriber to monitor robot "vision" images, if the target is 
        // located, uses cmd_client to issue appropriate velocity commands to chase target
        cmd_client = _n.serviceClient<ball_chaser::CommandVel>("/ball_chaser/command_robot");
        camera_subscriber = _n.subscribe("/camera/rgb/image_raw", 1, process_image_callback);
        lidar_subscriber = _n.subscribe("/hokuyo/laser_scan", 1, process_lidar_callback);
        jtstate_subscriber = _n.subscribe("/joint_states", 1, process_jtstates_callback);
        ROS_INFO_STREAM("Process Image Node initialization completed");
    }

    TargRect safe_target() {
        std::lock_guard<std::mutex> guard(targetlock);
        return target;
    }

    // Delete/explicitly remove copy ctor and assignment
    SphereBotCNS(SphereBotCNS const&)   = delete;
    void operator=(SphereBotCNS const&) = delete;

private:
    ros::NodeHandle _n;
    ros::ServiceClient cmd_client;
    ros::Subscriber camera_subscriber;
    ros::Subscriber lidar_subscriber;
    ros::Subscriber jtstate_subscriber;
    ros::Subscriber ltactile_subscriber;
    ros::Subscriber rtactile_subscriber;

    // Head joint command publishers
    ros::Publisher headpos_publisher;
    ros::Publisher spinepos_publisher;

    // Arm joint command publishers
    ros::Publisher armwheel_publisher;
    ros::Publisher forearm_publisher;
    ros::Publisher wrist_publisher;
    ros::Publisher finger_left_publisher;
    ros::Publisher finger_right_publisher;
    ros::Publisher hand_left_publisher;
    ros::Publisher hand_right_publisher;

    // -------- !!! (Start) THREAD SHARED RESOURCES !!! ---------

    // Members target, tardistz and jtstates are updated by the ros::spin
    // thread within subscription callbacks. The state machine thread 
    // manages mode and sends requisite joint and drive commands.

    TargRect target;
    std::mutex targetlock;

    std::atomic<RobotMode> mode;
    std::atomic<double> targdistz;
    std::unordered_map<std::string, std::atomic<double>> jtstates;
    // ---------- !!! (End) THREAD SHARED RESOURCES !!! -----------

    // Ctor private
    SphereBotCNS() : mode(RobotMode::SEARCH_BALL), targdistz(-1.0) 
    {
        // Init joint states with large dummy values
        jtstates["spherebot_head_joint"] = 1000.0;
        jtstates["spherebot_isocket_joint"] = 1000.0;
        jtstates["arm_wheel_joint"] = 1000.0;
        jtstates["arm_forearm_joint"] = 1000.0;
        jtstates["arm_wrist_joint"] = 1000.0;
        jtstates["arm_hand_left_joint"] = 1000.0;
        jtstates["arm_hand_right_joint"] = 1000.0;
        jtstates["arm_finger_left_joint"] = 1000.0;
        jtstates["arm_finger_right_joint"] = 1000.0;
    }
}; 

int main(int argc, char **argv) 
{
    // Initiate ROS and SphereBotCNS instance
    ros::init(argc, argv, "spherebot_cns");
    SPHEREBOT.initialize();
    
    // Start robot state machine
    ros::Rate smachine_rate(10); // 10Hz
    std::thread smachine_thrd(
        &SphereBotCNS::state_machine, 
        &SPHEREBOT, smachine_rate);

    // Eventloop
    ros::spin();
    smachine_thrd.join();
    return 0;
}