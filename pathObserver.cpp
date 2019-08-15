#include "ros/ros.h"
#include "std_msgs/String.h"
#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "std_msgs/Float32.h"
#include <cstdio>
#include <unistd.h>
#include <tf/tf.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <mavros_msgs/PositionTarget.h>
#define pi 3.1415926
int flag=0;
float KPx = 1;
float KPy = 1;
float KPz = 1;
float KProll = 1;                                    //Gain


bool landing = false;
bool landing2 = false;
bool target = false;
bool aruco_detect = false;
bool initial = false;
bool finishLoop1 = false;
bool init = false;                                 //switch

double errx , erry ,errz , err_roll;               //"err_roll" is defined yaw error
double tagRoll, tagPitch, tagYaw;
double InitRoll, InitPitch, InitYaw;
double roll, pitch, yaw;
double begin;

using namespace std;

typedef struct      
{
    float roll;
    float x;
    float y;
    float z;
}vir;

mavros_msgs::PositionTarget pose; //Some complex system requires all feautures that mavlink message provide(frame)
mavros_msgs::State current_state; //Current autopilot state(connected, armed,guided,mode)
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

geometry_msgs::Twist host_vel;
geometry_msgs::PoseStamped host_mocap; //A Pose with reference coordinate frame and timestamp(header,pose)
geometry_msgs::PoseStamped initial_pose;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        host_mocap = *msg;
        if(init==false)
        {
        initial_pose = host_mocap;

        tf::Quaternion q(
            host_mocap.pose.orientation.x,
            host_mocap.pose.orientation.y,
            host_mocap.pose.orientation.z,
            host_mocap.pose.orientation.w);

        tf::Matrix3x3(q).getRPY(InitRoll,InitPitch,InitYaw);         //record initial position

        init = true;
        }
        tf::Quaternion Q(
            host_mocap.pose.orientation.x,
            host_mocap.pose.orientation.y,
            host_mocap.pose.orientation.z,
            host_mocap.pose.orientation.w);

        tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}

geometry_msgs::PoseStamped tag_latest;
fiducial_msgs::FiducialTransformArray  tag_array;
void tag_cb(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    tag_array = *msg;
    if(tag_array.transforms.size()>0){
        for(int i=0;i<tag_array.transforms.size();i++){
            if(tag_array.transforms[i].fiducial_id == 6){
                aruco_detect = true;
                tag_latest.pose.position.x = tag_array.transforms[i].transform.translation.x;
                tag_latest.pose.position.y = tag_array.transforms[i].transform.translation.y;
                tag_latest.pose.position.z = tag_array.transforms[i].transform.translation.z;
                tag_latest.pose.orientation.x = tag_array.transforms[i].transform.rotation.x;
                tag_latest.pose.orientation.y = tag_array.transforms[i].transform.rotation.y;
                tag_latest.pose.orientation.z = tag_array.transforms[i].transform.rotation.z;            //detection
                tag_latest.pose.orientation.w = tag_array.transforms[i].transform.rotation.w;
                tf::Quaternion Q(
                    tag_latest.pose.orientation.x,
                    tag_latest.pose.orientation.y,
                    tag_latest.pose.orientation.z,
                    tag_latest.pose.orientation.w);

                tf::Matrix3x3(Q).getRPY(tagRoll,tagPitch,tagYaw);
            }       
        }        
    }
    else{
        aruco_detect = false;
    }
}

void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::PoseStamped& tag_latest, mavros_msgs::PositionTarget* vs)
{
       
    float ux, uy, uz, uroll;

    if((aruco_detect == true)&&(landing == false))
    {   
        ROS_INFO("Tracking!!");
        pose.coordinate_frame= mavros_msgs::PositionTarget::FRAME_BODY_NED;
        errx = 0.6*(0 - (-tag_latest.pose.position.x));
        erry = -0.6*(0 - (-tag_latest.pose.position.y));
        errz = -0.5*(-0.8 - (-tag_latest.pose.position.z));                           //K_camTobody*error_cam = u_body
        err_roll = 0.3*(0-tagYaw);                                                    //tag tracking
        

        vir.x = host_mocap.pose.position.x;
        vir.y = host_mocap.pose.position.y;
        vir.z = host_mocap.pose.position.z;

        if((abs(errx)/0.6 <= 0.03) && (abs(erry)/0.6 <= 0.03))
        {
                aruco_detect = false;
                landing2 = true;                                                      //landing condition                                              
                begin = ros::Time::now().toSec();
        }

    }

    else if((aruco_detect == false)||(landing==true))
    {
        pose.coordinate_frame= mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        errx = vir.x - host_mocap.pose.position.x;
        erry = vir.y - host_mocap.pose.position.y;                                    //keyboard control
        errz = vir.z - host_mocap.pose.position.z;
        err_roll = vir.roll - yaw;
        ROS_INFO("no detection");
    }

    if(err_roll>pi)
    {
    err_roll = err_roll - 2*pi;
    }
    else if(err_roll<-pi)
    {
    err_roll = err_roll + 2*pi;
    }
    ux = KPx*errx;
    uy = KPy*erry;
    uz = KPz*errz;
    uroll = KProll*err_roll;                                                          //defensive mechanisms
    if(ux<=-0.4 ||ux>=0.4)
    {
    ux = 0.4*ux/abs(ux);
    }
    if(uy<=-0.4 ||uy>=0.4)
    {
    uy = 0.4*uy/abs(uy);
    }

    vs->velocity.x = ux;
    vs->velocity.y = uy;
    vs->velocity.z = uz;
    vs->yaw_rate = uroll;
 }

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {                                       //keyboard input
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_tag_follow");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                    ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                       ("/mavros/setpoint_position/local", 10);
    ros::Publisher bf_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 5);  //(1)

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                           ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                             ("/mavros/set_mode");

    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose", 1, host_pos);//(2)

    ros::Subscriber tag_sub = nh.subscribe("/fiducial_transforms",5,tag_cb);  //(3)

    ros::Rate rate(50);

                    //topic:(1)mavros  (2)vrpn server  (3)tag detection


    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {

        ros::spinOnce();
        rate.sleep();
    }

    
    pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX|
                     mavros_msgs::PositionTarget::IGNORE_AFY|
                     mavros_msgs::PositionTarget::IGNORE_AFZ|
                     mavros_msgs::PositionTarget::IGNORE_PX|
                     mavros_msgs::PositionTarget::IGNORE_PY|
                     mavros_msgs::PositionTarget::IGNORE_PZ|
                     mavros_msgs::PositionTarget::IGNORE_YAW;                 //mavros setting
    vir vir;
    pose.velocity.x = 0;
    pose.velocity.y = 0;
    pose.velocity.z = 0;
    pose.yaw_rate= 0;

    vir.x = initial_pose.pose.position.x;
    vir.y = initial_pose.pose.position.y;
    vir.z = 0.5;
    vir.roll = InitYaw;                                                       //initial condition


    //send a few setpoints before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        bf_pub.publish(pose);

        vir.x = initial_pose.pose.position.x;
        vir.y = initial_pose.pose.position.y;
        vir.z = initial_pose.pose.position.z + 0.7;                           //ensure that px4 received data
        vir.roll = InitYaw;													  //if px4 doesn't received data,
        ros::spinOnce();													  //drone will be out of control.
        rate.sleep();
    }

    mavros_msgs::SetMode POSCTL_set_mode;
    POSCTL_set_mode.request.custom_mode = "POSCTL";
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;



    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {


        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {                                                                   //taking off after 10 sec

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

//keyboard
    int c = getch();

        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                vir.z += 0.05;
                break;
            case 66:    // key down
                vir.z += -0.05;
                break;
            case 67:    // key CW(->)
                vir.roll -= 0.05;
                break;
            case 68:    // key CCW(<-)
                vir.roll += 0.05;
                break;
            case 119:    // key foward  (w)
                vir.x += 0.15;
                break;
            case 120:    // key back    (x)
                vir.x += -0.15;
                break;
            case 97:    // key left     (a)
                vir.y += 0.15;
                break;
            case 100:    // key right   (d)
                vir.y -= 0.15;
                break;
            case 105:    // type "i" ,the drone will track a path.
            {
                if(initial == true){
                initial = false;
                }
                else if(initial == false){
                initial = true;
                begin = ros::Time::now().toSec();

                }
                break;
            }
            case 115:    // origin      (s)
            {
            vir.x = initial_pose.pose.position.x;
            vir.y = initial_pose.pose.position.y;
            vir.z = initial_pose.pose.position.z+0.2;
                break;
            }
            case 116:    // (t)
            {
                if(landing == true){
                landing = false;
                }
                else if(landing == false){                                          //"landing"determined whether to cancel "tracking mode"
                landing = true;
                }
                break;
            }
              case 108:    // close arming //l
            {
            offb_set_mode.request.custom_mode = "MANUAL";
            set_mode_client.call(offb_set_mode);
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
                break;
            }
            case 63:
            return 0;
            break;
            }
        }

        if(vir.roll>pi){
            vir.roll = vir.roll - 2*pi;
        }
        else if(vir.roll<-pi){
            vir.roll = vir.roll + 2*pi;
        }


        if(initial == true && aruco_detect == false){                               //open loop motion
            pose.coordinate_frame= mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            double t = ros::Time::now().toSec();

            if((t-begin)<=10){                                                      //local frame:x:+ 2 m(foward)
                pose.velocity.x = 0.2;
                pose.velocity.y = 0;
                pose.velocity.z = 0;
                pose.yaw_rate = 0;
                /*ROS_INFO("loop1 ");*/
             }
            else {
                finishLoop1 = true;
                initial = false;                                  
                begin = ros::Time::now().toSec();
            }
        }

        else if(finishLoop1 == true && aruco_detect == false){
            pose.coordinate_frame= mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            double t = ros::Time::now().toSec();

            if((t-begin)<=5){                                                       //local frame:y:-1 m(right)
                pose.velocity.x = 0;
                pose.velocity.y = -0.2;
                pose.velocity.z = 0;
                pose.yaw_rate = 0;
                /*ROS_INFO("loop2 ");*/
            }
            else {
                //finishLoop2 = true;
                finishLoop1 = false;
                vir.x = host_mocap.pose.position.x;
                vir.y = host_mocap.pose.position.y;				                     //hover
                vir.z = host_mocap.pose.position.z;
                //begin = ros::Time::now().toSec();
            }
        }


        else if(landing2 == true){                                                   //"landing2" represent a landing flag 
            break;
        }

        else{
            follow(vir,host_mocap,tag_latest,&pose);                                 //closed loop motion
        }

        pose.header.stamp=ros::Time::now();
        bf_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

    }//while

    while(ros::ok()){
        pose.coordinate_frame= mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        double t = ros::Time::now().toSec();

        if((t-begin)<=7){                                    						//landing (z:-1.4 m)
            pose.velocity.x = 0;
            pose.velocity.y = 0;
            pose.velocity.z = -0.2;
            pose.yaw_rate = 0;
        /*ROS_INFO("land ");*/
        }
        else {
            offb_set_mode.request.custom_mode = "MANUAL";
            set_mode_client.call(offb_set_mode);									//stop
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
            break;
        }
        pose.header.stamp=ros::Time::now();
        bf_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}