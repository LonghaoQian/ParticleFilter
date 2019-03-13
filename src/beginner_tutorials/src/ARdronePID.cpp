 #include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "ardrone_autonomy/Navdata.h"
#include <termios.h>
#include <sstream>
#include <fstream>
#include <iostream>
using namespace std;
//#include <QKeyEvent>  will fix the later
# define M_PI       3.14159265358979323846  /* pi */
# define windowsize 3
# define windowsize_ARdrone 5
//
int keyboard_input;


struct opitrack_velocity{
    double vx;
    double vy;
    double vz;
    double omega_x;
    double omega_y;
    double omega_z;
    double time_stamp;
};

struct opitrack_pose{
    double x;
    double y;
    double z;
    double q0;
    double q1;
    double q2;
    double q3;
    double t;
};


////////////////////// Signal Generator ////////////////////////////
class SignalGenerator{
    int    signalflag;            //signal generator running flag
    double signalrunTime;      //signal running time
    double signalfrequency;    //signal frequency
    double signalamplitude;    //signal amplitude
    double signalduration;     //signal duration
    double timeincrement;      //time increment based on the rosrate
    double angularrate;        //angular velocity of the signal
    double signalvalue;        //signal output value
public:
    void Initialize(double SquareDuration,double SquareWaveAmplitude,double SquareWaveFrequency, double Rosrate);
    void Stop();
    void Start();
    int GetSignalStatus();
    double RunTimeUpdate();

};
void SignalGenerator::Initialize(double SquareDuration,double SquareWaveAmplitude,double SquareWaveFrequency, double Rosrate)
{
    signalflag = 0;
    signalrunTime =0;
    signalduration = SquareDuration;
    signalfrequency = SquareWaveFrequency;
    signalamplitude = SquareWaveAmplitude;
    timeincrement = 1/Rosrate;
    angularrate = 2*M_PI*signalfrequency;

}
void SignalGenerator::Start()
{
    signalflag = 1;
    signalrunTime =0;
}


void SignalGenerator::Stop()
{
       signalflag = 0;
       signalflag = 0;
}

double SignalGenerator::RunTimeUpdate()
{
    if(signalflag == 1)//if the generator is running
    {
        if (signalrunTime<signalduration)
        {
            double signal = sin(angularrate*signalrunTime);//use sine wave to generate square wave
            if (signal>0)//normal
            {
                signalvalue = signalamplitude;
            }else
            {
                signalvalue = -signalamplitude;
            }
                signalrunTime+=timeincrement;
        }else// signal end, stop the output
        {
            signalflag = 0;
        }

    }else{
        signalvalue = 0;// if the generator stops
    }
    return signalvalue;
}
int SignalGenerator::GetSignalStatus()
{
    switch(signalflag)
    {
    case 1:
        //ROS_INFO("Signal generator is running");
        break;
    case 0:
        //ROS_INFO("Signal generator has stopped");
        break;
    }
    return signalflag;
}
///////////////////////////// ARDrone Command//////////////////////////////////
class ARDroneCommand{
    int State; // command state
    // publishers
    ros::Publisher pubTakeoff;  //Takeoff command
    ros::Publisher pubLand;     // Landing command
    ros::Publisher pubReset;    // Reset command
    ros::Publisher pubMove;     // Move command
    // subscribers
    ros::Subscriber subNavdata; // Drone Nav Data
    // Command Messages
    std_msgs::Empty takeoff_;
    std_msgs::Empty landing_;
    std_msgs::Empty reset_;
    geometry_msgs::Twist command_;
    // Nav Messages
    static ardrone_autonomy::Navdata navdata;
    static void ReceiveNavdata(const ardrone_autonomy::Navdata& Navmsg);
    // Variables and Functions for yaw rate calculation
    double delta_T;
    double yaw_angle[2];
    double yaw_rate_raw[windowsize_ARdrone];
    double yaw_rate_filtered;
    void CalculateYawRateFromYawAngle();// calculate velocity info from pose update measurements
    void MovingWindowAveraging();// a filter using moving window
    void PushRawYawRate(double& newyawrate);// push newly measured velocity into raw velocity buffer
    void PushYawAngle();//push newly measured pose into dronepose buffer
public:
    void Initialize(ros::NodeHandle& n,double Controlrate);
    // Basic Command Functions
    void TakeOFF();
    void Land();
    void Reset();
    void VelocityCommandUpdate(double vx,double vy,double vz,double az);
    void Stop();
    void GetDroneState();//Get drone flying state
    double GetDroneYaw();
    double GetYawRate();
    double GetRawYawRate();
    void RosWhileLoopRun();// based on external command
    double AngularError(double reference, double state);
    double Rad2Deg(double rad);
    double Deg2Rad(double rad);
    /*
     * com = -1 0 1 2 3 4
    */

};

void ARDroneCommand::Initialize(ros::NodeHandle& n,double Controlrate)
{
    //Advertise Publisher
    pubTakeoff   = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    pubLand      = n.advertise<std_msgs::Empty>("ardrone/land", 1);
    pubReset     = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
    pubMove      = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    subNavdata   = n.subscribe("ardrone/navdata", 1, this->ReceiveNavdata);
    State = 0;
    for(int i=0;i<windowsize_ARdrone;i++)
    {
        yaw_rate_raw[i] = 0;
    }
    yaw_angle[0] = 0;
    yaw_angle[1] = 0;
    yaw_rate_filtered = 0;
    delta_T = 1/Controlrate;
}
void ARDroneCommand::Land()
{
    pubLand.publish(landing_);
}
void ARDroneCommand::TakeOFF()
{
    //This function will only perform take_off when the navdata.state ==2 (Landed)
    if(navdata.state==2)
    {pubTakeoff.publish(takeoff_);}
}
void ARDroneCommand::Reset()
{pubReset.publish(reset_);}
void  ARDroneCommand::VelocityCommandUpdate(double vx,double vy,double vz,double az)
{
    command_.linear.x = vx;
    command_.linear.y = vy;
    command_.linear.z = vz;
    command_.angular.z = az;
}
void ARDroneCommand::Stop()
{
    command_.linear.x = 0;
    command_.linear.y = 0;
    command_.linear.z = 0;
    command_.angular.z = 0;
}
void ARDroneCommand::GetDroneState()
{
    /* 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
       6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
    Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)*/
    switch(navdata.state)
    {
    case 0:
        ROS_INFO("Unknown");
        break;
    case 1:
        ROS_INFO("Init");
        break;
    case 2:
        ROS_INFO("Landed");
        break;
    case 3:
        ROS_INFO("Flying");
        break;
    case 4:
        ROS_INFO("Hovering");
        break;
    case 5:
        ROS_INFO("Test");
        break;
    case 6:
        ROS_INFO("Test");
        break;
    case 7:
        ROS_INFO("Goto Fix Point");
        break;
    case 8:
        ROS_INFO("Landing");
        break;
    case 9:
        ROS_INFO("Looping");
        break;
    }
    ROS_INFO("Battery precent is: [%f]",navdata.batteryPercent);
}
void ARDroneCommand::RosWhileLoopRun()
{
    //Do the velocity calculation
    CalculateYawRateFromYawAngle();
    // push moving command to drone
    pubMove.publish(command_);

}

double ARDroneCommand::GetDroneYaw()
{
    return navdata.rotZ; // return yaw angle in magnetometer (mag change it to yaw_angle[1])
}


void ARDroneCommand::PushYawAngle()
{
    yaw_angle[0] = yaw_angle[1];
    yaw_angle[1] = navdata.rotZ;
}
void ARDroneCommand::PushRawYawRate(double &newyawrate)
{
    /* Logic:
     * a(i-1) = a(i), i = 2...windowsize
     * should fristly start from  i = 2. a(1) = a(2); a(2) = a(3);....; a(N-1) = a(N)
     * secondly a(N) = a_new
    */
    for(int i = 1;i<windowsize_ARdrone;i++)//first step
    {
        yaw_rate_raw[i-1] = yaw_rate_raw[i];
    }
    yaw_rate_raw[windowsize_ARdrone-1] = newyawrate;// second step update the last variable in the velocity buffer
}

void ARDroneCommand::MovingWindowAveraging()
{
    /* Logic: Average the raw velocity measurement in the
    */
    double weight = (double)1/windowsize_ARdrone;// the weight on each velocity to be summed up.
    // create a temporary variable to store the summed velocity and initialize it witht the 1st buffer value
    double velocitytemp;
    velocitytemp = weight*yaw_rate_raw[0];

    for(int i = 1;i<windowsize_ARdrone;i++)// sum starts from the second buffer value
    {
        velocitytemp += weight*yaw_rate_raw[i];
    }
    // the filtered vlocity is just the weighted summed result
    yaw_rate_filtered = velocitytemp;
}

void ARDroneCommand::CalculateYawRateFromYawAngle()
{
    PushYawAngle();
    double yaw_rate_onestep;



    yaw_rate_onestep = Rad2Deg(AngularError(Deg2Rad(yaw_angle[1]), Deg2Rad(yaw_angle[0])))/delta_T;
    PushRawYawRate(yaw_rate_onestep);
    MovingWindowAveraging();
}
double  ARDroneCommand::GetYawRate()
{
    return yaw_rate_filtered;
}
double  ARDroneCommand::GetRawYawRate()
{
    return yaw_rate_raw[windowsize_ARdrone-1];
}

double ARDroneCommand::AngularError(double reference, double state)
{
    // only accepts rad
    double cr,sr;
    double cs,ss;
    cr = cos(reference);
    sr = sin(reference);
    cs = cos(state);
    ss = sin(state);
    // const
    double xr[2];
    xr[0] = cr;
    xr[1] = sr;
    double x[2];
    x[0] = cs;
    x[1] = ss;
    double y[2];
    y[0] = -ss;
    y[1] = cs;
    double xrs[2];// calculate the reference vector in the rotated coordinate
    xrs[0] = xr[0]*x[0]+xr[1]*x[1];
    xrs[1] = xr[0]*y[0]+xr[1]*y[1];
    return atan2(xrs[1],xrs[0]);
}
double  ARDroneCommand::Rad2Deg(double rad)
{
    return 180/M_PI*rad;
}
double  ARDroneCommand::Deg2Rad(double deg)
{
    return deg/180*M_PI;
}
void ARDroneCommand::ReceiveNavdata(const ardrone_autonomy::Navdata& Navmsg)
{
        navdata = Navmsg; // update navdata
}
ardrone_autonomy::Navdata ARDroneCommand::navdata; //declare the static variable
///////////////////////////// OptiTrack ////////////////////////////////
class OptiTrackFeedback{
    ros::Subscriber subOptiTrack;// OptiTrack Data
    static geometry_msgs::PoseStamped OptiTrackdata;
    static unsigned int OptiTrackFlag; // OptiTrackState 0: no data feed,: 1 data feed present
    unsigned int FeedbackState;// 0 no feedback, 1 has feedback
    static void OptiTrackCallback(const geometry_msgs::PoseStamped& msg);
    opitrack_velocity dronevelocity_raw[windowsize];// raw velocity buffer from numerical differentiation
    opitrack_velocity dronevelocity_filtered;// filtered velocity
    opitrack_pose     dronepose[2];// pose info from optitrack dronepose[1] should be the newly mesaured value, dronepose[0] is value of the last measurment (in world frame by default, if other frames
    // are used , please changle the frame selectioin in the launch file
    void CalculateVelocityFromPose();// calculate velocity info from pose update measurements
    void MovingWindowAveraging();// a filter using moving window
    void PushRawVelocity(opitrack_velocity& newvelocity);// push newly measured velocity into raw velocity buffer
    void PushPose();//push newly measured pose into dronepose buffer
    void SetZeroVelocity();
public:
    void Initialize(ros::NodeHandle& n);
    int GetOptiTrackState();
    opitrack_velocity GetVelocity();
    opitrack_velocity GetRaWVelocity();
    opitrack_pose GetPose();
    void RosWhileLoopRun();// This function should be put into ros while loop
    void GetEulerAngleFromQuaterion_NormalConvention(double (&eulerangle)[3]);
    void GetEulerAngleFromQuaterion_OptiTrackYUpConvention(double (&eulerangle)[3]);
};

void OptiTrackFeedback::Initialize(ros::NodeHandle& n)
{
    subOptiTrack = n.subscribe("vrpn_client_node/ARdrone/pose", 1, this->OptiTrackCallback);
    //Initialize all velocity
    for(int i =0;i<windowsize;i++)
    {
        dronevelocity_raw[i].omega_x = 0;
        dronevelocity_raw[i].omega_y = 0;
        dronevelocity_raw[i].omega_z = 0;
        dronevelocity_raw[i].time_stamp = 0;
        dronevelocity_raw[i].vx=0;
        dronevelocity_raw[i].vy=0;
        dronevelocity_raw[i].vz=0;
    }
    dronevelocity_filtered.omega_x=0;
    dronevelocity_filtered.omega_y=0;
    dronevelocity_filtered.omega_z=0;
    dronevelocity_filtered.time_stamp=0;
    dronevelocity_filtered.vx=0;
    dronevelocity_filtered.vy=0;
    dronevelocity_filtered.vz=0;
    //Initialize all pose
    for(int i = 0;i<2;i++)
    {
        dronepose[i].q0 = 1;
        dronepose[i].q1 = 0;
        dronepose[i].q2 = 0;
        dronepose[i].q3 = 0;
        dronepose[i].t = 0;
        dronepose[i].x = 0;
        dronepose[i].y = 0;
        dronepose[i].z = 0;

    }
    // Initialize flag
    OptiTrackFlag = 0;
    FeedbackState = 0;
}
void OptiTrackFeedback::CalculateVelocityFromPose()
{

    /* Logic:
     * 1) push the current pose into buffer
     * 2) determine whether the buffer has a valid time value  (dronepose[0].t >0); if so calculate velocity
     * 3) if not just set the velocity_onestep as zero
     * 4) push current time, and velocity_onestep into the velocity buffer
     * 5) calculate filtered velocity
    */
    // perform the Logic:
    // step (1): push the current pose into buffer
    PushPose();
    // step (2): determine whether the buffer has a valid time value  (dronepose[0].t >0); if so calculate velocity
    double dt = 0.0;
    opitrack_velocity velocity_onestep;
  if (dronepose[0].t >0)// calculate only when last time stamp has been recorded.
  {
      // step (2)
      dt = dronepose[1].t - dronepose[0].t;// time step
      // calculate x direction velocity
      velocity_onestep.vx = (dronepose[1].x - dronepose[0].x)/dt;
      velocity_onestep.vy = (dronepose[1].y - dronepose[0].y)/dt;
      velocity_onestep.vz = (dronepose[1].z - dronepose[0].z)/dt;
      // will add rotation speed later
      velocity_onestep.omega_x = 0;
      velocity_onestep.omega_y = 0;
      velocity_onestep.omega_z = 0;
  }else// step (3): if not set velocity to zero and only record the time
  {
      velocity_onestep.vx = 0.0;
      velocity_onestep.vy = 0.0;
      velocity_onestep.vz = 0.0;
      velocity_onestep.omega_x = 0;
      velocity_onestep.omega_y = 0;
      velocity_onestep.omega_z = 0;
      // will add rotation speed later
  }
  velocity_onestep.time_stamp = dronepose[1].t;
  // step (4): push current time, and velocity_onestep into the velocity buffer
  PushRawVelocity(velocity_onestep);
  // step (5): calculate filtered velocity
  MovingWindowAveraging();
}
void OptiTrackFeedback::PushPose()
{
    dronepose[0] = dronepose[1];// straightforward push the pose into buffer
    // update the latest pose
    double t_current = (double)OptiTrackdata.header.stamp.sec + (double)OptiTrackdata.header.stamp.nsec*0.000000001;
    dronepose[1].t = t_current;
    // take a special note at the order of the quaterion
    dronepose[1].q0 = OptiTrackdata.pose.orientation.w;
    dronepose[1].q1 = OptiTrackdata.pose.orientation.x;
    dronepose[1].q2 = OptiTrackdata.pose.orientation.y;
    dronepose[1].q3 = OptiTrackdata.pose.orientation.z;
    // pose is straight forward
    dronepose[1].x =  OptiTrackdata.pose.position.x;
    dronepose[1].y =  OptiTrackdata.pose.position.y;
    dronepose[1].z =  OptiTrackdata.pose.position.z;
}

void OptiTrackFeedback::PushRawVelocity(opitrack_velocity& newvelocity)
{
    /* Logic:
     * a(i-1) = a(i), i = 2...windowsize
     * should fristly start from  i = 2. a(1) = a(2); a(2) = a(3);....; a(N-1) = a(N)
     * secondly a(N) = a_new
    */
    for(int i = 1;i<windowsize;i++)//first step
    {
        dronevelocity_raw[i-1] = dronevelocity_raw[i];
    }
    dronevelocity_raw[windowsize-1] = newvelocity;// second step update the last variable in the velocity buffer
}

void OptiTrackFeedback::MovingWindowAveraging()
{

    /* Logic: Average the raw velocity measurement in the
    */
    double weight = (double)1/windowsize;// the weight on each velocity to be summed up.
    // create a temporary variable to store the summed velocity and initialize it witht the 1st buffer value
    opitrack_velocity velocitytemp;
    velocitytemp.omega_x= weight*dronevelocity_raw[0].omega_x;
    velocitytemp.omega_y= weight*dronevelocity_raw[0].omega_y;
    velocitytemp.omega_z= weight*dronevelocity_raw[0].omega_z;
    velocitytemp.vx = weight*dronevelocity_raw[0].vx;
    velocitytemp.vy = weight*dronevelocity_raw[0].vy;
    velocitytemp.vz = weight*dronevelocity_raw[0].vz;

    for(int i = 1;i<windowsize;i++)// sum starts from the second buffer value
    {
        velocitytemp.omega_x+= weight*dronevelocity_raw[i].omega_x;
        velocitytemp.omega_y+= weight*dronevelocity_raw[i].omega_y;
        velocitytemp.omega_z+= weight*dronevelocity_raw[i].omega_z;
        velocitytemp.vx += weight*dronevelocity_raw[i].vx;
        velocitytemp.vy += weight*dronevelocity_raw[i].vy;
        velocitytemp.vz += weight*dronevelocity_raw[i].vz;
    }
    velocitytemp.time_stamp = dronevelocity_raw[windowsize-1].time_stamp;
    // the filtered vlocity is just the weighted summed result
    dronevelocity_filtered = velocitytemp;
}

opitrack_pose OptiTrackFeedback::GetPose()
{
    return dronepose[1];//return the latest pose
}

opitrack_velocity OptiTrackFeedback::GetVelocity()
{
    return dronevelocity_filtered;// return the filtered velocity
}
opitrack_velocity OptiTrackFeedback::GetRaWVelocity()
{
    return dronevelocity_raw[windowsize-1];// return the filtered velocity
}
void  OptiTrackFeedback::SetZeroVelocity()
{
    for(int i =0;i<windowsize;i++)
    {
        dronevelocity_raw[i].omega_x = 0;
        dronevelocity_raw[i].omega_y = 0;
        dronevelocity_raw[i].omega_z = 0;
        dronevelocity_raw[i].vx=0;
        dronevelocity_raw[i].vy=0;
        dronevelocity_raw[i].vz=0;
    }
    dronevelocity_filtered.omega_x=0;
    dronevelocity_filtered.omega_y=0;
    dronevelocity_filtered.omega_z=0;
    dronevelocity_filtered.vx=0;
    dronevelocity_filtered.vy=0;
    dronevelocity_filtered.vz=0;
}

void OptiTrackFeedback::RosWhileLoopRun()
{
    if(OptiTrackFlag==1)
    {// update the velocity only when there is OptiTrack feedback
        CalculateVelocityFromPose();
        FeedbackState=1;
    }else{
        // if the optitrack measurements no longer feedback, when the pose update will stop and we only return 0 velocity
        SetZeroVelocity();
        FeedbackState=0;
    }

    OptiTrackFlag = 0;// reset the feedback flag to 0
}
int OptiTrackFeedback::GetOptiTrackState()
{
    return FeedbackState;
}
void OptiTrackFeedback::GetEulerAngleFromQuaterion_NormalConvention(double (&eulerangle)[3])
{


    /* Normal means the following https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    */
//    eulerangle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
//    eulerangle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
//    eulerangle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));


    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (dronepose[1].q0 * dronepose[1].q1 + dronepose[1].q2 * dronepose[1].q3);
    double cosr_cosp = +1.0 - 2.0 * (dronepose[1].q1 * dronepose[1].q1 +dronepose[1].q2 * dronepose[1].q2);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (dronepose[1].q0 * dronepose[1].q2 - dronepose[1].q3 * dronepose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (dronepose[1].q0 * dronepose[1].q3 + dronepose[1].q1 * dronepose[1].q2);
    double cosy_cosp = +1.0 - 2.0 * (dronepose[1].q2 * dronepose[1].q2 + dronepose[1].q3 * dronepose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    //double yaw  = atan2(2.0 * (dronepose[1].q3 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q2), -1.0 + 2.0 * (dronepose[1].q0 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q1));
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedback::GetEulerAngleFromQuaterion_OptiTrackYUpConvention(double (&eulerangle)[3])
{

    // OptiTrack gives a quaternion with q2 and q3 flipped.
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (dronepose[1].q0 * dronepose[1].q1 + dronepose[1].q3 * dronepose[1].q2);
    double cosr_cosp = +1.0 - 2.0 * (dronepose[1].q1 * dronepose[1].q1 +dronepose[1].q3 * dronepose[1].q3);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (dronepose[1].q0 * dronepose[1].q3 - dronepose[1].q2 * dronepose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (dronepose[1].q0 * dronepose[1].q2 + dronepose[1].q1 * dronepose[1].q3);
    double cosy_cosp = +1.0 - 2.0 * (dronepose[1].q2 * dronepose[1].q2 + dronepose[1].q3 * dronepose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedback::OptiTrackCallback(const geometry_msgs::PoseStamped& msg)
{
        OptiTrackdata = msg; // update optitrack data
        OptiTrackFlag = 1;// signal a new measurement feed has been revcieved.
}
geometry_msgs::PoseStamped OptiTrackFeedback::OptiTrackdata;//declare optitrack data
unsigned int OptiTrackFeedback::OptiTrackFlag;// optitrack flag
////////////////////////////// Incremental PID /////////////////////////////
class Incremental_PID{
    /* single channel digital pid controller
     * in the initialiation function, put kp ki and kd as for a continous PID, the initializer will automatically convert them into
     * corresponding digital PID gains
    */
    double delta_T;
    double kp;
    double ki;
    double kd;
    double upperlimit;
    double lowerlimit;
    double u_k;// output u(k)
    double e_k;//e(k)
    double e_k_1;//e(k-1)
    double e_k_2;//e(k-2)
    void PID_update(double e);// control output update
    unsigned int pid_flag;//  flag 0: stop  1: running 2; fault
public:
    void Initialize(double controlrate, double kp, double kd, double ki, double upperlimit, double lowerlimit);//
    void Start();//start the controller
    void Stop(); //stop the controller
    void Reset(); //reset the contoller
    unsigned int GetPIDState();
    void RosWhileLoopRun(double e);// in ros while loop update the controller e is the PID input and u is the output
    double GetPIDOutPut();
    // some utility tools to help calculate errors
    double LinearError(double reference, double state);
    double AngularError(double reference, double state);// only accepts rad
    double Rad2Deg(double rad);
    double Deg2Rad(double deg);

};
void Incremental_PID::Initialize(double controlrate, double kp_, double kd_, double ki_, double upperlimit_, double lowerlimit_)
{
    delta_T = 1/controlrate;
    // digital gains
    kp = kp_;
    ki = ki_*delta_T;
    kd = kd_/delta_T;
    upperlimit = upperlimit_;
    lowerlimit = lowerlimit_;
    //
    u_k = 0;
    e_k = 0;
    e_k_1 = 0;
    e_k_2 = 0;
}
void Incremental_PID::Start()
{
    pid_flag = 1;
}
void Incremental_PID::Stop()
{
    pid_flag = 0;
}
void Incremental_PID::Reset()
{
    pid_flag = 0;
    u_k = 0;
    e_k = 0;
    e_k_1 = 0;
    e_k_2 = 0;
}
void Incremental_PID::PID_update(double e)
{
    // update errors
    e_k_2 = e_k_1;
    e_k_1 = e_k;
    e_k = e;
    // calculate current output
    double delta_u;// calculate output increment
    delta_u = kp*(e_k-e_k_1)+ki*e_k+kd*(e_k-2*e_k_1+e_k_2);
    u_k += delta_u;
}
void Incremental_PID::RosWhileLoopRun(double e)
{

    switch(pid_flag)
    {
    case 0:
        break; // if stopped, then do nothing
    case 1: {
        // if all normal run update
                PID_update(e);
    }
        break;
    case 2: {
        PID_update(e);// still do update
    }
        break;

    }
}
unsigned int Incremental_PID::GetPIDState()
{
    return pid_flag;// return flag as state
}
double Incremental_PID::LinearError(double reference, double state)
{
    return reference-state;
}
double Incremental_PID::AngularError(double reference, double state)
{
    // only accepts rad
    double cr,sr;
    double cs,ss;
    cr = cos(reference);
    sr = sin(reference);
    cs = cos(state);
    ss = sin(state);
    // const
    double xr[2];
    xr[0] = cr;
    xr[1] = sr;
    double x[2];
    x[0] = cs;
    x[1] = ss;
    double y[2];
    y[0] = -ss;
    y[1] = cs;
    double xrs[2];// calculate the reference vector in the rotated coordinate
    xrs[0] = xr[0]*x[0]+xr[1]*x[1];
    xrs[1] = xr[0]*y[0]+xr[1]*y[1];
    return atan2(xrs[1],xrs[0]);
}
double Incremental_PID::Rad2Deg(double rad)
{
    return 180/M_PI*rad;
}
double Incremental_PID::Deg2Rad(double deg)
{
    return deg/180*M_PI;
}
double Incremental_PID::GetPIDOutPut()
{

    double u= u_k;
    // determine whether the PID has reached a preset bound
    if(u_k-upperlimit>0)
    {
        u = upperlimit;// limit the magnitude
        pid_flag = 2;
    }
    if(lowerlimit-u_k>0)
    {
        u = lowerlimit;// limit the magnitude
        pid_flag = 2;
    }

    return u;
}
////////////////////////////// ARdrone Flight Control /////////////////





///////////////////////////// Keyboard Command ///////////////////////////////
//Get Keyboard Input
char getch()
{
        keyboard_input = -1; //if not key is pressed, then return -1 for the keyboard_input. reset this flag every time
        fd_set set;
        struct timeval timeout;
        int rv;
        char buff = 0;
        int len = 1;
        int filedesc = 0;
        FD_ZERO(&set);
        FD_SET(filedesc, &set);

        timeout.tv_sec = 0;
        //timeout.tv_usec = 1000;//
        timeout.tv_usec = 1000;
        rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

        struct termios old = {0};
        if (tcgetattr(filedesc, &old) < 0)
                ROS_ERROR("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(filedesc, TCSANOW, &old) < 0)
                ROS_ERROR("tcsetattr ICANON");

        if(rv == -1){
                ROS_ERROR("select");
        }
        else if(rv == 0)
        {
                //{ROS_INFO("-----");
        }
        else
                {read(filedesc, &buff, len );
        keyboard_input = 1;}

        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
                ROS_ERROR ("tcsetattr ~ICANON");
        return (buff);
}
// Initialize all the stuff
void Initialization()
{
    keyboard_input = -1;
}

//

class DataRecorder{
    int state;                     //recorder state
    const char* filename;
    unsigned int numbe_of_column;  //dimension of the data
    double delta_T;
    double recordingtime;          // recordingtime
    ofstream file;                 //file object that store the
public:
    void StartRecording();
    void RecorderUpdate(double* data);
    void StopRecording();
    void Initialize(const char* name,unsigned int ud, double controlrate);
};

void DataRecorder::Initialize(const char* name,unsigned int ud, double controlrate)
{
    filename = name;
    numbe_of_column = ud;
    state = 0;
    delta_T = 1/controlrate;
    recordingtime=0;
    //name a file
    file.open(filename, std::ofstream::out);
    file.close();
}
void DataRecorder::StartRecording()
{
    state = 1;
    file.open(filename, std::ofstream::out);
}
void DataRecorder::RecorderUpdate(double* data)
{
    if(state ==1)
    {
        file <<"t--"<<recordingtime<<"--data";
        for(int i=0;i<numbe_of_column;i++)
        {
        file <<"--"<<data[i];
        }
        file <<endl;
        recordingtime+=delta_T;
    }
}
void DataRecorder::StopRecording()
{
    state = 0;
    file.close();
}

void KeybordEvent(char buff,ARDroneCommand& comchannel,
                  SignalGenerator& signal,
                  DataRecorder& recorder,
                  OptiTrackFeedback& optitrack,
                  Incremental_PID& yaw_rate_pid,
                  Incremental_PID& vertical_speed_pid,
                  Incremental_PID& x_speed_pid,
                  Incremental_PID& z_speed_pid)
{
    if(keyboard_input>-1)//if keyboard is
    {
        switch(buff)
               {/*t*/case 116: {
                    comchannel.TakeOFF();
                        ROS_INFO("TakeOFF_command_sent");}
                     break;
            /*Space*/case 32:
                    {
                        signal.Stop();
                        recorder.StopRecording();
                        comchannel.Land();
                        yaw_rate_pid.Stop();
                        vertical_speed_pid.Stop();
                        x_speed_pid.Stop();
                        x_speed_pid.Reset();
                        z_speed_pid.Stop();
                        z_speed_pid.Reset();
                        ROS_INFO("Land");
                    }
                     break;
                /*r*/case 114: {
                        comchannel.Reset();
                        ROS_INFO("Reset_command_sent");
                        comchannel.GetDroneState();
                     }
                     break;
                /*w*/case 119:
                     {
                        signal.Start();
                        recorder.StartRecording();
                        yaw_rate_pid.Start();
                        vertical_speed_pid.Start();
                         x_speed_pid.Start();
                          z_speed_pid.Start();
                        ROS_INFO("Start Test");
                     }
                     break;
                /*s*/case 115:
                     {
                        signal.Stop();
                        recorder.StopRecording();
                        yaw_rate_pid.Stop();
                        yaw_rate_pid.Reset();
                        vertical_speed_pid.Stop();
                        vertical_speed_pid.Reset();
                        x_speed_pid.Stop();
                        x_speed_pid.Reset();
                        z_speed_pid.Stop();
                        z_speed_pid.Reset();
                        ROS_INFO("End Test");
                     }
                     break;
                /*q*/case 113:{
                               comchannel.GetDroneState();
                               signal.GetSignalStatus();
                               if(optitrack.GetOptiTrackState()==1)
                               {
                                   ROS_INFO("OptiTrack Normal");
                               }else{

                                   ROS_INFO("No OptiTrack Data Feed");
                               }

                                }
                     break;
                /*e*/case 101: comchannel.Stop();
                     break;


              }

    }else
    { // do nothing if keyboard is not pressed
    }


}



int main(int argc, char **argv)
{
  //Initialization custom function
    SignalGenerator sig_1;
    SignalGenerator vertial_signal;
    double Control_Rate = 40;// Hz the rate
    double SquareWaveTime = 40;// Time for the signal generator
    double SquareWaveAmplitude = 0.3;//m/s amplitude for square waves
    double SquareWaveFrequency = 0.25;//Frequency of the square wave
    double SampleNumber  = Control_Rate *SquareWaveTime;
    double SignalOutput = 0;
    sig_1.Initialize(SquareWaveTime,SquareWaveAmplitude,SquareWaveFrequency,Control_Rate);
    vertial_signal.Initialize(SquareWaveTime,SquareWaveAmplitude,SquareWaveFrequency,Control_Rate);
  // Initialize ros node
    ros::init(argc, argv, "ARdronePID");
    ros::NodeHandle n;
    // Initialize OptiTrack System
    OptiTrackFeedback OpTiFeedback;
    OpTiFeedback.Initialize(n);
  // ARDrone class
    ARDroneCommand comchannel_1;
    comchannel_1.Initialize(n,Control_Rate);
    // PID control
    Incremental_PID yaw_rate_pid;
    Incremental_PID vertical_speed_pid;
    Incremental_PID horizontal_x_pid;
    Incremental_PID horizontal_z_pid;
    //yaw_rate_pid.Initialize(Control_Rate,3.9/100,0,26/100,0.9,-0.9);
    //yaw_rate_pid.Initialize(Control_Rate,1,0,1,1,-1);
    yaw_rate_pid.Initialize(Control_Rate,5,0,0,1,-1);// kp = 5 1
    vertical_speed_pid.Initialize(Control_Rate,2,0.2,3,1,-1);
    horizontal_x_pid.Initialize(Control_Rate,0.25,0,0,1,-1);
    horizontal_z_pid.Initialize(Control_Rate,0.25,0,0,1,-1);
    // Initialize Data Recorder
    DataRecorder psi_recorder;
    DataRecorder velocity_recorder;
    psi_recorder.Initialize("psi_data.txt",6,Control_Rate);
    velocity_recorder.Initialize("vz_data.txt",4,Control_Rate);
    double data[6];
    double dataz[4];
    double yaw_angle_cmd = -100;
    double yaw_error;
    double vz_error;
    double command_valtitue = 1;
    double command_x = 0;
    double command_z = 0;
    double z_error = 0;
    double euler_optitrack[3];
    double ke = 1;
    double k = 1;
    double s_x_w = 0;
    double s_y_w = 0;
    double s_x_b = 0;
    double s_y_b = 0;
    // x and y poistion error

// Set Ros Excution Rate
    ros::Rate loop_rate(Control_Rate);
    velocity_recorder.StartRecording();
    //double yaw_ref = 70;
  while (ros::ok())
  {
      //////////////////////////////// Fix this part with QT library later /////////////////////
      // get keyboard command
      int c = 0;
      c=getch();
      // perform keyboard command
      KeybordEvent(c, comchannel_1, sig_1,velocity_recorder,OpTiFeedback,yaw_rate_pid,vertical_speed_pid,horizontal_x_pid,horizontal_z_pid);
      //////////////////////////////// Fix this part with QT library later /////////////////////
      SignalOutput = sig_1.RunTimeUpdate();
      // send commands
      comchannel_1.VelocityCommandUpdate(horizontal_x_pid.GetPIDOutPut(),-horizontal_z_pid.GetPIDOutPut(),vertical_speed_pid.GetPIDOutPut(),yaw_rate_pid.GetPIDOutPut());
      data[1] = yaw_rate_pid.GetPIDOutPut();
      comchannel_1.RosWhileLoopRun();// flush the commands and calculations

      //ROS_INFO("Signal is [%f]", SignalOutput);
      ros::spinOnce();// do the loop once
      OpTiFeedback.RosWhileLoopRun();
      // Yaw control loop:
      //yaw_error = SignalOutput*100 - comchannel_1.GetYawRate();// rate control
      yaw_error = comchannel_1.Rad2Deg(comchannel_1.AngularError(comchannel_1.Deg2Rad(yaw_angle_cmd),comchannel_1.Deg2Rad(comchannel_1.GetDroneYaw())));// angle control
      yaw_rate_pid.RosWhileLoopRun(yaw_error/100);// push error into pid
      // Altitude Control Loop
      //vz_error = SignalOutput - OpTiFeedback.GetVelocity().vy;
      z_error =  OpTiFeedback.GetPose().y - command_valtitue;
      //vz_error = z_error + OpTiFeedback.GetVelocity().vy;
      //vz_error = OpTiFeedback.GetVelocity().vy - SignalOutput;
      vertical_speed_pid.RosWhileLoopRun(-2*vz_error);
      //save data
      if(sig_1.GetSignalStatus()==0)
      {
          psi_recorder.StopRecording();
      }
      data[0] = SignalOutput;

      data[2] = comchannel_1.GetYawRate();
      data[3] = comchannel_1.GetRawYawRate();
      data[4] = comchannel_1.GetDroneYaw();
      data[5] = yaw_error;
      psi_recorder.RecorderUpdate(data);

      // z changle loop:
      if(sig_1.GetSignalStatus()==0)
      {
          velocity_recorder.StopRecording();
      }
      dataz[0] = SignalOutput;
      dataz[1] = OpTiFeedback.GetPose().y;
      dataz[2] = OpTiFeedback.GetRaWVelocity().vy;
      dataz[3] = OpTiFeedback.GetVelocity().vy;
      velocity_recorder.RecorderUpdate(dataz);
      // Get body angles
      OpTiFeedback.GetEulerAngleFromQuaterion_OptiTrackYUpConvention(euler_optitrack);
      // Horizontal r
      s_x_w = ke*(OpTiFeedback.GetPose().x -command_x)+OpTiFeedback.GetVelocity().vx;
      s_y_w = ke*(OpTiFeedback.GetPose().z -command_z)+OpTiFeedback.GetVelocity().vz;
      // Transfer velocity and poisiton
      s_x_b = s_x_w*cos(-euler_optitrack[2])+s_y_w*sin(-euler_optitrack[2]);
      s_y_b = -s_x_w*sin(-euler_optitrack[2])+s_y_w*cos(-euler_optitrack[2]);
      horizontal_x_pid.RosWhileLoopRun(-s_x_b);
      horizontal_z_pid.RosWhileLoopRun(-s_y_b);
      //ROS_INFO("y measurement is [%f]", OpTiFeedback.GetPose().y);
      //ROS_INFO("yaw angle is [%f] x position error is [%f] z position error is [%f]",euler_optitrack[2]*57.3,s_x_b,s_y_b);
      // loop wait
      loop_rate.sleep();

  }
  velocity_recorder.StopRecording();
  //pubLand.publish(landing_);// after the
  return 0;
}

