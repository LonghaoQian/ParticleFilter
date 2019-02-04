#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"


/*
 *
 * Buffer data structure
*/


struct velocitybuffer{
    double vx;
    double vy;
    double vz;
    double omega_x;
    double omega_y;
    double omega_z;
};

struct posebuffer{
    double x;
    double y;
    double z;
    double q0;
    double q1;
    double q2;
    double q3;
    double t;
};

velocitybuffer velocity[4];
posebuffer pose_last;
// initialize the buffer
void BufferInitialize()
{
   for (int i=0; i<4; i++) {
       velocity[i].vx = 0.0;
       velocity[i].vy = 0.0;
       velocity[i].vz = 0.0;
    }
   pose_last.x = 0.0;
   pose_last.y = 0.0;
   pose_last.z = 0.0;
   pose_last.t = -1.0; // invalide time
  ROS_INFO("Buffer initialized");
}
// Push current data to buffer
void BufferPush(velocitybuffer& velocityupdate, const geometry_msgs::PoseStamped& msg, double& t_current)
{
    // velocity:
    velocity[0] = velocity[1];
    velocity[1] = velocity[2];
    velocity[2] = velocity[3];
    velocity[3] = velocityupdate;
    // pose:
    pose_last.x = msg.pose.position.x;
    pose_last.y = msg.pose.position.y;
    pose_last.z = msg.pose.position.z;
    pose_last.t = t_current;
}
void AverageVelocity(velocitybuffer& average_v)
{
    velocitybuffer vel_temp;
    vel_temp.vx = 0.0;
    vel_temp.vy = 0.0;
    for (int i=0; i<4; i++) {
        vel_temp.vx = vel_temp.vx + 0.25*velocity[i].vx;
        vel_temp.vy = vel_temp.vy + 0.25*velocity[i].vy;
     }
    average_v = vel_temp;
}
/* Logic:
 * 1) determine whether the buffer has a valid time value  (pose_last.t >0)
 * 2) if so calculate dt and dx,..., dz, to get velocity of the current time to velocity_onestep
 * 3) if not just set the velocity_onestep as zero
 * 4) push current time, current pose, and current velocity_onestep into the buffer velocity and pose_last
*/


// Modify the message to geometry_msgs/PoseStamped
void PoseCallback(const geometry_msgs::PoseStamped& msg)
{
  //ROS_INFO("Optitrack Stream Detected: [%f]", msg.pose.position.x);
  ROS_INFO("Optitrack Stream Detected: [%d]", msg.header.stamp.sec);
  velocitybuffer velocity_onestep;
  double dt = 0.0;
  double t_current = (double)msg.header.stamp.sec + (double)msg.header.stamp.nsec*0.000000001;
  // perform the Logic:
   // step (1)
  if (pose_last.t >0)// calculate only when last time stamp has been recorded.
  {
      // step (2)
      dt = t_current - pose_last.t;
      // calculate x direction velocity
      velocity_onestep.vx = (msg.pose.position.x - pose_last.x)/dt;
      velocity_onestep.vy = (msg.pose.position.y - pose_last.y)/dt;
      velocity_onestep.vz = (msg.pose.position.z - pose_last.z)/dt;

  }else// if not set velocity to zero and only record the time
  {
      // step (3)
      velocity_onestep.vx = 0.0;
      velocity_onestep.vy = 0.0;
      velocity_onestep.vz = 0.0;

  }
  // step (4)
  // push the current time and pose into buffer
  BufferPush(velocity_onestep,msg,t_current);
  // display
  velocitybuffer velocity_aver;
  AverageVelocity(velocity_aver);
  ROS_INFO("x velocity true [%f]", velocity_onestep.vx);
  ROS_INFO("x velocity averaged [%f]", velocity_aver.vx);
  ROS_INFO("time last [%f]", pose_last.t);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /*
   * Initialize the buffer
  */
  BufferInitialize();
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "VRPN_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ROS_INFO("Openup listener");
  ros::Subscriber sub = n.subscribe("vrpn_client_node/ARdrone/pose", 1000, PoseCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  //ros::spin();
// %EndTag(SPIN)%
// try using while loop

  ros::Rate loop_rate(10);
  while (ros::ok())
  {

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    //++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

