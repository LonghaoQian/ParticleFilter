// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
// %EndTag(MSG_HEADER)%

#include <sstream>


ardrone_autonomy::Navdata navdata;

void ReceiveNavdata(const ardrone_autonomy::Navdata& Navmsg)
{
    navdata = Navmsg; // update navdata
    ROS_INFO("the drone state is [%d]", navdata.state);
}


int main(int argc, char **argv)
{
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
// %Tag(INIT)%
  ros::init(argc, argv, "ARdrone_cmdv1");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// Subscriber block
  ROS_INFO("Open subscriber to recieve drone feed");
  ros::Subscriber subNavdata = n.subscribe("ardrone/navdata", 10, ReceiveNavdata);
// Publisher block
  ROS_INFO("Open subscriber to recieve drone feed");
// takeoff command//
  ros::Publisher pubTakeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
 // Landing command
  ros::Publisher pubLand = n.advertise<std_msgs::Empty>("ardrone/land", 10);
  // Reset command
  ros::Publisher pubReset = n.advertise<std_msgs::Empty>("ardrone/reset", 10);
  // Send command
  ros::Publisher pubMove = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);


// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(20);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  //int count = 0;

  std_msgs::Empty takeoff_;
  std_msgs::Empty landing_;
  std_msgs::Empty reset_;

  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%

  //  switch {

   // }


  //  pubTakeoff.publish(takeoff_);

// %EndTag(PUBLISH)%

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

