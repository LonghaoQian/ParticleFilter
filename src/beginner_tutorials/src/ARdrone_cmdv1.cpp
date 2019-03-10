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
#include <termios.h>
#include <sstream>

//
ardrone_autonomy::Navdata navdata;
int keyboard_input;
std_msgs::Empty takeoff_;
std_msgs::Empty landing_;
std_msgs::Empty reset_;
geometry_msgs::Twist command_;
//
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
                {ROS_INFO("-----");}
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
    command_.linear.x = 0;
    command_.linear.y = 0;
    command_.linear.z = 0;
    command_.angular.x = 0;
    command_.angular.y = 0;
    command_.angular.z = 0;

}

//
void ReceiveNavdata(const ardrone_autonomy::Navdata& Navmsg)
{
    navdata = Navmsg; // update navdata
    //ROS_INFO("the drone state is [%d]", navdata.state);
}

void KeybordEvent(char buff, ros::Publisher& pubTakeoff, ros::Publisher& pubLand,ros::Publisher& pubReset, ros::Publisher& pubMove)
{
    if(keyboard_input>-1)//if keyboard is
    {
        ROS_INFO("sending command [%d]", buff);
        switch(buff)
               {case 116: if(navdata.state==2){
                pubTakeoff.publish(takeoff_);
                ROS_INFO("TAKE_OFF");
                     }
                     break;
                case 32: pubLand.publish(landing_);
                     ROS_INFO("LANDING");
                     break;
                case 114: pubReset.publish(reset_);

              }

    }else
    { // do nothing if keyboard is not pressed
    }
    if ((navdata.state==3)||(navdata.state==4))// if the drone is flying
    {
        if(keyboard_input>-1) // if the keyboard is pressed, set command according to the keyboard
        {
            ROS_INFO("sending Movment command [%d]", buff);
            switch(buff)
            {
                case 119: command_.linear.x = 1;//forward
                    break;
                case 115: command_.linear.x = -1;//backward
                    break;
                case 97: command_.linear.y = 1;//left
                    break;
                case 100: command_.linear.y = -1;//right
                    break;
                case 113: command_.angular.z = 1;//yaw left
                    break;
                case 101: command_.angular.z = -1;//yaw right
                    break;
                case 102: command_.linear.z = 1;//acsend
                    break;
                case 118: command_.linear.z = -1;//decsend
                    break;
            }
        }else // if the keyboard is not pressed, set command to zero.
        {
            command_.linear.x = 0;
            command_.linear.y = 0;
            command_.linear.z = 0;
            command_.angular.x = 0;
            command_.angular.y = 0;
            command_.angular.z = 0;

        }
        pubMove.publish(command_); // publish command
    }



    //when keyboard is not pressed, clear all the movement command

}


int main(int argc, char **argv)
{
  Initialization();
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
  ROS_INFO("Open publisher to recieve drone feed");
// takeoff command//
  ros::Publisher pubTakeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
 // Landing command
  ros::Publisher pubLand    = n.advertise<std_msgs::Empty>("ardrone/land", 10);
  // Reset command
  ros::Publisher pubReset   = n.advertise<std_msgs::Empty>("ardrone/reset", 10);
  // Send command
  ros::Publisher pubMove    = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);


// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(20);
// %EndTag(LOOP_RATE)%

  while (ros::ok())
  {
      // get keyboard command
      int c = 0;
      c=getch();
      // perform keyboard command
      KeybordEvent(c,pubTakeoff,  pubLand, pubReset, pubMove);

      ROS_INFO("keyboard command is [%d]", keyboard_input);

      ros::spinOnce();// do the loop once

      loop_rate.sleep();

  }
  pubLand.publish(landing_);// after the
  return 0;
}
// %EndTag(FULLTEXT)%

