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
//
int keyboard_input;
//std_msgs::Empty takeoff_;
//std_msgs::Empty landing_;
//std_msgs::Empty reset_;
//geometry_msgs::Twist command_;


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
    //OptiTrack Messages
public:
    void Initialize(ros::NodeHandle& n);
    // Basic Command Functions
    void TakeOFF();
    void Land();
    void Reset();
    void VelocityCommandUpdate(double vx,double vy,double vz,double az);
    void Stop();
    void GetDroneState();
    double GetDroneYaw();
    int ExternalCommand(int com);// based on external command
    /*
     * com = -1 0 1 2 3 4
    */

};


void ARDroneCommand::Initialize(ros::NodeHandle& n)
{
    //Advertise Publisher
    pubTakeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    pubLand    = n.advertise<std_msgs::Empty>("ardrone/land", 1);
    pubReset   = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
    pubMove    = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    subNavdata = n.subscribe("ardrone/navdata", 1, this->ReceiveNavdata);
    State = 0;
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
    pubMove.publish(command_);
}
void ARDroneCommand::Stop()
{
    command_.linear.x = 0;
    command_.linear.y = 0;
    command_.linear.z = 0;
    command_.angular.z = 0;
    pubMove.publish(command_);
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
int ARDroneCommand::ExternalCommand(int com)
{

    switch(com)
    {
    case -1:
        break;
    case 0:
        break;

    }
    return 0;
}

double ARDroneCommand::GetDroneYaw()
{
    return navdata.rotZ;
}

void ARDroneCommand::ReceiveNavdata(const ardrone_autonomy::Navdata& Navmsg)
{
        navdata = Navmsg; // update navdata
}
ardrone_autonomy::Navdata ARDroneCommand::navdata; //declare the static variable
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
    file.open(filename);
    file.close();
}
void DataRecorder::StartRecording()
{
    state = 1;
    file.open(filename, std::ofstream::out | std::ofstream::app);
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

void KeybordEvent(char buff,ARDroneCommand& comchannel, SignalGenerator& signal, DataRecorder& recorder)
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
                        ROS_INFO("Land");
                    }
                     break;
                /*r*/case 114: comchannel.Reset();
                     break;
                /*w*/case 119:
                     {
                        signal.Start();
                        recorder.StartRecording();
                        ROS_INFO("Start Test");
                     }
                     break;
                /*s*/case 115:
                     {
                        signal.Stop();
                        recorder.StopRecording();
                        ROS_INFO("End Test");
                     }
                     break;
                /*q*/case 113:{
                               comchannel.GetDroneState();
                               signal.GetSignalStatus();
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
    Initialization();
    double Control_Rate = 40;// Hz the rate
    double SquareWaveTime = 20;// Time for the signal generator
    double SquareWaveAmplitude = 0.3;//m/s amplitude for square waves
    double SquareWaveFrequency = 0.25;//Frequency of the square wave
    double SampleNumber  = Control_Rate *SquareWaveTime;
    double SignalOutput = 0;
    sig_1.Initialize(SquareWaveTime,SquareWaveAmplitude,SquareWaveFrequency,Control_Rate);
  // Initialize ros node
    ros::init(argc, argv, "ARdronePID");
    ros::NodeHandle n;
  // ARDrone class
    ARDroneCommand comchannel_1;
    comchannel_1.Initialize(n);
    // Initialize Data Recorder
    DataRecorder psi_recorder;
    psi_recorder.Initialize("psi_data.txt",2,Control_Rate);
    double data[2];
// Set Ros Excution Rate
    ros::Rate loop_rate(Control_Rate);


  while (ros::ok())
  {
      //////////////////////////////// Fix this part with QT library later /////////////////////
      // get keyboard command
      int c = 0;
      c=getch();
      // perform keyboard command
      KeybordEvent(c, comchannel_1, sig_1,psi_recorder);
      //////////////////////////////// Fix this part with QT library later /////////////////////
      SignalOutput = sig_1.RunTimeUpdate();
      comchannel_1.VelocityCommandUpdate(0,0,0,SignalOutput);
      if(sig_1.GetSignalStatus()==0)
      {
          psi_recorder.StopRecording();
      }
      //ROS_INFO("Signal is [%f]", SignalOutput);

      ros::spinOnce();// do the loop once

      //save data
      data[0] = SignalOutput;
      data[1] = comchannel_1.GetDroneYaw();
      psi_recorder.RecorderUpdate(data);

      // loop wait
      loop_rate.sleep();

  }
  //pubLand.publish(landing_);// after the
  return 0;
}

