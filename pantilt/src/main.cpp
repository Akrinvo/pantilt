#include<ros/ros.h>
#include<std_msgs/String.h>
#include<vector>
#include<iostream>
#include <stdio.h>
#include <string.h>
#include <bits/stdc++.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>



using namespace std;

string command ;
int  serial_port = open("/dev/ttyACM0", O_RDWR);
struct termios tty;

void callback(const std_msgs::String::ConstPtr &msg)
{
    
       command = msg->data;
       cout << command << endl;
       char commandChar[command.length()+1];

       strcpy(commandChar,command.c_str());
       vector<uint8_t> vectorChar(commandChar , commandChar + command.length()+1);
       //vectorChar.push_back(command);
       write(serial_port,vectorChar.data(),vectorChar.size());


}


bool serialSetup()
{


 if(tcgetattr(serial_port, &tty) != 0) {
       printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
             return 1;
   }
  //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 5;
  tty.c_cc[VMIN] = 0;
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

 


 

  return 0;

}













int main(int argc , char** argv)
{
    ros::init(argc,argv,"pantilt");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("keyPress",1,callback);


    ros::Rate rate(10);
    ros::spin();

return 0;
}