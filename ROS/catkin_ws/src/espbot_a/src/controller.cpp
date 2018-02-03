#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <sstream>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35

int servo_level;
ros::Publisher servoPosPublisher;
void mainFunction();

int main(int argc, char **argv)
{
  servo_level = 90;
  ros::init(argc, argv, "ESPBot_Controller");
  ros::NodeHandle n;
  servoPosPublisher = n.advertise<std_msgs::Int16>("ServoPositionTopic", 1000);

  std_msgs::Int16 msg;
  msg.data = servo_level;
  servoPosPublisher.publish(msg);

  mainFunction();
  return 0;
}

void mainFunction()
{
  int kfd = 0;
  struct termios cooked, raw;
  char c;
  bool dirty = false;
  int step = 1;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the gate");

  ros::Rate loop_rate(50); // 50 Hz
  while (ros::ok())
  {
    // get the next event from the keyboard
    if (::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    std::ostringstream ss;

    switch (c)
    {
    case KEYCODE_L:
      std::cout << " LEFT" << std::endl;
      if (servo_level >= step)
      {
        servo_level -= step;
        dirty = true;
        ss << "Setting servo to " << servo_level << std::endl;
      }
      else if (servo_level > 0)
      {
        servo_level = 0;
        dirty = true;
        ss << "Setting servo to " << servo_level << std::endl;
      }
      std::cout << ss.str();
      break;
    case KEYCODE_R:
      std::cout << " RIGHT" << std::endl;
      if (servo_level <= (180 - step))
      {
        servo_level += step;
        dirty = true;
        ss << "Setting servo to " << servo_level << std::endl;
      }
      else if (servo_level < 180)
      {
        servo_level = 180;
        dirty = true;
        ss << "Setting servo to " << servo_level << std::endl;
      }
      std::cout << ss.str();
      break;
    case KEYCODE_U:
      std::cout << " UP" << std::endl;
      if (servo_level < 180)
      {
        servo_level = 180;
        dirty = true;
        ss << "Setting servo to MAX" << std::endl;
      }
      std::cout << ss.str();
      break;
    case KEYCODE_D:
      std::cout << " DOWN" << std::endl;
      if (servo_level > 0)
      {
        servo_level = 0;
        dirty = true;
        ss << "Setting servo to MIN" << std::endl;
      }
      std::cout << ss.str();
      break;
    case KEYCODE_1:
      std::cout << " 1" << std::endl;
      step = 1;
      ss << "Very slow" << std::endl;
      std::cout << ss.str();
      break;
    case KEYCODE_2:
      std::cout << " 2" << std::endl;
      step = 3;
      ss << "Slow" << std::endl;
      std::cout << ss.str();
      break;
    case KEYCODE_3:
      std::cout << " 3" << std::endl;
      step = 7;
      ss << "Medium" << std::endl;
      std::cout << ss.str();
      break;
    case KEYCODE_4:
      std::cout << " 4" << std::endl;
      step = 12;
      ss << "Fast" << std::endl;
      std::cout << ss.str();
      break;
    case KEYCODE_5:
      std::cout << " 5" << std::endl;
      step = 20;
      ss << "Vary fast" << std::endl;
      std::cout << ss.str();
      break;
    }

    std_msgs::Int16 msg;
    msg.data = servo_level;

    if (dirty == true)
    {
      servoPosPublisher.publish(msg);
      dirty = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}