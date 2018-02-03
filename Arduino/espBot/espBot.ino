
#include <ESP8266WiFi.h>
#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

//ROS
#define SERVO_TOPIC "/ServoPositionTopic"
#define SENSOR1_TOPIC "/ESPBot_Feed_1"
#define SENSOR2_TOPIC "/ESPBot_Feed_2"

//Servo
Servo servo;

void servoCallback(const std_msgs::Int16 &msg)
{

  int position = msg.data;

  Serial.print("Received message: ");
  Serial.println(position);

  servo.write(position);
}

//WIFI
const char *ssid = "SSID";
const char *password = "PASSWORD";

IPAddress server(192, 168, 1, 20);
IPAddress ip_address;
WiFiClient client;

int status = WL_IDLE_STATUS;

class WiFiHardware
{

public:
  WiFiHardware(){};

  void init()
  {
    // do your initialization here. this probably includes TCP server/client setup
    Serial.println("Init method");
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read()
  {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read(); //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t *data, int length)
  {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for (int i = 0; i < length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time()
  {
    return millis(); // easy; did this one for you
  }
};

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 1000)
    delay(500);

  if (i == 1001)
  {
    Serial.print("Could not connect to");
    Serial.println(ssid);
    //  while(1) delay(500);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Ready! Use ");
    Serial.print(WiFi.localIP());
    Serial.println(" to access client");
  }
}

ros::NodeHandle_<WiFiHardware> nh;

ros::Subscriber<std_msgs::Int16> servoSubscriber(SERVO_TOPIC, &servoCallback);

std_msgs::Int16 sensor1_msg;
std_msgs::Bool sensor2_msg;

ros::Publisher sensor1(SENSOR1_TOPIC, &sensor1_msg);
ros::Publisher sensor2(SENSOR2_TOPIC, &sensor2_msg);

void SetupROS()
{
  Serial.println("initializing the node");
  nh.initNode();

  delay(1000);

  Serial.println("adveritsing and subscribing");
  nh.advertise(sensor1);
  nh.advertise(sensor2);

  nh.subscribe(servoSubscriber);
}

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  setupWiFi();
  delay(2000);

  servo.attach(2);

  SetupROS();
}

void check_connection()
{

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connection to Wifi broken, setuping again");
    setupWiFi();
  }
  if (!client.connected())
  {
    Serial.println("client not connected, reconnecting");

    client.connect(server, 11411);
    int tries = 1;
    while (!client.connected())
    {
      delay(100);
      tries++;

      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("Connection to Wifi broken, setuping again");
        setupWiFi();
        delay(1000);
      }

      if (tries % 10)
      {
        Serial.println("still trying to reconnect");
      }

      if (tries % 500)
      {
        Serial.println("trying to re initialize the node");
        SetupROS();
        delay(2000);
      }

      if (tries % 1000)
      {
        Serial.println("RESETTING");
        ESP.restart();
      }
    }
  }
}

void loop()
{
  int sensor1Value = 1024 - analogRead(A0); // when dark 0
  bool sensor2Value = !digitalRead(5);      // when dark False

  sensor1_msg.data = sensor1Value;
  sensor2_msg.data = sensor2Value;

  check_connection();
  if (client.connected())
  {
    sensor1.publish(&sensor1_msg);
    sensor2.publish(&sensor2_msg);
    nh.spinOnce();
  }

  delay(10);
}
