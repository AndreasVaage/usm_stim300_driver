/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to an Arduino Ethernet Shield
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */

#define HWSERIAL Serial1

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>

// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <ros/time.h>
#include <usm_stim300_driver/UInt8MultiArrayStamped.h>
#include <usm_stim300_driver/UInt8UInt8.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Header.h>

const byte stim_300_TOV_pin = 2;
//const byte stim_300_trigger_pin = 3;
const byte camera_trigger_pin = 22;
const byte camera0_sync_pin = 20;
const byte camera0_not_leak_pin = 21;
const byte camera1_not_leak_pin = 19;
const byte camera1_sync_pin = 18;

volatile byte stim_300_sync_flag = LOW;
volatile byte stim_300_data_sent = LOW;
volatile byte camera0_sync_flag = LOW;
volatile byte camera1_sync_flag = LOW;
volatile byte camera0_leak = LOW;
volatile byte camera1_leak = LOW;
volatile unsigned long frame0_count = 0;
volatile unsigned long frame1_count = 0;

// Set the shield settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(169, 254, 0, 177); // think it have to be same subnet as server

// Set the rosserial socket server IP address
IPAddress server(169,254,0,64);
//IPAddress server(10,22,74,185);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher

usm_stim300_driver::UInt8MultiArrayStamped datagram_msg;
ros::Publisher chatter("datagram", &datagram_msg);

std_msgs::Header cam0_time_msg;
std_msgs::Header cam1_time_msg;
ros::Publisher cam0_time_publisher("cam0_time",&cam0_time_msg);
ros::Publisher cam1_time_publisher("cam1_time",&cam1_time_msg);

uint8_t frames_per_imu = 0;

bool run_cameras = false;

void commandCb(const usm_stim300_driver::UInt8UInt8& req)
{
  switch (req.command)
  {
    case 0:
      run_cameras = false;
      break;
    case 1:
      run_cameras = true;
      break;
    case 2:
      frames_per_imu = req.data;
      break;
    default:
      // resp.result = false;
      return;
  }
  // Serial.print("Run cameras = ");
  // Serial.println(run_cameras);
  // Serial.print("fpIMU = ");
  // Serial.println(frames_per_imu);
  // resp.result = true;
}
ros::Subscriber<usm_stim300_driver::UInt8UInt8> sub_VI_command("VI_command", &commandCb);

byte buffer[63];
byte first_received_datagram = HIGH;
size_t n_read_bytes = 0;
uint8_t imu_counter = 0;

bool waiting_for_last_camera = false;

void setup()
{ 
  pinMode(stim_300_TOV_pin, INPUT);
  pinMode(camera0_not_leak_pin, INPUT_PULLUP);
  pinMode(camera1_not_leak_pin, INPUT_PULLUP);
  pinMode(camera0_sync_pin, INPUT_PULLUP);
  pinMode(camera1_sync_pin, INPUT_PULLUP);
  pinMode(camera_trigger_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(stim_300_TOV_pin), registerIMUTime, CHANGE);
  attachInterrupt(digitalPinToInterrupt(camera0_sync_pin), registerCam0Time, FALLING);
  attachInterrupt(digitalPinToInterrupt(camera1_sync_pin), registerCam1Time, FALLING);
  attachInterrupt(digitalPinToInterrupt(camera0_not_leak_pin), registerCam0Leak, FALLING);
  attachInterrupt(digitalPinToInterrupt(camera1_not_leak_pin), registerCam1Leak, FALLING);


  // Use serial to monitor the process
  //Serial.begin(115200);

  HWSERIAL.begin(921600,SERIAL_8N1);
  HWSERIAL.setTimeout(2); // ms

  // Wait for the cameras to setup before connecting to ethernet
  // Yes, we are this desperate
  delay(5000);

  // Connect the Ethernet
  Ethernet.begin(mac, ip);

  // Let some time for the Ethernet Shield to be initialized
  delay(1000);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    //Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    //Serial.println("Ethernet cable is not connected.");
  }

  //Serial.println("");
  //Serial.println("Ethernet connected");
  //Serial.println("IP address: ");
  //Serial.println(Ethernet.localIP());
  //Serial.print("IMU interupt pin: ");
  //Serial.println(digitalPinToInterrupt(stim_300_TOV_pin));

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  //Serial.print("IP = ");
  //Serial.println(nh.getHardware()->getLocalIP());

  datagram_msg.data = (uint8_t*) malloc(63 * sizeof(uint8_t));
  datagram_msg.data_length = 63;
  
  cam0_time_msg.frame_id = "cam0_link";
  cam0_time_msg.seq = 0;
  cam1_time_msg.frame_id = "cam1_link";
  cam1_time_msg.seq = 0;

  // Start to be polite
  nh.advertise(chatter);
  nh.advertise(cam0_time_publisher);
  nh.advertise(cam1_time_publisher);
  nh.subscribe(sub_VI_command);
}

void loop()
{
  update(frames_per_imu, run_cameras);
  nh.spinOnce();
}

void update(int n_imu_per_cam_msg, bool trigger_cameras)
{
  if (stim_300_sync_flag == HIGH)
  {
    if(digitalRead(stim_300_TOV_pin) == LOW)
    {
      imu_counter++;
      datagram_msg.stamp = nh.now();
    }
    else
    {
      if (first_received_datagram == HIGH)
      {
        HWSERIAL.clear();
        first_received_datagram = LOW;
      }
      else if (n_read_bytes = HWSERIAL.readBytes(datagram_msg.data, 63) == 63)
      {
        if (nh.connected())
        {
          chatter.publish( &datagram_msg );
        }
        else
        {
          // Serial.println("Not Connected");
        }
      }
      else
      {
        // Serial.println("Not correct amount of data");
        // Serial.println(n_read_bytes);
      }
    }
    stim_300_sync_flag = LOW;
  }

  if (camera0_sync_flag == HIGH)
  {
    cam0_time_msg.stamp = nh.now();
    cam0_time_publisher.publish( &cam0_time_msg);
    cam0_time_msg.seq = frame0_count;
    camera0_sync_flag = LOW;
  }

  if (camera1_sync_flag == HIGH)
  {
    cam1_time_msg.stamp = nh.now();
    cam1_time_publisher.publish( &cam1_time_msg);
    cam1_time_msg.seq = frame1_count;
    camera1_sync_flag = LOW;
  }

  if (imu_counter >= n_imu_per_cam_msg)
  {
    if (trigger_cameras)
    {
      digitalWrite(camera_trigger_pin, HIGH);
    }
    imu_counter = 0;
  }
  if (imu_counter == 5)
  {
    digitalWrite(camera_trigger_pin, LOW);
  }
}

void registerIMUTime()
{
  stim_300_sync_flag = HIGH;
}
void registerCam0Time()
{
  frame0_count++;
  camera0_sync_flag = HIGH;
}
void registerCam1Time()
{
  frame1_count++;
  camera1_sync_flag = HIGH;
}
void registerCam0Leak()
{
  camera0_leak = HIGH;
}
void registerCam1Leak()
{
  camera1_leak = HIGH;
}
