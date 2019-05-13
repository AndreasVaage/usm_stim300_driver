
#include "driver_stim300.h"
#include "ros/ros.h"


constexpr int defaultSampleRate{ 125 };
constexpr double averageAllanVarianceOfGyro{ 0.0001 * 2 * 0.00046};
constexpr double averageAllanVarianceOfAcc{ 100 * 2 * 0.0052};


class NullSerialDriver : public SerialDriver
{
public:
    NullSerialDriver() = default;

    void open(BAUDRATE baudrate) override{}

    void close()override{}

    bool readByte(uint8_t& byte)override
    {
      return false;
    }

    void writeByte(uint8_t byte)override{}

    ~NullSerialDriver()override = default;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stim300_driver_node");

  ros::NodeHandle node;


  double stanardDeivationOfGyro{ 0 };
  double stanardDeviationOfAcc{ 0 };
  double varianceOfGyro{ 0 };
  double varianceOfAcc{ 0 };
  int sampleRate{ 0 };


  node.param("stanard_deviation_of_gyro", stanardDeivationOfGyro, averageAllanVarianceOfGyro);
  node.param("stanard_deviation_of_acc", stanardDeviationOfAcc, averageAllanVarianceOfAcc);
  node.param("sample_rate", sampleRate, defaultSampleRate);
  varianceOfGyro = sampleRate * pow(stanardDeivationOfGyro, 2);
  varianceOfAcc = sampleRate * pow(stanardDeviationOfAcc, 2);

  sensor_msgs::Imu imu_msg_template{};
  imu_msg_template.orientation_covariance[0] = -1;
  imu_msg_template.angular_velocity_covariance[0] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[4] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[8] = varianceOfGyro;
  imu_msg_template.linear_acceleration_covariance[0] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[4] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[8] = varianceOfAcc;
  imu_msg_template.orientation.x = 0;
  imu_msg_template.orientation.y = 0;
  imu_msg_template.orientation.z = 0;
  imu_msg_template.header.frame_id = "imu_link";

  NullSerialDriver dummy_serial_driver;
  DriverStim300 driver_stim300(node,imu_msg_template,dummy_serial_driver);


  ROS_INFO("STIM300 IMU initialized successfully");

  ros::spin();

  return 0;
}