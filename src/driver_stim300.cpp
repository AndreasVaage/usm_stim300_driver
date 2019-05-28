#include <utility>

#include "driver_stim300.h"

DriverStim300::DriverStim300(ros::NodeHandle& nh, sensor_msgs::Imu& imu_msg, SerialDriver& serial_driver):
    DriverStim300(serial_driver)
{
  imu_msg_ = imu_msg;
  datagram_sub_ = nh.subscribe("datagram", 10, &DriverStim300::datagramCallBack, this);
  imu_publisher_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
  depth_publisher_ = nh.advertise<sensor_msgs::FluidPressure>("depth/data_raw", 10);
  pressure_msg_.header.frame_id = "pressure_link";
}

DriverStim300::DriverStim300(SerialDriver& serial_driver, stim_300::DatagramIdentifier datagram_id,
                             stim_300::GyroOutputUnit gyro_output_unit, stim_300::AccOutputUnit acc_output_unit,
                             stim_300::InclOutputUnit incl_output_unit, SerialDriver::BAUDRATE baudrate,
                             uint16_t serial_read_timeout_ms)
  : serial_driver_(serial_driver)
  , serial_read_timeout_ms_(serial_read_timeout_ms)
  , datagram_id_(datagram_id)
  , mode_(Mode::Normal)
  , n_read_bytes_(0)
  , in_sync_(false)
  , checksum_is_ok_(false)
  , no_internal_error_(true)
  , crc_dummy_bytes_(stim_300::numberOfPaddingBytes(datagram_id))
  , sensor_data_()
  , datagram_parser_(datagram_id, gyro_output_unit, acc_output_unit, incl_output_unit)
  , datagram_size_(stim_300::calculateDatagramSize(datagram_id))
{
  serial_driver_.open(baudrate);
}


DriverStim300::~DriverStim300()
{
  serial_driver_.close();
}

double DriverStim300::getAccX() const
{
  return sensor_data_.acc[0];
}
double DriverStim300::getAccY() const
{
  return sensor_data_.acc[1];
}
double DriverStim300::getAccZ() const
{
  return sensor_data_.acc[2];
}
double DriverStim300::getGyroX() const
{
  return sensor_data_.gyro[0];
}
double DriverStim300::getGyroY() const
{
  return sensor_data_.gyro[1];
}
double DriverStim300::getGyroZ() const
{
  return sensor_data_.gyro[2];
}
double DriverStim300::getAux() const
{
  return sensor_data_.aux;
}
uint16_t DriverStim300::getLatency_us() const
{
  return sensor_data_.latency_us;
}
bool DriverStim300::isChecksumGood() const
{
  return checksum_is_ok_;
}
bool DriverStim300::isSensorStatusGood() const
{
  return no_internal_error_;
}
uint8_t DriverStim300::getInternalMeasurmentCounter() const
{
  return sensor_data_.counter;
}

double DriverStim300::getAverageTemp() const
{
  double sum{ 0 };
  uint8_t count{ 0 };

  return count != 0 ? sum / count : std::numeric_limits<double>::quiet_NaN();
}

void DriverStim300::datagramCallBack(const usm_stim300_driver::UInt8MultiArrayStamped& message)
{
  auto begin = message.data.cbegin();
  auto it = std::next(begin);

  if (*begin != stim_300::datagramIdentifierToRaw(datagram_id_))
  {
    ROS_WARN("STIM300: message does not start with correct datagram");
    return;
  }

  // End codesection

  no_internal_error_ = datagram_parser_.parseDatagram(it, sensor_data_);

  checksum_is_ok_ = verifyChecksum(begin, it, sensor_data_.crc);

  publishImuData(message.stamp);
}

void DriverStim300::publishImuData(ros::Time time)
{
    if (!isChecksumGood())
    {
      ROS_WARN("stim300 CRC error ");
      return;
    }

    if (!isSensorStatusGood())
    {
      ROS_WARN("STIM300: Internal hardware error");
      return;
    }
    time = time - ros::Duration(0, getLatency_us() * 1000);  // convert from microseconds to nano seconds

    if (ros::Time::now() - time > ros::Duration(1, 0))
    {
      ROS_WARN("STIM300: Not realtime. More than 1 sec latency");
      return;
    }
    imu_msg_.header.stamp = time;
    imu_msg_.linear_acceleration.x = getAccX();
    imu_msg_.linear_acceleration.y = getAccY();
    imu_msg_.linear_acceleration.z = getAccZ();
    imu_msg_.angular_velocity.x = getGyroX();
    imu_msg_.angular_velocity.y = getGyroY();
    imu_msg_.angular_velocity.z = getGyroZ();
    imu_publisher_.publish(imu_msg_);

    pressure_msg_.header.stamp = time;
    pressure_msg_.fluid_pressure = getAux() * 8.857 - 8.857 * 1.613;
    depth_publisher_.publish(pressure_msg_);
}

bool DriverStim300::processPacket()
{
  if (this->mode_ == Mode::Normal)
  {
    // Read from buffer until we find a datagram identifyer.
    // Read the amount of bytes one datagram should contain.
    // Parse that datagram.
    // TODO: Make this code section cleaner

    // begin codesection

    uint8_t byte;
    while (serial_driver_.readByte(byte))
    {
      if (byte == stim_300::datagramIdentifierToRaw(datagram_id_))
      {
        if (n_read_bytes_ == datagram_size_)
        {
          n_read_bytes_ = 0;
          in_sync_ = true;
          // std::cout<<"In sync"<<std::endl;
        }
        else if (not in_sync_)
        {
          in_sync_ = true;
          // std::cout<<"Initialised sync"<<std::endl;
          n_read_bytes_ = 0;
        }
        else
        {
          // std::cout<<"Random byte is equal datagram id"<<std::endl;
        }
      }

      buffer_.push_back(byte);
      n_read_bytes_++;
      if (buffer_.size() > datagram_size_)
        buffer_.erase(buffer_.begin());

      if (n_read_bytes_ == datagram_size_)
        break;
    }
    if (buffer_.empty())
    {
      // std::cout << "Empty buffer" << std::endl;
      return false;
    }

    // std::cout<<"N read bytes: "<<n_read_bytes_<<std::endl;
    auto begin = buffer_.cbegin();
    auto it = std::next(begin);

    if (*begin != stim_300::datagramIdentifierToRaw(datagram_id_))
    {
      return false;
    }

    // End codesection

    no_internal_error_ = datagram_parser_.parseDatagram(it, sensor_data_);

    checksum_is_ok_ = verifyChecksum(begin, it, sensor_data_.crc);

    return true;
  }
  else if (this->mode_ == Mode::Service)
  {
    // std::string s(buffer_.begin(), buffer_.end());
    // std::cout << s << "\n";
    return false;
  }

  return false;
}

bool DriverStim300::verifyChecksum(std::vector<uint8_t>::const_iterator begin, std::vector<uint8_t>::const_iterator end,
                                   uint32_t& expected_CRC)
{
  assert(datagram_size_ == (end - begin));
  boost::crc_basic<32> crc_32_calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false, false);
  uint8_t buffer_CRC[datagram_size_ - sizeof(uint32_t) + crc_dummy_bytes_];
  std::copy(begin, end - sizeof(uint32_t) + crc_dummy_bytes_, buffer_CRC);

  /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
  for (size_t i = 0; i < crc_dummy_bytes_; ++i)
    buffer_CRC[sizeof(buffer_CRC) - (1 + i)] = 0x00;

  crc_32_calculator.process_bytes(buffer_CRC, sizeof(buffer_CRC));

  return crc_32_calculator.checksum() == expected_CRC;
}
