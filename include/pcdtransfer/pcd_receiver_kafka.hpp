#ifndef PCD_RECEIVER_KAFKA_HPP
#define PCD_RECEIVER_KAFKA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/serialization.hpp>


// pcc_common modules (assuming these are custom modules)
#include <struct.h>
#include <io.h>
#include <pcc_module.h>
#include <encoder.h>
#include <decoder.h>
#include <serialize.h>
#include <archive.h>

#include <kafka/KafkaConsumer.h>

// Class declaration
class PCD_Receiver : public rclcpp::Node {
public:
  PCD_Receiver();

private:
  void consumePcloudData();
  void extractPointCloud(const std::vector<char>& serialized_data);
  void convertPcloudToPointCloud2(const std::vector<std::vector<char>>& combined_strings);
  void stop_kafka_consumer();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<kafka::clients::consumer::KafkaConsumer> consumer_;
};

#endif // PCD_RECEIVER_HPP
