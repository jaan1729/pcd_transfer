#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/serialization.hpp>

// pcc_common modules
#include <struct.h>
#include <io.h>
#include <pcc_module.h>
#include <encoder.h>
#include <decoder.h>
#include <serialize.h>
#include <archive.h>

class PCD_Recevier : public rclcpp::Node
{
public:
  PCD_Recevier() : Node("pcd_receiver")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "serialized_pcd", 10, std::bind(&PCD_Recevier::serializedPcdCallback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  }

private:
    void serializedPcdCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received serialized point cloud message");
        
        rclcpp::SerializedMessage serialized_msg;
        stringSerializer.serialize_message(&msg, &serialized_msg);
        const auto buffer_begin = serialized_msg.get_rcl_serialized_message().buffer;
        const auto buffer_end = buffer_begin + serialized_msg.size();
        std::string serialized_data(buffer_begin, buffer_end);
        
        // std::string serialized_data = msg->data;
        extractPointCloud(serialized_data);
    }
    
    void extractPointCloud(const std::string& serialized_data) {

        std::cout << "Size of serialized_data: " << serialized_data.size() << std::endl;
        // std::cout << "serialized_data: " << serialized_data << std::endl;
        std::vector<std::string> combined_strings = deserialize(serialized_data);
        
        convertPcloudToPointCloud2(combined_strings);

    }
    void convertPcloudToPointCloud2(const std::vector<std::string> & combined_strings) {
        sensor_msgs::msg::PointCloud2 msg;

        float pitch_precision = 0.18;
        float yaw_precision = 0.45;
        float threshold = 0.5;
        std::string format = "binary";
        int tile_size = 4;
        /*******************************************************************/
        // initialization

        int row = (VERTICAL_DEGREE/yaw_precision);
        row = ((row + tile_size-1)/tile_size)*tile_size;
        int col = HORIZONTAL_DEGREE/pitch_precision + tile_size;
        col = ((col + tile_size-1)/tile_size)*tile_size;
        
        std::vector<point_cloud> pcloud_data = extract_pointcloud(combined_strings, row, col, pitch_precision, yaw_precision, threshold, tile_size);
        std::cout << "Size of pcloud_data: " << pcloud_data.size() << std::endl;
        msg.header.frame_id = "map"; // Set the frame ID
        msg.width = pcloud_data.size(); // Set the width
        msg.height = 1; // Set the height (assuming a single row)

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(4,
                                      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(pcloud_data.size());
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
        
        for (const auto& pcloud : pcloud_data) {
            *iter_x = pcloud.x;
            *iter_y = pcloud.y;
            *iter_z = pcloud.z;
            *iter_i = pcloud.r;
            ++iter_x; ++iter_y; ++iter_z; ++iter_i;
        }
        
        

        publisher_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    rclcpp::Serialization<std_msgs::msg::String> stringSerializer;
    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCD_Recevier>());
  rclcpp::shutdown();
  return 0;
}
