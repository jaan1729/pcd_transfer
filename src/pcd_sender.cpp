#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

// pcc_common modules
#include <struct.h>
#include <io.h>
#include <pcc_module.h>
#include <encoder.h>
#include <decoder.h>
#include <serialize.h>
#include <archive.h>

class PCD_Sender : public rclcpp::Node
{
public:
  PCD_Sender() : Node("pcd_sender")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", 10, std::bind(&PCD_Sender::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("serialized_pcd", 10);
    
    
    
  }
  
  

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud message");
    convertPointCloud2ToPcloud(*msg);
    compressPointCloud();
    this->pcloud_data.clear();
  }

  void convertPointCloud2ToPcloud(const sensor_msgs::msg::PointCloud2& msg) {
    
    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_i(msg, "intensity");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
        this->pcloud_data.push_back(point_cloud(std::move((*iter_x)),
                                                std::move((*iter_y)),
                                                std::move((*iter_z)),
                                                std::move(*iter_i)));
    }
    
    // RCLCPP_INFO(this->get_logger(), "Shape of pcloud_data: %zu", this->pcloud_data.size());
} 

  void compressPointCloud() {
    pcloud_data = this->pcloud_data;
    PccResult pcc_res;
    std::cout << "Size of pcloud_data: " << pcloud_data.size() << std::endl;
    //TODO: These values need to be configurable in future.

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

    std::vector<std::string> combined_strings = compress_pointcloud(pcloud_data, row, col, pitch_precision, yaw_precision, threshold, tile_size, pcc_res);

    
    // std::vector<std::string> combined_strings;
    // combined_strings.push_back("This is a long string with new line characters.\n This is after first line in first element.\n");
    // combined_strings.push_back("It spans multiple lines.\n Second element has this.\n");
    // combined_strings.push_back("Each line is a separate element in the vector.\n");
    // combined_strings.push_back("You can access each element using indexing.\n");
    // combined_strings.push_back("For example, combined_strings[0] will give you the first line.\n");

    
    std::string serialized_data = serialize(combined_strings);
    this->msg.data = serialized_data;
    // this->msg.data = "Hello World! How are you? \n Where are you coming from? Hey, I'm so and so and I'm coming from some place out of this galaxy. Do you mind taking me to the space port?"; // "serialized_data";
    std::cout << "serialized_data msg: " << this->msg.data.size() << std::endl;
    combined_strings.clear();

    publisher_->publish(this->msg);
    
    }

    std::vector<point_cloud> pcloud_data;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std_msgs::msg::String msg;
  };

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCD_Sender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
