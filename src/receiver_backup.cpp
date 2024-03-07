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

#include <kafka/KafkaConsumer.h>

using namespace std::chrono_literals;

class PCD_Receiver : public rclcpp::Node
{
public:
  PCD_Receiver() : Node("pcd_receiver")
  {
    // Read configuration parameters from ROS parameters server
    std::string kafka_brokers =  "localhost:9092";
    std::string topic_name = "pcd_topic";
    
    // const kafka::Properties props({{"bootstrap.servers", kafka_brokers}});

    // Create a Kafka consumer
    // consumer_ = std::make_unique<kafka::clients::consumer::KafkaConsumer>(props);

    kafka::Topic topic = topic_name;

    // Subscribe to the topic
    // consumer_->subscribe({topic});

    

    // Start a background thread to run the consumer loop
    // std::thread consumer_thread(&PCD_Receiver::consumePcloudData, this);
    // Create a wall timer for polling Kafka topic
    std::cout << "In the constructor before wall timer"<< std::endl;
    auto timer = create_wall_timer(1000ms, std::bind(&PCD_Receiver::consumePcloudData, this));

    // consumer_thread.detach();

    // Create a publisher for PointCloud2 messages
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    // Register shutdown callback (optional)
    // rclcpp::on_shutdown([this]() {
    //   // Perform cleanup tasks here (e.g., disconnect Kafka consumer)
    //   stop_kafka_consumer();
    // });

  }

private:
  void consumePcloudData()
  {
    
      // Poll for messages from Kafka topic
      std::cout << "In consumePcloudData" << std::endl;
    //   auto records = consumer_->poll(std::chrono::milliseconds(1000));
    //   std::cout<< "calling consumePcloudData" << std::endl;
    //   // Process messages
    //   for (const auto& record : records) {
    //     // Extract serialized point cloud data from message value
    //     const std::string& serialized_data = record.value().toString();
    //     std::cout << "Got a new message..." << std::endl;
    //     std::cout << serialized_data << std::endl;
    //     // Process and publish the point cloud data
    //     // extractPointCloud(serialized_data);
      
    // }
    

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

    void stop_kafka_consumer() {
        if (consumer_) {
            consumer_->close();
        }
    }
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    std::unique_ptr<kafka::clients::consumer::KafkaConsumer> consumer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // signal(SIGINT, stopRunning); // Register SIGINT signal handler
  std::cout << "The node is updated and running" << std::endl;
  rclcpp::spin(std::make_shared<PCD_Receiver>());
  
  rclcpp::shutdown();
  return 0;
}
