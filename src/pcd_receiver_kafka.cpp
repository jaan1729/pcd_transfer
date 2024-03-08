#include <chrono>
#include <thread>

#include "pcd_transfer/pcd_receiver_kafka.hpp"  // Include your custom header file


using namespace std::chrono_literals;
PCD_Receiver::PCD_Receiver() : Node("pcd_receiver") {
  // Read configuration parameters from ROS parameters server
  std::string kafka_brokers = "localhost:9092";
  std::string topic_name = "pcd_topic";

  // Create Kafka properties and consumer
  
  kafka::Properties props({{"bootstrap.servers", {kafka_brokers}},
                            {"max.pool.records", {"1"}}
                                              // {"max.partition.fetch.bytes", {"20485760"}}
                                              // {"fetch.min.bytes", {"1048576"}}, // Adjust minimum fetch size (optional)
                                              // {"fetch.size", {"2097152"}} // Adjust fetch size (optional)});
});
  consumer_ = std::make_unique<kafka::clients::consumer::KafkaConsumer>(props);

  kafka::Topic topic = topic_name;

  // Subscribe to the topic
  consumer_->subscribe({topic});

  // Start a background thread to run the consumer loop
  std::thread consumer_thread(&PCD_Receiver::consumePcloudData, this);
  consumer_thread.detach();

  // Create a wall timer for polling Kafka topic (optional)
  std::cout << "In the constructor before wall timer" << std::endl;
  // auto timer = create_wall_timer(1000ms, std::bind(&PCD_Receiver::consumePcloudData, this));

  // Create a publisher for PointCloud2 messages
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_op", 10);

  // Register shutdown callback (optional)
  rclcpp::on_shutdown([this]() {
    // Perform cleanup tasks here (e.g., disconnect Kafka consumer)
    stop_kafka_consumer();
  });
}

void PCD_Receiver::consumePcloudData() {
  while (rclcpp::ok()) {
    // Poll for messages from Kafka topic
    
    auto records = consumer_->poll(std::chrono::milliseconds(100));

    // Process messages
    for (const auto& record : records) {
      // Extract serialized point cloud data from message value
      const auto message = record.value();
      auto data = message.data();
      auto size = message.size();
      const auto* beg = reinterpret_cast<const char*>(data);
      std::vector<char> serialized_data(beg, beg + size);
      

      std::cout << "Got a new message..." << std::endl;
      std::cout << "Size of serialized_data: " << serialized_data.size() << std::endl;
      
      std::vector<std::vector<char>> combined_strings = deserialize(serialized_data);
      
      for (const auto& str : combined_strings) {
        std::cout << "Size of combined_strings[" << &str - &combined_strings[0] << "]: " << str.size() << std::endl;
      }
      convertPcloudToPointCloud2(combined_strings);
      // Process and publish the point cloud data
      // convertPcloudToPointCloud2(combined_strings);
    }
  }
}

// Implementations for extractPointCloud, convertPcloudToPointCloud2, and stop_kafka_consumer (assuming they are defined in pcc_common modules)
void PCD_Receiver::extractPointCloud(const std::vector<char>& serialized_data) {
  // ... (implementation using pcc_common modules)
    
    // save the serialized data to a file
    
    // std::cout << "serialized_data: " << serialized_data << std::endl;
    std::vector<std::vector<char>> combined_strings = deserialize(serialized_data);
    
    // for (const auto& str : combined_strings) {
    //   std::cout << str << std::endl;
    // }
    
    convertPcloudToPointCloud2(combined_strings);
}

void PCD_Receiver::convertPcloudToPointCloud2(const std::vector<std::vector<char>>& combined_strings) {
  // ... (implementation using pcc_common modules)
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

void PCD_Receiver::stop_kafka_consumer() {
  if (consumer_) {
    consumer_->close();
  }
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create an instance of your node
    auto node = std::make_shared<PCD_Receiver>();

    // Spin the node (handle callbacks and timers)
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
