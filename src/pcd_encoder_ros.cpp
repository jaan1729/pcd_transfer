#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// pcc_common modules
#include <struct.h>
#include <io.h>
#include <pcc_module.h>
#include <encoder.h>
#include <decoder.h>
#include <serialize.h>
#include <archieve.h>

class PCD_Encoder : public rclcpp::Node
{
public:
  PCD_Encoder() : Node("pcd_encoder")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", 10, std::bind(&PCD_Encoder::pointCloudCallback, this, std::placeholders::_1));

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

    double proj_time, fit_time;
    float psnr, total_pcloud_size;

    /*******************************************************************/
    // convert range map

    std::cout << "CURRENT pcloud size: " << pcloud_data.size() << std::endl;
    
    // Characterize Range Map
    // floating map;
    std::cout << "row: " << row << " col: " << col << std::endl;
    std::cout << "pitch_precision: " << pitch_precision << " yaw_precision: " << yaw_precision << std::endl;
    cv::Mat* f_mat = new cv::Mat(row, col, CV_32FC4, cv::Scalar(0.f,0.f,0.f,0.f));
    
    proj_time = map_projection(*f_mat, pcloud_data, pitch_precision, yaw_precision, 'e');
    
    pcc_res.proj_times->push_back(proj_time);
    
    // compute compression rate: bit-per-point (bpp)
    pcc_res.compression_rate->push_back(8.0f*f_mat->cols*f_mat->rows/pcloud_data.size());
    
    // loss error compute;
    //psnr = compute_loss_rate<cv::Vec4f>(*f_mat, pcloud_data, pitch_precision, yaw_precision);
    
    // update the info;
    pcc_res.loss_rate->push_back(psnr);
    
    /*******************************************************************/
    // fitting range map
    int mat_div_tile_sizes[] = {row/tile_size, col/tile_size};
    std::vector<cv::Vec4f> coefficients;
    std::vector<int> tile_fit_lengths;
    std::vector<float> unfit_nums;

    cv::Mat* b_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);
    cv::Mat* occ_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);
  
    // encode the occupatjon map  
    encoder::encode_occupation_mat(*f_mat, *occ_mat, tile_size, mat_div_tile_sizes);

    fit_time = encoder::single_channel_encode(*f_mat, *b_mat, mat_div_tile_sizes, coefficients, 
                                              unfit_nums, tile_fit_lengths,
                                              threshold, tile_size);
    delete f_mat;

    pcc_res.fit_times->push_back(fit_time);

    size_t totalMemory = 2 * b_mat->rows * b_mat->cols * b_mat->elemSize() + sizeof(cv::Vec4f) * coefficients.size() + sizeof(float) * unfit_nums.size() + sizeof(int) * tile_fit_lengths.size();
    RCLCPP_INFO(this->get_logger(), "Total memory for all variables: %zu bytes", totalMemory);
    RCLCPP_INFO(this->get_logger(), "Memory for pcloud_data: %zu bytes", sizeof(pcloud_data[0]) * pcloud_data.size());
    
    // 1. b_mat: binary map for tile type
    std::string serialized_b_mat;
    serialize_b_mat(*b_mat, serialized_b_mat);
    delete b_mat;

    std::string serialized_coefficients;
    serialize_coefficients(coefficients, serialized_coefficients);
    coefficients.clear();

    std::string serialized_occ_mat;
    serialize_occ_mat(*occ_mat, serialized_occ_mat);
    delete occ_mat;

    std::string serialized_unfit_nums;
    serialize_unfit_nums(unfit_nums, serialized_unfit_nums);
    unfit_nums.clear();

    std::string serialized_tile_fit_lengths;
    serialize_tile_fit_lengths(tile_fit_lengths, serialized_tile_fit_lengths);
    tile_fit_lengths.clear();
    
    // 6. make a tar.gz file
    std::vector<std::string> combined_string;
    combined_string.push_back(serialized_b_mat);
    combined_string.push_back(serialized_coefficients);
    combined_string.push_back(serialized_occ_mat);
    combined_string.push_back(serialized_unfit_nums);
    combined_string.push_back(serialized_tile_fit_lengths);
    
    std::string serialized_data;  
    archieve_data(combined_string, serialized_data);
    combined_string.clear();

    publisher_->publish(serialized_data);
    
    }

    std::vector<point_cloud> pcloud_data;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  };

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
