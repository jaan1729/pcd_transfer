#include "archive.h"

// Function to split a string into a vector of strings based on a delimiter
std::vector<std::string> split(const std::string& str, const std::string& delimiter) {
  std::vector<std::string> tokens;
  std::istringstream stream(str);
  std::string token;
  while (std::getline(stream, token, delimiter[0])) {
    tokens.push_back(token);
  }
  return tokens;
}




std::vector<std::string> compress_pointcloud(std::vector<point_cloud> pcloud_data, int row, int col, float pitch_precision, float yaw_precision, float threshold, int tile_size, PccResult &pcc_res) {
  /*******************************************************************/
  // initialization

  double proj_time, fit_time;
  float psnr, total_pcloud_size;

  /*******************************************************************/
  // convert range map

  std::cout << "CURRENT pcloud size: " << pcloud_data.size() << std::endl;
  
  // Characterize Range Map
  // floating map;
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

  cv ::Mat* b_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);
  cv::Mat* occ_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);

  // encode the occupatjon map  
  encoder::encode_occupation_mat(*f_mat, *occ_mat, tile_size, mat_div_tile_sizes);

  fit_time = encoder::single_channel_encode(*f_mat, *b_mat, mat_div_tile_sizes, coefficients, 
                                            unfit_nums, tile_fit_lengths,
                                            threshold, tile_size);
  delete f_mat;

  pcc_res.fit_times->push_back(fit_time);

  // what we need to store:
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

  // std::string serialized_data;  
  // compress_data(combined_string, serialized_data);
  // combined_string.clear();
  // std::cout << "serialized_data size: " << serialized_data.size() * sizeof(serialized_data[0]) << std::endl;

  return combined_string;
}

std::vector<point_cloud> extract_pointcloud(const std::vector<std::string>& combined_strings, int row, int col, float pitch_precision, float yaw_precision, float threshold, int tile_size){
  // 2. Unarchive the compressed file
  // std::vector<std::string> combined_string;
  // extract_data(serialized_data, combined_string);
  // std::cout<<"Data Extracted"<<std::endl;
  // 3. Seperate the data according to different values
  std::string serialized_b_mat = combined_strings[0];
  std::string serialized_coefficients = combined_strings[1];
  std::string serialized_occ_mat = combined_strings[2];
  std::string serialized_unfit_nums = combined_strings[3];
  std::string serialized_tile_fit_lengths = combined_strings[4];

  // delete combined_string;

  // 4. Decode matrices from string data
  int mat_div_tile_sizes[] = {row/tile_size, col/tile_size};
  std::vector<cv::Vec4f> coefficients;
  std::vector<int> tile_fit_lengths;
  std::vector<float> unfit_nums;

  cv::Mat* b_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);
  cv::Mat* occ_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);
  std::cout<<"Created Matrices"<<std::endl;
  deserialize_b_mat(*b_mat, serialized_b_mat);
  std::cout<<"Ëxtracted bmat"<<std::endl;
  deserialize_coefficients(coefficients, serialized_coefficients);
  std::cout<<"Ëxtracted coefficiants"<<std::endl;
  deserialize_occ_mat(*occ_mat, serialized_occ_mat);
  std::cout<<"Ëxtracted occ mat"<<std::endl;
  deserialize_unfit_nums(unfit_nums, serialized_unfit_nums);
  std::cout<<"Ëxtracted unfit nums"<<std::endl;
  deserialize_tile_fit_lengths(tile_fit_lengths, serialized_tile_fit_lengths);
  std::cout<<"Ëxtracted lengths"<<std::endl;
    

  // reconstruct the range image
  cv::Mat* r_mat = new cv::Mat(row, col, CV_32FC1, 0.f);
  // decoding
  decoder::single_channel_decode(*r_mat, *b_mat, mat_div_tile_sizes, coefficients, 
                                 *occ_mat, tile_fit_lengths, unfit_nums, tile_size);

  delete b_mat;
  delete occ_mat;

  std::vector<point_cloud> restored_pcloud;
  restore_pcloud(*r_mat, pitch_precision, yaw_precision, restored_pcloud);
  
  delete r_mat;
  
  
  return restored_pcloud;
}

// int main() {
//   std::vector<std::string> data = {"apple", "banana", "cherry"};
//   std::string compressed_data;
//   compress_data(data, compressed_data);
//   std::vector<std::string> output;
//   extract_data(compressed_data, output);
  
//   // Output the oringinal strings
//   std::cout << "Original Strings Before Compression: " << std::endl;
//   for (const std::string& s : data) {
//     std::cout << s << std::endl;
//   }

//   // Output the decompressed strings
//   std::cout << "Output Strings Agter Compression: " << std::endl;
//   for (const std::string& s : output) {
//     std::cout << s << std::endl;
//   }
//   std::string s = "äpple";
//   std::cout << "Compressed String" << compressed_data << std::endl;
//   std::cout << "Input Size: " << data.size() * data[0].size() << std::endl;
//   std::cout << "Compressed Size: " << compressed_data.size() << std::endl;
//   std::cout << "APPLE Size: " << s.size() << std::endl;
// }

