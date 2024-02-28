/*
 * This is an example of 3d lidar compression.
 * */

#include "pcc_module.h"
#include "encoder.h"
#include "decoder.h"
#include "io.h"
#include "serialize.h"
#include "archieve.h"

#include <fstream>
#include <sstream>

int main(int argc, char** argv) { 
  
  std::string file_path, out_file;
  std::string input_format("binary");
  float pitch_precision, yaw_precision, threshold;
  int tile_size;

  namespace po = boost::program_options;

  po::options_description opts("PCC options");
  opts.add_options()
    ("help,h", "Print help messages")
    ("path", po::value<std::string>(&file_path)->required(), "compressed path")
    ("file", po::value<std::string>(&file_name)->required(), "compressed data filename")
    ("out", po::value<std::string>(&out_file)->required(), "output pointcloud filename")
    ("pitch,p", po::value<float>(&pitch_precision)->required(), "pitch precision")
    ("yaw,y", po::value<float>(&yaw_precision)->required(), "yaw precision")
    ("format,f",  po::value<std::string>(&input_format),
     "trace_file input format: binary(default) or ascii")
    ("tile,l", po::value<int>(&tile_size)->required(), "fitting tile size");

  po::variables_map vm;

  try 
  {
    po::store(po::parse_command_line(argc, argv, opts), vm);
    
    if (vm.count("help")) 
    {
      std::cout << "Point Cloud Compression" << std::endl 
        << opts << std::endl;
      return 0;
    }

    po::notify(vm);
  } catch(po::error& e) { 
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
    std::cerr << opts << std::endl; 
    return -1;
  }
  
  /*******************************************************************/
  // initialization

  int row = (VERTICAL_DEGREE/yaw_precision);
  row = ((row + tile_size-1)/tile_size)*tile_size;
  int col = HORIZONTAL_DEGREE/pitch_precision + tile_size;
  col = ((col + tile_size-1)/tile_size)*tile_size;

  int mat_div_tile_sizes[] = {row/tile_size, col/tile_size};

  // unarchive the compressed point cloud
  std::string cmd = "tar -xzvf " + file_path;
  if (system(cmd.c_str()) == -1) {
    std::cout << "[ERROR]: 'tar' command executed failed." << std::endl;
     exit(-1);
  }

  file_name = file_path + "/" + file_name;
  std::string out_file_name = file_path + "/" + out_file; // TODO: Find a better way to do this

  // 1. Read the binary compressed file 
  // TODO: Move this as a function to io
  
  std::string serialized_data;
  std::ifstream serialized_file(file_name, std::ios::binary);
  if (serialized_file) {
    std::ostringstream oss;
    oss << serialized_file.rdbuf();
    serialized_data = oss.str();
    serialized_file.close();
  } else {
    std::cerr << "Failed to open serialized_data.bin" << std::endl;
    // Handle the error accordingly
  }
  
  // 2. Unarchive the compressed file
  std::vector<std::string> combined_string;
  unarchive_data(serialized_data, *combined_string);
  // 3. Seperate the data according to different values
  std::string serialized_b_mat = combined_string[0];
  std::string serialized_coefficients = combined_string[1];
  std::string serialized_occ_mat = combined_string[2];
  std::string serialized_unfit_nums = combined_string[3];
  std::string serialized_tile_fit_lengths = combined_string[4];

  delete combined_string;

  // 4. Decode matrices from string data
  int mat_div_tile_sizes[] = {row/tile_size, col/tile_size};
  std::vector<cv::Vec4f> coefficients;
  std::vector<int> tile_fit_lengths;
  std::vector<float> unfit_nums;

  cv::Mat* b_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);
  cv::Mat* occ_mat = new cv::Mat(row/tile_size, col/tile_size, CV_32SC1, 0.f);

  deserialize_b_mat(*b_mat, serialized_b_mat);
  deserialize_coefficients(coefficients, serialized_coefficients);
  deserialize_occ_mat(*occ_mat, serialized_occ_mat);
  deserialize_unfit_nums(*unfit_nums, serialized_unfit_nums);
  deserialize_tile_fit_lengths(tile_fit_lengths, serialized_tile_fit_lengths);
    

  // reconstruct the range image
  cv::Mat* r_mat = new cv::Mat(row, col, CV_32FC1, 0.f);
  // decoding
  decoder::single_channel_decode(*r_mat, *b_mat, mat_div_tile_sizes, coefficients, 
                                 *occ_mat, tile_fit_lengths, unfit_nums, tile_size);

  std::vector<point_cloud> restored_pcloud;
  restore_pcloud(*r_mat, pitch_precision, yaw_precision, restored_pcloud);
  
  pcloud2bin(out_file_name, restored_pcloud);
  // output_cloud(pcloud_data, "org.ply");
  // output_cloud(restored_pcloud, "restored.ply");
  std::cout << "**********************************************************" << std::endl;
  
  delete r_mat;
  delete b_mat;
  delete occ_mat;


  return 0;
  
}

