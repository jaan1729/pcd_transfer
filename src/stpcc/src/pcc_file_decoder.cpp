/*
 * This is an example of 3d lidar compression.
 * */

#include <stdlib.h>
#include "pcc_module.h"
#include "encoder.h"
#include "decoder.h"
#include "io.h"
#include "serialize.h"
#include "archive.h"

// int main(int argc, char** argv) { 
  
//   std::string file_path, out_file, file_name;
//   std::string input_format("binary");
//   float pitch_precision, yaw_precision, threshold;
//   int tile_size;

//   namespace po = boost::program_options;

//   po::options_description opts("PCC options");
//   opts.add_options()
//     ("help,h", "Print help messages")
//     ("path", po::value<std::string>(&file_path)->required(), "compressed path")
//     ("file", po::value<std::string>(&file_name)->required(), "compressed data filename")
//     ("out", po::value<std::string>(&out_file)->required(), "output pointcloud filename")
//     ("pitch,p", po::value<float>(&pitch_precision)->required(), "pitch precision")
//     ("yaw,y", po::value<float>(&yaw_precision)->required(), "yaw precision")
//     ("format,f",  po::value<std::string>(&input_format),
//      "trace_file input format: binary(default) or ascii")
//     ("tile,l", po::value<int>(&tile_size)->required(), "fitting tile size");

//   po::variables_map vm;

//   try 
//   {
//     po::store(po::parse_command_line(argc, argv, opts), vm);
    
//     if (vm.count("help")) 
//     {
//       std::cout << "Point Cloud Compression" << std::endl 
//         << opts << std::endl;
//       return 0;
//     }

//     po::notify(vm);
//   } catch(po::error& e) { 
//     std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
//     std::cerr << opts << std::endl; 
//     return -1;
//   }
  
//   /*******************************************************************/
//   // initialization

//   int row = (VERTICAL_DEGREE/yaw_precision);
//   row = ((row + tile_size-1)/tile_size)*tile_size;
//   int col = HORIZONTAL_DEGREE/pitch_precision + tile_size;
//   col = ((col + tile_size-1)/tile_size)*tile_size;

//   int mat_div_tile_sizes[] = {row/tile_size, col/tile_size};

//   file_name = file_path + "/" + file_name;
//   std::string out_file_name = file_path + "/" + out_file; // TODO: Find a better way to do this
//   std::cout<<file_name<<std::endl;
//   // 1. Read the binary compressed file 
//   // TODO: Move this as a function to io
  
//   std::string serialized_data;
//   std::ifstream serialized_file(file_name, std::ios::binary);
//   if (serialized_file) {
//     serialized_file.seekg(0, std::ios::end);
//     std::size_t file_size = serialized_file.tellg();
//     serialized_file.seekg(0, std::ios::beg);
    
//     // Resize the string to accommodate the expected decompressed size
//     serialized_data.resize(file_size);

//     // Read compressed data into the string
//     serialized_file.read(serialized_data.data(), file_size);
//     std::cout<<"File Read"<<std::endl;
//   } else {
//     std::cerr << "Failed to open serialized_data.bin" << std::endl;
//     // Handle the error accordingly
//   }
  
  
  
//   std::vector<point_cloud> restored_pcloud = decompress_pointcloud(serialized_data, row, col, pitch_precision, yaw_precision, threshold, tile_size);
//   std::cout<<"Extracted Pointcloud and saving to"<< out_file_name << std::endl;
//   pcloud2bin(out_file_name, restored_pcloud);
//   // output_cloud(pcloud_data, "org.ply");
//   // output_cloud(restored_pcloud, "restored.ply");
//   std::cout << "**********************************************************" << std::endl;
  
//   return 0;
  
// }

