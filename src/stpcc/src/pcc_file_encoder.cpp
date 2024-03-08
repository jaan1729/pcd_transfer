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


// A function that takes pointcloud data and returns a compressed binary string



// int main(int argc, char** argv) { 
  
//   std::string file_path, file_name, out_file;
//   std::string input_format("binary");
//   float pitch_precision, yaw_precision, threshold;
//   int tile_size;

//   namespace po = boost::program_options;

//   po::options_description opts("PCC options");
//   opts.add_options()
//     ("help,h", "Print help messages")
//     ("path", po::value<std::string>(&file_path)->required(), "raw point cloud data path")
//     ("file", po::value<std::string>(&file_name)->required(), "raw point cloud data filename")
//     ("out", po::value<std::string>(&out_file)->required(), "compressed data filename")
//     ("pitch,p", po::value<float>(&pitch_precision)->required(), "pitch precision")
//     ("yaw,y", po::value<float>(&yaw_precision)->required(), "yaw precision")
//     ("threshold,t", po::value<float>(&threshold)->required(), "threshold value for fitting")
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
  
//   // create a vector to store frames;
//   std::vector<point_cloud> pcloud_data;
//   file_name = file_path + "/" + file_name;
  
//   std::string out_file_name = file_path + "/" + out_file; // TODO: Find a better way to do this
//   load_pcloud(file_name, pcloud_data);

//   std::vector<std::string> filenames;
//   filenames.push_back(file_name);
//   export_filenames(filenames, "filenames.bin");

//   PccResult pcc_res;

//   /*******************************************************************/
//   // initialization

//   int row = (VERTICAL_DEGREE/yaw_precision);
//   row = ((row + tile_size-1)/tile_size)*tile_size;
//   int col = HORIZONTAL_DEGREE/pitch_precision + tile_size;
//   col = ((col + tile_size-1)/tile_size)*tile_size;

  
  
//   std::string serialized_data;  
//   serialized_data = compress_pointcloud(pcloud_data, row, col, pitch_precision, yaw_precision, threshold, tile_size, pcc_res);

//   std::cout << "serialized_data size: " << serialized_data.size() * sizeof(serialized_data[0]) << std::endl;

//   std::ofstream outfile(out_file_name, std::ios::binary);
//   outfile.write(serialized_data.c_str(), serialized_data.size());
//   outfile.close();
//   return 0;
  
// }


