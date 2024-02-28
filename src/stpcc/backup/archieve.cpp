#include "archieve.h"
#include <zlib.h>

// void archieve_data(const std::vector<std::string>& data, std::string& serialized_data) {
//     std::ostringstream serialized_output;
//     std::ostringstream oss;
//     // Start archive using Boost.Archive library
//     boost::archive::binary_oarchive oa(serialized_output);

//     // Serialize each string object
//     for (const auto& str : data) {
//         oa << str;
//     }

//     // Use Boost.Iostreams for compression
//     boost::iostreams::filtering_ostream out;
//     out.push(boost::iostreams::gzip_compressor());  // Use gzip_compressor() for compression
//     out.push(serialized_output);
//     boost::iostreams::copy(out, oss);
//     // Copy archive data to string output
//     std::string serialized_data = oss.str() // Directly copy to serialized_data
// }



void archive_data(const std::vector<std::string>& data, std::string& serialized_data) {
    std::ostringstream serialized_output;
    // Start archive using Boost.Archive library
    boost::archive::binary_oarchive oa(serialized_output);

    // Serialize each string object
    for (const auto& str : data) {
        oa << str;
    }

    // Use Boost.Iostreams for compression
    boost::iostreams::filtering_ostream out;
    out.push(boost::iostreams::gzip_compressor());  // Use gzip_compressor() for compression
    out.push(boost::iostreams::back_inserter(serialized_data));
    out << serialized_output.str();
    boost::iostreams::close(out);
}

void unarchive_data(const std::string& serialized_data, std::vector<std::string>& data) {
  std::istringstream serialized_input(serialized_data);
  std::vector<std::string> deserialized_data;

  try {
    // Decompression and deserialization
    boost::archive::binary_iarchive ia(
        boost::iostreams::filtering_istream(boost::iostreams::filtering_stream_base(serialized_input),
                                            boost::iostreams::gzip_decompressor()));

    // Desinize each string object
    std::string str;
    while (ia >> str) {
      deserialized_data.push_back(str);
    }

    // Handle potential end-of-stream gracefully
    if (ia.eof()) {
      std::cout << "Reached end of deserialized data." << std::endl;
    }
  } catch (const boost::archive::archive_exception& e) {
    std::cerr << "Deserialization error: " << e.what() << std::endl;
    // Consider additional error handling or recovery actions here.
  } catch (const std::exception& e) {
    std::cerr << "Unexpected error during decompression: " << e.what() << std::endl;
    // Handle other potential exceptions (e.g., file errors, network errors).
  }

  // Swap the deserialized data into the original vector
  std::swap(data, deserialized_data);
}
