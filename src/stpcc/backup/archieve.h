#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>  // Corrected header for gzip compressor
#include <sstream>

void archive_data(const std::vector<std::string>& data, std::string& serialized_data);

void unarchive_data(const std::string& serialized_data, std::vector<std::string>& data);