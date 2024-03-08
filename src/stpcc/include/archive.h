#ifndef ARCHIVE_H
#define ARCHIVE_H

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

// #include <gzip/compress.hpp>
// #include <gzip/config.hpp>
// #include <gzip/decompress.hpp>
// #include <gzip/utils.hpp>
// #include <gzip/version.hpp>

#include "pcc_module.h"
#include "encoder.h"
#include "decoder.h"
#include "io.h"
#include "serialize.h"
#include "archive.h"

void compress_string(const std::string& data, std::string& compressed_data);

void extract_string(const std::string& compressed_data, std::string& decompressed_data);

void compress_data(const std::vector<std::string>& data, std::string& serialized_data);

void extract_data(const std::string& serialized_data, std::vector<std::string>& data);

std::vector<std::vector<char>> compress_pointcloud(std::vector<point_cloud> pcloud_data, int row, int col, float pitch_precision, float yaw_precision, float threshold, int tile_size, PccResult &pcc_res);

std::vector<point_cloud> extract_pointcloud(const std::vector<std::vector<char>>& combined_strings, int row, int col, float pitch_precision, float yaw_precision, float threshold, int tile_size);

#endif
