#ifndef SERIALIZE_H
#define SERIALIZE_H

/* std library */
#include <cmath>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

/* openCV library */
#include <opencv2/highgui/highgui.hpp>

#include "struct.h"
#include "config.h"

void serialize_b_mat(const cv::Mat& b_mat, std::vector<char>& serialized_data);

void deserialize_b_mat(cv::Mat& b_mat, const std::vector<char>& serialized_data);

void serialize_coefficients(const std::vector<cv::Vec4f>& coefficients, std::vector<char>& serialized_data);

void deserialize_coefficients(std::vector<cv::Vec4f>& coefficients, const std::vector<char>& serialized_data);

void serialize_occ_mat(const cv::Mat& occ_mat, std::vector<char>& serialized_data);

void deserialize_occ_mat(cv::Mat& occ_mat, const std::vector<char>& serialized_data);

void serialize_unfit_nums(const std::vector<float>& data, std::vector<char>& serialized_data);

void deserialize_unfit_nums(std::vector<float>& data, const std::vector<char>& serialized_data);

void serialize_tile_fit_lengths(const std::vector<int>& data, std::vector<char>& serialized_data);

void deserialize_tile_fit_lengths(std::vector<int>& data, const std::vector<char>& serialized_data);

void serialize_plane_offsets(const std::vector<std::vector<float>>& data, std::vector<char>& serialized_data);

void deserialize_plane_offsets(std::vector<std::vector<float>>& data, const std::vector<char>& serialized_data, int size);

void serialize_filenames(const std::vector<std::string>& data, std::string& serialized_data);

void deserialize_filenames(std::vector<std::string>& data, const std::string& serialized_data);

void compress_string(const std::string& combined_string);

std::vector<char> serialize(const std::vector<std::vector<char>>& data);

std::vector<std::vector<char>> deserialize(const std::vector<char>& serialized_data);

// TODO: Add serialize_impl header and implementation files

#endif
