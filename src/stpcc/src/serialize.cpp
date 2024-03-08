#include "io.h"
#include <sstream>
#include <zlib.h>


void serialize_b_mat(const cv::Mat& b_mat, std::vector<char>& serialized_data) {
    

    int cnt = 0;
    char code = 0;
    for (int row = 0; row < b_mat.rows; row++) {
        for (int col = 0; col < b_mat.cols; col++) {
            if (cnt == 8) {
                serialized_data.push_back(code); // Append to the vector
                cnt = 0;
                code = 0;
            }
            int status = b_mat.at<int>(row, col);
            code += (status << cnt);
            cnt++;
        }
    }
    if (cnt > 0) {
        serialized_data.push_back(code); // Append the final code
    }
}

void deserialize_b_mat(cv::Mat& b_mat, const std::vector<char>& serialized_data) {
    
    int cnt = 0;
    char code = 0;

    int data_index = 0;
    // Read first byte (assuming it's already available in the vector)
    code = serialized_data[data_index++];

    for (int row = 0; row < b_mat.rows; row++) {
        for (int col = 0; col < b_mat.cols; col++) {
            if (cnt == 8) {
                // Read next byte if needed
                if (data_index < serialized_data.size()) {
                    code = serialized_data[data_index++];
                }
                cnt = 0;
            }
            if ((code >> cnt & 1) == 1) {
                b_mat.at<int>(row, col) = 1;
            } else {
                b_mat.at<int>(row, col) = 0;
            }
            cnt++;
        }
    }
}




void serialize_coefficients(const std::vector<cv::Vec4f>& coefficients, std::vector<char>& serialized_data) {
    int num_coefficients = coefficients.size();
    int bytes_per_coefficient = 4 * sizeof(float); // Each Vec4f has 4 floats

    // Calculate total required buffer size
    serialized_data.resize(num_coefficients * bytes_per_coefficient);

    char* data_ptr = serialized_data.data(); // Pointer to the start of the buffer

    for (const auto& c : coefficients) {
        // Efficiently copy each float directly into the buffer
        std::memcpy(data_ptr, &c[0], sizeof(float));
        data_ptr += sizeof(float);
        std::memcpy(data_ptr, &c[1], sizeof(float));
        data_ptr += sizeof(float);
        std::memcpy(data_ptr, &c[2], sizeof(float));
        data_ptr += sizeof(float);
        std::memcpy(data_ptr, &c[3], sizeof(float));
        data_ptr += sizeof(float);
    }
}


void deserialize_coefficients(std::vector<cv::Vec4f>& coefficients, const std::vector<char>& serialized_data) {
    int num_elements = serialized_data.size() / sizeof(float);
    if (num_elements % 4 != 0) {
        std::cerr << "Invalid serialized data size. Data size must be a multiple of 4 bytes (float size)." << std::endl;
        return;
    }

    const char* data_ptr = serialized_data.data(); // Pointer to the start of the byte array

    for (int i = 0; i < num_elements; i += 4) {
        float c[4];
        // Efficiently copy data from the buffer directly into the float array
        std::memcpy(c, data_ptr, sizeof(float) * 4);
        data_ptr += sizeof(float) * 4;
        coefficients.push_back(cv::Vec4f(c[0], c[1], c[2], c[3]));
    }
}

void serialize_occ_mat(const cv::Mat& occ_mat, std::vector<char>& serialized_data) {
    int num_elements = occ_mat.rows * occ_mat.cols;

    // Calculate required buffer size
    serialized_data.resize(num_elements * sizeof(unsigned short));

    unsigned short* data_ptr = reinterpret_cast<unsigned short*>(serialized_data.data()); // Pointer to buffer

    for (int row = 0; row < occ_mat.rows; row++) {
        for (int col = 0; col < occ_mat.cols; col++) {
            *data_ptr++ = (unsigned short)occ_mat.at<int>(row, col);
        }
    }
}

void deserialize_occ_mat(cv::Mat& occ_mat, const std::vector<char>& serialized_data) {
    int num_elements = serialized_data.size() / sizeof(unsigned short);
    if (num_elements != occ_mat.rows * occ_mat.cols) {
        std::cerr << "Invalid serialized data size. Expected " << (occ_mat.rows * occ_mat.cols) << " elements, got " << num_elements << " elements." << std::endl;
        return;
    }

    const unsigned short* data_ptr = reinterpret_cast<const unsigned short*>(serialized_data.data()); // Pointer to buffer

    for (int row = 0; row < occ_mat.rows; row++) {
        for (int col = 0; col < occ_mat.cols; col++) {
            occ_mat.at<int>(row, col) = *data_ptr++;
        }
    }
}


void serialize_unfit_nums(const std::vector<float>& data, std::vector<char>& serialized_data) {
    // Precalculate required buffer size for efficiency
    int buffer_size = data.size() * sizeof(unsigned short);
    serialized_data.resize(buffer_size);

    unsigned short* data_ptr = reinterpret_cast<unsigned short*>(serialized_data.data()); // Pointer to buffer

    for (auto d : data) {
        unsigned short quantized_d = (unsigned short)(d * 256);
        *data_ptr++ = quantized_d;
    }
}

void deserialize_unfit_nums(std::vector<float>& data, const std::vector<char>& serialized_data) {
    int num_elements = serialized_data.size() / sizeof(unsigned short);

    const unsigned short* data_ptr = reinterpret_cast<const unsigned short*>(serialized_data.data()); // Pointer to buffer

    for (int i = 0; i < num_elements; i++) {
        float d = ((float)(*data_ptr++)) / 256.0f;
        data.push_back(d);
    }
}


void serialize_tile_fit_lengths(const std::vector<int>& data, std::vector<char>& serialized_data) {
    int num_elements = data.size();
    // Precalculate buffer size for efficiency
    int buffer_size = num_elements * sizeof(unsigned short);
    serialized_data.resize(buffer_size);

    unsigned short* data_ptr = reinterpret_cast<unsigned short*>(serialized_data.data()); // Pointer to buffer

    for (auto d : data) {
        *data_ptr++ = (unsigned short)d; // Cast to unsigned short for serialization
    }
}

void deserialize_tile_fit_lengths(std::vector<int>& data, const std::vector<char>& serialized_data) {
    int num_elements = serialized_data.size() / sizeof(unsigned short);

    const unsigned short* data_ptr = reinterpret_cast<const unsigned short*>(serialized_data.data()); // Pointer to buffer

    for (int i = 0; i < num_elements; i++) {
        data.push_back(*data_ptr++);
    }
}

void serialize_plane_offsets(const std::vector<std::vector<float>>& data, std::vector<char>& serialized_data) {
    // Calculate total number of floats to serialize
    int num_floats = 0;
    for (const auto& vec : data) {
        num_floats += vec.size();
    }

    // Resize the buffer to hold all floats
    serialized_data.resize(num_floats * sizeof(float));

    float* data_ptr = reinterpret_cast<float*>(serialized_data.data()); // Pointer to buffer

    for (const auto& vec : data) {
        std::memcpy(data_ptr, &vec[0], vec.size() * sizeof(float)); // Efficiently copy floats
        data_ptr += vec.size();
    }
}

void deserialize_plane_offsets(std::vector<std::vector<float>>& data, const std::vector<char>& serialized_data, int size) {
    int num_floats = serialized_data.size() / sizeof(float);

    const float* data_ptr = reinterpret_cast<const float*>(serialized_data.data()); // Pointer to buffer

    int floats_read = 0;
    while (floats_read < num_floats) {
        std::vector<float> vec(data_ptr, data_ptr + size); // Create a vector from the buffer
        data.push_back(vec);
        data_ptr += size;
        floats_read += size;
    }
}


void serialize_filenames(const std::vector<std::string>& data, std::string& serialized_data) {
    std::ostringstream oss;

    for (const auto& str : data)
        oss << str << std::endl;

    serialized_data = oss.str();
}

void deserialize_filenames(std::vector<std::string>& data, const std::string& serialized_data) {
    std::istringstream iss(serialized_data);

    std::string str;
    while (getline(iss, str))
        data.push_back(str);
}

// Function to escape newline characters
std::string escapeNewlines(const std::string& str) {
    std::string escapedStr;
    for (char c : str) {
        if (c == '\n') {
            escapedStr += "\\n";
        } else {
            escapedStr += c;
        }
    }
    return escapedStr;
}

// // **Serialization**
// std::vector<char> serialize(const std::vector<std::vector<char>>& data) {
//     std::vector<char> serialized_data;

//     // Calculate total size needed for lengths, buffer, and null terminators
//     size_t total_size = sizeof(size_t); // Start with size for vector length

//     for (const auto& str : data) {
//         total_size += sizeof(size_t) + str.size() + 1; // Size, string, null terminator
//     }

//     serialized_data.reserve(total_size); // Pre-allocate for efficiency

//     // Append vector length
//     serialized_data.insert(serialized_data.end(), reinterpret_cast<char*>(&total_size), reinterpret_cast<char*>(&total_size) + sizeof(size_t));

//     // Append each string with its size
//     for (const auto& str : data) {
//         size_t str_size = str.size();
//         serialized_data.insert(serialized_data.end(), reinterpret_cast<char*>(&str_size), reinterpret_cast<char*>(&str_size) + sizeof(size_t));
//         serialized_data.insert(serialized_data.end(), str.begin(), str.end());
//         // serialized_data.push_back('\0'); // Add null terminator
//     }
//     serialized_data.push_back('\0'); // Add null terminator
//     return serialized_data;
// }

// std::vector<std::vector<char>> deserialize(const std::vector<char>& data) {
//     std::vector<std::vector<char>> result;

//     const char* data_ptr = data.data(); // Pointer to the start of the buffer

//     // Read the vector length
//     size_t vec_length = *reinterpret_cast<const size_t*>(data_ptr);
//     data_ptr += sizeof(size_t);

//     for (size_t i = 0; i < vec_length; ++i) {
//         // Read the string size
//         size_t str_size = *reinterpret_cast<const size_t*>(data_ptr);
//         data_ptr += sizeof(size_t);

//         // Read the string
//         std::vector<char> str(data_ptr, data_ptr + str_size);
//         data_ptr += str_size;

//         result.push_back(str);
//     }

//     return result;
// }




// Serialization function
std::vector<char> serialize(const std::vector<std::vector<char>>& data) {
    std::vector<char> serialized_data;

    // Serialize the vector length
    size_t vec_length = data.size();
    serialized_data.insert(serialized_data.end(), reinterpret_cast<const char*>(&vec_length), reinterpret_cast<const char*>(&vec_length) + sizeof(size_t));

    for (const auto& str : data) {
        // Serialize the string size
        size_t str_size = str.size();
        serialized_data.insert(serialized_data.end(), reinterpret_cast<const char*>(&str_size), reinterpret_cast<const char*>(&str_size) + sizeof(size_t));

        // Serialize the string
        serialized_data.insert(serialized_data.end(), str.begin(), str.end());
    }

    return serialized_data;
}

// Deserialization function
std::vector<std::vector<char>> deserialize(const std::vector<char>& serialized_data) {
    std::vector<std::vector<char>> result;

    const char* data_ptr = serialized_data.data(); // Pointer to the start of the buffer

    // Read the vector length
    size_t vec_length = *reinterpret_cast<const size_t*>(data_ptr);
    data_ptr += sizeof(size_t);

    for (size_t i = 0; i < vec_length; ++i) {
        // Read the string size
        size_t str_size = *reinterpret_cast<const size_t*>(data_ptr);
        data_ptr += sizeof(size_t);

        // Read the string
        std::vector<char> str(data_ptr, data_ptr + str_size);
        data_ptr += str_size;

        result.push_back(str);
    }

    return result;
}
