#include "io.h"
#include <sstream>
#include <zlib.h>
void serialize_b_mat(const cv::Mat& b_mat, std::string& serialized_data) {
    std::ostringstream oss;

    int cnt = 0;
    char code = 0;
    for (int row = 0; row < b_mat.rows; row++) {
        for (int col = 0; col < b_mat.cols; col++) {
            if (cnt == 8) {
                oss.write(&code, sizeof(code));
                cnt = 0;
                code = 0;
            }
            int status = b_mat.at<int>(row, col);
            code += (status << cnt);
            cnt++;
        }
    }
    if (cnt > 0) {
        oss.write(&code, sizeof(code));
    }

    serialized_data = oss.str();
}

void deserialize_b_mat(cv::Mat& b_mat, const std::string& serialized_data) {
    std::istringstream iss(serialized_data);

    int cnt = 0;
    char code = 0;
    iss.read(&code, 1);
    for (int row = 0; row < b_mat.rows; row++) {
        for (int col = 0; col < b_mat.cols; col++) {
            if (cnt == 8) {
                iss.read(&code, sizeof(code));
                cnt = 0;
            }
            if ((code >> cnt) & 1 == 1) {
                b_mat.at<int>(row, col) = 1;
            } else {
                b_mat.at<int>(row, col) = 0;
            }
            cnt++;
        }
    }
}

void serialize_coefficients(const std::vector<cv::Vec4f>& coefficients, std::string& serialized_data) {
    std::ostringstream oss;

    for (auto c : coefficients) {
        oss.write(reinterpret_cast<const char*>(&c[0]), sizeof(c[0]));
        oss.write(reinterpret_cast<const char*>(&c[1]), sizeof(c[1]));
        oss.write(reinterpret_cast<const char*>(&c[2]), sizeof(c[2]));
        oss.write(reinterpret_cast<const char*>(&c[3]), sizeof(c[3]));
    }

    serialized_data = oss.str();
}

void deserialize_coefficients(std::vector<cv::Vec4f>& coefficients, const std::string& serialized_data) {
    std::istringstream iss(serialized_data);

    float c[4];
    while (iss.read(reinterpret_cast<char*>(c), sizeof(float) * 4)) {
        coefficients.push_back(cv::Vec4f(c[0], c[1], c[2], c[3]));
    }
}

void serialize_occ_mat(const cv::Mat& occ_mat, std::string& serialized_data) {
    std::ostringstream oss;

    unsigned short code;
    for (int row = 0; row < occ_mat.rows; row++) {
        for (int col = 0; col < occ_mat.cols; col++) {
            code = (unsigned short)occ_mat.at<int>(row, col);
            oss.write(reinterpret_cast<const char*>(&code), sizeof(code));
        }
    }

    serialized_data = oss.str();
}

void deserialize_occ_mat(cv::Mat& occ_mat, const std::string& serialized_data) {
    std::istringstream iss(serialized_data);

    unsigned short code = 0;
    for (int row = 0; row < occ_mat.rows; row++) {
        for (int col = 0; col < occ_mat.cols; col++) {
            iss.read(reinterpret_cast<char*>(&code), sizeof(code));
            occ_mat.at<int>(row, col) = code;
        }
    }
}

void serialize_unfit_nums(const std::vector<float>& data, std::string& serialized_data) {
    std::ostringstream oss;

    for (auto d : data) {
        unsigned short quantized_d = (unsigned short)(d * 256);
        oss.write(reinterpret_cast<const char*>(&quantized_d), sizeof(quantized_d));
    }

    serialized_data = oss.str();
}

void deserialize_unfit_nums(std::vector<float>& data, const std::string& serialized_data) {
    std::istringstream iss(serialized_data);

    unsigned short quantized_d;
    while (iss.read(reinterpret_cast<char*>(&quantized_d), sizeof(quantized_d))) {
        float d = ((float)quantized_d) / 256.0f;
        data.push_back(d);
    }
}

void serialize_tile_fit_lengths(const std::vector<int>& data, std::string& serialized_data) {
    std::ostringstream oss;

    for (auto d : data) {
        unsigned short quantized_d = (unsigned short)(d);
        oss.write(reinterpret_cast<const char*>(&quantized_d), sizeof(quantized_d));
    }

    serialized_data = oss.str();
}

void deserialize_tile_fit_lengths(std::vector<int>& data, const std::string& serialized_data) {
    std::istringstream iss(serialized_data);

    unsigned short quantized_d;
    while (iss.read(reinterpret_cast<char*>(&quantized_d), sizeof(quantized_d))) {
        data.push_back(quantized_d);
    }
}

void serialize_plane_offsets(const std::vector<std::vector<float>>& data, std::string& serialized_data) {
    std::ostringstream oss;

    for (auto vec : data) {
        for (auto d : vec) {
            oss.write(reinterpret_cast<const char*>(&d), sizeof(d));
        }
    }

    serialized_data = oss.str();
}

void deserialize_plane_offsets(std::vector<std::vector<float>>& data, const std::string& serialized_data, int size) {
    std::istringstream iss(serialized_data);

    std::vector<float> vec;
    float d;
    while (iss.read(reinterpret_cast<char*>(&d), sizeof(d))) {
        vec.push_back(d);
        if (vec.size() == size) {
            data.push_back(std::vector<float>(vec));
            vec.clear();
        }
    }
}

void serialize_filenames(const std::vector<std::string>& data, std::string& serialized_data) {
    std::ostringstream oss;

    for (auto str : data)
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

// **Serialization**
std::string serialize(const std::vector<std::string>& data) {
    std::stringstream ss;
    ss << data.size() << '\n';

    for (const auto& str : data) {
        ss << escapeNewlines(str).size() << '\n'; // Store escaped string's size
        ss << escapeNewlines(str) << '\n';
    }
    return ss.str();
}

std::vector<std::string> deserialize(const std::string& data) {
    std::vector<std::string> result;
    std::stringstream ss(data);

    size_t vectorSize;
    ss >> vectorSize;

    // Skip the newline character after reading vectorSize
    ss.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    for (size_t i = 0; i < vectorSize; ++i) {
        size_t strSize;
        std::string str;
        ss >> strSize; 
        ss.ignore(); // Skip the newline character after reading strSize
        std::getline(ss, str); // Read the entire string (accounts for spaces)

        // Unescape newline characters
        std::string unescapedStr;
        for (size_t i = 0; i < str.size(); ++i) {
            if (str[i] == '\\' && i + 1 < str.size() && str[i + 1] == 'n') {
                unescapedStr += '\n';
                i++; // Skip the escaped '\n'
            } else {
                unescapedStr += str[i];
            }
        }
        result.push_back(unescapedStr);
    }
    return result;
}