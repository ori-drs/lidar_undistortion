#pragma once
#include <vector>
#include <string>
#include <array>

/* copied from ouster_client repository (1.14 beta version)
BSD 3-Clause License

Copyright (c) 2018, ouster-lidar
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
namespace ouster {
namespace util {

struct version {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
};

const version invalid_version = {0, 0, 0};

version version_of_string(const std::string& s);


inline bool operator==(const version& u, const version& v) {
    return u.major == v.major && u.minor == v.minor && u.patch == v.patch;
}

inline bool operator<(const version& u, const version& v) {
    return (u.major < v.major) || (u.major == v.major && u.minor < v.minor) ||
           (u.major == v.major && u.minor == v.minor && u.patch < v.patch);
}

inline bool operator<=(const version& u, const version& v) {
    return u < v || u == v;
}

inline bool operator!=(const version& u, const version& v) { return !(u == v); }

inline bool operator>=(const version& u, const version& v) { return !(u < v); }

inline bool operator>(const version& u, const version& v) { return !(u <= v); }

/**
 * Get string representation of a version
 * @param version
 * @return string representation of the version
 */
std::string to_string(const version& v);
} // namespace util

namespace sensor {

enum lidar_mode {
    MODE_UNSPEC = 0,
    MODE_512x10,
    MODE_512x20,
    MODE_1024x10,
    MODE_1024x20,
    MODE_2048x10
};

struct data_format {
    uint32_t pixels_per_column;
    uint32_t columns_per_packet;
    uint32_t columns_per_frame;
    std::vector<int> pixel_shift_by_row;
};

struct sensor_info {
    std::string hostname;
    std::string sn;
    std::string fw_rev;
    lidar_mode mode;
    std::string prod_line;
    data_format format;
    std::vector<double> beam_azimuth_angles;
    std::vector<double> beam_altitude_angles;
    std::vector<double> imu_to_sensor_transform;
    std::vector<double> lidar_to_sensor_transform;
    double lidar_origin_to_beam_origin_mm;
};

data_format default_data_format(lidar_mode mode);

/**
 * Get number of columns in a scan for a lidar mode
 * @param lidar_mode
 * @return number of columns per rotation for the mode
 */
int n_cols_of_lidar_mode(lidar_mode mode);

const util::version min_version = {1, 12, 0};

/**
 * Design values for altitude and azimuth offset angles for gen1 sensors. Can be
 * used if calibrated values are not available.
 */
const std::vector<double> gen1_altitude_angles = {
  16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
  12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
  8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
  3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
  -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
  -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
  -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
  -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

const std::vector<double> gen1_azimuth_angles = {
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

const std::vector<double> imu_to_sensor_transform = {
    1, 0, 0, 6.253, 0, 1, 0, -11.775, 0, 0, 1, 7.645, 0, 0, 0, 1};

const std::vector<double> lidar_to_sensor_transform = {
    -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 36.18, 0, 0, 0, 1};

sensor_info parse_metadata(const std::string& meta);

lidar_mode lidar_mode_of_string(const std::string& s);

double default_lidar_origin_to_beam_origin(std::string prod_line);

const std::array<std::pair<lidar_mode, std::string>, 5> lidar_mode_strings = {
    {{MODE_512x10, "512x10"},
     {MODE_512x20, "512x20"},
     {MODE_1024x10, "1024x10"},
     {MODE_1024x20, "1024x20"},
     {MODE_2048x10, "2048x10"}}};

} // namespace sensor
} // namespace ouster

namespace lidar_undistortion {

void populate_metadata_defaults(ouster::sensor::sensor_info& info,
                                ouster::sensor::lidar_mode specified_lidar_mode);

std::string read_metadata(const std::string& meta_file);
} // namespace lidar_undistortion
