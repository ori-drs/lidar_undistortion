// Copyright (C) 2007, 2009, 2010, 2012, 2019 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#pragma once

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/calibration.h>
#include "lidar_undistortion/velodyne_container_base.hpp"

namespace velodyne {

constexpr double SQR(float val) {
  return val*val;
}

/**
 * Raw Velodyne packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;  // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;        // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;    // [µs]

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
}
raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
}
raw_packet_t;

struct RawDataConfig {
  std::string model;
  std::string calibrationFile;  ///< calibration file name
  double max_range;             ///< maximum range to publish
  double min_range;             ///< minimum range to publish
  double view_direction;
  double view_width;
};

/** \brief Velodyne data conversion class */
class RawData {
public:
  RawData() = default;
  virtual ~RawData() = default;

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns an optional calibration
   */
  std::string getCalibrationFilename(ros::NodeHandle private_nh);

  int numLasers() {
    return calibration_.num_lasers;
  }


  /** \brief Set up for data processing offline.
   * Performs the same initialization as in setup, in the abscence of a ros::NodeHandle.
   * this method is useful if unpacking data directly from bag files, without passing
   * through a communication overhead.
   *
   * @param calibration_file path to the calibration file
   * @param max_range_ cutoff for maximum range
   * @param min_range_ cutoff for minimum range
   * @param model
   * @returns 0 if successful;
   *           errno value for failure
   */
  int setup(const RawDataConfig& config);

  template <class PointT>
  void unpack(const velodyne_msgs::VelodynePacket& pkt,
              velodyne::VelodyneContainerBase<PointT>& data,
              const uint64_t& scan_start_time);

  int scansPerPacket() const;

private:
  /** configuration parameters */

  RawDataConfig config_;
  int min_angle;                ///< minimum angle to publish
  int max_angle;                ///< maximum angle to publish

  /**
   * Calibration file
   */
  velodyne_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // timing offset lookup table
  std::vector< std::vector<float> > timing_offsets;

  /** \brief setup per-point timing offsets
   * 
   *  Runs during initialization and determines the firing time for each point in the scan
   * 
   *  NOTE: Does not support all sensors yet (vlp16, vlp32, and hdl32 are currently supported)
   */
  bool buildTimings();

  void setParameters();

  /** add private function to handle the VLP16 **/
  template <class PointT>
  void unpack_vlp16(const velodyne_msgs::VelodynePacket& pkt,
                    velodyne::VelodyneContainerBase<PointT>& data,
                    const uint64_t scan_start_time);
};

/** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
template <class PointT>
void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt,
                     velodyne::VelodyneContainerBase<PointT>& data,
                     const uint64_t& scan_start_time)
{
  using velodyne_pointcloud::LaserCorrection;
  ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp.toNSec());

  /** special parsing for the VLP16 **/
  if (calibration_.num_lasers == 16) {
    unpack_vlp16(pkt, data, scan_start_time);
    return;
  }

  int64_t time_diff_start_to_this_packet = (pkt.stamp.toNSec() - scan_start_time);

  const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {

    // upper bank lasers are numbered [0..31]
    // NOTE: this is a change from the old velodyne_common implementation

    int bank_origin = 0;
    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {

      float x, y, z;
      float intensity;
      const uint8_t laser_number  = j + bank_origin;

      const LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block_t &block = raw->blocks[i];
      union two_bytes tmp;
      tmp.bytes[0] = block.data[k];
      tmp.bytes[1] = block.data[k+1];
      if (tmp.bytes[0]==0 &&tmp.bytes[1]==0 ) //no laser beam return
      {
        continue;
      }

      /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
      if ((block.rotation >= min_angle
           && block.rotation <= max_angle
           && min_angle < max_angle)
          ||(min_angle > max_angle
             && (raw->blocks[i].rotation <= max_angle
                 || raw->blocks[i].rotation >= min_angle))){
        float distance = tmp.uint * calibration_.distance_resolution_m;
        distance += corrections.dist_correction;

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
            cos_rot_table_[block.rotation] * cos_rot_correction +
            sin_rot_table_[block.rotation] * sin_rot_correction;
        float sin_rot_angle =
            sin_rot_table_[block.rotation] * cos_rot_correction -
            cos_rot_table_[block.rotation] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
        if (xx < 0) xx=-xx;
        if (yy < 0) yy=-yy;

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;
        if (corrections.two_pt_correction_available) {
          distance_corr_x =
              (corrections.dist_correction - corrections.dist_correction_x)
              * (xx - 2.4) / (25.04 - 2.4)
              + corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y =
              (corrections.dist_correction - corrections.dist_correction_y)
              * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
        ///the expression wiht '-' is proved to be better than the one with '+'
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
        /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */

        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[i].data[k+2];

        float focal_offset = 256
            * (1 - corrections.focal_distance / 13100)
            * (1 - corrections.focal_distance / 13100);
        float focal_slope = corrections.focal_slope;
        intensity += focal_slope * (std::abs(focal_offset - 256 *
                                             SQR(1 - static_cast<float>(tmp.uint)/65535)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        float time = 0;
        if (timing_offsets.size()){
          time = timing_offsets[i][j] + time_diff_start_to_this_packet;
        }
        data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[i].rotation, distance, intensity, time);
      }
    }
    data.newLine();
  }
}

/** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
template <class PointT>
void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt,
                           velodyne::VelodyneContainerBase<PointT>& data,
                           const uint64_t scan_start_time)
{
  float azimuth;
  float azimuth_diff;
  int raw_azimuth_diff;
  float last_azimuth_diff=0;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  int64_t time_diff_start_to_this_packet = (pkt.stamp.toNSec() - scan_start_time);

  const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    //std::cerr << "Block: " << block << std::endl;

    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header) {
      // Do not flood the log with messages, only issue at most one
      // of these warnings per minute.
      ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                               << block << " header value is "
                               << raw->blocks[block].header);
      return;                         // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float)(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET-1)){
      raw_azimuth_diff = raw->blocks[block+1].rotation - raw->blocks[block].rotation;
      azimuth_diff = (float)((36000 + raw_azimuth_diff)%36000);
      // some packets contain an angle overflow where azimuth_diff < 0
      if(raw_azimuth_diff < 0)//raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
      {
        ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block+1].rotation);
        // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use the same difference
        if(last_azimuth_diff > 0){
          azimuth_diff = last_azimuth_diff;
        }
        // otherwise we are not able to use this data
        // TODO: we might just not use the second 16 firings
        else{
          continue;
        }
      }
      last_azimuth_diff = azimuth_diff;
    }else{
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
      //std::cerr << "Firing: " << firing << std::endl;
      for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
        //std::cerr << "DSR: " << dsr << std::endl;
        velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];

        /** Position Calculation */
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k+1];

        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

        /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
        if ((azimuth_corrected >= min_angle
             && azimuth_corrected <= max_angle
             && min_angle < max_angle)
            ||(min_angle > max_angle
               && (azimuth_corrected <= max_angle
                   || azimuth_corrected >= min_angle))){

          // convert polar coordinates to Euclidean XYZ
          float distance = tmp.uint * calibration_.distance_resolution_m;
          distance += corrections.dist_correction;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle =
              cos_rot_table_[azimuth_corrected] * cos_rot_correction +
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle =
              sin_rot_table_[azimuth_corrected] * cos_rot_correction -
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x =
                (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4)
                + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y =
                (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;


          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

          /** Intensity Calculation */
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[block].data[k+2];

          float focal_offset = 256 * SQR(1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (std::abs(focal_offset - 256 *
                                               SQR(1 - tmp.uint/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          uint64_t time = 0;
          if (timing_offsets.size())
            time = static_cast<uint64_t>(timing_offsets[block][firing * 16 + dsr]*1e9)
                + static_cast<uint64_t>(time_diff_start_to_this_packet);
          //std::cerr << "[" << u++ << "]" << " data.addPoint("<<x_coord<<", "<<y_coord<<", " << z_coord << ", " << corrections.laser_ring << ", " << azimuth_corrected << ", " << distance << ", "<< intensity << ", " << time << ")" << std::endl;
          data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance, intensity, time);
        } else {
          ROS_WARN_STREAM("NOT ANGLE");
        }
      }
      data.newLine();
    }
  }
}

}  // namespace velodyne_rawdata
