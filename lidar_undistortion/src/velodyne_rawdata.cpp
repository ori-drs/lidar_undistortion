/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2019, Kaarta Inc, Shawn Hanna
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Shawn Hanna
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include "lidar_undistortion/velodyne_rawdata.hpp"

using namespace velodyne;

constexpr double from_degrees(double degrees){
  return degrees * M_PI / 180.0;
}

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

/** Update parameters: conversions and update */
void RawData::setParameters() {
  //converting angle parameters into the velodyne reference (rad)
  double tmp_min_angle = config_.view_direction + config_.view_width/2;
  double tmp_max_angle = config_.view_direction - config_.view_width/2;

  //computing positive modulo to keep theses angles into [0;2*M_PI]
  tmp_min_angle = fmod(fmod(tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
  tmp_max_angle = fmod(fmod(tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);

  //converting into the hardware velodyne ref (negative yaml and degrees)
  //adding 0.5 perfomrs a centered double to int conversion
  min_angle = 100 * (2*M_PI - tmp_min_angle) * 180 / M_PI + 0.5;
  max_angle = 100 * (2*M_PI - tmp_max_angle) * 180 / M_PI + 0.5;
  if (min_angle == max_angle) {
    //avoid returning empty cloud if min_angle = max_angle
    min_angle = 0;
    max_angle = 36000;
  }
}

int RawData::scansPerPacket() const {
  if(calibration_.num_lasers == 16) {
    return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK *
        VLP16_SCANS_PER_FIRING;
  } else {
    return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
  }
}

/**
   * Build a timing table for each block/firing. Stores in timing_offsets vector
   */
bool RawData::buildTimings(bool print){
  // vlp16
  if (config_.model == "VLP16"){
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (size_t i=0; i < timing_offsets.size(); ++i){
      timing_offsets[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6; // seconds
    double single_firing = 2.304 * 1e-6; // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x){
      for (size_t y = 0; y < timing_offsets[x].size(); ++y){
        if (dual_mode){
          dataBlockIndex = (x - (x % 2)) + (y / 16);
        }
        else{
          dataBlockIndex = (x * 2) + (y / 16);
        }
        dataPointIndex = y % 16;
        //timing_offsets[block][firing]
        timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
      }
    }
  }
  // vlp32
  else if (config_.model == "32C"){
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (size_t i=0; i < timing_offsets.size(); ++i){
      timing_offsets[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6; // seconds
    double single_firing = 2.304 * 1e-6; // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x){
      for (size_t y = 0; y < timing_offsets[x].size(); ++y){
        if (dual_mode){
          dataBlockIndex = x / 2;
        }
        else{
          dataBlockIndex = x;
        }
        dataPointIndex = y / 2;
        timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
      }
    }
  }
  // hdl32
  else if (config_.model == "32E"){
    // timing table calculation, from velodyne user manual
    timing_offsets.resize(12);
    for (size_t i=0; i < timing_offsets.size(); ++i){
      timing_offsets[i].resize(32);
    }
    // constants
    double full_firing_cycle = 46.080 * 1e-6; // seconds
    double single_firing = 1.152 * 1e-6; // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets.size(); ++x){
      for (size_t y = 0; y < timing_offsets[x].size(); ++y){
        if (dual_mode){
          dataBlockIndex = x / 2;
        }
        else{
          dataBlockIndex = x;
        }
        dataPointIndex = y / 2;
        timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
      }
    }
  }
  else{
    timing_offsets.clear();
    std::cerr << "Timings not supported for model " << config_.model.c_str() << std::endl;
  }

  if(timing_offsets.empty()){
    std::cerr << "NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED VELODYNE SENSOR?" << std::endl;
    return false;
  }

  if(print){
    std::cout << "VELODYNE TIMING TABLE:" << std::endl;
    for (size_t x = 0; x < timing_offsets.size(); ++x){
      for (size_t y = 0; y < timing_offsets[x].size(); ++y){
        printf("%04.3f ", timing_offsets[x][y] * 1e6);
      }
      printf("\n");
    }
  }

  return true;
}

/** Set up for on-line operation. */
//std::string RawData::getCalibrationFilename(ros::NodeHandle private_nh) {
//
//  std::string calibrationFile = "";
//  // get path to angles.config file for this device
//  if (!private_nh.getParam("calibration", calibrationFile)) {
//    ROS_ERROR_STREAM("No calibration angles specified! Using test values!");
//
//    // have to use something: grab unit test version as a default
//    std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
//    calibrationFile = pkgPath + "/params/64e_utexas.yaml";
//  }
//
//  return calibrationFile;
//}

/** Set up for offline operation */
bool RawData::setup(const RawDataConfig& config, bool print) {
  config_ = config;
  if(print){
    std::cout << "data ranges to publish: ["
              << config_.min_range << ", "
              << config_.max_range << "]" << std::endl;

    std::cout << "correction angles: " << config_.calibrationFile << std::endl;
  }
  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized) {
    std::cerr << "Unable to open calibration file: " << config_.calibrationFile <<std::endl;
    return false;
  }
  setParameters();

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }
  buildTimings(print);
  if(print){
    std::cout << "Correctly setup Velodyne calibration. " << std::endl;
    std::cout << "Number of lasers: " << calibration_.num_lasers << "." << std::endl;
  }
  return true;
}





