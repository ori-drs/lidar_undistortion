#include "lidar_undistortion/pose_buffer.hpp"
#include "lidar_undistortion/print_macros.hpp"
#include <iostream>

bool PoseBuffer::canInterpolate(uint64_t nsec){
  return !pose_history_.empty() &&
    nsec >= pose_history_.begin()->first &&
    nsec <= pose_history_.rbegin()->first &&
    (pose_history_.find(nsec) != pose_history_.end() ||
           pose_history_.size() >= 2);
}

void PoseBuffer::addPose(uint64_t nsec, const Eigen::Isometry3d &pose){
  DEBUG_PRINTLN("Adding new element to pose history...");
  pose_history_[nsec] = pose;
  DEBUG_PRINTLN("Pose history size: " << pose_history_.size());
  // clean up history longer than 1 s
  DEBUG_PRINTLN("Check if history has to be cleaned...");
  DEBUG_PRINTLN(toString());
  DEBUG_PRINTLN("Buffer size  : " << buffer_size_);
  DEBUG_PRINTLN("Buffer length: " << endTime() - startTime());
  const auto& old_it = pose_history_.lower_bound(pose_history_.rbegin()->first - buffer_size_);


  if(old_it != pose_history_.end()){
    DEBUG_PRINTLN("Erasing history prior to " << old_it->first);
    pose_history_.erase(pose_history_.begin(), old_it);
    DEBUG_PRINTLN("Pose history size: " << pose_history_.size());
  } else {
    DEBUG_PRINTLN("No cleaning necessary.");
  }
}

bool PoseBuffer::getInterpolatedPose(const uint64_t &nsec,
                                           Eigen::Isometry3d& pose) const
{
  if(pose_history_.empty() || nsec < pose_history_.begin()->first || nsec > pose_history_.rbegin()->first)
  {
    DEBUG_PRINTLN("[ getInterpolatedPose ]: empty history or requested time out of bounds. "
                    " Pose history size: " << pose_history_.size());
    return false;
  }
  // this is the first element greater or equal than utime, or end()
  PoseHistory::const_iterator it_low = pose_history_.lower_bound(nsec);
  // if it's end(), we return
  if(it_low == pose_history_.end()){
    DEBUG_PRINTLN("[ getInterpolatedPose ]: reached bottom. "
                  "\n Pose history size: " << pose_history_.size());
    return false;
  }
  // if it's equal, we return it
  if(it_low->first == nsec){
    pose = it_low->second;
    DEBUG_PRINTLN("[ getInterpolatedPose ]: " << it_low->first << " == " << nsec <<
                 " Returning pose. \n Pose history size: " << pose_history_.size());

    return true;
  }

  // at this point we have to interpolate, and we can't do it with less than
  // two items in history
  if(pose_history_.size() < 2){
    DEBUG_PRINTLN("[ getInterpolatedPose ]: size less than 2. "
                    "\n history size: " << pose_history_.size());

    return false;
  }
  // if we are here it means it_low has a value greater than what we ask
  PoseHistory::const_iterator it_high = it_low;

  // now it_low contains the biggest element smaller than nsec
  --it_low;

  // alpha is 1 if the requested time coincides with it_low, 0 if equal to it_high
  double alpha = (double)(it_high->first - nsec) / (double)(it_high->first - it_low->first);

  Eigen::Isometry3d iso_low = it_low->second;
  Eigen::Isometry3d iso_high = it_high->second;

  pose.setIdentity();

  pose.translation() = iso_low.translation() * alpha + iso_high.translation() * (1.0 - alpha);
  // in slerp, the paramter t is used as the opposite of alpha
  // (1 - t) * p0 + t * p1

  // the "linear()" returns a reference to the rotation
  // matrix of the transform
  pose.linear() = Quaternion(iso_high.rotation()).slerp(alpha, Quaternion(iso_low.rotation())).toRotationMatrix();
  DEBUG_PRINTLN("Alpha %      : " << std::floor(alpha*100));
  DEBUG_PRINTLN("iso low tran : " << iso_low.translation().transpose());
  DEBUG_PRINTLN("iso intr tran: " << pose.translation().transpose());
  DEBUG_PRINTLN("iso high tran: " << iso_high.translation().transpose()<< std::endl);
  return true;
}
