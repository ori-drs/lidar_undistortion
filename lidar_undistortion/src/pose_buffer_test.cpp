#define DEBUG_MODE 1
#include "lidar_undistortion/pose_buffer.hpp"
#include <random>
#include <iostream>
#include <iomanip>
int main(int argc, char** argv) {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint32_t> dis;
  std::uniform_real_distribution<> disr(0, 1);

  bool test_passed = true;

  for(int i = 0; i < 1000000; i++){
    PoseBuffer pb;

    double alpha = disr(gen);

    uint64_t time1 = dis(gen);
    uint64_t time2 = time1 + dis(gen) + 1;
    uint64_t time_inter = std::round(alpha * static_cast<double>(time1) + (1-alpha)*static_cast<double>(time2));

    pb.setBufferSize(time2 - time1 + 1);

    double time_alpha = static_cast<double>(time2 - time_inter) / static_cast<double>(time2 - time1);

    Eigen::Vector3d pos1 = Eigen::Vector3d::Random();
    Eigen::Vector3d pos2 = Eigen::Vector3d::Random();

    Eigen::Quaterniond orient1 = Eigen::Quaterniond::UnitRandom();
    Eigen::Quaterniond orient2 = Eigen::Quaterniond::UnitRandom();

    Eigen::Vector3d pos_inter = pos1 * time_alpha + (1 - time_alpha) * pos2;
    Eigen::Quaterniond orient_inter = orient2.slerp(time_alpha, orient1);

    Eigen::Isometry3d iso1(Eigen::Isometry3d::Identity());
    iso1.translate(pos1);
    iso1.rotate(orient1);

    Eigen::Isometry3d iso2(Eigen::Isometry3d::Identity());
    iso2.translate(pos2);
    iso2.rotate(orient2);

    Eigen::Isometry3d expected(Eigen::Isometry3d::Identity());
    expected.translate(pos_inter);
    expected.rotate(orient_inter);

    pb.addPose(time1, iso1);
    pb.addPose(time2, iso2);

    Eigen::Isometry3d result;
    pb.getInterpolatedPose(time_inter, result);

    bool temp_result = result.isApprox(expected, 1e-14);

    test_passed = test_passed && temp_result;

    if(!temp_result) {
        std::cerr << "FAILURE!!" << std::endl;
        std::cout << "alpha      : "<< std::setprecision(15) << alpha << std::endl;
        std::cout << "after_alpha: "<< std::setprecision(15) << time_alpha << std::endl;
        std::cout << "time1 = " << time1 << std::endl;
        std::cout << "time_inter = " << time_inter << std::endl;
        std::cout << "time2 = " << time2 << std::endl;
        std::cerr << std::endl << result.matrix() << std::endl;
        std::cerr << std::endl << expected.matrix() << std::endl;
        std::cerr << std::endl << expected.matrix() - result.matrix() << std::endl;
      }
  }

  if(test_passed){
    std::cerr << "ALL TEST PASSED!!" << std::endl;
    return 0;
  }
  return -1;
}
