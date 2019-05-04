// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef CAMERA_H
#define CAMERA_H

#include <eigen3/Eigen/Core>
#include <sophus/se3.h>

namespace vslam{
namespace ba{
	
class Camera{
public:
	
	Camera(const Sophus::SE3& pose, int id, bool fixed = false);
	const Sophus::SE3& getPose();
	void setPose(const Sophus::SE3& pose);
	int getId();
	void setId(int id);
	void setFixed();
	bool isFixed();
	void addDeltaPose(const Eigen::Matrix<double, 6, 1>& delta_pose);
	void addDeltaPose(const Sophus::SE3& delta_pose);

	int state_index_;
private:
	Sophus::SE3 pose_;
	int id_;
	bool fixed_;
}; // class CameraPose
	
} //namespace ba
} //namespace vslam

#endif
