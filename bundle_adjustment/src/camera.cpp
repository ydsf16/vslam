// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <camera.h>
#include <iostream>

namespace vslam {
namespace ba {

	Camera::Camera ( const Sophus::SE3& pose, int id, bool fixed ):
pose_(pose), id_(id), fixed_(fixed), state_index_(-1)
{
} // Camera

const Sophus::SE3& Camera::getPose()
{
	return pose_;
} // getPose

void Camera::setPose ( const Sophus::SE3& pose )
{
	pose_ = pose;
} // setPose

int Camera::getId()
{
	return id_;
} // getId

void Camera::setId ( int id )
{
	id_ = id;
} // setId

void Camera::setFixed()
{
	fixed_ = true;
} // setFixed

bool Camera::isFixed()
{
	return fixed_;
} // isFixed


void Camera::addDeltaPose ( const Eigen::Matrix< double, int ( 6 ), int ( 1 ) >& delta_pose )
{
	Sophus::SE3 delta_SE3 = Sophus::SE3::exp(delta_pose);
	pose_ = delta_SE3 * pose_;
} // addDeltaPose

void Camera::addDeltaPose ( const Sophus::SE3& delta_pose )
{
	pose_ = delta_pose * pose_;
} // addDeltaPose
	
} // namespace ba
} // namespace vslam