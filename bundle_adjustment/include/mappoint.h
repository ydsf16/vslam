// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <eigen3/Eigen/Core>

namespace vslam{
namespace ba{
	
class MapPoint{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	MapPoint(const Eigen::Vector3d& position, int id, bool fixed = false);
	const Eigen::Vector3d& getPosition();
	void setPosition(const Eigen::Vector3d& position);
	int getId();
	void setId(int id);
	void setFixed();
	bool isFixed();
	void addDeltaPosition(const Eigen::Vector3d& delta_position);
	
	int state_index_;
private:
	Eigen::Vector3d position_; 
	int id_;
	bool fixed_; 	
}; //class MapPoint
	
} //namespace ba
} //namespace vslam

#endif
