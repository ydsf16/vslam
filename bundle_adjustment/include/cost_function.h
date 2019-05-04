// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <mappoint.h>
#include <camera.h>

namespace vslam{
namespace ba{
	
class CostFunction{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	CostFunction(MapPoint* map_point, Camera* camera, 
				 double fx, double fy, double cx, double cy, const Eigen::Vector2d& ob_z);
	
	void setHuberParameter(double b = 1.0);
	void setCovariance(const Eigen::Matrix2d& cov);
	
	void computeInterVars(Eigen::Vector2d& e, Eigen::Matrix2d& weighted_info, double& weighted_e2);
	
	void computeJT(Eigen::Matrix<double, 2, 6>& JT);
	void computeJX(Eigen::Matrix<double, 2, 3>& JX);
	
	MapPoint* map_point_;
	Camera* camera_;
private:
	double fx_, fy_, cx_, cy_;
	Eigen::Vector2d ob_z_;
	Eigen::Matrix2d info_matrix_;
	double huber_b_;                                      
}; // class CostFunction
		
} // namespace ba
} // namespace vslam

#endif