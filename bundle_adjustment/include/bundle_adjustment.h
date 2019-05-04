// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

#include <mappoint.h>
#include <camera.h>
#include <cost_function.h>

#include <map>
#include <set>
#include <vector>

namespace vslam{
namespace ba{
	
class BundleAdjustment{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	BundleAdjustment();
	~BundleAdjustment(); // free memory.
	
	void addMapPoint(MapPoint* mpt);
	void addCamera(Camera* cam);
	void addCostFunction(CostFunction* cost_func);
	MapPoint* getMapPoint(int id);
	Camera* getCamera(int id);
	void setConvergenceCondition(int max_iters, double min_delta, double min_error);
	void setVerbose(bool flag);
	void optimize();
	
private:
	void optimizationInit();    // init for the optimization.
	void computeStateIndexes(); // compute index for every state.
	void computeHAndbAndError();
	void solveNormalEquation();
	void inverseM(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_inv);
	void updateStates();
	void recoverStates();
	
	std::map<int, MapPoint*> mappoints_; // all mappoints
	std::map<int, Camera*> cameras_; // all cameras.
	std::set<CostFunction*> cost_functions_; // all cost functions.
	
	Eigen::MatrixXd J_; // Jocabian matrix.
	Eigen::MatrixXd JTinfo_;
	Eigen::MatrixXd H_; // Hassian matrix.
	Eigen::MatrixXd r_; // residual vector.
	Eigen::MatrixXd b_;
	Eigen::MatrixXd info_matrix_; // information matrix.
	Eigen::MatrixXd Delta_X_;	  // Delta_X_
	Eigen::MatrixXd I_;
	double lambda_;
	
	int n_cam_state_; // number of cameras in the state vector.
	int n_mpt_state_; // number of map points in the state vector.
	
	/* Convergence condition */
	int max_iters_;
	double min_delta_;
	double min_error_;
	double sum_error2_;
	double last_sum_error2_;

	bool verbose_;
}; // BundleAdjustment	

} //namespace ba
} //namespace vslam



#endif
