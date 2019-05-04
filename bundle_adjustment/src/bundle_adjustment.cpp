// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <bundle_adjustment.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>
#include <iomanip>
#include <run_timer.h>

namespace vslam
{

namespace ba
{

BundleAdjustment::BundleAdjustment ()
    :max_iters_ ( 20 ), min_delta_ ( 1e-10 ), min_error_ ( 1e-10 ),
     last_sum_error2_ ( std::numeric_limits<double>::max() ), verbose_ ( false )
{
} // BundleAdjustment

BundleAdjustment::~BundleAdjustment()
{
    // delete all
    std::map<int, Camera*>::iterator it_cam;
    for ( it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam ++ ) {
        Camera* cam = it_cam->second;
        delete cam;
    } // for all cameras

    std::map<int, MapPoint*>::iterator it_mpt;
    for ( it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++ ) {
        MapPoint* mpt = it_mpt->second;
        delete mpt;
    }

    std::set<CostFunction*>::iterator it_cost;
    for ( it_cost = cost_functions_.begin(); it_cost != cost_functions_.end(); it_cost++ ) {
        CostFunction* cost_func = *it_cost;
        delete cost_func;
    }
} // ~BundleAdjustment


void BundleAdjustment::addMapPoint ( MapPoint* mpt )
{
    mappoints_[mpt->getId()] = mpt; // add mpt.
} // addMapPoint

void BundleAdjustment::addCamera ( Camera* cam )
{
    cameras_[cam->getId()] = cam;
} // addCamera

void BundleAdjustment::addCostFunction ( CostFunction* cost_func )
{
    cost_functions_.insert ( cost_func );
} // addCostFunction

Camera* BundleAdjustment::getCamera ( int id )
{
    std::map<int, Camera*>::iterator iter = cameras_.find ( id );
    if ( iter != cameras_.end() ) {
        return iter->second;
    }
    return nullptr;
} // getCamera

MapPoint* BundleAdjustment::getMapPoint ( int id )
{
    std::map<int, MapPoint*>::iterator iter = mappoints_.find ( id );
    if ( iter != mappoints_.end() ) {
        return iter->second;
    }
    return nullptr;
} // getMapPoint

void BundleAdjustment::setConvergenceCondition ( int max_iters, double min_delta, double min_error )
{
    max_iters_ = max_iters;
    min_delta_ = min_delta;
    min_error_ = min_error;
} // setConvergenceCondition

void BundleAdjustment::setVerbose ( bool verbose_flag )
{
    verbose_ = verbose_flag;
} // setVerbose

void BundleAdjustment::optimize()
{
    // initlization.
    optimizationInit();

    int niter = 0;
    double upsilon = 2.0;
    const double tau = 1e-8;

    // compute A and g
    computeHAndbAndError();
	last_sum_error2_ = sum_error2_;
	
    // found.
    bool found = ( ( -b_ ).lpNorm<Eigen::Infinity>() < min_error_ );
	
    //mu
    std::vector<double> aa;
    aa.reserve ( H_.rows() );
    for ( int i = 0; i < H_.rows(); i ++ ) {
        aa.push_back ( H_ ( i,i ) );
    }
    auto max_aa = std::max_element ( aa.begin(), aa.end() );
    double mu = tau* ( *max_aa );

	double total_time = 0.0;
    while ( !found && niter < max_iters_ ) {
		// start timer.
		Runtimer t;
		t.start();
		
        niter ++; // add iter

        // Solve the normal equaltion.
        H_ += mu * I_;
        solveNormalEquation();

        double delta = Delta_X_.norm(); // check if the step size.

		if ( delta < min_delta_ ) {
            break;
        }
        
        // update states.
        updateStates();

		// compute new H_ and error.
		computeHAndbAndError();

		double varrho = (last_sum_error2_ - sum_error2_) / (Delta_X_.transpose() * (mu * Delta_X_ + b_))(0,0) ;
		if(varrho > 0)
		{
			last_sum_error2_ = sum_error2_;
			// check if found.
			found = ( ( -b_ ).lpNorm<Eigen::Infinity>() < min_error_ );
			mu = mu * std::max<double>(0.3333, 1.0-std::pow(2.0*varrho-1.0, 3));
			upsilon = 2.0;
		}
		else
		{
			recoverStates();
			computeHAndbAndError();
			mu = mu * upsilon;
			upsilon *= 2.0;
		}
		
		t.stop();
		total_time += t.duration();
		// show
		if(verbose_)
		{ 
			std::cout << std::fixed << "Iter: " << std::left <<std::setw(4) << niter 
			<< " Cost: "<< std::left <<std::setw(20)  << std::setprecision(10) << sum_error2_ 
			<< " Step: " << std::left <<std::setw(14) << std::setprecision(10) << delta 
			<< " Time " << std::left <<std::setw(10) << std::setprecision(3) << t.duration() 
			<< " Total_time " << std::left <<std::setw(10) << std::setprecision(3) << total_time << std::endl;
		}
    } // iter
} // optimize

void BundleAdjustment::optimizationInit()
{
    computeStateIndexes();
    int state_size = n_cam_state_ * 6 + n_mpt_state_ * 3;
    int obs_size = cost_functions_.size() * 2;
    J_.resize ( obs_size, state_size );
    JTinfo_.resize ( state_size, obs_size );
    H_.resize ( state_size, state_size );
    r_.resize ( obs_size, 1 );
    b_.resize ( state_size, 1 );
    info_matrix_.resize ( obs_size, obs_size );
    Delta_X_.resize ( state_size, 1 );

    I_.resize ( state_size, state_size );
    for ( int i = 0; i < state_size; i ++ ) {
        I_ ( i,i ) = 1.0;
    }
} // optimizationInit


void BundleAdjustment::computeStateIndexes()
{
    int INDEX = 0;
    n_cam_state_ = 0;
    std::map<int, Camera*>::iterator it_cam;
    for ( it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam ++ ) {
        Camera* cam = it_cam->second;
        if ( cam->isFixed() == false ) {
            cam->state_index_ = INDEX;
            INDEX ++;
            n_cam_state_++; // get number of camera in the state vector.
        } else {
            cam->state_index_ = -1;
        }
    } // for all cameras

    INDEX = 0;
    n_mpt_state_ = 0;
    std::map<int, MapPoint*>::iterator it_mpt;
    for ( it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++ ) {
        MapPoint* mpt = it_mpt->second;

        if ( mpt->isFixed() == false ) {
            mpt->state_index_ = INDEX;
            INDEX++;
            n_mpt_state_ ++; // get number of mpts in the state vector.
        } else {
            mpt->state_index_ = -1;
        }
    }
} // computeStateIndexes


void BundleAdjustment::computeHAndbAndError()
{
    int cnt = 0;
    J_.setZero();
    H_.setZero();
    r_.setZero();
    info_matrix_.setZero();
	JTinfo_.setZero();
	
    sum_error2_ = 0.0;
    std::set<CostFunction*>::iterator iter;
    for ( iter = cost_functions_.begin(); iter != cost_functions_.end(); iter ++ ) {
        CostFunction* cost_func = *iter;

        // Compute Jacobian.
        Camera* cam = cost_func->camera_;
        if ( cam->isFixed() == false ) {
            Eigen::Matrix<double, 2, 6> JT;
            cost_func->computeJT ( JT );
            J_.block ( 2*cnt, cam->state_index_*6, 2,6 ) = JT;
        }

        MapPoint* mpt = cost_func->map_point_;
        if ( mpt->isFixed() == false ) {
            Eigen::Matrix<double, 2, 3> JX;
            cost_func->computeJX ( JX );
            J_.block ( 2*cnt, n_cam_state_*6 + mpt->state_index_*3, 2, 3 ) = JX;
        }

        // Compute Residual and Information matrix.
        Eigen::Vector2d e;
        Eigen::Matrix2d weighted_info;
        double weighted_e2;
        cost_func->computeInterVars ( e, weighted_info, weighted_e2 );

        r_.block ( 2*cnt, 0, 2, 1 ) = e;
        info_matrix_.block ( 2*cnt, 2*cnt, 2, 2 ) = weighted_info;

        sum_error2_ += weighted_e2; // add all error2.
        cnt ++;
    }// for all cost function

    // Compute H and b.
    JTinfo_ = J_.transpose() * info_matrix_;
    H_ = JTinfo_ * J_;
    b_ = -JTinfo_ * r_;
} // computeHAndb


void BundleAdjustment::solveNormalEquation()
{
    if ( n_cam_state_ >= 1 && n_mpt_state_ >= 1 ) {
        // Solve by schur.
        const Eigen::MatrixXd& C = H_.block ( 0, 0, n_cam_state_*6, n_cam_state_*6 );
        const Eigen::MatrixXd& M = H_.block ( n_cam_state_*6, n_cam_state_*6, n_mpt_state_*3, n_mpt_state_*3 );
		const Eigen::MatrixXd& Etrans = H_.block ( n_cam_state_*6, 0, n_mpt_state_*3, n_cam_state_*6);
		const Eigen::MatrixXd& E = H_.block ( 0, n_cam_state_*6, n_cam_state_*6, n_mpt_state_*3);

        const Eigen::MatrixXd& bc = b_.block ( 0, 0, n_cam_state_*6, 1 );
        const Eigen::MatrixXd& bm = b_.block ( n_cam_state_*6, 0, n_mpt_state_*3, 1 );

        // inverse M.
        Eigen::MatrixXd M_inv; 
        inverseM ( M, M_inv );

        // Solve Delta X_c
        Delta_X_.block ( 0, 0, n_cam_state_*6, 1 ) =
            ( C-E*M_inv*Etrans ).fullPivHouseholderQr().solve ( bc-E*M_inv*bm );

        // Solve Delta X_m
        Delta_X_.block ( n_cam_state_*6, 0, n_mpt_state_*3, 1 ) =
            M_inv * ( bm - Etrans * Delta_X_.block ( 0, 0, n_cam_state_*6, 1 ) );
    } else 
	{
        Delta_X_ = H_.llt().solve ( b_ );
    }
} // solveNormalEquation

void BundleAdjustment::inverseM ( const Eigen::MatrixXd& M, Eigen::MatrixXd& M_inv )
{
    // memory copy
    M_inv = M;
    for ( int i = 0; i < n_mpt_state_; i ++ ) {
        M_inv.block ( 3*i,3*i, 3,3 ) = M_inv.block ( 3*i,3*i, 3,3 ).inverse();
    }
} // inverseC

void BundleAdjustment::updateStates()
{
    std::map<int, Camera*>::iterator it_cam;
    for ( it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam ++ ) {
        Camera* cam = it_cam->second;
        if ( cam->isFixed() == false ) {
            int index = cam->state_index_;
            cam->addDeltaPose ( Delta_X_.block ( index*6, 0, 6, 1 ) );
        }
    } // for all cameras

    std::map<int, MapPoint*>::iterator it_mpt;
    for ( it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++ ) {
        MapPoint* mpt = it_mpt->second;
        if ( mpt->isFixed() == false ) {
            int index = mpt->state_index_;
            mpt->addDeltaPosition ( Delta_X_.block ( n_cam_state_*6 + index*3, 0, 3, 1 ) );
        }
    }
} // update

void BundleAdjustment::recoverStates()
{
    std::map<int, Camera*>::iterator it_cam;
    for ( it_cam = cameras_.begin(); it_cam != cameras_.end(); it_cam ++ ) {
        Camera* cam = it_cam->second;
        if ( cam->isFixed() == false ) {
            int index = cam->state_index_;
            cam->addDeltaPose ( -Delta_X_.block ( index*6, 0, 6, 1 ) );
        }
    } // for all camera_poses_

    std::map<int, MapPoint*>::iterator it_mpt;
    for ( it_mpt = mappoints_.begin(); it_mpt != mappoints_.end(); it_mpt++ ) {
        MapPoint* mpt = it_mpt->second;
        if ( mpt->isFixed() == false ) {
            int index = mpt->state_index_;
            mpt->addDeltaPosition ( -Delta_X_.block ( n_cam_state_*6 + index*3, 0, 3, 1 ) );
        }
    }
} // recoverStates

} // namespace ba

} // namespace vslam
