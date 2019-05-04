// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <bundle_adjustment.h>
#include <opencv4/opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace vslam::ba;

class Observation
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Observation ( int mpt_id, int cam_id, const Eigen::Vector2d& ob )
        :mpt_id_ ( mpt_id ), cam_id_ ( cam_id ), ob_ ( ob ) {}

    int mpt_id_;
    int cam_id_;
    Eigen::Vector2d ob_;
};

// created
void createData ( int n_mappoints, int n_cameras, double fx, double fy, double cx, double cy,
                  double height, double width,
                  std::vector<Eigen::Vector3d>& mappoints, std::vector<Sophus::SE3>& cameras,
                  std::vector<Observation>& observations );

void addNoise (std::vector< Eigen::Vector3d >& mappoints, std::vector< Sophus::SE3 >& cameras, 
			   std::vector< Observation >& observations, 
			   double mpt_noise, double cam_trans_noise, double cam_rot_noise, double ob_noise 
              );


int main ( int argc, char **argv )
{
	/**** Step 1. Create data ****/
    // Params
    const int n_mappoints = 1000;
    const int n_cameras = 6;
	
	// Camera params
    const double fx = 525.0;
    const double fy = 525.0;
    const double cx = 320.0;
    const double cy = 240.0;
    const double height = 640;
    const double width = 480;
	
	// Create data.
    std::cout << "Start create data...\n";
    std::vector<Eigen::Vector3d> mappoints;
    std::vector<Sophus::SE3> cameras;
    std::vector<Observation> observations;
    createData ( n_mappoints, n_cameras, fx, fy, cx, cy, height, width, mappoints, cameras, observations );
    std::cout << "Total mpt: " << mappoints.size() << "  cameras: " << cameras.size() << "  observations: " << observations.size() << std::endl;

	
	/**** Motion only BA TST ****/
	std::cout << "\n**** Start motion only BA test ****\n";
	// Noise params standard deviation
	double mpt_noise = 0.01; // 2 cm 
	double cam_trans_noise = 0.1; // 10 cm
	double cam_rot_noise = 0.1; // 0.1 rad.
	double ob_noise = 1.0; // 1.0 pixel
	
 	// add gaussian noise.
 	std::vector<Eigen::Vector3d> noise_mappoints;
	noise_mappoints = mappoints;
 	std::vector<Sophus::SE3> noise_cameras;
	noise_cameras = cameras;
 	std::vector<Observation> noise_observations;
	noise_observations = observations;
 	addNoise(noise_mappoints, noise_cameras, noise_observations, mpt_noise, cam_trans_noise, cam_rot_noise, ob_noise );

  	BundleAdjustment ba_mba;
	ba_mba.setConvergenceCondition(20, 1e-5, 1e-10);
	ba_mba.setVerbose(true);
	
	// add mappoints
	for(size_t i = 0; i < noise_mappoints.size(); i ++)
	{
		const Eigen::Vector3d& npt = noise_mappoints.at(i);
		MapPoint* mpt = new MapPoint(npt, i);
		mpt->setFixed(); // fixed all mappoints.
		ba_mba.addMapPoint(mpt);
	} // add mappoints
 	
	// add cameras.
	for(size_t i = 0; i < noise_cameras.size(); i ++)
	{
		const Sophus::SE3& ncam = noise_cameras.at(i);
		Camera* cam = new Camera(ncam, i);
		ba_mba.addCamera(cam);
	} // add cameras.
	
	// add observations
	for(size_t i = 0; i < noise_observations.size(); i ++)
	{
		const Observation& ob = noise_observations.at(i);
		MapPoint* mpt = ba_mba.getMapPoint(ob.mpt_id_);
		Camera* cam = ba_mba.getCamera(ob.cam_id_);
		CostFunction* cost_func = new CostFunction(mpt, cam, fx, fy, cx, cy, ob.ob_);
		ba_mba.addCostFunction(cost_func);
	} // add observations.
	
	// Optimize
	ba_mba.optimize();
	
	// Compute pose Error
	double sum_rot_error = 0.0;
	double sum_trans_error = 0.0;
	for(size_t i = 0; i < cameras.size(); i ++)
	{
		Camera* cam = ba_mba.getCamera(i);
		const Sophus::SE3& opt_pose = cam->getPose();
		const Sophus::SE3& org_pose = cameras.at(i);
		Sophus::SE3 pose_err = opt_pose * org_pose.inverse();
		sum_rot_error += pose_err.so3().log().norm();
		sum_trans_error += pose_err.translation().norm();
	}
	std::cout << "Mean rot error: " << sum_rot_error / (double)(cameras.size())
	<< "\tMean trans error: " <<  sum_trans_error / (double)(cameras.size()) << std::endl;
	
	/**** Struct only BA TST ****/
	std::cout << "\n**** Start struct only BA test ****\n";
	// Noise params standard deviation
	mpt_noise = 0.1; // 10 cm.
	cam_trans_noise = 0.0;
	cam_rot_noise = 0.00; 
	ob_noise = 1.0;
	
	// add gaussian noise.
	noise_mappoints = mappoints;
	noise_cameras = cameras;
	noise_observations = observations;
	addNoise(noise_mappoints, noise_cameras, noise_observations, mpt_noise, cam_trans_noise, cam_rot_noise, ob_noise );
	
	BundleAdjustment ba_sba;
	ba_sba.setConvergenceCondition(20, 1e-5, 1e-10);
	ba_sba.setVerbose(true);
	
	// add mappoints
	for(size_t i = 0; i < noise_mappoints.size(); i ++)
	{
		const Eigen::Vector3d& npt = noise_mappoints.at(i);
		MapPoint* mpt = new MapPoint(npt, i);
		ba_sba.addMapPoint(mpt);
	} // add mappoints
	
	// add cameras.
	for(size_t i = 0; i < noise_cameras.size(); i ++)
	{
		const Sophus::SE3& ncam = noise_cameras.at(i);
		Camera* cam = new Camera(ncam, i);
		cam->setFixed(); // fixed all cameras.
		ba_sba.addCamera(cam);
	} // add cameras.
	
	// add observations
	for(size_t i = 0; i < noise_observations.size(); i ++)
	{
		const Observation& ob = noise_observations.at(i);
		MapPoint* mpt = ba_sba.getMapPoint(ob.mpt_id_);
		Camera* cam = ba_sba.getCamera(ob.cam_id_);
		CostFunction* cost_func = new CostFunction(mpt, cam, fx, fy, cx, cy, ob.ob_);
		ba_sba.addCostFunction(cost_func);
	} // add observations.
	
	// Optimize
	ba_sba.optimize();
	
	// Compute point Error
	double sum_point_error = 0.0;
	for(size_t i = 0; i < mappoints.size(); i ++)
	{
		MapPoint* mpt = ba_sba.getMapPoint(i);
		const Eigen::Vector3d& opt_mpt = mpt->getPosition();
		const Eigen::Vector3d& org_mpt = mappoints.at(i);
		sum_point_error += (opt_mpt - org_mpt).norm();
	}
	std::cout << "Mean point error: " << sum_point_error / (double)(mappoints.size())<< std::endl;
	
	
	/**** Full BA TST ****/
	std::cout << "\n**** Start full BA test ****\n";
	// Noise params standard deviation
	mpt_noise = 0.05;
	cam_trans_noise = 0.1;
	cam_rot_noise = 0.1; 
	ob_noise = 1.0;
	
	// add gaussian noise.
	noise_mappoints = mappoints;
	noise_cameras = cameras;
	noise_observations = observations;
	addNoise(noise_mappoints, noise_cameras, noise_observations, mpt_noise, cam_trans_noise, cam_rot_noise, ob_noise );
	
	BundleAdjustment full_ba;
	full_ba.setConvergenceCondition(20, 1e-5, 1e-10);
	full_ba.setVerbose(true);
	
	// add mappoints
	for(size_t i = 0; i < noise_mappoints.size(); i ++)
	{
		const Eigen::Vector3d& npt = noise_mappoints.at(i);
		MapPoint* mpt = new MapPoint(npt, i);
		full_ba.addMapPoint(mpt);
	} // add mappoints
	
	// add cameras.
	for(size_t i = 0; i < noise_cameras.size(); i ++)
	{
		const Sophus::SE3& ncam = noise_cameras.at(i);
		Camera* cam = new Camera(ncam, i, i==0); // fixed the first camera.
		full_ba.addCamera(cam);
	} // add cameras.
	
	// add observations
	for(size_t i = 0; i < noise_observations.size(); i ++)
	{
		const Observation& ob = noise_observations.at(i);
		MapPoint* mpt = full_ba.getMapPoint(ob.mpt_id_);
		Camera* cam = full_ba.getCamera(ob.cam_id_);
		CostFunction* cost_func = new CostFunction(mpt, cam, fx, fy, cx, cy, ob.ob_);
		full_ba.addCostFunction(cost_func);
	} // add observations.
	
	// Optimize
	full_ba.optimize();
	
	// Compute pose Error
	sum_rot_error = 0.0;
	sum_trans_error = 0.0;
	for(size_t i = 0; i < cameras.size(); i ++)
	{
		Camera* cam = full_ba.getCamera(i);
		const Sophus::SE3& opt_pose = cam->getPose();
		const Sophus::SE3& org_pose = cameras.at(i);
		Sophus::SE3 pose_err = opt_pose * org_pose.inverse();
		sum_rot_error += pose_err.so3().log().norm();
		sum_trans_error += pose_err.translation().norm();
	}
	std::cout << "Mean rot error: " << sum_rot_error / (double)(cameras.size())
	<< "\tMean trans error: " <<  sum_trans_error / (double)(cameras.size()) << std::endl;
	
	// Compute point Error
	sum_point_error = 0.0;
	for(size_t i = 0; i < mappoints.size(); i ++)
	{
		MapPoint* mpt = full_ba.getMapPoint(i);
		const Eigen::Vector3d& opt_mpt = mpt->getPosition();
		const Eigen::Vector3d& org_mpt = mappoints.at(i);
		sum_point_error += (opt_mpt - org_mpt).norm();
	}
	std::cout << "Mean point error: " << sum_point_error / (double)(mappoints.size())<< std::endl;
	

    return 0;
}


void createData ( int n_mappoints, int n_cameras, double fx, double fy, double cx, double cy,
                  double height, double width,
                  std::vector<Eigen::Vector3d>& mappoints, std::vector<Sophus::SE3>& cameras,
                  std::vector<Observation>& observations )
{
    // camera parameters.
    const double angle_range = 0.1; // ±0.2rad
    const double x_range = 1.0; // ±1.5m
    const double y_range = 1.0;
    const double z_range = 0.5;

    // mappoint parameters.
    const double x_min = -5.0;
    const double x_max = 5.0;
    const double y_min = -5.0;
    const double y_max = 5.0;
    const double z_min = 0.6;
    const double z_max = 8.0;

    //mappoints.reserve ( n_mappoints );
    //cameras.reserve ( n_cameras );

    // random seed.
    cv::RNG rng ( cv::getTickCount() );

    // Create cameras.
    Eigen::Matrix3d Rx, Ry, Rz;
    Eigen::Matrix3d R; 
    Eigen::Vector3d t;
    for ( int i = 0; i < n_cameras; i ++ ) {
        // Rotation.
        double tz = rng.uniform ( -angle_range, angle_range );
        double ty = rng.uniform ( -angle_range, angle_range );
        double tx = rng.uniform ( -angle_range, angle_range );

        Rz << cos ( tz ), -sin ( tz ), 0.0,
           sin ( tz ), cos ( tz ), 0.0,
           0.0, 0.0, 1.0;
        Ry << cos ( ty ), 0.0, sin ( ty ),
           0.0, 1.0, 0.0,
           -sin ( ty ), 0.0, cos ( ty );
        Rx << 1.0, 0.0, 0.0,
           0.0, cos ( tx ), -sin ( tx ),
           0.0, sin ( tx ), cos ( tx );
        R = Rz * Ry * Rx;

        // translation.
        double x = rng.uniform ( -x_range, x_range );
        double y = rng.uniform ( -y_range, y_range );
        double z = rng.uniform ( -z_range, z_range );
        t << x, y, z;

        // SE3
        Sophus::SE3 cam ( R, t );
        cameras.push_back ( cam );
    } // Create camera.


    // Create mappoints.
    std::vector<Eigen::Vector3d> tmp_mappoints;
    // tmp_mappoints.reserve ( n_mappoints );
    for ( int i = 0; i < n_mappoints; i ++ ) {
        double x = rng.uniform ( x_min, x_max );
        double y = rng.uniform ( y_min, y_max );
        double z = rng.uniform ( z_min, z_max );
        tmp_mappoints.push_back ( Eigen::Vector3d ( x,y,z ) );
    } // Create mappoints

    // Select good mappoints.
    for ( int i = 0; i < n_mappoints; i ++ ) {
        const Eigen::Vector3d& ptw = tmp_mappoints.at ( i );
        int n_obs = 0.0;
        for ( int nc = 0; nc < n_cameras; nc ++ ) {
            const Sophus::SE3& cam_pose = cameras.at ( nc );
            // project ptw to image.
            const Eigen::Vector3d ptc = cam_pose * ptw;
            Eigen::Vector2d uv (
                fx*ptc[0]/ptc[2] + cx,
                fy*ptc[1]/ptc[2] + cy
            );

            if ( uv[0]<0 || uv[1]<0 || uv[0]>=width || uv[1]>=height || ptc[2] < 0.1 ) {
                continue;
            }
            n_obs ++;
        }// for all camras.

        if ( n_obs < 2 ) { // a point must be observted by at least two camera.
            continue;
        }

        mappoints.push_back ( ptw );
    } // for all mappoints.


    // Create observations.
    for ( size_t i = 0; i < mappoints.size(); i ++ ) {
        const Eigen::Vector3d& ptw = mappoints.at ( i );
        for ( int nc = 0; nc < n_cameras; nc ++ ) {
            const Sophus::SE3& cam_pose = cameras.at ( nc );

            // project ptw to image.
            const Eigen::Vector3d ptc = cam_pose * ptw;
            Eigen::Vector2d uv (
                fx*ptc[0]/ptc[2] + cx,
                fy*ptc[1]/ptc[2] + cy
            );

            Observation ob ( i, nc, uv );
            observations.push_back ( ob );
        } // for all cameras.
    } // for all mappoints
    
    mappoints.shrink_to_fit();
	cameras.shrink_to_fit();
	observations.shrink_to_fit();
}; // createData

void addNoise ( std::vector< Eigen::Vector3d >& mappoints, std::vector< Sophus::SE3 >& cameras, std::vector< Observation >& observations, double mpt_noise, double cam_trans_noise, double cam_rot_noise, double ob_noise )
{
    cv::RNG rng ( cv::getTickCount() );

    // for all mappoints
    for ( size_t i = 0; i < mappoints.size(); i ++ ) {
        double nx = rng.gaussian ( mpt_noise );
        double ny = rng.gaussian ( mpt_noise );
        double nz = rng.gaussian ( mpt_noise );
        mappoints.at ( i ) += Eigen::Vector3d ( nx, ny, nz );
    }// for all mappoints

    // for all cameras
	Eigen::Matrix3d Rx, Ry, Rz;
	Eigen::Matrix3d R; 
	Eigen::Vector3d t;
	for(size_t i = 0; i < cameras.size(); i ++)
	{
		// skip the first camera.
		if(i == 0)
			continue;
		
		double tz = rng.gaussian ( cam_rot_noise );
		double ty = rng.gaussian ( cam_rot_noise );
		double tx = rng.gaussian ( cam_rot_noise );
		
		Rz << cos ( tz ), -sin ( tz ), 0.0,
		sin ( tz ), cos ( tz ), 0.0,
		0.0, 0.0, 1.0;
		Ry << cos ( ty ), 0.0, sin ( ty ),
		0.0, 1.0, 0.0,
		-sin ( ty ), 0.0, cos ( ty );
		Rx << 1.0, 0.0, 0.0,
		0.0, cos ( tx ), -sin ( tx ),
		0.0, sin ( tx ), cos ( tx );
		R = Rz * Ry * Rx;
		
		// translation.
		double x = rng.gaussian ( cam_trans_noise );
		double y = rng.gaussian ( cam_trans_noise );
		double z = rng.gaussian ( cam_trans_noise );
		t << x, y, z;
		
		// SE3
		Sophus::SE3 cam_noise ( R, t );
		cameras.at(i) *= cam_noise;
	} // for all cameras.

	// for all observations
	for(size_t i = 0; i < observations.size(); i ++)
	{
		double x = rng.gaussian ( ob_noise );
		double y = rng.gaussian ( ob_noise );
		observations.at(i).ob_ += Eigen::Vector2d(x,y);
	} // for all observations
} // addNoise


