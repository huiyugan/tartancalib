#include<aslam/TartanCalibWorker.hpp>
constexpr const auto PI = boost::math::constants::pi<double>();

namespace aslam
{
    namespace cameras
    {
        void TartanCalibWorker::compute_xyz(const Eigen::MatrixXd& fov, const Eigen::MatrixXd& resolution)
        {
            num_points_ = resolution.coeff(0,0)*resolution.coeff(1,0);
            xyz_.resize(3,num_points_);
            
            xyz_.row(0) = Eigen::Matrix<float, 1,Eigen::Dynamic>::LinSpaced(resolution.coeff(1,0),-1,1).matrix().replicate(1,resolution.coeff(0,0));
            
            yy_ = Eigen::Matrix<float, 1,Eigen::Dynamic>::LinSpaced(resolution.coeff(0,0),-1,1).matrix().replicate(resolution.coeff(1,0),1);
            yy_.resize(1,num_points_);
            xyz_.row(1) = yy_;

            xyz_.row(0)*= tan(fov.coeff(0,0) / 2.0 / 180 * PI);
            xyz_.row(1)*= tan(fov.coeff(1,0) / 2.0 / 180 * PI);
            xyz_.row(2) = Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_points_);
            xyzs_.push_back(xyz_);
            // SM_INFO_STREAM("xyz_shape : \n" << xyz_.cols() << "  " << xyz_.rows() << std::endl);
        }
        void TartanCalibWorker::compute_xyzs(void)
        {
                // mvs::PointMat1 ax = 
                //     mvs::PointMat1::LinSpaced(shape.w, 0, 1) * static_cast<Scalar_t>( fov_x / 180 * PI );
                // mvs::PointMat1 ay = 
                //     mvs::PointMat1::LinSpaced(shape.h, 0, 1) * static_cast<Scalar_t>( fov_y  / 180 * PI );

                xyzs_.empty();
                for (int i = 0; i< num_views_; i++)
                {
                    compute_xyz(fovs_.col(i),resolutions_.col(i));
                }
                
                cv::viz::Viz3d myWindow("Coordinate Frame");
                myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

                std::vector<cv::Point3f> pts3d;
                for (int i = 0; i<xyzs_[0].cols() ; i ++)
                {
                    pts3d.push_back(cv::Point3f(xyzs_[0].coeff(0,i),xyzs_[0].coeff(1,i),xyzs_[0].coeff(2,i)));
                }

                // pts3d_.push_back(cv::Point3f(1, 2, 3));
                cv::viz::WCloud cloud_widget1(pts3d, cv::viz::Color::green());
                myWindow.showWidget("cloud 1", cloud_widget1);
                myWindow.spin();
        }
    }
}