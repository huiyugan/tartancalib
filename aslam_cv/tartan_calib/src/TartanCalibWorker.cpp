#include<aslam/TartanCalibWorker.hpp>
constexpr const auto PI = boost::math::constants::pi<double>();

namespace aslam
{
    namespace cameras
    {
        void TartanCalibWorker::compute_rotation(const Eigen::MatrixXd& pose )
        {
            rot_mat_ = Eigen::Matrix<float,3,3>::Zero();
            rot_mat_(0,0) = 1;
            rot_mat_(1,1) = 1;
            rot_mat_(2,2) = 1;

        }

        void TartanCalibWorker::compute_xyz(const Eigen::MatrixXd& fov, const Eigen::MatrixXd& resolution,const Eigen::MatrixXd& pose)
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
            compute_rotation(pose);
            xyz_=rot_mat_*xyz_.eval();

            xyzs_.push_back(xyz_);
            // SM_INFO_STREAM("xyz_shape : \n" << xyz_.cols() << "  " << xyz_.rows() << std::endl);
        }
        void TartanCalibWorker::compute_xyzs(void)
        {

                xyzs_.empty();
                for (int i = 0; i< num_views_; i++)
                {
                    compute_xyz(fovs_.col(i),resolutions_.col(i),poses_.col(i));
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