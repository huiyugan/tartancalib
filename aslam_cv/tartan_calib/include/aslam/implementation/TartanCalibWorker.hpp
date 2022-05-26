constexpr const auto PI = boost::math::constants::pi<double>();

namespace aslam
{
    namespace cameras
    {

        template< typename C>
        void TartanCalibWorker<C>::compute_remap(const Eigen::Matrix<float, 3, Eigen::Dynamic>& xyz, const Eigen::MatrixXd& resolution) 
        {
            xyz_ = xyz;
            empty_pixels_ = false;
            resolution_out_ = cv::Size(resolution.coeff(1,0),resolution.coeff(0,0));
            map_x_float_ = cv::Mat(resolution_out_, CV_32FC1);
            map_y_float_ = cv::Mat(resolution_out_, CV_32FC1);


            // // Compute the remap maps
            for (size_t v = 0; v < resolution.coeff(0,0); ++v) {
                for (size_t u = 0; u < resolution.coeff(1,0); ++u) {
                xyz_idx_ = u+v*resolution.coeff(1,0);
                // SM_INFO_STREAM(std::to_string(xyz_idx_)+" \n");
                xyz_ray_ = xyz_.col(xyz_idx_).cast<double>();
                camera_->vsEuclideanToKeypoint(xyz_ray_,distorted_pixel_location_);
                //insert point into map
                map_x_float_.at<float>(v, u) = static_cast<float>(distorted_pixel_location_(0));
                map_y_float_.at<float>(v, u) = static_cast<float>(distorted_pixel_location_(1));
                }
            }
            cv::convertMaps(map_x_float_, map_y_float_, map_x_, map_y_, CV_16SC2);
            // SM_INFO_STREAM("map:" <<map_x_float_ <<" \n");
            maps_x_.push_back(map_x_);
            maps_y_.push_back(map_y_);

        }

        template< typename C>
        void TartanCalibWorker<C>::compute_remaps(void)
        {
            for (size_t i =0; i < num_views_; i++)
            {
                compute_remap(xyzs_[i],resolutions_.col(i));
                cv::Mat undistorted_image;
                cv::remap(obslist_[0].image(),undistorted_image,map_x_,map_y_,cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
                cv::imshow("test",obslist_[0].image());
                cv::waitKey(1000);
                cv::imshow("test",undistorted_image);
                cv::waitKey(1000);

            }

        }

// template<typename C>
// Eigen::VectorXd e2k(const C * camera, Eigen::Vector3d const & p) {
//   Eigen::VectorXd k;
//   camera->vsEuclideanToKeypoint(p, k);
//   return k;
        


            //     Eigen::VectorXd k;
            //     Eigen::Vector3d p = xyzs_[0].col(0);
            //     camera_->vsEuclideanToKeypoint(p, k);

            // Eigen::Vector2d distorted_pixel_location;
            // distortPixel(
            //     used_camera_parameters_pair_.getInputPtr()->K(),
            //     used_camera_parameters_pair_.getInputPtr()->R(),
            //     used_camera_parameters_pair_.getOutputPtr()->P(),
            //     used_camera_parameters_pair_.getInputPtr()->distortionModel(), D,
            //     pixel_location, &distorted_pixel_location);

            // // Insert in map
            // map_x_float.at<float>(v, u) =
            //     static_cast<float>(distorted_pixel_location.x());
            // map_y_float.at<float>(v, u) =
            //     static_cast<float>(distorted_pixel_location.y());

            // if ((distorted_pixel_location.x() < 0) ||
            //     (distorted_pixel_location.y() < 0) ||
            //     (distorted_pixel_location.x() >= resolution_in.width) ||
            //     (distorted_pixel_location.y() >= resolution_in.height)) {
            //     empty_pixels_ = true;
            // }
        

        
        template< typename C>
        void TartanCalibWorker<C>::compute_rotation(const Eigen::MatrixXd& pose )
        {
            rot_mat_ = Eigen::Matrix<float,3,3>::Zero();
            rot_mat_x_ = Eigen::Matrix<float,3,3>::Zero();
            rot_mat_z_ = Eigen::Matrix<float,3,3>::Zero();
            
            rot_x_ = pose.coeff(0,0)/ 180 * PI;
            rot_z_ = pose.coeff(1,0)/ 180 * PI;

            rot_mat_x_(0,0) = 1;
            rot_mat_x_(1,1) = cos(rot_x_);
            rot_mat_x_(1,2) = -sin(rot_x_);
            rot_mat_x_(2,1) = sin(rot_x_);
            rot_mat_x_(2,2) = cos(rot_x_);

            rot_mat_z_(0,0) = cos(rot_z_);
            rot_mat_z_(0,1) = -sin(rot_z_);
            rot_mat_z_(1,0) = sin(rot_z_);
            rot_mat_z_(1,1) = cos(rot_z_);
            rot_mat_z_(2,2) = 1;

            rot_mat_ = rot_mat_z_*rot_mat_x_;
        }
        
        template< typename C>
        void TartanCalibWorker<C>::compute_xyz(const Eigen::MatrixXd& fov, const Eigen::MatrixXd& resolution,const Eigen::MatrixXd& pose)
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
            xyz_=rot_mat_*xyz_;

            xyzs_.push_back(xyz_);
            
        }

        template< typename C>
        void TartanCalibWorker<C>::compute_xyzs(void)
        {

                xyzs_.empty();
                for (int i = 0; i< num_views_; i++)
                {
                    compute_xyz(fovs_.col(i),resolutions_.col(i),poses_.col(i));
                }
                
                if (verbose_)
                {
                    cv::viz::Viz3d myWindow("Coordinate Frame");
                    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
                    std::vector<cv::Point3f> pts3d;
                    
                    for (int j = 0; j<num_views_; j++)
                    {
                        pts3d.empty();
                        for (int i = 0; i<xyzs_[j].cols() ; i ++)
                        {
                            pts3d.push_back(cv::Point3f(xyzs_[j].coeff(0,i),xyzs_[j].coeff(1,i),xyzs_[j].coeff(2,i)));
                        }

                        cv::viz::WCloud cloud_widget1(pts3d,cv::viz::Color::green());
                        myWindow.showWidget("cloud 1", cloud_widget1);
                    }
                    myWindow.spin();

                }

        }
    }
}