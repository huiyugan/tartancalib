constexpr const auto PI = boost::math::constants::pi<double>();

namespace aslam
{
    namespace cameras
    {
        /// \brief this function will go over all the observations and project it back onto the original image frame
        /// the information remains within the information class
        template< typename C>
        void TartanCalibWorker<C>::project_to_original_image(void)
        {
            for (int i=0; i<num_frames_; i++ )
            {
                // get original frame
                cv::Mat img_gray = obslist_[i].image();
                cv::Mat img_color;
                cv::cvtColor(img_gray,img_color,cv::COLOR_GRAY2BGR);

            
                for (int j=0; j< num_views_; j++)
                {
                    obs_ = new_obslist_[i][j];

                    cv::Point2f img_center(resolutions_.col(j).coeff(1,0)/2.,resolutions_.col(j).coeff(0,0)/2.);
                    std::vector<cv::Point2f> outCornerList;
                    obs_.getCornersImageFrame(outCornerList);
                    
                    num_corners_ = outCornerList.size();
                    xyz_.resize(3,num_corners_);
                    cv::Point2f corner;

                    std::vector<float> x_s;
                    std::vector<float> y_s;
                    
                    for (int k =0; k<num_corners_; k++)
                    {
                        //scale points between -1 and 1
                        corner = outCornerList[k];
                        xyz_(0,k) = corner.x;
                        xyz_(1,k) = corner.y;
                        x_s.push_back(corner.x);
                        y_s.push_back(corner.y);

                    }

                    xyz_.row(0) -= img_center.x*Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_corners_);
                    xyz_.row(1) -= img_center.y*Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_corners_);

                    xyz_.row(0)*=((tan(fovs_.col(j).coeff(0,0) / 2.0 / 180 * PI))/img_center.x);
                    xyz_.row(1)*=((tan(fovs_.col(j).coeff(1,0) / 2.0 / 180 * PI))/img_center.y);
                    xyz_.row(2)=Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_corners_);
                    
                    compute_rotation(poses_.col(j));
                    xyz_ = rot_mat_*xyz_;
                    
                    for (int k=0; k<num_corners_; k++)
                    {
                        xyz_ray_ = xyz_.col(k).cast<double>();
                        camera_->vsEuclideanToKeypoint(xyz_ray_,distorted_pixel_location_);
                        corner.x = distorted_pixel_location_(0);
                        corner.y = distorted_pixel_location_(1);
                        cv::circle(img_color, corner,5, cv::Scalar(0,255,0),3);
                    }
   
                }

                // plot the original detections as red circles
                std::vector<cv::Point2f> outCornerList;
                obs_ = obslist_[i];
                obs_.getCornersImageFrame(outCornerList);
                for (auto corner_old:outCornerList)
                {
                    cv::circle(img_color, corner_old,5, cv::Scalar(0,0,255),3);
                }
                cv::imwrite("test_"+std::to_string(i)+".jpg",img_color);
                // cv::waitKey(10000);
                // cv::Mat img = reprojections_[i][j];
                
            }


        }

        template< typename C>
        void TartanCalibWorker<C>::compute_corners(void)
        {
            for (int i= 0; i<num_frames_; i++)
            {
                std::vector<GridCalibrationTargetObservation> frame_obs;
                
                for (int j= 0; j< num_views_; j++)
                {
                    gd_.findTargetNoTransformation(reprojections_[i][j],obs_);
                    frame_obs.push_back(obs_);
                }
                new_obslist_.push_back(frame_obs);
            }
            
        }

        template< typename C>
        void TartanCalibWorker<C>::compute_reprojections(void)
        { 
            for (int i=0; i<num_frames_; i++)
            {
                std::vector<cv::Mat> frame_reprojections;
                for (int j=0; j< num_views_; j++)
                {
                    // SM_INFO_STREAM("New img \n");
                    cv::Mat undistorted_image_;
                    cv::remap(obslist_[i].image(),undistorted_image_,maps_x_[j],maps_y_[j],cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT);
                    frame_reprojections.push_back(undistorted_image_);
                }
                reprojections_.push_back(frame_reprojections);
            }  
        }


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
            cv::Mat map_x_, map_y_;
            cv::convertMaps(map_x_float_, map_y_float_, map_x_, map_y_, CV_16SC2);
            
            maps_x_.push_back(map_x_);
            maps_y_.push_back(map_y_);

        }

        template< typename C>
        void TartanCalibWorker<C>::compute_remaps(void)
        {
            for (size_t i =0; i < num_views_; i++)
            {
                compute_remap(xyzs_[i],resolutions_.col(i));
            }

        }
      

        
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