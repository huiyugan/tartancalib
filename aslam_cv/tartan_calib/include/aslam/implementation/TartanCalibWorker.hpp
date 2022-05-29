constexpr const auto PI = boost::math::constants::pi<double>();

namespace aslam
{
    namespace cameras
    {
        /// \brief This is a more or less hardcoded function to show 4 pinhole projections into one view, including their detected corners
        template< typename C>
        void TartanCalibWorker<C>::show_pinholes(std::vector<aslam::cameras::GridCalibrationTargetObservation> obs, bool imsave)
        {
            int img_size = resolutions_(0,0); //hard assumption: all pictures are square and of the same size
            cv::Mat img_total = cv::Mat::zeros(3*img_size,3*img_size,CV_32F); //3X because we're making a cross of pinhole projections
            cv::Point2f src_center(img_size/2.0, img_size/2.0);
            cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, -90, 1.0);  

            cv::Mat dst ;
            cv::warpAffine(obs[3].image(), dst, rot_mat, cv::Size(img_size,img_size));
            dst.copyTo(img_total(cv::Rect(2*img_size,img_size,img_size,img_size)));


            // img_total(cv::Range(0,img_size),cv::Range(img_size,2*img_size)) = obs[2].image();
            obs[0].image().copyTo(img_total(cv::Rect(img_size,0,img_size,img_size)));
            
            rot_mat = cv::getRotationMatrix2D(src_center, 90, 1.0); 
            cv::warpAffine(obs[2].image(), dst, rot_mat, cv::Size(img_size,img_size));
            dst.copyTo(img_total(cv::Rect(0,img_size,img_size,img_size)));

            rot_mat = cv::getRotationMatrix2D(src_center, 180, 1.0); 

            cv::warpAffine(obs[1].image(), dst, rot_mat, cv::Size(img_size,img_size));
            dst.copyTo(img_total(cv::Rect(img_size,2*img_size,img_size,img_size)));
            
            obs[4].image().copyTo(img_total(cv::Rect(img_size,img_size,img_size,img_size))); // img_center
            // cv::imshow("is this not black?",obs[2].image());
            // cv::waitKey(2000);

            // cv::Mat img_gray = obs[0].image();
            // cv::Mat img_color;
            // cv::cvtColor(img_gray,img_color,cv::COLOR_GRAY2BGR);
            // int num_views = obs.size();

            // for (int i = 0; i<num_views;i++)
            // {
            //     // plot the original detections as red circles
            //     std::vector<cv::Point2f> outCornerList;
            //     obs_ = obs[i];
            //     obs_.getCornersImageFrame(outCornerList);
            //     SM_INFO_STREAM("sizee " << std::to_string(obs.size()));
            //     for (auto corner_old:outCornerList)
            //     {
            //         cv::circle(img_color, corner_old,5, cv::Scalar(0,255,0),2);
            //     }
            // }
            if (imsave)
            {
                
                aslam::Time stamp = obs[0].time();
                cv::imwrite("frame_"+std::to_string(stamp.toSec())+".png",img_total);
            }
            else{
                cv::imshow("Showing some obs",img_total); //assumption is that all obs come from the same image and in the same frame!!
                cv::waitKey(2000);
            }

        }

        template< typename C>
        void TartanCalibWorker<C>::show_obs(aslam::cameras::GridCalibrationTargetObservation obs, bool imsave)
        {
            cv::Mat img_gray = obs.image();
            cv::Mat img_color;
            cv::cvtColor(img_gray,img_color,cv::COLOR_GRAY2BGR);

            // plot the original detections as red circles
            std::vector<cv::Point2f> outCornerList;
            obs_ = obs;
            obs_.getCornersImageFrame(outCornerList);
            for (auto corner_old:outCornerList)
            {
                cv::circle(img_color, corner_old,5, cv::Scalar(0,255,0),2);
            }

            if (imsave)
            {
                aslam::Time stamp = obs_.time();
                cv::imwrite("frame_"+std::to_string(stamp.toSec())+".png",img_color);
            }
            else{
                cv::imshow("Showing some obs",img_color); //assumption is that all obs come from the same image and in the same frame!!
                cv::waitKey(2000);
            }

        }

        /// \brief this function will go over all the observations and project it back onto the original image frame
        /// the information remains within the information class

        template< typename C>
        void TartanCalibWorker<C>::merge_obs(void)
        {
            // output_obslist_.empty();
            // // we're looping over all frames
            // for (int i=0; i< num_frames_; i++)
            // {
            //     obs_ = obslist_[i];
            //     for (int j=0; j<num_views_;j++)
            //     {
            //         new_obslist_[i][j].getCornersIdx(outCornerIdx_);

            //         std::vector<cv::Point2f> outCornerList;
            //         new_obslist_[i][j].getCornersImageFrame(outCornerList); 

            //         for (int k=0; k< outCornerList.size(); k++)
            //         {
            //             Eigen::Vector2d b;
            //             b(0) = outCornerList[k].x;
            //             b(1) =  outCornerList[k].y;
            //             obs_.updateImagePoint(outCornerIdx_[k],b);
            //         }
            //     }
            //     output_obslist_.push_back(obs_); 
            //     // show_obs(output_obslist_[i],true);            
            // }

            


        }

        template< typename C>
        void TartanCalibWorker<C>::project_to_original_image(void)
        {
            for (int i=0; i<num_frames_; i++ )
            {
                for (auto &reprojection: reprojection_wrappers_ )
                {
                    obs_ = reprojection.obslist_[i];
                    obs_.getCornersIdx(outCornerIdx_);

                    cv::Point2f img_center(reprojection.resolution_.coeff(1,0)/2.,reprojection.resolution_.coeff(0,0)/2.);
                    std::vector<cv::Point2f> outCornerList;
                    obs_.getCornersImageFrame(outCornerList);
                    
                    num_corners_ = outCornerList.size();
                    xyz_.resize(3,num_corners_);
                    cv::Point2f corner;
                    
                    for (int k =0; k<num_corners_; k++)
                    {
                        corner = outCornerList[k];
                        xyz_(0,k) = corner.x;
                        xyz_(1,k) = corner.y;
                    }

                    xyz_.row(0) -= img_center.x*Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_corners_);
                    xyz_.row(1) -= img_center.y*Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_corners_);

                    xyz_.row(0)*=((tan(reprojection.fov_.coeff(0,0) / 2.0 / 180 * PI))/img_center.x);
                    xyz_.row(1)*=((tan(reprojection.fov_.coeff(1,0) / 2.0 / 180 * PI))/img_center.y);
                    xyz_.row(2)=Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_corners_);
                    
                    compute_rotation(reprojection.pose_);
                    xyz_ = rot_mat_*xyz_;
                    
                    for (size_t k=0; k<num_corners_; k++)
                    {
                        xyz_ray_ = xyz_.col(k);
                        camera_->vsEuclideanToKeypoint(xyz_ray_.cast<double>(),distorted_pixel_location_);
                        corner.x = distorted_pixel_location_(0);
                        corner.y = distorted_pixel_location_(1);

                        Eigen::Vector2d b;
                        b(0) = corner.x;
                        b(1) = corner.y;

                        new_obslist_[i].updateImagePoint(outCornerIdx_[k],b);
                    }
                }
                show_obs(new_obslist_[i],true);

            }


        }

        template< typename C>
        void TartanCalibWorker<C>::compute_corners(void)
        {
            for (int i= 0; i<num_frames_; i++)
            {
                std::vector<GridCalibrationTargetObservation> frame_obs;
                
                for (auto &reprojection: reprojection_wrappers_ )
                {
                    obs_.setImage(reprojection.obslist_[i].image());
                    gd_.findTargetNoTransformation(reprojection.obslist_[i].image(),obs_);
                    obs_.setTime(obslist_[i].time());
                    reprojection.obslist_[i] = obs_;
                    frame_obs.push_back(obs_);
                }
                show_pinholes(frame_obs,true);
            }
        }

        template< typename C>
        void TartanCalibWorker<C>::compute_reprojections(void)
        { 
            for (int i=0; i<num_frames_; i++)
            {
                std::vector<cv::Mat> frame_reprojections;
                for (auto &reprojection: reprojection_wrappers_ )
                {
                    // SM_INFO_STREAM("New img \n");
                    cv::Mat undistorted_image_;
                    cv::remap(reprojection.obslist_[i].image(),undistorted_image_,reprojection.map_x_,reprojection.map_y_,cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT);
                    reprojection.obslist_[i].setImage(undistorted_image_);

                    ///TODO:Remove
                    frame_reprojections.push_back(undistorted_image_);
                }
                ///TODO:Remove
                reprojections_.push_back(frame_reprojections);
            }  
        }


        template< typename C>
        void TartanCalibWorker<C>::compute_remap(aslam::cameras::ReprojectionWrapper<C>& reprojection) 
        {
            empty_pixels_ = false;
            resolution_out_ = cv::Size(reprojection.resolution_.coeff(1,0),reprojection.resolution_.coeff(0,0));
            map_x_float_ = cv::Mat(resolution_out_, CV_32FC1);
            map_y_float_ = cv::Mat(resolution_out_, CV_32FC1);

            // // Compute the remap maps
            for (size_t v = 0; v < reprojection.resolution_.coeff(0,0); ++v) {
                for (size_t u = 0; u < reprojection.resolution_.coeff(1,0); ++u) {
                xyz_idx_ = u+v*reprojection.resolution_.coeff(1,0);
                xyz_ray_ = reprojection.xyz_.col(xyz_idx_);
                
                camera_->vsEuclideanToKeypoint(xyz_ray_.cast<double>(),distorted_pixel_location_);

                //insert point into map
                map_x_float_.at<float>(v, u) = static_cast<float>(distorted_pixel_location_(0));
                map_y_float_.at<float>(v, u) = static_cast<float>(distorted_pixel_location_(1));
                }
            }
            cv::Mat map_x_, map_y_;
            cv::convertMaps(map_x_float_, map_y_float_, map_x_, map_y_, CV_16SC2);
            
            reprojection.map_x_ = map_x_;
            reprojection.map_y_ = map_y_;

            ///TODO: remove
            maps_x_.push_back(map_x_);
            maps_y_.push_back(map_y_);
        }

        template< typename C>
        void TartanCalibWorker<C>::compute_remaps(void)
        {
            for (auto &reprojection: reprojection_wrappers_ )
            {
                compute_remap(reprojection);
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
        void TartanCalibWorker<C>::compute_xyz( aslam::cameras::ReprojectionWrapper<C>& reprojection)
        {
            num_points_ = reprojection.resolution_.coeff(0,0)*reprojection.resolution_.coeff(1,0);
            xyz_.resize(3,num_points_);
            
            xyz_.row(0) = Eigen::Matrix<float, 1,Eigen::Dynamic>::LinSpaced(reprojection.resolution_.coeff(1,0),-1,1).matrix().replicate(1,reprojection.resolution_.coeff(0,0));
            
            yy_ = Eigen::Matrix<float, 1,Eigen::Dynamic>::LinSpaced(reprojection.resolution_.coeff(0,0),-1,1).matrix().replicate(reprojection.resolution_.coeff(1,0),1);
            yy_.resize(1,num_points_);
            xyz_.row(1) = yy_;

            xyz_.row(0)*= tan(reprojection.fov_.coeff(0,0) / 2.0 / 180 * PI);
            xyz_.row(1)*= tan(reprojection.fov_.coeff(1,0) / 2.0 / 180 * PI);
            xyz_.row(2) = Eigen::Matrix<float, 1,Eigen::Dynamic>::Ones(1,num_points_);
            compute_rotation(reprojection.pose_);
            xyz_=rot_mat_*xyz_;
            reprojection.xyz_ = xyz_;

            ///TODO:remove 
            xyzs_.push_back(xyz_);
            
        }

        template< typename C>
        void TartanCalibWorker<C>::compute_xyzs(void)
        {


                for (auto &reprojection: reprojection_wrappers_ )
                {
                    compute_xyz(reprojection);
                }

                if (debug_mode_==DebugMode::pointcloudprojection)
                {
                    cv::viz::Viz3d myWindow("Coordinate Frame");
                    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
                    std::vector<cv::Point3f> pts3d;
                    
                    for (auto reprojection: reprojection_wrappers_ )
                    {
                        pts3d.empty();
                        if (reprojection.xyz_.cols() > 0)
                        {
                            for (int i = 0; i<reprojection.xyz_.cols() ; i ++)
                            {
                                pts3d.push_back(cv::Point3f(reprojection.xyz_.coeff(0,i),reprojection.xyz_.coeff(1,i),reprojection.xyz_.coeff(2,i)));
                            }
                            cv::viz::WCloud cloud_widget1(pts3d,cv::viz::Color::green());
                            myWindow.showWidget("cloud 1", cloud_widget1);
                        }



                    }
                    myWindow.spin();

                }

        }
    }
}