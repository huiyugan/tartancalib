constexpr const auto PI = boost::math::constants::pi<double>();

namespace aslam
{
    namespace cameras
    {
        template< typename C>
        void TartanCalibWorker<C>::debug_show(void)
        {   
            for (auto debug_mode_:debug_modes_)
            {
                switch(debug_mode_)
                {
                    case DebugMode::originalprojection:
                    {
                        SM_INFO_STREAM("Saving the original image with all detected corners. Red corners are the originally detected corners, green are new corners.");
                        
                        for (int i =0; i<num_frames_; i++)
                        {
                            std::vector<cv::Scalar> colors;
                            std::vector<aslam::cameras::GridCalibrationTargetObservation> obs;
                            
                            // these are the newly found corners projected back into the original coordinate frame 
                            obs.push_back(new_obslist_[i]);
                            colors.push_back(cv::Scalar(0,255,0));
        
                            // add the original image
                            obs.push_back(obslist_[i]);
                            colors.push_back(cv::Scalar(0,0,255));
                            aslam::Time stamp = obslist_[i].time();

                            cv::Mat color_img = get_mat(obs,true,0.,colors);
                            cv::imwrite("frame_"+std::to_string(stamp.toSec())+"_original.png",color_img);
                        }
                        
                        break;
                    }
                    case DebugMode::pinholeprojection:
                    {
                        SM_INFO_STREAM("This mode is pretty hardcoded and shows the cube pinhole configuration in a cross, including all detected corners.");
                        
                        int img_size = resolutions_(0,0); //hard assumption: all pictures are square and of the same size
                        
                        std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist;
                        std::vector<cv::Scalar> colors;
                        colors.push_back(cv::Scalar(0,255,0));

                        for (int i=0; i<num_frames_; i++)
                        {
                            cv::Mat img_total(3*img_size,3*img_size,CV_8UC3,CV_RGB(0,0,0)); //3X because we're making a cross of pinhole projections
                            
                            obslist.clear();
                            cv::Mat temp_img;
                            obslist.push_back(reprojection_wrappers_[3].obslist_[i]);
                            temp_img = get_mat(obslist,true,-90.,colors);
                            temp_img.copyTo(img_total(cv::Rect(2*img_size,img_size,img_size,img_size)));

                            obslist.clear();
                                
                            obslist.push_back(reprojection_wrappers_[0].obslist_[i]);
                            temp_img = get_mat(obslist,true,0.,colors);
                            temp_img.copyTo(img_total(cv::Rect(img_size,0,img_size,img_size)));
                        
                            obslist.clear();
                            obslist.push_back(reprojection_wrappers_[2].obslist_[i]);
                            temp_img = get_mat(obslist,true,90.,colors);
                            temp_img.copyTo(img_total(cv::Rect(0,img_size,img_size,img_size)));

                            obslist.clear();
                            obslist.push_back(reprojection_wrappers_[1].obslist_[i]);
                            temp_img = get_mat(obslist,true,180.,colors);
                            temp_img.copyTo(img_total(cv::Rect(img_size,2*img_size,img_size,img_size)));
                            
                            obslist.clear();
                            obslist.push_back(reprojection_wrappers_[4].obslist_[i]);
                            temp_img = get_mat(obslist,true,0.,colors);
                            temp_img.copyTo(img_total(cv::Rect(img_size,img_size,img_size,img_size)));
                            
                            aslam::Time stamp = obslist_[i].time();
                            cv::imwrite("frame_"+std::to_string(stamp.toSec())+"_pinhole.png",img_total);
                        }

                        break;
                    }
                    default:
                        SM_INFO_STREAM("Default debug mode means no plotting.");

                }
            }


        }

        template< typename C>
        cv::Mat TartanCalibWorker<C>::get_mat(std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist, bool show_corners, float rotation_angle,std::vector<cv::Scalar> colors)
        {
            cv::Mat img_gray = obslist[0].image(); //asumption: all obs have the same image, just different corners
            cv::Mat img_color;
            cv::cvtColor(img_gray,img_color,cv::COLOR_GRAY2BGR); //convert to color to be able to plot colored dots

            std::vector<cv::Point2f> outCornerList;
            
            // getting corners as circles
            if (show_corners)
            {
                for (int i=0; i<obslist.size();i++)
                {
                    obslist[i].getCornersImageFrame(outCornerList);   
                        for (auto corner : outCornerList)
                        {
                            cv::circle(img_color, corner,5, colors[i],2);
                        }         
                }
            }
            
            // performing any desired rotation
            cv::Point2f src_center(img_gray.cols/2.0, img_gray.rows/2.0);
            cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, rotation_angle, 1.0);  

            cv::Mat dst ;
            cv::warpAffine(img_color, dst, rot_mat, cv::Size(img_gray.cols,img_gray.rows));

            return dst;
        }



        template< typename C>
        void TartanCalibWorker<C>::project_to_original_image(void)
        {
            for (int i=0; i<num_frames_; i++ )
            {
                for (auto reprojection: reprojection_wrappers_ )
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
            }


        }

        template< typename C>
        void TartanCalibWorker<C>::compute_corners(void)
        {
            for (int i= 0; i<num_frames_; i++)
            {
                
                for (auto &reprojection: reprojection_wrappers_ )
                {
                    obs_.setImage(reprojection.obslist_[i].image());
                    gd_.findTargetNoTransformation(reprojection.obslist_[i].image(),obs_);
                    obs_.setTime(obslist_[i].time());
                    reprojection.obslist_[i] = obs_;
                }
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

                for (auto debug_mode_ : debug_modes_)
                {
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
}