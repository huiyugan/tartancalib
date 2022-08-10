constexpr const auto PI = boost::math::constants::pi<double>();
namespace plt = matplotlibcpp;
namespace aslam
{
    namespace cameras
    {

        template<typename C>
        bool TartanCalibWorker<C>::export_dataset(std::string path)
        {
            // open file
            FILE* file = fopen(path.c_str(), "wb");
            if (!file) {
                return false;
            }

            //write header
            // File format identifier
            uint8_t header[10];
            header[0] = 'c';
            header[1] = 'a';
            header[2] = 'l';
            header[3] = 'i';
            header[4] = 'b';
            header[5] = '_';
            header[6] = 'd';
            header[7] = 'a';
            header[8] = 't';
            header[9] = 'a';
            fwrite(header, 1, 10, file);

            // File format version
            // version 1 means it's an aruco/checkboard target
            // we dump x y locations of corners in the target frame because they may not be evenly spaced.
            const u32 version = 1;
            write_one(&version, file);

            // Camers
            /// TODO: for now we just support single-cam output, because every camera gets a seperate tartan calib class
            u32 num_cameras = 1;
            write_one(&num_cameras, file);
            for (int camera_index = 0; camera_index < num_cameras; ++ camera_index) {
                u32 width = in_width_;
                write_one(&width, file);
                u32 height = in_height_;
                write_one(&height, file);
            }

            // Imagesets
            u32 num_imagesets = num_frames_;
            write_one(&num_imagesets, file);
            for (int imageset_index = 0; imageset_index < num_imagesets; ++ imageset_index) {
                const string& filename = "placeholder.png"; // we extract imgs from a bag, so not sure what else to put here
                u32 filename_len = filename.size();
                write_one(&filename_len, file);
                fwrite(filename.data(), 1, filename_len, file);
                
                std::vector<cv::Point2f> outCornerList;
                new_obslist_[imageset_index].getCornersIdx(outCornerIdx_);
                new_obslist_[imageset_index].getCornersImageFrame(outCornerList);

                for (int camera_index = 0; camera_index < num_cameras; ++ camera_index) {
                    
                    u32 num_features = outCornerList.size();
                    write_one(&num_features, file);
                    for (int feature_idx =0; feature_idx <num_features; feature_idx++ ) {
                        cv::Point2f point = outCornerList[feature_idx];
                        write_one(&point.x, file);
                        write_one(&point.y, file);
                        i32 id_32bit = outCornerIdx_[feature_idx];
                        write_one(&id_32bit, file);
                    }
                }
            }

            u32 num_known_geometries = 1; // number of calibration targets in view
            write_one(&num_known_geometries, file);


            /// /// \brief dump an april tag
            ///   point ordering: (e.g. 2x2 grid)
            ///          12-----13  14-----15
            ///          | TAG 3 |  | TAG 4 |
            ///          8-------9  10-----11
            ///          4-------5  6-------7
            ///    y     | TAG 1 |  | TAG 2 |
            ///   ^      0-------1  2-------3
            ///   |-->x
            for (int geometry_index = 0; geometry_index < num_known_geometries; ++ geometry_index) {
                // adding information on number of rows and columns
                // in theory we only need one of the two
                i32 rows = target_->rows();
                i32 cols = target_->cols();

                write_one(&rows,file);
                write_one(&cols,file);

                Eigen::MatrixXd target_points;
                target_points = target_->points().transpose(); 

                // number of features we're about to spit out              
                u32 feature_id_to_position_size = target_points.cols();
                write_one(&feature_id_to_position_size, file);

                for (int j=0; j <feature_id_to_position_size; j++) {

                    i32 id_32bit = j;
                    write_one(&id_32bit, file);

                    float target_x = static_cast<float>(target_points.coeff(0,j));
                    write_one(&target_x, file);

                    float target_y = static_cast<float>(target_points.coeff(1,j));
                    write_one(&target_y, file);
                }
            }
  
            fclose(file);            

        }

        template< typename C>
        void TartanCalibWorker<C>::match_quads(aslam::cameras::ReprojectionWrapper<C>& reprojection)
        {  
            cv::Point2f distorted_pixel_cv;
            Eigen::Vector3d outPoint;
            Eigen::VectorXd keypoint(2),target_original(4),target_transformed(4);
            Eigen::MatrixXd all_target_original,all_target_transformed;

            all_target_original = target_->points().transpose();
            all_target_original.conservativeResize(4,all_target_original.cols());
            

            // the board definition only contains the corners of tags, but there is another corner at the corners of the board
            // we are adding these corners to use them for the refine window size below
            
            double min_delta_x = std::numeric_limits<double>::infinity();
            double min_delta_y = std::numeric_limits<double>::infinity();
            double min_x = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();


            for (int z = 0; z < all_target_original.cols(); z++)
            {
                if (z > 0)
                {
                    min_delta_x = std::min(abs(static_cast<double>(all_target_original.coeff(0,z)-all_target_original.coeff(0,z-1))),min_delta_x);
                    min_delta_y = std::min(abs(static_cast<double>(all_target_original.coeff(1,z)-all_target_original.coeff(1,z-1))),min_delta_y);
                }
                min_x = std::min(min_x,static_cast<double>(all_target_original.coeff(0,z)));
                max_x = std::max(max_x,static_cast<double>(all_target_original.coeff(0,z)));
                min_y = std::min(min_y,static_cast<double>(all_target_original.coeff(1,z)));
                max_y = std::max(max_x,static_cast<double>(all_target_original.coeff(1,z)));
            }
            // adding the corners

            all_target_original.conservativeResize(all_target_original.rows(),all_target_original.cols()+1);
            all_target_original(0,all_target_original.cols()-1) = min_x - min_delta_x;
            all_target_original(1,all_target_original.cols()-1) = min_y ;

            all_target_original.conservativeResize(all_target_original.rows(),all_target_original.cols()+1);
            all_target_original(0,all_target_original.cols()-1) = max_x + min_delta_x;
            all_target_original(1,all_target_original.cols()-1) = min_y ;

            all_target_original.conservativeResize(all_target_original.rows(),all_target_original.cols()+1);
            all_target_original(0,all_target_original.cols()-1) = min_x - min_delta_x;
            all_target_original(1,all_target_original.cols()-1) = max_y ;

            all_target_original.conservativeResize(all_target_original.rows(),all_target_original.cols()+1);
            all_target_original(0,all_target_original.cols()-1) = max_x + min_delta_x;
            all_target_original(1,all_target_original.cols()-1) = max_y ;
                    
            // make sure last row is ones so that it's homogeneous
            all_target_original.row(all_target_original.rows()-1) = Eigen::Matrix<double, 1,Eigen::Dynamic>::Ones(1,all_target_original.cols());
            
            for (int j=0; j<num_frames_; j++)
            {
                
                cv::Mat img_gray = reprojection.obslist_[j].image();
                cv::Mat img_color;
                cv::cvtColor(img_gray,img_color,cv::COLOR_GRAY2BGR); //convert to color to be able to plot colored dots
                //check the already found corners 
                new_obslist_[j].getCornersIdx(outCornerIdx_);
                
                std::vector<cv::Point2f> outCornerList_imageframe;  
                if (new_obslist_[j].hasSuccessfulObservation())
                {
                    new_obslist_[j].getCornersImageFrame(outCornerList_imageframe);
                }

                for (int corner_idx =0; corner_idx < outCornerList_imageframe.size(); corner_idx++)
                {
                    // cv::circle(img_color, cv::Point2f(outCornerList_imageframe[corner_idx].x,outCornerList_imageframe[corner_idx].y), 5,cv::Scalar(255,0,0),2);   
                }

                // we are wiping this entry in new_obslist_ because:
                // 1) if there aren't enough tags, we will reject the entire observation
                // 2) if we somehow can't re-find the quad for some of the original observations, we will reject that observation because the quality of it is unknown
                for (auto corner_idx : outCornerIdx_)
                {
                    new_obslist_[j].removeImagePoint(corner_idx);
                }
                if (outCornerIdx_.size() >= minInitCornersAutoComplete)
                { 
                    // retrieve quad points from image
                    Eigen::MatrixXd quads = tagDetector_->extractQuads(reprojection.obslist_[j].image());

                    // retrieve predicted 
                    sm::kinematics::Transformation out_T_t_c;
                    camera_->estimateTransformation(reprojection.obslist_[j],out_T_t_c);
                    Eigen::Matrix4d T = out_T_t_c.T().inverse();                 
                    all_target_transformed = T*all_target_original;

                    int num_target_corners = all_target_transformed.cols();
                    int num_quads = quads.cols()/4;

                    cv::Point2f pixel;
                    Eigen::MatrixXd target_image_frame(2,num_target_corners);
                    cv::Mat tagCorners(1, 2, CV_32F);

                    for (int i=0; i< num_target_corners; i++)
                    {
                        camera_->vsEuclideanToKeypoint(all_target_transformed.col(i).cast<double>(),distorted_pixel_location_);
                        // cv::circle(img_color, cv::Point2f(distorted_pixel_location_(0),distorted_pixel_location_(1)),5, cv::Scalar(0,0,0),2);    
                        target_image_frame.col(i) = distorted_pixel_location_;
                    }
                
                    Eigen::Index index_reprojection, index_target;
                    Eigen::VectorXd norms_reprojection, norms_target;

                    // for each quad, we check if all 4 corners are close to a reprojection
                    // if positive, we add the quad to the detections
                    for (int k = 0; k < num_quads; k++)
                    {
                        std::vector<Eigen::Index> quad_idxs;
                        float norm_quad = 0.0;
                        for (int q = 0; q < 4; q++)
                        {
                            norms_reprojection = (target_image_frame.colwise() - quads.col(k*4+q)).colwise().squaredNorm();
                            norms_reprojection.minCoeff(&index_reprojection);
                            if (!std::count(quad_idxs.begin(), quad_idxs.end(), index_reprojection))
                            {
                                quad_idxs.push_back(index_reprojection);
                                if (norms_reprojection(index_reprojection) > norm_quad )
                                {
                                    norm_quad = norms_reprojection(index_reprojection);
                                }
                            }
                        }

                        
                        /// TODO: Make threshold tunable
                        /// we check if the corner was already detected and if not if it's close enough to where we expect it to be
                        // if (norms_reprojection(index_reprojection) < 10.0 & !std::count(outCornerIdx_.begin(), outCornerIdx_.end(), k) )
                        // at this point we override Kalibr corners for the adaptive subpixel refinement
                        if (norm_quad < correction_threshold && quad_idxs.size() == 4 )
                        {
                            // SM_INFO_STREAM("K: "<<k);

                            for (int q = 0; q <4; q++)
                            {
                                // ///TODO: Save above instead of recomputing
                                // norms_reprojection = (target_image_frame.colwise() - quads.col(k*4+q)).colwise().squaredNorm();
                                // norms_reprojection.minCoeff(&index_reprojection);
                                index_reprojection = quad_idxs[q];

                                norms_target = (target_image_frame.colwise() - target_image_frame.col(index_reprojection)).colwise().squaredNorm();
                                std::vector<size_t> idx(norms_target.size());
                                index_target = 0;
                                
                                std::iota(idx.begin(),idx.end(),0);
                                std::stable_sort(idx.begin(), idx.end(),[&norms_target](size_t i1, size_t i2) {return norms_target[i1] <norms_target[i2];});
                                index_target = idx[1];

                                float tag_size = sqrt(norms_target(index_target));
                                if (tag_size > minTagSizeAutoComplete)
                                {
                                    int refine_window_size = static_cast<int>(tag_size/2.0);
                                    int refine_raw = refine_window_size;
                                    if (refine_window_size < minResizewindowSize)
                                    {
                                        refine_window_size = minResizewindowSize;
                                    }
                                    else if (refine_window_size > maxResizewindowSize)
                                    {
                                        refine_window_size = maxResizewindowSize;
                                    }
                                    
      
                                    //subpixel refinement

                                    // start from quads
                                    // tagCorners.at<float>(0,0) = quads.coeff(0,k*4+q);
                                    // tagCorners.at<float>(0,1) = quads.coeff(1,k*4+q);

                                    // start from reprojections
                                    tagCorners.at<float>(0,0) = target_image_frame.coeff(0,index_reprojection);
                                    tagCorners.at<float>(0,1) = target_image_frame.coeff(1,index_reprojection);

                                    // cv::circle(img_color, cv::Point2f(tagCorners.at<float>(0,0),tagCorners.at<float>(0,1)),refine_window_size, cv::Scalar(0,0,255),2);    
                                    cv::cornerSubPix(reprojection.obslist_[j].image(), tagCorners, cv::Size(refine_window_size, refine_window_size), cv::Size(-1, -1),cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,40,0.03));
                                    cv::circle(img_color, cv::Point2f(tagCorners.at<float>(0,0),tagCorners.at<float>(0,1)),refine_window_size, cv::Scalar(0,255,0),2);   
                                    quads(0,index_reprojection) = tagCorners.at<float>(0,0);
                                    quads(1,index_reprojection) = tagCorners.at<float>(0,1);

                                    new_obslist_[j].updateImagePoint(index_reprojection,quads.col(index_reprojection));
                                }
                            }

                        }


                        // SM_INFO_STREAM("NORM: "<<norms<<"\n");
                    }
                    reprojection.obslist_[j] = new_obslist_[j];
                    
                    std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist({reprojection.obslist_[j]});
                    
                    aslam::Time stamp = obslist_[j].time();
                    cv::imwrite("autofill_"+std::to_string(stamp.toSec())+".png",img_color);

                }
                // we delete the corners if they don't have the required number of initial corners to make a good estimation of the pose of the board

            }
        }

        template< typename C>
        void TartanCalibWorker<C>::homography_reprojection(aslam::cameras::ReprojectionWrapper<C>& reprojection)
        {  
            reprojection.homography_.clear();
            for (int i=0; i<num_frames_; i++)
            { 
                std::vector<cv::Scalar> colors;
                colors.push_back(cv::Scalar(0,255,0));

                // std::vector<aslam::cameras::GridCalibrationTargetObservation> obslistt;
                // obslistt.push_back(reprojection.obslist_[i]);
                // cv::imshow("test",get_mat(obslistt,true,0.,colors,5.0));
                // cv::waitKey(2000);

                aslam::cameras::GridCalibrationTargetObservation& obs = reprojection.obslist_[i];
                // obs.setImage(warped_img);
                // gd_.findTargetNoTransformation(reprojection.obslist_[i].image(),obs); //find the corners in the pinhole reprojection


                cv::Mat img = obs.image();
                std::vector<cv::Point2f> outCornerList_imageframe;  
                std::vector<cv::Point2f> outCornerList_desired;
                std::vector<cv::Point3f> outCornerList_targetframe;  
                reprojection.homography_.push_back(cv::Mat::eye(3,3,CV_64F));
                float max_x = 0;
                float max_y = 0;

                float desired_min = 0.2;
                float desired_max = 0.8;
                if (obs.hasSuccessfulObservation())
                {
                    obs.getCornersTargetFrame(outCornerList_targetframe);
                    obs.getCornersImageFrame(outCornerList_imageframe);

                    for (auto corner3d: outCornerList_targetframe)
                    {
                        if(corner3d.x > max_x)
                        {
                            max_x = corner3d.x;
                        }
                        if(corner3d.y > max_y)
                        {
                            max_y = corner3d.y;
                        }
                    }

                    for (auto corner3d: outCornerList_targetframe)
                    {
                        outCornerList_desired.push_back(cv::Point2f((obs.imCols()-(corner3d.x/max_x*(desired_max-desired_min)+desired_min)*obs.imCols()),((corner3d.y/max_y*(desired_max-desired_min)+desired_min)*obs.imRows())));
                    }

                    reprojection.homography_[i] = cv::findHomography(outCornerList_imageframe,outCornerList_desired);
                    
                    // it's possible findHomography returns an empty matrix 
                    // https://stackoverflow.com/questions/28331296/opencv-findhomography-generating-an-empty-matrix
                    if (reprojection.homography_[i].empty())
                    {
                        reprojection.homography_[i] = cv::Mat::eye(3,3,CV_64F);
                    }

                    cv::Mat warped_img;
                    cv::warpPerspective(obs.image(),warped_img,reprojection.homography_[i],obs.image().size());
                    obs.setImage(warped_img);
                    gd_.findTargetNoTransformation(obs.image(),obs);

                    std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist;
                    obslist.push_back(obs);

                    
                    cv::Mat cornered_img = get_mat(obslist,true,0.,colors,5.0);
                    aslam::Time stamp = obs.time();

                    cv::imwrite("homography_"+std::to_string(stamp.toSec())+".png",cornered_img);
                }
            }
        }

        template< typename C>
        std::vector<aslam::cameras::GridCalibrationTargetObservation> TartanCalibWorker<C>::getNewObs(void)
        {
            for (int i= 0; i<num_frames_; i++)
            {
                obs_ = new_obslist_[i];
                obs_.getCornersIdx(outCornerIdx_);
                num_corners_end += outCornerIdx_.size();
            }
            SM_INFO_STREAM("Ended tartan calib with "<<num_corners_end<<" corners.");
            
            std::ofstream myfile;
            myfile.open (log_file,std::ios_base::app);
            myfile << std::to_string(num_corners_end)<<"\n";
            myfile.close();

            export_dataset("generic_input.bin");

            return new_obslist_;
        }

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
                            
                            // add the original image
                            obs.push_back(obslist_[i]);
                            colors.push_back(cv::Scalar(0,0,255));
                            aslam::Time stamp = obslist_[i].time();

                            // these are the newly found corners projected back into the original coordinate frame 
                            obs.push_back(new_obslist_[i]);
                            colors.push_back(cv::Scalar(0,255,0));
        
                            
                            cv::Mat color_img = get_mat(obs,false,0.,colors,5.0);
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
                            temp_img = get_mat(obslist,true,-90.,colors,15.0);
                            temp_img.copyTo(img_total(cv::Rect(2*img_size,img_size,img_size,img_size)));

                            obslist.clear();
                                
                            obslist.push_back(reprojection_wrappers_[0].obslist_[i]);
                            temp_img = get_mat(obslist,true,0.,colors,15.0);
                            temp_img.copyTo(img_total(cv::Rect(img_size,0,img_size,img_size)));
                        
                            obslist.clear();
                            obslist.push_back(reprojection_wrappers_[2].obslist_[i]);
                            temp_img = get_mat(obslist,true,90.,colors,15.0);
                            temp_img.copyTo(img_total(cv::Rect(0,img_size,img_size,img_size)));

                            obslist.clear();
                            obslist.push_back(reprojection_wrappers_[1].obslist_[i]);
                            temp_img = get_mat(obslist,true,180.,colors,15.0);
                            temp_img.copyTo(img_total(cv::Rect(img_size,2*img_size,img_size,img_size)));
                            
                            obslist.clear();
                            obslist.push_back(reprojection_wrappers_[4].obslist_[i]);
                            temp_img = get_mat(obslist,true,0.,colors,15.0);
                            temp_img.copyTo(img_total(cv::Rect(img_size,img_size,img_size,img_size)));
                            
                            aslam::Time stamp = obslist_[i].time();
                            cv::imwrite("frame_"+std::to_string(stamp.toSec())+"_pinhole.png",img_total);
                        }

                        break;
                    }
                    case DebugMode::targetpointcloud:
                    {
                        cv::viz::Viz3d myWindow("Coordinate Frame");
                        myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
                        for (int i=0; i<num_frames_;i++)
                        {
                            sm::kinematics::Transformation out_T_t_c;
                            camera_->estimateTransformation(obslist_[i],out_T_t_c);
                            Eigen::Matrix4d T = out_T_t_c.T().inverse();

                            std::vector<cv::Point3f> pts3d;
                            std::vector<cv::Point2f> outCornerList;
                            cv::Point2f distorted_pixel_cv;
                            Eigen::Vector3d outPoint;
                            Eigen::VectorXd keypoint(2),target_original(4),target_transformed(4);
                            Eigen::MatrixXd all_target_original,all_target_transformed;

                            // obslist_[i].getCornersTargetFrame(pts3d);

                            cv::Mat img_gray = obslist_[i].image();
                            cv::Mat img_color;
                            cv::cvtColor(img_gray,img_color,cv::COLOR_GRAY2BGR); //convert to color to be able to plot colored dots
                            all_target_original = target_->points().transpose();
                            all_target_original.conservativeResize(4,all_target_original.cols());
                            all_target_original.row(all_target_original.rows()-1) = Eigen::Matrix<double, 1,Eigen::Dynamic>::Ones(1,all_target_original.cols());
                            all_target_transformed = T*all_target_original;

                            for (int j = 0; j<all_target_original.cols();j++)
                            {
                                camera_->vsEuclideanToKeypoint(all_target_transformed.col(j).cast<double>(),distorted_pixel_location_);
                                distorted_pixel_cv.x = distorted_pixel_location_(0);
                                distorted_pixel_cv.y = distorted_pixel_location_(1);

                                cv::circle(img_color, distorted_pixel_cv,5, cv::Scalar(0,255,0),2);           
                                pts3d.push_back(cv::Point3f(all_target_transformed.coeff(0,j),all_target_transformed.coeff(1,j),all_target_transformed.coeff(2,j)));
                            }

                            cv::viz::WCloud cloud_widget1(pts3d,cv::viz::Color::green());
                            myWindow.showWidget("cloud 1", cloud_widget1);
                            myWindow.spin();

                            cv::imshow("3D target points reprojected into camera",img_color);
                            cv::waitKey(2000);
                        }        
                        break;   
                    }
                    case DebugMode::individualprojections:
                    {
                        SM_INFO_STREAM("This mode saves all reprojections seperately.");
                       
                        std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist;
                        std::vector<cv::Scalar> colors;
                        colors.push_back(cv::Scalar(0,255,0));

                        for (int i=0; i<num_frames_; i++)
                        {
                            for (int j=0; j<num_views_; j++)
                            {
                            obslist.clear();
                            cv::Mat temp_img;
                            obslist.push_back(reprojection_wrappers_[j].obslist_[i]);
                            temp_img = get_mat(obslist,false,0.,colors,2.0);

                            aslam::Time stamp = obslist_[i].time();
                            cv::imwrite("frame_"+std::to_string(stamp.toSec())+"_projection_"+std::to_string(j)+".png",temp_img);
                            }
            
                        }
                        break;
                    }
                    case DebugMode::reprojectionpoints:
                    {
                        SM_INFO_STREAM("Plotting the observed points in a 2d plane.");
                        
                        std::vector<cv::Scalar> colors;
                        std::vector<aslam::cameras::GridCalibrationTargetObservation> obs;
                        for (int i =0; i<num_frames_; i++)
                        {
                            
                            // these are the newly found corners projected back into the original coordinate frame 
                            obs.push_back(new_obslist_[i]);
                            colors.push_back(cv::Scalar(0,255,0));
        
                            // add the original image
                            obs.push_back(obslist_[i]);
                            colors.push_back(cv::Scalar(0,0,255));

                        }
                        const std::time_t now = std::time(nullptr) ; // get the current time point

                        // convert it to (local) calendar time
                        // http://en.cppreference.com/w/cpp/chrono/c/localtime
                        const std::tm calendar_time = *std::localtime( std::addressof(now) ) ;

                        cv::Mat color_img = get_mat(obs,false,0.,colors,5.0);
                        cv::imwrite("all_points_"+std::to_string(calendar_time.tm_hour)+"_"+std::to_string(calendar_time.tm_min)+"_"+std::to_string(calendar_time.tm_sec)+".png",color_img);

                        break;
                    }
                    
                    default:
                        SM_INFO_STREAM("Default debug mode means no plotting.");

                }
            }


        }

        template< typename C>
        cv::Mat TartanCalibWorker<C>::get_mat(std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist, bool show_corners, float rotation_angle,std::vector<cv::Scalar> colors,float circle_size)
        {
            cv::Mat img_gray = obslist[0].image(); //asumption: all obs have the same image, just different corners
            cv::Mat img_color;
            cv::cvtColor(img_gray,img_color,cv::COLOR_GRAY2BGR); //convert to color to be able to plot colored dots

            std::vector<cv::Point2f> outCornerList;
            
            // getting corners as circles

                for (int i=0; i<obslist.size();i++)
                {
                    obslist[i].getCornersImageFrame(outCornerList);   
                    obslist[i].getCornersIdx(outCornerIdx_);
                    int num_corners = outCornerList.size();
                        for (int j=0; j<num_corners; j++)
                        {
                            cv::circle(img_color, outCornerList[j],circle_size, colors[i],2);
                            if (show_corners)
                            {
                                cv::putText(img_color,std::to_string(outCornerIdx_[j]),outCornerList[j],cv::FONT_HERSHEY_COMPLEX_SMALL,1.0,cv::Scalar(255,0,0),2,false);
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


                    if (reprojection.reproj_type_ == ReprojectionMode::pinhole || reprojection.reproj_type_==ReprojectionMode::homography)
                    {
                        cv::Point2f img_center(reprojection.resolution_.coeff(1,0)/2.,reprojection.resolution_.coeff(0,0)/2.);
                        std::vector<cv::Point2f> outCornerList;
                        obs_.getCornersImageFrame(outCornerList);
                        
                        num_corners_ = outCornerList.size();
                        xyz_.resize(3,num_corners_);
                        cv::Point2f corner;



                        if (reprojection.reproj_type_ == ReprojectionMode::homography && num_corners_ >0 )
                        {
                            std::vector<cv::Point2f> pinholePoints;
                            cv::perspectiveTransform(outCornerList,pinholePoints,reprojection.homography_[i].inv());
                            outCornerList = pinholePoints;
                        }
                        
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
            for (auto &reprojection: reprojection_wrappers_ )
            {
                if (reprojection.reproj_type_ == ReprojectionMode::cornerpredictor)
                {
                    match_quads(reprojection);
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
                    if (reprojection.reproj_type_ == ReprojectionMode::homography || reprojection.reproj_type_ == ReprojectionMode::pinhole || reprojection.reproj_type_ ==  ReprojectionMode::cornerpredictor )
                    {
                        obs_.setImage(reprojection.obslist_[i].image());
                        gd_.findTargetNoTransformation(reprojection.obslist_[i].image(),obs_);
                        obs_.setTime(obslist_[i].time());
                        reprojection.obslist_[i] = obs_;                        
                    }
                }
            }
            
            for (auto &reprojection: reprojection_wrappers_ )
            {
                // if a homography was requested we'll try to find the homography here
                // at a later point when merging in the corners we check this again and use the corners
                if(reprojection.reproj_type_ == ReprojectionMode::homography)
                {
                    homography_reprojection(reprojection);
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
                    if(reprojection.reproj_type_ == ReprojectionMode::homography || reprojection.reproj_type_ == ReprojectionMode::pinhole)
                    {
                    // pinhole reprojection
                    cv::Mat undistorted_image_;
                    cv::remap(reprojection.obslist_[i].image(),undistorted_image_,reprojection.map_x_,reprojection.map_y_,cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT);
                    reprojection.obslist_[i].setImage(undistorted_image_);

                    ///TODO:Remove
                    frame_reprojections.push_back(undistorted_image_);
                    }
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
                if (reprojection.reproj_type_ == ReprojectionMode::pinhole || reprojection.reproj_type_ == ReprojectionMode::homography)
                {
                    compute_remap(reprojection);
                }
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
            
            // homography first gets a pinhole, then computes a homography in that pinhole
            if (reprojection.reproj_type_ == ReprojectionMode::pinhole || reprojection.reproj_type_ == ReprojectionMode::homography)
            {

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