#ifndef ASLAM_TARTAN_CALIB
#define ASLAM_TARTAN_CALIB
#define CVPLOT_HEADER_ONLY
#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/serialization/export.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <vector>
#include <utility>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include <boost/serialization/nvp.hpp>
#include <sm/logging.hpp>
#include <boost/math/constants/constants.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/assign/list_of.hpp> // for 'map_list_of()'

namespace aslam
{
    
    namespace cameras
    {
      enum class DebugMode 
      {
        pointcloudprojection,
        pinholeprojection,
        originalprojection,
        none
      };
      
      enum class ReprojectionMode 
      {
        pinhole,
        none
      };


    template< typename C>
    class ReprojectionWrapper:
    public boost::enable_shared_from_this<ReprojectionWrapper<C>>
    {
      public:
        ReprojectionWrapper(std::vector<aslam::cameras::GridCalibrationTargetObservation>& obslist, const Eigen::MatrixXd & fov, const  Eigen::MatrixXd & pose, const  Eigen::MatrixXd & resolution,const ReprojectionMode reproj_type):
        obslist_(obslist),
        fov_(fov),
        pose_(pose),
        resolution_(resolution),
        reproj_type_(reproj_type)
        {

        };
      
        std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist_;
        Eigen::MatrixXd fov_,pose_,resolution_;
        cv::Mat map_x_, map_y_;
        ReprojectionMode reproj_type_;
        Eigen::Matrix<float, 3, Eigen::Dynamic> xyz_;
    };


    template< typename C>
    class TartanCalibWorker:
      public boost::enable_shared_from_this<TartanCalibWorker<C>>
    {
        public:  
            TartanCalibWorker(const C* camera,aslam::cameras::GridDetector gd,const std::vector<aslam::cameras::GridCalibrationTargetObservation>& obslist, const  Eigen::MatrixXd & fovs, const  Eigen::MatrixXd & poses, const  Eigen::MatrixXd & resolutions,const std::vector<std::string>& reproj_types, const std::vector<std::string>& debug_modes)
            : obslist_(obslist),
            gd_(gd),
            camera_(camera),
            fovs_(fovs),
            poses_(poses),
            resolutions_(resolutions),
            in_width_(obslist[0].imCols()),
            in_height_(obslist[0].imRows()) // assumption: this array only contains images of the same resolution, as we expect to create one TartanCalibWorker class per camera
             { 
              num_frames_ = obslist.size();
              num_views_ = fovs.cols();
              new_obslist_ = obslist_; // eventually we're outputting this variable, but we initialize it with the right observations (i.e., images and time stamps)
              SM_ASSERT_TRUE(std::runtime_error,num_views_ == poses_.cols() && poses_.cols() == resolutions_.cols(), "All inserted tartan matrices need the same number of columns." );
              
              // loading reprojectionwrapper classes
              for (int i = 0; i < num_views_; i++)
              {
                reprojection_wrappers_.push_back(ReprojectionWrapper<C>(obslist_,fovs.col(i),poses.col(i),resolutions.col(i),StringToReprojectionMode(reproj_types[i])));
              }

              for (auto debug_mode : debug_modes)
              {
                debug_modes_.push_back(StringToDebug(debug_mode));
              }

            
            };
            TartanCalibWorker(){};
            ~TartanCalibWorker(){}
            void compute_xyzs();
            void compute_xyz(aslam::cameras::ReprojectionWrapper<C>& reprojection);
            void compute_rotation(const Eigen::MatrixXd& pose );
            void compute_remap(aslam::cameras::ReprojectionWrapper<C>& reprojection);
            void project_to_original_image(void);
            void compute_remaps(); 
            void compute_reprojections();
            void compute_corners();
            cv::Mat get_mat(std::vector<aslam::cameras::GridCalibrationTargetObservation>,bool,float,std::vector<cv::Scalar> );   
            void debug_show(void);

            
           
            std::string ReprojectionModeToString(DebugMode e)
            {
              switch (e)
              {
                case ReprojectionMode::pinhole: return "pinhole";
                case ReprojectionMode::none: return "none";
              }
            }
            ReprojectionMode StringToReprojectionMode(std::string e)
            {
              std::map<std::string, ReprojectionMode> reprojection_mode = boost::assign::map_list_of
              ("pinhole",ReprojectionMode::pinhole)
              ("none",ReprojectionMode::none);
              return reprojection_mode[e];
            }
            
            // DebugMode converters
            std::string DebugToString(DebugMode e)
            {
              switch (e)
              {
                case DebugMode::pointcloudprojection: return "pointcloudprojection";
                case DebugMode::pinholeprojection: return "pinholeprojection";
                case DebugMode::originalprojection: return "originalprojection";
                case DebugMode::none: return "none";
              }
            }
            DebugMode StringToDebug(std::string e)
            {
              std::map<std::string, DebugMode> debug_mode = boost::assign::map_list_of
              ("pointcloudprojection",DebugMode::pointcloudprojection)
              ("pinholeprojection", DebugMode::pinholeprojection)
              ("originalprojection",DebugMode::originalprojection)
              ("none",DebugMode::none);
              return debug_mode[e];
            }

                
            /// \brief Serialization
            enum {CLASS_SERIALIZATION_VERSION = 4};
            BOOST_SERIALIZATION_SPLIT_MEMBER()

            /// \brief Serialization support
            template<class Archive>
            void save(Archive & ar, const unsigned int /*version*/) const
            {

            }
            template<class Archive>
            void load(Archive & ar, const unsigned int /*version*/)
            {
            }
          
        private:
            // key class variables
            std::vector<DebugMode> debug_modes_; //determines what the user will see
            std::vector<aslam::cameras::ReprojectionWrapper<C>> reprojection_wrappers_; 
            std::vector<std::string> reproj_types_;

            Eigen::Matrix<float, 3, Eigen::Dynamic> xyz_; // rows are xyz, each column is a point
            Eigen::Matrix<float,3,3> rot_mat_,rot_mat_x_,rot_mat_z_;
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> xx_,yy_; //vector with x and y values in the global
            Eigen::MatrixXd fovs_, poses_, resolutions_;
            std::vector<Eigen::Matrix<float, 3, Eigen::Dynamic>> xyzs_;
            
            std::vector<unsigned int> outCornerIdx_;
            aslam::cameras::GridCalibrationTargetObservation obs_;
            std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist_;
            int num_frames_,num_views_,num_points_,in_width_,in_height_,xyz_idx_,num_corners_;
            float rot_x_, rot_z_;
            bool verbose_,empty_pixels_;
            const C* camera_;

            // used to create remap
            Eigen::VectorXf xyz_ray_;
            Eigen::VectorXd distorted_pixel_location_; 

            // map parameters
            cv::Size resolution_out_;
            cv::Mat map_x_float_,map_y_float_;
            std::vector<cv::Mat> maps_x_, maps_y_;
            
            std::vector<std::vector<cv::Mat>> reprojections_;
            std::vector<aslam::cameras::GridCalibrationTargetObservation> new_obslist_;
            aslam::cameras::GridDetector gd_;
            std::vector<aslam::cameras::GridCalibrationTargetObservation> output_obslist_;
            

    };
    }
}


// namespace boost {
// namespace serialization {

// template<class Archive>
// void serialize(Archive & ar, const unsigned int /* version */) {
  
// }

// }  // namespace serialization
// }  // namespace boost

SM_BOOST_CLASS_VERSION_T1(aslam::cameras::TartanCalibWorker);
#include<aslam/implementation/TartanCalibWorker.hpp>
#endif /* ASLAM_TARTAN_CALIB */
