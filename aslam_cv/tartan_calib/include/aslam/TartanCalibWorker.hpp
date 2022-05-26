#ifndef ASLAM_TARTAN_CALIB
#define ASLAM_TARTAN_CALIB
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
namespace aslam
{

    namespace cameras
    {
    template< typename C>
    class TartanCalibWorker:
      public boost::enable_shared_from_this<TartanCalibWorker<C>>
    {
        public:  
            TartanCalibWorker(const C* camera,const std::vector<aslam::cameras::GridCalibrationTargetObservation>& obslist, const  Eigen::MatrixXd & fovs, const  Eigen::MatrixXd & poses, const  Eigen::MatrixXd & resolutions, const bool verbose)
            : obslist_(obslist),
            camera_(camera),
            fovs_(fovs),
            poses_(poses),
            resolutions_(resolutions),
            verbose_(verbose),
            in_width_(obslist[0].imCols()),
            in_height_(obslist[0].imRows()) // assumption: this array only contains images of the same resolution, as we expect to create one TartanCalibWorker class per camera
             { 
              num_frames_ = obslist.size();
              num_views_ = fovs.cols();
              SM_ASSERT_TRUE(std::runtime_error,num_views_ == poses_.cols() && poses_.cols() == resolutions_.cols(), "All inserted tartan matrices need the same number of columns." );
              
            };
            TartanCalibWorker(){};
            ~TartanCalibWorker(){}
            void compute_xyzs();
            void compute_xyz(const Eigen::MatrixXd& fov, const Eigen::MatrixXd& resolution,const Eigen::MatrixXd& pose);
            void compute_rotation(const Eigen::MatrixXd& pose );
            void compute_remap(const Eigen::Matrix<float, 3, Eigen::Dynamic>& xyz, const Eigen::MatrixXd& resolution);
            void compute_remaps(); 
               


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
            Eigen::Matrix<float, 3, Eigen::Dynamic> xyz_; // rows are xyz, each column is a point
            Eigen::Matrix<float,3,3> rot_mat_,rot_mat_x_,rot_mat_z_;
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> xx_,yy_; //vector with x and y values in the global
            Eigen::MatrixXd fovs_, poses_, resolutions_;
            std::vector<Eigen::Matrix<float, 3, Eigen::Dynamic>> xyzs_;
            std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist_;
            int num_frames_,num_views_,num_points_,in_width_,in_height_,xyz_idx_;
            float rot_x_, rot_z_;
            bool verbose_,empty_pixels_;
            const C* camera_;

            // used to create remap
            Eigen::VectorXd xyz_ray_;
            Eigen::VectorXd distorted_pixel_location_; 

            // map parameters
            cv::Size resolution_out_;
            cv::Mat map_x_float_,map_y_float_, map_x_, map_y_;
            std::vector<cv::Mat> maps_x_, maps_y_;

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
