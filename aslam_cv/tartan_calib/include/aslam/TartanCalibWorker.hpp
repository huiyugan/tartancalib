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

namespace aslam
{
    namespace cameras
    {
    class TartanCalibWorker
    {
        public:
            TartanCalibWorker(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& obslist, const  Eigen::MatrixXd & fovs, const  Eigen::MatrixXd & poses, const  Eigen::MatrixXd & resolutions, const bool verbose)
            : obslist_(obslist),
            fovs_(fovs),
            poses_(poses),
            resolutions_(resolutions),
            verbose_(verbose)
             {
              num_frames_ = obslist.size();
              num_views_ = fovs.cols();
              SM_ASSERT_TRUE(std::runtime_error,num_views_ == poses_.cols() && poses_.cols() == resolutions_.cols(), "All inserted tartan matrices need the same number of columns." );
            };
            TartanCalibWorker(){};
            ~TartanCalibWorker(){}
            void compute_xyzs(void);
            void compute_xyz(const Eigen::MatrixXd& fov, const Eigen::MatrixXd& resolution,const Eigen::MatrixXd& pose);
            void compute_rotation(const Eigen::MatrixXd& pose );
            



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
            int num_frames_,num_views_,num_points_;
            float rot_x_, rot_z_;
            bool verbose_;

    };
    }
}


// namespace boost {
// namespace serialization {

// template<class Archive>
// void serialize(Archive & ar, aslam::cameras::TartanCalibWorker & t, const unsigned int /* version */) {
  
// }

// }  // namespace serialization
// }  // namespace boost

SM_BOOST_CLASS_VERSION(aslam::cameras::TartanCalibWorker);
#endif /* ASLAM_TARTAN_CALIB */
