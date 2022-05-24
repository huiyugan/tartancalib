#ifndef ASLAM_TARTAN_CALIB
#define ASLAM_TARTAN_CALIB
#include <Eigen/Core>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/serialization/export.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <vector>
#include <utility>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <ros/platform.h>
#include <iostream>
#include <cmath>
//#include <ros/exception.h>
#include <boost/math/special_functions/round.hpp>
//#include "rostime_decl.h"
//#include <aslam/Exceptions.hpp>
#include <boost/serialization/nvp.hpp>

namespace aslam
{
    namespace cameras
    {
    class TartanCalibWorker
    {
        public:
            TartanCalibWorker(const std::vector<aslam::cameras::GridCalibrationTargetObservation>& obslist, const  Eigen::MatrixXd & fovs, const  Eigen::MatrixXd & poses, const  Eigen::MatrixXd & resolutions)
            : obslist_(obslist),
            fovs_(fovs),
            poses_(poses),
            resolutions_(resolutions) {};
            TartanCalibWorker(){};
            ~TartanCalibWorker(){}
            // bool get_xyz(void);


          /// \brief Serialization
          enum {CLASS_SERIALIZATION_VERSION = 1};
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
            Eigen::MatrixXd fovs_, poses_, resolutions_;
            std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist_;

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
