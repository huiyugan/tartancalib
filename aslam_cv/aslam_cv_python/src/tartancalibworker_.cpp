#include <aslam/TartanCalibWorker.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/operators.hpp>
#include <sm/python/boost_serialization_pickle.hpp>

using namespace boost::python; 
using namespace aslam::cameras;
// class aslam::cameras::Tartan_Calib_Worker;


void exportTartan()
{       
        using namespace aslam::cameras;
        class_<aslam::cameras::TartanCalibWorker>("TartanCalibWorker")
        .def(init<>())
        .def(init<const std::vector<aslam::cameras::GridCalibrationTargetObservation>&, const Eigen::MatrixXd&, const Eigen::MatrixXd &, const Eigen::MatrixXd&>())
        .def("get_xyz",&aslam::cameras::TartanCalibWorker::get_xyz)
        .def_pickle(sm::python::pickle_suite<aslam::cameras::TartanCalibWorker>())
        ;
}
