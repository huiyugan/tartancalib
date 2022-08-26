// Copyright 2019 ETH Zürich, Thomas Schöps
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "libvis/eigen.h"
#include "libvis/image.h"
// #include <libvis/libvis.h>

namespace vis {

// The positions are specified in pixel-center origin convention.
template <typename T, typename Derived, typename C>
bool RefineFeatureByTartan(
    int num_samples,
    vector<vector<Eigen::VectorXd>> samples_pointcloud,
    const Image<T>& image,
    int window_half_size,
    const MatrixBase<Derived>& position,
    const Mat4d& local_pattern_tr_pixel,
    Vec2f* out_position,
    float* final_cost,
    bool debug,
    C* camera,
    cv::Mat cv_image
    ) {
      C* local_camera = camera; //local camera will be changed parameters
      int num_iterations = 100;

      
      for (int it = 0; it < num_iterations; it++)
      {
        // step 1: create the first pointcloud
        for (int num_sample = 0; num_sample < num_samples; num_sample++)
        {
          Eigen::VectorXd positive_sample_image_frame, negative_sample_image_frame;
          Eigen::MatrixXd positive_outJacobian, negative_outJacobian;
          vector<Eigen::VectorXd> sample_couple = samples_pointcloud[num_sample]; 
          
          local_camera->vsEuclideanToKeypoint(sample_couple[0],positive_sample_image_frame,positive_outJacobian);
          local_camera->vsEuclideanToKeypoint(sample_couple[1],negative_sample_image_frame,negative_outJacobian);

          cv::circle(cv_image, cv::Point2f(positive_sample_image_frame(0),positive_sample_image_frame(1)),0, cv::Scalar(0,255,0),2);           
          cv::circle(cv_image, cv::Point2f(negative_sample_image_frame(0),negative_sample_image_frame(1)),0, cv::Scalar(0,255,0),2);    
          cv::imshow("test",cv_image);
          cv::waitKey(20000);

        }
        

      }

    }
}

  