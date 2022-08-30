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


template <typename T, typename C>
bool FitSymmetry(
    int num_samples_symmetry,
    int num_meta_samples_axis,
    vector<vector<Eigen::Vector2d>> samples_targetframe,
    vector<vector<Eigen::Vector2d>> meta_target_locations,
    const Image<T>& image,
    Vec2f* out_position,
    float* final_cost,
    C* camera,
    cv::Mat cv_image,
    Eigen::Matrix4d T_target_to_euc
    ) {
      cv::Mat intensity_img(cv::Size(num_meta_samples_axis,num_meta_samples_axis),CV_32FC1);
       TargetToIntensity(image,
               T_target_to_euc,
                        camera,
    samples_targetframe[0][0]
    );
  }


template <typename T, typename C>
double TargetToIntensity(
    const Image<T>& image,
    Eigen::Matrix4d T_target_to_euc,
    C* camera,
    Eigen::Vector2d target_location
    )
    {
      Eigen::VectorXd distorted_pixel_location;
      Eigen::VectorXd target_location_H = target_location;
      target_location_H  << 0.0 , 1.0;
      Eigen::VectorXd sample_3D = T_target_to_euc*target_location_H;
      camera->vsEuclideanToKeypoint(sample_3D,distorted_pixel_location);
      
      Vec2f sample_pos = Vec2f(distorted_pixel_location(1),distorted_pixel_location(0));    
      return image.InterpolateBilinear(sample_pos);
    }



// The positions are specified in pixel-center origin convention.
template <typename T, typename Derived, typename C>
bool ComputSymmetryCost(
    int num_samples,
    vector<vector<Eigen::VectorXd>> samples_pointcloud,
    const Image<T>& image,
    int window_half_size,
    const MatrixBase<Derived>& position,
    const Mat4d& local_pattern_tr_pixel,
    const Eigen::VectorXd corner_3d,
    Vec2f* out_position,
    float* out_cost,
    bool debug,
    C* original_camera,
    cv::Mat cv_image,
    Matrix<float, 6, 6>* H,
    Matrix<float, 6, 1>* b
    ) {
      // doing a deep copy with the original cam, we can't change the original camera
      C* local_camera = original_camera;
      *out_cost = 0;
      // SM_INFO_STREAM("1");
      // Eigen::MatrixXd original_cam_params;
      // original_camera->getParameters(original_cam_params,true,true,true);
      // SM_INFO_STREAM("Original_params: "<< original_cam_params);
      // local_camera->setParameters(original_cam_params,true,true,true);
      // SM_INFO_STREAM("1");
      Eigen::MatrixXd original_cam_params;
      original_camera->getParameters(original_cam_params,true,true,true);
      int num_intr_params = original_cam_params.cols();
      Eigen::VectorXd corner_imageframe;

      // //debug effort
      // cv::Mat grad_x(cv::Size(1028,1224),CV_32FC1);
      // float intensity;
      // Eigen::Matrix<float, 1, 2> gradient;
      // Vec2f sample_coord;
      // for (int i = 0; i< 1028; i++)
      // {
      //   SM_INFO_STREAM("ROW "<<i);
      //   for (int j = 0; j < 1224; j++)
      //   {
      //       sample_coord = Vec2f(i,j);
      //       image.InterpolateBilinearWithJacobian(sample_coord, &intensity, &gradient);
      //       grad_x.at<float>(i,j) = gradient.coeff(0,1);
      //   }
      // }
      // cv::imwrite("grad_debug.png",grad_x);

        // step 1: create the first pointcloud
        for (int num_sample = 0; num_sample < num_samples; num_sample++)
        {
          Eigen::VectorXd positive_sample_image_frame, negative_sample_image_frame;
          Eigen::MatrixXd positive_outJacobian_projection, negative_outJacobian_projection;
          vector<Eigen::VectorXd> sample_couple = samples_pointcloud[num_sample]; 
          // getting jacobian of keypoint w.r.t. intrinsics
          // local_camera->euclideanToKeypointIntrinsicsJacobian(sample_couple[0],positive_outJacobian_projection,true,false,false);
          // local_camera->euclideanToKeypointIntrinsicsJacobian(sample_couple[1],negative_outJacobian_projection,true,false,false);
        
          // reproject 3D points into image plane
          local_camera->vsEuclideanToKeypoint(sample_couple[0],positive_sample_image_frame);
          local_camera->vsEuclideanToKeypoint(sample_couple[1],negative_sample_image_frame);

          Vec2f sample_pos = Vec2f(positive_sample_image_frame(1),positive_sample_image_frame(0));
          Vec2f sample_neg = Vec2f(negative_sample_image_frame(1),negative_sample_image_frame(0));
          bool in_image = (image.ContainsPixelCenterConv(sample_pos) && image.ContainsPixelCenterConv(sample_neg));

          if (in_image)
          {
            // cv::circle(cv_image, cv::Point2f(positive_sample_image_frame(0),positive_sample_image_frame(1)),0, cv::Scalar(255,0,0),2); 
            // cv::circle(cv_image, cv::Point2f(negative_sample_image_frame(0),negative_sample_image_frame(1)),0, cv::Scalar(255,0,0),2); 

            // gradient of positive sample
            float intensity_pos;
            Eigen::Matrix<float, 1, 2> gradient_pos;
            image.InterpolateBilinearWithJacobian(sample_pos, &intensity_pos, &gradient_pos);
            // float grad_y = gradient_pos.coeff(0,0);
            // gradient_pos(0,0) = gradient_pos(0,1);
            // gradient_pos(0,1) = grad_y;

            // gradient of negative sample
            float intensity_neg;
            Eigen::Matrix<float, 1, 2> gradient_neg;
            image.InterpolateBilinearWithJacobian(sample_neg, &intensity_neg, &gradient_neg);
            // grad_y = gradient_neg.coeff(0,0);
            // gradient_neg(0,0) = gradient_neg(0,1);
            // gradient_neg(0,1) = grad_y;

            double residual = static_cast<double>(intensity_pos - intensity_neg);
            *out_cost += static_cast<float>(residual*residual);

            // SM_INFO_STREAM("Raw Jacobian positive :"<<positive_outJacobian_projection);
            // SM_INFO_STREAM("Img jac positive :"<<gradient_pos);

            // SM_INFO_STREAM("Raw Jacobian positive :"<<negative_outJacobian_projection);
            // SM_INFO_STREAM("Img jac positive :"<<gradien_neg);
          }
          else
          {
            *out_cost = numeric_limits<float>::infinity();
          }
        // *out_cost = static_cast<float>(cost);
        // Eigen::MatrixXd cam_params;
        // local_camera->getParameters(cam_params,true,false,false);
        

        // Eigen::MatrixXd jacobian_corrected = learning_rate*jacobian_intrinsics.cast<double>();
        // // SM_INFO_STREAM("Jacobian : "<<jacobian_corrected);
        // // SM_INFO_STREAM("Cost: "<<cost);
        // const Eigen::MatrixXd& updated_cam_params = cam_params - jacobian_corrected ;
        // local_camera->setParameters(updated_cam_params,true,false,false);
        // // SM_INFO_STREAM("ORIGINAL PARAMETERS: " << cam_params);
        // // debugging

        local_camera->vsEuclideanToKeypoint(corner_3d,corner_imageframe);
        // int red = static_cast<int>((1-static_cast<float>(it)/static_cast<float>(num_iterations))*255.);
        // int green =  static_cast<int>((static_cast<float>(it)/static_cast<float>(num_iterations))*255.);
        cv::circle(cv_image, cv::Point2f(corner_imageframe(0),corner_imageframe(1)),0, cv::Scalar(255,0,0),2);           
        // cv::circle(cv_image, cv::Point2f(negative_sample_image_frame(0),negative_sample_image_frame(1)),0, cv::Scalar(0,255,0),2);    
        // cv::imwrite("test_"+std::to_string(it)+".png",cv_image);
      }
     
    
    }


}