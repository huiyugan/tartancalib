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

template <typename C>
Vec2f TargetToImageFrame(
    Eigen::Vector4d target_location,
    Eigen::Matrix4d T_target_to_euc,
    C* camera)
{
  Eigen::VectorXd distorted_pixel_location;
  Eigen::VectorXd sample_3D = T_target_to_euc*target_location; 
  camera->vsEuclideanToKeypoint(sample_3D,distorted_pixel_location);
  return Vec2f(distorted_pixel_location(0),distorted_pixel_location(1));
}

template <typename T, typename C, size_t rows, size_t cols>
bool DebugScreen(
    Eigen::Vector4d start_target_frame,
    int num_samples_symmetry,
    int num_meta_samples_axis,
    vector<vector<Eigen::VectorXd>> samples_targetframe,
    Eigen::Vector4d (&meta_locations)[rows][cols],
    const Image<T>& image,
    Vec2f* out_position,
    float* final_cost,
    C* camera,
    cv::Mat cv_image,
    Eigen::Matrix4d T_target_to_euc
    )
    {
      cv::Mat intensity_img(cv::Size(num_meta_samples_axis,num_meta_samples_axis),CV_32FC1);
      for (int i = 0; i< num_meta_samples_axis; i++)
      {
        for (int j = 0; j< num_meta_samples_axis; j++)
        {
          intensity_img.at<float>(i,j) = static_cast<float>(TargetToIntensity(image, T_target_to_euc, camera, start_target_frame +  meta_locations[i][j]));
        }
      }
      cv::imwrite("intensity_test.png",intensity_img);

      cv::Mat symmetry_img(cv::Size(num_meta_samples_axis,num_meta_samples_axis),CV_32FC1);
      
      for (int i = 0; i< num_meta_samples_axis; i++)
      {
        for (int j = 0; j< num_meta_samples_axis; j++)
        {
          Eigen::Vector2d Jacobian;
          Jacobian.setZero();

          symmetry_img.at<float>(i,j) = static_cast<float>(TargetToSymmetry(image, T_target_to_euc, camera, start_target_frame +  meta_locations[i][j],samples_targetframe,num_samples_symmetry,cv_image));
          // symmetry_img.at<float>(i,j) = static_cast<float>(TargetToJacobian(image, T_target_to_euc, camera, start_target_frame +  meta_locations[i][j],samples_targetframe,num_samples_symmetry,cv_image,&Jacobian));

        }
      }
      double minVal; 
      double maxVal; 
      cv::Point minLoc; 
      cv::Point maxLoc;

      minMaxLoc( symmetry_img, &minVal, &maxVal, &minLoc, &maxLoc );
      symmetry_img *= 255.0/maxVal;
      symmetry_img = 255.0 - symmetry_img;
      SM_INFO_STREAM("Best point: "<<minLoc);
      cv::imwrite("symmetry_test.png",symmetry_img);
      // cv::waitKey(20000);

      // SM_INFO_STREAM("Best target: "<<start_target_frame +  meta_locations[minLoc.x][minLoc.y]);
      *out_position = TargetToImageFrame(start_target_frame, T_target_to_euc, camera);
    }  

template <typename T, typename C, size_t rows, size_t cols>
bool FitSymmetry(
    Eigen::Vector4d start_target_frame,
    int num_samples_symmetry,
    int num_meta_samples_axis,
    vector<vector<Eigen::VectorXd>> samples_targetframe,
    Eigen::Vector4d (&meta_locations)[rows][cols],
    const Image<T>& image,
    Vec2f* out_position,
    float* final_cost,
    C* camera,
    cv::Mat cv_image,
    Eigen::Matrix4d T_target_to_euc
    ) {
      // SM_INFO_STREAM("Target initial position: "<<start_target_frame);
      Eigen::Vector4d intermediate_target_frame = start_target_frame;
      // DebugScreen(start_target_frame, num_samples_symmetry,num_meta_samples_axis,samples_targetframe,meta_locations,image,out_position,final_cost,camera,cv_image,T_target_to_euc);
      constexpr int kDim = 2;
      Matrix<double, kDim, kDim> H;
      Matrix<double, kDim, 1> b;
      double lambda = -1;
      double last_step_squared_norm = numeric_limits<float>::infinity();
      
      constexpr int num_it = 10;
      for (int i = 0; i< num_it; i++)
      {
        Eigen::Vector2d Jacobian;
        double cost = TargetToJacobian(image, T_target_to_euc, camera, intermediate_target_frame,samples_targetframe,num_samples_symmetry,cv_image,&Jacobian,&H, &b);
        double jac_norm = Jacobian.norm();
        Eigen::Vector2d Jac_norm = Jacobian/jac_norm * 0.00005;

        intermediate_target_frame(0) -= Jac_norm(0);
        intermediate_target_frame(1) -= Jac_norm(1);
        // SM_INFO_STREAM("Refined target position: "<<out_pos);
      }
      

      *out_position = TargetToImageFrame(intermediate_target_frame,T_target_to_euc,camera);
      // constexpr int kMaxIterationCount = 0;
      // for (int iteration = 0; iteration < kMaxIterationCount; ++ iteration) {
       
      //   // *out_position = TargetToImageFrame(start_target_frame, T_target_to_euc, camera);

      //   // if (kDebug) {
      //   //   SM_INFO_STREAM("cost: " << cost);
      //   // }
      //   // if (final_cost) {
      //   //   *final_cost = cost;
      //   // }
    
      // // Initialize lambda?
      // if (lambda < 0) {
      //   lambda = 0.001f * (1.f / kDim) * H.diagonal().sum();
      // }
    
      // bool applied_update = false;
      // for (int lm_iteration = 0; lm_iteration < 1; ++ lm_iteration) {
      //   Matrix<double, kDim, kDim> H_LM;
      //   H_LM.triangularView<Eigen::Upper>() = H.triangularView<Eigen::Upper>();
      //   H_LM.diagonal().array() += lambda;
      
      
      //   Eigen::Matrix<double, kDim, 1> x = H_LM.cast<double>().selfadjointView<Eigen::Upper>().ldlt().solve(b.cast<double>());

      

      //   double test_cost;
      //   // SM_INFO_STREAM("x: "<<x);
      // if (!CostFunction::ComputeCornerRefinementCost(test_pixel_tr_pattern_samples, image, num_samples, pattern_samples, &test_cost)) {
      //   if (kDebug) {
      //   //   LOG(INFO) << "  CostFunction::ComputeCornerRefinementCost() failed, aborting.";
      //   }
      //   return false;
      // }
      
      // if (kDebug) {
      //   // LOG(INFO) << "  test_cost: " << test_cost << ", cost: " << cost;
      // }
      
      // if (test_cost < cost) {
      //   if (final_cost) {
      //     *final_cost = test_cost;
      //   }
      //   last_step_squared_norm = Vec2f(x(2), x(5)).squaredNorm();  // using the translation only
      //   pixel_tr_pattern_samples = test_pixel_tr_pattern_samples;
      //   lambda *= 0.5f;
      //   applied_update = true;
      //   break;
      // } else {
        // lambda *= 2.f;
      // }
    // }
    
    // if (applied_update) {
    //   // Since the element at (2, 2) is always 1, we can directly assign the
    //   // translation values instead of computing:
    //   // *out_position = (pixel_tr_pattern_samples * Vec3f(0, 0, 1)).hnormalized();
    //   out_position->x() = pixel_tr_pattern_samples(0, 2);
    //   out_position->y() = pixel_tr_pattern_samples(1, 2);
    //   if (kDebug) {
    //     // LOG(INFO) << "New position: " << out_position->transpose();
    //   }
    // } else {
    //   // Cannot find an update that improves the cost. Treat this as converged.
    //   if (kDebug) {
    //     // LOG(INFO) << "Cannot find an update to improve the cost. Returning convergence (iteration " << iteration << ").";
    //   }
    // }
  // }
  }




template <typename T, typename C>
double TargetToIntensity(
    const Image<T>& image,
    Eigen::Matrix4d T_target_to_euc,
    C* camera,
    Eigen::VectorXd target_location
    )
    {
      Eigen::VectorXd distorted_pixel_location;
      Eigen::VectorXd target_location_H = target_location;
      // target_location_H  << 0.0 , 1.0;
      Eigen::VectorXd sample_3D = T_target_to_euc*target_location_H;
      camera->vsEuclideanToKeypoint(sample_3D,distorted_pixel_location);
      Vec2f sample_pos = Vec2f(distorted_pixel_location(1),distorted_pixel_location(0));    
      return image.InterpolateBilinear(sample_pos);
    }



template <typename T, typename C>
double TargetToSymmetry(
    const Image<T>& image,
    Eigen::Matrix4d T_target_to_euc,
    C* camera,
    Eigen::VectorXd target_location,
    vector<vector<Eigen::VectorXd>> samples_targetframe,
    int num_samples_symmetry,
    cv::Mat cv_img
    )
    {
      double C_symmetry = 0;
      double intensity_pos, intensity_neg;
      Eigen::VectorXd distorted_pixel_location;
      Eigen::VectorXd target_location_pos, target_location_neg, pos_3D, neg_3D, image_pos, image_neg;
      for (int i = 0; i < num_samples_symmetry ; i++)
      {
        vector<Eigen::VectorXd> sample_pair = samples_targetframe[i];
        
        target_location_pos = sample_pair[0] + target_location;
        target_location_neg = sample_pair[1] + target_location;

        pos_3D =  T_target_to_euc*target_location_pos;
        neg_3D =  T_target_to_euc*target_location_neg;

        camera->vsEuclideanToKeypoint(pos_3D,image_pos);
        camera->vsEuclideanToKeypoint(neg_3D,image_neg);
        
        Vec2f sample_pos = Vec2f(image_pos(1),image_pos(0));   
        Vec2f sample_neg = Vec2f(image_neg(1),image_neg(0)); 

        intensity_pos = image.InterpolateBilinear(sample_pos);
        intensity_neg = image.InterpolateBilinear(sample_neg);

        C_symmetry += (intensity_pos-intensity_neg)*(intensity_pos-intensity_neg);
        // cv::circle(cv_img, cv::Point2f(sample_pos(1),sample_pos(0)),3, cv::Scalar(255,0,0),2);
        // cv::circle(cv_img, cv::Point2f(sample_neg(1),sample_neg(0)),3, cv::Scalar(255,0,0),2);

        // SM_INFO_STREAM("Positive location: "<<sample_pos);
        // SM_INFO_STREAM("Negative location: "<<sample_neg);

        // cv::imshow("test",cv_img);
        // cv::waitKey(20000);

      }
      return C_symmetry;

    }


template <typename T, typename C>
double TargetToJacobian(
    const Image<T>& image,
    Eigen::Matrix4d T_target_to_euc,
    C* camera,
    Eigen::Vector4d target_location,
    vector<vector<Eigen::VectorXd>> samples_targetframe,
    int num_samples_symmetry,
    cv::Mat cv_img,
    Eigen::Vector2d* Jacobian,
    Matrix<double, 2, 2>* H,
    Matrix<double, 2, 1>* b
    )
    {
      H->triangularView<Eigen::Upper>().setZero();
      b->setZero();
      double C_symmetry = 0;
      double intensity_pos, intensity_neg;
      Eigen::Matrix<double, 1, 2> gradient_pos, gradient_neg, gradient_pos_, gradient_neg_;
      Eigen::VectorXd distorted_pixel_location;
      Eigen::MatrixXd Jacobian_cam_pos, Jacobian_cam_neg, Jacobian_cam_pos_T, Jacobian_cam_neg_T, Jacobian_transformation;
      for (int i = 0; i < num_samples_symmetry ; i++)
      {
        vector<Eigen::VectorXd> sample_pair = samples_targetframe[i];
        Eigen::VectorXd target_location_pos, target_location_neg, pos_3D, neg_3D, image_pos, image_neg;
        

        target_location_pos = sample_pair[0] + target_location;
        target_location_neg = sample_pair[1] + target_location;

        pos_3D =  T_target_to_euc*target_location_pos;
        neg_3D =  T_target_to_euc*target_location_neg;

        // jacobian of camera model, image coordinates w.r.t. euclidean coordinates
        camera->vsEuclideanToKeypoint(pos_3D,image_pos,Jacobian_cam_pos);
        camera->vsEuclideanToKeypoint(neg_3D,image_neg,Jacobian_cam_neg);

        // getting the transpose
        Jacobian_cam_pos_T = Jacobian_cam_pos.transpose();
        Jacobian_cam_neg_T = Jacobian_cam_neg.transpose();
        
        Vec2f sample_pos = Vec2f(image_pos(1),image_pos(0));   
        Vec2f sample_neg = Vec2f(image_neg(1),image_neg(0)); 

        // image gradients
        image.InterpolateBilinearWithJacobian(sample_pos, &intensity_pos, &gradient_pos);
        image.InterpolateBilinearWithJacobian(sample_neg, &intensity_neg, &gradient_neg);

        gradient_pos_(0,0) = gradient_pos(0,1);
        gradient_pos_(0,1) = gradient_pos(0,0);

        gradient_neg_(0,0) = gradient_neg(0,1);
        gradient_neg_(0,1) = gradient_neg(0,0);


        // Jacobian of transformation between euclidean and target coordinates
        Jacobian_transformation = T_target_to_euc.block<3,2>(0,0);
        
        MatrixXd jacobian_pos = gradient_pos_*Jacobian_cam_pos_T*Jacobian_transformation;
        MatrixXd jacobian_neg = gradient_neg_*Jacobian_cam_neg_T*Jacobian_transformation;

        // SM_INFO_STREAM("Suggested changes: "<<(jacobian_pos-jacobian_neg));
        *Jacobian += (jacobian_pos-jacobian_neg);

        C_symmetry += (intensity_pos-intensity_neg)*(intensity_pos-intensity_neg);
        // cv::circle(cv_img, cv::Point2f(sample_pos(1),sample_pos(0)),3, cv::Scalar(255,0,0),2);
        // cv::circle(cv_img, cv::Point2f(sample_neg(1),sample_neg(0)),3, cv::Scalar(255,0,0),2);

        // SM_INFO_STREAM("Positive location: "<<sample_pos);
        // SM_INFO_STREAM("Negative location: "<<sample_neg);

        // cv::imshow("test",cv_img);
        // cv::waitKey(20000);

      }
      Eigen::MatrixXd Jacobian_mat = *Jacobian;
      // SM_INFO_STREAM("Jacobian RAW"<<Jacobian_mat);
      H->triangularView<Eigen::Upper>() += Jacobian_mat.transpose() * Jacobian_mat;
      *b += C_symmetry * Jacobian_mat;
      return C_symmetry;

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
        // cv::circle(cv_image, cv::Point2f(corner_imageframe(0),corner_imageframe(1)),0, cv::Scalar(255,0,0),2);           
        // cv::circle(cv_image, cv::Point2f(negative_sample_image_frame(0),negative_sample_image_frame(1)),0, cv::Scalar(0,255,0),2);    
        // cv::imwrite("test_"+std::to_string(it)+".png",cv_image);
      }
     
    
    }


}