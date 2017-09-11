// Copyright (c) <2017>, <Nanyang Technological University> All rights reserved.

// This file is part of correlation_flow.

//     correlation_flow is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     Foobar is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

#ifndef CORRELATION_FLOW_H
#define CORRELATION_FLOW_H

#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <fftw3.h>
#include "common/timer.h"
#include "common/debug.h"
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
using namespace std;
using namespace Eigen;

class CorrelationFlow
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CorrelationFlow(ros::NodeHandle);

    void callback(const sensor_msgs::ImageConstPtr&);

private:

    inline ArrayXXcf fft(const ArrayXXf&);

    inline ArrayXXf ifft(const ArrayXXcf&);

    inline ArrayXXcf gaussian_kernel();

    inline ArrayXXcf gaussian_kernel(const ArrayXXcf&);

    inline ArrayXXcf kernel_lp();

    inline ArrayXXcf kernel_lp(const ArrayXXcf&);

    inline ArrayXXf log_polar(const cv::Mat);

    inline float get_psr(const ArrayXXf&, ArrayXXf::Index, ArrayXXf::Index);

    inline void publish(const std_msgs::Header);

    inline void compute_trans(const ArrayXXf&);

    inline void compute_rs(const ArrayXXf&);

    inline void compute_velocity(double);


private:

    ros::NodeHandle nh;
    ros::Publisher pub_twist;
    ros::Publisher pub_vector;

    int width, height;
    float focal_x, focal_y;

    fftwf_plan fft_plan;

    float max_response;
    float max_response_rs;
    float lamda, rs_lamda;
    ArrayXXf::Index max_index[2];// index of max value
    ArrayXXf::Index max_index_rs[2]; //index for rotation/scale

    cv::Mat image;
    cv::Mat sample_cv;
    
    ArrayXXf  sample;
    ArrayXXf  sample_lp;
    ArrayXXcf sample_fft;      // key depth cloud
    ArrayXXcf filter_fft;      // key depth cloud
    ArrayXXcf filter_fft_rs;
    ArrayXXcf target_fft;      // correlation target
    ArrayXXf  output;          // correlation output
    ArrayXXf  output_rs;
    
    ArrayXXcf train_fft;
    ArrayXXcf train_fft_conj;

    ArrayXXcf train_lp_fft;
    ArrayXXcf train_lp_fft_conj;

    float train_square;
    float train_lp_square;

    float sigma, rs_sigma;
    ArrayXXcf kernel;
    ArrayXXcf xyf;
    ArrayXXf  xy;
    ArrayXXf  xxyy;

    bool initialized;
    Jeffsan::Timer timer;
    double ros_time;
    float lowpass_weight;
    Vector3d velocity;
    float yaw_rate;

    bool rs_switch;
};
#endif
