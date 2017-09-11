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

#include <math.h>
#include <ros/ros.h>
#include "correlation_flow/correlation_flow.h"


CorrelationFlow::CorrelationFlow(ros::NodeHandle nh):nh(nh)
{
    if(!nh.getParam("image_width", width)) ROS_ERROR("Can't get Param image_width");
    if(!nh.getParam("image_height", height)) ROS_ERROR("Can't get Param image_height");
    if(!nh.getParam("focal_x", focal_x)) ROS_ERROR("Can't get Param focal_x");
    if(!nh.getParam("focal_y", focal_y)) ROS_ERROR("Can't get Param focal_y");

    velocity = Vector3d::Zero();
    lowpass_weight = 0.10;
    if(nh.getParam("lowpass_weight", lowpass_weight))
        ROS_WARN("Get lowpass_weight:%f", lowpass_weight);

    lamda = 0.1;if(nh.getParam("trans/lamda", lamda)) ROS_WARN("Get trans/lamda:%f", lamda);
    sigma = 0.2;if(nh.getParam("trans/sigma", sigma)) ROS_WARN("Get trans/sigma:%f", sigma);

    rs_lamda = 0.001;if(nh.getParam("rs/lamda", rs_lamda)) ROS_WARN("Get rs/lamda:%f", rs_lamda);
    rs_sigma = 0.2;if(nh.getParam("rs/sigma", rs_sigma)) ROS_WARN("Get rs/sigma:%f", rs_sigma);

    yaw_rate = 0;
    rs_switch = true;
    if(nh.getParam("rs_switch", rs_switch))
        ROS_WARN("calculate rotation and scale: %s", rs_switch?"true":"false");    

    ArrayXXf target = ArrayXXf::Zero(width, height);
    target(width/2, height/2) = 1;
    target_fft = fft(target);
    filter_fft = fft(ArrayXXf::Zero(width, height));
    filter_fft_rs = fft(ArrayXXf::Zero(width, height));

    initialized = false;

    pub_twist = nh.advertise<geometry_msgs::TwistStamped>("vision_speed/speed_twist", 1);
    pub_vector = nh.advertise<geometry_msgs::Vector3Stamped>("vision_speed/speed_vector", 1);
}


void CorrelationFlow::callback(const sensor_msgs::ImageConstPtr& msg)
{
    timer.tic();
    image = cv_bridge::toCvShare(msg, "mono8")->image;
    image(cv::Rect((image.cols-width)/2, (image.rows-height)/2, width, height)).convertTo(sample_cv, CV_32FC1, 1/255.0);
    
    sample = Eigen::Map<ArrayXXf>(&sample_cv.at<float>(0,0), width, height);
    
    sample_lp = log_polar(sample_cv);

    if (initialized == false)
    {   
        train_fft = fft(sample);
        kernel = gaussian_kernel();
        filter_fft = target_fft/(kernel + lamda);

        train_lp_fft = fft(sample_lp);
        kernel = kernel_lp();
        filter_fft_rs = target_fft/(kernel + rs_lamda);

        initialized = true;
        ros_time = msg->header.stamp.toSec();
        ROS_WARN("initialized.");
        return;
    }

    // update ROS TIME
    double dt = msg->header.stamp.toSec() - ros_time;

    ros_time = msg->header.stamp.toSec();

    compute_trans(sample);

    if (rs_switch == true)
        compute_rs(sample_lp);

    compute_velocity(dt);

    float trans_psr = get_psr(output, max_index[0], max_index[1]);

    float rs_psr;
    if (rs_switch == true)
        rs_psr = get_psr(output_rs, max_index_rs[0], max_index_rs[1]);
    else
        rs_psr = 0;

    publish(msg->header);

    timer.toc("callback:");

    ROS_WARN("vx=%+.4f, vy=%+.4f, vz=%+.7f m/s, wz=%+.7f degree/s with psr: %.1f rs_psr: %.1f", 
        velocity(0), velocity(1), velocity(2), yaw_rate, trans_psr, rs_psr);
}


inline void CorrelationFlow::publish(const std_msgs::Header header)
{
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = header.stamp;
    twist.twist.angular.z = yaw_rate;
    tf::vectorEigenToMsg(velocity, twist.twist.linear);
    pub_twist.publish(twist);

    geometry_msgs::Vector3Stamped vector;
    vector.header.stamp = header.stamp;
    tf::vectorEigenToMsg(velocity, vector.vector);
    pub_vector.publish(vector);
}



inline ArrayXXcf CorrelationFlow::fft(const ArrayXXf& x)
{
    ArrayXXcf xf = ArrayXXcf(x.rows()/2+1, x.cols());

    fft_plan = fftwf_plan_dft_r2c_2d(x.cols(), x.rows(), (float(*))(x.data()), 
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major
    
    fftwf_execute(fft_plan);
    
    return xf;
}


inline ArrayXXf CorrelationFlow::ifft(const ArrayXXcf& xf)
{
    ArrayXXf x = ArrayXXf((xf.rows()-1)*2, xf.cols());
    
    ArrayXXcf cxf = xf;
    
    fft_plan = fftwf_plan_dft_c2r_2d(xf.cols(), (xf.rows()-1)*2, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    
    fftwf_execute(fft_plan);
    
    return x/x.size();
}


inline ArrayXXcf CorrelationFlow::gaussian_kernel()
{
    unsigned int N = height * width;

    train_square = train_fft.square().abs().sum()/N; // Parseval's Theorem

    float xx = train_square;

    float yy = train_square;

    train_fft_conj = train_fft.conjugate();
    
    xyf = train_fft * train_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline ArrayXXcf CorrelationFlow::gaussian_kernel(const ArrayXXcf& xf)
{
    unsigned int N = height * width;

    float xx = xf.square().abs().sum()/N; // Parseval's Theorem

    float yy = train_square;
    
    xyf = xf * train_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline ArrayXXcf CorrelationFlow::kernel_lp()
{
    unsigned int N = height * width;

    train_lp_square = train_lp_fft.square().abs().sum()/N; // Parseval's Theorem

    float xx = train_lp_square;

    float yy = train_lp_square;

    train_lp_fft_conj = train_lp_fft.conjugate();
    
    xyf = train_lp_fft * train_lp_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(rs_sigma*rs_sigma)*xxyy).exp());
}


inline ArrayXXcf CorrelationFlow::kernel_lp(const ArrayXXcf& xf)
{
    unsigned int N = height * width;

    float xx = xf.square().abs().sum()/N; // Parseval's Theorem

    float yy = train_lp_square;
    
    xyf = xf * train_lp_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(rs_sigma*rs_sigma)*xxyy).exp());
}


inline ArrayXXf CorrelationFlow::log_polar(const cv::Mat img)
{
    cv::Mat log_polar_img;

    cv::Point2f center((float)img.cols/2, (float)img.rows/2);

    double radius = (double)img.rows / 2;

    double M = (double)img.cols / log(radius);

    cv::logPolar(img, log_polar_img, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

    return Eigen::Map<ArrayXXf>(&log_polar_img.at<float>(0,0), img.cols, img.rows);
}


inline float CorrelationFlow::get_psr(const ArrayXXf& output, ArrayXXf::Index x, ArrayXXf::Index y)
{
    float side_lobe_mean = (output.sum()-max_response)/(output.size()-1);

    float std  = sqrt((output-side_lobe_mean).square().mean());

    return (max_response - side_lobe_mean)/std;
}


inline void CorrelationFlow::compute_trans(const ArrayXXf& xf)
{
    sample_fft = fft(xf);

    // correlation response of current frame
    kernel = gaussian_kernel(sample_fft);
    output = ifft(filter_fft*kernel);
    max_response = output.maxCoeff(&(max_index[0]), &(max_index[1]));
    
    // update filter
    train_fft = sample_fft;
    kernel = gaussian_kernel();
    filter_fft = target_fft/(kernel + lamda);
}


inline void CorrelationFlow::compute_rs(const ArrayXXf& xf)
{
    sample_fft = fft(xf);

    // correlation response of current frame
    kernel = kernel_lp(sample_fft);
    output_rs = ifft(filter_fft_rs*kernel);
    max_response_rs = output_rs.maxCoeff(&(max_index_rs[0]), &(max_index_rs[1]));
    
    // printf("%d %d\n", max_index_rs[0], max_index_rs[1]);
    // show_image(output_rs/max_response_rs,height,width,"rs_output");
    // cv::waitKey(1);
    // update filter
    train_lp_fft = sample_fft;
    kernel = kernel_lp();
    filter_fft_rs = target_fft/(kernel + rs_lamda);
}


inline void CorrelationFlow::compute_velocity(double dt)
{

    if(dt<1e-5) {ROS_WARN("image msg time stamp is INVALID, set dt=0.03s"); dt=0.03;}

    // veclocity calculation
    float vx = -1.0*((max_index[0]-width/2)/dt)/focal_x;
    float vy = -1.0*((max_index[1]-height/2)/dt)/focal_y;

    float vz = 0;
    if (rs_switch == true)
    {
        double radius = (double)height / 2;
        double M = (double)width / log(radius);
        float scale = exp((max_index_rs[0]-width/2)/M);
        vz = (scale-1)/dt;

        float rotation = (max_index_rs[1]-height/2)*360.0/height;
        yaw_rate = (rotation*M_PI/180.0)/dt;
    }
    
    // printf("scale=%f\n",scale);
    // printf("rotation=%f\n",rotation);

    Vector3d v = Vector3d(vx, vy, vz);
    velocity = lowpass_weight * v + (1-lowpass_weight) * velocity; // low pass filter
}
