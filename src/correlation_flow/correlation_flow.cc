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

string filename;
CorrelationFlow::CorrelationFlow(ros::NodeHandle nh):nh(nh)
{
    lowpass_w = 0.10;
    vx_prev = 0;
    vy_prev = 0;

    nh.getParam("image_width", width);
    nh.getParam("image_height", height);
    nh.getParam("focal_x", focal_x);
    nh.getParam("focal_y", focal_y);
    // width = 320;
    // height = 240;
    lamda = 0.1;
    sigma = 0.2;

    ArrayXXf target = ArrayXXf::Zero(width, height);
    target(width/2, height/2) = 1;
    target_fft = fft(target);
    filter_fft = fft(ArrayXXf::Zero(width, height));

    // rot_resolution = 60.0;
    // target_dim = 360 / rot_resolution;
    // ArrayXXf target_rot = ArrayXXf::Zero(target_dim, 1);
    // target_rot(target_dim/2, 0) = 1;
    // target_rot_fft = fft(target_rot);
    // filter_rot_fft = fft(ArrayXXf::Zero(target_dim, 1));

    // max_level = 0.1;
    // scale_factor = 0.05;
    // sca_target_dim = 2 * max_level / scale_factor+1;
    // ArrayXXf target_sca = ArrayXXf::Zero(sca_target_dim, 1);
    // target_sca(1, 0) = 1;
    // target_sca_fft = fft(target_sca);
    // filter_sca_fft = fft(ArrayXXf::Zero(sca_target_dim, 1));

    initialized = false;

    pub = nh.advertise<geometry_msgs::TwistStamped>("/corr_flow", 1000);

    // filename = "/home/jitete/drones/src/correlation_flow/results/cf1_t.txt";

    // file.open(filename, ios::trunc|ios::out);
    // file.close();
}

// void CorrelationFlow::callback_imu(const sensor_msgs::Imu& msg_imu)
// {
//     q.w() = msg_imu.orientation.w;
//     q.x() = msg_imu.orientation.x;
//     q.y() = msg_imu.orientation.y;
//     q.z() = msg_imu.orientation.z;
// }

void CorrelationFlow::callback(const sensor_msgs::ImageConstPtr& msg)
{
    timer.tic();

    image = cv_bridge::toCvCopy(msg, "mono8")->image;
    // cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    imgROI = cv::Rect((image.cols-width)/2, (image.rows-height)/2, width, height);
    cropImg = image(imgROI);
    cropImg.convertTo(cropImg, CV_32FC1);

    sample = Eigen::Map<ArrayXXf>(&cropImg.at<float>(0,0), width, height)/255.0;
    sample_fft = fft(sample);

    std_msgs::Header h = msg->header;
    t_now = h.stamp.toSec();

    if (initialized == false)
    {   
        train = sample;
        train_fft = sample_fft;
        kernel = gaussian_kernel();
        filter_fft = target_fft/(kernel + lamda);

        // rotation_base(image);
        // kernel_rot = rotation_kernel(train);
        // filter_rot_fft = target_rot_fft/(kernel_rot + lamda);

        // scale_base(image);
        // kernel_sca = scale_kernel(train);
        // filter_sca_fft = target_sca_fft/(kernel_sca + lamda);

        t_prev = t_now;

        initialized = true;
        printf("initialized.\n");

        return;
    }

    // motion of current frame
    kernel = gaussian_kernel(sample_fft);
    output = ifft(filter_fft*kernel);
    max_response = output.maxCoeff(&(max_index[0]), &(max_index[1]));

    // kernel_rot = rotation_kernel(sample);
    // output_rot = ifft(kernel_rot*filter_rot_fft);
    // max_responseR = output_rot.maxCoeff(&(max_indexR[0]), &(max_indexR[1]));

    // kernel_sca = scale_kernel(sample);
    // output_sca = ifft(kernel_sca*filter_sca_fft);
    // max_responseS = output_sca.maxCoeff(&(max_indexS[0]), &(max_indexS[1]));

    float trans_psr = get_psr(output, max_index[0], max_index[1]);
    // float rot_psr = get_psr(output_rot, max_indexR[0], max_indexR[1]);
    
    // update filter
    train = sample;
    train_fft = sample_fft;
    kernel = gaussian_kernel();
    filter_fft = target_fft/(kernel + lamda);

    // rotation_base(image);
    // kernel_rot = rotation_kernel(train);
    // filter_rot_fft = target_rot_fft/(kernel_rot + lamda);

    // scale_base(image);
    // kernel_sca = scale_kernel(train);
    // filter_sca_fft = target_sca_fft/(kernel_sca + lamda);

    // compute vx, vy, vz, wz
    double vx, vy;
    double delt_t;
    delt_t = t_now - t_prev;

    // for Microsoft camera, use fx=572.44 fy=572.89 z=0.86 facing down
    // for another camera, use fx=605.65 fy=609.22 z=1.78 facing up
	vx = -1.0*((max_index[0]-width/2)/delt_t)/focal_x;
	vy = -1.0*((max_index[1]-height/2)/delt_t)/focal_y;
    
    // rotation = (max_indexR[0]-target_dim/2)*rot_resolution;
    // wz = (rotation*M_PI/180.0)/delt_t;

    // low pass filter
    vx = lowpass_w*vx + (1-lowpass_w)*vx_prev;
    vy = lowpass_w*vy + (1-lowpass_w)*vy_prev;

    geometry_msgs::TwistStamped vmsg;
    vmsg.header.stamp = h.stamp;
    vmsg.twist.linear.x = vx;
    vmsg.twist.linear.y = vy;
    vmsg.twist.linear.z = 0;
    pub.publish(vmsg);

    vx_prev = vx;
    vy_prev = vy;

    t_prev = t_now;
    // save_file(vmsg, filename);


    timer.toc("callback:");

    ROS_WARN("fx=%f, fy=%f m/s with psr: %f", vx, vy, trans_psr);
    // ROS_WARN("angle rate is %f degree/s with psr: %f", wz, rot_psr);
    // ROS_WARN("index, %d scaling factor is %f\n", max_indexS[0], 1-max_level+max_indexS[0]*scale_factor);
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

    train_square = train.square().abs().sum();

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

    float xx = ifft(xf).square().abs().sum();

    float yy = train_square;
    
    xyf = xf * train_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


// inline void CorrelationFlow::rotation_base(const cv::Mat& img)
// {
//     rot_base.clear();
//     rot_base.push_back(sample);
//     cv::Mat rot_mat(2, 3, CV_32FC1);
//     cv::Mat img_rot;
//     double angle;

//     for (int i=1; i<target_dim; i++)
//     {
//         angle = i*rot_resolution;
//         rot_mat = cv::getRotationMatrix2D(cv::Point(img.cols/2, img.rows/2), angle, 1.0);
//         cv::warpAffine(img, img_rot, rot_mat, img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 127);
//         img_rot = img_rot(imgROI);
//         img_rot.convertTo(img_rot, CV_32FC1);
//         basis = Eigen::Map<ArrayXXf>(&img_rot.at<float>(0,0), width, height)/255.0;
//         rot_base.push_back(basis);
//     }
// }


// inline ArrayXXcf CorrelationFlow::rotation_kernel(const ArrayXXf& arr0)
// {
//     unsigned int N = height * width;
    
//     ArrayXXf ker = ArrayXXf::Zero(target_dim, 1);

//     for(int i=0; i<target_dim; i++)
//     {
//         float diff_square = (arr0 - rot_base.at(i)).square().abs().sum()/N/N;

//         ker(i, 0) = exp(-1/(sigma*sigma)*diff_square);
//     }
    
//     return fft(ker);
// }


// inline void CorrelationFlow::scale_base(const cv::Mat& imgs)
// {
//     sca_base.clear();
//     cv::Mat sca_mat(2, 3, CV_32FC1);
//     cv::Mat img_scale;

//     for (int i = 0; i<sca_target_dim; i++)
//     {
//         sca_mat = cv::getRotationMatrix2D(cv::Point(imgs.cols/2, imgs.rows/2), 0, 1-max_level + i*scale_factor);
//         cv::warpAffine(imgs, img_scale, sca_mat, imgs.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 127);
//         img_scale = img_scale(imgROI);
//         img_scale.convertTo(img_scale, CV_32FC1);
//         basis_s = Eigen::Map<ArrayXXf>(&img_scale.at<float>(0,0), width, height)/255.0;
//         sca_base.push_back(basis_s);
//     }
// }


// inline ArrayXXcf CorrelationFlow::scale_kernel(const ArrayXXf& arr1)
// {
//     unsigned int N = height * width;
    
//     ArrayXXf ker = ArrayXXf::Zero(sca_target_dim, 1);

//     for(int i=0; i<sca_target_dim; i++)
//     {

//         float diff_square = (arr1 - sca_base.at(i)).square().abs().sum()/N/N;

//         ker(i, 0) = exp(-1/(sigma*sigma)*diff_square);

//     }

//     return fft(ker);
// }


inline float CorrelationFlow::get_psr(const ArrayXXf& output, ArrayXXf::Index x, ArrayXXf::Index y)
{
    float max_output = output(x, y);

    float side_lobe_mean = (output.sum()-max_output)/(output.size()-1);

    float std  = sqrt((output-side_lobe_mean).square().mean());

    return (max_response - side_lobe_mean)/std;
}


inline void CorrelationFlow::save_file(geometry_msgs::TwistStamped twist, string filename)
{
    file.open(filename.c_str(), ios::app);
    file<<boost::format("%.9f") % (twist.header.stamp.toSec())<<" "
        <<twist.twist.linear.x<<" "
        <<twist.twist.linear.y<<" "
        <<0<<" "
        <<0<<" "
        <<0<<" "
        <<0<<" "
        <<0<<endl;
    file.close();
}
