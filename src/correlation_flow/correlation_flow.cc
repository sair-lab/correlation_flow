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

#include <ros/ros.h>
#include "correlation_flow/correlation_flow.h"


CorrelationFlow::CorrelationFlow(ros::NodeHandle nh):nh(nh)
{
	width = 360;
	height = 240;
	lamda = 0.1;
    sigma = 0.2;

	ArrayXXf target = ArrayXXf::Zero(width, height);
    target(width/2, height/2) = 1;
    target_fft = fft(target);
    filter_fft = fft(ArrayXXf::Zero(width, height));

    max_rotation = 10.0;
    rot_resolution = 1.0;
    target_dim = 2*int(max_rotation/rot_resolution)+1;
    ArrayXXf target_rot = ArrayXXf::Zero(target_dim, 1);
    target_rot(target_dim/2, 1) = 1;
    target_rot_fft = fft(target_rot);
    filter_rot_fft = fft(ArrayXXf::Zero(target_dim, 1));

    initialized = false;
}


void CorrelationFlow::callback(const sensor_msgs::ImageConstPtr& msg)
{
	timer.tic();

	cv::resize(cv_bridge::toCvShare(msg, "bgr8")->image, image, cv::Size(width, height));
	cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
	image.convertTo(sample_cv, CV_32FC1);
    sample = Eigen::Map<ArrayXXf>(&sample_cv.at<float>(0,0), width, height)/255.0;
    sample_fft = fft(sample);


    if (initialized == false)
    {
        train_fft = sample_fft;
        kernel = gaussian_kernel();
        filter_fft = target_fft/(kernel + lamda);

        rotation_base(sample_cv);
        kernel_rot = rotation_kernel(sample);
        filter_rot_fft = target_rot_fft/(kernel_rot + lamda);

        initialized = true;
        return;
    }

    //motion of current frame
    kernel = gaussian_kernel(sample_fft);
    output = ifft(filter_fft*kernel);
    max_response = output.maxCoeff(&(max_index[0]), &(max_index[1]));

    kernel_rot = rotation_kernel(sample);
    output_rot = ifft(kernel_rot*filter_rot_fft);
    max_responseR = output_rot.maxCoeff(&(max_indexR[0]), &(max_indexR[1]));

    //update filter
    train_fft = sample_fft;
    kernel = gaussian_kernel();
    filter_fft = target_fft/(kernel + lamda);

    rotation_base(sample_cv);
    kernel_rot = rotation_kernel(sample);
    filter_rot_fft = target_rot_fft/(kernel_rot + lamda);

    timer.toc("callback:");
    ROS_WARN("x=%d, y=%d\n", int(max_index[0] - width/2), int(max_index[1] - height/2));
    ROS_WARN("rotaion angle is %f\n", (max_indexR[0]-target_dim/2)*rot_resolution);
}


inline ArrayXXcf CorrelationFlow::fft(const ArrayXXf& x)
{
    ArrayXXcf xf = ArrayXXcf(width/2+1, height);

    fft_plan=fftwf_plan_dft_r2c_2d(height, width, (float(*))(x.data()), 
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major
    
    fftwf_execute(fft_plan);
    
    return xf;
}


inline ArrayXXf CorrelationFlow::ifft(const ArrayXXcf& xf)
{
    ArrayXXf x = ArrayXXf(width, height);
    
    ArrayXXcf cxf = xf;
    
    fft_plan=fftwf_plan_dft_c2r_2d(height, width, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    
    fftwf_execute(fft_plan);
    
    return x/x.size();
}


inline ArrayXXcf CorrelationFlow::gaussian_kernel()
{
    unsigned int N = height * width;

    train_square = train_fft.square().abs().sum()/N;

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

    float xx = xf.square().abs().sum()/N;

    float yy = train_square;
    
    xyf = xf * train_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline void CorrelationFlow::rotation_base(const cv::Mat& img)
{
    rot_base.clear();

    cv::Mat rot_mat(2, 3, CV_32FC1);
    cv::Mat img_rot;
    cv::Point center = cv::Point(img.cols/2, img.rows/2);
    double scale = 1.0;
    double angle = -1.0*int(max_rotation/rot_resolution)*rot_resolution;
    
    for (int i=0; i<target_dim; i++)
    {
        if (i==int(target_dim/2))
        {
            rot_base.push_back(sample);
        }
        else
        {
            angle = angle + rot_resolution;
            rot_mat = cv::getRotationMatrix2D(center, angle, scale);
            cv::warpAffine(img, img_rot, rot_mat, img.size());
            basis = Eigen::Map<ArrayXXf>(&img_rot.at<float>(0,0), width, height)/255.0;
            rot_base.push_back(basis);
        }
    }

    //return;
}


inline ArrayXXcf CorrelationFlow::rotation_kernel(const ArrayXXf& arr0)
{
    //unsigned int N = height * width;
    ArrayXXf ker = ArrayXXf::Zero(target_dim, 1);

    for(int i=0; i<target_dim; i++)
    {
        ArrayXXf diff = arr0 - rot_base.at(i);

        ker(i, 0) = -1/(sigma*sigma)*(diff.square().abs().sum());
    }

    return fft(ker.exp());
}

/*
inline double CorrelationFlow::rotation_estimation(const ArrayXXcf& arr1)
{
    kernel_rot = rotation_kernel(arr1);

    output_rot = ifft(kernel_rot*filter_rot_fft);


}
*/