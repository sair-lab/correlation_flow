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
}


void CorrelationFlow::callback(const sensor_msgs::ImageConstPtr& msg)
{
	timer.tic();

	cv::resize(cv_bridge::toCvShare(msg, "bgr8")->image, image, cv::Size(width, height));
	cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
	image.convertTo(sample_cv, CV_32FC1);
    sample = Eigen::Map<ArrayXXf>(&sample_cv.at<float>(0,0), width, height)/255.0;

    sample_fft = fft(sample);
    output = ifft(filter_fft*sample_fft);
    filter_fft = target_fft/(sample_fft + lamda);
    max_response = output.maxCoeff(&(max_index[0]), &(max_index[1]));

    timer.toc("callback:");
    ROS_WARN("x=%d, y=%d", max_index[0] - width/2,  max_index[1] - height/2);
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

    sample_square = sample_fft.square().abs().sum()/N;

    float xx = sample_square;

    float yy = sample_square;

    sample_fft_conj = sample_fft.conjugate();
    
    xyf = sample_fft * sample_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}


inline ArrayXXcf CorrelationFlow::gaussian_kernel(const ArrayXXcf& xf)
{
    unsigned int N = height * width;

    float xx = xf.square().abs().sum()/N;

    float yy = sample_square;
    
    xyf = xf * sample_fft_conj;
    
    xy = ifft(xyf);

    xxyy = (xx+yy-2*xy)/N;

    return fft((-1/(sigma*sigma)*xxyy).exp());
}
