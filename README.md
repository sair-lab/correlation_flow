# correlation_flow
Correlation Flow: Velocity Estimation in 3-D space $v_x, v_y, v_z, \omega_x$
 
# Platform
  Codes have been tested on Ubuntu 16.04 with ROS kinetic.
  
# Install Dependencies:
  1. Install FFT library: 
	```
	sudo apt-get install libfftw3-dev libfftw3-doc
	```

# Install Intel Math Kernel Library (MKL) （optional）:

  1. Download MKL from Intel website
  2. Extract downloaded file 
  	```
  	tar -zxvf [name_of_downloaded_file]
  	```
  3. Go to extracted folder, give permission: 
  	```
  	sudo chmod +x install.sh
  	```
  4. Run installation 
	```
  	./install.sh
  	```
  5. Link library, add to .bashrc: 
  	```
  	source /opt/intel/bin/compilervars.sh intel64
  	```
  6. Try compile in ROS workspace

# Papers associated with this work:
#### Chen Wang, Tete Ji, Thien-Minh Nguyen, and Lihua Xie, "Correlation Flow: Robust Optical Flow using Kernel Cross-Correlators"

For more details about kernel cross-correlator, refer to:

https://github.com/wang-chen/correlation_flow/blob/master/doc/kcc.pdf
