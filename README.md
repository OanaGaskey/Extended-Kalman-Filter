# Extended Kalman Filter

Sensor fusion algorithm using LiDAR and RADAR data to track pedestrian, predicting and updating dynamic state estimation.

![GIF](images/EKF.gif)


This project implements the extended Kalman Filter for tracking a pedestrian, but it could track any traffic participant in the same manner. The intention is to measure the pedestrian’s position and velocity.

Since we are only interested in 2D movement, the state variables are `px`,`py`,`vx`,`vy`. The sensors used for detecting the pedestrian are RADAR and Laser (LIDAR). The advantage of having multiple types of sensors which are fused is that you can get a higher performance. The Laser has a better position accuracy, but the RADAR measures the velocity directly by using the Doppler Effect. The input data for the algorithm is synthetic, meaning that there are not real measurements from real sensors.  
 

This project is implemented in C++ using the Eigen library. The source code is located in `FusionEKF.cpp` and `kalman_filter.cpp` files in the `src` folder above. 

The simulator for this project can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

To run the code, from the terminal use:
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

The starter code for this project is provided by Udacity and can be found [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)
At the same location you can find installation details 


## Implementation Details

### Calculate RMSE

Root Mean Squared Error is used to evaluate the performance of the algorithm. The pedestrian's estimated position using the Extended Kalman Filter is compared with the ground truth. The goal of the project is to get `px` , `py` RMSE below `.11` meters and `vx` , `vy` RMSE below `.52` meters per second.

After checking that the size of the estimations vector is not zero and that the estimations and the ground truth data matches in size, I loop over all estimations and subtract the ground truth values. 

Because I am using VectorXd from Eigen library, vectors can be subtracted directly. 

```
// accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // calculate residuals
    r = estimations[i]-ground_truth[i];
    r = r.array()*r.array();
    //cout << r << endl;
    sqr_residuals.push_back(r);
  }  

```

Now that we have the error vector for each cycle we can go ahead and raise to the power of 2. All these squared residuals are kept into: `vector <VectorXd> sqr_residuals; `

```
  // calculate the mean
  for (int i=0; i < sqr_residuals.size(); ++i) {
    rmse = rmse + sqr_residuals[i];
  }
  rmse = rmse.array()/sqr_residuals.size();
  // calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
```

This is a convenient format because now I can sum them up and divide by the number of cycles to calculate the mean and finally to extract the squared root. This provides a four elements vector that is the result the function returns. 


### Calculate Jacobian

The Jacobian matrix here is calculated to get the first partial derivative of the h function’s variables with respect of the space state variables. The equations are provided below:

![jacobian](images/jacobian.JPG)

The Jacobian matrix needs to be calculated at each cycle because the space state vector changes and the derivatives need to be calculated in different points. 

```
MatrixXd Hj(3,4);
// recover state parameters
float px = x_state(0);
float py = x_state(1);
float vx = x_state(2);
float vy = x_state(3);
float pxysqr = powf(px, 2) + powf(py, 2);
float pxysqrrt =  hypotf(px, py);

// check division by zero
if (fabs(pxysqr) < 0.0001) {
  cout << "CalculateJacobian () - Error - Division by Zero" << endl;
  return Hj;
}
// compute the Jacobian matrix  
Hj << px/pxysqrrt, py/pxysqrrt, 0, 0,
      -py/pxysqr, px/pxysqr, 0, 0,
      py*(vx*py-vy*px)/powf(pxysqrrt,3), px*(vy*px-vx*py)/powf(pxysqrrt,3), px/pxysqrrt, py/pxysqrrt;

return Hj; 
```

This is later to be used for the Extended aspect of the Kalman Filter which represents the fact that we need to linearize non-linear functions by taking the tangent in the respective point so transformation can be applied and the Gaussian covariance is kept. 

For this implementation I chose to calculate 𝑝 + 𝑝  and 𝑝 + 𝑝  first and then process each element of the matrix. The function returns the Jacobian matrix only if division by zero can be avoided. 
