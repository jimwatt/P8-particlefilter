## Unscented Kalman Filter and Sensor Fusion
---

**Bicycle Tracking Project**

The goals / steps of this project are the following:

* Implement an Unscented Kalman Filter to fuse data from radar and laser sensors to track the state of a cyclist.

[//]: # "Image References"
[image1]: ./images/rmse.png
## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
**1. Code must compile without errors with `cmake` and `make`.**

Done.  Just try it.

**2. Tracking Accuracy.**

RMSE tracking errors are [X : 0.0686, Y : 0.0816, VX : 0.3549, VZ : 0.2345]

![alt text][image1]

**3. My Sensor Fusion algorithm follows the general processing flow as taught in the online lessons.**

* The Unscented Kalman filter algorithm is implemented in [./src/unscented_kalman_filter.cpp](./src/unscented_kalman_filter.cpp).

**4. The Kalman Filter algorithm handles the first measurements appropriately.**

* Very basic state initialization is implemented. I used linearizations of the respective measurement functions to map the sensor covariances to the state space.

**5. The Kalman Filter algorithm first predicts then updates.**

* That's what it does.

**6. The Kalman Filter can handle radar and lidar measurements.**

* Yup.

**7. The code is efficient.**

* For the most part, I followed the provided outline.  However, I did use a more _functional programming_ approach to the implementation of the Kalman Filter itself.  Notice that the Unscented Kalman filter defined  in unscented_kalman_filter.cpp is not an object and has no state.
* I prefer this approach because otherwise we have to know the "secret handshakes" to use the kalman filter object. 

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  

* The UKF works well and handles the nonlinear radar measurement function and bicycle prediction functions well.
* Once the UKF was implemented, I did have to spend a few iterations finding the appropriate values for the prediction noise, and the state covariance matrix when initializing the state.  I did this by outputting each of the state variable values to obtain a rough feel for their magnitude and variation.  I treated them as independent random variables in the initialization.


