## Localization with the Particle Filter
---

**Kidnapped Vehicle Localization Project**

The goals / steps of this project are the following:

* Implement a particle filter to estimate the location of a moving vehicle given an *a priori* map of known landmarks and a stream of noisy measurements of those landmarks.

[//]: # "Image References"
[image1]: ./images/rmse.png
## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
**1. Accuracy.**

- Done.  See figure above.
- RMSE tracking errors are [X : 0.0686, Y : 0.0816, VX : 0.3549, VZ : 0.2345]

**2. Performance.**

- Done.  See figure above.
- System time for execution is 

**3. The Code Implements the Particle Filter**

* The particle filter algorithm is implemented in [./src/particle_filter.cpp](./src/particle_filter.cpp).

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  

* The UKF works well and handles the nonlinear radar measurement function and bicycle prediction functions well.
* Once the UKF was implemented, I did have to spend a few iterations finding the appropriate values for the prediction noise, and the state covariance matrix when initializing the state.  I did this by outputting each of the state variable values to obtain a rough feel for their magnitude and variation.  I treated them as independent random variables in the initialization.


