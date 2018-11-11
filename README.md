# Particle Filter Localization

![alt text][image1]

This project is in conjunction with the Udacity Self-Driving Car course.  In particular, the goal of this project is to use a simple particle filter to localize a moving vehicle using noisy measurements of known landmarks. 

* More detail about the approach is provided in [writeup.md](./writeup.md).

## Getting Started

### Prerequisites

```
CMake
uWS		(Socket for interfacing with Udacity simulator)	
```

### Installing

1. Clone this project from GitHub:

```
git clone https://github.com/jimwatt/P8-particlefilter.git
```

2. Change into the `build` directory, and compile with `CMake`.

   ```
   cd build
   cmake ..
   make
   ```

## Running the Code

* Run the executable from the `build` directory:
  ```./particle_filter```
* The executable will wait until measurements are provided by the simulator through the uWS socket.





## Authors

* **James Watt**

<!--## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details-->

## Acknowledgments
This project is a submission to the Udacity Self-Driving Car nanodegree:

* ```
  https://github.com/udacity/CarND-Kidnapped-Vehicle-Project.git
  ```

## Localization with the Particle Filter

------

**Kidnapped Vehicle Localization Project**

The goals / steps of this project are the following:

- Implement a particle filter to estimate the location of a moving vehicle given an *a priori* map of known landmarks and a stream of noisy measurements of those landmarks.

[//]: #	"Image References"
[image1]: ./particlefilter.png

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points

![alt text][image1]

**1. Accuracy.**

- Done.  See figure above.
- RMSE tracking errors are [X : 0.133, Y : 0.124, YAW : 0.004]

**2. Performance.**

- Done.  See figure above.
- System time for execution is 97.04 seconds.

**3. The Code Implements the Particle Filter**

- The particle filter algorithm is implemented in [./src/particle_filter.cpp](./src/particle_filter.cpp).
- The main functional components are *predict*, *update weights*, and *resample*.

------

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  

- The particle filter was straightforward to implement and worked well.  No major algorithmic improvements were required to improve the accuracy and execution times.
- Once the particle filter was implemented, I did try to improve runtime performance using several Nearest Neighbor libraries (`ANN`, `FLANN`,  and `NABO`, for example.)  These libraries index the map landmark locations into a kd-tree, allowing fast look up of nearest neighbors for a query point. Unfortunately, since the map did not have many landmarks (only 42), these approaches did not provide any appreciable acceleration.  In a very dense map, with hundreds of thousands of points, these approaches will be indispensable.
- An improvement to the current code would be to assign observations to landmarks using a 2D assignment solver to optimize global cost (using the Jonker-Volgenant-Castanon algorithm, for example) rather than simply choosing nearest neighbors.

