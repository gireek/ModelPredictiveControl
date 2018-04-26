# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Reflection

* The simulator provides the following data as an input to the program:

    ptsx (Array) - The global x positions of the waypoints.
    ptsy (Array) - The global y positions of the waypoints.
    psi (float) - The orientation of the vehicle in radians
    x (float) - The global x position of the vehicle.
    y (float) - The global y position of the vehicle.
    steering_angle (float) - The current steering angle in radians.
    throttle (float) - The current throttle value [-1, 1].

* The coordinates sent by the simulator are global coordinates of the map used by the simulator and they must be transformed to the vehicle's coordinates for computation of the error costs.

	for (int i = 0; i < ptsx.size(); i++){
            double x_ = ptsx[i] - px;
            double y_ = ptsy[i] - py;
            ptsx_t[i] = x_ * cos(-psi) - y_ * sin(-psi);
            ptsy_t[i] = x_ * sin(-psi) + y_ * cos(-psi);
	}

The polyfit() function is used to obtain the coefficients of a polynomial curve of degree 3 that best fits the waypoints. This is required trajectory for vehicle to stay on track. CTE is the derivative of the polynomial and the psi error is the - arctan of the derivative of the polynomial.

* Latency

A 100 ms latency is introduced in the program. The following equations describe the predicted latency state:

	  double lat = 0.100;
          double lat_px = v * lat;
          double lat_py = 0.0;
          double lat_psi = v * -delta / Lf * lat;
          double lat_v = v + a * lat;
          double lat_cte = cte + v * sin(epsi) * lat;
          double lat_epsi = epsi + v * -delta / Lf * lat;

* The cost function 

This part of the project was more interesting than all the past 4 projects in Term 2 combined. The intuition behind the one single factor that made my car to go at speeds of 90 mph smoothly and very elegantly slow down at curves was the addition of the factor of product of velocity and delta at a specific position. This made velocity less when curve was sharp and and velocity more otherwise and since my ref_v which is considered as standard by another cost function made the car run fast at other instances. My ref_v was finally set at 105. Other parameters I have taken are very crazy numbers ranging from 2 to 12000 and they have been reached after numerous tries of understanding giving importance to which part in the cost will give the desired behaviour. This is what I would state as the meat of this project and is given as follows:

    for (int t = 0; t < N; t++) {
      fg[0] += 12000 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 10000 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 2 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    for (int t = 0; t < N - 1; t++) {
      fg[0] += 20 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 20 * CppAD::pow(vars[a_start + t], 2);
      fg[0] += 1700 * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);     // The game changing code line
    }

    for (int t = 0; t < N - 2; t++) {
      fg[0] += 2000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

* N and dt

Initially I thought that long duration will givee better results. I selected various combinations on (N, dt) such as (25, 0.05), (20,0.1) , (30 , 0.03), (20 , 0.03) and so on. Greater N is causing the model to slow down, while reducing dt was causing the model to preform worse. Eventually the optimal I could get was by using (N=20, dt=0.03) which essentially was a prediction for 0.6 second.

* Conclusion

The final model was able to drive the vehicle correctly in the simulator with speed reaching over 90 miles an hour. The ride was smooth and there were no sudden jerks or turns. The vehicle always stayed on track.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
