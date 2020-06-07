# Hybrid A* Path Planning

This project implements Hybrid-A* Path Planning algorithm for a non-holonomic vehicle, which is inspired by this [Demo Video](https://www.youtube.com/watch?time_continue=2&v=qXZt-B7iUyw).

The Hybrid-A* algorithm is described here, [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf).

The code is ready to run a real Autonomous Vehicle with minor modifications, although it runs standalone as a demo in this repository.

## Algorithm Description

* A 3D discrete search space is used but unlike traditional A*, hybrid-A* associates with each grid cell a continuous 3D state of the vehicle. The resulting path is guaranteed to be drivable (standard A* can only produce piece-wise linear paths).
* The search algorithm is guided by two heuristics -
	* 'non-holonomic-without-obstacles' uses Dubin's path length ignoring obstacles
	* 'holonomic-with-obstacles' uses shortest path in 2D computed using A* (or Dijkstra's) ignoring holonomic constraints of vehicle
* To improve search speed, the algorithm analytically expands nodes closer to goal using dubins path and checks it for collision with current obstacle map.

## Parameters

- map
- initial
- goal
- velocity

## Images

<img src="https://imgur.com/wDC3stV.png" alt="Example1" width="400"/>             <img src="https://imgur.com/GZH6w0V.png" alt="Example2" width="400"/>

## Future Work

- Path Smoothing
- Reverse Motion
- User Interaction
- Viusalization Optimization
- Motion Control

## Resources

* [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
