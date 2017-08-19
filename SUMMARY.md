# Model Documentation

[//]: # (Image References)

[lane1]: ./results/lane-estimation.png "Lane Estimation"

We are feeding back to the simulator always `MAX_POINTS` (= 50) points. Each time the path_planner get's control (`onMessage` function) we look for the previously generated trajectory and stored further points of the trajectory. If we have stored points we safely do either 1) use this stored points to add up for the full `MAX_POINTS` of 2) generate new trajectory from the current state (actually end_path state) and use this new trajectory.

By default we are always generating new trajectory in that way car more reactive to the change in the road situation. But in one case where we are at the 'end of the world' (finishing the loop and s changes from ~6490 to 0) we are feeding the previous trajectory and avoid generation of the new one until car_s and end_path_s cross the `end of the world` point.

## Trajectory Generation

Trajectory generation always starts from the `end_path_s` point. We also can use the predicted point from past generated trajectory in the current time. Due to not linear changes of `end_path_s` we found that it's safer to use predicted value `curr_s` combined with not linear `end_path_s` with a coefficient `0.5`. Such smoothing helps to avoid stretching in the trajectory (that leads to exceeding jerk) due to not linear nature of transformation from Frenet to XY coordinates.

For trajectory generation we also need the initial speed and acceleration which we are calculating from previous trajectory as first and second derivative at the point of `curr_time` (which is end of path).

Thus we have initial state for trajectory generation `s_start` and `d_start` that we can use in `JMT` procedure.

To generate trajectory we also need to know the final values of `s` and `d` and time `T` which are determining from the `targetSpeed` and `targetLane` params.

`targetSpeed` is always `MAX_SPEED` in case there is no car's ahead, or speed of the forward car, or a fraction of the speed of the forward car if it is between a current car position and end of path or closer than `3 * CAR_LENGTH` from the end of path.

`targetLane` is determined by comparing the availability and speed of traffic among adjacent lanes.

End `d` is always defined by `targetLane` with speed and acceleration is `0`.

End `s` is calculated as average speed (`avg_v`) traveled over time which is lower of `delta_v / 4.5` or `3.0` seconds. In other words for trajectories that need speed increase less than `13.5` m/s we are using T = 3 seconds by default and for others we do not want to accelerate faster than `5.0` m/s.

See `genTraj()` function for details.


## Lane estimation

By looking to adjacent lanes we want to know that they are safe to make a lane change. Safe distance is considering as `1 x CAR_LENGTH` ahead of end path and `3 x CAR_LENGTH` behind our car. Lane estimation also returns the speed for the estimated lane which can be `SPEED_LIMIT` if there are no cars or speed of the closest car.

![Lane Estimation][lane1]


## Trajectory connection and smoothing

To make a smooth trajectory we are using previous path points (up to 3 points) and new trajectory (up to 3 points) which is used to build splines `splX(t)` and `splY(t)` which is then used to generate the XY points of the trajectory. Final trajectory is smooth and connected to previous points.

Additional squeezing required when the trajectory stretched over curved parts of the road and the final travel distance in XY coordinates becomes longer than in SD coordinates which leads to the higher velocity than was planned by `JMT`.

## Video of the final driving

[![Driving Result](https://img.youtube.com/vi/I7l9d_QbXkI/0.jpg)](https://youtu.be/I7l9d_QbXkI)


## What can be done better

1) Better smoothing of the path and translation between SD and XY coordinates.
2) Add prediction for other cars states and compare it with a generated trajectory.
3) Generate more trajectories and add cost functions to evaluate them.
4) Check distance to the cars on the left and right sides and keep distance from them. (not just follow the middle of the lane)
