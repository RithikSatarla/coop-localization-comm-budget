## Dataset summary

| dataset | robot | duration_s | landmark_obs | robot_obs | robot_obs_per_min | robot_range_median_m | robot_range_max_m | gt_dropout_s | odom_missing_s |
|---|---|---|---|---|---|---|---|---|---|
| 9 | 1 | 500 | 2019 | 563 | 67.56 | 2.2 | 4.73 | 8.8 | 0 |
| 9 | 2 | 500 | 2125 | 409 | 49.08 | 2.3 | 6.14 | 8.8 | 0 |
| 9 | 3 | 500 | 1955 | 432 | 51.84 | 2.14 | 7.23 | 22.5 | 0 |
| 9 | 4 | 500 | 791 | 172 | 20.64 | 2.58 | 4.99 | 9.5 | 0 |
| 9 | 5 | 500 | 2159 | 738 | 88.56 | 1.99 | 5.58 | 8.8 | 0 |

## Base methods: team position RMSE [m], mean over the five robots

| regime | dataset | Dead reckoning | EKF landmarks | EKF cooperative |
|---|---|---|---|---|
| full_map | 9 | 5.721 | 3.621 | 3.719 |
| map_blind | 9 | 5.721 | 5.337 | 5.41 |

## Base methods: per-robot position RMSE [m]

| regime | dataset | robot | blind | Dead reckoning | EKF landmarks | EKF cooperative |
|---|---|---|---|---|---|---|
| full_map | 9 | 1 | False | 6.264 | 6.819 | 6.783 |
| full_map | 9 | 2 | False | 4.203 | 1.731 | 4.296 |
| full_map | 9 | 3 | False | 5.767 | 1.083 | 1.071 |
| full_map | 9 | 4 | False | 7.859 | 4.101 | 4.1 |
| full_map | 9 | 5 | False | 4.51 | 4.369 | 2.347 |
| map_blind | 9 | 1 | False | 6.264 | 6.819 | 6.542 |
| map_blind | 9 | 2 | False | 4.203 | 1.731 | 3.464 |
| map_blind | 9 | 3 | True | 5.767 | 5.767 | 6.613 |
| map_blind | 9 | 4 | True | 7.859 | 7.859 | 7.29 |
| map_blind | 9 | 5 | True | 4.51 | 4.51 | 3.139 |

## Base methods: heading RMSE [rad], mean over robots

| regime | dataset | Dead reckoning | EKF landmarks | EKF cooperative |
|---|---|---|---|---|
| full_map | 9 | 1.613 | 1.142 | 1.129 |
| map_blind | 9 | 1.613 | 1.485 | 1.493 |

## Fusion ablation: covariance intersection (CI) vs naive inflation, unconstrained communication, team position RMSE [m]

| regime | dataset | method | rmse_xy | rmse_xy_max_robot | fused | gated | declined |
|---|---|---|---|---|---|---|---|
| full_map | 9 | EKF cooperative (naive) | 3.663 | 6.802 | 1440 | 874 | 0 |
| full_map | 9 | EKF cooperative (CI) | 3.719 | 6.783 | 445 | 502 | 1367 |
| map_blind | 9 | EKF cooperative (naive) | 5.238 | 6.93 | 1558 | 756 | 0 |
| map_blind | 9 | EKF cooperative (CI) | 5.41 | 7.29 | 530 | 251 | 1533 |

## Sweep: Message drop probability (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | p=0 | 55.534 | 1 | 3.719 |  |  | 1.129 |
| full_map | p=0.25 | 41.379 | 0.745 | 3.248 |  |  | 1.045 |
| full_map | p=0.5 | 27.68 | 0.498 | 3.365 |  |  | 1.073 |
| full_map | p=0.75 | 13.996 | 0.252 | 3.303 |  |  | 1.061 |
| full_map | p=0.9 | 5.702 | 0.103 | 3.606 |  |  | 1.15 |
| full_map | p=1 | 0 | 0 | 3.621 |  |  | 1.142 |
| map_blind | p=0 | 55.534 | 1 | 5.41 |  | 5.681 | 1.493 |
| map_blind | p=0.25 | 41.379 | 0.745 | 5.728 |  | 6.125 | 1.636 |
| map_blind | p=0.5 | 27.68 | 0.498 | 5.538 |  | 6.087 | 1.631 |
| map_blind | p=0.75 | 13.996 | 0.252 | 5.637 |  | 6.225 | 1.561 |
| map_blind | p=0.9 | 5.702 | 0.103 | 5.744 |  | 6.531 | 1.565 |
| map_blind | p=1 | 0 | 0 | 5.337 |  | 6.045 | 1.485 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 9 |
|---|---|---|
| full_map | p=0 | 3.719 |
| full_map | p=0.25 | 3.248 |
| full_map | p=0.5 | 3.365 |
| full_map | p=0.75 | 3.303 |
| full_map | p=0.9 | 3.606 |
| full_map | p=1 | 3.621 |
| map_blind | p=0 | 5.41 |
| map_blind | p=0.25 | 5.728 |
| map_blind | p=0.5 | 5.538 |
| map_blind | p=0.75 | 5.637 |
| map_blind | p=0.9 | 5.744 |
| map_blind | p=1 | 5.337 |

## Sweep: Comm radius (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | 1 m | 0.096 | 0.002 | 3.621 |  |  | 1.142 |
| full_map | 2 m | 16.559 | 0.298 | 3.626 |  |  | 1.149 |
| full_map | 4 m | 53.014 | 0.955 | 3.583 |  |  | 1.06 |
| full_map | 8 m | 55.534 | 1 | 3.719 |  |  | 1.129 |
| full_map | unlimited | 55.534 | 1 | 3.719 |  |  | 1.129 |
| map_blind | 1 m | 0.096 | 0.002 | 5.337 |  | 6.045 | 1.485 |
| map_blind | 2 m | 16.559 | 0.298 | 4.696 |  | 4.976 | 1.6 |
| map_blind | 4 m | 53.014 | 0.955 | 5.581 |  | 5.891 | 1.547 |
| map_blind | 8 m | 55.534 | 1 | 5.41 |  | 5.681 | 1.493 |
| map_blind | unlimited | 55.534 | 1 | 5.41 |  | 5.681 | 1.493 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 9 |
|---|---|---|
| full_map | 1 m | 3.621 |
| full_map | 2 m | 3.626 |
| full_map | 4 m | 3.583 |
| full_map | 8 m | 3.719 |
| full_map | unlimited | 3.719 |
| map_blind | 1 m | 5.337 |
| map_blind | 2 m | 4.696 |
| map_blind | 4 m | 5.581 |
| map_blind | 8 m | 5.41 |
| map_blind | unlimited | 5.41 |

## Sweep: Max update rate (minimum interval between teammate updates) (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | 1 s | 13.439 | 0.242 | 3.577 |  |  | 1.057 |
| full_map | 5 s | 4.152 | 0.075 | 3.968 |  |  | 1.158 |
| full_map | 10 s | 2.616 | 0.047 | 3.955 |  |  | 1.166 |
| full_map | 30 s | 1.32 | 0.024 | 4.038 |  |  | 1.231 |
| full_map | inf | 0 | 0 | 3.621 |  |  | 1.142 |
| map_blind | 1 s | 13.439 | 0.242 | 5.751 |  | 6.167 | 1.63 |
| map_blind | 5 s | 4.152 | 0.075 | 5.49 |  | 6.01 | 1.642 |
| map_blind | 10 s | 2.616 | 0.047 | 5.407 |  | 6.211 | 1.522 |
| map_blind | 30 s | 1.32 | 0.024 | 5.424 |  | 5.554 | 1.674 |
| map_blind | inf | 0 | 0 | 5.337 |  | 6.045 | 1.485 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 9 |
|---|---|---|
| full_map | 1 s | 3.577 |
| full_map | 5 s | 3.968 |
| full_map | 10 s | 3.955 |
| full_map | 30 s | 4.038 |
| full_map | inf | 3.621 |
| map_blind | 1 s | 5.751 |
| map_blind | 5 s | 5.49 |
| map_blind | 10 s | 5.407 |
| map_blind | 30 s | 5.424 |
| map_blind | inf | 5.337 |

## Sweep: Covariance-threshold trigger (tau, m^2) (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | tau=0.003 | 55.39 | 0.997 | 3.719 |  |  | 1.129 |
| full_map | tau=0.01 | 18.599 | 0.335 | 3.189 |  |  | 1.024 |
| full_map | tau=0.03 | 8.616 | 0.155 | 3.194 |  |  | 1.027 |
| full_map | tau=0.1 | 9.048 | 0.163 | 3.368 |  |  | 1.084 |
| full_map | tau=0.3 | 4.92 | 0.089 | 3.218 |  |  | 1.043 |
| full_map | tau=1 | 2.712 | 0.049 | 3.219 |  |  | 1.044 |
| full_map | tau=3 | 1.344 | 0.024 | 3.224 |  |  | 1.043 |
| full_map | tau=10 | 0 | 0 | 3.621 |  |  | 1.142 |
| map_blind | tau=0.003 | 55.486 | 0.999 | 5.41 |  | 5.681 | 1.493 |
| map_blind | tau=0.01 | 39.766 | 0.716 | 5.686 |  | 6.071 | 1.587 |
| map_blind | tau=0.03 | 35.567 | 0.64 | 4.987 |  | 4.968 | 1.482 |
| map_blind | tau=0.1 | 18.959 | 0.341 | 4.584 |  | 5.312 | 1.387 |
| map_blind | tau=0.3 | 16.295 | 0.293 | 5.437 |  | 6.301 | 1.555 |
| map_blind | tau=1 | 12.072 | 0.217 | 5.268 |  | 5.929 | 1.585 |
| map_blind | tau=3 | 3.888 | 0.07 | 5.179 |  | 5.782 | 1.593 |
| map_blind | tau=10 | 3.48 | 0.063 | 5.268 |  | 5.93 | 1.71 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 9 |
|---|---|---|
| full_map | tau=0.003 | 3.719 |
| full_map | tau=0.01 | 3.189 |
| full_map | tau=0.03 | 3.194 |
| full_map | tau=0.1 | 3.368 |
| full_map | tau=0.3 | 3.218 |
| full_map | tau=1 | 3.219 |
| full_map | tau=3 | 3.224 |
| full_map | tau=10 | 3.621 |
| map_blind | tau=0.003 | 5.41 |
| map_blind | tau=0.01 | 5.686 |
| map_blind | tau=0.03 | 4.987 |
| map_blind | tau=0.1 | 4.584 |
| map_blind | tau=0.3 | 5.437 |
| map_blind | tau=1 | 5.268 |
| map_blind | tau=3 | 5.179 |
| map_blind | tau=10 | 5.268 |

## Accuracy vs messages: knee points (pooled over families and per family)

| knee_found | family | setting | messages_per_robot_min | rmse_xy | message_fraction | fraction_of_gain | scope | regime | rmse_xy_full_comm | rmse_xy_no_comm | messages_per_robot_min_full_comm |
|---|---|---|---|---|---|---|---|---|---|---|---|
| True | trigger | tau=3 | 1.344 | 3.224 | 0.024 |  | pooled | full_map | 3.719 | 3.621 | 55.534 |
| True | drop | p=0.75 | 13.996 | 3.303 | 0.252 |  | drop | full_map | 3.719 | 3.621 | 55.534 |
| False |  |  |  |  |  |  | radius | full_map | 3.719 | 3.621 | 55.534 |
| False |  |  |  |  |  |  | rate | full_map | 3.577 | 3.621 | 13.439 |
| True | trigger | tau=3 | 1.344 | 3.224 | 0.024 |  | trigger | full_map | 3.719 | 3.621 | 55.39 |
| True | trigger | tau=3 | 3.888 | 5.179 | 0.07 |  | pooled | map_blind | 5.41 | 5.337 | 55.534 |
| False |  |  |  |  |  |  | drop | map_blind | 5.41 | 5.337 | 55.534 |
| False |  |  |  |  |  |  | radius | map_blind | 5.41 | 5.337 | 55.534 |
| False |  |  |  |  |  |  | rate | map_blind | 5.751 | 5.337 | 13.439 |
| True | trigger | tau=3 | 3.888 | 5.179 | 0.07 |  | trigger | map_blind | 5.41 | 5.337 | 55.486 |

## Matched-budget comparison: covariance-threshold trigger vs the nearest delivered-message rate of each other family

| regime | trigger_setting | trigger_messages_per_robot_min | trigger_rmse_xy | family | setting | messages_per_robot_min | rmse_xy | budget_ratio | rmse_difference |
|---|---|---|---|---|---|---|---|---|---|
| full_map | tau=0.003 | 55.39 | 3.719 | rate | 1 s | 13.439 | 3.577 | 4.121 | 0.143 |
| full_map | tau=0.003 | 55.39 | 3.719 | drop | p=0 | 55.534 | 3.719 | 0.997 | -4.44e-16 |
| full_map | tau=0.003 | 55.39 | 3.719 | radius | 8 m | 55.534 | 3.719 | 0.997 | 0 |
| full_map | tau=0.01 | 18.599 | 3.189 | rate | 1 s | 13.439 | 3.577 | 1.384 | -0.387 |
| full_map | tau=0.01 | 18.599 | 3.189 | drop | p=0.75 | 13.996 | 3.303 | 1.329 | -0.114 |
| full_map | tau=0.01 | 18.599 | 3.189 | radius | 2 m | 16.559 | 3.626 | 1.123 | -0.437 |
| full_map | tau=0.03 | 8.616 | 3.194 | rate | 1 s | 13.439 | 3.577 | 0.641 | -0.382 |
| full_map | tau=0.03 | 8.616 | 3.194 | drop | p=0.9 | 5.702 | 3.606 | 1.511 | -0.412 |
| full_map | tau=0.03 | 8.616 | 3.194 | radius | 2 m | 16.559 | 3.626 | 0.52 | -0.432 |
| full_map | tau=0.1 | 9.048 | 3.368 | rate | 1 s | 13.439 | 3.577 | 0.673 | -0.208 |
| full_map | tau=0.1 | 9.048 | 3.368 | drop | p=0.75 | 13.996 | 3.303 | 0.646 | 0.065 |
| full_map | tau=0.1 | 9.048 | 3.368 | radius | 2 m | 16.559 | 3.626 | 0.546 | -0.258 |
| full_map | tau=0.3 | 4.92 | 3.218 | rate | 5 s | 4.152 | 3.968 | 1.185 | -0.75 |
| full_map | tau=0.3 | 4.92 | 3.218 | drop | p=0.9 | 5.702 | 3.606 | 0.863 | -0.389 |
| full_map | tau=0.3 | 4.92 | 3.218 | radius | 2 m | 16.559 | 3.626 | 0.297 | -0.409 |
| full_map | tau=1 | 2.712 | 3.219 | rate | 10 s | 2.616 | 3.955 | 1.037 | -0.736 |
| full_map | tau=1 | 2.712 | 3.219 | drop | p=0.9 | 5.702 | 3.606 | 0.476 | -0.387 |
| full_map | tau=1 | 2.712 | 3.219 | radius | 2 m | 16.559 | 3.626 | 0.164 | -0.407 |
| full_map | tau=3 | 1.344 | 3.224 | rate | 30 s | 1.32 | 4.038 | 1.018 | -0.815 |
| full_map | tau=3 | 1.344 | 3.224 | drop | p=0.9 | 5.702 | 3.606 | 0.236 | -0.383 |
| full_map | tau=3 | 1.344 | 3.224 | radius | 2 m | 16.559 | 3.626 | 0.081 | -0.403 |
| map_blind | tau=0.003 | 55.486 | 5.41 | rate | 1 s | 13.439 | 5.751 | 4.129 | -0.341 |
| map_blind | tau=0.003 | 55.486 | 5.41 | drop | p=0 | 55.534 | 5.41 | 0.999 | 8.88e-16 |
| map_blind | tau=0.003 | 55.486 | 5.41 | radius | 8 m | 55.534 | 5.41 | 0.999 | 0 |
| map_blind | tau=0.01 | 39.766 | 5.686 | rate | 1 s | 13.439 | 5.751 | 2.959 | -0.065 |
| map_blind | tau=0.01 | 39.766 | 5.686 | drop | p=0.25 | 41.379 | 5.728 | 0.961 | -0.042 |
| map_blind | tau=0.01 | 39.766 | 5.686 | radius | 4 m | 53.014 | 5.581 | 0.75 | 0.105 |
| map_blind | tau=0.03 | 35.567 | 4.987 | rate | 1 s | 13.439 | 5.751 | 2.646 | -0.764 |
| map_blind | tau=0.03 | 35.567 | 4.987 | drop | p=0.25 | 41.379 | 5.728 | 0.86 | -0.741 |
| map_blind | tau=0.03 | 35.567 | 4.987 | radius | 4 m | 53.014 | 5.581 | 0.671 | -0.594 |
| map_blind | tau=0.1 | 18.959 | 4.584 | rate | 1 s | 13.439 | 5.751 | 1.411 | -1.167 |
| map_blind | tau=0.1 | 18.959 | 4.584 | drop | p=0.75 | 13.996 | 5.637 | 1.355 | -1.053 |
| map_blind | tau=0.1 | 18.959 | 4.584 | radius | 2 m | 16.559 | 4.696 | 1.145 | -0.113 |
| map_blind | tau=0.3 | 16.295 | 5.437 | rate | 1 s | 13.439 | 5.751 | 1.213 | -0.314 |
| map_blind | tau=0.3 | 16.295 | 5.437 | drop | p=0.75 | 13.996 | 5.637 | 1.164 | -0.2 |
| map_blind | tau=0.3 | 16.295 | 5.437 | radius | 2 m | 16.559 | 4.696 | 0.984 | 0.741 |
| map_blind | tau=1 | 12.072 | 5.268 | rate | 1 s | 13.439 | 5.751 | 0.898 | -0.483 |
| map_blind | tau=1 | 12.072 | 5.268 | drop | p=0.75 | 13.996 | 5.637 | 0.862 | -0.369 |
| map_blind | tau=1 | 12.072 | 5.268 | radius | 2 m | 16.559 | 4.696 | 0.729 | 0.571 |
| map_blind | tau=3 | 3.888 | 5.179 | rate | 5 s | 4.152 | 5.49 | 0.936 | -0.31 |
| map_blind | tau=3 | 3.888 | 5.179 | drop | p=0.9 | 5.702 | 5.744 | 0.682 | -0.565 |
| map_blind | tau=3 | 3.888 | 5.179 | radius | 2 m | 16.559 | 4.696 | 0.235 | 0.483 |
| map_blind | tau=10 | 3.48 | 5.268 | rate | 5 s | 4.152 | 5.49 | 0.838 | -0.221 |
| map_blind | tau=10 | 3.48 | 5.268 | drop | p=0.9 | 5.702 | 5.744 | 0.61 | -0.476 |
| map_blind | tau=10 | 3.48 | 5.268 | radius | 2 m | 16.559 | 4.696 | 0.21 | 0.572 |
