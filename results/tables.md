## Dataset summary

| dataset | robot | duration_s | landmark_obs | robot_obs | robot_obs_per_min | robot_range_median_m | robot_range_max_m | gt_dropout_s | odom_missing_s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1 | 1500 | 4771 | 952 | 38.08 | 2.51 | 6.45 | 0.9 | 9.5 |
| 1 | 2 | 1500 | 5543 | 1156 | 46.24 | 2.41 | 6.56 | 0 | 10.7 |
| 1 | 3 | 1500 | 6771 | 1969 | 78.76 | 3.49 | 7.43 | 0 | 12.2 |
| 1 | 4 | 1500 | 3269 | 722 | 28.88 | 2.32 | 5.81 | 1.4 | 9.4 |
| 1 | 5 | 1500 | 7137 | 1654 | 66.16 | 2.4 | 5.7 | 0 | 10.9 |
| 2 | 1 | 1861 | 6631 | 1884 | 60.73 | 2.41 | 6.03 | 0 | 5.9 |
| 2 | 2 | 1861 | 6049 | 1684 | 54.28 | 2.23 | 6.88 | 1.6 | 6.9 |
| 2 | 3 | 1861 | 8624 | 2108 | 67.95 | 2.29 | 5.87 | 0 | 7.8 |
| 2 | 4 | 1861 | 3956 | 1030 | 33.2 | 2.59 | 5.28 | 0.6 | 14.3 |
| 2 | 5 | 1861 | 11296 | 2087 | 67.27 | 2.61 | 7.05 | 0 | 8.6 |
| 3 | 1 | 1800 | 7011 | 1292 | 43.07 | 2.26 | 7.28 | 1.1 | 8.1 |
| 3 | 2 | 1800 | 6697 | 1427 | 47.57 | 2.26 | 7.05 | 0.6 | 9.3 |
| 3 | 3 | 1800 | 8855 | 1817 | 60.57 | 3 | 8.85 | 116.8 | 10.3 |
| 3 | 4 | 1800 | 2229 | 161 | 5.37 | 2.4 | 5.09 | 0.6 | 293.9 |
| 3 | 5 | 1800 | 10363 | 2196 | 73.2 | 2.58 | 8.95 | 0.6 | 5.3 |
| 4 | 1 | 1400 | 6728 | 1341 | 57.47 | 2.16 | 5.15 | 0.6 | 11.3 |
| 4 | 2 | 1400 | 5242 | 1135 | 48.64 | 2.07 | 7.62 | 0.6 | 10 |
| 4 | 3 | 1400 | 6443 | 1277 | 54.73 | 2.48 | 5.87 | 0.6 | 12.7 |
| 4 | 4 | 1400 | 4779 | 756 | 32.4 | 2.66 | 6.19 | 0.6 | 17 |
| 4 | 5 | 1400 | 9368 | 2336 | 100.11 | 2.36 | 6.79 | 0.6 | 12.5 |

## Base methods: team position RMSE [m], mean over the five robots

| regime | dataset | Dead reckoning | EKF landmarks | EKF cooperative |
|---|---|---|---|---|
| full_map | 1 | 4.863 | 0.185 | 0.19 |
| full_map | 2 | 4.332 | 0.193 | 0.186 |
| full_map | 3 | 3.929 | 0.121 | 0.122 |
| full_map | 4 | 3.494 | 0.111 | 0.111 |
| map_blind | 1 | 4.863 | 2.97 | 0.352 |
| map_blind | 2 | 4.332 | 2.992 | 0.361 |
| map_blind | 3 | 3.929 | 2.956 | 0.95 |
| map_blind | 4 | 3.494 | 1.97 | 0.277 |

## Base methods: per-robot position RMSE [m]

| regime | dataset | robot | blind | Dead reckoning | EKF landmarks | EKF cooperative |
|---|---|---|---|---|---|---|
| full_map | 1 | 1 | False | 5.131 | 0.191 | 0.186 |
| full_map | 1 | 2 | False | 4.677 | 0.15 | 0.151 |
| full_map | 1 | 3 | False | 6.455 | 0.153 | 0.182 |
| full_map | 1 | 4 | False | 1.629 | 0.24 | 0.242 |
| full_map | 1 | 5 | False | 6.425 | 0.189 | 0.19 |
| full_map | 2 | 1 | False | 4.027 | 0.178 | 0.172 |
| full_map | 2 | 2 | False | 3.039 | 0.188 | 0.177 |
| full_map | 2 | 3 | False | 5.269 | 0.16 | 0.153 |
| full_map | 2 | 4 | False | 6.162 | 0.262 | 0.257 |
| full_map | 2 | 5 | False | 3.165 | 0.18 | 0.172 |
| full_map | 3 | 1 | False | 1.622 | 0.085 | 0.086 |
| full_map | 3 | 2 | False | 3.453 | 0.12 | 0.121 |
| full_map | 3 | 3 | False | 6.832 | 0.151 | 0.155 |
| full_map | 3 | 4 | False | 3.194 | 0.119 | 0.118 |
| full_map | 3 | 5 | False | 4.546 | 0.13 | 0.133 |
| full_map | 4 | 1 | False | 3.102 | 0.08 | 0.079 |
| full_map | 4 | 2 | False | 4.689 | 0.092 | 0.091 |
| full_map | 4 | 3 | False | 4.617 | 0.109 | 0.105 |
| full_map | 4 | 4 | False | 2.794 | 0.166 | 0.168 |
| full_map | 4 | 5 | False | 2.267 | 0.111 | 0.111 |
| map_blind | 1 | 1 | False | 5.131 | 0.191 | 0.193 |
| map_blind | 1 | 2 | False | 4.677 | 0.15 | 0.153 |
| map_blind | 1 | 3 | True | 6.455 | 6.455 | 0.437 |
| map_blind | 1 | 4 | True | 1.629 | 1.629 | 0.61 |
| map_blind | 1 | 5 | True | 6.425 | 6.425 | 0.366 |
| map_blind | 2 | 1 | False | 4.027 | 0.178 | 0.174 |
| map_blind | 2 | 2 | False | 3.039 | 0.188 | 0.182 |
| map_blind | 2 | 3 | True | 5.269 | 5.269 | 0.497 |
| map_blind | 2 | 4 | True | 6.162 | 6.162 | 0.633 |
| map_blind | 2 | 5 | True | 3.165 | 3.165 | 0.317 |
| map_blind | 3 | 1 | False | 1.622 | 0.085 | 0.086 |
| map_blind | 3 | 2 | False | 3.453 | 0.12 | 0.121 |
| map_blind | 3 | 3 | True | 6.832 | 6.832 | 0.643 |
| map_blind | 3 | 4 | True | 3.194 | 3.194 | 3.237 |
| map_blind | 3 | 5 | True | 4.546 | 4.546 | 0.665 |
| map_blind | 4 | 1 | False | 3.102 | 0.08 | 0.079 |
| map_blind | 4 | 2 | False | 4.689 | 0.092 | 0.09 |
| map_blind | 4 | 3 | True | 4.617 | 4.617 | 0.382 |
| map_blind | 4 | 4 | True | 2.794 | 2.794 | 0.463 |
| map_blind | 4 | 5 | True | 2.267 | 2.267 | 0.373 |

## Base methods: heading RMSE [rad], mean over robots

| regime | dataset | Dead reckoning | EKF landmarks | EKF cooperative |
|---|---|---|---|---|
| full_map | 1 | 1.364 | 0.099 | 0.098 |
| full_map | 2 | 1.538 | 0.104 | 0.099 |
| full_map | 3 | 1.423 | 0.082 | 0.082 |
| full_map | 4 | 1.325 | 0.067 | 0.066 |
| map_blind | 1 | 1.364 | 0.739 | 0.231 |
| map_blind | 2 | 1.538 | 1.012 | 0.178 |
| map_blind | 3 | 1.423 | 0.985 | 0.371 |
| map_blind | 4 | 1.325 | 0.745 | 0.173 |

## Fusion ablation: covariance intersection (CI) vs naive inflation, unconstrained communication, team position RMSE [m]

| regime | dataset | method | rmse_xy | rmse_xy_max_robot | fused | gated | declined |
|---|---|---|---|---|---|---|---|
| full_map | 1 | EKF cooperative (naive) | 0.182 | 0.237 | 6385 | 68 | 0 |
| full_map | 1 | EKF cooperative (CI) | 0.19 | 0.242 | 2832 | 4 | 3617 |
| map_blind | 1 | EKF cooperative (naive) | 0.487 | 0.931 | 6196 | 257 | 0 |
| map_blind | 1 | EKF cooperative (CI) | 0.352 | 0.61 | 1999 | 1 | 4453 |
| full_map | 2 | EKF cooperative (naive) | 0.173 | 0.204 | 8709 | 84 | 0 |
| full_map | 2 | EKF cooperative (CI) | 0.186 | 0.257 | 3594 | 1 | 5198 |
| map_blind | 2 | EKF cooperative (naive) | 0.499 | 0.789 | 8627 | 166 | 0 |
| map_blind | 2 | EKF cooperative (CI) | 0.361 | 0.633 | 2448 | 1 | 6344 |
| full_map | 3 | EKF cooperative (naive) | 0.121 | 0.15 | 6866 | 27 | 0 |
| full_map | 3 | EKF cooperative (CI) | 0.122 | 0.155 | 2928 | 1 | 3964 |
| map_blind | 3 | EKF cooperative (naive) | 1.239 | 3.019 | 5979 | 914 | 0 |
| map_blind | 3 | EKF cooperative (CI) | 0.95 | 3.237 | 2098 | 8 | 4787 |
| full_map | 4 | EKF cooperative (naive) | 0.109 | 0.168 | 6829 | 16 | 0 |
| full_map | 4 | EKF cooperative (CI) | 0.111 | 0.168 | 2824 | 3 | 4018 |
| map_blind | 4 | EKF cooperative (naive) | 0.239 | 0.363 | 6797 | 48 | 0 |
| map_blind | 4 | EKF cooperative (CI) | 0.277 | 0.463 | 2503 | 8 | 4334 |

## Sweep: Message drop probability (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | p=0 | 53.234 | 1 | 0.152 | 0.042 |  | 0.086 |
| full_map | p=0.25 | 39.775 | 0.747 | 0.151 | 0.04 |  | 0.086 |
| full_map | p=0.5 | 26.572 | 0.499 | 0.15 | 0.04 |  | 0.086 |
| full_map | p=0.75 | 13.447 | 0.253 | 0.15 | 0.04 |  | 0.087 |
| full_map | p=0.9 | 5.395 | 0.101 | 0.151 | 0.041 |  | 0.087 |
| full_map | p=1 | 0 | 0 | 0.153 | 0.042 |  | 0.088 |
| map_blind | p=0 | 53.234 | 1 | 0.485 | 0.312 | 0.719 | 0.238 |
| map_blind | p=0.25 | 39.775 | 0.747 | 0.488 | 0.328 | 0.724 | 0.242 |
| map_blind | p=0.5 | 26.572 | 0.499 | 0.533 | 0.372 | 0.798 | 0.266 |
| map_blind | p=0.75 | 13.447 | 0.253 | 0.521 | 0.313 | 0.779 | 0.268 |
| map_blind | p=0.9 | 5.395 | 0.101 | 0.696 | 0.413 | 1.07 | 0.333 |
| map_blind | p=1 | 0 | 0 | 2.722 | 0.502 | 4.446 | 0.87 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 1 | dataset 2 | dataset 3 | dataset 4 |
|---|---|---|---|---|---|
| full_map | p=0 | 0.19 | 0.186 | 0.122 | 0.111 |
| full_map | p=0.25 | 0.185 | 0.185 | 0.122 | 0.11 |
| full_map | p=0.5 | 0.184 | 0.185 | 0.121 | 0.11 |
| full_map | p=0.75 | 0.184 | 0.186 | 0.121 | 0.11 |
| full_map | p=0.9 | 0.183 | 0.188 | 0.121 | 0.111 |
| full_map | p=1 | 0.185 | 0.193 | 0.121 | 0.111 |
| map_blind | p=0 | 0.352 | 0.361 | 0.95 | 0.277 |
| map_blind | p=0.25 | 0.345 | 0.361 | 0.976 | 0.27 |
| map_blind | p=0.5 | 0.399 | 0.364 | 1.087 | 0.282 |
| map_blind | p=0.75 | 0.438 | 0.386 | 0.981 | 0.281 |
| map_blind | p=0.9 | 0.666 | 0.528 | 1.277 | 0.314 |
| map_blind | p=1 | 2.97 | 2.992 | 2.956 | 1.97 |

## Sweep: Comm radius (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | 1 m | 0.493 | 0.009 | 0.153 | 0.042 |  | 0.088 |
| full_map | 2 m | 17.556 | 0.329 | 0.149 | 0.039 |  | 0.086 |
| full_map | 4 m | 44.875 | 0.84 | 0.151 | 0.04 |  | 0.086 |
| full_map | 8 m | 53.216 | 1 | 0.152 | 0.042 |  | 0.086 |
| full_map | unlimited | 53.234 | 1 | 0.152 | 0.042 |  | 0.086 |
| map_blind | 1 m | 0.493 | 0.009 | 2.864 | 0.295 | 4.683 | 0.96 |
| map_blind | 2 m | 17.556 | 0.329 | 1.003 | 0.471 | 1.582 | 0.5 |
| map_blind | 4 m | 44.875 | 0.84 | 0.558 | 0.447 | 0.84 | 0.291 |
| map_blind | 8 m | 53.216 | 1 | 0.485 | 0.312 | 0.719 | 0.238 |
| map_blind | unlimited | 53.234 | 1 | 0.485 | 0.312 | 0.719 | 0.238 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 1 | dataset 2 | dataset 3 | dataset 4 |
|---|---|---|---|---|---|
| full_map | 1 m | 0.185 | 0.193 | 0.121 | 0.111 |
| full_map | 2 m | 0.18 | 0.184 | 0.12 | 0.111 |
| full_map | 4 m | 0.185 | 0.184 | 0.122 | 0.111 |
| full_map | 8 m | 0.19 | 0.186 | 0.122 | 0.111 |
| full_map | unlimited | 0.19 | 0.186 | 0.122 | 0.111 |
| map_blind | 1 m | 3.149 | 3.05 | 2.76 | 2.498 |
| map_blind | 2 m | 0.929 | 1.254 | 1.454 | 0.376 |
| map_blind | 4 m | 0.363 | 0.386 | 1.224 | 0.259 |
| map_blind | 8 m | 0.352 | 0.361 | 0.95 | 0.277 |
| map_blind | unlimited | 0.352 | 0.361 | 0.95 | 0.277 |

## Sweep: Max update rate (minimum interval between teammate updates) (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | 1 s | 13.48 | 0.255 | 0.15 | 0.04 |  | 0.086 |
| full_map | 5 s | 4.17 | 0.079 | 0.151 | 0.041 |  | 0.087 |
| full_map | 10 s | 2.455 | 0.046 | 0.15 | 0.04 |  | 0.086 |
| full_map | 30 s | 1.096 | 0.021 | 0.152 | 0.041 |  | 0.087 |
| full_map | inf | 0 | 0 | 0.153 | 0.042 |  | 0.088 |
| map_blind | 1 s | 13.48 | 0.255 | 0.428 | 0.204 | 0.623 | 0.229 |
| map_blind | 5 s | 4.17 | 0.079 | 0.445 | 0.205 | 0.651 | 0.243 |
| map_blind | 10 s | 2.455 | 0.046 | 0.492 | 0.227 | 0.73 | 0.255 |
| map_blind | 30 s | 1.096 | 0.021 | 0.735 | 0.314 | 1.134 | 0.345 |
| map_blind | inf | 0 | 0 | 2.722 | 0.502 | 4.446 | 0.87 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 1 | dataset 2 | dataset 3 | dataset 4 |
|---|---|---|---|---|---|
| full_map | 1 s | 0.183 | 0.186 | 0.12 | 0.11 |
| full_map | 5 s | 0.184 | 0.189 | 0.121 | 0.111 |
| full_map | 10 s | 0.179 | 0.191 | 0.121 | 0.111 |
| full_map | 30 s | 0.183 | 0.192 | 0.121 | 0.111 |
| full_map | inf | 0.185 | 0.193 | 0.121 | 0.111 |
| map_blind | 1 s | 0.353 | 0.363 | 0.726 | 0.268 |
| map_blind | 5 s | 0.351 | 0.381 | 0.747 | 0.299 |
| map_blind | 10 s | 0.373 | 0.482 | 0.814 | 0.299 |
| map_blind | 30 s | 0.978 | 0.655 | 0.982 | 0.324 |
| map_blind | inf | 2.97 | 2.992 | 2.956 | 1.97 |

## Sweep: Covariance-threshold trigger (tau, m^2) (mean over datasets)

| regime | setting | messages_per_robot_min | message_fraction | rmse_xy | rmse_xy_std_datasets | rmse_xy_blind | rmse_theta |
|---|---|---|---|---|---|---|---|
| full_map | tau=0.003 | 43.756 | 0.822 | 0.152 | 0.042 |  | 0.086 |
| full_map | tau=0.01 | 8.529 | 0.158 | 0.151 | 0.04 |  | 0.086 |
| full_map | tau=0.03 | 1.46 | 0.027 | 0.152 | 0.041 |  | 0.087 |
| full_map | tau=0.1 | 0.335 | 0.006 | 0.151 | 0.041 |  | 0.087 |
| full_map | tau=0.3 | 0.024 | 0.000427 | 0.152 | 0.041 |  | 0.087 |
| full_map | tau=1 | 0.004 | 6.72e-05 | 0.153 | 0.043 |  | 0.088 |
| full_map | tau=3 | 0.004 | 6.72e-05 | 0.152 | 0.042 |  | 0.088 |
| full_map | tau=10 | 0 | 0 | 0.153 | 0.042 |  | 0.088 |
| map_blind | tau=0.003 | 50.161 | 0.943 | 0.485 | 0.312 | 0.719 | 0.238 |
| map_blind | tau=0.01 | 36.724 | 0.689 | 0.485 | 0.311 | 0.719 | 0.239 |
| map_blind | tau=0.03 | 33.609 | 0.631 | 0.49 | 0.32 | 0.727 | 0.24 |
| map_blind | tau=0.1 | 24.736 | 0.464 | 0.553 | 0.405 | 0.831 | 0.261 |
| map_blind | tau=0.3 | 10.178 | 0.194 | 0.505 | 0.293 | 0.752 | 0.258 |
| map_blind | tau=1 | 4.903 | 0.094 | 0.765 | 0.443 | 1.184 | 0.346 |
| map_blind | tau=3 | 2.411 | 0.046 | 1.084 | 0.418 | 1.717 | 0.49 |
| map_blind | tau=10 | 1.992 | 0.038 | 1.309 | 0.368 | 2.091 | 0.57 |

Per dataset (team position RMSE [m]):

| regime | setting | dataset 1 | dataset 2 | dataset 3 | dataset 4 |
|---|---|---|---|---|---|
| full_map | tau=0.003 | 0.19 | 0.186 | 0.123 | 0.111 |
| full_map | tau=0.01 | 0.183 | 0.188 | 0.122 | 0.111 |
| full_map | tau=0.03 | 0.184 | 0.19 | 0.121 | 0.111 |
| full_map | tau=0.1 | 0.181 | 0.193 | 0.121 | 0.111 |
| full_map | tau=0.3 | 0.182 | 0.193 | 0.121 | 0.111 |
| full_map | tau=1 | 0.186 | 0.193 | 0.121 | 0.111 |
| full_map | tau=3 | 0.184 | 0.193 | 0.121 | 0.111 |
| full_map | tau=10 | 0.185 | 0.193 | 0.121 | 0.111 |
| map_blind | tau=0.003 | 0.352 | 0.361 | 0.95 | 0.277 |
| map_blind | tau=0.01 | 0.357 | 0.359 | 0.948 | 0.277 |
| map_blind | tau=0.03 | 0.357 | 0.361 | 0.968 | 0.276 |
| map_blind | tau=0.1 | 0.359 | 0.378 | 1.159 | 0.316 |
| map_blind | tau=0.3 | 0.366 | 0.401 | 0.941 | 0.313 |
| map_blind | tau=1 | 0.709 | 0.511 | 1.406 | 0.433 |
| map_blind | tau=3 | 1.508 | 0.715 | 1.379 | 0.735 |
| map_blind | tau=10 | 1.146 | 0.894 | 1.743 | 1.453 |

## Accuracy vs messages: knee points (pooled over families and per family)

| knee_found | scope | regime | rmse_xy_full_comm | rmse_xy_no_comm | messages_per_robot_min_full_comm | family | setting | messages_per_robot_min | rmse_xy | message_fraction | fraction_of_gain |
|---|---|---|---|---|---|---|---|---|---|---|---|
| False | pooled | full_map | 0.152 | 0.153 | 53.234 |  |  |  |  |  |  |
| False | drop | full_map | 0.152 | 0.153 | 53.234 |  |  |  |  |  |  |
| False | radius | full_map | 0.152 | 0.153 | 53.234 |  |  |  |  |  |  |
| False | rate | full_map | 0.15 | 0.153 | 13.48 |  |  |  |  |  |  |
| False | trigger | full_map | 0.152 | 0.153 | 43.756 |  |  |  |  |  |  |
| True | pooled | map_blind | 0.485 | 2.722 | 53.234 | rate | 10 s | 2.455 | 0.492 | 0.046 | 0.997 |
| True | drop | map_blind | 0.485 | 2.722 | 53.234 | drop | p=0.9 | 5.395 | 0.696 | 0.101 | 0.906 |
| True | radius | map_blind | 0.485 | 2.722 | 53.234 | radius | 2 m | 17.556 | 1.003 | 0.329 | 0.768 |
| True | rate | map_blind | 0.428 | 2.722 | 13.48 | rate | 10 s | 2.455 | 0.492 | 0.046 | 0.972 |
| True | trigger | map_blind | 0.485 | 2.722 | 50.161 | trigger | tau=0.3 | 10.178 | 0.505 | 0.194 | 0.991 |

## Matched-budget comparison: covariance-threshold trigger vs the nearest delivered-message rate of each other family

| regime | trigger_setting | trigger_messages_per_robot_min | trigger_rmse_xy | family | setting | messages_per_robot_min | rmse_xy | budget_ratio | rmse_difference |
|---|---|---|---|---|---|---|---|---|---|
| full_map | tau=0.003 | 43.756 | 0.152 | rate | 1 s | 13.48 | 0.15 | 3.246 | 0.003 |
| full_map | tau=0.003 | 43.756 | 0.152 | drop | p=0.25 | 39.775 | 0.151 | 1.1 | 0.002 |
| full_map | tau=0.003 | 43.756 | 0.152 | radius | 4 m | 44.875 | 0.151 | 0.975 | 0.002 |
| full_map | tau=0.01 | 8.529 | 0.151 | rate | 1 s | 13.48 | 0.15 | 0.633 | 0.000914 |
| full_map | tau=0.01 | 8.529 | 0.151 | drop | p=0.75 | 13.447 | 0.15 | 0.634 | 0.000352 |
| full_map | tau=0.01 | 8.529 | 0.151 | radius | 2 m | 17.556 | 0.149 | 0.486 | 0.002 |
| full_map | tau=0.03 | 1.46 | 0.152 | rate | 30 s | 1.096 | 0.152 | 1.332 | -0.000167 |
| full_map | tau=0.03 | 1.46 | 0.152 | drop | p=0.9 | 5.395 | 0.151 | 0.271 | 0.000877 |
| full_map | tau=0.03 | 1.46 | 0.152 | radius | 1 m | 0.493 | 0.153 | 2.964 | -0.000959 |
| full_map | tau=0.1 | 0.335 | 0.151 | rate | 30 s | 1.096 | 0.152 | 0.306 | -0.000264 |
| full_map | tau=0.1 | 0.335 | 0.151 | drop | p=0.9 | 5.395 | 0.151 | 0.062 | 0.000781 |
| full_map | tau=0.1 | 0.335 | 0.151 | radius | 1 m | 0.493 | 0.153 | 0.681 | -0.001 |
| full_map | tau=0.3 | 0.024 | 0.152 | rate | 30 s | 1.096 | 0.152 | 0.021 | 2.96e-05 |
| full_map | tau=0.3 | 0.024 | 0.152 | drop | p=0.9 | 5.395 | 0.151 | 0.004 | 0.001 |
| full_map | tau=0.3 | 0.024 | 0.152 | radius | 1 m | 0.493 | 0.153 | 0.048 | -0.000763 |
| full_map | tau=1 | 0.004 | 0.153 | rate | 30 s | 1.096 | 0.152 | 0.003 | 0.001 |
| full_map | tau=1 | 0.004 | 0.153 | drop | p=0.9 | 5.395 | 0.151 | 0.000669 | 0.002 |
| full_map | tau=1 | 0.004 | 0.153 | radius | 1 m | 0.493 | 0.153 | 0.007 | 0.000465 |
| full_map | tau=3 | 0.004 | 0.152 | rate | 30 s | 1.096 | 0.152 | 0.003 | 0.000681 |
| full_map | tau=3 | 0.004 | 0.152 | drop | p=0.9 | 5.395 | 0.151 | 0.000669 | 0.002 |
| full_map | tau=3 | 0.004 | 0.152 | radius | 1 m | 0.493 | 0.153 | 0.007 | -0.000112 |
| map_blind | tau=0.003 | 50.161 | 0.485 | rate | 1 s | 13.48 | 0.428 | 3.721 | 0.057 |
| map_blind | tau=0.003 | 50.161 | 0.485 | drop | p=0 | 53.234 | 0.485 | 0.942 | 4.84e-06 |
| map_blind | tau=0.003 | 50.161 | 0.485 | radius | 8 m | 53.216 | 0.485 | 0.943 | 3.25e-06 |
| map_blind | tau=0.01 | 36.724 | 0.485 | rate | 1 s | 13.48 | 0.428 | 2.724 | 0.058 |
| map_blind | tau=0.01 | 36.724 | 0.485 | drop | p=0.25 | 39.775 | 0.488 | 0.923 | -0.003 |
| map_blind | tau=0.01 | 36.724 | 0.485 | radius | 4 m | 44.875 | 0.558 | 0.818 | -0.073 |
| map_blind | tau=0.03 | 33.609 | 0.49 | rate | 1 s | 13.48 | 0.428 | 2.493 | 0.063 |
| map_blind | tau=0.03 | 33.609 | 0.49 | drop | p=0.25 | 39.775 | 0.488 | 0.845 | 0.002 |
| map_blind | tau=0.03 | 33.609 | 0.49 | radius | 4 m | 44.875 | 0.558 | 0.749 | -0.067 |
| map_blind | tau=0.1 | 24.736 | 0.553 | rate | 1 s | 13.48 | 0.428 | 1.835 | 0.125 |
| map_blind | tau=0.1 | 24.736 | 0.553 | drop | p=0.5 | 26.572 | 0.533 | 0.931 | 0.02 |
| map_blind | tau=0.1 | 24.736 | 0.553 | radius | 2 m | 17.556 | 1.003 | 1.409 | -0.45 |
| map_blind | tau=0.3 | 10.178 | 0.505 | rate | 1 s | 13.48 | 0.428 | 0.755 | 0.078 |
| map_blind | tau=0.3 | 10.178 | 0.505 | drop | p=0.75 | 13.447 | 0.521 | 0.757 | -0.016 |
| map_blind | tau=0.3 | 10.178 | 0.505 | radius | 2 m | 17.556 | 1.003 | 0.58 | -0.498 |
| map_blind | tau=1 | 4.903 | 0.765 | rate | 5 s | 4.17 | 0.445 | 1.176 | 0.32 |
| map_blind | tau=1 | 4.903 | 0.765 | drop | p=0.9 | 5.395 | 0.696 | 0.909 | 0.069 |
| map_blind | tau=1 | 4.903 | 0.765 | radius | 2 m | 17.556 | 1.003 | 0.279 | -0.239 |
| map_blind | tau=3 | 2.411 | 1.084 | rate | 10 s | 2.455 | 0.492 | 0.982 | 0.592 |
| map_blind | tau=3 | 2.411 | 1.084 | drop | p=0.9 | 5.395 | 0.696 | 0.447 | 0.389 |
| map_blind | tau=3 | 2.411 | 1.084 | radius | 1 m | 0.493 | 2.864 | 4.893 | -1.779 |
| map_blind | tau=10 | 1.992 | 1.309 | rate | 10 s | 2.455 | 0.492 | 0.811 | 0.817 |
| map_blind | tau=10 | 1.992 | 1.309 | drop | p=0.9 | 5.395 | 0.696 | 0.369 | 0.613 |
| map_blind | tau=10 | 1.992 | 1.309 | radius | 1 m | 0.493 | 2.864 | 4.043 | -1.555 |
