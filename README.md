# temperature_gradient_planner
This repository contains ros node that implements continiously updated artificial temperature gradient based motion planner.

#### Reference
`Golan, Y., Edelman, S., Shapiro, A., & Rimon, E. (2017). Online Robot Navigation Using Continuously Updated Artificial Temperature Gradients. IEEE Robotics and Automation Letters, 2, 1280-1287.`

#### For using planner with matlab
`rosrun temperature_adient_navigation temperature_gradient_navigation_node _use_offline_map:=true`

#### For using planner with gazebo, online

`roslaunch temperature_gradient_navigation load_map.launch input_map:=opti_map`
`roslaunch temperature_gradient_navigation spawn_agent.launch `
`roslaunch temperature_gradient_navigation temperature_gradient_navigation_online.launch`

#### For using planner with gazebo, offline

`roslaunch temperature_gradient_navigation load_map.launch input_map:=opti_map`
`roslaunch temperature_gradient_navigation spawn_agent.launch `
`roslaunch temperature_gradient_navigation temperature_gradient_navigation_offline.launch input_map:=opti_map`

#### Demos

![Dynamic Map Demo](demo/temperature_field_planner_dynamic_map_demo.gif)

![Offline Tracjectory Demo](demo/temperature_field_planner_offline_trajectory_demo.gif)


