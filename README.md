# temperature_gradient_planner
This repository contains ros node that implements continiously updated artificial temperature gradient based motion planner.

#### Using ros service to call planner

```
rosservice call /temperature_gradient_navigation_node/traverse "qstart:
- 256
- 50
qgoal:
- 256
- 256
outfile_name: '/home/ozzdemir/example.csv'" 

```