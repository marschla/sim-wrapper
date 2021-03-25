# Duckietown Simulator Wrapper

Contains the docker image for the Duckietown Simulator Wrapper.

1) Build the sim-wrapper 
```shell script
 dts devel build -f
```

2) Run the sim-wrapper 
```shell script
 docker run -it --rm --net=host -v /PATH_TO_CALI_FILE:/data -e duckietown/sim-wrapper:v2-amd64
```

The Calibration file and more information can be found here: https://docs.duckietown.org/daffy/duckietown-robotics-development/out/duckietown_simulation.html
