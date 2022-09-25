# **My-Planning-Control-Studies**


## *Installation*

* Install requirements

Firstly, install ros foxy from [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

Install the matrix library, which i have developed, from [here](https://github.com/CihatAltiparmak/Matrix)

Create any workspace and clone this repository here.

```shell

mkdir jarbay_ws/src -p
cd jarbay_ws/src
git clone https://github.com/CihatAltiparmak/My-Planning-Control-Studies
cd ..
source /opt/ros/foxy/setup.bash
colcon build
``` 

Source bash file
```
source install/setup.bash
```

Now, open three terminal and just do it by sourcing `install/setup.bash` in all terminal respectively ;)

terminal1
```shell
source install/setup.bash
ros2 run control lqr
```

terminal2
```shell
source install/setup.bash
python3 src/jarcar/src/car.py
```

terminal3
```shell
source /opt/ros/foxy/setup.bash
python3 src/jarcar/src/a.py
```

## *Video from our controller*

[![LQR Controller For Car-Like Vehicles](https://img.youtube.com/vi/8At5h5JDbkI/0.jpg)](https://www.youtube.com/watch?v=8At5h5JDbkI)

## **Useful Links**

https://doi.org/10.1109/ChiCC.2016.7553742

https://automaticaddison.com/linear-quadratic-regulator-lqr-with-python-code-example/

## **Citation**

@INPROCEEDINGS{7553742,

  author={Lin, Fengda and Lin, Zijian and Qiu, Xiaohong},

  booktitle={2016 35th Chinese Control Conference (CCC)}, 

  title={LQR controller for car-like robot}, 

  year={2016},

  volume={},

  number={},

  pages={2515-2518},

  abstract={This work studies the trajectory tracking for a car-like robot. Based on the kinematic equations of the mobile robot, an tracking error model is obtained. And then, this nonlinear model is linearized around origin. Based on local linearized model, an optimal controller is designed for the trajectory tracking problem by using optimal linear quadratic (LQ) design approach. The simulation shows the effectiveness of optimal LQR (linear quadratic regulator) controller for the cases where the robot tracks both straight and curve trajectories.},

  keywords={},

  doi={10.1109/ChiCC.2016.7553742},

  ISSN={1934-1768},

  month={July},}



