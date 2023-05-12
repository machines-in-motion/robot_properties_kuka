Robot Properties Kuka
---------------------

### What it is

URDF and ROS integration of KUKA robots (iiwa and lwr4)

### Installation

1. Install Pinocchio if you have not done so already.

   The simplest way to do so is using Conda:

   ```
   conda install -c conda-forge pinocchio
   ```

   Alternatively you can use any of the ways listed here: https://stack-of-tasks.github.io/pinocchio/download.html.

2. Install bullet_utils:

  ```
  git clone git@github.com:machines-in-motion/bullet_utils.git
  cd bullet_utils
  pip3 install .
  ```

3. Install robot_properties_kuka:

  ```
  git clone git@gitlab.is.tue.mpg.de:robotics/robot_properties_kuka.git
  cd robot_properties_kuka
  pip3 install .
  ```

### Examples

**Loading KUKA LWR iiwa 14 in PyBullet**

```
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_kuka.iiwaWrapper import IiwaRobot

env = BulletEnvWithGround(p.GUI)
robot = env.add_robot(IiwaRobot)
```


### Authors

- Johannes Pfleging
- Maximilien Naveau
- Sébastien Kleff

### Copyrights

Copyright(c) 2018-2023 Max Planck Gesellschaft, New York University

### License

BSD 3-Clause License

