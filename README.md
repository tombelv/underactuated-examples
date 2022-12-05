# underactuated-examples
This repo contains a collection of coding examples for the course Underactuated Robots

## 2R manipulator on pybullet using pinocchio (`pybullet-pinocchio-2R`)

This is a dynamic simulation of a 2R manipulator in a vertical position, it uses pybullet and its GUI for the simulation and pinocchio to compute the kinematic and dynamics quantities relevant for control.

It requires to install 
* pinocchio
* pybullet
which can be install via `pip install pybullet pin`.

The example can be executed by running (from the main repo directory):
```
python pybullet-pinocchio-2R
```
