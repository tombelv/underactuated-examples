# underactuated-examples
This repo contains a collection of coding examples for the course Underactuated Robots

## `floating-base-dynamics`

Contains two simple examples in which the floating base dynamics is computed using MATLAB Symbolic Math Toolbox.
It also shows the derivation of the Centroidal Dynamics and the CMM for a 2-link robot and the constrained reduced model for the cart-pendulum system.

### Requirements
These are Live Scripts that require MATLAB and the Symbolic Math Toolbox.

## `pybullet-pinocchio-2R`

This is a dynamic simulation of a 2R manipulator in a vertical position, it uses pybullet and its GUI for the simulation and pinocchio to compute the kinematic and dynamics quantities relevant for control.

### Requirements
It requires to install 
* pinocchio
* pybullet

which can be install via 
```
pip install pybullet pin
```
### Execution
The example can be executed by running (from the main repo directory):
```
python pybullet-pinocchio-2R
```


## `multiple-shooting-SRB`
Example of multiple shooting trajectory optimization for the balancing of a planar single rigid body using contact forces.

### Requirements

* MATLAB
* CasADi (MATLAB interface)

The CasADi directory must be placed somewhere in the MATLAB path or can be imported directly in the code. Here it is assumed that a `casadi-matlab` directory is present in the parent directory.

### Execution
Simply run the `srb_to.m` script from MATLAB.