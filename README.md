# ADCS---Reaction-Wheel-Simulation

Spacecraft attitude control simulation was performed using both reaction wheels and control moment gyroscopes in a pyramid configuration in MATLAB and Simulink. The objective of the simulated control is to point the spacecraft nadir from some initial condition by aligning the body-fixed reference frame with the LVLH reference frame. This is accomplished using error quaternion-based full-state feedback control. The pyramid configuration of the reaction wheels and control moment gyroscopes is shown below:

<img src="/figures/pyramid_config.png">

## Reaction Wheels

The simulation outputs for the reaction wheel control are shown below:

<img src="/figures/rw_w.png">

<img src="/figures/rw_q.png">

<img src="/figures/rw_eul.png">

<img src="/figures/rw_sens_angle.png">

<img src="/figures/wheel_speeds.png">

## Control Moment Gyroscopes

The simulation outputs for the CMG control are shown below:

<img src="/figures/cmg_w.png">

<img src="/figures/cmg_q.png">

<img src="/figures/cmg_eul.png">

<img src="/figures/cmg_sens_angle.png">