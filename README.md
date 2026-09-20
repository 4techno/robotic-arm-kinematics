# Robotic Arm Kinematics Engine

An analytical forward and inverse kinematics (IK) solver with smooth coordinate trajectory interpolation for multi-DOF articulated robotic arms. Designed for embedded stepper controllers communicating via serial step/direction protocols.

## Features

- **Analytical Inverse Kinematics**: Solves joint angles $(\theta_1, \theta_2, \theta_3)$ for targeted Cartesian tool-tip coordinates $(X, Y, \phi)$.
- **Forward Kinematics Verification**: Computes end-effector position using Denavit-Hartenberg (DH) parameter conventions.
- **Workspace Reachability Bounds**: Validates target coordinates against physical link lengths $(L_1, L_2, L_3)$ before sending stepper commands.
- **Trajectory Interpolation**: Linear Cartesian path generator with trapezoidal velocity profiling for jerk-free movement.

## Mathematical Model

For a 2-link/3-link planar arm with link lengths $L_1, L_2$:

$$D = \frac{X^2 + Y^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$

Elbow joint angle ($\theta_2$):
$$\theta_2 = \text{atan2}\left(\pm \sqrt{1 - D^2}, D\right)$$

Base joint angle ($\theta_1$):
$$\theta_1 = \text{atan2}(Y, X) - \text{atan2}(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)$$

Wrist orientation angle ($\theta_3$):
$$\theta_3 = \phi - \theta_1 - \theta_2$$

## Quick Start

```bash
git clone https://github.com/4techno/robotic-arm-kinematics.git
cd robotic-arm-kinematics
python kinematics.py --x 180 --y 120 --phi 0
```

## Example Run

```
[*] Solving Inverse Kinematics for target (X=180.0 mm, Y=120.0 mm, Phi=0.0 deg)...
------------------------------------------------------------
Status              : REACHABLE
Base Angle (Theta 1): 24.3 deg
Elbow Angle(Theta 2): 62.1 deg
Wrist Angle(Theta 3): -86.4 deg
Forward Check Error : 0.002 mm
------------------------------------------------------------
[✓] Solution verified. Ready for serial stepper dispatch.
```

## Integration

Output joint angles translate directly into microstepping pulse counts:
$$\text{Steps}_i = \theta_i \times \frac{\text{Microsteps} \times \text{GearRatio}}{360^\circ}$$

## License

MIT License. Open for robotics control and automated manufacturing research.
