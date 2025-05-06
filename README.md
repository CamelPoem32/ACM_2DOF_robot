# ACM_2DOF_robot

## 1. Idea
![manipulators](https://www.universal-robots.com/media/1814006/ur16e_all_3.jpg)
One of the main task of robot is moving of an object from start point to another. And it is possible that load that being moved is unknown and can affect on dynamics of the system - so standart controll won't work. In this situation Adapive one can help


## 2. System Model
### Robot Dynamics
The Euler-Lagrange equations for a two-link robot:

![Robot Dynamics](https://latex.codecogs.com/svg.image?$$\mathbf{M}(\mathbf{q},m_{\text{load}})\ddot{\mathbf{q}}&plus;\mathbf{C}(\mathbf{q},\dot{\mathbf{q}},m_{\text{load}})\dot{\mathbf{q}}&plus;\mathbf{G}(\mathbf{q},m_{\text{load}})=\boldsymbol{\tau},$$)

where:
- $\mathbf{q} = [q_1, q_2]^T$: Joint angles
- $m_{\text{load}}$: Unknown payload mass at the end effector
- $\mathbf{M}$: Inertia matrix
- $\mathbf{C}$: Coriolis and centrifugal forces matrix
- $\mathbf{G}$: Gravitational force vector
