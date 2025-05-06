# ACM_2DOF_robot

## 1. System Model
### Robot Dynamics
The Euler-Lagrange equations for a two-link robot:

![Robot Dynamics](https://latex.codecogs.com/svg.image?$$\mathbf{M}(\mathbf{q},m_{\text{load}})\ddot{\mathbf{q}}&plus;\mathbf{C}(\mathbf{q},\dot{\mathbf{q}},m_{\text{load}})\dot{\mathbf{q}}&plus;\mathbf{G}(\mathbf{q},m_{\text{load}})=\boldsymbol{\tau},$$)

where:
- $\mathbf{q} = [q_1, q_2]^T$: Joint angles
- $m_{\text{load}}$: Unknown payload mass at the end effector
- $\mathbf{M}$: Inertia matrix
- $\mathbf{C}$: Coriolis and centrifugal forces matrix
- $\mathbf{G}$: Gravitational force vector
