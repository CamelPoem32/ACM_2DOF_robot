# ACM_2DOF_robot

## 1. System Model
### Robot Dynamics
The Euler-Lagrange equations for a two-link robot:
\[
\mathbf{M}(\mathbf{q}, m_{\text{load}}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}, m_{\text{load}}) \dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}, m_{\text{load}}) = \boldsymbol{\tau},
]\
where:
- $\mathbf{q} = [q_1, q_2]^T$: Joint angles
- $m_{\text{load}}$: Unknown payload mass at the end effector
- $\mathbf{M}$: Inertia matrix
- $\mathbf{C}$: Coriolis and centrifugal forces matrix
- $\mathbf{G}$: Gravitational force vector
