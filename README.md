### 1. System Model

#### Robot Dynamics

The Euler-Lagrange equations for a two-link robot:
$$
\mathbf{M}(\mathbf{q}, m_{\text{load}}) \ddot{\mathbf{q}} + 
\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}, m_{\text{load}}) \dot{\mathbf{q}} + 
\mathbf{G}(\mathbf{q}, m_{\text{load}}) = 
\boldsymbol{\tau},
$$
where:

- \( q = [q_1, q_2]^T \): Joint angles  
- \( m_{\text{load}} \): Unknown payload mass at the end effector  
- \( M \): Inertia matrix  
- \( C \): Coriolis and centrifugal forces matrix  
- \( G \): Gravitational force vector  