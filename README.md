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

$l_c = 0.5L $ -  position of center of masses

Matrixes comes from equation:

![E_L_equation](https://latex.codecogs.com/svg.image?$$\frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot{\theta}_i}\right)-\frac{\partial\mathcal{L}}{\partial\theta_i}=\tau_i,\quad&space;i=1,2,$$)


Kinetic Energy T

![Kinetic](https://latex.codecogs.com/svg.image?$$T=T_1&plus;T_2.$$)




First Link

$$
x_{c1} = l_c \cos(\theta_1), \quad y_{c1} = l_c \sin(\theta_1), \quad l_c = \frac{L}{2},
$$

$$
\dot{x}_{c1} = -l_c \sin(\theta_1) \dot{\theta}_1, \quad \dot{y}_{c1} = l_c \cos(\theta_1) \dot{\theta}_1,
$$

$$
T_1 = \frac{1}{2} (m l_c^2 + I) \dot{\theta}_1^2, \quad I = \frac{1}{12} m L^2.
$$

Second Link

$$
x_{c2} = L \cos(\theta_1) + l_c \cos(\theta_1 + \theta_2), \quad y_{c2} = L \sin(\theta_1) + l_c \sin(\theta_1 + \theta_2),
$$

$$
\dot{x}_{c2} = -L \sin(\theta_1) \dot{\theta}_1 - l_c \sin(\theta_1 + \theta_2) (\dot{\theta}_1 + \dot{\theta}_2),
$$

$$
\dot{y}_{c2} = L \cos(\theta_1) \dot{\theta}_1 + l_c \cos(\theta_1 + \theta_2) (\dot{\theta}_1 + \dot{\theta}_2),
$$

$$
T_2 = \frac{1}{2} m \left[ L^2 \dot{\theta}_1^2 + l_c^2 (\dot{\theta}_1 + \dot{\theta}_2)^2 + 2 L l_c \dot{\theta}_1 (\dot{\theta}_1 + \dot{\theta}_2) \cos(\theta_2) \right] + \frac{1}{2} I (\dot{\theta}_1 + \dot{\theta}_2)^2.
$$

Total Kinetic Energy

$$
T = \frac{1}{2} \dot{\theta}^T M(\theta) \dot{\theta},
$$

$$
M_{11} = m l_c^2 + I + m (L^2 + l_c^2 + 2 L l_c \cos(\theta_2)) + I,
$$

$$
M_{12} = M_{21} = m (l_c^2 + L l_c \cos(\theta_2)) + I,
$$

$$
M_{22} = m l_c^2 + I.
$$

Potential Energy V

$$
V = m g (l_c + L) \sin(\theta_1) + m g l_c \sin(\theta_1 + \theta_2).
$$

Lagrangian

$$
\mathcal{L} = T - V.
$$

Inertia Matrix M(\theta)

$$
\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = M(\theta) \dot{\theta}, \quad \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{\theta}} \right) = M(\theta) \ddot{\theta} + \dot{M}(\theta) \dot{\theta},
$$

$$
\dot{M} = \frac{\partial M}{\partial \theta_2} \dot{\theta}_2, \quad \frac{\partial M_{11}}{\partial \theta_2} = -2 m L l_c \sin(\theta_2), \quad \frac{\partial M_{12}}{\partial \theta_2} = -m L l_c \sin(\theta_2), \quad \frac{\partial M_{22}}{\partial \theta_2} = 0.
$$

Coriolis and Centrifugal Forces C(\theta, \dot{\theta})

$$
\frac{\partial \mathcal{L}}{\partial \theta_i} = \frac{\partial T}{\partial \theta_i} - \frac{\partial V}{\partial \theta_i},
$$

$$
\frac{\partial T}{\partial \theta_1} = 0, \quad \frac{\partial T}{\partial \theta_2} = -m L l_c \sin(\theta_2) \dot{\theta}_1 (\dot{\theta}_1 + \dot{\theta}_2),
$$

$$
\frac{\partial V}{\partial \theta_1} = m g (l_c + L) \cos(\theta_1) + m g l_c \cos(\theta_1 + \theta_2), \quad \frac{\partial V}{\partial \theta_2} = m g l_c \cos(\theta_1 + \theta_2),
$$

$$
C \dot{\theta} = \dot{M} \dot{\theta} - \begin{bmatrix} \frac{\partial T}{\partial \theta_1} \\ \frac{\partial T}{\partial \theta_2} \end{bmatrix} = \begin{bmatrix} -m L l_c \sin(\theta_2) (2 \dot{\theta}_1 \dot{\theta}_2 + \dot{\theta}_2^2) \\ m L l_c \sin(\theta_2) \dot{\theta}_1^2 \end{bmatrix},
$$

$$
h = m L l_c \sin(\theta_2), \quad C = \begin{bmatrix} -h \dot{\theta}_2 & -h (\dot{\theta}_1 + \dot{\theta}_2) \\ h \dot{\theta}_1 & 0 \end{bmatrix}.
$$

Gravity Vector G(\theta)

$$
G_1 = m g (l_c + L) \cos(\theta_1) + m g l_c \cos(\theta_1 + \theta_2),
$$

$$
G_2 = m g l_c \cos(\theta_1 + \theta_2).
$$