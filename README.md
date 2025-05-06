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

![lc](https://latex.codecogs.com/svg.image?$\quad&space;l_c=\frac{L}{2}$-position&space;of&space;center&space;of&space;masses)

Matrixes comes from equation:

![E_L_equation](https://latex.codecogs.com/svg.image?$$\frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot{\theta}_i}\right)-\frac{\partial\mathcal{L}}{\partial\theta_i}=\tau_i,\quad&space;i=1,2,$$)


Kinetic Energy T

![Kinetic](https://latex.codecogs.com/svg.image?$$T=T_1&plus;T_2.$$)




First Link

![link1](https://latex.codecogs.com/svg.image?$$x_{c1}=l_c\cos(\theta_1),\quad&space;y_{c1}=l_c\sin(\theta_1)$$$$\dot{x}_{c1}=-l_c\sin(\theta_1)\dot{\theta}_1,\quad\dot{y}_{c1}=l_c\cos(\theta_1)\dot{\theta}_1,$$$$T_1=\frac{1}{2}(m&space;l_c^2&plus;I)\dot{\theta}_1^2,\quad&space;I=\frac{1}{12}m&space;L^2.$$)

Second Link

![link2](https://latex.codecogs.com/svg.image?$$x_{c2}=L\cos(\theta_1)&plus;l_c\cos(\theta_1&plus;\theta_2),\quad&space;y_{c2}=L\sin(\theta_1)&plus;l_c\sin(\theta_1&plus;\theta_2),$$$$\dot{x}_{c2}=-L\sin(\theta_1)\dot{\theta}_1-l_c\sin(\theta_1&plus;\theta_2)(\dot{\theta}_1&plus;\dot{\theta}_2),$$$$\dot{y}_{c2}=L\cos(\theta_1)\dot{\theta}_1&plus;l_c\cos(\theta_1&plus;\theta_2)(\dot{\theta}_1&plus;\dot{\theta}_2),$$$$T_2=\frac{1}{2}m\left[L^2\dot{\theta}_1^2&plus;l_c^2(\dot{\theta}_1&plus;\dot{\theta}_2)^2&plus;2&space;L&space;l_c\dot{\theta}_1(\dot{\theta}_1&plus;\dot{\theta}_2)\cos(\theta_2)\right]&plus;\frac{1}{2}I(\dot{\theta}_1&plus;\dot{\theta}_2)^2.$$)

Total Kinetic Energy

![Total](https://latex.codecogs.com/svg.image?$$T=\frac{1}{2}\dot{\theta}^T&space;M(\theta)\dot{\theta},$$$$M_{11}=m&space;l_c^2&plus;I&plus;m(L^2&plus;l_c^2&plus;2&space;L&space;l_c\cos(\theta_2))&plus;I,$$$$M_{12}=M_{21}=m(l_c^2&plus;L&space;l_c\cos(\theta_2))&plus;I,$$$$M_{22}=m&space;l_c^2&plus;I.$$)

Potential Energy V

![Potential](https://latex.codecogs.com/svg.image?$$V=m&space;g(l_c&plus;L)\sin(\theta_1)&plus;m&space;g&space;l_c\sin(\theta_1&plus;\theta_2).$$)

Lagrangian

![L](https://latex.codecogs.com/svg.image?$\mathcal{L}=T-V.$)

Inertia Matrix M

![M](https://latex.codecogs.com/svg.image?$$\frac{\partial\mathcal{L}}{\partial\dot{\theta}}=M(\theta)\dot{\theta},\quad\frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot{\theta}}\right)=M(\theta)\ddot{\theta}&plus;\dot{M}(\theta)\dot{\theta},$$$$\dot{M}=\frac{\partial&space;M}{\partial\theta_2}\dot{\theta}_2,\quad\frac{\partial&space;M_{11}}{\partial\theta_2}=-2&space;m&space;L&space;l_c\sin(\theta_2),\quad\frac{\partial&space;M_{12}}{\partial\theta_2}=-m&space;L&space;l_c\sin(\theta_2),\quad\frac{\partial&space;M_{22}}{\partial\theta_2}=0.$$)

Coriolis and Centrifugal Forces C

![M1](https://latex.codecogs.com/svg.image?$$\frac{\partial\mathcal{L}}{\partial\theta_i}=\frac{\partial&space;T}{\partial\theta_i}-\frac{\partial&space;V}{\partial\theta_i},$$$$\frac{\partial&space;T}{\partial\theta_1}=0,\quad\frac{\partial&space;T}{\partial\theta_2}=-m&space;L&space;l_c\sin(\theta_2)\dot{\theta}_1(\dot{\theta}_1&plus;\dot{\theta}_2),$$$$\frac{\partial&space;V}{\partial\theta_1}=m&space;g(l_c&plus;L)\cos(\theta_1)&plus;m&space;g&space;l_c\cos(\theta_1&plus;\theta_2),\quad\frac{\partial&space;V}{\partial\theta_2}=m&space;g&space;l_c\cos(\theta_1&plus;\theta_2),$$)

$$
C \dot{\theta} = \begin{bmatrix} -m L l_c \sin(\theta_2) (2 \dot{\theta}_1 \dot{\theta}_2 + \dot{\theta}_2^2) \\ m L l_c \sin(\theta_2) \dot{\theta}_1^2 \end{bmatrix},
$$

$$
h = m L l_c \sin(\theta_2), \quad C = \begin{bmatrix} -h \dot{\theta}_2 & -h (\dot{\theta}_1 + \dot{\theta}_2) \\ h \dot{\theta}_1 & 0 \end{bmatrix}.
$$


Gravity Vector G

![G](https://latex.codecogs.com/svg.image?$$G_1=m&space;g(l_c&plus;L)\cos(\theta_1)&plus;m&space;g&space;l_c\cos(\theta_1&plus;\theta_2),$$$$G_2=m&space;g&space;l_c\cos(\theta_1&plus;\theta_2).$$)