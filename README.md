# Vehicle-Lateral-Optimal-Control

# Linear Quadratic Regulator (LQR)
A system is described by the standard linear state space model:

$$
\dot{x} = Ax + Bu 
$$

$$
y = Cx
$$

The objective is to bring the non-zero initial state to zero in the infinite time horizon.

The cost function takes the quadratic form:

$$
J = \frac{1}{2} \int_{0}^{\infty} (x^T Qx + u^T Ru)dt
$$

(The advantage of using the quadratic form: avoid negative numbers in the area when the state x is negative; a minimum value can always be found)

$$
Q = \begin{bmatrix}
q_1 & 0 & \ldots & 0 \\
0 & q_2 & \ldots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \ldots & q_n
\end{bmatrix}
$$

And \( R \) is:

$$
R = \begin{bmatrix}
r_1 & 0 & \ldots & 0 \\
0 & r_2 & \ldots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \ldots & r_m
\end{bmatrix}
$$

Note from 

$$
{x^TQx} = q_{1}x_{1}^2 + q_{2}x_{2}^2 + \ldots + q_{n}x_{n}^2
$$

where \(Q\) is defined as a matrix with diagonal elements \(q_i\). Where

$$
q_i \geq 0
$$

$$
r_i > 0
$$

for \(i = 1, 2, ... , m\).

\(q_i\) are relative weightings among \(x_i\).

If \(q_1\) is bigger than \(q_2\), there is a higher penalty/price on error in \(x_1\) than in \(x_2\), and control will try to make \(x_1\) smaller than \(x_2\), vice versa. (Pay more attention to \(q_1\))

The same principle applies to

$$
u^TRu = r_1u_1^2 + r_2u_2^2 + \ldots + r_mu_m^2
$$
## LQR General Solution (Raccati Equation)
For a LQR problem defined as

System:

$$
\dot{x} = Ax + Bu 
$$

$$
y = Cx
$$

State feedback:

$$
u = -Kx
$$

The closed loop system is:

$$
\dot{x} = (A - BK)x = A_{cl}x .
$$

Cost function:

$$
J = \frac{1}{2} \int_{0}^{\infty} (x^T Qx + u^T Ru) dt
$$

Control law:

$$
u = -Kx = -R^{-1}B^TPx,
$$

Will bring 𝑥 (∞) → 0 and u (∞) → 0
where 

$$
A^TP + PA + Q = PBR^{-1}B^TP
$$

that is, the optimal control law is a linear feedback of the state vector \( x \), as assumed.

## Dynamic Model of Lateral Vehicle Motion
### "Bicycle" dynamic model:

$$
a_y = \left(\frac{d^2y}{dt^2}\right)_{\text{inertial}} = \dot{v}_y + v_x\dot{\psi}
$$

$$
F_{yf} + F_{yr} = m a_y = m(\dot{v}_y + v_x \ddot{\psi})
$$

$$
l_f F_{yf} - l_r F_{yr} = I_z \dot{\psi}
$$

after considering the steering angle δ .

$$
(F_{yf} \cos(\delta) - F_{xf} \sin(\delta)) + F_{yr} = m(\dot{v}_y + v_x r)
$$

$$
l_f (F_{yf} \cos(\delta) - F_{xf} \sin(\delta)) - l_r F_{yr} = I_z \dot{\psi} = I_z r
$$

### Front and Rear Tire Forces:
$$
F_{yf} = c_f\alpha_f = c_f(\delta - \theta_{vf})
$$

$$
F_{yr} = c_r\alpha_r = c_r(-\theta_{vr})
$$

$$
\theta_{vf} = \tan^{-1}\left(\frac{v_{yf}}{v_{xf}}\right) = \tan^{-1}\left(\frac{v_y + l_f r}{v_x}\right)
$$

$$
\theta_{vr} = \tan^{-1}\left(\frac{v_{yr}}{v_{xr}}\right) = \tan^{-1}\left(\frac{v_y - l_r r}{v_x}\right)
$$
### Lateral and Yaw Dynamics:
$$
\dot{v_y} = \frac{c_f}{m} \left( \delta - \tan^{-1}\left(\frac{v_y + l_f r}{v_x}\right) \right) \cos(\delta) - \frac{c_r}{m} \tan^{-1}\left(\frac{v_y - l_r r}{v_x}\right) - \frac{F_{xf} \sin(\delta)}  - v_x r
$$

$$
\dot{r} = \frac{1}{I_z} \left( l_f c_f \left( \delta - \tan^{-1}\left(\frac{v_y + l_f r}{v_x}\right) \right) \cos(\delta) + l_r c_r \tan^{-1}\left(\frac{v_y - l_r r}{v_x}\right) - l_f F_{xf} \sin(\delta) \right)
$$

after small angle assumptions and re-group by variables:

$$
\cos(\delta) \approx 1
$$

$$
\sin(\delta) \approx 0
$$

$$
\tan^{-1}(\delta) \approx \delta
$$

$$
\dot{v}_y = -\frac{(c_f + c_r)}{mv_x}v_y + \left[\frac{(l_rc_r - l_fc_f)}{mv_x}\right]v_x r + \frac{c_f}{m}\delta
$$

$$
\dot{r} = \frac{l_f c_f - l_r c_r}{I_z v_x}v_y + \left[-\frac{(c_f l_f^2 + c_r l_r^2)}{I_z v_x}\right]r + \frac{l_f c_f}{I_z}\delta
$$

![请添加图片描述](https://img-blog.csdnimg.cn/direct/e218bb8e6ed94f13992b647d5dfc4aaa.png)

</div>

<p align="center">Dynamic Bicycle Model</p>

### Linearized Dynamic Model of Lateral Vehicle Motion


If we use state 

$$
\mathbf{X} =
\begin{bmatrix}
y\\
v_y \\
\psi \\
\dot{\psi}
\end{bmatrix} \ and \ input \ \delta \ ,
$$ 

rewrite in state space model: 

$$
 \mathbf{\dot{X}} = \mathbf{A}\mathbf{X} + \mathbf{B}\delta \ , 
$$ 

it is

$$
\frac{d}{dt}
\begin{bmatrix} 
y\\ 
\ v_y \\ 
\psi 
\\ 
\dot{\psi} 
\end{bmatrix} =
\begin{bmatrix}
0 & 0 & 0 & 0 \\
1 & -\frac{(c_f + c_r)}{m v_x} & 0 & \frac{(l_r c_r - l_f c_f)}{m v_x} - v_x \\
0 & 0 & 0 & 1 \\
0 & \frac{l_f c_f - l_r c_r}{I_z v_x} & 0 & -\frac{(c_f l_f^2 + c_r l_r^2)}{I_z v_x}
\end{bmatrix}
\begin{bmatrix} 
y \\ 
v_y \\ 
\psi \\ 
\dot{\psi} \end{bmatrix} +
\begin{bmatrix}
0 \\ 
\frac{c_f}{m} \\
0 \\
\frac{l_f c_f}{I_z}
\end{bmatrix}
\delta
$$

## Trajectory tracking with LQR
#### Path Coordinates Model (Error dynamics)
For path tracking, it is useful to express the bicycle model with respect to the path function of its length 𝑠 and with the constant longitudinal velocity assumption.
We can choose 

$$
x = \begin{bmatrix} e_{cg} & \dot{e_{cg}} & e_{\theta} & \dot{e_{\theta}} \end{bmatrix}^T
$$

as our system state and $u = \delta$.

- $e_{cg}$: Orthogonal distance of the C.G. to the nearest path waypoint;
- $\dot{e_{cg}}$: Relative speed between vehicle C.G and path;
- $e_{\theta}$: Heading/Yaw difference between vehicle and path, $e_{\theta} = \theta - \theta_p(s)$
- $\dot{e_{\theta}}$: Relative yaw rate between vehicle C.G and path, $e_{\theta} = r - r(s)$ where $r(s) = \dot{\theta(s)}$ is the yaw rate derived from the path.

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/7edf499e4ce14b38b0cd6e526b09a421.png)

</div>

<p align="center">Dynamic Bicycle Model in path coordinates</p>
With the constant longitudinal velocity assumption,

$$
\dot{e_cg} = v_y + v_x \tan(\theta - \theta_p(s)) 
\\ = v_y + v_x \tan(e_{\theta})
$$


Thus, the acceleration of C.G. is:

$$
\ddot{e}_{cg} = (\dot{v}_y + v_xr') - \dot{v}_y(s) \\
= \dot{v}_y + v_x(r - r(s)) 
$$

Convert lateral dynamic to error dynamics:

$$
v_y = \dot{e_cg} - v_x\sin(e_\theta)
$$

$$
\dot{v_y} = \dot{e_cg} - v_x\dot{e}_\theta
$$

$$
\theta = e_\theta + \theta_p (s) 
$$

$$
r = \dot{e}_\theta + r(s) 
$$

$$
\dot{r} = \ddot{e}_\theta + \dot{r}(s)
$$

Now, we have the linear lateral dynamic model. Rewrite it as:

$$ 
\dot{x} = Ax + B_1\delta + B_2 \ r_{des}, \text{ where } x = (e_{cg} \, \dot{e_cg} \, e_\theta \,\dot{e}_\theta)^T. 
$$

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\ 
0 & -\frac{(c_f + c_r)}{mv_x} & \frac{c_f + c_r}{m} & \frac{(l_r c_r - l_f c_f)}{mv_x}  \\
0 & 0 & 1 & 0 \\ 
0 & \frac{l_r c_r - l_f c_f}{I_z v_x} & \frac{l_r c_r - l_f c_f}{I_z} & -\frac{(c_f l_f^2 +  c_r l_r^2)}{I_z v_x} & 0 \\
\end{bmatrix}
, B_1 = \begin{bmatrix}
0 \\
\frac{c_f}{m} \\
0 \\
\frac{l_f c_f}{I_z} \\
\end{bmatrix}
, B_2 = \begin{bmatrix}
0 \\
-\frac{l_f c_r - l_r c_f}{m v} -v\\
0 \\
-\frac{l_f^2 c_f + l_r^2 c_r}{I_z v} \\
\end{bmatrix}
$$

Basic work flow:

- Check the controllability matrix has full rank: $[B_1, A B_1, A^2 B_1, A^3 B_1]$.
- Convert the continuous time system to discrete time.
- 
$$ x(k+1) = A_dx(k) + B_{1d}\delta(k) + B_{2d}\ r_{des}(k) $$

- Use the full state feedback law:

$$
\delta = -Kx = -k_1 e_{cg} - k_2 \dot{e_cg} - k_3 e_\theta - k_4 \dot{e}_\theta.
$$

Apply LQR in this situation, we have

$$ \delta^*(k) = -Kx(k) $$

Where  

$$ 
K = (R + B_d^T P B_d)^{-1} B_d^T P A_d. 
$$

Objective cost function to be minimized by the control is

$$ J = \sum_{k=0}^{\infty} x(k)^T Q x(k) + \delta(k)^T R \delta(k) $$

where P satisfies the matrix difference Riccati equation

$$ P = A_d^T P A_d - A_d^T P B_d(R + B_d^T P B_d)^{-1} B_d^T P A_d + Q $$


##  LQR tuning
Let 

$$  
Q = 
\begin{bmatrix} 
1 & 0 \\
0 & 1 
\end{bmatrix} 
$$ 

, we choose different \( R \):
- When \( R = 10 \):
  - Penalty on control effort is large → Less control effort is used → slower response.
- When \( R = 1 \):
  - Penalty on control effort is small → More control effort is used → faster response.
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/0d79c454f6b14e9fbcae56b97a1f4db3.png)
<p align="center">Position error response to initial condition</p>

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/065025b1e8b949fc8eac4c39652842bf.png)
<p align="center">Control effort</p>

Let \( R = 1 \), we choose different \( Q \):
- When

$$
Q = 
\begin{bmatrix} 
10 & 0 \\ 
0 & 1 
\end{bmatrix}
$$


  - Penalty on position error is greater than penalty on speed error → prioritizes minimizing position error → may result in a faster response in position tracking.
- When

$$
Q = 
\begin{bmatrix} 
1 & 0 \\ 
0 & 1 
\end{bmatrix}
$$


  - Penalty on position error is equal to penalty on speed error → balanced approach → may result in a response that does not prioritize one over the other.

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/be8343eaeb19404fafa8e1a687cd6eff.png)

</div>
<p align="center">Position error response to initial condition</p>



![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/f28885c68cf54e408ec60f458c7bf43a.png)

</div>

<p align="center">Speed error response to initial condition</p>

## Different controller comparison

| Controller name | Cross-track error (Overshoot)           | Planner requirement | Robustness | Suitable Application      |
|-----------------|-----------------------------------------|---------------------|------------|----------------------------|
| Pure Pursuit    | High steady state error (high speed)    | No                  | High       | Slow driving; discontinuous path |
| Stanley Method  | Better than PP but still when speed increase, error increase | Continuous         | Mid        | Smooth high speed; Parking |
| LQR             | Large error in the curvy road, good tracking in straight road | Continuous         | Low        | High way drive             |
| Preview Control | Good tracking performance with the preview information.        | Low                | Mid        | High way and urban drive   |
