# Pose estimation

## Pose definition

The Thymio robot in our use case has 3 DOF, two translational and one rotational : $[x, y, \theta]^T$. The goal is to find the change in position over one time step :
$$
\begin{equation}
\begin{bmatrix}
    x_+ \\
    y_+ \\
    \theta_+
\end{bmatrix} =
\begin{bmatrix}
    x \\
    y \\
    \theta
\end{bmatrix} +
\begin{bmatrix}
    \Delta x \\
    \Delta y \\
    \Delta \theta
\end{bmatrix}
\end{equation}
$$

## Linear integration

### Unit conversion

First, we solve the simple case of linear motion in the $x$ direction. From the Thymio cheat sheet we get the constants :
$$
\begin{equation}
\begin{aligned}
\Delta t &= \frac{1}{100 \text{ Hz}} = \frac{10}{1000} \text{ s} \\
v_{mm/s} &= 20 \text{ cm/s} = 200 \text{ mm/s} \\
v_{lsb/s} &= 500 \text{ lsb/s}
\end{aligned}
\end{equation}
$$

To have a large enough full scale range of $\approx \pm 2^{15} = \pm 32'768 \approx \pm 32 \text{ m}$, the initial increment estimation is computed in $\text{mm}$ :
$$
\begin{align}
\Delta x_{mm} &= v_{lsb/s} \cdot \frac{v_{mm/s}}{v_{lsb/s}} \cdot \Delta t \\
&= v_{lsb/s} \cdot \frac{200}{500} \cdot \frac{10}{1000} \\
&= v_{lsb/s} \cdot \frac{2}{500} \\
\end{align}
$$
However, because of fixed point arithmetic, only the speeds $250$ and $500$ can be differentiated. They will respectively give increments of $\pm 1$ and $\pm 2$ on the position estimation, so almost $8$ bits of precision are lost.

### Full bit precision

To take into acount those lost $8$ bits, we must estimate the increment in $\mu m$ :
$$
\begin{equation}
\Delta x_{\mu m} = 1000 \cdot \Delta x_{mm} = v_{lsb/s} \cdot \frac{20}{5} = v_{lsb/s} \cdot 4
\end{equation}
$$

After experimentation and calibration, it turns out that the given speeds in $mm/s$ and $lsb/s$ are not exact. The calibrated integration is closer to :
$$
\begin{equation}
\Delta x_{\mu m} = v_{lsb/s} \cdot 3.1254 = v_{lsb/s} \cdot \frac{31'254}{10'000}
\end{equation}
$$

Which can be done without losing any bits of precision with a `math.muldiv` operation, which performs the computation in a 32 bits register.

## Curve integration

### Change in heading

When moving one time step, the robot advance by $\Delta x_L$ and $\Delta x_R$ on each wheel. The wheel pitch $p$ being constant, this means that they are two arc length of two concentric circles of radiuses $p + r$ and $r$. Assuming $\Delta x_L$ is the interior arc and $\Delta x_R$ the exterior arc, from the definition of the radian $\alpha = d / r$, we can solve the increment in angle that $\Delta x_L$ and $\Delta x_R$ describe :
$$
\begin{align}
\Delta \theta = \frac{\Delta x_L}{r} &= \frac{\Delta x_R}{p + r} \\
\Rightarrow \frac{r}{\Delta x_L} &= \frac{p + r}{\Delta x_R} \\
\Rightarrow r &= \frac{p \cdot \Delta x_L}{\Delta x_R - \Delta x_L} \\
\Rightarrow \Delta \theta &= \frac{\Delta x_R - \Delta x_L}{p}
\end{align}
$$

### Change in position

From $\theta_+ = \theta + \Delta \theta$, we can determine the increments $\Delta x$ and $\Delta y$ using the traveled arc length $\Delta s$ and the radius $R$ around the instantaneous center of curvature (**ICC**) :
$$
\begin{align}
\Delta s &= \frac{\Delta x_L + \Delta x_R}{2} \\
R &= \frac{\Delta s}{\Delta \theta} = \frac{\Delta x_L + \Delta x_R}{2 \cdot \Delta \theta} \\
\Delta x &= R \cdot \left(\sin\theta_+ - \sin \theta\right) \\
\Delta y &= -R \cdot \left(\cos\theta_+ - \cos \theta\right)
\end{align}
$$

### Straight line special case

Developping further the expression to compute $R$, we get :
$$
\begin{equation}
R = \frac{\Delta x_L + \Delta x_R}{2 \cdot \Delta \theta} = \frac{p \cdot (\Delta x_L + \Delta x_R)}{2 \cdot (\Delta x_R - \Delta x_L)}
\end{equation}
$$

When going in a straight line, $\Delta x_R = \Delta x_L$, and a division by $0$ appear, which make the expression blow up to infinity. To avoid this, under a certain threshold, another set of equations must be used :
$$
\begin{align}
\Delta x &= \Delta s \cdot \cos(\theta) \\
\Delta y &= \Delta s \cdot \sin(\theta) \\
\Delta \theta &= 0
\end{align}
$$

This threshold must garantee that the computed radius $R$ can be represented in a 16 bit signed integer :
$$
\begin{align}
|R| &< 2^{15} \\
\frac{p}{2} \cdot \left|\frac{\Delta x_L + \Delta x_R}{\Delta x_R - \Delta x_L}\right| &< 2^{15} \\
\left|\frac{\Delta x_L + \Delta x_R}{\Delta x_R - \Delta x_L}\right| &< \frac{2^{16}}{p}
\end{align}
$$

### Mid-point rule

For numerical integration and stability, a simpler and cheaper solution is to use the **mid-point rule** also known as **Runge–Kutta 2** (**RK2**). The idea is that the path does not perfectly follow a circle, hence the best approximation is at the mid-point heading :
$$
\begin{align}
\theta_{mid} &= \theta + \frac{\Delta \theta}{2} \\
\Delta x &\approx \Delta s \cdot \cos \theta_{mid} \\
\Delta y &\approx \Delta s \cdot \sin \theta_{mid}
\end{align}
$$

This integrates the differential-drive unicycle model :
$$
\begin{align}
\dot x &= v \cdot \cos \theta \\
\dot y &= v \cdot \sin \theta \\
\dot \theta &= \omega
\end{align}
$$

### Angle integration

We computed earlier that we need to find the following angle increment :
$$
\begin{equation}
\Delta \theta = \frac{\Delta x_R - \Delta x_L}{p} \, rad
\end{equation}
$$

On the Thymio, angles use a fixed point representation in the range $[-\pi, \pi[$ mapped to $[-2^{15}, 2^{15}[$. So one radian is equal to :
$$
1 \, rad = \frac{2^{15}}{\pi} \approx 10'430
$$

If we try to deduce a constant with $p \approx 95'000 \, \mu m$ to compute angles in these units, we get :
$$
\begin{equation}
\Delta \theta = \frac{\Delta x_R - \Delta x_L}{p} \, rad = (\Delta x_R - \Delta x_L) \cdot \frac{2^{15}}{\pi \cdot p} \approx (\Delta x_R - \Delta x_L) \cdot \frac{1098}{10000}
\end{equation}
$$

Here again, two variables can be used to not lose any increment due to integer rounding. $|\Delta x_R - \Delta x_L| \approx 3$, so using only this will lose incrments $1$ to $3$.

## Controller

### Astolfi

The pose estimation is perfectly suited for an Astolfi controller, with the change of coordiates :
$$
\begin{align}
\rho &= \sqrt{\Delta x^2 + \Delta y^2} \\
\alpha &= \text{atan2}(\Delta y, \Delta x) - \theta \\
\beta &= \Delta \theta -\alpha
\end{align}
$$

Here $\rho$ is the distance to the target, $\alpha$ the robot to target heading error, and $\beta$ the final target heading error. The control law is :
$$
\begin{align}
v &= k_\rho \cdot \rho \\
\omega &= k_\alpha \cdot \alpha + k_\beta \cdot \beta \\
k_\rho &> 0, k_\beta < 0, k_\alpha - k_\rho > 0
\end{align}
$$

This controller offer a nice spline motion, but it gets numerically unstable as the robot reach the goal. When $(\Delta x, \Delta y) \rightarrow (0, 0)$, the function $\text{atan2}$ tends to jump around, and the angles $\alpha$ and $\beta$ tends to fight for angular velocity.

### Modified controller

To fix this, we need a new framework. Lets only take the distance error $e$ and the the robot to target heading error $\varepsilon$ to release one constraint :
$$
\begin{align}
e &= \sqrt{\Delta x^2 + \Delta y^2} \\
\varepsilon &= \text{atan2}(\Delta y, \Delta x) - \theta
\end{align}
$$

Then let the controller be defined as :
$$
\begin{align}
v &= k_e \cdot e \cdot \frac{|\varepsilon|}{\pi} \\
\omega &= k_\omega \cdot \varepsilon \cdot \min(e, e_{\max}) \\
k_e &> 0, k_\omega < 0
\end{align}
$$

This way, the $v$ tends to $0$ if the robot is not heading in the right direction, and otherwise act as a classic P controller. $\omega$ also tends to $0$ under a threshold $e_{\max}$.
