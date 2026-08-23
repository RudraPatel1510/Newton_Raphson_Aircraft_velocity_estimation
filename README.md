# Aircraft Velocity Estimation using Newton–Raphson

A Python implementation for estimating an aircraft's **total airspeed** from aerodynamic and flight-state measurements using the **Newton–Raphson numerical method**.

The project is intended as a reference implementation for future **real-time embedded deployment on ESP32**.

---

## Overview

The velocity is estimated by solving a nonlinear aerodynamic lift equation:

$$
f(V)=
m a_x \sin(\theta_p-\theta)
+
m a_z \cos(\theta_p-\theta)
-
\frac{1}{2}\rho V^2 A
\left[
C_{L0}+C_{\alpha}(\theta_p-\theta)
\right]
$$

where the flight-path angle is estimated as:

$$
\theta =
\sin^{-1}
\left(
\frac{V_v\cos(\theta_r)}{V}
\right)
$$

The required velocity is obtained by solving:

$$
f(V)=0
$$

using Newton–Raphson:

$$
V_{n+1}
=
V_n-
\frac{f(V_n)}{f'(V_n)}
$$

---

## Aerodynamic Model

The lift coefficient is modeled using the linear approximation:

$$
C_L=C_{L0}+C_{\alpha}\alpha
$$

where:

$$
\alpha=\theta_p-\theta
$$

The derivative of the flight-path angle is:

$$
\frac{d\theta}{dV}
=
-\frac{V_v\cos(\theta_r)}
{V^2
\sqrt{
1-
\left(
\frac{V_v\cos(\theta_r)}{V}
\right)^2
}}
$$

The analytical derivative $f'(V)$ is implemented directly in the code, avoiding numerical differentiation.

---

## Input Parameters

The algorithm uses:

| Parameter | Description | Unit |
|---|---|---|
| `mass` | Aircraft mass | kg |
| `ax` | Forward acceleration | m/s² |
| `az` | Vertical acceleration | m/s² |
| `Current Pitch (deg)` | Pitch angle | ° |
| `Current Roll (deg)` | Roll angle | ° |
| `rho` | Air density | kg/m³ |
| `A` | Reference/wing area | m² |
| `cl0` | Zero-angle lift coefficient | - |
| `C_alpha` | Lift-curve slope | rad⁻¹ |
| `vertical_speed` | Vertical velocity | m/s |

---

## Newton–Raphson Implementation

The solver calculates both the function and its analytical derivative:

```python
fv, fdv = fvfdv(...)
