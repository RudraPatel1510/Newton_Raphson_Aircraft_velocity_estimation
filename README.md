# Aircraft Velocity Estimation using Newton–Raphson

A Python-based aerodynamic velocity estimation system that estimates an aircraft's **total airspeed** using flight dynamics, IMU measurements, aircraft parameters, and vertical velocity.

The project formulates the aerodynamic lift equation as a nonlinear function of velocity and solves it using the **Newton–Raphson numerical method**. The implementation is designed for iterative estimation, where the velocity estimated from the previous measurement is used as the initial guess for the next measurement.

---

## Overview

Accurate airspeed estimation is important for aircraft and UAV flight-control systems, particularly when a dedicated airspeed sensor is unavailable or unreliable.

This project estimates total velocity (V) using:

* Aircraft mass
* Forward acceleration
* Vertical acceleration
* Pitch angle
* Roll angle
* Vertical velocity
* Air density
* Wing/reference area
* Zero-angle lift coefficient
* Lift-curve slope

The nonlinear relationship between velocity, angle of attack, and aerodynamic lift is solved using Newton–Raphson iteration.

### Core concept

The aerodynamic lift equation is:

[
L = \frac{1}{2}\rho V^2 A C_L
]

where the lift coefficient is approximated using a linear aerodynamic model:

[
C_L = C_{L0} + C_{\alpha}\alpha
]

The angle of attack is estimated from aircraft pitch and flight-path angle:

[
\alpha = \theta_p-\theta
]

where

[
\theta =
\sin^{-1}
\left(
\frac{V_v\cos(\theta_r)}{V}
\right)
]

Therefore, the nonlinear equation solved by the program is:

[
\boxed{
f(V)=
m a_x\sin(\theta_p-\theta)
+
m a_z\cos(\theta_p-\theta)
--------------------------

\frac{1}{2}\rho V^2A
\left[
C_{L0}+C_\alpha(\theta_p-\theta)
\right]
}
]

The objective is to find:

[
f(V)=0
]

---

# Newton–Raphson Method

The velocity is obtained iteratively using:

[
\boxed{
V_{n+1}
=======

V_n-\frac{f(V_n)}{f'(V_n)}
}
]

The derivative used by the implementation is derived analytically.

Let:

[
\delta=\theta_p-\theta
]

and

[
\theta =
\sin^{-1}
\left(
\frac{V_v\cos(\theta_r)}{V}
\right)
]

Then:

[
\frac{d\theta}{dV}
==================

-\frac{V_v\cos(\theta_r)}
{V^2
\sqrt{
1-
\left(
\frac{V_v\cos(\theta_r)}{V}
\right)^2
}}
]

The derivative of the complete function is:

[
\boxed{
\begin{aligned}
f'(V)=&
\left[
-m a_x\cos(\delta)
+
m a_z\sin(\delta)
\right]
\frac{d\theta}{dV}
\
&-\rho VA
\left[
C_{L0}+C_\alpha\delta
\right]
+
\frac{1}{2}\rho V^2A C_\alpha
\frac{d\theta}{dV}
\end{aligned}
}
]

This analytical derivative avoids numerical differentiation and reduces the computational overhead of each Newton–Raphson iteration.

---

# Project Workflow

The implementation follows this process:

```text
        Flight Dataset
              │
              ▼
      Read sensor/aircraft data
              │
              ▼
      Initial velocity estimate
              │
              ▼
       Calculate θ(V)
              │
              ▼
      Calculate α = θp - θ
              │
              ▼
       Calculate f(V)
              │
              ▼
      Calculate f'(V)
              │
              ▼
     Newton–Raphson Update
              │
              ▼
        Convergence?
          /       \
        No         Yes
        │           │
        └───┐       ▼
            │   Estimated V
            ▼
      Next iteration
```

For sequential flight data, the estimated velocity from the previous row is used as the initial guess for the next row:

```python
prev_velocity = velocity
```

This is particularly useful for aircraft applications because velocity generally changes continuously rather than jumping randomly between measurements.

---

# Project Structure

```text
.
├── README.md
├── velocity_estimation.py
└── Datasets/
    ├── velocity_data_2.csv
    └── velocity_results_2.csv
```

The exact filenames can be changed according to the repository structure.

---

# Input Dataset

The program reads the input data from:

```python
df = pd.read_csv('Datasets/velocity_data_2.csv')
```

The dataset contains the parameters required by the aerodynamic model.

### Required Parameters

| Parameter             | Description                                    | Unit          |
| --------------------- | ---------------------------------------------- | ------------- |
| `mass`                | Aircraft mass                                  | kg            |
| `ax`                  | Forward/body-axis acceleration                 | m/s²          |
| `az`                  | Vertical/body-axis acceleration                | m/s²          |
| `Current Pitch (deg)` | Aircraft pitch angle                           | degree        |
| `Current Roll (deg)`  | Aircraft roll angle                            | degree        |
| `rho`                 | Air density                                    | kg/m³         |
| `A`                   | Reference/wing area                            | m²            |
| `cl0`                 | Zero-angle lift coefficient                    | dimensionless |
| `C_alpha`             | Lift-curve slope                               | rad⁻¹         |
| `vertical_speed`      | Vertical velocity                              | m/s           |
| `total_speed`         | Reference/initial velocity used by the dataset | m/s           |

---

# Output

The program creates a new DataFrame containing the estimated velocity and number of Newton–Raphson iterations.

Two additional columns are generated:

```text
calculated_velocity
iterations
```

The resulting dataset is saved using:

```python
result_df.to_csv(
    'Datasets/velocity_results_2.csv',
    index=False
)
```

Example:

| total_speed | calculated_velocity | iterations |
| ----------: | ------------------: | ---------: |
|       55.00 |               54.98 |          3 |
|       55.12 |               55.11 |          2 |
|       55.25 |               55.24 |          2 |

The exact values depend on the input dataset and aerodynamic parameters.

---

# Implementation

The implementation is divided into two primary functions.

## 1. `fvfdv()`

```python
def fvfdv(
    V, m, ax, az, theta_p, rho, A,
    CL0, C_alpha, Vv, theta_r
):
```

This function evaluates:

[
f(V)
]

and

[
f'(V)
]

for the current velocity estimate.

### Angle calculation

```python
arg = (Vv * math.cos(theta_r)) / V
```

followed by:

```python
theta = math.asin(arg)
```

The argument is constrained to the valid domain of the inverse sine function:

```python
arg = max(min(arg, 1.0), -1.0)
```

### Angle difference

```python
delta = theta_p - theta
```

Then:

```python
s = math.sin(delta)
c = math.cos(delta)
```

are calculated once and reused.

### Derivative of theta

```python
dtheta_dV = -(Vv * math.cos(theta_r)) / (
    V**2 * denom
)
```

This represents:

[
\frac{d\theta}{dV}
]

### Function evaluation

```python
fv = (
    m * ax * s
    + m * az * c
    - 0.5 * rho * V**2 * A
      * (CL0 + C_alpha * delta)
)
```

### Derivative evaluation

```python
fdv = (
    (-m * ax * c + m * az * s) * dtheta_dV
    - rho * V * A * (CL0 + C_alpha * delta)
    + 0.5 * rho * V**2 * A
      * C_alpha * dtheta_dV
)
```

---

# 2. `nrv()`

The `nrv()` function performs the Newton–Raphson iterations.

```python
def nrv(
    V0, tol, max_iter,
    m, ax, az,
    theta_p, rho, A,
    CL0, C_alpha,
    Vv, theta_r
):
```

The initial estimate is:

```python
V = V0
```

At every iteration:

```python
fv, fdv = fvfdv(...)
```

are calculated and the Newton–Raphson update is performed:

```python
V_new = V - fv / fdv
```

Convergence is checked using:

```python
if abs(V_new - V) < tol:
```

The algorithm stops when the change in velocity becomes smaller than the specified tolerance.

---

# Sequential Velocity Estimation

One of the important features of this implementation is the use of the previous velocity estimate as the next initial guess.

```python
prev_velocity = df.loc[0, "total_speed"]
```

For every subsequent measurement:

```python
velocity, count = nrv(
    V0=prev_velocity,
    ...
)
```

After convergence:

```python
prev_velocity = velocity
```

This creates a sequential estimation process:

[
V_{k-1}
\rightarrow
V_{k,0}
\rightarrow
V_k
]

where (V_{k,0}) is the initial guess for the current measurement.

This approach is especially useful for aircraft and UAV applications because the actual velocity normally changes gradually between consecutive measurements.

---

# Numerical Safeguards

The implementation includes several protections against numerical problems.

### 1. Inverse sine domain protection

Since:

[
-1 \leq \sin^{-1}(x) \leq 1
]

the argument is clamped:

```python
arg = max(min(arg, 1.0), -1.0)
```

### 2. Near-zero derivative

Newton–Raphson requires division by (f'(V)).

Therefore:

```python
if abs(fdv) < 1e-12:
```

prevents division by a nearly zero derivative.

### 3. Maximum iteration limit

The solver is limited by:

```python
max_iter=10
```

to prevent an unsuccessful solution from running indefinitely.

### 4. Convergence tolerance

The default convergence criterion is:

```python
tol=1e-6
```

---

# Requirements

Python 3.x is recommended.

Install the required dependencies:

```bash
pip install numpy pandas
```

The mathematical calculations use Python's built-in:

```python
math
```

module.

---

# Running the Project

Clone the repository:

```bash
git clone <YOUR_REPOSITORY_URL>
cd <YOUR_REPOSITORY_NAME>
```

Install dependencies:

```bash
pip install pandas
```

Run the Python program:

```bash
python velocity_estimation.py
```

Make sure the input dataset exists at:

```text
Datasets/velocity_data_2.csv
```

The resulting file will be generated at:

```text
Datasets/velocity_results_2.csv
```

---

# Computational Characteristics

The implementation uses an **analytical derivative** rather than finite-difference approximation.

Therefore, each Newton–Raphson iteration requires:

* Evaluation of the nonlinear aerodynamic equation
* Evaluation of its analytical derivative
* One Newton–Raphson update

The method does not require a separate numerical derivative calculation.

For real-time embedded implementation, computational cost can potentially be reduced further by:

* Reusing trigonometric results
* Avoiding unnecessary recalculation of constant aircraft parameters
* Using optimized trigonometric implementations
* Limiting iterations based on convergence behaviour
* Using the previous velocity estimate as the next initial guess

The mathematical model can therefore serve as the Python reference implementation before migration to an embedded platform such as an ESP32.

---

# Applications

This approach can be useful for:

* UAV airspeed estimation
* Aircraft flight-state estimation
* Sensor-fusion systems
* GPS/GNSS-assisted velocity estimation
* GPS-denied navigation research
* Embedded flight-control systems
* Real-time aerodynamic state estimation
* Experimental aircraft instrumentation

---

# Limitations

The current implementation assumes a simplified aerodynamic model:

[
C_L=C_{L0}+C_\alpha\alpha
]

This is a linear approximation and may not accurately represent the aircraft at high angles of attack, stall conditions, or highly nonlinear aerodynamic regimes.

Accuracy also depends on:

* IMU measurement quality
* Pitch and roll estimation
* Vertical velocity accuracy
* Air-density estimation
* Correct acceleration coordinate/sign conventions
* Accuracy of (C_{L0})
* Accuracy of (C_\alpha)
* Validity of the aerodynamic model

The Newton–Raphson method itself does not guarantee convergence for every possible initial guess. However, using a physically reasonable initial estimate and the previous velocity as the next initial guess significantly improves practical convergence for continuously sampled flight data.

---

# Future Work

Possible extensions include:

* ESP32 implementation
* Real-time IMU integration
* GNSS/IMU sensor fusion
* Adaptive aerodynamic coefficients
* Extended Kalman Filter integration
* Robust Newton–Raphson safeguards
* Automatic handling of invalid velocity domains
* Real-time velocity plotting
* Comparison against calibrated airspeed sensors
* Testing with real flight logs
* Optimization of trigonometric calculations for embedded hardware

---

# Author

Developed as an aerodynamic velocity-estimation research and development project, with the objective of implementing a computationally efficient nonlinear velocity estimator suitable for future embedded aircraft/UAV applications.

---

## License

Add the appropriate license for your repository, such as MIT, Apache-2.0, or a proprietary/research license.
