# Aircraft Velocity Estimation using Newton–Raphson

Estimating aircraft airspeed without relying entirely on a dedicated airspeed sensor is an interesting problem in flight-state estimation.

This project explores an aerodynamic approach where the aircraft's **mass, acceleration, attitude, air density, wing characteristics, and vertical velocity** are used to estimate its total velocity.

The nonlinear aerodynamic equation is solved using the **Newton–Raphson method**.

---

## How It Works

The aerodynamic model starts from the lift relationship:

**L = ½ρV²ACₗ**

where the lift coefficient is approximated as:

**Cₗ = Cₗ₀ + Cₐₗₚₕₐ α**

The angle of attack is estimated from the aircraft attitude and vertical velocity:

**α = θₚ − θ**

where:

**θ = sin⁻¹(Vᵥ cos(θᵣ) / V)**

Combining these relationships with the measured aircraft accelerations gives the nonlinear function:

**f(V) = m·aₓ·sin(θₚ − θ) + m·a_z·cos(θₚ − θ) − ½ρV²A[Cₗ₀ + Cₐₗₚₕₐ(θₚ − θ)]**

The required velocity is the value of **V** for which:

**f(V) = 0**

---

## Solving for Velocity

Because the equation contains velocity inside both the aerodynamic term and the inverse-sine calculation, it cannot be conveniently rearranged into a simple closed-form expression.

Newton–Raphson is therefore used:

**Vₙ₊₁ = Vₙ − f(Vₙ) / f′(Vₙ)**

The derivative **f′(V)** is calculated analytically and implemented directly in the code.

This avoids numerical differentiation and keeps each iteration computationally lightweight.

---

## Sequential Estimation

For flight data, velocity is estimated continuously rather than as isolated measurements.

The previous estimated velocity is therefore used as the initial guess for the next sample:

```python
prev_velocity = velocity
