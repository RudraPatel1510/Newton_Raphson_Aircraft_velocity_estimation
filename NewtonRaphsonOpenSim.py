import math
import pandas as pd


def fvfdv(V, m, ax, az, theta_p, rho, A, CL0, C_alpha, Vv, theta_r):
    arg = (Vv * math.cos(theta_r)) / V
    arg = max(min(arg, 1.0), -1.0)  
    theta = math.asin(arg)
    delta = theta_p - theta

    s = math.sin(delta)
    c = math.cos(delta)

    denom = math.sqrt(1 - arg**2)
    dtheta_dV = -(Vv * math.cos(theta_r)) / (V**2 * denom)

    fv = (m * ax * s +
          m * az * c -
          0.5 * rho * V**2 * A * (CL0 + C_alpha * delta))

    fdv = ((-m * ax * c + m * az * s) * dtheta_dV
           - rho * V * A * (CL0 + C_alpha * delta)
           + 0.5 * rho * V**2 * A * C_alpha * dtheta_dV)

    return fv, fdv


def nrv(V0, tol, max_iter, m, ax, az, theta_p, rho, A, CL0, C_alpha, Vv, theta_r):
    V = V0
    for i in range(max_iter):
        fv, fdv = fvfdv(V, m, ax, az, theta_p, rho, A, CL0, C_alpha, Vv, theta_r)

        if abs(fdv) < 1e-12:  
            print("Derivative near zero, stopping.")
            return V, i+1

        V_new = V - fv / fdv

        if abs(V_new - V) < tol:
            return V_new, i+1 

        V = V_new

    print("Did not converge within max iterations.")
    return V, i+1


# Load CSV
df = pd.read_csv('Datasets/velocity_data_2.csv')

calculated_velocities = []
iterations = []
prev_velocity = df.loc[0, "total_speed"]  # start with first row's V0

for i, row in df.iterrows():
    velocity, count = nrv(
        V0=prev_velocity,
        tol=1e-6,
        max_iter=10,
        m=row["mass"],
        ax=row["ax"],
        az=row["az"],
        theta_p=math.radians(row["Current Pitch (deg)"]),
        rho=row["rho"],
        A=row["A"],
        CL0=row["cl0"],
        C_alpha=row["C_alpha"],
        Vv=row["vertical_speed"],
        theta_r=math.radians(row["Current Roll (deg)"])
    )
    calculated_velocities.append(velocity)
    iterations.append(count)
    prev_velocity = velocity  # use current result as next initial guess

# Add new column to dataframe
result_df = df.copy()
result_df["calculated_velocity"] = calculated_velocities
result_df["iterations"] = iterations
# Save back to CSV (overwrite or create new file)
result_df.to_csv('Datasets/velocity_results_2.csv', index=False)
