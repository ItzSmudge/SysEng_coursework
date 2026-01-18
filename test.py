import numpy as np
import scipy.integrate

# Your parameters
M = 0.5
m = 0.2
l = 0.3
b = 0.1
I = 0.006
g = 9.81
denom = I*(M + m) + M*m*l**2

def dynamics(t, state, u=0):
    x, x_dot, theta, theta_dot = state
    
    x_ddot = (-(I + m*l**2)*b*x_dot + m**2 * l**2 * g * theta + (I + m*l**2)*u) / denom
    theta_ddot = (-m*l*b*x_dot + m*l*g*(M + m)*theta + m*l*u) / denom
    
    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

# Initial state
state = np.array([0.0, 0.0, 0.2, 0.0])

print("Testing integration methods:")
print(f"Initial state: {state}")

# Test 1: Manual Euler with small timestep
print("\n--- Test 1: Euler method, dt=0.001 ---")
dt = 0.001
state_euler = state.copy()
for i in range(10):
    derivs = dynamics(0, state_euler, 0)
    state_euler = state_euler + dt * derivs
    print(f"Step {i+1}: theta={state_euler[2]:.6f}, theta_dot={state_euler[3]:.6f}")
    if np.any(np.abs(state_euler) > 100):
        print("EXPLOSION!")
        break

# Test 2: solve_ivp with RK45
print("\n--- Test 2: solve_ivp RK45, t=0 to 0.01 ---")
state_ivp = state.copy()
for i in range(10):
    sol = scipy.integrate.solve_ivp(
        lambda t, y: dynamics(t, y, 0),
        [0, 0.001],
        state_ivp,
        method='RK45'
    )
    state_ivp = sol.y[:, -1]
    print(f"Step {i+1}: theta={state_ivp[2]:.6f}, theta_dot={state_ivp[3]:.6f}, status={sol.status}")
    if np.any(np.abs(state_ivp) > 100):
        print("EXPLOSION!")
        break

# Test 3: solve_ivp continuous
print("\n--- Test 3: solve_ivp continuous, t=0 to 0.1 ---")
sol_continuous = scipy.integrate.solve_ivp(
    lambda t, y: dynamics(t, y, 0),
    [0, 0.1],
    state,
    method='RK45',
    dense_output=True,
    max_step=0.001
)
print(f"Status: {sol_continuous.status}")
print(f"Message: {sol_continuous.message}")
print(f"Final state: {sol_continuous.y[:, -1]}")
print(f"Number of evaluations: {sol_continuous.nfev}")

# Test 4: Check what happens at t=5 seconds
print("\n--- Test 4: Full simulation to 5 seconds ---")
sol_full = scipy.integrate.solve_ivp(
    lambda t, y: dynamics(t, y, 0),
    [0, 5.0],
    state,
    method='RK45',
    max_step=0.001,
    dense_output=True
)
print(f"Status: {sol_full.status}")
print(f"Message: {sol_full.message}")

# Sample at regular intervals
times = np.linspace(0, 5, 11)
states = sol_full.sol(times)
for i, t in enumerate(times):
    print(f"t={t:.1f}s: x={states[0,i]:.3f}, theta={states[2,i]:.3f}, theta_dot={states[3,i]:.3f}")
    if np.any(np.abs(states[:,i]) > 1e6):
        print("EXPLOSION DETECTED!")
        break