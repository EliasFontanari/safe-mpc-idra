import numpy as np

def f(x):
    r = 2.0
    return np.exp( x)

def rk4_step(x, dt):
    k1 = f(x)
    k2 = f(x + dt/2 * k1)
    k3 = f(x + dt/2 * k2)
    k4 = f(x + dt  * k3)
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
def rk4_back_step(x, dt):
    k1 = -f(x)
    k2 = -f(x + dt/2 * k1)
    k3 = -f(x + dt/2 * k2)
    k4 = -f(x + dt  * k3)
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
# example
x = 0.1
dt = 0.1
print(f"t={( 0)*dt:.1f}  x={x:.6f}")
for i in range(5):
    x = rk4_step(x, dt)
    print(f"t={( i+1)*dt:.1f}  x={x:.6f}")
for i in range(5):
    x = rk4_back_step(x, dt)
    print(f"t={( i+1+5)*dt:.1f}  x={x:.6f}")