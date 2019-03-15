import numpy as np
import time
from scipy.integrate import odeint
from math import *
from impedance_modeles import MassSpringDamper


y0 = [np.pi - 0.1, 0.0]
F_external = 10
t = np.linspace(0, 20, 201)
mode = 'overdamped'
sol = odeint(MassSpringDamper, y0, t, args=(F_external, mode,))


import matplotlib.pyplot as plt
plt.plot(t, sol[:, 0], 'b', label='theta(t)')
# plt.plot(t, sol[:, 1], 'g', label='omega(t)')
plt.legend(loc='best')
plt.xlabel('t')
plt.grid()
plt.show()