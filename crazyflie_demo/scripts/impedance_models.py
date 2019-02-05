#!/usr/bin/env python

from scipy.integrate import odeint
import numpy as np
import time
import matplotlib.pyplot as plt


def MassSpringDamper(state,t,F):
	x = state[0]
	xd = state[1]
	m = 2.0 # Kilograms
	b = 12.6
	k = 20.0 # Newtons per meter
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]

F = 12
y0 = [np.pi/4.-0.1, 0.0]
t = np.linspace(0, 10, 101)

#sol = odeint(MassSpringDamper, y0, t, args=(F,))


def Pendulum(state, t, M):
    theta, omega = state
    b = 25.
    k = 0.
    J = 5.
    dydt = [omega, (M - b*omega - k*np.sin(theta))/J ]
    return dydt

M = 5
y0 = [-np.pi /4., 0.3]
t = np.linspace(0,10,101)

# sol = odeint(Pendulum, y0, t, args=(M,))

# plt.plot(t, sol[:, 0], 'b', label='x(t)')
# plt.plot(t, sol[:, 1], 'g', label='v(t)')
# plt.legend(loc='best')
# plt.xlabel('t')
# plt.grid()
# plt.show()

