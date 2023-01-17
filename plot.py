import numpy as np
import matplotlib.pyplot as plt

X = np.genfromtxt("systemStates.csv", delimiter = ',')

X_est = np.genfromtxt("estimatedStates.csv", delimiter = ',')

N = np.shape(X)[1]

Ts = 1e-3

t = np.arange(N)*Ts

plt.figure(figsize = (16, 9))

plt.plot(t, X[2], lw = 3.5, label = 'Ground Truth')

plt.plot(t, X_est[2], lw = 3.5, linestyle = '--', label = 'Estimated')

plt.title(r"System Velocity Along x-axis ($v_x$)", fontsize = 30)

plt.xlabel(r"time (s)", fontsize = 30)

plt.ylabel(r"velocity (m/s)", fontsize = 30)

plt.legend(fontsize = 30)

plt.xticks(fontsize = 25)

plt.yticks(fontsize = 25)

plt.grid()

plt.tight_layout()

plt.savefig("system_x_velocity.pdf")

plt.figure(figsize = (16, 9))

plt.plot(t, X[3], lw = 3.5, label = 'Ground Truth')

plt.plot(t, X_est[3], lw = 3.5, linestyle = '--', label = 'Estimated')

plt.title(r"System Velocity Along y-axis ($v_y$)", fontsize = 30)

plt.xlabel(r"time (s)", fontsize = 30)

plt.ylabel(r"velocity (m/s)", fontsize = 30)

plt.legend(fontsize = 30)

plt.xticks(fontsize = 25)

plt.yticks(fontsize = 25)

plt.grid()

plt.tight_layout()

plt.savefig("system_y_velocity.pdf")

plt.figure(figsize = (16, 9))

plt.plot(t, X_est[4], lw = 3.5, label = r'Estimated Parameter $b_1$')

plt.plot(t, X_est[5], lw = 3.5, label = r'Estimated Parameter $b_2$')

plt.axhline(y = 1.85, lw = 3.5, linestyle = '--', c = 'tab:red', label = r'Ground Truth Value of Parameter $b_1$')

plt.axhline(y = 0.75, lw = 3.5, linestyle = '--', c = 'tab:green', label = r'Ground Truth Value of Parameter $b_2$')

plt.title(r"Identified System Drag Parameters", fontsize = 30)

plt.xlabel(r"time (s)", fontsize = 30)

plt.ylabel(r"viscous drag (1/s)", fontsize = 30)

plt.xticks(fontsize = 25)

plt.yticks(fontsize = 25)

plt.grid()

plt.tight_layout()

plt.savefig("identified_parameters.pdf")

plt.show()

