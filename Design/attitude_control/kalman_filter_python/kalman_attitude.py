import numpy as np
import matplotlib.pyplot as plt

# state transformation matrix
A = np.matrix([[1, 0.01, 0],[0, 1, 0.01], [0, 0, 1]])

# command matrix
B = np.matrix([[0.0],[0.0],[3.0*0.04/8.1e-4]])
# B = np.matrix([[0.0],[0.0],[0.0]])

# initial state (known or estimated)
X = np.matrix([[0.0],[0.0],[0.0]])

# initial error covariance
P = np.matrix(np.identity(3))

# system error covariance
Q = 0.01 * np.matrix(np.identity(3))

# measurement to state translation
H = np.matrix([0.0,1.0,0.0])

# measurement noise covariance
R = np.matrix([0.02])

X_prio_hist = []
P_prio_hist = []
u_hist = []
K_hist = []
X_hist = []
P_hist = []

Z=np.loadtxt("gammadot_noise.csv")
U=np.loadtxt("command_exact.csv")
gamma=np.loadtxt("gamma_exact.csv")
gammadot=np.loadtxt("gammadot_exact.csv")

e_int = 0
k_P = -5.0
k_I = -5.0
k_D = -1.0

"""# updating K and P

# determine command from PID
for z in Z:
    e_int = e_int + 0.01 * X[0,0]
    u = k_P * X[0,0] + k_I * e_int + k_D * X[1,0]
    
# take command from file
# for z, u in zip(Z, U):
    
    # prediction
    # X_prio = A*X + B*u
    X_prio = A*X
    P_prio = A*P*A.T + Q
    
    # correction
    K = P_prio*H.T*np.linalg.inv(H*P_prio*H.T+R)
    X = X_prio + K*(z-H*X_prio)
    P = (np.matrix(np.identity(3)) - K*H)*P_prio
    
    # save history
    u_hist.append(u)
    X_prio_hist.append(X_prio)
    P_prio_hist.append(P_prio)
    K_hist.append(K)
    X_hist.append(X)
    P_hist.append(P)"""

# # converged K and P from last run
K = np.matrix([[0.00733],
               [0.267],
               [0.258]])

# P = np.matrix([[ 0.04871656,  0.08012494,  0.04486424],
        # [ 0.08012494,  1.88353956,  1.08584885],
        # [ 0.04486424,  1.08584885,  1.78593403]])

for z in Z:
    # prediction
    X_prio = A*X
    
    # correction
    X = X_prio + K*(np.matrix(z).T-H*X_prio)
    
    # save history
    X_prio_hist.append(X_prio)
    X_hist.append(X)

print("Final Kalman gain matrix K")
print(K)
print("Final error covariance matrix P")
print(P)

gamma_hist = np.array(X_hist)[:,0,0]
gammadot_hist = np.array(X_hist)[:,1,0]

# f, ax = plt.subplots(2, sharex=True)
f, ax = plt.subplots(3, sharex=True)

ax[0].set_ylim([-0.012,0.032])
ax[1].set_ylim([-0.001,0.012])
ax[2].set_ylim([-0.09,0.005])

ax[0].set_ylabel("gamma'")
ax[1].set_ylabel("gamma")
ax[2].set_ylabel("u")

ax[0].plot(Z, 'b-')
ax[0].plot(gammadot_hist, 'r-')
ax[0].plot(gammadot, 'g-')

ax[1].plot(gamma, 'g-')
ax[1].plot(gamma_hist, 'r-')

ax[2].plot(U, 'g-')
ax[2].plot(u_hist, 'r-')

plt.savefig("kalman_attitude_Bdirect.png")
plt.show()