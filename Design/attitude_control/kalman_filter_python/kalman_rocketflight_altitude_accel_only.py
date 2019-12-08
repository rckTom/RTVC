import numpy as np
import matplotlib.pyplot as plt

# state transformation matrix
A = np.matrix([[1, 0.01, 0],[0, 1, 0.01], [0, 0, 1]])

# initial state (known or estimated)
X = np.matrix([[0.0],[0.0],[0.0]])

# initial error covariance
P = np.matrix(np.identity(3))

# system error covariance
Q = 0.1 * np.matrix(np.identity(3))

# measurement to state translation
H = np.matrix([[0,0,1.0]])

# measurement noise covariance
R = np.matrix([[0.01]])

X_prio_hist = []
P_prio_hist = []
K_hist = []
X_hist = []
P_hist = []

v=np.loadtxt("v.csv")
a=np.loadtxt("a.csv")
# Z=np.loadtxt("z_alti_accel.csv")
# Z=Z[:,1]
Z=a+np.random.normal(0, 0.044, a.shape)

# # updating K and P
# for z in Z:
#     # prediction
#     X_prio = A*X
#     P_prio = A*P*A.T + Q
    
#     # correction
#     K = P_prio*H.T*np.linalg.inv(H*P_prio*H.T+R)
#     X = X_prio + K*(np.matrix(z).T-H*X_prio)
#     P = (np.matrix(np.identity(3)) - K*H)*P_prio
    
#     # save history
#     X_prio_hist.append(X_prio)
#     P_prio_hist.append(P_prio)
#     K_hist.append(K)
#     X_hist.append(X)
#     P_hist.append(P)

# # converged K and P from last run
# K = np.matrix([[0.00086076],
#                [0.00904875],
#                [0.09512492]])

# P = np.matrix([[  1.72873653e+02,   3.17903414e+01,   8.60761712e-04],
#                [  3.17903414e+01,   8.08000951e+00,   9.04875078e-03],
#                [  8.60761712e-04,   9.04875078e-03,   9.51249220e-02]])

# converged K and P from Q=0.1 and R=0.01
# K = np.matrix([[7.7e-7],
#                [8.4e-4],
#                [9.2e-1]])
K = np.matrix([[0.0e+0],
               [0.0e+0],
               [9.2e-1]])

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

h_hist = np.array(X_hist)[:,0,0]
v_hist = np.array(X_hist)[:,1,0]
a_hist = np.array(X_hist)[:,2,0]

t_apo = np.argmax((h_hist > 10)*(v_hist<0))
h_apo = h_hist[t_apo]

# plt.plot(Z[:,0])
plt.plot(h_hist)
plt.plot(v)
plt.plot(v_hist)
# plt.plot(Z[:,1])
plt.plot(Z)
plt.plot(a_hist)
plt.scatter(t_apo, h_apo, c="red")
plt.show()