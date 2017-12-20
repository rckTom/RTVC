import numpy as np
import matplotlib.pyplot as plt

# state transformation matrix
A = np.matrix([[1, 0.01, 0],[0, 1, 0.01], [0, 0, 1]])

# initial state (known or estimated)
X = np.matrix([[0.0],[0.0],[0.0]])

# initial error covariance
P = np.matrix(np.identity(3))

# system error covariance
Q = 0.01 * np.matrix(np.identity(3))

# measurement to state translation
H = np.matrix([[1.0,0,0],[0,0,1.0]])

# measurement noise covariance
R = np.matrix([[0.25,0],[0,1.0]])

X_prio_hist = []
P_prio_hist = []
K_hist = []
X_hist = []
P_hist = []

Z=np.loadtxt("z_alti_accel.csv")
v=np.loadtxt("v.csv")
a=np.loadtxt("a.csv")

# # updating K and P
# for z in Z:
    # # prediction
    # X_prio = A*X
    # P_prio = A*P*A.T + Q
    
    # # correction
    # K = P_prio*H.T*np.linalg.inv(H*P_prio*H.T+R)
    # X = X_prio + K*(np.matrix(z).T-H*X_prio)
    # P = (np.matrix(np.identity(3)) - K*H)*P_prio
    
    # # save history
    # X_prio_hist.append(X_prio)
    # P_prio_hist.append(P_prio)
    # K_hist.append(K)
    # X_hist.append(X)
    # P_hist.append(P)

# converged K and P from last run
K = np.matrix([[ 0.18899183,  0.00023435],
        [ 0.18099849,  0.0084989 ],
        [0.00093741,  0.09512343]])

P = np.matrix([[  4.72479563e-02,   4.52496214e-02,   2.34353030e-04],
        [  4.52496214e-02,   1.05442011e+00,   8.49890329e-03],
        [2.34353030e-04,   8.49890329e-03,   9.51234271e-02]])

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

plt.plot(Z[:,0])
plt.plot(h_hist)
plt.plot(v)
plt.plot(v_hist)
plt.plot(Z[:,1])
plt.plot(a_hist)
plt.scatter(t_apo, h_apo, c="red")
plt.show()