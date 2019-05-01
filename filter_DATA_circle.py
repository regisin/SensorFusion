import numpy as np
import matplotlib.pyplot as plt
import KF
from math import *

#Read measurements
ekf_data_file = open('EKF_DATA_circle.txt', 'r')
Zraw = []
for line in ekf_data_file:
    if line[0][0] != '%':
        explode_line = line.split(',')
        explode_line[-1] = explode_line[-1][:-2]
        Zraw.append(explode_line)
            
Zraw = np.array(Zraw, dtype='float')

#Calibrate IMU by the first measurement of odometry (second entry in Z)
delta = float(Zraw[1][3])-float(Zraw[1][4])
Zraw[:,4] += delta # adds delta to every item in column 4 (imu theta)

#Extract velocities from odometry
ini = Zraw[0]
measurements = []
time_step = 1
for i in xrange(len(Zraw)-1):
    data = {}
    ii = i+1
    timestamp = Zraw[ii][0] #timestamp
    data['timestamp'] = timestamp

    z = []
    z.append(Zraw[ii][6])       # x gps
    z.append(Zraw[ii][7])       # y gps
    z.append(.14)               # Fixed velocity given
    z.append(Zraw[ii][4])       # theta imu
    z.append(.14*tan(Zraw[ii-1][3])) # Angular velocity based on odometry theta
    
    R = np.zeros((5,5))
    # If cov GPS is zero, use Odometry data
    if Zraw[ii][8] == 0.0 and Zraw[ii][9] == 0.0:
        z[0] = Zraw[ii][1]
        z[1] = Zraw[ii][2]
        R[0][0] = .1 # cov x odo
        R[1][1] = .1 # cov y odo
    else:
        R[0][0] = Zraw[ii][8] # cov x gps
        R[1][1] = Zraw[ii][9] # cov y gps
    R[2][2] = .01
    R[3][3] = Zraw[ii][5] # cov imu
    R[4][4] = .01
    
    data['z'] = z
    data['R'] = R
    
    measurements.append(data)

#Initialization
V = .14     # speed of robot
dt = .001   # time step
A = np.array([[1, 0, dt*V*0, 0, 0],
              [0, 1, dt*V*0, 0, 0],
              [0, 0, 1, 0, 0],
              [0, 0, 0, 1, dt],
              [0, 0, 0, 0, 1]])
B = np.zeros_like(A)
H = np.array([[1, 0, 0, 0, 0],
              [0, 1, 0, 0, 0],
              [0, 0, 1, 0, 0],
              [0, 0, 0, 1, 0],
              [0, 0, 0, 0, 1]])
Q = np.array([[.00004, 0, 0, 0, 0],
              [0, .00004, 0, 0, 0],
              [0, 0, .00001, 0, 0],
              [0, 0, 0, .00001, 0],
              [0, 0, 0, 0, .00001]])
R = np.array([[.04, 0, 0, 0, 0],
              [0, .04, 0, 0, 0],
              [0, 0, .01, 0, 0],
              [0, 0, 0, .01, 0],
              [0, 0, 0, 0, .01]])
P = np.array([[.01, 0, 0, 0, 0],
              [0, .01, 0, 0, 0],
              [0, 0, .01, 0, 0],
              [0, 0, 0, .01, 0],
              [0, 0, 0, 0, .01]])
x = np.array([[0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, V, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0]])
u = np.zeros_like(x)

#Kalman filter
kf = KF.KalmanFilter(A,B,H,x,P,Q,R);

#Main for loop
time = []
truevalue = []
measured = []
predicted = []

for data in measurements:
    timestamp = data['timestamp']
    R = data['R']
    z = data['z']
    time.append(timestamp)
    measured.append(z)
    predicted.append(np.diag(kf.state()))
    theta = np.diag(kf.state())[3]
    A = np.array([[1, 0, dt*V*theta, 0, 0],
                  [0, 1, dt*V*theta, 0, 0],
                  [0, 0, 1, 0, 0],
                  [0, 0, 0, 1, dt],
                  [0, 0, 0, 0, 1]])
    kf.SetR(R)
    kf.SetA(A)
    kf.iterate(u, z)
    
time = np.array(time)
truevalue = np.array(truevalue)
measured = np.array(measured)
predicted = np.array(predicted)

#Plot
# Heading
plt.plot(time,Zraw[:,3][1:], marker='None', linestyle='--')
plt.plot(time,Zraw[:,4][1:], marker='None', linestyle='-.')
plt.plot(time,predicted[:,3], marker='None', linestyle='-')
plt.legend(['Odo', 'IMU', 'KF'])
plt.ylabel('Heading (rad)')
plt.xlabel('Time')
plt.savefig('heading_DATA_circle.eps')
plt.clf()
# Trajectory
plt.plot(Zraw[:,1],Zraw[:,2], marker='None', linestyle='--')
plt.plot(measured[:,0],measured[:,1], marker='.', linestyle='None')
plt.plot(predicted[:,0], predicted[:,1], marker='None', linestyle='-')
plt.legend(['Odo', 'GPS', 'KF'], loc=2)
plt.ylabel('Y coordinate')
plt.xlabel('X coordinate')
plt.savefig('trajectory_DATA_circle.eps')
plt.clf()

