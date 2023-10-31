from math import pi
import numpy as np
from sympy import Function, MatrixSymbol, Matrix, cos, sin, pprint, eye
from sympy import FunctionMatrix, symbols, Lambda, MatPow

import matplotlib.pyplot as plt
from tqdm import tqdm

np.set_printoptions(suppress=True)

#VAriable to store the Circle Trajectory
CIRCLE_POINTS = []

tol = 1e-16

#Create class to store different DH paramters of each joint
class DHParam_Revolute:
   def __init__(self, d, alpha, a, theta, type = 'R'):
    self.d = d
    self.alpha = alpha
    self.a = a
    self.theta = symbols(theta)
    self.type = type


#Store DH Parameters of Panda robot in home position
Panda_DHParam = {
    'J1' : DHParam_Revolute(d=0.333 , alpha= pi/2, a = 0    , theta = "θ1", type= 'R'),
    'J2' : DHParam_Revolute(d=0 , alpha= -pi/2, a = 0    , theta = "θ2", type= 'R'),
    'J3' : DHParam_Revolute(d=0.316 ,    alpha=pi/2, a = 0.088    , theta = "θ3", type= 'R'),
    'J4' : DHParam_Revolute(d=0     , alpha=-pi/2, a = -0.088, theta = "θ4", type= 'R'),
    'J5' : DHParam_Revolute(d=0.384     , alpha=pi/2, a = 0 , theta = "θ5", type= 'R'),
    'J6' : DHParam_Revolute(d=0     , alpha=-pi/2, a = 0.088, theta = "θ6", type= 'R'),
    'J7' : DHParam_Revolute(d=-0.107     , alpha=0, a = 0, theta = "θ7", type= 'R')
}



def GetJacobian(config):

    #Create five configurations of the robot with different values for theta1 to theta8
    TF_list = [ ]

    TF_FixedToEndEffector = np.eye(4)

    #Perform Forward Kinematics to find the Transformations and 
    # enf effector location
    for i in range(1,len(Panda_DHParam)+1):
        Joint = 'J'+str(i)
        
        #Compute transformation matrix from i-1 to ith link
        if Panda_DHParam[Joint].type == 'R':

            d = Panda_DHParam[Joint].d
            a = Panda_DHParam[Joint].a
            alpha = Panda_DHParam[Joint].alpha
            theta =config[i-1]

            TF = np.array([
                [cos(theta),   -1*sin(theta)*cos(alpha),    sin(theta)*sin(alpha),  a*cos(theta)],
                [sin(theta)  ,    cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha),  a*sin(theta)],
                [0           ,    sin(alpha),               cos(alpha),             d           ],
                [0,               0,                        0,                      1           ]
            ])

        # Find Each links transformation with respect to the fixed frame
        TF_FixedToEndEffector = TF_FixedToEndEffector@TF
        TF_list.append(TF_FixedToEndEffector)    

    #Append points to the list
    CIRCLE_POINTS.append([TF_FixedToEndEffector[1][3], TF_FixedToEndEffector[2][3]])


    #Find jacobian from the transformations
    TF_list.insert(0, eye(4))
    Jacobian = []
    #J[i] = [Jv{3x1} Jw{3x1}] {6x1} --> each column
    for i in range(1,len(Panda_DHParam)+1):
        Joint = 'J'+str(i)

        if Panda_DHParam[Joint].type == 'R':

            # All w.r.t to fixed frame ie. Zi-1 to {0} frame, Oi-1, On etc
            Zi_1 = np.squeeze(TF_list[i-1][:3,2])
            Oi_1 = np.squeeze(TF_list[i-1][:3,3])
            On = np.squeeze(TF_list[len(TF_list)-1][:3,3])

            O_diff = On - Oi_1
            
            #J linear velocity component for the revolute joint is given as 
            # Zi-1x(On - Oi-1)
            Ji_linear = np.cross(Zi_1, O_diff)


            #J linear velocity component for the revolute joint is given as 
            # Zi
            Ji_angular = np.squeeze(TF_list[i][:3,2])

            #Concatenate linear and angular velocity components
            Ji = np.concatenate((Ji_linear, Ji_angular), axis=0)
            Jacobian.append(Ji)

        if Panda_DHParam[Joint].type == 'P':
            Zi = np.squeeze(TF_list[i][:3,2])

            Ji_linear = Zi
            Ji_angular = np.zeros(3)
            Ji = np.concatenate((Ji_linear, Ji_angular), axis=0)
            Jacobian.append(Ji)
    
    #Take transpose to match dimentions as [6xn]
    Jacobian = np.array(Jacobian, dtype=np.float32).T


    return Jacobian

#As per the DH-frame choice, the inital configuration of the robot can be taken as
q =  [0, 0, 0, -pi/2, 0, pi, 0]

#Angular velocity to completer the full circle under 5 seconds
theta_dot = 2*pi/5 

r = 0.1 #radius in meters

# Resolution of the trajectory is taken as 0.01 radians
detlta_theta = 0.01

# Time required to move detlta_theta radians at theta_dot velocity
delta_t = (5/(2*pi))*detlta_theta

# Loop ove the 360 degrees to draw the Circle
for theta in tqdm(np.arange(0, 2*pi , detlta_theta)):

    #The projected velocity components of the circle are found as
    # yd =       r * theta_dot * cos(theta)
    # zd =  -1 * r * theta_dot * sin(theta)
    V = np.array( [0, r*theta_dot*cos(theta) , -1*r*theta_dot*sin(theta), 0, 0, 0 ], dtype=np.float32)

    #Find the jacobian from the joint angles
    J = GetJacobian(q)

    if theta == 0:
        print(f"Inital Settings at angle {theta} radinas : \n")
        print(f"Joint angles are calculated as : {q}\n")
        print("Jacobians is calculated as : ")
        print(J)
        print(f"\nEnd effectory location is calculated as : {CIRCLE_POINTS}\n")

    #Calculate inverse jacobian
    J_inv = np.linalg.pinv(J)

    #Find the joint velocities
    q_dot = J_inv @ V

    #Integrate the joint angles
    q += q_dot * delta_t


#Plot the circle
CIRCLE_POINTS = np.array(CIRCLE_POINTS)
figure = plt.figure("Panda Robot tracking a circular trajectory")
ax = figure.add_subplot(111)
ax.set_xlabel('Y axis')
ax.set_ylabel('Z axis')
ax.plot(CIRCLE_POINTS[:,0], CIRCLE_POINTS[:,1])
ax.set_aspect(1)

plt.show()
