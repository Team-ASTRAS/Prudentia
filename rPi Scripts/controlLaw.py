from enum import Enum, unique
import numpy as np
from scipy import linalg
import time
from utilities import log

@unique
class ControlRoutine(Enum):
    stabilize = 1 #Stabilize angular position
    realTimeControl = 2 #Change target vector based on RTC input
    attitudeInput = 3 #Set a target vector for Prudentia
    search = 4 #Search routines

class ControlLawSingleton:

    controlRoutine = ControlRoutine.stabilize

    #Functions named with the format routineName are functions that are called
    #by main.py when the state machine is set to run a particular routine

    def routineStabilize(self):
        pass

    def routineRealTimeControl(self):
        pass

    def routineAttitudeInput(self):
        pass

    def routineSearch(self):
        pass

    def __init__(self):
        np.set_printoptions(precision=5)

    def initialize(self):

        #Moment of Inertias
        I = np.array([[1.01560749, 0.00028915, 0.00002257], [0.00028915, 0.17343628, 0.00936307], [0.00002257, 0.00936307, 1.01561864]]);

        inverseI = linalg.inv(I)

        print(inverseI)

        #Reaction wheels
        Irw = 0.000453158

        largeAngle = np.cos(np.deg2rad(70.0))
        smallAngle = np.sqrt(0.5 * (1 - np.square(largeAngle)))
        
        #This is the 3D vector representation of the reaction wheel's applied momentum, orientated about the X axis
        #This is previously "e" in simulink files
        motorAngles = np.array([[largeAngle, smallAngle, smallAngle],
                               [largeAngle,-smallAngle, smallAngle],
                               [largeAngle,-smallAngle,-smallAngle],
                               [largeAngle, smallAngle,-smallAngle]]) 

        
        #Consider adding max torque, rpm


        # Gain setup
        k1 = (4-2)/3
        k2 = (I[2,2] - I[0,0])/I[1,1]
        k3 = (I[1,1] - I[0,0])/I[2,2]
        
        n = np.sqrt(398600 / 6378**3)
        F = -2 * n**2 * np.array([[4*k1, 0, 0], [0, 3*k2, 0], [0, 0, k3]])
        G = n * np.array([[0, 0, (1.0-k1)], [0, 0, 0], [(k3-1.0), 0, 0 ]])

        # CARE Equation
        A_top = np.concatenate((np.zeros([3,3]), 0.5*np.eye(3)), axis=1)
        A_bottom = np.concatenate((F, G), axis=1)
        
        A = np.concatenate((A_top, A_bottom), axis=0)
        B = np.concatenate((np.zeros([3,3]), np.diag(np.diag(inverseI))), axis=0) #TODO Should we mask along the diagonal like MATLAB is doing?
        Q = np.diag([1,1,1,100,100,100]) * (1 / 10000)
        R = np.array([[0.01, 0, 0], [0, 0.006, 0], [0, 0, 0.1]])
        
        S = linalg.solve_continuous_are(A, B, Q, R)
        
        inverseR = linalg.inv(R)
        transposeB = np.transpose(B)

        print(inverseR.shape)
        print(transposeB.shape)

        _X = np.matmul(inverseR, transposeB)

        K = -1 * np.dot(np.dot(inverseR, transposeB), S)

        log("Initialized gain matrix K:")
        log(K)
        
        #Temp
        time.sleep(1000)

    def setTarget(self, roll, pitch, yaw):
        self.targetQ = euler2quat(roll, pitch, yaw)

    #Returns a quaternion [q0, qhat] based on inputs roll, pitch, yaw (degrees)
    def euler2quat(self, roll, pitch, yaw):
        r = np.deg2rad(roll) / 2
        p = np.deg2rad(pitch) / 2
        y = np.deg2rad(yaw) / 2

        quat = np.array([np.cos(r)*np.cos(p)*np.cos(y) - np.sin(r)*np.sin(p)*np.sin(y),
                         np.sin(r)*np.cos(p)*np.cos(y) + np.cos(r)*np.sin(p)*np.sin(y),
                         np.cos(r)*np.sin(p)*np.cos(y) - np.sin(r)*np.cos(p)*np.sin(y),
                         np.cos(r)*np.cos(p)*np.sin(y) + np.sin(r)*np.sin(p)*np.cos(y)])
        
        return quat

        
cls = ControlLawSingleton()
cls.initialize()