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

class LqrMode(Enum):
    nominal = 1 #Standard LQR controller tuned for fast pointing
    yawSweep = 2 #Overrides target pitch to 0 if yaw is high
    yawSweepLockRoll = 3 #Overrides target pitch to 0 and roll to rollBody if yaw and roll are high
    correctivePitch = 4 #Immediately rotate toward neutal plane. Occurs on pitch boundry breaks
  
class EulerSet:
    r = 0
    p = 0
    y = 0

class ModelFunctions:

    #Normalize quaternion.
    def normalizeQuat(self, quat):
        norm = np.linalg.norm(quat)
        if norm == 0: 
           return quat
        return quat / norm

    #Returns the quaternion error between the target and observed quaternions, in the body frame.
    def qError(self, qTarget_B, qObserved_B):
        
        q1 = normalizeQuat(qObserved_B)
        q2 = qTarget_B #Expecting an already normalized quaternion
        
        return quatMultiply(qObserved_B, qTarget_B)
    
    #Multiplies quaternion q1 (LHS) by q2 (RHS)
    def quatMultiply(self, q1, q2):
        qExpanded = np.array([[q1[0], -q1[1],  q1[2], -q1[3]],
                             [q1[1],  q1[0],  q1[3], -q1[2]],
                             [q1[2], -q1[3],  q1[0],  q1[1]], 
                             [q1[3],  q1[2], -q1[1],  q1[0]]])

        qFlipped = np.array([q2[0],-q2[1], -q2[2], -q2[3]])

        return np.dot(qExpanded, qFlipped)

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

    #Returns roll, pitch, and yaw (results.r, results.p, results.y) in degrees, given a quaternion.
    def quat2euler(self, q):
        result = EulerSet()
        result.r = np.arctan2(-2 * (q[2] * q[3] - q[0] * q[1]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2) * 180 / np.pi
        result.p = np.arcsin(2*(q[1] * q[3] + q[0] * q[2])) * 180 / np.pi
        result.y = np.arctan2(-2 * (q[1] * q[2] - q[0] * q[3]), q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2) * 180 / np.pi
        return result

class ControlLawSingleton:
    
    mf = ModelFunctions()

    controlRoutine = ControlRoutine.stabilize
    lqrMode = LqrMode.nominal

    yawSweepThreshold = 20
    rollSweepThreshold = 20
    correctivePitchThreshold = 20
    
    #Functions named with the format routineName are functions that are called
    #by main.py when the state machine is set to run a particular routine
    
    def __init__(self):
        np.set_printoptions(precision=4)
        self.initialize()

    def routineStabilize(self):
        pass

    def routineRealTimeControl(self):
        pass

    def routineAttitudeInput(self, w_B, q_B, qTarget_I):
        qTarget_B = RotateInertialToBody(qTarget_I)
        q_I = RotateBodyToInertial(q_B)

        qError = mf.qError(qTarget_B, qBody)
        lqrMode = getControllerType(q_I, qError)
        qErrorAdjusted = adjustAttitude(lqrMode,)
        pass

    def routineSearch(self):
        pass


    def initialize(self):

        #Moment of Inertias
        I = np.array([[1.01560749, 0.00028915, 0.00002257], [0.00028915, 0.17343628, 0.00936307], [0.00002257, 0.00936307, 1.01561864]]);

        inverseI = linalg.inv(I)

        #Reaction wheels
        Irw = 0.000453158

        largeAngle = np.cos(np.deg2rad(70.0))
        smallAngle = np.sin(np.deg2rad(70.0))
        
        #This is the 3D vector representation of the reaction wheel's applied momentum, orientated about the X axis
        #This is previously "e" in simulink files
        motorAngles = np.array([[largeAngle, smallAngle, 0],
                               [largeAngle, -smallAngle, 0],
                               [largeAngle, 0, smallAngle],
                               [largeAngle, 0, -smallAngle]]) 

        
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
        
        K = -1 * np.dot(np.dot(linalg.inv(R), np.transpose(B)), S)

        log("Initialized gain matrix K:")
        log(K)
      
    
    def RotateBodyToInertial(self, q):
        return q

    def RotateInertialToBody(self, q):
        return q        
    
    def adjustAttitude(lqrMode, q_B, qTarget_I, qError):
        if lqrMode == LqrMode.nominal:
            return qError
        
        elif lqrMode == LqrMode.correctivePitch:
            # Override pitch to zero.
            rpy = quat2euler(q_B)
            rpy.p = 0
            return mf.euler2quat(rpy.r, rpy.p, rpy.y)

    def getControllerType(self, qObserved, qError):
        #Returns a LqrMode enum representing the target override state.
        rpyError = mf.quat2euler(qError)
        yawErrorAbs = np.abs(rpyError.y)
        rollErrorAbs = np.abs(rpyError.r)

        pitchInertial = np.abs(mf.quat2euler(qObserved).p)

        if pitchInertial > self.correctivePitchThreshold:
            #Override target pitch to 0
            return LqrMode.correctivePitch

        elif yawErrorAbs > self.yawSweepThreshold:

            if rollErrorAbs > self.rollSweepThreshold:
                #Override target pitch to 0, AND roll to bodyRoll
                return LqrMode.yawSweepLockRoll
            else:
                #Override target pitch to 0
                return LqrMode.yawSweep
        else:
            return LqrMode.nominal

 

cls = ControlLawSingleton()
q1 = cls.mf.euler2quat(-20,16,44)
q2 = cls.mf.euler2quat(170,-15,-50)
qerror = cls.mf.quatMultiply(q1, q2)
res = cls.mf.quat2euler(q1)
print("RPY: [%s, %s, %s]" %(res.r, res.p, res.y))
res = cls.mf.quat2euler(q2)
print("RPY: [%s, %s, %s]" %(res.r, res.p, res.y))
