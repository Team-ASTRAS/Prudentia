from enum import Enum, unique
import numpy as np
from scipy import linalg
import time
from utilities import log

@unique
class ControlRoutine(Enum):
    stabilize = 1 #Stabilize angular position
    attitudeInput = 2 #Set a target vector for Prudentia
    search = 3 #Search routines

class LqrMode(Enum):
    nominal = 1 #Standard LQR controller tuned for fast pointing
    yawSweep = 2 #Overrides target pitch to 0 if yaw is high
    yawSweepLockRoll = 3 #Overrides target pitch to 0 and roll to rollBody if yaw and roll are high
    correctivePitch = 4 #Immediately rotate toward neutal plane. Occurs on pitch boundry breaks
  
class EulerSet:
    r = 0
    p = 0
    y = 0

class routineReport:
    qError = np.array([0, 0, 0, 0])
    qErrorAdjusted = np.array([0, 0, 0, 0])
    lqrMode = LqrMode.nominal
    inertialTorque = np.array([0, 0, 0])
    motorTorques = np.array([0, 0, 0, 0])
    motorAlpha = np.array([0, 0, 0, 0])

#Returns the quaternion error between the target and observed quaternions.
def getQuatError(qObserved, qTarget):
    return quatMultiply(qObserved, qTarget)

#This is currently unused 2/23/2021. We shpuld always have a unit quaternion.
def normalizeQuat(self, quat):
    norm = np.linalg.norm(quat)
    if norm == 0: 
        return quat
    return quat / norm

#Multiplies quaternion q1 (LHS) by q2 (RHS)
def quatMultiply(q1, q2):
    qExpanded = np.array([[ q1[3],  q1[2], -q1[1],  q1[0]],
                          [-q1[2],  q1[3],  q1[0],  q1[1]],
                          [ q1[1], -q1[0],  q1[3],  q1[2]], 
                          [-q1[0], -q1[1], -q1[2],  q1[3]]])

    qFlipped = np.array([-q2[0], -q2[1], -q2[2], q2[3]])

    return np.dot(qExpanded, qFlipped)

#Returns a quaternion [qhat, q0] based on inputs roll, pitch, yaw (degrees)
def ypr2quat(yaw, pitch, roll):
    r = np.deg2rad(roll) / 2
    p = np.deg2rad(pitch) / 2
    y = np.deg2rad(yaw) / 2

    quat = np.array([-np.cos(r)*np.sin(p)*np.sin(y) + np.sin(r)*np.cos(p)*np.cos(y),
                      np.cos(r)*np.sin(p)*np.cos(y) + np.sin(r)*np.cos(p)*np.sin(y),
                     -np.sin(r)*np.sin(p)*np.cos(y) + np.cos(r)*np.cos(p)*np.sin(y),
                      np.sin(r)*np.sin(p)*np.sin(y) + np.cos(r)*np.cos(p)*np.cos(y)])
        
    return quat

#Returns roll, pitch, and yaw (results.r, results.p, results.y) in degrees, given a quaternion.
def quat2ypr(q):
    result = EulerSet()
    result.y = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2) * 180 / np.pi
    result.p = np.arcsin(-2 * (q[0] * q[2] - q[1] * q[3])) * 180 / np.pi
    result.r = np.arctan2(2 * (q[1] * q[2] + q[0] * q[3]), q[3]**2 - q[0]**2 - q[1]**2 + q[2]**2) * 180 / np.pi
    return result

def ypr2str(ypr):
    return "[%s, %s, %s]" % (round(ypr.y, 4), round(ypr.p), round(ypr.r)) 

class ControlLawSingleton:

    lqrMode = LqrMode.nominal

    yawSweepThreshold = 45
    rollSweepThreshold = 45
    correctivePitchThreshold = 20
    
    K_nominal = np.zeros((3,6))
    K_yawSweep = np.zeros((3,6))
    K_correctivePitch = np.zeros((3,6))
    
    #Functions named with the format routineName are functions that are called
    #by main.py when the state machine is set to run a particular routine
    
    def __init__(self):
        np.set_printoptions(precision=4, suppress=True, floatmode='maxprec_equal')
        self.initialize()

    def routineStabilize(self, q, w):
        ypr = quat2ypr(q)
        ypr.p = 0
        ypr.r = 0
        qTarget = ypr2quat(ypr.y, ypr.p, ypr.r)
        qError = getQuatError(q, qTarget)

        lqrMode = self.getControllerType(q, qError)
        qErrorAdjusted = self.adjustAttitude(lqrMode, q, qTarget, qError)
        inertialTorque = self.getTorque(lqrMode, qErrorAdjusted, w)

        results = routineReport()
        results.qError = qError
        results.qErrorAdjusted = qErrorAdjusted
        results.lqrMode = lqrMode
        results.inertialTorque = inertialTorque
        results.motorAccel = np.dot(self.IrwArray, -1 * inertialTorque)
        results.motorTorques = results.motorAlpha * self.Irw
        return results

    def routineAttitudeInput(self, q, w, qTarget):
        qError = getQuatError(q, qTarget)

        lqrMode = self.getControllerType(q, qError)
        qErrorAdjusted = self.adjustAttitude(lqrMode, q, qTarget, qError)
        inertialTorque = self.getTorque(lqrMode, qErrorAdjusted, w)
        
        results = routineReport()
        results.qError = qError
        results.qErrorAdjusted = qErrorAdjusted
        results.lqrMode = lqrMode
        results.inertialTorque = inertialTorque
        results.motorAccel = np.dot(self.IrwArray, -1 * inertialTorque)
        results.motorTorques = results.motorAlpha * self.Irw
        return results

    def routineSearch(self):
        pass

    def initialize(self):

        #Moment of Inertias
        I = np.array([[1.01560749, 0.00028915, 0.00002257],
                      [0.00028915, 0.17343628, 0.00936307],
                      [0.00002257, 0.00936307, 1.01561864]]);

        #inverseI = linalg.inv(I)       TODO: Why don't we take the full inverse?? 
        inverseI = np.array([[1/I[0,0], 0,        0       ],
                             [0,        1/I[1,1], 0       ],
                             [0,        0,        1/I[2,2]]])

        #Reaction wheels
        self.Irw = 0.000453158

        largeAngle = np.cos(np.deg2rad(70.0))
        smallAngle = np.sin(np.deg2rad(70.0))
        
        #This is the 3D vector representation of the reaction wheel's applied momentum, orientated about the X axis
        #This is previously "e" in simulink files
        #motorAngles = np.array([[largeAngle, smallAngle, 0],
        #                       [largeAngle, -smallAngle, 0],
        #                       [largeAngle, 0, smallAngle],
        #                       [largeAngle, 0, -smallAngle]]) 

        motorAngles = np.array([[largeAngle, largeAngle, largeAngle, largeAngle],
                                [smallAngle, 0,          -smallAngle, 0],
                                [0,          smallAngle, 0,          -smallAngle]])

        self.IrwArray = np.linalg.pinv(self.Irw * motorAngles)

        #Consider adding max torque, rpm

        # Gain setup
        k1 = (I[1,1] - I[2,2])/I[0,0]
        k2 = (I[1,1] - I[2,2])/I[1,1]
        k3 = (I[1,1] - I[0,0])/I[2,2]

        mu = 398600 #km^3/s^2 (Earth)
        a = 6378; #km (Earth)
        n = np.sqrt(mu / a**3)

        F = -2 * n**2 * np.array([[4*k1, 0, 0], [0, 3*k2, 0], [0, 0, k3]])
        G = n * np.array([[0, 0, (1.0-k1)], [0, 0, 0], [(k3-1.0), 0, 0 ]])

        # CARE Equation
        A_top = np.concatenate((np.zeros([3,3]), 0.5*np.eye(3)), axis=1)
        A_bottom = np.concatenate((F, G), axis=1)
        
        A = np.concatenate((A_top, A_bottom), axis=0)
        B = np.concatenate((np.zeros([3,3]), np.diag(np.diag(inverseI))), axis=0) #TODO Should we mask along the diagonal like MATLAB is doing?
        R = np.eye(3)
        
        # Gains for nominal operation
        Q_nominal = np.diag([1,1,1,10,10,10])
        S_nominal = linalg.solve_continuous_are(A, B, Q_nominal, R)
        self.K_nominal = -1 * np.dot(np.dot(linalg.inv(R), np.transpose(B)), S_nominal)

        # Gains for yawSweep and yawSweepLockRoll operations
        Q_yawSweep = np.diag([1,1,1,10,15,10])
        S_yawSweep = linalg.solve_continuous_are(A, B, Q_yawSweep, R)
        self.K_yawSweep = -1 * np.dot(np.dot(linalg.inv(R), np.transpose(B)), S_yawSweep)

        # Gains for correctivePitch operation
        Q_correctivePitch = np.diag([1,1,1,10,10,10])
        S_correctivePitch = linalg.solve_continuous_are(A, B, Q_correctivePitch, R)
        self.K_correctivePitch = -1 * np.dot(np.dot(linalg.inv(R), np.transpose(B)), S_correctivePitch)
        
        #log("Initialized A matrix:")
        #log(A)
        #log("Initialized B matrix:")
        #log(B)
        #log("Initialized R matrix:")
        #log(R)

        log("---------- Initializing gain matricies ----------")
        log("Initialized gain matrix K_nominal:")
        log(self.K_nominal)
        log("Initialized gain matrix K_yawsweep:")
        log(self.K_yawSweep)
        log("Initialized gain matrix K_correctivePitch:")
        log(self.K_correctivePitch)
        log("-------------------------------------------------")
       
    def adjustAttitude(self, lqrMode, q, qTarget, qError):
        if lqrMode == LqrMode.nominal:
            #Do nothing, return qError
            return qError
        
        elif lqrMode == LqrMode.correctivePitch:
            # Overwrite target with current position, and override pitch to zero.
            rpy = quat2ypr(q)
            rpy.p = 0
            newTarget = ypr2quat(rpy.y, rpy.p, rpy.r)
            return getQuatError(q, newTarget)
        
        elif lqrMode == LqrMode.yawSweep:
            #Override pitch to zero
            rpy = quat2ypr(qTarget)
            rpy.p = 0
            newTarget = ypr2quat(rpy.y, rpy.p, rpy.r)
            return getQuatError(q, newTarget)
        
        elif lqrMode == LqrMode.yawSweepLockRoll:
            #Override pitch to zero, set target roll to body roll.
            rpyTarget = quat2ypr(qTarget)
            rpyPrudentia = quat2ypr(q)
            rpy = EulerSet()
            rpy.r = rpyPrudentia.r
            rpy.p = 0
            rpy.y = rpyTarget.y
            newTarget = ypr2quat(rpy.y, rpy.p, rpy.r)
            return getQuatError(q, newTarget)
            
        else:
            log("Error in adjustAttitude! lqrMode was not set to an expected value! lqrMode: %s " % lqrMode)
            return None

    def getControllerType(self, qObserved, qError):
        #Returns a LqrMode enum representing the target override state.
        rpyError = quat2ypr(qError)
        yawErrorAbs = np.abs(rpyError.y)
        rollErrorAbs = np.abs(rpyError.r)

        pitchInertial = np.abs(quat2ypr(qObserved).p)

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

    def getTorque(self, lqrMode, qError, w):
        qHatError = qError[:3]
        xbar = np.concatenate((qHatError, w), axis=0)
        if lqrMode == LqrMode.nominal:
            K = self.K_nominal
        
        elif lqrMode == LqrMode.correctivePitch:
            K = self.K_correctivePitch
        
        elif lqrMode == LqrMode.yawSweep or lqrMode == LqrMode.yawSweepLockRoll:
            K = self.K_yawSweep

        else:
            log("Error in getTorque! lqrMode was not set to an expected value! lqrMode: %s " % lqrMode)
            K = self.K_nominal

        return np.dot(K, xbar)


if __name__ == "__main__":
    cls = ControlLawSingleton()

    q = ypr2quat(0,0,0)
    w = np.array([0, 0, 0])
    qTarget = ypr2quat(-90, 0, 0)

    res = cls.routineAttitudeInput(q, w, qTarget)
    log("IN q:             %s Euler: %s" % (q , ypr2str(quat2ypr(q))) )
    log("IN w:             %s" % w)
    log("IN qTarget:       %s Euler: %s" % (qTarget , ypr2str(quat2ypr(qTarget))) )
    log("-"*20)
    log("OUT qError:       %s Euler: %s]" % (res.qError , ypr2str(quat2ypr(res.qError))) )
    log("OUT qAdjusted:    %s Euler: %s]" % (res.qErrorAdjusted , ypr2str(quat2ypr(res.qErrorAdjusted))) )
    log("OUT lqrMode:      %s" % res.lqrMode)
    log("OUT Inert Torque: %s" % res.inertialTorque)
    log("OUT Motor Torque: %s" % res.motorTorques)
    log("OUT Motor Alpha:  %s (rad/s)" % res.motorAlpha)
    log("OUT Motor Alpha:  %s (rpm)" % (res.motorAlpha * 9.5493))
    deltaDC = (res.motorAlpha * 9.5493 + 267.34) / 524.55
    log("OUT deltaDC:      %s" % deltaDC)