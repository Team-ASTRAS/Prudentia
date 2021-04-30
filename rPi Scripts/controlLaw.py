from enum import Enum, unique
import numpy as np
from scipy import linalg
import time
from utilities import log
from math import cos, sin

@unique
class ControlRoutine(Enum):
    stabilize = 1 #Stabilize angular position
    attitudeInput = 2 #Set a target vector for Prudentia
    search = 3 #Search routines

class PdMode(Enum):
    nominal = 1 #Standard LQR controller tuned for fast pointing
    yawSweep = 2 #Overrides target pitch to 0 if yaw is high
    yawSweepLockRoll = 3 #Overrides target pitch to 0 and roll to rollBody if yaw and roll are high
    correctivePitch = 4 #Immediately rotate toward neutal plane. Occurs on pitch boundry breaks
  

class routineReport:
    
    def __init__(self, qError, qErrorAdjusted, pdMode, inertialTorque, motorAccels, motorTorques):
        self.qError = qError
        self.qErrorAdjusted = qErrorAdjusted
        self.pdMode = pdMode
        self.inertialTorque = inertialTorque
        self.motorAccels = motorAccels
        self.motorTorques = motorTorques
    
    qError = np.array([0, 0, 0, 0])
    qErrorAdjusted = np.array([0, 0, 0, 0])
    pdMode = PdMode.nominal
    inertialTorque = np.array([0, 0, 0])
    motorTorques = np.array([0, 0, 0, 0]) 
    motorAccels = np.array([0, 0, 0, 0])

#Returns the quaternion error between the target and observed quaternions.
def getQuatError(qObserved, qTar):
    return quatMultiplyFlipped(qObserved, qTar)

#This is currently unused 2/23/2021. We shpuld always have a unit quaternion.
def normalizeQuat(quat):
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

    return np.dot(qExpanded, q2)

#Multiplies quaternion q1 (LHS) by q2 (RHS). Flips qhat of q2, used to find qError.
def quatMultiplyFlipped(q1, q2):
    qExpanded = np.array([[ q1[3],  q1[2], -q1[1],  q1[0]],
                          [-q1[2],  q1[3],  q1[0],  q1[1]],
                          [ q1[1], -q1[0],  q1[3],  q1[2]], 
                          [-q1[0], -q1[1], -q1[2],  q1[3]]])

    qFlipped = np.array([-q2[0], -q2[1], -q2[2], q2[3]])

    return np.dot(qExpanded, qFlipped)

#Returns a quaternion [qhat, q0] based on inputs roll, pitch, yaw (rad)
def ypr2quat(ypr):
    y = ypr[0] / 2
    p = ypr[1] / 2
    r = ypr[2] / 2

    quat = np.array([-np.cos(r)*np.sin(p)*np.sin(y) + np.sin(r)*np.cos(p)*np.cos(y),
                      np.cos(r)*np.sin(p)*np.cos(y) + np.sin(r)*np.cos(p)*np.sin(y),
                     -np.sin(r)*np.sin(p)*np.cos(y) + np.cos(r)*np.cos(p)*np.sin(y),
                      np.sin(r)*np.sin(p)*np.sin(y) + np.cos(r)*np.cos(p)*np.cos(y)])
        
    return quat

#Returns roll, pitch, and yaw in radians, given a quaternion. DO NOT USE QERROR HERE, USE quatError2ypr()
def quat2ypr(q):
    result = np.array([0.0, 0.0, 0.0])
    result[2] = np.arctan2(2 * (q[1] * q[2] + q[0] * q[3]), q[3]**2 - q[0]**2 - q[1]**2 + q[2]**2)
    result[1] = np.arcsin(-2 * (q[0] * q[2] - q[1] * q[3]))
    result[0] = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2)
    
    return result

#Returns roll, pitch, and yaw in radians, given an error quaternion.
#Since qError flips qHat, the ZYX rotation must be reversed to XYZ, or qHat must be flipped then the conversion result negated
def quatError2ypr(q):
    qFlipped = np.array([-q[0], -q[1], -q[2], q[3]])
    return -1 * quat2ypr(qFlipped)

#Returns a quaternion based on a normalized euler axis and angle. Expects units in radians.
def eulerAxis2quat(axis, angle):
    qHat = axis * sin(angle / 2)
    qReal = cos(angle / 2)
    return np.append(qHat, qReal)

class ControlLawSingleton:

    pdMode = PdMode.nominal

    yawSweepThreshold = np.radians(45)
    rollSweepThreshold = np.radians(45)
    correctivePitchThreshold = np.radians(20)
    
    enableSaturation = True
    maxTorque = np.array([0.055, 0.055, 0.055]) * 0.75
    loopFrequency = 20

    I = np.array([[ 0.27021632, -0.00050396, -0.00034281],
                  [-0.00050396, 1.5538962  , 2.344e-05  ],
                  [-0.00034281, 2.344e-05  , 1.55353581 ]]);
    
    Irw = 0.000453158
    motorAngle = 45

    overshoot = 0.05 # arbitrary percent OS
    ts = 20 # seconds, arbitrary settling time

    #Functions named with the format routineName are functions that are called
    #by main.py when the state machine is set to run a particular routine
    
    def __init__(self):
        np.set_printoptions(precision=4, suppress=True, floatmode='maxprec_equal')
        self.initialize()

    def routineStabilize(self, q, w):
        pass

    def routineAttitudeInput(self, q, w, qTarget):
        
        qError = getQuatError(q, qTarget)

        pdMode = self.getControllerType(q, qError)
        
        qErrorAdjusted = self.adjustAttitude(pdMode, q, qTarget, qError)
        
        inertialTorque = self.getTorque(pdMode, qErrorAdjusted, w)
        
        motorAccels = self.getMotorAccels(inertialTorque)
        
        motorTorques = motorAccels * self.Irw
        
        results = routineReport(qError, qErrorAdjusted, pdMode, inertialTorque, motorAccels, motorTorques)
        
        return results
    
    def getMotorAccels(self, inertialTorque):
        motorAccels = np.dot(self.IrwArray, -1 * inertialTorque)
        
        return motorAccels

    def routineSearch(self):
        pass

    def initialize(self):

        #Reaction wheel angles
        sinAngle = np.sin(np.deg2rad(self.motorAngle))
        cosAngle = np.cos(np.deg2rad(self.motorAngle))

        self.motorAngles = np.array([[sinAngle,  sinAngle,  sinAngle,  sinAngle],
                                     [0,         -cosAngle,  0,         cosAngle],
                                     [cosAngle,   0,         -cosAngle, 0        ]])
        
        #cosAngle30 = np.cos(np.deg2rad(self.motorAngle))*np.cos(np.deg2rad(30))
        #cosAngle60 = np.cos(np.deg2rad(self.motorAngle))*np.cos(np.deg2rad(60))
        #self.motorAngles = np.array([[sinAngle,    sinAngle,    sinAngle,    sinAngle],
        #                             [cosAngle60,  cosAngle30, -cosAngle60, -cosAngle30],
        #                             [cosAngle30, -cosAngle60, -cosAngle30,  cosAngle60]])
       
        self.IrwArray = np.linalg.pinv(self.Irw * self.motorAngles)
        
        # Optimal PID Controller, LQR can't work with no gravity gradient torque
        z = np.sqrt((np.log(self.overshoot)**2.0)/((np.pi**2.0)+(np.log(self.overshoot)**2.0)))
        zwn = 4/self.ts
        wn = zwn/z

        #Calculate gains
        self.kd = 2*zwn*np.diag(self.I)
        self.kp = (wn**2)*np.diag(self.I)
    
        print("I: %s" % self.I)
        print("%%os:%s, ts: %s" % (self.overshoot, self.ts))
        print("KD: %s, KP: %s" % (self.kd, self.kp))

        # Gain setup
        #k1 = (I[1,1] - I[2,2])/I[0,0]
        #k2 = (I[1,1] - I[2,2])/I[1,1]
        #k3 = (I[1,1] - I[0,0])/I[2,2]

        #mu = 398600 #km^3/s^2 (Earth)
        #a = 6378; #km (Earth)
        #n = np.sqrt(mu / a**3)

        #F = -2 * n**2 * np.array([[4*k1, 0, 0], [0, 3*k2, 0], [0, 0, k3]])
        #G = n * np.array([[0, 0, (1.0-k1)], [0, 0, 0], [(k3-1.0), 0, 0 ]])

        # CARE Equation
        #A_top = np.concatenate((np.zeros([3,3]), 0.5*np.eye(3)), axis=1)
        #A_bottom = np.concatenate((F, G), axis=1)
        
        #A = np.concatenate((A_top, A_bottom), axis=0)
        #B = np.concatenate((np.zeros([3,3]), np.diag(np.diag(inverseI))), axis=0) #TODO Should we mask along the diagonal like MATLAB is doing?
        #R = np.eye(3)
        
        ## Gains for nominal operation
        #Q_nominal = np.diag([1,1,1,10,10,10])
        #S_nominal = linalg.solve_continuous_are(A, B, Q_nominal, R)
        #self.K_nominal = -1 * np.dot(np.dot(linalg.inv(R), np.transpose(B)), S_nominal)

        ## Gains for yawSweep and yawSweepLockRoll operations
        #Q_yawSweep = np.diag([1,1,1,10,15,10])
        #S_yawSweep = linalg.solve_continuous_are(A, B, Q_yawSweep, R)
        #self.K_yawSweep = -1 * np.dot(np.dot(linalg.inv(R), np.transpose(B)), S_yawSweep)

        ## Gains for correctivePitch operation
        #Q_correctivePitch = np.diag([1,1,1,10,10,10])
        #S_correctivePitch = linalg.solve_continuous_are(A, B, Q_correctivePitch, R)
        #self.K_correctivePitch = -1 * np.dot(np.dot(linalg.inv(R), np.transpose(B)), S_correctivePitch)
        
        #log("Initialized A matrix:")
        #log(A)
        #log("Initialized B matrix:")
        #log(B)
        #log("Initialized R matrix:")
        #log(R)

        #log("---------- Initializing gain matricies ----------")
        #log("Initialized gain matrix K_nominal:")
        #log(self.K_nominal)
        #log("Initialized gain matrix K_yawsweep:")
        #log(self.K_yawSweep)
        #log("Initialized gain matrix K_correctivePitch:")
        #log(self.K_correctivePitch)
        #log("-------------------------------------------------")
       
    def adjustAttitude(self, lqrMode, q, qTarget, qError):
        if lqrMode == PdMode.nominal:
            #Do nothing, return qError
            return qError
        
        elif lqrMode == PdMode.correctivePitch:
            # Overwrite target with current position, and override pitch to zero.
            ypr = quat2ypr(q)
            ypr[1] = 0
            newTarget = ypr2quat(ypr)
            return getQuatError(q, newTarget)
        
        elif lqrMode == PdMode.yawSweep:
            #Override pitch to zero
            ypr = quat2ypr(qTarget)
            ypr[1] = 0
            newTarget = ypr2quat(ypr)
            return getQuatError(q, newTarget)
        
        elif lqrMode == PdMode.yawSweepLockRoll:
            #Override pitch to zero, set target roll to body roll.
            yprTarget = quat2ypr(qTarget)
            yprPrudentia = quat2ypr(q)
            ypr = [0,0,0]
            ypr[2] = yprPrudentia[2]
            ypr[1] = 0
            ypr[0] = yprTarget[0]
            newTarget = ypr2quat(ypr)
            return getQuatError(q, newTarget)
            
        else:
            log("Error in adjustAttitude! lqrMode was not set to an expected value! lqrMode: %s " % lqrMode)
            return None

    def getControllerType(self, qObserved, qError):
        #Returns a LqrMode enum representing the target override state.
        yprError = quat2ypr(qError)
        yawErrorAbs = np.abs(yprError[0])
        rollErrorAbs = np.abs(yprError[2])

        pitchInertial = np.abs(quat2ypr(qObserved)[1])
        
        if pitchInertial > self.correctivePitchThreshold:
            #Override target pitch to 0
            return PdMode.correctivePitch

        elif yawErrorAbs > self.yawSweepThreshold:

            if rollErrorAbs > self.rollSweepThreshold:
                #Override target pitch to 0, AND roll to bodyRoll
                return PdMode.yawSweepLockRoll
            else:
                #Override target pitch to 0
                return PdMode.yawSweep
        else:
            return PdMode.nominal

    def getTorque(self, lqrMode, qError, w):

        derivativeTorque = -((self.kd/np.diag(self.I))*w) 
        proportionalTorque = -((self.kp/np.diag(self.I)*qError[0:3])*np.diag(self.I))
        
        torque = derivativeTorque + proportionalTorque

        if self.enableSaturation:
            for i in range(3):
                if torque[i] > self.maxTorque[i]:
                    torque[i] = self.maxTorque[i]
                elif torque[i] < - self.maxTorque[i]:
                    torque[i] = - self.maxTorque[i]

        return torque


if __name__ == "__main__":
    cls = ControlLawSingleton()
    
    inputEuler = [90, -10, -90]
    q = ypr2quat(np.radians([0, 0, 0]))
    w = np.array([0, 0, 0])
    qTarget = ypr2quat(np.radians(inputEuler))

    res = cls.routineAttitudeInput(q, w, qTarget)


    log("Motor Angle Configuration: %s" % cls.motorAngle)
    log("IN q:             %s Euler: %s" % (q , np.degrees(quat2ypr(q))) )
    log("IN w:             %s" % w)
    log("IN qTarget:       %s Euler: %s" % (qTarget , np.degrees(quat2ypr(qTarget))) )
    log("-" * 20)
    log("OUT qError:       %s Euler: %s]" % (res.qError , np.degrees(quatError2ypr(res.qError))) )
    #log("OUT qAdjusted:    %s Euler: %s]" % (res.qErrorAdjusted , np.degrees(quat2ypr(res.qErrorAdjusted))) )
    log("OUT lqrMode:      %s" % res.pdMode)
    log("OUT Inert Torque: %s" % res.inertialTorque)
    #log("OUT Motor Torque: %s" % res.motorTorques)
    #log("OUT Motor Alpha:  %s (rad/s)" % res.motorAccels)
    log("OUT Motor Alpha:  %s (rpm)" % (res.motorAccels * 60 / (2 * np.pi)))

