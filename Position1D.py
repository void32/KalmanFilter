import numpy
import matplotlib.pyplot as plt
import random

class Kalman1D:
    def __init__(self,A,B,H,x,P):
        self.A = A #State transition matrix - maps state at t-1 to state at t
        self.B = B #Control-input matrix  - maps control input to state change
        self.H = H #Observation/Emission matrix - maps from state to observation (some times this is just removing other entries than the observed fx H*x=[0 0 1][pos vel acc]=[acc])
        self.x = x #State vector of the Markov chain- mean for the gausian distribution
        self.P = P #Covarince matrix - try to plot this to visualize the gaussian distribution for time t

        #The noise covariance matrices - describe the statistics of the noises
        self.Q = numpy.matrix([0.00001]) #Noise when updating the covariance matrix 
        self.R = numpy.matrix([0.1]) #Covariance of the sensor noise

    #Calculate state vector and covariance for next time step given previous state and current control input
    def Prediction(self, u):
        #u: Control-input
        self.x = self.A*self.x + self.B*u #Prediction of the mean state at time t
        self.P = self.A*self.P*numpy.transpose(self.A)+self.Q #Prediction of the covarince at time t - how certain are we that x is close to the actual state

    #Update with sensor data and minimize the variance for the current state estimate P
    def Update(self, z): 
        #z: observation/evidence - the sensor input
        est_z = self.H*self.x #estimated observation given the model and evidence up to t-1        

        #The kalmain gain k_gain that minimize the variance P (note: independent of z so it can be done offline)
        S = self.H*self.P*numpy.transpose(self.H) + self.R
        k_gain = self.P*numpy.transpose(self.H)*numpy.linalg.inv(S) #rate of how much we beleive in the observation at time t
        
        correction_term = z - est_z #differens from what the model estimated to what is observed
        self.x = self.x+k_gain*(correction_term) #update state with evidence for time t

        size = self.P.shape[0]
        I = numpy.eye(size) #The identity matrix.
        self.P = (I-k_gain*self.H)*self.P #update covariance with evidence for time t when we


def ComputeValue(mean, stdDeviation, N):
        retList = []
        for meassure in range(N):
            retList.append(random.normalvariate(mean, stdDeviation))
        return retList

#Example
value = 1.25
noise = 0.25 
noOfMeasurements = 200
initialState = 2.5

#State transiition matrix - previously state to the current state
A = numpy.matrix([1.0])

#Control vector - Maps control input to state change
B = numpy.matrix([0])

#Observer - maps state to meassure
H = numpy.matrix([1])

#Init state.
stateBegin = numpy.matrix([initialState])

#init variance
initP = numpy.matrix([0.1])

kalman1d = Kalman1D(A,B,H,stateBegin, initP)
#Create noisy measure values
pltMeassure = []
pltKalman = []
meassured = ComputeValue(value, noise, noOfMeasurements);
for measure in meassured:
    pltMeassure.append(measure)
    pltKalman.append(kalman1d.x[0,0])
    kalman1d.Prediction(0)
    kalman1d.Update(measure)

#Real values
plt.plot(range(noOfMeasurements), [value]*noOfMeasurements, 'g--')

#meassured values
plt.plot(range(noOfMeasurements), pltMeassure, 'b-')

#Filtered
plt.plot(range(noOfMeasurements), pltKalman, 'r-')

plt.show()
