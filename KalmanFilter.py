import numpy
import matplotlib.pyplot as plt
import random

class KalmanFilter:
    def __init__(self,A,B,H,x,P, Q, R):
        self.A = A #State transition matrix - maps state at t-1 to state at t
        self.B = B #Control-input matrix  - maps control input to state change
        self.H = H #Observation/Emission matrix - maps from state to observation (some times this is just removing other entries than the observed fx H*x=[0 0 1][pos vel acc]=[acc])
        self.x = x #State vector of the Markov chain- mean for the gausian distribution
        self.P = P #Covarince matrix - try to plot this to visualize the gaussian distribution for time t

        #The noise covariance matrices - describe the statistics of the noises
        self.Q = Q #numpy.matrix([0.00001]) # Covariance of the (transition) process noise 
        self.R = R #numpy.matrix([0.1]) # Covariance of the (sensor) observation noise

    #Calculate state vector and covariance for next time step given previous state and current control input
    def Prediction(self, u):
        #u: Control-input
        self.x = self.A*self.x + self.B*u #Prediction of the mean state at time t
        self.P = self.A*self.P*numpy.transpose(self.A)+self.Q #Prediction of the covarince at time t - how certain are we that x is close to the actual state

    #Update with sensor data and minimize the variance for the current state estimate P
    def Update(self, z): 
        #z: observation/evidence - the sensor input
        # print(type(self.H),(self.H))
        # print(type(self.x),(self.x))
        est_z = self.x*self.H #estimated observation given the model and evidence up to t-1        

        #The kalmain gain k_gain that minimize the variance P (note: independent of z so it can be done offline)
        S = self.H*self.P*numpy.transpose(self.H) + self.R
        k_gain = self.P*numpy.transpose(self.H)*numpy.linalg.inv(S) #rate of how much we beleive in the observation at time t        
        correction_term = z - est_z #differens from what the model estimated to what is observed
        self.x = self.x+(correction_term)*k_gain #update state with evidence for time t

        size = self.P.shape[0]
        I = numpy.eye(size) #The identity matrix.
        self.P = (I-k_gain*self.H)*self.P #update covariance with evidence for time t when we

