import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from KalmanFilter import KalmanFilter

class Robot:
    __handleRadius = 3
    __lineLenght = 6
    __circle = None
    __line = None
    #The static kinematics:
    # __position        2D vector    
    # __orientation     floating point value
    # __speed        floating point value
    # __rotation        floating point value

    #The steering kinematics:
    # linear/acceleration   floating point value
    # angular              floating point value
        
    def __init__(self, initialPosition, initialOrientation):
        self.__setPosition(initialPosition) 
        self.__orientation = initialOrientation 
        self.__speed = 0.0
        self.__rotation = 0.0

    def __str__(self):
        return "drone: pos="+str(self.__position)+" ori="+str(self.__orientation)+" vel="+str(self.__speed)+" rot="+str(self.__rotation)

    def __setPosition(self, newPosition):
        self.__position = newPosition
        if self.__circle is not None:
            self.__circle.center = newPosition
            xList = [newPosition[0],newPosition[0]+np.cos(self.__orientation)*self.__lineLenght]
            self.__line[0].set_xdata(xList)
            yList = [newPosition[1],newPosition[1]+np.sin(self.__orientation)*self.__lineLenght]
            self.__line[0].set_ydata(yList)
 
    def getPosition(self):
        return self.__position

    def getVelocity(self):
        return np.matrix([[self.__speed*np.cos(self.__orientation)], 
                          [self.__speed*np.sin(self.__orientation)]])

    #GPSdevice simulator
    def getGPSSimPosition(self):
        mean = self.getPosition().getA1()
        cov = [[100,  0],
               [  0,100]]
        return np.random.multivariate_normal(mean,cov)

    def getGPSSimVelocity(self):
        mean = self.getVelocity().getA1()
        cov = [[0.01, 0],
               [   0, 0.01]]
        return np.random.multivariate_normal(mean,cov)


    #Drawing drone
    def setupDrawing(self, figure):
        centerX, centerY = self.__position[0,0],self.__position[1,0]
        self.__circle = plt.Circle((centerX,centerY), self.__handleRadius,fc=np.random.random(3),picker=True, alpha=0.5)
        figure.gca().add_patch(self.__circle)
        xList = [centerX,centerX+np.cos(self.__orientation)*self.__lineLenght]
        yList = [centerY,centerY+np.sin(self.__orientation)*self.__lineLenght]
        self.__line = figure.gca().plot(xList, yList)
    
    #move the drone by curser
    def pickedUpAndMoved(self, newPosition, index):
        #moving a Drone
        if newPosition is not None:
            self.__setPosition(newPosition)

    def kinematicsUpdate(self, linear, angular): 
        #update position and orientation
        self.__setPosition(self.__position + self.__speed * np.matrix([[np.cos(self.__orientation)], [np.sin(self.__orientation)]]))
        self.__orientation += self.__rotation

        #update steering: velocity and rotation 
        self.__speed += linear
        self.__rotation = np.max(np.min(angular, np.pi*0.01),-np.pi*0.01)

#Plot
plt.ion()
figure = plt.figure('map')
figure.gca().set_aspect('equal')


#make the drone
robot = Robot(np.matrix([[0],
                         [0]]), 0)
robot.setupDrawing(figure)

#Initial acceleration 
angular= 0
linear = 0
def updateDrone():
    global angular,linear
    robot.kinematicsUpdate(linear, angular)
    figure.canvas.draw()
    angular= 0.0
    linear = 0.0 #constant speed/no acceleration

droneTimer = figure.canvas.new_timer(interval=100)
droneTimer.add_callback(updateDrone)
droneTimer.start()

# Filter for GPS position
#State:
#   position X
#   position Y
#   Speed X / change in position X
#   Speed Y / change in position Y
class Position2DFilter():
    __kalmanFilter = None    

    #ini_x - initial state
    #iniP - initial covariance to be in this state
    def __init__(self, init_x, initP):
        #State transiition matrix - previously state to the current state
        A = np.matrix([[1.0, 0.0, 1.0, 0.0],
                       [0.0, 1.0, 0.0, 1.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])

        #Control vector - Maps control input to state change
        B = np.matrix([[0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])

        #Observer - maps state to meassure
        H = np.matrix([[1.0, 0.0, 0.0, 0.0],  
                       [0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]])

        ##The noise covariance matrices - describe the statistics of the noise##

        # Covariance matrix of the (transition) process noise
        Q = np.matrix([[0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0]]) 
 
        # Covariance matrix of the (sensor) observation noise
        R = np.matrix([[100.0, 0.0, 0.0, 0.0],
                       [0.0, 100.0, 0.0, 0.0],
                       [0.0, 0.0, 0.001, 0.0],
                       [0.0, 0.0, 0.0, 0.001]])        

        #Construct the Kalman Filter
        self.__kalmanFilter = KalmanFilter(A,B,H,init_x, initP, Q, R)      

    #Add GPS event position to the filter
    def AddObservedPosition(self, obsState):
        self.__kalmanFilter.Prediction(np.matrix([[0.0], 
                                                  [0.0],
                                                  [0.0], 
                                                  [0.0]
                                                 ]))
        self.__kalmanFilter.Update(obsState)

    def getState(self):
        return self.__kalmanFilter.x

#make the kalman filter
robotPos = robot.getGPSSimPosition()
initialState = np.matrix([[robotPos[0]], 
                          [robotPos[1]],
                          [0.0],
                          [0.0]]) #assume the root to start with no speed

initialCov = np.matrix([[1, 0, 0, 0], 
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
posFilter = Position2DFilter(initialState,initialCov);

actualPositionsX = []
actualPositionsY = []
actualTrail = figure.gca().plot(actualPositionsX,actualPositionsY,".-")
def updateActualPosition():
    pos = robot.getPosition();  
    actualPositionsX.append(pos[0])
    actualPositionsY.append(pos[1])
    actualTrail[0].set_xdata(actualPositionsX)
    actualTrail[0].set_ydata(actualPositionsY)
    figure.canvas.draw()

gpsSimPositionsX = []
gpsSimPositionsY = []
gpsSimTrail = figure.gca().plot(gpsSimPositionsX,gpsSimPositionsY,"x--")
def updateSimulatedGPSPosition():
    gpsSimPos = robot.getGPSSimPosition(); 
    gpsSimVel = robot.getGPSSimVelocity();
    observedState = np.matrix([[gpsSimPos[0]],
                               [gpsSimPos[1]],
                               [gpsSimVel[0]],
                               [gpsSimVel[1]]]);

    posFilter.AddObservedPosition(observedState);

    gpsSimPositionsX.append(gpsSimPos[0])
    gpsSimPositionsY.append(gpsSimPos[1])
    gpsSimTrail[0].set_xdata(gpsSimPositionsX)
    gpsSimTrail[0].set_ydata(gpsSimPositionsY)
    figure.canvas.draw()

filterGpsSimPositionsX = []
filterGpsSimPositionsY = []
filterGpsSimTrail = figure.gca().plot(filterGpsSimPositionsX,filterGpsSimPositionsY,"o-")
def updateFilterGPSPosition():
    # Use Kalman filter
    filterGpsSimPos = [posFilter.getState()[0,0], posFilter.getState()[1,0]]
    filterGpsSimPositionsX.append(filterGpsSimPos[0])
    filterGpsSimPositionsY.append(filterGpsSimPos[1])
    filterGpsSimTrail[0].set_xdata(filterGpsSimPositionsX)
    filterGpsSimTrail[0].set_ydata(filterGpsSimPositionsY)
    figure.canvas.draw()
    
    print(posFilter.getState())

droneTimer = figure.canvas.new_timer(interval=1)
droneTimer.add_callback(updateActualPosition)
droneTimer.add_callback(updateSimulatedGPSPosition)
droneTimer.add_callback(updateFilterGPSPosition)
droneTimer.start()

#State for user picking logic 
movable = None
index = None #spline path controle point index

def on_pick(event):
    pass

def motion_notify_event(event):
    global figure
    if movable is not None:
        movable.pickedUpAndMoved(np.matrix([[event.xdata], 
                                            [event.ydata]]), index)
    figure.canvas.draw()

def button_press_event(event):
    pass

def button_release_event(event):
    global movable, index
    movable, index = None, None


def key_press_callback(event):
    global angular,linear
    if event.key=='up':
        linear += 0.33
        #print "^"
    elif event.key=='down':
        linear -= 0.33
        #print "V"
    elif event.key=='left':
        angular += 0.15*np.pi
        #print "<"
    elif event.key=='right':
        angular -= 0.15*np.pi
        #print ">"
    elif event.key=='escape':
        exit()
        


figure.canvas.mpl_connect('pick_event', on_pick)
figure.canvas.mpl_connect('motion_notify_event', motion_notify_event)
figure.canvas.mpl_connect('button_press_event', button_press_event)
figure.canvas.mpl_connect('button_release_event', button_release_event)
figure.canvas.mpl_connect('key_press_event', key_press_callback)

#Disable default keys on the plot e.g. 'k' is logaritmic scale
figure.canvas.mpl_disconnect(figure.canvas.manager.key_press_handler_id)

#Set the axis limits
plt.axis([-150, 150, -100, 100])

#Display the figures on screen
plt.show(block=True)

