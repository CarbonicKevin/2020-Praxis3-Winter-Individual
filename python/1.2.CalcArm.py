import numpy as np
import matplotlib.pyplot as plt
import scipy

class arm:
    def __init__(self, lList, delta, p0=np.array([0,0], dtype=float)):
        """
        Input Variables
        lList = [l1, l2]
        """
        # Defining robot properties
        self.l1 = lList[0]
        self.l2 = lList[1]

        # Defining Delta, the offset from the EE pos
        self.delta = np.array([delta, 0])

        # Defining Joint Vectors
        self.p0 = p0
        self.p1 = np.array([None, None], dtype=float)
        self.p2 = np.array([None, None], dtype=float)

        # Defining Required Joint Angles
        self.gamma0 = None
        self.gamma1 = None
        self.gamma2 = None

        # Defining L and Theta
        self.L     = None
        self.theta = None
        self.alpha = None

        # Defining Configuration Space Variables
        self.p0Angle = None
        self.p1Angle = None
        self.p2Angle = None

    def invKin(self, coord):
        """
        Input Variables
        coord = np.array([delta, 0])
        """
        self.p2 = coord-self.delta

        self.theta = np.arctan2(self.p2[1], self.p2[0])    # angle to desired end effector 
        self.L     = np.sqrt(self.p2[0]**2+self.p2[1]**2)  # Dist to desired end effector

        self.gamma1 = np.arccos((self.l1**2+self.l2**2-self.L**2)/(2*self.l1*self.l2))
        self.gamma0 = np.arcsin((self.l2/self.L)*np.sin(self.gamma1))
        self.gamma2 = np.arcsin((self.l1/self.L)*np.sin(self.gamma1))

        self.alpha = np.pi-self.theta

        if abs(2*np.pi-(self.alpha + self.gamma2)) > np.pi/2: sign = 1
        else: sign = -1

        self.p0Angle = (self.theta+sign*self.gamma0)
        self.p1Angle = (np.pi - self.gamma1)
        self.p2Angle = self.gamma2-(2*np.pi-(self.alpha + sign*self.gamma2)-(2*np.pi-2*self.gamma2)/2)

        self.p1 = self.p0 + np.array([self.l1*np.cos(self.p0Angle), self.l1*np.sin(self.p0Angle)])

        self.p0Angle = self.p0Angle * (180/np.pi) * -1
        self.p1Angle = self.p1Angle * (180/np.pi) * (-1*sign)
        self.p2Angle = self.p2Angle * (180/np.pi) * -1


    def plt(self):
        fig, ax = plt.subplots()

        # Plot grid
        ax.plot([ 50,  75, 100], [-50, -50, -50], 'k')
        ax.plot([ 50,  75, 100], [  0,   0,   0], 'k')
        ax.plot([ 50,  75, 100], [ 50,  50,  50], 'k')
        ax.plot([ 50,  50,  50], [ 50,   0, -50], 'k')
        ax.plot([ 75,  75,  75], [ 50,   0, -50], 'k')
        ax.plot([100, 100, 100], [ 50,   0, -50], 'k')

        ax.plot([ 50,  75, 100], [-50, -50, -50], 'bo', linestyle='none')
        ax.plot([ 50,  75, 100], [  0,   0,   0], 'bo', linestyle='none')
        ax.plot([ 50,  75, 100], [ 50,  50,  50], 'bo', linestyle='none')
        ax.plot([ 50,  50,  50], [ 50,   0, -50], 'bo', linestyle='none')
        ax.plot([ 75,  75,  75], [ 50,   0, -50], 'bo', linestyle='none')
        ax.plot([100, 100, 100], [ 50,   0, -50], 'bo', linestyle='none')

        # Plot Joints
        ax.plot(self.p0[0], self.p0[1], 'o')
        ax.plot(self.p1[0], self.p1[1], 'o')
        ax.plot(self.p2[0], self.p2[1], 'o')

        # Connect Joints
        leng = np.sqrt((self.p0[0]-self.p1[0])**2+(self.p0[1]-self.p1[1])**2)
        ax.plot([self.p0[0], self.p1[0]], [self.p0[1], self.p1[1]], 'm', label="l1: {:.3} cm".format(leng))
        #ax.text(abs(self.p0[0]-self.p1[0])/2-10, abs(self.p0[1]-self.p1[1])/2, "l1: {:.3} cm".format(leng), bbox=bbox)

        # Connect Joints
        leng = np.sqrt((self.p1[0]-self.p2[0])**2+(self.p1[1]-self.p2[1])**2)
        #bbox=dict(boxstyle="square", ec='cyan', fc='cyan')
        ax.plot([self.p1[0], self.p2[0]], [self.p1[1], self.p2[1]], 'c', label="l2: {:.3} cm".format(leng))
        #ax.text(abs(self.p1[0]-self.p2[0])/2-10+self.p1[0], abs(self.p1[1]-self.p2[1])/2+self.p1[1], "l2: {:.3} cm".format(leng), bbox=bbox)

        ax.plot([self.p2[0], self.delta[0]+self.p2[0]], [self.p2[1], self.p2[1]], 'r', label="End Effector")

        textstr = "P0 Angle: {:.4}\nP1 Angle: {:.4}\nP2 Angle: {:.4}".format(self.p0Angle, self.p1Angle, self.p2Angle)
        props = dict(boxstyle='round', facecolor='red', alpha=0.5)
        ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14, verticalalignment='top', bbox=props)

        ax.axis('square')
        ax.legend()
        plt.savefig("C:/Users/kevin/Documents/GitHub/Praxis-3-2020-Winter-Individual/python/images/{},{}.png".format(self.p2[0]+self.delta[0], self.p2[1]))

a = arm([60,60], 13.3447, p0=[0,0])

a.invKin(np.array([120,  50]))
a.plt()
a.invKin(np.array([120,   0]))
a.plt()
a.invKin(np.array([120, -50]))
a.plt()

a.invKin(np.array([ 75,  50]))
a.plt()
a.invKin(np.array([ 75,   0]))
a.plt()
a.invKin(np.array([ 75, -50]))
a.plt()

a.invKin(np.array([ 50,  50]))
a.plt()
a.invKin(np.array([ 50,   0]))
a.plt()
a.invKin(np.array([ 50, -50]))
a.plt()