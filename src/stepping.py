# 
# Copyright CNRS-LAAS
# Author: Florent Lamiraux
#
# This script control nao robot in order to make a oscillatory motion of the
# ZMP between the feet centers and steps in place.
#

from math import sqrt, cos, sin, pi
from dynamic_graph.sot.core import RobotSimu, VectorConstant, MatrixConstant, RPYToMatrix, Derivator_of_Vector, FeatureGeneric, FeatureJointLimits, Compose_R_and_T, Task, Constraint, GainAdaptive, SOT, MatrixHomoToPoseRollPitchYaw

from dynamic_graph.sot import R3, SE3
from dynamic_graph.sot.se3 import R
from dynamic_graph.sot.dynamics.humanoid_robot import HumanoidRobot
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas
from dynamic_graph.sot.dynamics.test import Stepper
from dynamic_graph import enableTrace, plug

def toViewerConfig(config):
    return config + 10*(0.,)

robot = Hrp2Laas("robot")
rvName = 'hrp'

try:
    import robotviewer
    clt = robotviewer.client()
    hasRobotViewer = True
except:
    hasRobotViewer = False
    print "no robotviewer"

timeStep = .005
gravity = 9.81

solver = SOT('solver')
solver.damping.value = 1e-6
solver.setNumberDofs(robot.dimension)
plug(solver.control, robot.device.control)
plug(robot.device.state, robot.dynamic.position)

# Push tasks
#  Feet tasks.
solver.push(robot.name + '_task_right-ankle')
solver.push(robot.name + '_task_left-ankle')

# Center of mass
robot.featureCom.selec.value = '111'
solver.push(robot.name + '_task_com')

# Head
robot.gaze.selec.value = '111000'
solver.push(robot.name + '_task_gaze')

robot.tasks['right-ankle'].controlGain.value = 200.
robot.tasks['left-ankle'].controlGain.value = 200.
robot.tasks['left-ankle'].controlGain.value = 200.
robot.tasks['gaze'].controlGain.value = 200.

# Get height of center of mass
robot.dynamic.com.recompute(0)
com = robot.dynamic.com.value
omega = sqrt(gravity/com[2])

# Get positions of foot centers
#
ankleWrtFoot = robot.dynamic.getAnklePositionInFootFrame()
footCenterWrtAnkle = R3(tuple(map(lambda x: -x, ankleWrtFoot)))
lf = SE3(robot.dynamic.signal("left-ankle").value)*footCenterWrtAnkle
# right foot by symmetry
ankleWrtFoot = (ankleWrtFoot[0], -ankleWrtFoot[1], ankleWrtFoot[2])
footCenterWrtAnkle = R3(tuple(map(lambda x: -x, ankleWrtFoot)))
rf = SE3(robot.dynamic.signal("right-ankle").value)*footCenterWrtAnkle

# Create and plug stepper
#
stepper = Stepper("stepper")
stepper.setLeftFootCenter(lf.toTuple())
stepper.setRightFootCenter(rf.toTuple())
stepper.setLeftAnklePosition(robot.leftAnkle.position.value)
stepper.setRightAnklePosition(robot.rightAnkle.position.value)
stepper.setCenterOfMass(com)
stepper.setFootWidth(robot.dynamic.getSoleWidth())
plug(stepper.comGain, robot.comTask.controlGain)
plug(stepper.comReference, robot.featureComDes.errorIN)
plug(stepper.zmpReference, robot.device.zmp)
plug(stepper.leftAnkleReference, robot.leftAnkle.reference)
plug(stepper.rightAnkleReference, robot.rightAnkle.reference)

def norm(v):
    return sqrt(reduce(lambda x,y:x + y*y, v, 0.))

class Player (object):
    """
    Run a simulation by incrementing time and computing state along time
    """
    def __init__(self, signalList):
        self.signalList = signalList
        
    def play(self):
        totalTime = 25.
        nbSteps = int(totalTime/timeStep) + 1
        path = []
        n = len(self.signalList)
        # Initialize empty list for each task
        plot = n*[[]]
        
        
        for i in xrange(nbSteps):
            print (i)
            if i == 10:
                stepper.start()
            t = timeStep*i
            robot.device.increment(timeStep)
            config = robot.device.state.value
            path.append(config)
            if hasRobotViewer:
                clt.updateElementConfig(rvName, toViewerConfig(config))

            for signal, j in zip(self.signalList, range(n)):
                plot[j] = plot[j] + [signal()]
        return (tuple(path), plot)

    def draw(self, plot):
        import matplotlib.pyplot as pl
        fig = []
        ax = []
        for p, signal in zip(plot, self.signalList):
            fig.append(pl.figure())
            ax.append(fig[-1].add_subplot(111))
            ax[-1].plot(map(lambda x: timeStep*x, xrange(len(p))), p)
            leg = ax[-1].legend((signal.name,))
        pl.show()


if __name__ == '__main__':
    signalList = [robot.comTask,robot.tasks['left-ankle'],robot.tasks['right-ankle']]
    p = Player(signalList)
    path, plot = p.play(3.)
    p.draw(plot)
