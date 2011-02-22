# 
# Copyright CNRS-LAAS
# Author: Florent Lamiraux
#
# This script control nao robot in order to make a oscillatory motion of the
# ZMP between the feet centers and steps in place.
#

from math import sqrt, cos, sin, pi
from dynamic_graph.sot.se3 import R, R3, SE3
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas
from dynamic_graph.sot.dynamics.test import Stepper as StepperEntity
from dynamic_graph import enableTrace, plug
from dynamic_graph.sot.dynamics.solver import Solver

def toViewerConfig(config):
    return config + 10*(0.,)

try:
    import robotviewer
    clt = robotviewer.client()
    hasRobotViewer = True
    rvName = 'hrp'
except:
    hasRobotViewer = False
    print "no robotviewer"

timeStep = .005
gravity = 9.81

class Stepper(object):
    signalList = None
    def __init__(self, robot, solver):
        self.solver = solver
        self.robot = robot
        # Push tasks
        #  Feet tasks.
        self.solver.push(self.robot.name + '_task_right-ankle')
        self.solver.push(self.robot.name + '_task_left-ankle')

        # Center of mass
        self.robot.featureCom.selec.value = '111'
        self.solver.push(self.robot.name + '_task_com')

        # Head
        self.robot.gaze.selec.value = '111000'
        solver.push(self.robot.name + '_task_gaze')

        self.robot.tasks['right-ankle'].controlGain.value = 200.
        self.robot.tasks['left-ankle'].controlGain.value = 200.
        self.robot.tasks['left-ankle'].controlGain.value = 200.
        self.robot.tasks['gaze'].controlGain.value = 200.

        # Get height of center of mass
        self.robot.dynamic.com.recompute(0)
        com = self.robot.dynamic.com.value
        omega = sqrt(gravity/com[2])

        # Get positions of foot centers
        #
        ankleWrtFoot = self.robot.dynamic.getAnklePositionInFootFrame()
        footCenterWrtAnkle = R3(tuple(map(lambda x: -x, ankleWrtFoot)))
        lf = SE3(self.robot.dynamic.signal("left-ankle").value)*footCenterWrtAnkle
        # right foot by symmetry
        ankleWrtFoot = (ankleWrtFoot[0], -ankleWrtFoot[1], ankleWrtFoot[2])
        footCenterWrtAnkle = R3(tuple(map(lambda x: -x, ankleWrtFoot)))
        rf = SE3(self.robot.dynamic.signal("right-ankle").value)*footCenterWrtAnkle

        # Create and plug stepper entity
        #
        self.entity = StepperEntity("stepper")
        self.entity.setLeftFootCenter(lf.toTuple())
        self.entity.setRightFootCenter(rf.toTuple())
        self.entity.setLeftAnklePosition(self.robot.leftAnkle.position.value)
        self.entity.setRightAnklePosition(self.robot.rightAnkle.position.value)
        self.entity.setCenterOfMass(com)
        self.entity.setFootWidth(self.robot.dynamic.getSoleWidth())
        plug(self.entity.comGain, self.robot.comTask.controlGain)
        plug(self.entity.comReference, self.robot.featureComDes.errorIN)
        plug(self.entity.zmpReference, self.robot.device.zmp)
        plug(self.entity.leftAnkleReference, self.robot.leftAnkle.reference)
        plug(self.entity.rightAnkleReference, self.robot.rightAnkle.reference)

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
                self.entity.start()
            t = timeStep*i
            self.robot.device.increment(timeStep)
            config = self.robot.device.state.value
            path.append(config)
            if hasRobotViewer:
                clt.updateElementConfig(rvName, toViewerConfig(config))

            for signal, j in zip(self.signalList, range(n)):
                plot[j] = plot[j] + [signal(self.robot)]
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

def norm(v):
    return sqrt(reduce(lambda x,y:x + y*y, v, 0.))


if __name__ == '__main__':
    robot = Hrp2Laas("robot")
    solver = Solver(robot)

    def errorCom(robot):
        return norm(robot.comTask.error.value)
    errorCom.name = 'center of mass'

    def errorLa(robot):
        return norm(robot.tasks['left-ankle'].error.value)
    errorLa.name = 'left ankle'

    def errorRa(robot):
        return norm(robot.tasks['right-ankle'].error.value)
    errorRa.name = 'right ankle'

    def errorGaze(robot):
        return norm(robot.tasks['gaze'].error.value)
    errorGaze.name = 'gaze'

    signalList = [errorCom, errorLa, errorRa, errorGaze]
    p = Stepper(robot, solver)
    p.signalList = signalList
    path, plot = p.play()
    p.draw(plot)
