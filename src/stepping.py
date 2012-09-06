#!/usr/bin/python
#
# Copyright CNRS-LAAS
# Author: Florent Lamiraux
#
# This script controls a humanoid robot in order to make a oscillatory
# motion of the ZMP between the feet centers and steps in place.
#

from math import sqrt, cos, sin, pi
from dynamic_graph.sot.se3 import R, R3, SE3
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas
from dynamic_graph.sot.dynamics.test import Stepper
from dynamic_graph import enableTrace, plug
from dynamic_graph.sot.dynamics.solver import Solver

rvName = 'hrp'

def toViewerConfig(config):
    return config + 10*(0.,)

try:
    import robotviewer
    clt = robotviewer.client()
    hasRobotViewer = True
except:
    hasRobotViewer = False
    print "no robotviewer"

timeStep = .005
gravity = 9.81

class Motion(object):
    signalList = []
    def __init__(self, robot, solver):
        self.solver = solver
        self.robot = robot
        # Push tasks
        # Waist
        self.robot.waist.selec.value = '111000'
        self.robot.tasks['waist'].controlGain.value = 180.
        self.solver.remove(robot.tasks ['posture'])
        self.solver.push(self.robot.tasks ['waist'])
        self.solver.push(robot.tasks ['posture'])

        # Get height of center of mass
        self.robot.com.recompute(0)
        com = self.robot.com.value
        self.zCom = com[2]
        omega = sqrt(gravity/self.zCom)

        # Get positions of foot centers
        #
        ankleWrtFoot = self.robot.dynamic.getAnklePositionInFootFrame()
        footCenterWrtAnkle = R3(tuple(map(lambda x: -x, ankleWrtFoot)))
        lf = SE3(self.robot.dynamic.signal("left-ankle").value)*\
            footCenterWrtAnkle
        # right foot by symmetry
        ankleWrtFoot = (ankleWrtFoot[0], -ankleWrtFoot[1], ankleWrtFoot[2])
        footCenterWrtAnkle = R3(tuple(map(lambda x: -x, ankleWrtFoot)))
        rf = SE3(self.robot.dynamic.signal("right-ankle").value)*\
            footCenterWrtAnkle

        # Create and plug stepper entity
        #
        self.stepper = Stepper("stepper")
        self.stepper.setLeftFootCenter(lf.toTuple())
        self.stepper.setRightFootCenter(rf.toTuple())
        self.stepper.setLeftAnklePosition(self.robot.leftAnkle.position.value)
        self.stepper.setRightAnklePosition(self.robot.rightAnkle.position.value)
        self.stepper.setCenterOfMass(com)
        self.stepper.setFootWidth(.5*self.robot.dynamic.getSoleWidth())
        self.stepper.setStepHeight(.08*self.zCom)
        self.stepper.setMaxComGain(200.)
        plug(self.stepper.comReference, self.robot.comRef)
        plug(self.stepper.comdot, self.robot.comdot)
        plug(self.stepper.zmpReference, self.robot.zmpRef)
        plug(self.stepper.leftAnkleReference, self.robot.leftAnkle.reference)
        plug(self.stepper.rightAnkleReference, self.robot.rightAnkle.reference)

        # Trace signals
        self.robot.initializeTracer ()
        self.robot.addTrace ('stepper', 'leftAnkleReference')
        self.robot.addTrace ('stepper', 'rightAnkleReference')
        self.robot.addTrace ('stepper', 'zmpReference')
        self.robot.startTracer ()

    def start (self):
        self.robot.stabilizer.setComGain (2.)
        self.stepper.start ()

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
                self.start()
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
    # Initialize the zmp signal to the current com.
    _com = robot.com.value
    robot.zmpRef.value = (_com[0], _com[1], 0.)

    # Create a solver.
    solver = Solver(robot)

    # Make sure com and feet desired positions match the current
    # positions.
    s = ['left-ankle', 'right-ankle']
    for i in s:
        robot.dynamic.signal(i).recompute(robot.dynamic.signal(i).time + 1)
        robot.features[i].reference.value = \
            robot.dynamic.signal(i).value
        robot.features[i]._feature.selec.value = '111111'
        robot.tasks[i].controlGain.value = 180.

    robot.comRef.value = robot.dynamic.com.value

    # Push com and feet tasks.
    #
    # The robot is currently in half-sitting, so this script freezes com
    # and feet position so that the robot will remain stable while the
    # user program is starting.
    solver.push (robot.tasks ['com'])
    for i in s:
        solver.push (robot.tasks [i])
    solver.push (robot.tasks ['posture'])

    def yCom(robot):
        return robot.com.value[1]
    yCom.name = 'y center of mass'

    def errorCom(robot):
        com = robot.stabilizer.com.value
        comDes = robot.stabilizer.comDes.value
        error = []
        for x,y in zip (com, comDes):
            error.append (x - y)
        return norm(error)
    errorCom.name = 'error center of mass'

    def errorLa(robot):
        return norm(robot.tasks['left-ankle'].error.value)
    errorLa.name = 'left ankle'

    def errorRa(robot):
        return norm(robot.tasks['right-ankle'].error.value)
    errorRa.name = 'right ankle'

    def errorGaze(robot):
        return norm(robot.tasks['gaze'].error.value)
    errorGaze.name = 'gaze'

    signalList = [yCom, errorCom, errorLa, errorRa, errorGaze]
    m = Motion(robot, solver)

    m.signalList = signalList
    path, plot = m.play()
    m.robot.stopTracer ()
    m.draw(plot)

