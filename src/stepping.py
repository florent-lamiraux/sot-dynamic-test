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
from dynamic_graph.tracer_real_time import *

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
        #  Feet tasks.
        self.solver.push(self.robot.name + '_task_right-ankle')
        self.solver.push(self.robot.name + '_task_left-ankle')

        # Center of mass
        self.robot.featureCom.selec.value = '111'
        self.solver.push(self.robot.name + '_task_com')

        # Waist
        self.robot.waist.selec.value = '111000'
        solver.push(self.robot.name + '_task_waist')

        self.robot.tasks['right-ankle'].controlGain.value = 200.
        self.robot.tasks['left-ankle'].controlGain.value = 200.
        self.robot.tasks['gaze'].controlGain.value = 200.

        # Get height of center of mass
        self.robot.dynamic.com.recompute(0)
        com = self.robot.dynamic.com.value
        zCom = com[2]
        omega = sqrt(gravity/zCom)

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
        self.stepper = Stepper("stepper")
        self.stepper.setLeftFootCenter(lf.toTuple())
        self.stepper.setRightFootCenter(rf.toTuple())
        self.stepper.setLeftAnklePosition(self.robot.leftAnkle.position.value)
        self.stepper.setRightAnklePosition(self.robot.rightAnkle.position.value)
        self.stepper.setCenterOfMass(com)
        self.stepper.setFootWidth(.25*self.robot.dynamic.getSoleWidth())
        self.stepper.setStepHeight(.04*zCom)
        self.stepper.setMaxComGain(200.)
        plug(self.stepper.comGain, self.robot.comTask.controlGain)
        plug(self.stepper.comReference, self.robot.featureComDes.errorIN)
        plug(self.stepper.zmpReference, self.robot.device.zmp)
        plug(self.stepper.leftAnkleReference, self.robot.leftAnkle.reference)
        plug(self.stepper.rightAnkleReference, self.robot.rightAnkle.reference)

        # Trace signals
        self.tracer = TracerRealTime('tr')
        self.tracer.setBufferSize(10485760)
        self.tracer.open('/tmp/','stepper_','.plot')
        self.tracer.add('robot_device.zmp','zmp')
        self.tracer.add('robot_dynamic.com', 'com')
        self.tracer.add('stepper.leftAnkleReference', 'leftAnkle')
        self.tracer.add('stepper.rightAnkleReference', 'rightAnkle')
        self.tracer.add('robot_dynamic.zmp', 'dyn_zmp')
        self.tracer.start()
        self.robot.device.after.addSignal('tr.triger')

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
                self.stepper.start()
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

    def yCom(robot):
        return robot.featureCom.errorIN.value[1]
    yCom.name = 'y center of mass'

    def errorCom(robot):
        return norm(robot.comTask.error.value)
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
    m.tracer.dump()
    m.draw(plot)

