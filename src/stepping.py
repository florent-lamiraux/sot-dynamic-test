#!/usr/bin/python
#
# Copyright CNRS-LAAS
# Author: Florent Lamiraux
#
# This script controls a humanoid robot in order to make a oscillatory
# motion of the ZMP between the feet centers and steps in place.
#

from math import sqrt, cos, sin, pi
from dynamic_graph.tools import addTrace
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.se3 import R, R3, SE3
from dynamic_graph.sot.hrp2_14.robot import Robot
from dynamic_graph.sot.dynamics.test import Stepper
from dynamic_graph import enableTrace, plug
from dynamic_graph.sot.dynamics.solver import Solver
from dynamic_graph.sot.core import Multiply_of_matrixHomo
from dynamic_graph.sot.dynamics import FlexibilityCompensation

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
        self.robot.tasks['waist'].controlGain.value = 10.
        self.solver.push(self.robot.tasks ['waist'])
        # Get height of center of mass
        self.robot.com.recompute(0)
        self.zCom = self.robot.com.value [2]

        # Correction of flexibility deviation on waist
        #
        self.waistRef = Multiply_of_matrixHomo (robot.name + '_waist_ref')
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

        cf = R(.5)*(lf + rf)
        com  = (cf [0], cf [1], self.zCom)
        # Create and plug stepper entity
        #
        self.stepper = Stepper("stepper")
        self.stepper.setLeftFootCenter(lf.toTuple())
        self.stepper.setRightFootCenter(rf.toTuple())
        self.stepper.setLeftAnklePosition(self.robot.leftAnkle.position.value)
        self.stepper.setRightAnklePosition(self.robot.rightAnkle.position.value)
        self.stepper.setCenterOfMass(com)
        self.stepper.setFootWidth(.5*self.robot.dynamic.getSoleWidth())
        self.stepper.setAngularFrequency (2.*pi)
        self.stepper.setMagnitude (0.)
        self.stepper.setMaxComGain(100.)
        robot.stabilizer.setGain2 ((66.84, 45.57, 22.67, -2.14))
        correction_waist = FlexibilityCompensation ('correction_waist')
        plug (self.robot.stabilizer.flexPosition, correction_waist.flexPosition)
        plug (self.robot.stabilizer.flexVelocity, correction_waist.flexVelocity)
        correction_waist.globalPosition.value = robot.waist.reference.value
        correction_waist.globalVelocity.value = 6*(0.,)
        plug (correction_waist.localPosition, robot.waist.reference)
        plug (correction_waist.localVelocity, robot.waist.velocity)

        plug (robot.stabilizer.stateFlex_inv, self.waistRef.sin1)
        self.waistRef.sin2.value = robot.waist.reference.value
        plug (self.waistRef.sout, robot.waist.reference)
        plug(self.stepper.comGain, self.robot.comTask.controlGain)
        plug(self.stepper.comReference, self.robot.comRef)
        plug(self.stepper.comdot, self.robot.comdot)
        plug(self.stepper.leftAnkleReference, self.robot.leftAnkle.reference)
        plug(self.stepper.rightAnkleReference, self.robot.rightAnkle.reference)

        # Initialize tracer
        self.tracer = TracerRealTime('trace')
        self.tracer.setBufferSize(2**23)
        self.tracer.open('/tmp/','dg_','.dat')
        # Recompute trace.triger at each iteration to enable tracing.
        self.robot.device.after.addSignal('{0}.triger'.format(self.tracer.name))
        # Trace signals
        self.addTrace (self.robot.device.name, 'forceLLEG')
        self.addTrace (self.robot.device.name, 'forceRLEG')
        self.addTrace(self.robot.dynamic.name, 'com')
        self.addTrace(self.robot.device.name, 'robotState')
        self.addTrace(self.robot.device.name, 'accelerometer')
        self.addTrace(self.robot.device.name, 'gyrometer')

    def addTrace (self, entityName, signalName):
        addTrace (self.robot, self.tracer, entityName, signalName)

    def start (self):
        self.tracer.start ()
        self.stepper.start ()

    def stop (self):
        self.stepper.stop ()
        self.tracer.stop ()
        self.tracer.dump ()
        self.tracer.close ()
        self.tracer.clear ()

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
    robot = Robot ("robot")
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

