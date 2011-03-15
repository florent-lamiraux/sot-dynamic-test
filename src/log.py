#!/usr/bin/python

from dynamic_graph.sot.openhrp.log import Log
from dynamic_graph.sot.se3 import R3
import matplotlib.pyplot as pl

if __name__ == '__main__':
    #
    # Usage: log.py directory prefix
    #    read in directory files
    #      ${prefix}-astate
    #      ${prefix}-rstate
    #      ${prefix}-kf
    # and plot some of values
    import sys

    directory = None
    prefix = None
    if len(sys.argv) > 1:
        directory = sys.argv[1]
    if len(sys.argv) > 2:
        prefix = sys.argv[2]
    l = Log(prefix = prefix, directory = directory)
    # Compute zmp for double support
    la = R3((0., 0.095, 0.)) 
    ra = R3((0., -0.095, 0.))
    l.computeZmpDoubleSupport(ra, la)

    fig1 = pl.figure()
    ax1 = fig1.add_subplot(211)
    ax2 = fig1.add_subplot(212)
    zmpRfx = []
    zmpLfx = []
    zmpRfy = []
    zmpLfy = []
    zmpx = []
    zmpy = []
    Mrx = []
    Mry = []
    Mrz = []
    Mlx = []
    Mly = []
    Mlz = []

    Fnr = []
    Fnl = []
    for (F0, F1) in zip(l.forceRa,l.forceLa):
        Fnr.append(F0[2])
        Fnl.append(F1[2])
    for (zRight, zLeft, z) in zip(l.zmpRf, l.zmpLf,
                                  l.zmpDoubleSupport):
        zmpRfx.append(zRight[0])
        zmpRfy.append(zRight[1])
        zmpLfx.append(zLeft[0])
        zmpLfy.append(zLeft[1])
        zmpx.append(z[0])
        zmpy.append(z[1])

    time = map(lambda x:.004*x, range(len(Fnr)))
    ax1.plot(time, Fnr)
    ax1.plot(time, Fnl)

    time = map(lambda x:.004*x, range(len(zmpRfx)))
    ax2.plot(time, zmpx)
    ax2.plot(time, zmpy)

    ax1.legend(('force right ankle', 'force left ankle'))
    ax2.legend(('x zmp double support', 'y zmp double support'))
    pl.show()
    
