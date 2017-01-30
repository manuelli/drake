
jointIdx = range(7)
addPlot(timeWindow=10, yLimits=[-50,50])

# addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, jointIdx)
addSignals('RESIDUAL_ACTUAL', msg.utime, msg.residual, jointIdx)
