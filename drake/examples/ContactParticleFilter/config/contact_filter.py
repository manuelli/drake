
jointIdx = range(7)
addPlot(timeWindow=10, yLimits=[-50,50])
# addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, joints, keyLookup=names)
# addSignals('RESIDUAL_OBSERVER_STATE_W_FOOT_FORCE', msg.utime, msg.residual, joints, keyLookup=names)
addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, joints, keyLookup=names)
addSignals('RESIDUAL_ACTUAL', msg.utime, msg.residual, joints, keyLookup=names)
