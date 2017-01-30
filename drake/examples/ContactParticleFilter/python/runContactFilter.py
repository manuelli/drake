__author__ = 'manuelli'
"""usage: directorPython runContactFilter.py -v5"""
# the -v5 is to get it to use atlas_v5. If you don't specify this it will try to launch with Valkyrie which currently
# doesn't work

from director.consoleapp import ConsoleApp
from director import visualization as vis
from director import roboturdf
from director import robotsystem
from director import drcargs
import contactfilter



app = ConsoleApp()
view = app.createView()



# robotStateModel, robotStateJointController = roboturdf.loadRobotModel('robot state model', view, parent='sensors', color=roboturdf.getRobotGrayColor(), visible=True)
# robotStateJointController.addLCMUpdater('EST_ROBOT_STATE')
# robotStateModel.addToView(view)

robotSystem = robotsystem.create(view)
contactFilter = contactfilter.ContactFilter(robotSystem.robotStateModel, robotSystem.robotStateJointController)
contactFilter.start()
view.show()
app.start()

