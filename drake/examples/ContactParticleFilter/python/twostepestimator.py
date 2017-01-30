__author__ = 'manuelli'
import numpy as np

from director import transformUtils
import director.vtkAll as vtk


from pythondrakemodel import PythonDrakeModel
import contactfilterutils as cfUtils

class TwoStepEstimator:

    def __init__(self, robotStateModel, robotStateJointController, linkMeshData, config):
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.config = config
        self.createDrakeModel()
        self.initializeRobotPoseTranslator()
        self.linkMeshData = linkMeshData

    def createDrakeModel(self, filename=None):
        self.drakeModel = PythonDrakeModel(self.config['robot']['floatingBaseType'], self.config['robot']['urdf'])

    def initializeRobotPoseTranslator(self):
        self.robotPoseTranslator = cfUtils.RobotPoseTranslator(self.robotStateModel.model, self.drakeModel.model)

    # be careful here if director and this use different models
    # for example if we are FIXED base and director has ROLLPITCHYAW
    def getCurrentPose(self):
        q_director = self.robotStateJointController.q
        q = self.robotPoseTranslator.translateDirectorPoseToRobotPose(q_director)
        return q

    def computeTwoStepEstimate(self, residual, linksWithContactForce):
        returnData = dict()
        if len(linksWithContactForce)==0:
            return returnData

        # do kinematics on our internal model
        q = self.getCurrentPose()
        self.drakeModel.model.setJointPositions(q)

        # stack the jacobians
        jacobianTransposeList = []

        for linkName in linksWithContactForce:
            linkId = self.drakeModel.model.findLinkID(linkName)
            J = self.drakeModel.geometricJacobian(0,linkId,linkId, 0, False)
            jacobianTransposeList.append(J.transpose())


        stackedJacobian = np.hstack(jacobianTransposeList)


        #do the pseudo inverse
        pinvJacobian = np.linalg.pinv(stackedJacobian)
        stackedWrenches = np.dot(pinvJacobian, residual)



        # unpack the wrenches
        for idx, linkName in enumerate(linksWithContactForce):
            startIdx = 6*idx
            endIdx = startIdx + 6
            wrench = stackedWrenches[startIdx:endIdx]
            torque = wrench[0:3]
            force = wrench[3:6]
            contactLocationData = self.computeContactLocation(linkName, force, torque)
            if contactLocationData is not None:
                returnData[linkName] = contactLocationData


        return returnData


    def computeContactLocation(self, linkName, force, torque):
        # want to find contactPoint such that force applied at contactPoint
        # leads to given torque, i.e. we want to solve for contactPoint such that
        # torque = contactPoint x force, where x denotes the cross product. This is
        # the same as solving torque = -force x contactPoint = -forceCross * contactPoint

        # everything here is in link frame
        forceCross = transformUtils.crossProductMatrix(force)
        forceCrossPseudoInverse = np.linalg.pinv(forceCross)
        contactPoint_d = -np.dot(forceCrossPseudoInverse, torque)

        forceNorm = np.linalg.norm(force)
        if forceNorm < 0.5:
            return None

        forceNormalized = force/forceNorm


        # now intersect line with linkMesh, choose the start and end of the ray
        # so that we find a contact point where the force is pointing "into" the link
        # mesh
        rayOrigin = contactPoint_d - 0.5*forceNormalized
        rayEnd = contactPoint_d + 0.5*forceNormalized

        ############# DEBUGGING
        # print ""
        # print "force", force
        # print "torque", torque
        # print "r_d", contactPoint_d
        # impliedTorque = np.cross(contactPoint_d, force)
        # print "implied torque", impliedTorque

        linkToWorld = self.robotStateModel.getLinkFrame(linkName)
        rayOriginInWorld = np.array(linkToWorld.TransformPoint(rayOrigin))
        rayEndInWorld = np.array(linkToWorld.TransformPoint(rayEnd))
        contactRayVisObjectName = linkName + " contact ray world frame"

        pt = self.raycastAgainstLinkMesh(linkName, rayOrigin, rayEnd)


        # data for later drawing and stuff
        d = dict()
        d['pt'] = pt # will be None if no intersection with link is found
        d['contactRay'] = dict()
        d['contactRay']['rayOriginInWorld'] = rayOriginInWorld
        d['contactRay']['rayEndInWorld'] = rayEndInWorld
        d['contactRay']['visObjectName'] = contactRayVisObjectName
        d['contactRay']['color'] = [1,1,0]
        d['force'] = force
        d['forceInWorld'] = linkToWorld.TransformVector(force)
        d['linkName'] = linkName


        if pt is not None:
            d['contactLocation'] = pt
            d['contactLocationInWorld'] = linkToWorld.TransformPoint(pt)


        return d

    def raycastAgainstLinkMesh(self, linkName, rayOrigin, rayEnd):
        meshToWorld = self.linkMeshData[linkName]['transform']
        rayOriginInWorld = np.array(meshToWorld.TransformPoint(rayOrigin))
        rayEndInWorld = np.array(meshToWorld.TransformPoint(rayEnd))

        # ### DEBUGGING
        # if self.showContactRay:
        #     d = DebugData()
        #     d.addLine(rayOriginInWorld, rayEndInWorld, radius=0.005)
        #     color=[1,0,0]
        #     obj = vis.updatePolyData(d.getPolyData(), "raycast ray in mesh frame", color=color)

        tolerance = 0.0 # intersection tolerance
        pt = [0.0, 0.0, 0.0] # data coordinate where intersection occurs
        lineT = vtk.mutable(0.0) # parametric distance along line segment where intersection occurs
        pcoords = [0.0, 0.0, 0.0] # parametric location within cell (triangle) where intersection occurs
        subId = vtk.mutable(0) # sub id of cell intersection

        result = self.linkMeshData[linkName]['locator'].IntersectWithLine(rayOriginInWorld, rayEndInWorld, tolerance, lineT, pt, pcoords, subId)

        # this means we didn't find an intersection
        if not result:
            return None

        # otherwise we need to transform it back to linkFrame
        worldToMesh = meshToWorld.GetLinearInverse()
        ptInLinkFrame = worldToMesh.TransformPoint(pt)
        return ptInLinkFrame