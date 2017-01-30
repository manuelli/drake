__author__ = 'manuelli'

import contactfiltergurobi
import forcesproqp
import numpy as np

NUM_FRICTION_CONE_BASIS_VECTORS = 4


# General interface that can be used by the CPF to solve the QP problems.
# Have the option of using either Gurobi or ForcesPro
class QPSolver:

    def __init__(self, numContactsList):
        self.numContactsList = numContactsList
        self.gurobi = contactfiltergurobi.ContactFilterGurobi(self.numContactsList)
        self.forcesPro = forcesproqp.ForcesProQP(self.numContactsList)

    def solve(self, numContacts, residual, H_list, weightMatrix, solverType='gurobi'):
        solnData = {}
        if solverType == 'gurobi':
            solnData = self.gurobi.solve(numContacts, residual, H_list, weightMatrix)
        elif solverType=='forcespro':
            solnData = self.forcesPro.solve(numContacts, residual, H_list, weightMatrix)
        else:
            ValueError("solver type must be one of gurobi or forcespro")

        return solnData


    def test(self, numContacts = 1):
        numVars = numContacts*NUM_FRICTION_CONE_BASIS_VECTORS
        weightMatrix = np.eye(NUM_FRICTION_CONE_BASIS_VECTORS)
        H_list = []
        for i in xrange(0,numContacts):
            H_list.append(np.eye(NUM_FRICTION_CONE_BASIS_VECTORS))


        residual = np.arange(1,NUM_FRICTION_CONE_BASIS_VECTORS+1)


        solnData = {}
        solnData['gurobi'] = self.solve(numContacts, residual, H_list, weightMatrix, solverType='gurobi')
        solnData['forcespro'] = self.solve(numContacts, residual, H_list, weightMatrix, solverType='forcespro')


        print "gurobi alpha vector "
        for i in xrange(0,numContacts):
            for j in xrange(0,NUM_FRICTION_CONE_BASIS_VECTORS):
                varName = "alpha_" + str(i) + "_" + str(j)
                print varName + " = ", solnData['gurobi']['alphaVals'][i,j]

        print ""

        print "forcespro alpha vector "
        for i in xrange(0,numContacts):
                for j in xrange(0,NUM_FRICTION_CONE_BASIS_VECTORS):
                    varName = "alpha_" + str(i) + "_" + str(j)
                    print varName + " = ", solnData['forcespro']['alphaVals'][i,j]

        print ""

        return solnData


def main():
    qpSolver = QPSolver([1,2])
    qpSolver.test(1)

if __name__ == "__main__":
    main()
