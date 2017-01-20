__author__ = 'manuelli'

import subprocess
import os
import time
import signal
import psutil
# from director import lcmUtils
# from lcmUtils import LCMLoggerManager

class AutoTest:

    def __init__(self):
        self.popenDict = dict()

    def launchDirector(self, contactFileName='singleContactForces', contactForceName='single_3'):
        print "starting director . . . "
        command = "director -v5 --startup $DRC_BASE/software/control/residual_detector/python/src/runContactParticleFilterStartup.py"
        command += " --CPFFilename " + contactFileName + ".out" + " --CPFForceName " + contactForceName
        print "launchDirector command ", command
        p = subprocess.Popen(command, shell=True)
        self.popenDict['director'] = p

    def launchCPF(self):
        print "starting CPF . . . ."
        command = 'directorPython runContactFilter.py -v5'
        cwd = os.getenv('DRC_BASE') + '/software/control/residual_detector/python/src/'
        command = 'directorPython ' + cwd + 'runContactFilter.py -v5'
        p = subprocess.Popen(command, shell=True)
        self.popenDict['cpf'] = p

    def killProcess(self, processName):
        killRecursive(self.popenDict[processName].pid)

    def launchLogger(self, filename='testAutoCPF'):
        print "starting logger . . . "
        command = 'lcm-logger /media/manuelli/DATA/logs/CASE/' + filename + ' -f'
        p = subprocess.Popen(command, shell=True)
        self.popenDict['logger'] = p

    def launchLogPlayer(self, filename='lcmlog__2016-07-18__14-57-34-995211__multiple_poses', speed=1.0):
        print "starting log player . . . "
        command = 'lcm-logplayer /media/manuelli/DATA/logs/CASE/' + filename + " -e EST_ROBOT_STATE"
        command += " -s " + str(speed)
        p = subprocess.Popen(command, shell=True)
        self.popenDict['logPlayer'] = p

    def testLogPlayer(self):
        self.launchLogPlayer('test', speed=10.0)
        p = self.popenDict['logPlayer']
        while p.poll() is None:
            print "logplayer still running . . . "
            time.sleep(1.0)

        print "logplayer finished, killing all processes"
        killAllProcesses()



    def test(self):
        self.launchLogPlayer()
        self.launchLogger(filename='test')
        self.launchDirector()
        self.launchCPF()

    def runSingleTest(self, contactFileName='singleContactForces', contactForceName='single_1', logFilenameExtension='',
                      runLogger=False, timeout=50):
        killAllProcesses()

        self.launchLogPlayer(speed=2.0)
        self.launchCPF()
        self.launchDirector(contactFileName=contactFileName, contactForceName=contactForceName)

        if runLogger:
            logFilename = contactFileName + "_" + contactForceName + '_' + logFilenameExtension
            self.launchLogger(filename=logFilename)


        # only do first 30 seconds or so
        startTime = time.time()
        # timeout = 50
        p = self.popenDict['logPlayer']
        while p.poll() is None:
            print "logplayer still running . . . "
            print "contact force name ", contactForceName
            print "contact file name ", contactFileName
            time.sleep(2.0)


            if (timeout is not None) and (time.time() - startTime > timeout):
                print "reached timeout, breaking"
                break


        killAllProcesses()


def killRecursive(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.get_children(recursive=True):
        if psutil.pid_exists(proc.id):
            proc.kill()
    process.kill()

def killAllProcesses():

        # script that kills a bunch of stuff
        subprocess.call(['./killAll.sh'])
        # for key, val in self.popenDict.iteritems():
        #     killRecursive(val.pid)


def runSingleContactTests(numContacts = 2):
    at = AutoTest()
    numEntries = []
    contactFileName = []
    contactForceNamePrefix = []
    if numContacts == 1:
        numEntries = 7
        contactFileName = 'singleContactForces'
        contactForceNamePrefix = 'single_'
    if numContacts == 2:
        numEntries = 4
        contactFileName = 'doubleContactForces'
        contactForceNamePrefix = 'double_'

    if numContacts == 3:
        numEntries =  3
        contactFileName = 'tripleContactForces'
        contactForceNamePrefix = 'triple_'
    #
    # singleNumEntries = 7
    # doubleNumEntries = 4
    # tripleNumEntries = 3
    #
    # numEntries = tripleNumEntries
    for idx in xrange(1,numEntries + 1):
        contactForceName = contactForceNamePrefix + str(idx)
        at.runSingleTest(contactFileName=contactFileName, contactForceName=contactForceName, logFilenameExtension='noise_02',
                         runLogger=True, timeout=60)


def main():
    # try:
    #     at = AutoTest()
    #     at.runSingleTest()
    # except KeyboardInterrupt:
    #     print "you pressed ctrl-C, attempting to kill all processes"
    #     killRecursive(os.getpid())

    at = AutoTest()
    at.runSingleTest(contactFileName='doubleContactForces', contactForceName='double_1', logFilenameExtension='no_noise')

if __name__ == "__main__":

    try:
        runSingleContactTests(numContacts=1)
        runSingleContactTests(numContacts=2)
        runSingleContactTests(numContacts=3)

        # main()
    except KeyboardInterrupt:
        print "you pressed ctrl-C, attempting to kill all processes"
        killAllProcesses()
        killRecursive(os.getpid())
    #
    # while True:
    #     print "in loop "
    #     time.sleep(1.0)
    #     print "director pid ", at.popenDict['director'].pid
    #     killRecursive(at.popenDict['director'].pid)

    # at.test()
    # time.sleep(5)
    # print "tring to kill director"
    # at.popenDict['director'].kill()
    # # os.kill(at.popenDict['director'].pid, signal.SIGTERM)
