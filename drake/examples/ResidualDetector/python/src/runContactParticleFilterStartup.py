__author__ = 'manuelli'
sys.path.append(os.path.join(director.getDRCBaseDir(), 'software/control/residual_detector/python/src'))

import contactparticlefilterstartup
import argparse
from director import drcargs
contactparticlefilterstartup.startup(robotSystem, globals())


# parser = drcargs.getGlobalArgParser()._parser
args = drcargs.getGlobalArgParser().getArgs()

if args.CPFFilename is not None:
    print "got testing options, calling additional launch script"
    contactparticlefilterstartup.testStartup(robotSystem, args, globals())