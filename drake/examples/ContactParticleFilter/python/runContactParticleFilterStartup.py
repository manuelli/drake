__author__ = 'manuelli'

import os
import sys
DRAKE_SOURCE_DIR = os.getenv('DRAKE_SOURCE_DIR')
sys.path.append(os.path.join(DRAKE_SOURCE_DIR, 'drake/examples/ContactParticleFilter/src'))

import contactparticlefilterstartup
contactparticlefilterstartup.startup(robotSystem, globals())

