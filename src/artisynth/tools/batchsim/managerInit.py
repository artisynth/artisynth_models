'''
This short Jython script can optionally be used to initialize a manager in the
Batch Simulation Framework.

@author: Francois Roewer-Despres
'''
from artisynth.tools.batchsim.manager import BatchManager
from jarray import array
import sys

if len(sys.argv) == 0: # No arguments at all
    args = None
elif len(sys.argv) == 1 and sys.argv[0] == "": # One argument that is empty string
    args = None
else: # An actual arguments' list
    args = array(sys.argv, String)

worker = BatchManager(args)
