'''
@author: Francois Roewer-Despres
'''

from artisynth.tools.batchsim.test import SimTaskRequester
from jarray import array
import sys

args = array(sys.argv, String)
worker = SimTaskRequester(args)
worker.run()
quit()
