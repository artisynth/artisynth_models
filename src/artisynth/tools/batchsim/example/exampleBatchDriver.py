'''
@author: Francois Roewer-Despres
'''

from artisynth.tools.batchsim.example import SpringMeshDemoBatchWorker
from jarray import array
import sys

args = array(sys.argv, String)
worker = SpringMeshDemoBatchWorker(args)
worker.run()
quit()
