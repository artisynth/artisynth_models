'''
@author: Connor Mayer
'''
from artisynth.models.jawTongue import BadinJawHyoidTongueContactBatchWorker
from jarray import array
import sys

args = array(sys.argv, String)
worker = BadinJawHyoidTongueContactBatchWorker(args)
worker.run()
