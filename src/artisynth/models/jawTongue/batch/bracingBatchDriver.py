'''
# @author: Connor Mayer
'''
from artisynth.models.jawTongue import JawHyoidFemMuscleTongueBatchWorker
from jarray import array
import sys

print(sys.argv)
args = array(sys.argv, String)
print(args)
worker = JawHyoidFemMuscleTongueBatchWorker(args)
worker.run()
