'''
This short Jython script drives a worker in the Batch Simulation Framework.

Ideally, BSF would be integrated into artisynth_core. Then, one could
start ArtiSynth with an option such as "-batch" or "-worker" and specify the
name of a BatchWorkerBase subclass to run. But this is not likely to happen in
the near future. In the mean time, BSF workers are driven from a Jython script
such as this one. This script shows the general format/template for any such
script:
1. Import a BatchWorkerBase subclass (here, SimpleTimedBatchWorker is used)
2. Import array from jarray
3. Import sys (for access to sys.argv)
4. Convert the command-line arguments passed to the script (but destined) for
the worker subclass from a Python list of strings (sys.argv) to a Java String[]
5. Instantiate the worker subclass (and passing it the String[] of arguments)
6. Run the worker
7. Quit ArtiSynth once there are no more simulations to run

Note that steps (2) to (4) are optional: if no command-line arguments will be
passed to the worker subclass, these lines can be ommitted, and None can be
passed to the constructor, such as in "worker = SimpleTimedBatchWorker(None)".
Alternatively, if some command-line arguments will always be passed to the
worker subclass, they can be hard-coded into the script file. For example:
  args = array(["-n", "my_worker_name"], String)

*** Please do not modify this template script. ***
*** Create a copy of this script and replace the worker subclass in the ***
*** copy, instead. ***

@author: Francois Roewer-Despres
'''
from artisynth.tools.batchsim import SimpleTimedBatchWorker # Change this.
from jarray import array
import sys

args = array(sys.argv, String)
worker = SimpleTimedBatchWorker(args) # And this (the constructor name).
worker.run()
quit()
