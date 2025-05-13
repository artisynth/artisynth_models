#!/bin/bash -v

# This script starts one BatchManager and a certain number of BatchWorkers to
# run simulations with BatchSim. It also records all the console output of
# these programs, for later reference.
# Options to the BatchManager, BatchWorker, or artisynth can of course be
# changed to reflect the user's needs; this file should be seen as a template/
# example of how to start up BatchSim.
# @author: Francois Roewer-Despres


java artisynth.tools.batchsim.manager.BatchManager -v -i 1 > manager_log.txt 2>&1 &
#                                           ^^^^^^
#                                           Options to BatchManager.

for i in {0..10} # <- Total number of workers can be set here.
do
  LOG=worker"$i"_console.txt
  echo "----------" $DATE "----------" > $LOG
  artisynth -noGui -model artisynth.models.jawTongue.JawHyoidFemMuscleTongue -script bracingBatchDriver.py > $LOG 2>&1 &
# ^^^
# Options to artisynth. To use a BatchWorker, a model should be provided (with
# the -model option), and a BatchDriver Jython script should be provided (with
# the -script option). Options to the BatchWorker should be passed as options
# to the script, not the model.
done

