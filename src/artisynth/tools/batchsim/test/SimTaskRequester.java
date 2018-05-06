package artisynth.tools.batchsim.test;

import argparser.BooleanHolder;
import artisynth.tools.batchsim.BatchWorkerBase;

public class SimTaskRequester extends BatchWorkerBase {

   protected BooleanHolder myVerboseHolder;

   public SimTaskRequester (String[] args) throws IllegalStateException {
      super (args);
   }

   @Override
   protected void otherParserOptions () {
      myVerboseHolder = new BooleanHolder (false);
      myParser.addOption (
         "-v, -verbose %v #print each simulation task to the console",
         myVerboseHolder);
   }

   @Override
   protected void startLogSession () {
   }

   @Override
   protected void setUpStopConditionMonitor () {
   }

   @Override
   protected void preSim () {
   }

   @Override
   protected void postSim () {
   }

   @Override
   protected void recordSimResults () {
   }

   @Override
   protected void addLogEntry () {
   }

   @Override
   public void run () throws IllegalStateException {
      try {
         while (true) {
            requestSimTask ();
            if (myVerboseHolder.value) {
               for (String[] propValPair : myCurrentTask) {
                  System.out.println (propValPair[0] + " " + propValPair[1]);
               }
            }
            if (myCurrentTask.size () == 0) {
               break;
            }
         }
      }
      catch (Exception e) {
         e.printStackTrace ();
         throw new IllegalStateException (e.getMessage ());
      }
      finally {
         closeWriters ();
      }
   }

}
