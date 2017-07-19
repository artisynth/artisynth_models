package artisynth.tools.batchsim.test;

import artisynth.tools.batchsim.BatchWorkerBase;

public class SimTaskRequester extends BatchWorkerBase {

   public SimTaskRequester (String[] args) throws IllegalStateException {
      super (args);
   }

   @Override
   protected void otherParserOptions () {
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
