package artisynth.tools.batchsim;

import java.util.Date;

import argparser.DoubleHolder;
import artisynth.tools.batchsim.conditions.Condition;
import artisynth.tools.batchsim.conditions.StopConditionMonitor;
import artisynth.tools.batchsim.conditions.TimeChecker.TimeCondition;
import artisynth.tools.batchsim.conditions.TimeChecker;

/**
 * A simple concrete subclass of {@link BatchWorkerBase} that can be used to
 * perform basic and straightforward simulations of any target model. Adds one
 * stop {@link Condition} to the {@link StopConditionMonitor} that causes the
 * simulation to stop after a specified amount of time. This time defaults to
 * {@link #DEFAULT_MAX_TIME}, but it can also be set explicitly through a new
 * command-line argument. Also, adds simple logging and records property-value
 * pairs. Finally, recording of simulation task results is done by calling
 * {@link #recordBinaryWayPoints(boolean)} with the argument <b>{@code true}
 * </b>.
 * 
 * @author Francois Roewer-Despres
 */
public class SimpleTimedBatchWorker extends BatchWorkerBase {

   /**
    * The default maximum amount of {@code ArtiSynth} time (seconds) for which
    * each simulation can run.
    */
   public static final Double DEFAULT_MAX_TIME = 1.0;

   /** The max time, defaulting to {@link #DEFAULT_MAX_TIME}. */
   protected double myMaxTime;
   protected DoubleHolder myTimeHolder;

   /**
    * Creates a new {@link SimpleTimedBatchWorker} by calling
    * {@link BatchWorkerBase#BatchWorkerBase(String[])}, and by setting the max
    * time value either to the default value of {@link #DEFAULT_MAX_TIME}, or to
    * a user-specified value.
    * 
    * @param args
    * the command-line arguments
    * @throws IllegalStateException
    * if anything goes wrong in the setup that leaves the
    * {@code SimpleTimedBatchWorker} unable to {@code run()} properly.
    */
   public SimpleTimedBatchWorker (String[] args) throws IllegalStateException {
      super (args);
      myMaxTime = myTimeHolder.value;
   }

   /**
    * {@inheritDoc}
    * <p>
    * This {@link SimpleTimedBatchWorker} subclass adds a max time command line
    * option.
    */
   @Override
   protected void otherParserOptions () {
      myTimeHolder = new DoubleHolder (DEFAULT_MAX_TIME);
      myParser.addOption (
         "-t, -maxTime %f #<TIME>#set max time (seconds) that a simulation "
         + "can last to TIME; TIME defaults to " + DEFAULT_MAX_TIME,
         myTimeHolder);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This {@link SimpleTimedBatchWorker} subclass simply calls
    * {@link #simpleStartLogSession()}.
    * 
    * @see #simpleStartLogSession()
    */
   @Override
   protected void startLogSession () {
      simpleStartLogSession ();
      myLogFileWriter.flush ();
   }

   /**
    * Records 2 header rows in the log file. The first is the current date and
    * time, and the second specifies the format for subsequent log entries:
    * 
    * <pre>
    * Task Counter Number, Success of Simulation, Last Step Size Used, Number Simulations Attempted
    * </pre>
    */
   protected void simpleStartLogSession () {
      myLogFileWriter
         .println (myLogComment + "---------- " + new Date () + " ----------");
      myLogFileWriter.println (
         myLogComment + "Format: Task Counter Number" + mySeparator
         + "Did Simulation Succeed" + mySeparator + "Last Step Size Used"
         + mySeparator + "Number of Simulations Attempted");
   }

   /**
    * {@inheritDoc}
    * <p>
    * This {@link SimpleTimedBatchWorker} subclass adds a max time
    * {@link TimeCondition} to the {@link StopConditionMonitor}.
    */
   @Override
   protected void setUpStopConditionMonitor () {
      addMaxTimeCondition ();
   }

   /**
    * Adds a {@link TimeCondition#NOT_LESS_THAN_MAX} type of {@link Condition}
    * to the {@link StopConditionMonitor}.
    */
   protected void addMaxTimeCondition () {
      myStopConditionMonitor.addConditionChecker (
         new TimeChecker (TimeCondition.NOT_LESS_THAN_MAX, 0, myMaxTime));
   }

   /**
    * {@inheritDoc}
    * <p>
    * This {@link SimpleTimedBatchWorker} subclass writes property values to
    * file. If it is the first ever write, then a header containing the property
    * paths is written first. The file is only written to once per simulation
    * task, even if the first simulation attempt fails and the simulation is
    * re-attempted.
    */
   @Override
   protected void preSim () {
      if (myNumSimsAttempted == 1) {
         writePropVals (first);
         first = false;
      }
   }

   private boolean first = true;

   /**
    * {@inheritDoc}
    * <p>
    * This {@link SimpleTimedBatchWorker} subclass does nothing in this method.
    */
   @Override
   protected void postSim () {
   }

   /**
    * {@inheritDoc}
    * <p>
    * This {@link SimpleTimedBatchWorker} subclass simply calls
    * {@link #recordBinaryWayPoints(boolean) recordBinaryWayPoints(true)}.
    */
   @Override
   protected void recordSimResults () {
      recordBinaryWayPoints (true);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This {@link SimpleTimedBatchWorker} subclass simply calls
    * {@link #addSimpleLogEntry()} and then flushes the stream.
    */
   @Override
   protected void addLogEntry () {
      addSimpleLogEntry ();
   }

   /**
    * Adds a log entry to the log file for the current simulation task by
    * following the format of {@link #simpleStartLogSession()}.
    */
   protected void addSimpleLogEntry () {
      StringBuilder builder = new StringBuilder ();
      builder.append (myTaskCounter + mySeparator);
      builder.append (myCurrentTaskSuccessful + mySeparator);
      builder.append (myLastStepSizeUsed + mySeparator);
      builder.append (myNumSimsAttempted);
      myLogFileWriter.println (builder);
   }

}
