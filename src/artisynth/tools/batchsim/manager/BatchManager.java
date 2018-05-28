package artisynth.tools.batchsim.manager;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.io.StringReader;
import java.io.StringWriter;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.atomic.AtomicLong;
import com.illposed.osc.OSCListener;
import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortIn;
import com.illposed.osc.OSCPortOut;

import argparser.ArgParseException;
import argparser.ArgParser;
import argparser.BooleanHolder;
import argparser.DoubleHolder;
import argparser.IntHolder;
import argparser.LongHolder;
import argparser.StringHolder;
import artisynth.core.driver.Main;
import artisynth.core.gui.jythonconsole.ArtisynthJythonConsole;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.workspace.RootModel;
import artisynth.tools.batchsim.manager.PropertySpecification.PhonyPropValue;

import static artisynth.tools.batchsim.manager.FileParser.*;
import static artisynth.tools.batchsim.manager.PropertySpecification.SpecificationType.*;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.util.IndentingPrintWriter;
import maspack.util.ReaderTokenizer;

/**
 * A {@code BatchManager} takes an input file containing
 * {@link PropertySpecification}s, and creates simulation tasks according to the
 * specifications. The {@code BatchManager} then "manages" the resulting tasks
 * in a "bag of tasks".
 * <p>
 * 
 * @author Francois Roewer-Despres
 */
public class BatchManager {

   /**
    * The default port number on which this {@link BatchManager} is listening
    * for simulation task requests from {@link BatchWorkerBase}s.
    */
   public static final int DEFAULT_REQUEST_PORT = 34365;

   /** The default host name of this manager. */
   public static final String HOST_NAME = getDefaultHostName ();

   /**
    * Returns either the local host name, or the loop back address name.
    * 
    * @return either the local host name, or the loop back address name
    */
   protected static String getDefaultHostName () {
      String hostName = null;
      try {
         hostName = InetAddress.getLocalHost ().getCanonicalHostName ();
      }
      catch (UnknownHostException e) {
         hostName = InetAddress.getLoopbackAddress ().getCanonicalHostName ();
      }
      return hostName;
   }

   /**
    * Message channel on which this {@link BatchManager} is listening for
    * simulation task requests from {@link BatchWorkerBase}s.
    */
   protected OSCPortIn myRequestChannel;

   /** The unique number value of the current task. */
   protected AtomicLong myTaskCounter;

   /** The total number of tasks created so far. */
   protected AtomicLong myTotalNumTasks;

   /** 1 if all tasks that need to be created have been created, 0 otherwise. */
   protected AtomicLong myDoneCreatingTasks;

   /**
    * The list of {@link PropertySpecification}s built by reading the input
    * file.
    */
   protected List<PropertySpecification> myPropertySpecifications;

   /**
    * The list of skip statement {@link CombinationChecker}s built by reading
    * the input file.
    */
   protected List<CombinationChecker> mySkipStatementCheckers;

   /**
    * A queue of tasks constructed from the all the possible combinations of
    * property-value pairs in the {@link #myPropertySpecifications}. Or in the
    * case where there are no combinatorial specifications, this queue contains
    * {@link #getTotalNumberOfTasks()} samplings of each property's assigned
    * distribution vector.
    */
   protected ArrayBlockingQueue<List<String[]>> myTaskQueue;

   /** For sampling probability distributions. */
   protected DistributionSampler mySampler;

   /** For parsing command-line arguments. */
   protected ArgParser myParser;

   /** Does the input file contain at least one combinatorial specification? */
   protected boolean inputFileContainsCombinatorialSpecs = false;

   /** Does the input file contain at least one probabilistic specification? */
   protected boolean inputFileContainsProbabilisticSpecs = false;

   /**
    * An instance of a {@code Jython Console}, for instantiating a
    * {@code Jython}-implemented {@link DistributionSampler} class, and possibly
    * for interacting with the user (if the interactive level is set high
    * enough).
    */
   protected ArtisynthJythonConsole myJythonConsole;

   /* Holders for values of command-line arguments/options. */
   protected BooleanHolder myHelpHolder;
   protected StringHolder myPropsFileHolder;
   protected LongHolder myNumMonteCarloHolder;
   protected IntHolder myPortHolder;
   protected IntHolder myInteractionLevelHolder;
   protected BooleanHolder myDebugHolder;
   protected BooleanHolder myDryRunHolder;
   protected BooleanHolder myCheckPropsHolder;
   protected IntHolder myBufferCapHolder;
   protected StringHolder myDelimiterCharHolder;
   protected BooleanHolder myRiskyDelimiterOkHolder;
   protected DoubleHolder myEpsilonHolder;
   protected IntHolder mySeedHolder;

   /**
    * Creates a new {@link BatchManager}. Command-line arguments should be
    * passed to this constructor for parsing.
    * 
    * @param args
    * command-line arguments
    */
   public BatchManager (String[] args) {
      setupParser ();
      myTotalNumTasks = new AtomicLong ();
      myDoneCreatingTasks = new AtomicLong ();
      myTaskQueue = new ArrayBlockingQueue<> (myBufferCapHolder.value);
      try {
         parse ((args == null) ? new String[0] : args);
         if (checkHelp ()) { // Can exit if stand-alone application.
            return; // Return control to Jython Console if running within AS.
         }
         validateDelimiter ();
         createJythonConsole ();
         try {
            readFile ();
         }
         catch (IOException e) {
            throw new Exception (
               "BatchManager: file error: " + e.getMessage ());
         }
         if (checkDryRun ()) { // Can exit if stand-alone application.
            return; // Return control to Jython Console if running within AS.
         }
         checkRootModelProps ();
         requestChannelSetup ();
         Utils.printCond (
            myJythonConsole, myInteractionLevelHolder.value == 2, System.out,
            "BatchManager: reachable at " + HOST_NAME + ":" + myPortHolder.value
            + "\nBatchManager: Type \"commands()\" at the interactive Jython "
            + "console for a list of available commands.");
      }
      catch (Exception e) {
         String message;
         if (e.getMessage () == null) {
            ByteArrayOutputStream baos = new ByteArrayOutputStream ();
            PrintStream stream = new PrintStream (baos);
            e.printStackTrace (stream);
            String stackTrace =
               new String (baos.toByteArray (), StandardCharsets.UTF_8);
            message =
               "\nBatchManager: unexpected internal error: printing stack trace:\n"
               + stackTrace;
         }
         else {
            message = e.getMessage ();
         }
         Utils.printCond (myJythonConsole, true, System.err, message);
         if (Main.getMain () == null) { // Exit only if not running within AS
            quit (1);
         }
         else { // Return control to Jython Console if runnning within AS
            return;
         }
      }

      // Create all the tasks in a separate thread so that other things can be
      // done concurrently, like user interaction in the Jython Console.
      new Thread (new Runnable () {

         @Override
         public void run () {
            try {
               if (inputFileContainsCombinatorialSpecs) {
                  combinatorialRunHelper (0, new LinkedList<PhonyPropValue> ());
               }
               else {
                  probabilisticRunHelper ();
               }
               // Once we return from the either helper, add the "empty task"
               // to the queue.
               myTaskQueue.put (new LinkedList<String[]> ());
               myDoneCreatingTasks.set (1);
            }
            catch (InterruptedException e) {
               // TODO Auto-generated catch block
               e.printStackTrace ();
            }
         }

         /**
          * Creates all the possible combinations of property-value pairs in the
          * {@link BatchManager#myPropertySpecifications}.
          * 
          * @param i
          * the index (from 0) of the i'th property specification in
          * {@link BatchManager#myPropertySpecifications} (0 should be passed in
          * on the first call)
          * @param task
          * an empty list into which all the tasks will be built, then removed
          * and added to the queue
          * @throws InterruptedException
          * if the queue is interrupted while waiting to put()
          */
         private void combinatorialRunHelper (int i, List<PhonyPropValue> task)
            throws InterruptedException {
            PropertySpecification propSpec = myPropertySpecifications.get (i);
            boolean redefed = propSpec.redefIfNecessary (task);
            if (propSpec.getSpecificationType () == COMBINATORIAL) {
               // Create a new task for each value in turn.
               for (Object value : propSpec.getCollection ()) {
                  boolean skip = false;
                  String val = (String)value;
                  String propPath = propSpec.getPropertyPath ();
                  task.add (
                     new PhonyPropValue (propSpec.isPhony (), propPath, val));
                  for (CombinationChecker checker : mySkipStatementCheckers) {
                     skip |= checker.pushAndCheck (propPath, val, task);
                     if (skip) {
                        break;
                     }
                  }
                  if (!skip) {
                     if (i == myPropertySpecifications.size () - 1) {
                        myTaskQueue.put (removePhonies (task));
                        myTotalNumTasks.getAndIncrement ();
                     }
                     else {
                        combinatorialRunHelper (i + 1, task);
                     }
                  }
                  task.remove (i);
                  for (CombinationChecker checker : mySkipStatementCheckers) {
                     checker.popIfNecessary (propPath);
                  }
               }
            }
            else { // Assume PROBABILISTIC
               // Create a single (possibly vector) value out of all sample IDs.
               task.add (
                  new PhonyPropValue (
                     propSpec.isPhony (), propSpec.getPropertyPath (),
                     Utils.createDistributionVectorAsString (
                        mySampler, propSpec)));
               if (i == myPropertySpecifications.size () - 1) {
                  myTaskQueue.put (removePhonies (task));
                  myTotalNumTasks.getAndIncrement ();
               }
               else {
                  combinatorialRunHelper (i + 1, task);
               }
               task.remove (i);
            }
            if (redefed) {
               propSpec.undoRedef ();
            }
         }

         /**
          * Returns a "scrubbed" task that has all the phony
          * {@link PhonyPropValue}s in the given task removed (and converted to
          * an array of String).
          * 
          * @param task
          * the task containing phonies
          * @return the scrubbed task
          */
         private List<String[]> removePhonies (List<PhonyPropValue> task) {
            List<String[]> scrubbed = new LinkedList<> ();
            for (PhonyPropValue ppv : task) {
               if (!ppv.phony) {
                  scrubbed.add (new String[] { ppv.propPath, ppv.value });
               }
            }
            return scrubbed;
         }

         /**
          * Creates {@link BatchManager#getTotalNumberOfTasks()} Monte Carlo
          * simulation tasks, and places them in the queue.
          * 
          * @throws InterruptedException
          * if the queue is interrupted while waiting to put()
          */
         private void probabilisticRunHelper () throws InterruptedException {
            for (int i = 0; i < myNumMonteCarloHolder.value; i++) {
               LinkedList<String[]> task = new LinkedList<> ();
               for (PropertySpecification propSpec : myPropertySpecifications) {
                  if (!propSpec.isPhony ()) {
                     task.add (
                        new String[] { propSpec.getPropertyPath (),
                                       Utils.createDistributionVectorAsString (
                                          mySampler, propSpec) });
                  }
               }
               myTaskQueue.put (task);
               myTotalNumTasks.getAndIncrement ();
            }
         }
      }).start (); // End of new thread definition.

      if (myInteractionLevelHolder.value == 2) {
         interactWithUser ();
      }
   }

   /**
    * Adds all options to {@link #myParser}.
    */
   protected void setupParser () {
      myHelpHolder = new BooleanHolder (false);
      myPropsFileHolder = new StringHolder ("props.psl");
      myNumMonteCarloHolder = new LongHolder (1);
      myPortHolder = new IntHolder (DEFAULT_REQUEST_PORT);
      myInteractionLevelHolder = new IntHolder (2);
      myDebugHolder = new BooleanHolder (false);
      myDryRunHolder = new BooleanHolder (false);
      myCheckPropsHolder = new BooleanHolder (false);
      myBufferCapHolder = new IntHolder (1 << 20);
      myDelimiterCharHolder = new StringHolder ("%");
      myRiskyDelimiterOkHolder = new BooleanHolder (false);
      myEpsilonHolder = new DoubleHolder (0.0001);
      mySeedHolder = new IntHolder (-1);

      myParser =
         new ArgParser (
            "java " + BatchManager.class.getName () + "[options]", false);
      myParser.addOption (
         "-h, -help %v #print this help message and exit", myHelpHolder);
      myParser.addOption (
         "-f, -file %s #<FILENAME>#read the property specifications from file "
         + "FILENAME; if FILENAME is `-', read from standard input (which "
         + "forces an interaction level of 0; see -i option); FILENAME "
         + "defaults to \"props.psl\"", myPropsFileHolder);
      myParser.addOption (
         "-m, -monteCarlo %d {[0, " + Long.MAX_VALUE + "]}"
         + "#<N>#if, and only if, the input file (see -f option) contains "
         + "*only* probabilistic value sets, perform N Monte Carlo"
         + "simulations instead of performing a \"combinatorial\" number of "
         + "simulations; N defaults to 1", myNumMonteCarloHolder);
      myParser.addOption (
         "-p, -port %d {[1100," + (Short.MAX_VALUE * 2 + 1)
         + "]}#<PORT>#bind the port to which the BatchManager socket is "
         + "listening for requests from a BatchWorker to PORT; PORT defaults "
         + "to " + DEFAULT_REQUEST_PORT, myPortHolder);
      myParser.addOption (
         "-i, -interactionLevel %d {[0,2]}#<LEVEL>#set the level of "
         + "interaction with the user to LEVEL; 0 means output nothing except "
         + "error messages; 1 means output error and warning messages; 2 means "
         + "fully interactive (e.g. interaction through Jython Console); LEVEL "
         + "defaults to 2 (unless ``-f -'' is given; see -f option)",
         myInteractionLevelHolder);
      myParser.addOption (
         "-v, -debug %v #print debugging (i.e. more *verbose*) messages",
         myDebugHolder);
      myParser.addOption (
         "-r, -dryRun %v #read the input file and exit (ensures the input file "
         + "can be parsed without errors) (see -f option)", myDryRunHolder);
      myParser.addOption (
         "-b, -bufferCap %d {[1," + Integer.MAX_VALUE
         + "]}#<CAP>#set the maximum "
         + "capacity of the buffer used to store created but unsent simulation "
         + "tasks to CAP; CAP defaults to " + (1 << 20) + " but can be set to "
         + "a larger value if more than " + (1 << 20) + " simulations tasks "
         + "will be created and knowing the exact number is important, but it "
         + "can also be set to a smaller value if so many simulation tasks "
         + "are being created that the JVM runs out of memory and crashes",
         myBufferCapHolder);
      myParser.addOption (
         "-c, -checkProps %v #if this BatchManager is running within an "
         + "instance of ArtiSynth, verify that the *currently-loaded* root "
         + "model's class and the classes of all its sub-components do in fact "
         + "have all the properties listed in the input file, and that, for "
         + "each such property, all the listed values are assignable to that "
         + "property (see -f option)", myCheckPropsHolder);
      myParser.addOption (
         "-d, -delimiter %s #<CHAR>#set the value delimiter character"
         + " of the input file to CHAR (see -f option); CHAR defaults to '%'",
         myDelimiterCharHolder);
      myParser.addOption (
         "-o, -riskyDelimOk %v #allow a delimiter character that is risky to "
         + "use because it already has syntactic significance (see -d option)",
         myRiskyDelimiterOkHolder);
      myParser.addOption (
         "-e, -epsilon %f #<EPS>#set the epsilon for comparing two doubles "
         + "for equality to EPS; EPS defaults to 0.0001", myEpsilonHolder);
      myParser.addOption (
         "-s, -seed %d {[-1, " + Integer.MAX_VALUE + "]}"
         + "#<SEED>#set the seed for the random number generator to SEED, or "
         + "use no seed at all if SEED is -1; SEED defaults to -1",
         mySeedHolder);
   }

   /**
    * Uses the {@link #myParser} to parse the given arguments.
    * 
    * @param args
    * the given command-line arguments
    * @throws ArgParseException
    * if the parser detects an error while parsing
    */
   protected void parse (String[] args) throws ArgParseException {
      int idx = 0;
      while (idx < args.length) {
         try {
            idx = myParser.matchArg (args, idx);
            String unmatched;
            if ((unmatched = myParser.getUnmatchedArgument ()) != null) {
               throw new ArgParseException (
                  "BatchManager: unrecognized argument \"" + unmatched + "\".");
            }
         }
         catch (ArgParseException e) {
            throw new ArgParseException (
               "BatchManager: error parsing options: " + e.getMessage ()
               + "\nUse -h or -help for help.");
         }
      }
   }

   /**
    * Checks if the -help flag was set. If so, exit with status 0 if running as
    * a stand-alone application. If not running as a stand-alone application, or
    * if the flag is not set, returns the value of the flag.
    * 
    * @return whether the -help flag was set or not
    */
   protected boolean checkHelp () {
      if (myHelpHolder.value) {
         Utils.printCond (
            myJythonConsole, true, System.out, myParser.getHelpMessage ());
         if (Main.getMain () == null) {
            quit (); // Doesn't return.
         }
      }
      return myHelpHolder.value;
   }

   /**
    * Checks if the -r flag was set. If so, exit with status 0 if running as a
    * stand-alone application. If not running as a stand-alone application, or
    * if the flag is not set, returns the value of the flag.
    * 
    * @return whether the -r flag was set or not
    */
   protected boolean checkDryRun () {
      if (myDryRunHolder.value) {
         if (Main.getMain () == null) {
            quit (); // Doesn't return.
         }
      }
      return myDryRunHolder.value;
   }

   /**
    * Checks that the supplied value delimiter string is valid (i.e. that it is
    * exactly 1 character, and that it is not blacklisted).
    * 
    * @throws IllegalArgumentException
    */
   protected void validateDelimiter () throws IllegalArgumentException {
      if (myDelimiterCharHolder.value.length () != 1) {
         throw new IllegalArgumentException (
            "BatchManager: delimiter must be exactly one character long, but ``"
            + myDelimiterCharHolder.value + "'' was given.");
      }
      char delim = myDelimiterCharHolder.value.charAt (0);
      BooleanHolder isRisky = new BooleanHolder ();
      if (isBlacklisted (delim, isRisky)) {
         if (isRisky.value) {
            throw new IllegalArgumentException (
               "BatchManager: `" + delim + "' is a risky delimiter character; "
               + "rerun with the -o option to use anyway.");
         }
         else {
            throw new IllegalArgumentException (
               "BatchManager: `" + delim
               + "' is not an allowed delimiter character.");
         }
      }
   }

   /**
    * Returns true if the given character is not a valid delimiter character.
    * 
    * @param c
    * the character to test
    * @param isRisky
    * {@code isRisky.value} will be set to {@code true} if the character is
    * risky to use as a delimiter character because it already has syntactic
    * significance, and false otherwise
    * @return true if the given character is not a valid delimiter character
    */
   protected boolean isBlacklisted (char c, BooleanHolder isRisky) {
      boolean toRet;
      switch (c) {
         case '{':
         case '}':
         case '[':
         case ']':
         case '#':
         case '(':
         case ')':
            toRet = true;
            isRisky.value = false;
            break;
         case '=':
         case '~':
         case '"':
         case '\'':
         case '@':
         case '$':
            toRet = !myRiskyDelimiterOkHolder.value;
            isRisky.value = true;
            break;
         default:
            toRet = Character.isWhitespace (c) || Character.isDigit (c);
            isRisky.value = false;
      }
      return toRet;
   }

   /**
    * Reads (and parses) the input file.
    * 
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected void readFile () throws IOException {
      Settings settings = new Settings ();
      settings.manager = this;
      settings.console = myJythonConsole;
      settings.comment = '#';
      settings.delim = myDelimiterCharHolder.value.charAt (0);
      settings.interactionLevel = myInteractionLevelHolder.value;
      settings.debug = myDebugHolder.value;
      settings.propsFileName = myPropsFileHolder.value;
      settings.epsilon = myEpsilonHolder.value;
      settings.seed = mySeedHolder.value;

      ParseResults results = new FileParser (settings).parse ();
      myPropertySpecifications = results.propertySpecifications;
      mySkipStatementCheckers = results.skipStatementCheckers;
      myInteractionLevelHolder.value = results.resultingInteractionLevel;
      inputFileContainsCombinatorialSpecs = results.containsCombinatorialSpecs;
      inputFileContainsProbabilisticSpecs = results.containsProbabilisticSpecs;
      mySampler = results.sampler;

      if (myPropertySpecifications.size () == 0) {
         throw new IOException (
            "input file contains no property specifications. Aborting.");
      }

      if (myInteractionLevelHolder.value == 2 || myDebugHolder.value) {
         StringWriter str = new StringWriter ();
         IndentingPrintWriter writer = new IndentingPrintWriter (str);
         writer.print ("BatchManager: summary of input file \"");
         writer.println (myPropsFileHolder.value + "\":");
         writer.println ("Property Specifications:");
         writer.addIndentation (2);
         for (PropertySpecification propSpec : myPropertySpecifications) {
            propSpec.print (writer);
         }
         writer.removeIndentation (2);
         writer.println ("Skip statements:");
         writer.addIndentation (2);
         if (mySkipStatementCheckers.isEmpty ()) {
            writer.println ("<none>");
         }
         for (CombinationChecker checker : mySkipStatementCheckers) {
            writer.println ("skip");
            writer.addIndentation (2);
            checker.print (writer);
            writer.removeIndentation (2);
         }
         writer.removeIndentation (2);
         Utils.printCond (myJythonConsole, true, System.out, str.toString ());
      }
   }

   /**
    * If running within an instance of {@code ArtiSynth}, and if the appropriate
    * command-line flag was set, checks each {@link PropertySpecification} in
    * {@link #myPropertySpecifications} to ensure that the target model or one
    * of its {@link ModelComponent}s has that specification's {@link Property},
    * and that each value in the accompanying value set (for combinatorial
    * property specifications) or that a randomly-sampled distribution vector
    * (for probabilistic property specifications) is a valid value to assign to
    * that {@code Property}.
    * 
    * @throws IllegalStateException
    * if the flag was set, but this {@link BatchManager} is not running within
    * an instance of ArtiSynth
    */
   protected void checkRootModelProps () throws IllegalStateException {
      if (myCheckPropsHolder.value == false) {
         return;
      }

      if (Main.getMain () == null) {
         throw new IllegalStateException (
            "BatchManager: state error: -c flag: not currently running within ArtiSynth");
      }

      RootModel root = Main.getMain ().getRootModel ();
      for (PropertySpecification propSpec : myPropertySpecifications) {
         Property prop = root.getProperty (propSpec.getPropertyPath ());
         if (prop == null) {
            throw new IllegalStateException (
               "BatchManager: model error: model class \""
               + root.getClass ().getName () + "\" does have property path \""
               + propSpec.getPropertyPath () + "\"");
         }
         PropertyInfo info = prop.getInfo ();
         if (propSpec.getSpecificationType () == COMBINATORIAL) {
            // Try each value in turn to make sure they are all legal.
            for (Object value : propSpec.getCollection ()) {
               if (value == null) {
                  if (!info.getNullValueOK ()) {
                     throw new IllegalStateException (
                        "BatchManager: model error: property path \""
                        + propSpec.getPropertyPath () + "\" of model class \""
                        + root.getClass ().getName ()
                        + "\" does not allow null values");
                  }
               }

               ReaderTokenizer rtok =
                  new ReaderTokenizer (new StringReader ((String)value));
               try {
                  info.scanValue (rtok);
               }
               catch (IOException e) {
                  throw new IllegalStateException (
                     "BatchManager: state error: " + e.getMessage ());
               }
            }
         }
         else { // Assume PROBABILISTIC
            // Create a single (possible vector) value on a 'best effort' basis.
            // That is, we cannot guarantee that the specified probability
            // distribution will only produce values in range for the property
            // (if the property has a range). The best we can do is sample the
            // distribution and try that one value. If that value passes the
            // test, we'll just have to hope that other values will too. And the
            // BatchSim documentation mentions this clearly, anyhow, so the user
            // will hopefully be aware of this.
            ReaderTokenizer rtok =
               new ReaderTokenizer (
                  new StringReader (
                     Utils.createDistributionVectorAsString (
                        mySampler, propSpec)));
            try {
               info.scanValue (rtok);
            }
            catch (IOException e) {
               throw new IllegalStateException (
                  "BatchManager: state error: " + e.getMessage ());
            }
         }
      }
      Utils.printCond (
         myJythonConsole, myInteractionLevelHolder.value == 2, System.out,
         "BatchManager: property check OK");
   }

   /**
    * Sets up the {@link #myRequestChannel}, and the {@link OSCListener}
    * associated with it. This listener, upon receiving a request, takes out a
    * task from the {@link #myTaskQueue} and replies back to the
    * {@link BatchWorkerBase} client. The listener is running in a separate
    * thread, so this method returns fairly quickly.
    * 
    * @throws IllegalStateException
    * if a networking error occurs
    */
   protected void requestChannelSetup () throws IllegalStateException {
      myTaskCounter = new AtomicLong ();
      try {
         myRequestChannel = new OSCPortIn (myPortHolder.value);
      }
      catch (SocketException e) {
         throw new IllegalStateException (
            "BatchManager: socket error: unable to receive requests on port "
            + myPortHolder.value);
      }

      OSCListener listener = new OSCListener () {

         HashMap<String,Object> myWorkers = new HashMap<> ();

         @Override
         public void acceptMessage (Date date, OSCMessage msgIn) {
            String args = Arrays.deepToString (msgIn.getArguments ());
            Utils.printCond (
               myJythonConsole, myDebugHolder.value, System.out,
               msgIn.getAddress () + ": " + args);

            InetAddress host = null;
            int port = -1;
            OSCPortOut sender = null;
            try {
               host = InetAddress.getByName ((String)msgIn.getArguments ()[1]);
               port = (int)msgIn.getArguments ()[2];
               sender = new OSCPortOut (host, port);
            }
            catch (SocketException | UnknownHostException e) {
               Utils.printCond (
                  myJythonConsole, myInteractionLevelHolder.value != 0,
                  System.err, "Warning! Unable to connect to \"" + host + ":"
                  + port + "\". Ignoring " + msgIn.getAddress () + ".");
               return;
            }

            boolean ignore = false;

            // It is unclear whether or not the OSC library is event- or
            // thread-based. If it's the former, then this synchronization is
            // unnecessary, since there is only OSCListener for all incoming
            // requests to the request channel. But if its the latter, then
            // we expect one thread per request, which means this
            // synchronization is justified, because of the use of peek() to
            // get the next task from the queue. We don't want multiple threads
            // peek()'ing the same value.
            synchronized (BatchManager.this) {
               List<String[]> task = null;
               try {
                  if (msgIn.getAddress ().equals ("ping")) {
                     ignore = true;
                     sender.send (new OSCMessage ("ping"));
                     myWorkers.put (args, null);
                  }
                  else {
                     // Peek, not take, in case we have to "ignore".
                     while ((task = myTaskQueue.peek ()) == null) {
                        // Do nothing.
                     }

                     if (task.isEmpty ()) {
                        OSCMessage msgOut = new OSCMessage ("DONE");
                        Utils.printCond (
                           myJythonConsole, myDebugHolder.value, System.out,
                           msgOut.getAddress () + ": "
                           + Arrays.deepToString (msgOut.getArguments ()));
                        sender.send (msgOut);
                        myWorkers.remove (args);
                        if (myWorkers.isEmpty ()) {
                           Utils.printCond (
                              myJythonConsole,
                              myInteractionLevelHolder.value == 2
                              || myDebugHolder.value, System.out,
                              "Quitting because all workers received DONE.");
                           quit ();
                        }
                     }
                     else {
                        long taskNo = myTaskCounter.getAndIncrement ();
                        for (int i = 0; i < task.size (); i++) {
                           String[] propValPair = task.get (i);
                           OSCMessage msgOut = new OSCMessage ("reply");

                           // Format of a simulation task message:
                           // 0 Task Number
                           // 1 Number of property-value pairs in the task
                           // 2 Pos. of the current prop-value pair in the task
                           // 3 Property path
                           // 4 Current value to which to set the property
                           // We have one such message per prop-value pair in
                           // the task. The reason we have to break up the task
                           // into many small messages is that a Java varargs
                           // cannot be very long, and some large tasks may
                           // exceed this inherent limit, causing the algorithm
                           // to break.
                           msgOut.addArgument (Long.toString (taskNo));
                           msgOut.addArgument (Integer.toString (task.size ()));
                           msgOut.addArgument (Integer.toString (i));

                           // Add the property path.
                           msgOut.addArgument (
                              (propValPair[0] == null) ? "null"
                                 : propValPair[0]);

                           // Add the current value of the property.
                           msgOut.addArgument (
                              (propValPair[1] == null) ? "null"
                                 : propValPair[1]);
                           Utils.printCond (
                              myJythonConsole, myDebugHolder.value, System.out,
                              msgOut.getAddress () + ": "
                              + Arrays.deepToString (msgOut.getArguments ()));
                           sender.send (msgOut);
                        }
                     }
                  }
               }
               catch (IOException e) {
                  Utils.printCond (
                     myJythonConsole, myInteractionLevelHolder.value != 0,
                     System.err,
                     "BatchManager: warning: Unable to reply to \"" + host + ":"
                     + port + "\". Ignoring " + msgIn.getAddress () + ".");
                  ignore = true;
               }
               finally {
                  sender.close ();
                  if (ignore || task.isEmpty ()) { // Short-circuit.
                     // Do nothing.
                  }
                  else {
                     try {
                        // Actually do the take()'ing here.
                        List<String[]> tookTask = myTaskQueue.take ();
                        if (task != tookTask) {
                           throw new AssertionError (
                              "BatchManager: internal synchronization error. Aborting.");
                        }
                     }
                     catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace ();
                     } // end catch
                  } // end else
               } // end finally
            } // synchronized
         } // end method
      }; // end class

      myRequestChannel.addListener ("ping", listener);
      myRequestChannel.addListener ("request", listener);
      myRequestChannel.startListening (); // Implicitly in a new thread.
   }

   /**
    * Either creates a new JythonConsole, or uses the one Main.getMain() already
    * created.
    */
   protected void createJythonConsole () {
      if (Main.getMain () == null
      || Main.getMain ().getJythonConsole () == null) {
         myJythonConsole = ArtisynthJythonConsole.createTerminalConsole ();
      }
      else {
         myJythonConsole = Main.getMain ().getJythonConsole ();
      }
   }

   /**
    * Starts an interactive session with the user in the current thread.
    */
   protected void interactWithUser () {
      myJythonConsole.getConsole ().set ("batchManager", this);
      String script =
         "_interpreter_.set('commands', batchManager.commands)\n"
         + "_interpreter_.set('progress', batchManager.progress)\n"
         + "_interpreter_.set('quit', batchManager.quit)\n";
      InputStream input =
         new ByteArrayInputStream (script.getBytes (StandardCharsets.UTF_8));
      myJythonConsole.execfile (input, "Commands setting");
      if (Main.getMain () == null) { // if myJythonConsole != Main's console
         myJythonConsole.interact ();
      }
   }

   /**
    * Prints a string representing the overall progress of BatchSim in running
    * the simulations the user requested to standard output.
    */
   public void progress () {
      long taskNo = myTaskCounter.get ();
      long total = myTotalNumTasks.get ();
      boolean doneCreating = myDoneCreatingTasks.get () == 1;
      String msg;
      if (doneCreating) {
         msg = "Completed %d of %d (%.2f %%).";
      }
      else {
         msg =
            "Completed %d of at least %d total simulations (at most "
            + "%.2f %% complete).";
      }
      msg = String.format (msg, taskNo, total, 100 * ((double)taskNo) / total);
      Utils.printCond (myJythonConsole, true, System.out, msg);
   }

   /**
    * Quits this {@link BatchManager} and {@code ArtiSynth} (if applicable).
    */
   public void quit () {
      quit (0);
   }

   /**
    * Quits this {@link BatchManager} and {@code ArtiSynth} (if applicable),
    * with an exit status of {@code status}. Note that {@code status} is ignored
    * by {@code ArtiSynth} (which always exits with status 0).
    * 
    * @param status
    * the exit status with which to exit, if not running within
    * {@code ArtiSynth}
    */
   public void quit (int status) {
      if (Main.getMain () == null) {
         System.exit (status);
      }
      else {
         Main.getMain ().quit ();
      }
   }

   /**
    * Prints a formatted string listing all the available commands that a user
    * can call when running interactively in the {@code Jython Console} to
    * standard output.
    */
   public void commands () {
      String msg =
         "  commands() -- output this list of commands\n"
         + "  progress() -- output progress information\n"
         + "  quit()     -- exit with status 0\n"
         + "  quit(st)   -- exit with status `st'";
      Utils.printCond (myJythonConsole, true, System.out, msg);
   }

   /**
    * The entry point when starting a {@link BatchManager} from the
    * command-line.
    * 
    * @param args
    * the command-line arguments to pass to the {@link BatchManager}
    */
   public static void main (String[] args) {
      new BatchManager (args);
   }

}
