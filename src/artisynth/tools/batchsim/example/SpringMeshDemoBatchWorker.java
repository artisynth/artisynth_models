package artisynth.tools.batchsim.example;

import java.io.File;
import java.io.IOException;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.workspace.RootModel;
import artisynth.demos.mech.SpringMeshDemo;
import artisynth.tools.batchsim.SimpleTimedBatchWorker;

/**
 * A demo Batch Worker that works with {@link SpringMeshDemo}, or any other
 * {@link RootModel} with a {@link NumericOutputProbe}.
 *
 * @author Francois Roewer-Despres
 */
public class SpringMeshDemoBatchWorker extends SimpleTimedBatchWorker {

   /** Output probe directory. */
   protected File myOutputDir;

   // References to the output probe of SpringMeshDemo.
   protected NumericOutputProbe myProbe;

   /**
    * Creates a new {@link SpringMeshDemoBatchWorker} with parameters set
    * according to {@code args}.
    * <p>
    * Ensure that the {@link SimpleTimedBatchWorker#myMaxTime} is at least 10
    * seconds, and creates an output directory for the probe data if it doesn't
    * exist.
    *
    * @param args
    * the command-line arguments
    * @throws IllegalStateException
    * if anything goes wrong in the setup
    */
   public SpringMeshDemoBatchWorker (String[] args)
   throws IllegalStateException {
      super (args);
      // InputProbe runs for 10 seconds. Can stop any time after that.
      if (myMaxTime < 10) {
         myMaxTime = 10;
      }
      
      // Create an output sub-directory for the output probe files.
      myOutputDir = new File (myOutputDirName, "output-probe-data");
      if (!myOutputDir.exists ()) {
         myOutputDir.mkdirs ();
      }
      
      // Get a reference to the output probe.
      myProbe = (NumericOutputProbe)myRootModel.getOutputProbes ().get (0);
   }

   @Override
   protected void recordSimResults () {
      File file = new File (myOutputDir, myTaskCounter + ".txt");
      myProbe.setAttachedFileName (file.getAbsolutePath ());
      try {
         myProbe.save ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
}
