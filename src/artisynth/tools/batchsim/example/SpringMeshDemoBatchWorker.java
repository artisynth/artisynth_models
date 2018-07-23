package artisynth.tools.batchsim.example;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import artisynth.core.mechmodels.Particle;
import artisynth.demos.mech.SpringMeshDemo;
import artisynth.tools.batchsim.SimpleTimedBatchWorker;
import maspack.matrix.Point3d;

/**
 * A demo Batch Worker that works with {@link SpringMeshDemo}.
 *
 * @author Francois Roewer-Despres
 */
public class SpringMeshDemoBatchWorker extends SimpleTimedBatchWorker {

   public static final String PNT7_PATH = "models/msmod/particles/pnt7";

   /** Writer for recording simulation results. */
   protected PrintWriter myOutputFileWriter;

   /** A reference to {@link SpringMeshDemo}'s "pnt7". */
   protected Particle myPnt7;

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
    * @throws IOException
    */
   public SpringMeshDemoBatchWorker (String[] args)
   throws IllegalStateException, IOException {
      super (args);
      // InputProbe runs for 10 seconds. Can stop any time after that.
      if (myMaxTime < 10) {
         myMaxTime = 10;
      }

      // Create an output file that is unique to this Batch Worker instance.
      File file = new File (myOutputDirName, myName + "_output.txt");
      // Create a print writer to write to the file.
      myOutputFileWriter =
         new PrintWriter (new BufferedWriter (new FileWriter (file)), true);

      // Get a reference to pnt7.
      myPnt7 = (Particle)myRootModel.findComponent (PNT7_PATH);
   }

   /**
    * Writes one line in the output file with the format:
    * 
    * <pre>
    * Task Number, Pnt7 Position x, Pnt7 Position y, Pnt7 Position z
    * </pre>
    * 
    * where the position is the final position of "pnt7" at the end of the
    * simulation.
    */
   @Override
   protected void recordSimResults () {
      StringBuilder builder = new StringBuilder ();
      builder.append (myTaskCounter).append (mySeparator);
      Point3d pos = myPnt7.getPosition ();
      builder.append (pos.x).append (mySeparator);
      builder.append (pos.y).append (mySeparator);
      builder.append (pos.z).append (mySeparator);
      myOutputFileWriter.println (builder);
   }

   /**
    * {@inheritDoc}
    * <p>
    * Closes the output file writer.
    */
   @Override
   public void closeWriters () {
      super.closeWriters ();
      myOutputFileWriter.close ();
   }
}
