package artisynth.models.tongue3d;

import java.io.IOException;

import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemMuscleModel;

public class TetTongueDemo extends HexTongueDemo {

   public TetTongueDemo () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
   }
   
   public static void createTetTongue(FemMuscleModel tongue, boolean linearMaterial, boolean useIcpMuscles) {
      readFromAnsysReader (tongue, "tongue");
      setupTongue(tongue, linearMaterial, useIcpMuscles, false /*scaleMuscleForceByVolume*/);
   }
   
   public static FemMuscleModel createTetTongue(boolean linearMaterial, boolean useIcpMuscles) {
      FemMuscleModel tongue = new FemMuscleModel ("tongue");
      createTetTongue(tongue, linearMaterial, useIcpMuscles);
      return tongue;
   }
   
   
   public static FemMuscleModel readFromAnsysReader (FemMuscleModel fem, String name) {
       try {
         AnsysReader.read (
            fem, geometrypath+name+".node", 
            geometrypath+name+".elem", 
            1, null, /*options=*/AnsysReader.TETRAHEDRALIZE_HEXES);
         
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      return fem;
   }

}
