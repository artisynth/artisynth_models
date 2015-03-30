package artisynth.models.tongue3d;

import java.io.IOException;

import maspack.interpolation.Interpolation.Order;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.workspace.DriverInterface;

public class StableFemMuscleTongueDemo extends FemMuscleTongueDemo {

   public StableFemMuscleTongueDemo () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      

      /*
       * change integrator, material, damping parameters for better stability 
       */
      
      mech.setIntegrator (Integrator.ConstrainedBackwardEuler);
      mech.setMaxStepSize (0.01);

      tongue.setMaterial (new MooneyRivlinMaterial ());
      HexTongueDemo.setTongueProperties (tongue);
      MooneyRivlinMaterial femMat = (MooneyRivlinMaterial)tongue.getMaterial();
      
      // use high bulk modulus instead of incomp constraint becuase its more stable
      tongue.setIncompressible(IncompMethod.OFF);
      femMat.setBulkModulus(100 * femMat.getC10());

      BlemkerMuscle muscleMat = new BlemkerMuscle();
      muscleMat.setOptLambda(1.1);
      muscleMat.setMaxLambda(1.5);
      muscleMat.setMaxStress(1e5); 
      tongue.setMuscleMaterial(muscleMat);

      // reduce stiffness damping to improve stability, increase particle damping accordingly
//      tongue.setParticleDamping(100);
//      tongue.setStiffnessDamping(0.001); 
	
      // from eigenvalue analysis at rest
      tongue.setParticleDamping(67.0158);
      tongue.setStiffnessDamping(1.3154e-04);
      
   }
   
   public void attach (DriverInterface driver) {
      super.attach(driver);
   }
 
}
