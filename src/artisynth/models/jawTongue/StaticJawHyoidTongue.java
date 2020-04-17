package artisynth.models.jawTongue;

import java.io.IOException;

import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.tongue3d.FemMuscleTongueDemo;

public class StaticJawHyoidTongue extends BadinJawHyoidTongue {

   public StaticJawHyoidTongue() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      // fix skeletal bodies
      myJawModel.rigidBodies ().get ("jaw").setDynamic (false);
      myJawModel.rigidBodies ().get ("hyoid").setDynamic (false);
      
      // remove jaw muscles and springs
      myJawModel.clearAxialSprings ();
      myJawModel.clearFrameSprings ();
      myJawModel.clearMultiPointSprings ();
      myJawModel.getMuscleExciters ().clear ();
      
      this.setAdaptiveStepping (false);

      FemMuscleTongueDemo.addExcitersFromFiles (tongue, ArtisynthPath.getSrcRelativePath (FemMuscleTongueDemo.class, "exciters/"));
   }
   
   
   public void addMexProbe(MuscleExciter mex, double duration, double ex) {
      NumericInputProbe ip = new NumericInputProbe(mex, "excitation", 0, duration);
      ip.addData(0, new VectorNd(1));
      VectorNd exData = new VectorNd(1);
      exData.set(0, ex);
      ip.addData(duration, exData);
      if (mex.getName() != null) 
         ip.setName(mex.getName());
      addInputProbe(ip);
   }
   
   public void attach (DriverInterface driver) {
      super.attach (driver);
   }
   
   protected void addTrackingController() {
      TrackingController trackingController = new TrackingController(myJawModel, "trackingController");
      // mid-sagittal tongue surface nodes from front to back:
      int[] midsagittalNodes = new int[] {926, 919, 912, 873, 869, 908, 859, 927, 928, 846, 847};
      // just pick three of these nodes:
      int[] targetNodes = new int[] {919, 912, 873};
      
      for (int nodeIdx : targetNodes) {
         trackingController.addMotionTarget(tongue.getByNumber (nodeIdx));
      }

      
      for (MuscleExciter ex : tongue.getMuscleExciters()) {
         trackingController.addExciter(ex);
      }
  
//      trackingController.addRegularizationTerms(/*l2norm*/0.1, /*damping*/0.1);
      trackingController.addL2RegularizationTerm(/*l2norm*/0.01);
      trackingController.setMaxExcitationJump (0.1);
      trackingController.setNormalizeH (true);
      trackingController.createProbesAndPanel (this);
      addController(trackingController);
      
      RenderProps.setPointRadius (tongue, 0.8);
      trackingController.setTargetsPointRadius (1.0);
   }
   
}