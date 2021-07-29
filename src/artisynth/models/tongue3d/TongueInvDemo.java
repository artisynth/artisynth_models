package artisynth.models.tongue3d;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.inverse.InverseManager;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.Shading;

public class TongueInvDemo extends HexTongueDemo {
   
   public String getAbout() {
      return"A 3D FEM itongue model being controlled by the inverse tracking controller.\n\n"+
	    "The inverse controller was developed by Ian Stavness, please cite: \n" +
	    "Ian Stavness, John E Lloyd, and Sidney Fels. "+
	    "Inverse-Dynamics Simulation of Muscular-Hydrostat FEM Models. " + 
	    "International Society of Biomechanics Congress (ISB '11), Talk, July 2011.";
   }
   
   public enum TongueTarget {
      SURFACE, TIP, EMA, TIP_PLUS_LATERAL, TADA;
   }

   public static final TongueTarget defaultTongueTarget = TongueTarget.TIP;
   TongueTarget myTongueTarget = defaultTongueTarget;
   
   public TongueInvDemo() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      // use old names and files names for probes created by InverseManager
      InverseManager.useLegacyNames = true;
      
      // tongue.setIncompressible (FemModel3d.IncompMethod.OFF);
      tongue.setGravity(0, 0, 0);
//      tongue.setIntegrator(Integrator.Trapezoidal);
      mech.setGravity(0, 0, 0);
//      mech.setIntegrator(Integrator.Trapezoidal);

      setActivationColor(tongue);
      
      tongue.setSurfaceRendering(SurfaceRender.Shaded);
      RenderProps.setShading(tongue, Shading.SMOOTH);
//    RenderProps.setAlpha(tongue, 0.15);
      
//      Main.getInverseManager ().setProbeDuration (5.0);
      
      removeAllInputProbes();
      removeAllOutputProbes();

      ArtisynthPath.setWorkingDir(new File(ArtisynthPath.getSrcRelativePath(
	    this, "data/invtongue/tip")));

//      loadProbes ("0probes.art");
      addInverseController();
      
      addTipProbes(1.0);

      // tongue.setParticleDamping (0);
      // tongue.setStiffnessDamping (0);
      // tongue.setIncompressible (IncompMethod.OFF);
      // tongue.setMaterial (new NeoHookeanMaterial (2000.0, 0.45));

      //trackingController.setEnabled (false);
   }

   public ArrayList<ModelComponent> createTargetList (TongueTarget tt) {
      switch (tt) {
         case SURFACE:
            return getSurfaceTarget ();
         case TIP:
            return getTipTargets ();
         case EMA:
            return getEmaTargets ();
         case TIP_PLUS_LATERAL:
            ArrayList<ModelComponent> targets = getTipTargets ();
            targets.addAll (getLateralTargets ());
            return targets;
         case TADA:
            return getTadaTargets ();
         default:
            System.err.println ("HexTongueInv: unknown target type: "
            + tt.toString ());
      }
      return null;
   }

   public ArrayList<ModelComponent> getSurfaceTarget() {
      ArrayList<ModelComponent> targets = new ArrayList<ModelComponent>();
      for (Vertex3d v : tongue.getSurfaceMesh().getVertices()) {
	 FemNode n = tongue.getSurfaceNode (v);
	 if (n != null && n.isActive()) {
	    targets.add(n);
	 }
      }
      return targets;
   }

   public ArrayList<ModelComponent> getTipTargets() {
      ArrayList<ModelComponent> targets = new ArrayList<ModelComponent>();
      for (Vertex3d v : tongue.getSurfaceMesh().getVertices()) {
	 FemNode n = tongue.getSurfaceNode (v);
	 if (n != null && n.isActive() && isTipNode(n)) {
	    targets.add(n);
	 }
      }
      return targets;
   }
   
   public ArrayList<ModelComponent> getLateralTargets() {
      int[] nodeNums = new int[]{166, 301, 299, 141, 579, 554, 712, 714};
      ArrayList<ModelComponent> targets = new ArrayList<ModelComponent>(nodeNums.length);
      for (int i = 0; i < nodeNums.length; i++) {
	 targets.add(tongue.getByNumber(nodeNums[i]));
      }
      return targets;
   }
   

   private boolean isTipNode(FemNode n) {
      Point3d pos = n.getPosition();
      return (pos.x <= 0.06 && pos.z >= 0.089313537);
   }

   public ArrayList<ModelComponent> getEmaTargets() {

      int[] emaNodeIdxs = new int[] { 926, 919, 912, 873, 869 };

      ArrayList<ModelComponent> targets = new ArrayList<ModelComponent>();
      for (int idx : emaNodeIdxs) {
	 targets.add(tongue.getNode(idx));
      }
      return targets;
   }
   
   public ArrayList<ModelComponent> getTadaTargets() {

      int[] tadaNodeIdxs = new int[] { 926 } ;//XXX fill in node numbers here for TADA points

      ArrayList<ModelComponent> targets = new ArrayList<ModelComponent>();
      for (int idx : tadaNodeIdxs) {
         targets.add(tongue.getNode(idx));
      }
      return targets;
   }
   
   private void loadProbes(String probeFileName) {
      try {
	 scanProbes(
	       ArtisynthIO.newReaderTokenizer(
		     ArtisynthPath.getWorkingDirPath() + "/" + probeFileName));
      } catch (IOException e) {
	 e.printStackTrace();
      }
   }

   public void attach(DriverInterface driver) {
      super.attach(driver);

//       removeAllInputProbes();
//       removeAllOutputProbes();

//       ArtisynthPath.setWorkingDir(new File(ArtisynthPath.getSrcRelativePath(
// 	    this, "data/invtongue/tip")));

// //      loadProbes ("0probes.art");
//       addInverseController();
      
//       addTipProbes();
   }
   
   public void addTipProbes(double duration) {
      String ptname = "tip";

      String[] comps = new String[]{"p/"+ptname+"_ref","models/tongue/nodes/"+ptname};
      String[] props = new String[]{"position", "velocity"};
      String[] names = new String[]{"ref", "model"};

      for (int i = 0; i < comps.length; i++) {
         ModelComponent comp = mech.findComponent(comps[i]);
         if (comp == null) {
            return;
         }
         
	 for (int j = 0; j < props.length; j++) {
	    String name = names[i] + "_" +ptname+ "_" + props[j];
	    NumericOutputProbe op = new NumericOutputProbe(
		  comp, props[j], null, duration);
	    op.setStartStopTimes(0, duration);
	    op.setName(name);
	    op.setAttachedFileName(name+".txt");
	    addOutputProbe(op);
	    
	 }
      }
   }
   
   TrackingController trackingController;
   protected void addInverseController() {

      
      trackingController = new TrackingController(mech, "tcon");
      ArrayList<ModelComponent> tongueTargetNodes = createTargetList(myTongueTarget);
      for (ModelComponent comp : tongueTargetNodes) {
	 trackingController.addMotionTarget((MotionTargetComponent)comp);
      }
      System.out.println("numTargetNodes = "+tongueTargetNodes.size());

      trackingController.setTargetsPointRadius (0.0006);
      
//      for (MuscleBundle b : tongue.getMuscleBundles()) {
//	 trackingController.addExciter(b);
//      }
      
      for (MuscleExciter ex : tongue.getMuscleExciters()) {
	 trackingController.addExciter(ex);
      }
  
      trackingController.addRegularizationTerms(/*l2norm*/0.1, /*damping*/0.1);
//      trackingController.addL2RegularizationTerm(1);

      trackingController.createProbesAndPanel (this);
      addController(trackingController);
      tongue.setMaxColoredExcitation(0.8);
      addBreakPoint(0.5);
      addBreakPoint(1.0);
   }
   
   public void excolor(double ex) {
      mvis();
      minvis(ex);
      tongue.setMaxColoredExcitation(ex);
   }
   
   public void minvis(double threshold) {
      for (MuscleExciter ex : tongue.getMuscleExciters()) {
	 if (ex.getNetExcitation() < threshold) {
	    for (ExcitationComponent e : ex.getTargetView()) {
	       if (e instanceof Muscle) {
		  RenderProps.setVisible((Muscle)e, false);
	       }
	       else if (e instanceof MuscleBundle) {
		  RenderProps.setVisible((MuscleBundle)e, false);
	       }
	    }
	 }
      }
   }
   
   public void mvis() {
      for (MuscleExciter ex : tongue.getMuscleExciters()) {
	 for (ExcitationComponent e : ex.getTargetView()) {
	    if (e instanceof Muscle) {
	       RenderProps.setVisibleMode((Muscle) e, PropertyMode.Inherited);
	    }
	    else if (e instanceof MuscleBundle) {
	       RenderProps.setVisible((MuscleBundle)e, true);
	    }
	 }
      }
   }

}
