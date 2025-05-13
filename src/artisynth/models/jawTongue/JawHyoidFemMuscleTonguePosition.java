package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.Probe;
import artisynth.models.dynjaw.JawModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.tongue3d.FemMuscleTongueDemo;
import artisynth.models.tongue3d.HexTongueDemo;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.Point3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.PathFinder;

import java.util.Timer;
import java.util.TimerTask;

public class JawHyoidFemMuscleTonguePosition extends JawHyoidFemMuscleTongue {


   public static final double hyoidTransStiffness = 200;
   public static final double hyoidRotStiffness = 10000;
//protected RigidBody body;

   public JawHyoidFemMuscleTonguePosition () {
      super();
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (new String[]{"HexElementMuscles"});

      myJawModel.setIntegrator(Integrator.ConstrainedBackwardEuler);
      // myJawModel.setMaxStepSizeSec (0.002);

      // FrameSpring hyoidSpring = myJawModel.frameSprings().get(0);
      // hyoidSpring.setStiffness(hyoidTransStiffness);
      // hyoidSpring.setRotaryStiffness(hyoidRotStiffness);
      // myJawModel.updateFrameMarkersRefPos();

      setupTongueRenderProps();
      FemMarker mkr = new FemMarker (-5,0, 145);
      RenderProps.setSphericalPoints (mkr,  2,  Color.BLUE);
      tongue.addMarker (mkr);
      
      Timer timer = new Timer();
      
      TimerTask task = new TimerTask() {
         public void run() {
            Point3d tipPos = mkr.getPosition ();
            System.out.println("pos: " + tipPos);
         }
      };
      timer.scheduleAtFixedRate (task, 0, 1000);
      
//      NumericOutputProbe mkrProbe =
//      new NumericOutputProbe ( //not sure how to format this
//        tongue, "markers/0:position", PathFinder.getSourceRelativePath (this, "PositionMkr.txt"), 0.01);
//      mkrProbe.setName("FemMarker Position");
//      mkrProbe.setDefaultDisplayRange (-4, 4);
//      mkrProbe.setStopTime (10);
//      addOutputProbe (mkrProbe);
   }
   
   public void addExciterProbe(String exciterName, double maxExcitation) {
	    if (exciterName.equals("bi_open") || exciterName.equals("bi_close")) {
	       if (getInputProbes().get (exciterName + " exciter probe") == null) {
	          NumericInputProbe nip =
	             new NumericInputProbe(this, "models/jawmodel/exciters/" + exciterName
	             + ":excitation", 0, 1.0);
	          
	          nip.addData (
	             new double[] { 0.00, 0.0,
	                            0.10, maxExcitation
	                          }, NumericInputProbe.EXPLICIT_TIME);
	          nip.setName (exciterName + " exciter probe");
	          nip.setInterpolationOrder (Order.CubicStep);
	          addInputProbe (nip);
	       }
	    } else {
	       if (getInputProbes().get (exciterName + " exciter probe") == null) {
	          NumericInputProbe nip =
	             new NumericInputProbe(this, "models/jawmodel/models/tongue/exciters/" + exciterName
	             + ":excitation", 0, 1.0);
	          
	          nip.addData (
	             new double[] { 0.00, 0.0,
	                            0.10, 0.0,
	                            0.80, maxExcitation
	                          }, NumericInputProbe.EXPLICIT_TIME);
	          nip.setName (exciterName + " exciter probe");
	          nip.setInterpolationOrder (Order.CubicStep);
	          addInputProbe (nip);
	    }
	 }
	}

   public void removeExciterProbe(String exciterName) {
	      Probe p = getInputProbes().get(exciterName + " exciter probe");
	      if (p != null) {
	         removeInputProbe(p);
	      }
	   }

   public static void setBlackWhite(MechModel mech, FemMuscleModel fem) {
      RenderProps.setFaceColor(fem, Color.WHITE);
      RenderProps.setLineColor(fem, Color.BLACK);
      fem.setSurfaceRendering(SurfaceRender.None);
      RenderProps.setLineWidth(fem, 2);
      RenderProps.setLineWidth(fem.getElements(), 2);

      for (RigidBody body : mech.rigidBodies())
      {
         RenderProps.setFaceColor(body, Color.WHITE);
         RenderProps.setLineColor(body, Color.BLACK);
         RenderProps.setDrawEdges(body, true);
         RenderProps.setLineWidth(body, 2);
         body.setAxisLength(0);
      }

      for (BodyConnector con : mech.bodyConnectors())
      {
         RenderProps.setVisible(con, false);
      }

      RenderProps.setVisible(mech.axialSprings(), false);
      RenderProps.setVisible(mech.multiPointSprings(), false);
      RenderProps.setVisible(mech.frameMarkers(), false);

      fem.setElementWidgetSize(1);

      for (FemElement e : fem.getElements()) {
         for (FemNode n : e.getNodes()) {
            if (n.getPosition().y < -1e-4) {
               RenderProps.setVisible(e, false);
               break;
            }
         }
      }

      for (MuscleBundle b : fem.getMuscleBundles()) {
         RenderProps.setVisible(b, false);
         RenderProps.setVisibleMode(b.getFibres(), PropertyMode.Inherited);
         for (Muscle m : b.getFibres()) {
            RenderProps.setVisibleMode(m, PropertyMode.Inherited);
         }
      }
   }

   public void attach(DriverInterface driver) {
      super.attach(driver);

      removeAllInputProbes();
      removeAllOutputProbes();

      File workingDir = new File(ArtisynthPath.getHomeDir() + "/r/");
      if (workingDir.exists()) {
         ArtisynthPath.setWorkingDir(workingDir);
      }

      if (myControlPanel != null) {
         FemMuscleTongueDemo.setSliderRange(
            myControlPanel, "excitation", 0,
            FemMuscleTongueDemo.muscleExcitationSliderMax);
      }
      
   }
	  public JawModel getJawModel () {
	      return myJawModel;
	   }

}
