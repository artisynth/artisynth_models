package artisynth.models.palate;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;

public class SoftPalate_spring extends RootModel{
   public static final double defaultMaxStepSizeSec = 0.05;
   public static final Integrator defaultIntegrator =
      Integrator.BackwardEuler;
   static double muscleMaxForce = 1000;
   
   FemMuscleModel softPalate;
   MechModel myMechMod;
   
   public SoftPalate_spring () {
      super ();
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      
      
      myMechMod = new MechModel ("mech");
      myMechMod.setMaxStepSize (defaultMaxStepSizeSec);
      myMechMod.setIntegrator (defaultIntegrator);
      addModel (myMechMod);
      
      // Add Soft Palate
      softPalate = RegisteredSoftPalate.createSoftPalate(RegisteredSoftPalate.softPalateGeometryDir,
	    "ianMesh_mod_simp", "landmarks_registered", /*linearMaterial=*/true);
      softPalate.setName ("ianSoftPalate");
      myMechMod.addModel (softPalate);
      RegisteredSoftPalate.setSoftPalateRendering (softPalate);
      
      // Add rigid body
      RigidBody tongue = new RigidBody ("tongue");
      setBodyMesh (tongue, "src/artisynth/models/template/geometry/rigidBodies/tongue/tongueBadin.obj", 1);
      myMechMod.addRigidBody (tongue);
      tongue.setDynamic(false);
      
      // Add Spring
      FemNode3d palate_R1 = softPalate.getNode(1166);
      FemNode3d palate_R2 = softPalate.getNode(412);
      FemNode3d palate_L1 = softPalate.getNode(91);
      FemNode3d palate_L2 = softPalate.getNode(84);
      
      //FrameMarker tongue_R1 = new FrameMarker(104.0378, 18.283799, 92.632475);
      //FrameMarker tongue_L1 = new FrameMarker(104.03781, -18.283789, 92.632491);
      FrameMarker tongue_R1 = new FrameMarker(98.250718, 16.99649, 92.632475);
      FrameMarker tongue_L1 = new FrameMarker(98.250718, -16.99649, 92.632475);

      tongue_R1.setFrame(tongue);
      tongue_L1.setFrame(tongue);
      myMechMod.addFrameMarker(tongue_R1);
      myMechMod.addFrameMarker(tongue_L1);
      
      MultiPointMuscle spring_R = new MultiPointMuscle ("spring_R");
      spring_R = MultiPointMuscle.createPai (muscleMaxForce, 0, 1.0, 0);
      spring_R.addPoint (palate_R1);
      spring_R.addPoint (tongue_R1);
      spring_R.addPoint (palate_R2);
      myMechMod.addMultiPointSpring (spring_R);
      
      MultiPointMuscle spring_L = new MultiPointMuscle ("spring_L");
      spring_L = MultiPointMuscle.createPai (muscleMaxForce, 0, 1.0, 0);
      spring_L.addPoint (palate_L1);
      spring_L.addPoint (tongue_L1);
      spring_L.addPoint (palate_L2);
      myMechMod.addMultiPointSpring (spring_L);
      
      RenderProps.setPointStyle (softPalate, PointStyle.SPHERE);
      
      RenderProps.setLineStyle (myMechMod.multiPointSprings(), LineStyle.SPINDLE);
      RenderProps.setLineColor (myMechMod.multiPointSprings(), Color.RED);
      RenderProps.setLineRadius (myMechMod.multiPointSprings(), 0.5);
      RenderProps.setPointStyle(myMechMod.frameMarkers(), PointStyle.SPHERE);
      RenderProps.setPointColor (myMechMod.frameMarkers(), Color.BLUE);
      RenderProps.setPointRadius (myMechMod.frameMarkers(), 0.3);
      
      // Set collision
      myMechMod.setCollisionBehavior(tongue, softPalate, true);
      
      // Set Most nodes not dynamic
      for (FemNode3d node: softPalate.getNodes()) {
	 node.setDynamic(false);
      }
      
   }
   private void setBodyMesh (RigidBody body, String meshname, double scale) {
      try {
         PolygonalMesh mesh = new PolygonalMesh (new File (meshname));
         mesh.scale (scale);
         body.setMesh (mesh, meshname);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   ControlPanel myControlPanel = null;
   public void attach (DriverInterface driver) {
      super.attach (driver);
      ControlPanel panel = new ControlPanel("options");
      panel.addWidget ("spring_R", myMechMod, "multiPointSprings/0" +
            ":excitation", 0.0, 1.0);
      panel.addWidget ("spring_L", myMechMod, "multiPointSprings/1" +
            ":excitation", 0.0, 1.0);
      addControlPanel(panel);
   }
}
