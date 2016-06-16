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
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;

public class SoftPalate_springPeter extends RootModel{
   public static final double defaultMaxStepSizeSec = 0.05;
   public static final Integrator defaultIntegrator =
      Integrator.BackwardEuler;
   static double muscleMaxForce = 1000;
   
   FemMuscleModel softPalate;
   MechModel myMechMod;
   
   public SoftPalate_springPeter () {
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
      
      // anchor the palate via muscle attachments and fixed points
      anchorPalate();
      
      // Set Most nodes not dynamic
//      for (FemNode3d node: softPalate.getNodes()) {
//	 double zValue = 115.0;	// all nodes above the plane z=115.0 are inactive for now
//	 if (node.getPosition().z > zValue) 
//	    node.setDynamic(false);
//      }
      
      
      // Set collision
      myMechMod.setCollisionBehavior(tongue, softPalate, true);
      
      // set some rendering behavior
      RenderProps.setPointStyle (softPalate, PointStyle.SPHERE);
      RenderProps.setLineStyle (myMechMod.multiPointSprings(), LineStyle.SPINDLE);
      RenderProps.setLineColor (myMechMod.multiPointSprings(), Color.RED);
      RenderProps.setLineRadius (myMechMod.multiPointSprings(), 0.5);
      RenderProps.setPointStyle(myMechMod.frameMarkers(), PointStyle.SPHERE);
      RenderProps.setPointColor (myMechMod.frameMarkers(), Color.BLUE);
      RenderProps.setPointRadius (myMechMod.frameMarkers(), 0.3);
      RenderProps.setPointStyle(myMechMod.particles(), PointStyle.SPHERE);
      RenderProps.setPointColor(myMechMod.particles(), Color.GREEN);
      RenderProps.setPointRadius(myMechMod.particles(), 0.8);
      
   }
   
   
   private void anchorPalate()
   {
      // the tensor veli palatini attaches to the hamulus
      Particle hamulus_R = new Particle(1.0, 111.0, 25.0, 127.0);
      Particle hamulus_L = new Particle(1.0, 111.0, -25.0, 127.0);
      hamulus_R.setDynamic(false);
      hamulus_L.setDynamic(false);
      myMechMod.addParticle(hamulus_R);
      myMechMod.addParticle(hamulus_L);
      
      // the levator veli palatini --> for now I just make all nodes above a cutoff point static, thus anchoring levator
      for (FemNode3d node: softPalate.getNodes()) 
      {
	 double zValue = 130.0;	// all nodes above the plane z=115.0 are inactive for now
	 if (node.getPosition().z > zValue) 
	    node.setDynamic(false);
      }
      
      // the palatoglossus should attach right at the tongue
      	// perhaps the nodes (R: node 425,412,413 ; L: node 56,84,70) should be attached to the tongue?
      
      // the palatopharyngeus attaches to the pharyngeal wall
      	// for now just anchored statically
      Particle pharAnchor_R = new Particle(1.0, 140.0, 18.0, 75.0);
      Particle pharAnchor_L = new Particle(1.0, 140.0, -18.0, 75.0);
      pharAnchor_R.setDynamic(false);
      pharAnchor_L.setDynamic(false);
      myMechMod.addParticle(pharAnchor_R);
      myMechMod.addParticle(pharAnchor_L);
      
      // finally, the soft palate should be attached to the hard palate
      	// for now, if we just anchor all nodes for x<98.0, that will do a decent job of anchoring the front edge
      for (FemNode3d node: softPalate.getNodes()) 
      {
	 double xValue = 98.0;
	 if (node.getPosition().x < xValue) 
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
   public void attach (DriverInterface driver) 
   {
      super.attach (driver);
      ControlPanel panel = new ControlPanel("options");
      panel.addWidget ("spring_R", myMechMod, "multiPointSprings/0" +
            ":excitation", 0.0, 1.0);
      panel.addWidget ("spring_L", myMechMod, "multiPointSprings/1" +
            ":excitation", 0.0, 1.0);
      panel.addWidget ("pharSpring_R", myMechMod, "multiPointSprings/2" +
            ":excitation", 0.0, 1.0);
      panel.addWidget ("pharSpring_L", myMechMod, "multiPointSprings/3" +
            ":excitation", 0.0, 1.0);
      addControlPanel(panel);
      
      // get the muscle controls from the existing soft palate model (these muscles look bad though!)
      myControlPanel = RegisteredSoftPalate.createControlPanel (this, softPalate, myMechMod);
      if (softPalate.getMuscleBundles().size()>0)
	 RegisteredSoftPalate.createMusclePanel(this, softPalate);
   }
   
   private void pullPharMuscles()
   {
      // This is just a hack to pull the palato-pharygeus muscles up from colliding with the tongue
      // I don't think this is needed, it takes care of itself automatically once simulation is started
      
      // good nodes: (R: 359, 357, 372), (L: 2,7,8)
      FemNode3d pPhar_R1 = softPalate.getNode(357);
      FemNode3d pPhar_L1 = softPalate.getNode(0);
      
      Particle pulley_R1 = new Particle(1.0, 155.0, 18.0, 105.0);
      Particle pulley_L1 = new Particle(1.0, 155.0, -18.0, 105.0);
      pulley_R1.setDynamic(false);
      pulley_L1.setDynamic(false);
      myMechMod.addParticle(pulley_R1);
      myMechMod.addParticle(pulley_L1);
      
      //MultiPointMuscle pharSpring_R = new MultiPointMuscle ("pharSpring_R");
      MultiPointMuscle pharSpring_R = MultiPointMuscle.createPai(muscleMaxForce, 0, 1.0, 0);
      pharSpring_R.setName("pharSpring_R");
      pharSpring_R.addPoint (pPhar_R1);
      pharSpring_R.addPoint (pulley_R1);
      myMechMod.addMultiPointSpring (pharSpring_R);
      
      MultiPointMuscle pharSpring_L = MultiPointMuscle.createPai(muscleMaxForce, 0, 1.0, 0);
      pharSpring_L.setName("pharSpring_L");
      pharSpring_L.addPoint (pPhar_L1);
      pharSpring_L.addPoint (pulley_L1);
      myMechMod.addMultiPointSpring (pharSpring_L);      
      
      // now pull the fem out
      //      pharSpring_R.setExcitation(0.35);
      //      pharSpring_L.setExcitation(0.35);
      //      super.advance(0, 1000000);
      //      pharSpring_R.setExcitation(0.0);
      //      pharSpring_L.setExcitation(0.0);
   }
}
