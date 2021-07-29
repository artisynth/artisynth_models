package artisynth.models.registration.demos;

import java.awt.Color;
import java.io.IOException;

import javax.swing.JSeparator;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RollPitchJoint;
import artisynth.core.mechmodels.SphericalRpyJoint;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.registration.DynamicRegistrationController;
import artisynth.models.registration.MeshCorrespondenceController;
import artisynth.models.registration.correspondences.ICPMeshCorrespondence;
import artisynth.models.registration.weights.GaussianWeightFunction;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;

/**
 * Demonstrates registration of an articulated arm
 */
public class ArticulatedRegistrationDemo extends RootModel {
   
   MechModel mech;
   RenderableComponentList<RigidBody> sources;
   RenderableComponentList<RigidBody> targets;
   RigidBody world;
   
   static final double BONE_DENSITY = 1000;
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      mech = new MechModel("mech");
      mech.setMaxStepSize(0.001);
      addModel(mech);
      
      sources = new RenderableComponentList<> (RigidBody.class, "sources");
      mech.add (sources);
      targets = new RenderableComponentList<> (RigidBody.class, "targets");
      mech.add (targets);
      
      world = new RigidBody("world");
      world.addMesh (MeshFactory.createOctahedralSphere (0.025, 1));
      RenderProps.setFaceColor (world, Color.BLUE);
      world.setDynamic(false);
      mech.add(world);

      // create squared arm
      createTargetArm();
      createSourceArm();
      addRegistrationControllers();
      
      setupRenderProps();
      
      addControlPanel();
   }
   
   void addRegistrationControllers() {
      
      DynamicRegistrationController dreg = new DynamicRegistrationController (mech);
      dreg.setName ("registration");
      addController (dreg);
      
      for (RigidBody source : sources) {
         String name = source.getName ().replace ("source_", "");
         String targetName = source.getName ().replace ("source", "target");
         RigidBody target = targets.get (targetName);
         if (target != null && target.numMeshComps () > 0) {
            //            // GMM-based registration correspondences
            //            GMMMeshCorrespondence corr = new GMMMeshCorrespondence ();
            //            corr.setNearestK (20);
            
            // Gaussian ICP weight function
            ICPMeshCorrespondence corr = new ICPMeshCorrespondence ();
            corr.setWeightFunction (new GaussianWeightFunction (1, 0.01, false));
            
            MeshCorrespondenceController mcc = dreg.addRegistrationTarget (source.getMeshComp (0), target.getMeshComp (0), 
               100, corr);
            mcc.setName (name + "_correspondences");
         }
      }
      
   }
   
   void setupRenderProps() {
      for (RigidBody rb : targets) {
         if (rb.getName ().startsWith ("target")) {
            RenderProps.setFaceColor (rb, Color.CYAN.darker ());
         }
      }
      RenderProps.setAlpha (targets, 0.5);
      
      for (RigidBody rb : sources) {
         if (rb.getName ().startsWith ("source")) {
            RenderProps.setFaceColor (rb, Color.MAGENTA.darker ());
         }
      }
   }
   
   void addControlPanel() {
      ControlPanel panel = new ControlPanel ("Articulated Panel");
            
      panel.addLabel ("TARGETS:");
      for (BodyConnector c : mech.bodyConnectors ()) {
         if (c.getName ().startsWith ("target")) {
            panel.addWidget (new JSeparator ());
            panel.addLabel (c.getName () + ":");
            if (c instanceof RevoluteJoint) {
               panel.addWidget (c, "theta");
            } else if (c instanceof RollPitchJoint) {
               panel.addWidget (c, "roll");
               panel.addWidget (c, "pitch");
            } else if (c instanceof SphericalRpyJoint) {
               panel.addWidget (c, "roll");
               panel.addWidget (c, "pitch");
               panel.addWidget (c, "yaw");
            }
         }
      }
      panel.pack ();
      
      addControlPanel (panel);
   }
   
   void createTargetArm() {
      
      int subdivides = 4;
      
      // humerus
      PolygonalMesh humerusMesh = MeshFactory.createBox(0.03, 0.03, 0.25);
      humerusMesh = MeshFactory.subdivide (humerusMesh, subdivides);
      RigidBody humerus = new RigidBody("target_humerus");
      humerus.addMesh(humerusMesh);
      humerus.setDensity(BONE_DENSITY);
      humerus.setFrameDamping(0.1);
      humerus.setRotaryDamping(0.05);
      targets.add(humerus);
      // move down so shoulder joint at 2.5cm from top
      humerus.setPose(new RigidTransform3d(new Vector3d(0,0,-0.1), AxisAngle.IDENTITY));
      humerus.updateAttachmentPosStates ();
      // add shoulder joint
      RollPitchJoint shoulder = new RollPitchJoint(humerus, world, new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,1,0,Math.PI/2)));
      shoulder.setName("target_shoulder");
      shoulder.setPitchRange(-60, 130);
      shoulder.setRollRange(-10, 130);
      shoulder.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(shoulder);
      
      // radius-ulna
      // PolygonalMesh radiusulnaMesh = MeshFactory.createBox (0.03,  0.03, 0.1);
      PolygonalMesh radiusulnaMesh = new PolygonalMesh();
      PolygonalMesh box1 = MeshFactory.createBox(0.03, 0.03, 0.1);
      box1.translate (new Vector3d(0, 0, 0.075));
      PolygonalMesh box2 = MeshFactory.createBox(0.03, 0.03, 0.1);
      box2.translate (new Vector3d(0, 0, -0.075));
      radiusulnaMesh.addMesh (box1);
      radiusulnaMesh.addMesh (box2);
      
      
      radiusulnaMesh = MeshFactory.subdivide (radiusulnaMesh, subdivides);
      radiusulnaMesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1, Math.PI/4)));
      RigidBody radiusulna = new RigidBody("target_radiusulna");
      radiusulna.addMesh(radiusulnaMesh);
      radiusulna.setDensity(BONE_DENSITY);
      radiusulna.setFrameDamping(0.1);
      radiusulna.setRotaryDamping(0.05);
      targets.add(radiusulna);
      // move down so 5cm overlap
      radiusulna.setPose(new RigidTransform3d(new Vector3d(0,0,-0.3), AxisAngle.IDENTITY));
      // add revolute joint
      RevoluteJoint elbow = new RevoluteJoint(radiusulna, humerus, 
         new RigidTransform3d(new Vector3d(0,0,-0.2), new AxisAngle(1,0,0, -Math.PI/2)));
      elbow.setName("target_elbow");
      elbow.setThetaRange(0, 150);
      elbow.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(elbow);
      
      // hand
      PolygonalMesh handMesh = MeshFactory.createBox(0.05, 0.02, 0.075);
      handMesh = MeshFactory.subdivide (handMesh, subdivides);
      RigidBody hand = new RigidBody("target_hand");
      hand.addMesh(handMesh);
      hand.setDensity(BONE_DENSITY);
      hand.setFrameDamping(0.05);
      hand.setRotaryDamping(0.005);
      targets.add(hand);
      hand.setPose(new RigidTransform3d(new Vector3d(0,0,-0.45), AxisAngle.IDENTITY));
      // add rpy-joint
      SphericalRpyJoint wrist = new SphericalRpyJoint(hand, radiusulna,
         new RigidTransform3d(new Vector3d(0,0,-0.425), AxisAngle.IDENTITY));
      wrist.setName("target_wrist");
      wrist.setRollRange(-90, 90);
      wrist.setPitchRange(-25, 25);
      wrist.setYawRange(-90, 50);
      wrist.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(wrist);

      // pinky
      PolygonalMesh pinky1Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      pinky1Mesh = MeshFactory.subdivide (pinky1Mesh, subdivides);
      pinky1Mesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1,Math.PI/4)));
      RigidBody pinky1 = new RigidBody("target_pinky1");
      pinky1.addMesh(pinky1Mesh);
      pinky1.setDensity(BONE_DENSITY);
      pinky1.setFrameDamping(0.001);
      pinky1.setRotaryDamping(0.0005);
      targets.add(pinky1);
      pinky1.setPose(new RigidTransform3d(new Vector3d(-0.0175,0,-0.5), AxisAngle.IDENTITY));
      RollPitchJoint pinkybase = new RollPitchJoint(pinky1, hand, 
         new RigidTransform3d(new Vector3d(-0.0175, 0, -0.485), new AxisAngle(0,1,0,-Math.PI/2)));
      pinkybase.setRollRange(-10, 95);
      pinkybase.setPitchRange(-20, 20);
      pinkybase.setName("target_pinky_MP");
      pinkybase.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(pinkybase);
      
      PolygonalMesh pinky2Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      pinky2Mesh = MeshFactory.subdivide (pinky2Mesh, subdivides);
      RigidBody pinky2 = new RigidBody("target_pinky2");
      pinky2.addMesh(pinky2Mesh);
      pinky2.setDensity(BONE_DENSITY);
      pinky2.setFrameDamping(0.002);
      pinky2.setRotaryDamping(0.0005);
      targets.add(pinky2);
      pinky2.setPose(new RigidTransform3d(new Vector3d(-0.0175,0,-0.54), AxisAngle.IDENTITY));
      RevoluteJoint pinkymid = new RevoluteJoint(pinky2, pinky1, 
         new RigidTransform3d(new Vector3d(-0.0175, 0, -0.525), new AxisAngle(0,1,0,-Math.PI/2)));
      pinkymid.setThetaRange(0, 95);
      pinkymid.setName("target_pinky_IP");
      pinkymid.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(pinkymid);
      
      // thumb
      PolygonalMesh thumb1Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      thumb1Mesh = MeshFactory.subdivide (thumb1Mesh, subdivides);
      thumb1Mesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1,Math.PI/4)));
      RigidBody thumb1 = new RigidBody("target_thumb1");
      // thumb1.addMesh(thumb1Mesh);
      thumb1.setDensity(BONE_DENSITY);
      thumb1.setFrameDamping(0.002);
      thumb1.setRotaryDamping(0.0005);
      targets.add(thumb1);
      thumb1.setPose(new RigidTransform3d(new Vector3d(0.0175,0,-0.5), AxisAngle.IDENTITY));
      RollPitchJoint thumbbase = new RollPitchJoint(thumb1, hand, 
         new RigidTransform3d(new Vector3d(0.0175, 0, -0.485), new AxisAngle(0,1,0,-Math.PI/2)));
      thumbbase.setRollRange(-10, 95);
      thumbbase.setPitchRange(-20, 20);
      thumbbase.setName("target_thumb_MP");
      thumbbase.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(thumbbase);
      
      PolygonalMesh thumb2Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      thumb2Mesh = MeshFactory.subdivide (thumb2Mesh, subdivides);
      RigidBody thumb2 = new RigidBody("target_thumb2");
      thumb2.addMesh(thumb2Mesh);
      thumb2.setDensity(BONE_DENSITY);
      thumb2.setFrameDamping(0.001);
      thumb2.setRotaryDamping(0.0005);
      targets.add(thumb2);
      thumb2.setPose(new RigidTransform3d(new Vector3d(0.0175,0,-0.54), AxisAngle.IDENTITY));
      RevoluteJoint thumbmid = new RevoluteJoint(thumb2, thumb1, 
         new RigidTransform3d(new Vector3d(0.0175, 0, -0.525), new AxisAngle(0,1,0,-Math.PI/2)));
      thumbmid.setThetaRange(0, 95);
      thumbmid.setName("target_thumb_IP");
      thumbmid.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(thumbmid);
      
      // set all as non-dynamic
      humerus.setDynamic (false);
      radiusulna.setDynamic (false);
      hand.setDynamic (false);
      pinky1.setDynamic (false);
      pinky2.setDynamic (false);
      thumb1.setDynamic (false);
      thumb2.setDynamic (false);
      
   }
   
   void createSourceArm() {
      
      int slices = 20;
      double s = 1.2; // Math.cbrt (6/Math.PI);
      
      // humerus
      PolygonalMesh humerusMesh = MeshFactory.createEllipsoid(0.015*s, 0.015*s, 0.125*s, slices);
      RigidBody humerus = new RigidBody("source_humerus");
      humerus.addMesh(humerusMesh);
      humerus.setDensity(BONE_DENSITY);
      humerus.setFrameDamping(0.1);
      humerus.setRotaryDamping(0.05);
      sources.add(humerus);
      // move down so shoulder joint at 2.5cm from top
      humerus.setPose(new RigidTransform3d(new Vector3d(0,0,-0.1), AxisAngle.IDENTITY));
      // add shoulder joint
      RollPitchJoint shoulder = new RollPitchJoint(humerus, world, new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,1,0,Math.PI/2)));
      shoulder.setName("source_shoulder");
      shoulder.setPitchRange(-60, 130);
      shoulder.setRollRange(-10, 130);
      shoulder.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(shoulder);
      
      // radius-ulna
      PolygonalMesh radiusulnaMesh = MeshFactory.createEllipsoid(0.015*s, 0.015*s, 0.125*s, slices);
      radiusulnaMesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1, Math.PI/4)));
      RigidBody radiusulna = new RigidBody("source_radiusulna");
      radiusulna.addMesh(radiusulnaMesh);
      radiusulna.setDensity(BONE_DENSITY);
      radiusulna.setFrameDamping(0.1);
      radiusulna.setRotaryDamping(0.05);
      sources.add(radiusulna);
      // move down so 5cm overlap
      radiusulna.setPose(new RigidTransform3d(new Vector3d(0,0,-0.3), AxisAngle.IDENTITY));
      // add revolute joint
      RevoluteJoint elbow = new RevoluteJoint(radiusulna, humerus, 
         new RigidTransform3d(new Vector3d(0,0,-0.2), new AxisAngle(1,0,0, -Math.PI/2)));
      elbow.setName("source_elbow");
      elbow.setThetaRange(0, 150);
      elbow.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(elbow);
      
      // hand
      PolygonalMesh handMesh = MeshFactory.createEllipsoid(0.025*s, 0.01*s, 0.0375*s, slices);
      RigidBody hand = new RigidBody("source_hand");
      hand.addMesh(handMesh);
      hand.setDensity(BONE_DENSITY);
      hand.setFrameDamping(0.05);
      hand.setRotaryDamping(0.005);
      sources.add(hand);
      hand.setPose(new RigidTransform3d(new Vector3d(0,0,-0.45), AxisAngle.IDENTITY));
      // add rpy-joint
      SphericalRpyJoint wrist = new SphericalRpyJoint(hand, radiusulna,
         new RigidTransform3d(new Vector3d(0,0,-0.425), AxisAngle.IDENTITY));
      wrist.setName("source_wrist");
      wrist.setRollRange(-90, 90);
      wrist.setPitchRange(-25, 25);
      wrist.setYawRange(-90, 50);
      wrist.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(wrist);
      
      // pinky
      PolygonalMesh pinky1Mesh = MeshFactory.createEllipsoid(0.0075*s, 0.0075*s, 0.02*s, slices);
      pinky1Mesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1,Math.PI/4)));
      RigidBody pinky1 = new RigidBody("source_pinky1");
      pinky1.addMesh(pinky1Mesh);
      pinky1.setDensity(BONE_DENSITY);
      pinky1.setFrameDamping(0.001);
      pinky1.setRotaryDamping(0.0005);
      sources.add(pinky1);
      pinky1.setPose(new RigidTransform3d(new Vector3d(-0.0175,0,-0.5), AxisAngle.IDENTITY));
      RollPitchJoint pinkybase = new RollPitchJoint(pinky1, hand, 
         new RigidTransform3d(new Vector3d(-0.0175, 0, -0.485), new AxisAngle(0,1,0,-Math.PI/2)));
      pinkybase.setRollRange(-10, 95);
      pinkybase.setPitchRange(-20, 20);
      pinkybase.setName("source_pinky_MP");
      pinkybase.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(pinkybase);
      
      PolygonalMesh pinky2Mesh = MeshFactory.createEllipsoid(0.0075*s, 0.0075*s, 0.02*s, slices);
      RigidBody pinky2 = new RigidBody("source_pinky2");
      pinky2.addMesh(pinky2Mesh);
      pinky2.setDensity(BONE_DENSITY);
      pinky2.setFrameDamping(0.002);
      pinky2.setRotaryDamping(0.0005);
      sources.add(pinky2);
      pinky2.setPose(new RigidTransform3d(new Vector3d(-0.0175,0,-0.54), AxisAngle.IDENTITY));
      RevoluteJoint pinkymid = new RevoluteJoint(pinky2, pinky1, 
         new RigidTransform3d(new Vector3d(-0.0175, 0, -0.525), new AxisAngle(0,1,0,-Math.PI/2)));
      pinkymid.setThetaRange(0, 95);
      pinkymid.setName("source_pinky_IP");
      pinkymid.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(pinkymid);
      
      // thumb
      PolygonalMesh thumb1Mesh = MeshFactory.createEllipsoid(0.0075*s, 0.0075*s, 0.02*s, slices);
      thumb1Mesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1,Math.PI/4)));
      RigidBody thumb1 = new RigidBody("source_thumb1");
      thumb1.addMesh(thumb1Mesh);
      thumb1.setDensity(BONE_DENSITY);
      thumb1.setFrameDamping(0.002);
      thumb1.setRotaryDamping(0.0005);
      sources.add(thumb1);
      thumb1.setPose(new RigidTransform3d(new Vector3d(0.0175,0,-0.5), AxisAngle.IDENTITY));
      RollPitchJoint thumbbase = new RollPitchJoint(thumb1, hand, 
         new RigidTransform3d(new Vector3d(0.0175, 0, -0.485), new AxisAngle(0,1,0,-Math.PI/2)));
      thumbbase.setRollRange(-10, 95);
      thumbbase.setPitchRange(-20, 20);
      thumbbase.setName("source_thumb_MP");
      thumbbase.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(thumbbase);
      
      PolygonalMesh thumb2Mesh = MeshFactory.createEllipsoid(0.0075*s, 0.0075*s, 0.02*s, slices);
      RigidBody thumb2 = new RigidBody("source_thumb2");
      thumb2.addMesh(thumb2Mesh);
      thumb2.setDensity(BONE_DENSITY);
      thumb2.setFrameDamping(0.001);
      thumb2.setRotaryDamping(0.0005);
      sources.add(thumb2);
      thumb2.setPose(new RigidTransform3d(new Vector3d(0.0175,0,-0.54), AxisAngle.IDENTITY));
      RevoluteJoint thumbmid = new RevoluteJoint(thumb2, thumb1, 
         new RigidTransform3d(new Vector3d(0.0175, 0, -0.525), new AxisAngle(0,1,0,-Math.PI/2)));
      thumbmid.setThetaRange(0, 95);
      thumbmid.setName("source_thumb_IP");
      thumbmid.setAlwaysAdjustBodyA (true);
      mech.addBodyConnector(thumbmid);
      
   }
   
   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);
      
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }
   }

}
