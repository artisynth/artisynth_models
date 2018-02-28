package artisynth.models.registration;

import java.io.IOException;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RollPitchJoint;
import artisynth.core.mechmodels.SphericalRpyJoint;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class ArticulatedRegistrationDemo extends RootModel {
   
   MechModel mech;
   RigidBody world;
   
   static final double BONE_DENSITY = 1000;
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      mech = new MechModel("mech");
      mech.setMaxStepSize(0.001);
      addModel(mech);
      
      world = new RigidBody("world");
      world.setDynamic(false);
      mech.add(world);
      
      // create squared arm
      createSquaredArm();
   }
   
   void createSquaredArm() {
      
      // humerus
      PolygonalMesh humerusMesh = MeshFactory.createBox(0.03, 0.03, 0.25);
      RigidBody humerus = new RigidBody("humerus");
      humerus.setMesh(humerusMesh);
      humerus.setDensity(BONE_DENSITY);
      humerus.setFrameDamping(0.1);
      humerus.setRotaryDamping(0.05);
      mech.add(humerus);
      // move down so shoulder joint at 2.5cm from top
      humerus.setPose(new RigidTransform3d(new Vector3d(0,0,-0.1), AxisAngle.IDENTITY));
      // add shoulder joint
      RollPitchJoint shoulder = new RollPitchJoint(humerus, world, new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,1,0,Math.PI/2)));
      shoulder.setName("shoulder");
      shoulder.setPitchRange(-60, 130);
      shoulder.setRollRange(-10, 130);
      mech.addBodyConnector(shoulder);
      
      
      // radius-ulna
      PolygonalMesh radulnaMesh = MeshFactory.createBox(0.03, 0.03, 0.25);
      radulnaMesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1, Math.PI/4)));
      RigidBody radulna = new RigidBody("radulna");
      radulna.setMesh(radulnaMesh);
      radulna.setDensity(BONE_DENSITY);
      radulna.setFrameDamping(0.1);
      radulna.setRotaryDamping(0.05);
      mech.add(radulna);
      // move down so 5cm overlap
      radulna.setPose(new RigidTransform3d(new Vector3d(0,0,-0.3), AxisAngle.IDENTITY));
      // add revolute joint
      RevoluteJoint elbow = new RevoluteJoint(radulna, humerus, 
         new RigidTransform3d(new Vector3d(0,0,-0.2), new AxisAngle(1,0,0, -Math.PI/2)));
      elbow.setName("elbow");
      elbow.setThetaRange(0, 150);
      mech.addBodyConnector(elbow);
      
      // hand
      PolygonalMesh handMesh = MeshFactory.createBox(0.05, 0.02, 0.075);
      RigidBody hand = new RigidBody("hand");
      hand.setMesh(handMesh);
      hand.setDensity(BONE_DENSITY);
      hand.setFrameDamping(0.05);
      hand.setRotaryDamping(0.005);
      mech.add(hand);
      hand.setPose(new RigidTransform3d(new Vector3d(0,0,-0.45), AxisAngle.IDENTITY));
      // add rpy-joint
      SphericalRpyJoint wrist = new SphericalRpyJoint(hand, radulna,
         new RigidTransform3d(new Vector3d(0,0,-0.425), AxisAngle.IDENTITY));
      wrist.setName("wrist");
      wrist.setRollRange(-90, 90);
      wrist.setPitchRange(-25, 25);
      wrist.setYawRange(-90, 50);
      mech.addBodyConnector(wrist);
      
      // pinky
      PolygonalMesh pinky1Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      pinky1Mesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1,Math.PI/4)));
      RigidBody pinky1 = new RigidBody("pinky1");
      pinky1.setMesh(pinky1Mesh);
      pinky1.setDensity(BONE_DENSITY);
      pinky1.setFrameDamping(0.001);
      pinky1.setRotaryDamping(0.0005);
      mech.add(pinky1);
      pinky1.setPose(new RigidTransform3d(new Vector3d(-0.0175,0,-0.5), AxisAngle.IDENTITY));
      RollPitchJoint pinkybase = new RollPitchJoint(pinky1, hand, 
         new RigidTransform3d(new Vector3d(-0.0175, 0, -0.485), new AxisAngle(0,1,0,-Math.PI/2)));
      pinkybase.setRollRange(-10, 95);
      pinkybase.setPitchRange(-20, 20);
      pinkybase.setName("pinky MP");
      mech.addBodyConnector(pinkybase);
      PolygonalMesh pinky2Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      RigidBody pinky2 = new RigidBody("pinky2");
      pinky2.setMesh(pinky2Mesh);
      pinky2.setDensity(BONE_DENSITY);
      pinky2.setFrameDamping(0.002);
      pinky2.setRotaryDamping(0.0005);
      mech.add(pinky2);
      pinky2.setPose(new RigidTransform3d(new Vector3d(-0.0175,0,-0.54), AxisAngle.IDENTITY));
      RevoluteJoint pinkymid = new RevoluteJoint(pinky2, pinky1, 
         new RigidTransform3d(new Vector3d(-0.0175, 0, -0.525), new AxisAngle(0,1,0,-Math.PI/2)));
      pinkymid.setThetaRange(0, 95);
      pinkymid.setName("pinky IP");
      mech.addBodyConnector(pinkymid);
      
      // thumb
      PolygonalMesh thumb1Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      thumb1Mesh.transform(new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1,Math.PI/4)));
      RigidBody thumb1 = new RigidBody("thumb1");
      thumb1.setMesh(thumb1Mesh);
      thumb1.setDensity(BONE_DENSITY);
      thumb1.setFrameDamping(0.002);
      thumb1.setRotaryDamping(0.0005);
      mech.add(thumb1);
      thumb1.setPose(new RigidTransform3d(new Vector3d(0.0175,0,-0.5), AxisAngle.IDENTITY));
      RollPitchJoint thumbbase = new RollPitchJoint(thumb1, hand, 
         new RigidTransform3d(new Vector3d(0.0175, 0, -0.485), new AxisAngle(0,1,0,-Math.PI/2)));
      thumbbase.setRollRange(-10, 95);
      thumbbase.setPitchRange(-20, 20);
      thumbbase.setName("thumb MP");
      mech.addBodyConnector(thumbbase);
      PolygonalMesh thumb2Mesh = MeshFactory.createBox(0.015, 0.015, 0.04);
      RigidBody thumb2 = new RigidBody("thumb2");
      thumb2.setMesh(thumb2Mesh);
      thumb2.setDensity(BONE_DENSITY);
      thumb2.setFrameDamping(0.001);
      thumb2.setRotaryDamping(0.0005);
      mech.add(thumb2);
      thumb2.setPose(new RigidTransform3d(new Vector3d(0.0175,0,-0.54), AxisAngle.IDENTITY));
      RevoluteJoint thumbmid = new RevoluteJoint(thumb2, thumb1, 
         new RigidTransform3d(new Vector3d(0.0175, 0, -0.525), new AxisAngle(0,1,0,-Math.PI/2)));
      thumbmid.setThetaRange(0, 95);
      thumbmid.setName("thumb IP");
      mech.addBodyConnector(thumbmid);
      
   }

}
