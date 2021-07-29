package artisynth.models.registration.test;

import java.io.IOException;

import artisynth.core.materials.LinearFrameMaterial;
import artisynth.core.mechmodels.DynamicComponent;
import artisynth.core.mechmodels.ForceComponent;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshFactory;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector3d;

public class FrameFrameForceTest extends RootModel {
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      MechModel mech = new MechModel ("mech");
      // mech.setGravity (new Vector3d(0,0,0)); 
      mech.setGravity (new Vector3d(0,0,-9.8));  // PARDISO hangs
      addModel (mech);
      
      // ground
      RigidBody ground = new RigidBody("ground");
      ground.setAxisLength (0.01);
      ground.setDynamic (false);
      mech.addRigidBody (ground);
      
      // box
      RigidBody box = new RigidBody("box");
      box.addMesh (MeshFactory.createBox (0.05, 0.05, 0.05));
      box.setPose (new RigidTransform3d(new Vector3d(0,0,0.1), AxisAngle.IDENTITY));
      box.setAxisLength (0.01);
      mech.addRigidBody (box);
      
      // frame spring
      FrameSpring fs = new FrameSpring ("spring");
      fs.setMaterial (new LinearFrameMaterial (0.1, 0.0001, 0, 0));
      fs.setFrames (box, ground, new RigidTransform3d(new Vector3d(0,0,0.05), AxisAngle.IDENTITY));
      mech.addFrameSpring (fs);
      
      //      RigidSphere sphere = new RigidSphere("sphere", 0.03, 1000);
      //      // sphere.addMesh (MeshFactory.createOctadecahedralSphere (0.03, 4));
      //      sphere.setPose (box.getPose ());
      //      mech.addRigidBody (sphere);
      //      
      //      // attach sphere to box
      //      FrameFrameAttachment attachment = new FrameFrameAttachment (sphere, box);
      //      attachment.setName ("box-to-sphere");
      //      mech.addAttachment (attachment); 
      //      sphere.setDynamic (false);
      
       // add force directly to box
       mech.addForceEffector (new StaticForce(box, new double[] {0, 0, 0, 0, 0, 0.000005}));
      
   }
   
   private static class StaticForce extends ModelComponentBase implements ForceComponent {

      DynamicComponent comp;
      double[] force;
      
      public StaticForce(DynamicComponent c, double[] force) {
         this.comp = c;
         this.force = force;
      }
      
      @Override
      public void applyForces (double t) {
         comp.addForce (force, 0);
      }

      @Override
      public void addSolveBlocks (SparseNumberedBlockMatrix M) {
         // nothing
      }

      @Override
      public void addPosJacobian (SparseNumberedBlockMatrix M, double s) {
         // nothing
      }

      @Override
      public void addVelJacobian (SparseNumberedBlockMatrix M, double s) {
         // nothing
      }

      @Override
      public int getJacobianType () {
         return Matrix.SYMMETRIC;
      }
      
   }

}
