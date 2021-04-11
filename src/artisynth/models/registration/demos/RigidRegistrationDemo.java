package artisynth.models.registration.demos;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.mechmodels.DynamicMeshComponent;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidMeshComp;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.workspace.RootModel;
import artisynth.models.registration.DynamicRegistrationController;
import artisynth.models.registration.MeshCorrespondenceController;
import artisynth.models.registration.correspondences.ICPMeshCorrespondence;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;

/**
 * Registration demo with a single rigid body
 */
public class RigidRegistrationDemo extends RootModel {
   
   MechModel mech;
   RenderableComponentList<RigidBody> sources;
   RenderableComponentList<RigidBody> targets;
   
   static final double BONE_DENSITY = 1000;
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      mech = new MechModel("mech");
      mech.setMaxStepSize(0.001);
      mech.setGravity (0, 0, 1e-10);
      addModel(mech);
      
      sources = new RenderableComponentList<> (RigidBody.class, "sources");
      mech.add (sources);
      targets = new RenderableComponentList<> (RigidBody.class, "targets");
      mech.add (targets);

      // create squared arm
      createTargets();
      createSources();
      addRegistrationControllers();
      
      setupRenderProps();
   }
   
   void addRegistrationControllers() {
      
      DynamicRegistrationController dmc = new DynamicRegistrationController(mech);
      dmc.setName ("registration");
      
      for (RigidBody source : sources) {
         String sourceName = source.getName ();
         String targetName = sourceName.replace ("source", "target");
         String name = sourceName.replace ("_source", "");
         RigidBody targetBody = targets.get (targetName);
         if (targetBody != null) {
            // add source to controller
            DynamicMeshComponent sourceMesh = source.getMeshComp (0);
            RigidMeshComp targetMesh = targetBody.getMeshComp (0);

            // add target with computed correspondences
            MeshCorrespondenceController mcc = dmc.addRegistrationTarget (sourceMesh, targetMesh, 
               1.0, new ICPMeshCorrespondence());
            mcc.setName (name + "_correspondences");
         }
      }
      
      addController (dmc);
      
   }
   
   void setupRenderProps() {
      for (RigidBody rb : targets) {
         if (rb.getName ().startsWith ("target")) {
            RenderProps.setFaceColor (rb, Color.CYAN);
            RenderProps.setAlpha (rb, 0.8);
         }
      }
   }
      
   void createTargets() {
      
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
      humerus.transformPose (new RigidTransform3d(new Vector3d(-0.1, 0, 0), AxisAngle.IDENTITY));
      
      // set all as non-dynamic
      humerus.setDynamic (false);
      
   }
   
   void createSources() {
      
      int slices = 20;
      double s = 1.2; // Math.cbrt (6/Math.PI);
      
      // humerus
      PolygonalMesh humerusMesh = MeshFactory.createEllipsoid(0.015*s, 0.015*s, 0.125*s, slices);
      // PolygonalMesh humerusMesh = MeshFactory.createBox(0.03, 0.03, 0.25);
      RigidBody humerus = new RigidBody("source_humerus");
      humerus.addMesh(humerusMesh);
      humerus.setDensity(BONE_DENSITY);
      humerus.setFrameDamping(0.1);
      humerus.setRotaryDamping(0.05);
      sources.add(humerus);
      // move down so shoulder joint at 2.5cm from top
      humerus.setPose(new RigidTransform3d(new Vector3d(0,0,-0.1), AxisAngle.IDENTITY));

   }

}
