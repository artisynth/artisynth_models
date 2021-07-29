package artisynth.models.registration.demos;

import java.awt.Color;
import java.io.IOException;

import javax.swing.JSeparator;

import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.core.femmodels.EmbeddedFem;
import artisynth.models.registration.DynamicRegistrationController;
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
 * Registration demo with a sphere surface embedded in a FEM grid
 */
public class EmbeddedRegistrationDemo extends RootModel {
   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      // turn off gravity for registration
      mech.setGravity(0, 0, 0);
      
      // create a FEM around a sphere
      PolygonalMesh sphere = MeshFactory.createOctahedralSphere(1, 3);
      FemModel3d fem = EmbeddedFem.createVoxelizedFem(null, sphere, RigidTransform3d.IDENTITY, 8, 0.5);
      fem.setName("fem");
      fem.setDensity (1000);
      FemMeshComp source = fem.addMesh("source", sphere);  // "embed" the mesh in the FEM
      fem.setMaterial(new LinearMaterial(30000, 0.3));     // make it more flexible for registration
      mech.addModel(fem);
      
      // add a target mesh (scaled to have same volume)
      PolygonalMesh target = MeshFactory.createBox(1.0, 1.0, 1.0);
      target = MeshFactory.subdivide (target, 4);
      double s = sphere.computeVolume()/target.computeVolume();
      s = Math.cbrt(s);
      target.scale(s);      
      target.transform(new RigidTransform3d(new Vector3d(2,2,2), AxisAngle.IDENTITY));
      FixedMeshBody fixedTarget = new FixedMeshBody("target", target);
      mech.addMeshBody(fixedTarget);
    
      ICPMeshCorrespondence icp = new ICPMeshCorrespondence();
      icp.setWeightFunction (new GaussianWeightFunction (1, 0.01, false));
      DynamicRegistrationController reg = new DynamicRegistrationController(mech);
      reg.setName ("registration");
      reg.addRegistrationTarget (source, target, 1.0, icp);
      reg.setForceScaling (1e6);
      addController(reg);
      
      
      RenderProps.setAlpha(fixedTarget, 0.9);
      RenderProps.setFaceColor(fixedTarget, Color.CYAN);
      RenderProps.setAlpha(source, 1);
      RenderProps.setFaceColor(source, Color.RED);
      
   }
   
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      
      ControlPanel panel = new ControlPanel("controls");
      // "get" the controller we added in build and add its properties to the control panel
      DynamicRegistrationController controller = (DynamicRegistrationController)getControllers().get("registration");
      panel.addLabel ("Controller Properties:");
      DynamicRegistrationController.addControls(panel, controller);
      
      panel.addWidget (new JSeparator());
      panel.addLabel ("Model Properties:");
      MechModel mech = (MechModel)(models().get("mech"));
      FemModel3d fem = (FemModel3d)(mech.models().get("fem"));
      panel.addWidget("fem material", fem, "material");
      
      addControlPanel(panel);
      
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }
   }
  
}
