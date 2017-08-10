package artisynth.models.registration;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;

public class RegistrationDemo extends RootModel {

   
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      // turn off gravity for registration
      mech.setGravity(0, 0, 0);
      
      // create a FEM around a sphere
      PolygonalMesh sphere = MeshFactory.createOctahedralSphere(1, 3);
      FemModel3d fem = createVoxelFem(null, sphere, 8, 0.5);
      fem.setName("fem");
      FemMeshComp source = fem.addMesh("source", sphere);  // "embed" the mesh in the FEM
      fem.setMaterial(new LinearMaterial(15000, 0.3));      // make it more flexible for registration
      mech.addModel(fem);
      
      // add a target mesh (scaled to have same volume)
      PolygonalMesh target = MeshFactory.createBox(1.0, 1.0, 1.0);
      double s = sphere.computeVolume()/target.computeVolume();
      s = Math.cbrt(s);
      target.scale(s);      
      target.transform(new RigidTransform3d(new Vector3d(2,2,2), AxisAngle.IDENTITY));
      FixedMeshBody fixedTarget = new FixedMeshBody("target", target);
      mech.addMeshBody(fixedTarget);
    
      FemMeshICPController controller = new FemMeshICPController(source, target);
      controller.setName("registration");
      controller.setPressureFunction(new LogNormalPressureFunction(15000, 2, 0.01, false));
      addController(controller);
      
      
      RenderProps.setAlpha(fixedTarget, 0.9);
      RenderProps.setFaceColor(fixedTarget, Color.CYAN);
      RenderProps.setAlpha(source, 1);
      RenderProps.setFaceColor(source, Color.RED);
      
      RenderProps.setVisible(controller, true);
      controller.setForceRenderScale(0.001);
      RenderProps.setLineStyle(controller, LineStyle.SOLID_ARROW);
      RenderProps.setLineColor(controller, Color.GREEN);
      RenderProps.setLineRadius(controller, 0.01);
   }
   
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      
      ControlPanel panel = new ControlPanel("controls");
      // "get" the controller we added in build and add its properties to the control panel
      FemMeshICPController controller = (FemMeshICPController)getControllers().get("registration");
      FemMeshICPController.addControls(panel, controller);
      
      MechModel mech = (MechModel)(models().get("mech"));
      FemModel3d fem = (FemModel3d)(mech.models().get("fem"));
      panel.addWidget("fem material", fem, "material");
      
      addControlPanel(panel);
   }
   
   // creates a voxelized finite element model surrounding a supplied surface mesh
   public static FemModel3d createVoxelFem(
      FemModel3d fem, PolygonalMesh mesh, int minRes, double maxElemWidth) {
      if (fem == null) {
         fem = new FemModel3d();
      }

      OBB obb = new OBB(mesh);
      // fem from OBB
      Vector3d hw = new Vector3d(obb.getHalfWidths());
      double dz = 2 * hw.z / minRes;
      if (dz > maxElemWidth) {
         dz = maxElemWidth;
      }
      // add a little bit
      // hw.add(dz / 8, dz / 8, dz / 8);

      int[] res = new int[3];
      res[0] = (int)(Math.round(2 * hw.x / dz));
      res[1] = (int)(Math.round(2 * hw.y / dz));
      res[2] = (int)(Math.round(2 * hw.z / dz));

      FemFactory.createHexGrid(
         fem, 2 * hw.x, 2 * hw.y, 2 * hw.z, res[0], res[1], res[2]);
      fem.transformGeometry(obb.getTransform());

      double dx, dy;
      dx = 2 * hw.x / res[0];
      dy = 2 * hw.y / res[1];
      dz = 2 * hw.z / res[2];
      double r = 1.05 * Math.sqrt(dx * dx + dy * dy + dz * dz);

      // outside and farther than r
      BVFeatureQuery query = new BVFeatureQuery();

      HashSet<FemNode3d> deleteThese = new HashSet<FemNode3d>();
      for (FemNode3d node : fem.getNodes()) {
         boolean inside =
            query.isInsideOrientedMesh(mesh, node.getPosition(), r);
         if (!inside) {
            deleteThese.add(node);
         }
      }

      // remove elements/nodes
      for (FemNode3d node : deleteThese) {
         // remove element dependencies
         ArrayList<FemElement3d> elems =
            new ArrayList<>(node.getElementDependencies());
         for (FemElement3d elem : elems) {
            fem.removeElement(elem);
         }
         // remove node
         fem.removeNode(node);
      }

      // remove un-necessary nodes
      deleteThese.clear();
      for (FemNode3d node : fem.getNodes()) {
         if (node.getElementDependencies().size() < 1) {
            deleteThese.add(node);
         }
      }
      for (FemNode3d node : deleteThese) {
         fem.removeNode(node);
      }

      return fem;
   }
}
