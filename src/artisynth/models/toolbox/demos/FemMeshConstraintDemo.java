package artisynth.models.toolbox.demos;

import java.awt.Color;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemFactory.FemElemType;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.ParticleMeshConstraint;
import artisynth.core.workspace.RootModel;

public class FemMeshConstraintDemo extends RootModel {

   public FemMeshConstraintDemo() {
      super (null);
   }

   public FemMeshConstraintDemo (String name) {
      this();
      setName (name);

      MechModel mech = new MechModel ("mech");
      mech.setGravity (0, 0, -9.8);
      mech.setPointDamping(0.1);

      double wx = 1;
      double wy = 3;
      double rbase = 0.3;
      double wz = 0.5;
      int nsegs = 32;

      PolygonalMesh mesh = MeshFactory.createHollowedBox (
         wx, wy, wz, rbase, nsegs);

      RenderProps.setPointStyle (mech, Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius (mech, 0.02);
      RenderProps.setPointColor (mech, Color.RED);
      
      FemModel3d block = new FemModel3d("block");
      FemFactory.createGrid(block, FemElemType.Hex, 1, 1, 1, 4, 4, 4);

      // point on mesh
      Point3d pos = new Point3d(-0.28839441, 0, -0.058345216);
      FemNode3d node = block.getNode(0);  // get first node
      Vector3d translation = new Vector3d(pos);
      translation.sub(node.getPosition());
      
      RigidTransform3d trans = new RigidTransform3d();
      trans.setTranslation(translation);
      block.transformGeometry(trans);
      
      
      ParticleMeshConstraint c = new ParticleMeshConstraint (node, mesh);

      mech.addModel(block);
      mech.addConstrainer (c);

      addModel (mech);
   }
}
