package artisynth.models.jawTongue;

import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.properties.PropertyList;
import artisynth.core.modelbase.ControllerBase;

public class MeshBlendController extends ControllerBase {

   PolygonalMesh mesh0;
   PolygonalMesh mesh1;
   PolygonalMesh blendedMesh;

   double blendFactor = 0;
   
   public static PropertyList myProps = new PropertyList (
      MeshBlendController.class, ControllerBase.class);

   static {
      myProps.add (
         "blendFactor", "number of priciple components used", 0, "[0,1]");
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   
   public MeshBlendController (PolygonalMesh mesh0, PolygonalMesh mesh1) {
      assert mesh0.numVertices () == mesh1.numVertices ();
      this.mesh0 = mesh0;
      this.mesh1 = mesh1;
      blendedMesh = new PolygonalMesh (mesh0);
      blendedMesh.setFixed (false);
   }
   
   public PolygonalMesh getBlendedMesh() {
      return blendedMesh;
   }

   Point3d pnt = new Point3d();
   @Override
   public void apply (double t0, double t1) {
      ArrayList<Vertex3d> vtx0 = mesh0.getVertices ();
      ArrayList<Vertex3d> vtx1 = mesh1.getVertices ();
      ArrayList<Vertex3d> blendedVtx = blendedMesh.getVertices ();
      for (int i = 0; i < blendedMesh.numVertices (); i++) {
         pnt.interpolate (vtx0.get (i).pnt, blendFactor, vtx1.get (i).pnt);
         blendedVtx.get (i).setPosition (pnt);
      }
   }

   public double getBlendFactor () {
      return blendFactor;
   }

   public void setBlendFactor (double blendFactor) {
      this.blendFactor = blendFactor;
   }
   
   

}
