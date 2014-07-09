package artisynth.models.jawTongue;

import maspack.geometry.MeshBase;
import maspack.matrix.VectorNd;
import maspack.render.color.ColorMapBase;
import maspack.render.color.HueColorMap;
import artisynth.core.femmodels.SkinMesh;

public class AirwaySkin extends SkinMesh {

   public AirwaySkin () {
   }

   public AirwaySkin (MeshBase mesh) {
      super (mesh);
   }

   protected static ColorMapBase defaultColorMap =  new HueColorMap(2.0/3, 0);
   protected ColorMapBase myColorMap = defaultColorMap.copy();
   
   public void colorWeights(int idx) {
      MeshBase mesh = getMesh();
      /* XXX - waiting for vertex coloring to be added to MeshBase*/
//      mesh.setVertexColoring(true);
      float[] rgb = new float[3];
      double a = mesh.getRenderProps ().getAlpha ();
      for (int i = 0; i < mesh.getNumVertices (); i++) {
         double w = getAttachment (i).getWeight (idx);
         myColorMap.getRGB (w, rgb);
         mesh.getVertex (i).setColor (rgb[0], rgb[1], rgb[2], a);
      }
   }
   
   public VectorNd getWeights(int idx) {
      int n = getMesh().getNumVertices ();
      VectorNd w = new VectorNd (n);
      for (int i = 0; i < n; i++) {
         w.set (i, getAttachment (i).getWeight (idx));
      }
      return w;
   }

}
