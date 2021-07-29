package artisynth.models.registration.correspondences;

import maspack.geometry.Feature;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

/**
 * Uses direct one-to-one correspondences
 */
public class DirectMeshCorrespondence extends MeshCorrespondenceComputer {
   
   // model/target
   MeshBase target;
   MeshBase model;
  
   private static class VertexEntry {
      @SuppressWarnings("unused")
      Feature nearfeature; // nearest feature
      Point3d nearPnt;     // nearest point
      double w;            // weight 
      
      VertexEntry() {
         nearfeature = null;
         nearPnt = new Point3d();
         w = 1.0;
      }
   }
   
   VertexEntry[] vinfo;    // force information
   
   public DirectMeshCorrespondence() {
   }
   
   @Override
   public void initialize(MeshBase model, MeshBase target) {
      
      this.target = target;
      this.model = model;
      
      this.vinfo = new VertexEntry[model.numVertices ()];
      for (int i=0; i<vinfo.length; ++i) {
         vinfo[i] = new VertexEntry();
      }
    
      // initialize distance/weights
      updateDistancesAndWeights ();      
   }
   
   @Override
   public double getCorrespondence (Point3d x, Vertex3d mvtx) {
      int m = mvtx.getIndex ();
      if (x != null) {
         x.set (vinfo[m].nearPnt);
      }
      return vinfo[m].w / model.numVertices ();
   }
   
   @Override
   public void apply(double t0, double t1) {
      // update distances
      updateDistancesAndWeights();
   }
  
   /**
    * Computes distances to target and updates vertex areas on source
    */
   public void updateDistancesAndWeights() {
      // maybe update tree
      
      // loop through each model vertex
      for (Vertex3d vtx : model.getVertices ()) {
         int vidx = vtx.getIndex ();
         
         VertexEntry dentry = vinfo[vidx];
         
         // use corresponding vertex if it exists
         if (vidx < target.numVertices ()) {
            Vertex3d tvtx = target.getVertex (vidx);
            dentry.nearfeature = tvtx;
            tvtx.getWorldPoint (dentry.nearPnt);
            dentry.w = 1.0;
         } else {
            dentry.w = 0;
         }
      }
   }
 
}
