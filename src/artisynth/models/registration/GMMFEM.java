package artisynth.models.registration;

import java.util.ArrayList;
import java.util.HashSet;

import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Boundable;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class GMMFEM {

   public static void computeP(PolygonalMesh source, PolygonalMesh target, 
      double sigma, double w, double[] Ps, double[] Px) {
      
      // for each point on target, find model points within the range of 2*sigma
      BVTree tree = source.getBVTree();
      double r = 2*sigma;
      Point3d c = new Point3d();
      for (Vertex3d tvtx : target.getVertices()) {
         tvtx.getWorldPoint(c);
         ArrayList<BVNode> nodes = new ArrayList<>();
         tree.intersectSphere(nodes, c, r);
         
         HashSet<Vertex3d> sourceVtxs = new HashSet<>();
         
         // Collect all vertices
         for (BVNode node : nodes) {
            for (Boundable b : node.getElements()) {
               Face f = (Face)b;
               HalfEdge he0 = f.firstHalfEdge();
               HalfEdge he = he0;
               do {
                  sourceVtxs.add(he.getHead());
               } while (he != he0);
            }
         }
         
         // Loop through all source vertices
         for (Vertex3d svtx : sourceVtxs) {
            
         }
         
         
         
      }
      
   }
   
}
