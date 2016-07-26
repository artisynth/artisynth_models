package artisynth.tools.rotation;

import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.spatialmotion.Twist;

/**
 * Computes a 2D centre of rotation by projecting all velocities
 * onto a plane and then finding location with minimum speed
 * 
 * @author antonio
 *
 */
public class PlaneProjectionRotationAxis extends RotationAxis2D {
  
   public PlaneProjectionRotationAxis() {
      super();
   }
   
   public PlaneProjectionRotationAxis(Vector3d normal, Point3d point) {
      super(normal, point);
   }
   
   public void compute(Vector3d w, Vector3d v0, Vector3d p0) {
      
      // b = vo + w x (pOrig-p0)   
      Vector3d b = new Vector3d();
      b.set(p0);
      b.sub(pOrig);  // any point on the plane, originally used pOrig but that sometimes gets large
      b.cross(w);
      b.add(v0);
      
      // Wv is equivalent to W x v
      Matrix3d W = crossProductMatrix(w);
      MatrixNd WZ = new MatrixNd(W);
      WZ.mul(Z);
      
      // Project to b plane: b := b - (n.b)n
      b.scaledAdd(-n.dot(b), n);
      this.vel.w.set(w);
      
      // G special matrix s.t. [trans(n)WZv]n = Gv 
      // [alpha beta] = trans(n)WZ
      // G = [alpha*n beta*n]
      double alpha = 0;
      double beta = 0;
      for (int i=0; i<3; i++) {
         alpha += n.get(i) * WZ.get(i, 0);
         beta += n.get(i) * WZ.get(i, 1);
      }
      MatrixNd G = new MatrixNd(3,2);
      for (int i=0; i<3; i++) {
         G.set(i,0,n.get(i)*alpha);
         G.set(i,1,n.get(i)*beta);
      }
      
      //A = trans(WZ)WZ + trans(WZ)G + trans(G)WZ + trans(G)G
      Matrix2d A = new Matrix2d();
      MatrixNd T = new MatrixNd();
      T.mulTransposeLeft(WZ, WZ);
      A.add(T);
      T.mulTransposeLeft(WZ, G);
      A.add(T);
      T.mulTransposeLeft(G, WZ);
      A.add(T);
      T.mulTransposeLeft(G, G);
      A.add(T);
      
      // invert, exit if failed
      boolean success = A.invert();
      if (!success) {
         // fallback to previous point
         setValidity(AxisValidity.INVALID);         
         vel.v.set(getVelocity(v0, w, p0, pnt));
         return;
      }
      
      //c = (trans(G)-trans(WZ))b
      Vector2d c = new Vector2d();
      G.transpose();
      WZ.transpose();
      WZ.negate();
      G.add(WZ);
      for (int i=0; i<3; i++) {
         c.x += G.get(0, i)*b.get(i);
         c.y += G.get(1, i)*b.get(i);
      }
      
      // solve Ax = c (A previously inverted)
      A.mul(c);
     
      Point3d pNew = new Point3d();
      for (int i=0; i<3; i++) {
         pNew.set(i, Z.get(i, 0)*c.x + Z.get(i, 1)*c.y);
      }
      pNew.add(pOrig);
      
      if (Double.isNaN(pNew.get(0))) {
         System.out.println("broken");
      }
      
      pnt.set(p0);
      vel.v.set(v0);
      setPoint(pNew);
      
      setValidity(AxisValidity.CENTER_AND_AXIS_VALID);
      
   }
   
   public Object clone() {
      return new PlaneProjectionRotationAxis(this.n, this.pnt);
   }

}
