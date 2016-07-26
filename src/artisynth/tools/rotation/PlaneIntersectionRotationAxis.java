package artisynth.tools.rotation;

import maspack.matrix.Matrix2d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

/**
 * Examines the motion of a plane in 3D to compute an instantaneous center
 * of rotation.  Essentially, this is the ICR problem restricted such that
 * the point must lie on specified plane (e.g. mid-sagittal plane).
 * 
 * @author antonio
 *
 */
public class PlaneIntersectionRotationAxis extends RotationAxis2D {
   
   public PlaneIntersectionRotationAxis() {
      super();
   }
   
   public PlaneIntersectionRotationAxis(Vector3d normal, Point3d point) {
      super(normal, point);
   }
   
     public void compute(Vector3d w, Vector3d v0, Vector3d p0) {
      
      Vector3d b = new Vector3d();
      
      b.set(p0);
      b.sub(pOrig);  // any point on the plane, originally used pOrig but that sometimes gets large
      b.cross(w);
      b.add(v0);
      b.scale(-1.0);
      
      this.vel.w.set(w);
      dir.set(w);
      dir.normalize();
      
      Matrix3d W = crossProductMatrix(w);
      MatrixNd WZ = new MatrixNd(W);
      WZ.mul(Z);
      
      // A = WZ
      // Q = WztWz
      // QtQ = (WztWz)t(QztWz) = T?
      
      MatrixNd T = new MatrixNd();
      T.mulTransposeLeft(WZ, WZ);
      Matrix2d WZtWZ = new Matrix2d();
      WZtWZ.add(T);
      
      Matrix2d Q = new Matrix2d();
      T.mulTransposeLeft(T,T);      
      Q.add(T);
      
      
      boolean success = Q.invert();
      if (!success) {
         // fallback to previous point
         setValidity(AxisValidity.INVALID);         
         vel.v.set(getVelocity(v0, w, p0, pnt));
         return;
      }
      
      MatrixNd R = new MatrixNd();
      R.mul(WZ, WZtWZ);
      R.transpose();
      R.mul(Q, R);
      
      Vector2d c = new Vector2d();
      for (int i=0; i<3; i++) {
         c.x += R.get(0, i)*b.get(i);
         c.y += R.get(1, i)*b.get(i);
      }
      
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

   @Override
   public Object clone() {
      return new PlaneIntersectionRotationAxis(n, pnt);
   }   

}
