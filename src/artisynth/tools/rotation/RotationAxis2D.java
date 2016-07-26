package artisynth.tools.rotation;

import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector3d;
import maspack.spatialmotion.Twist;

public abstract class RotationAxis2D extends RotationAxis {

   Point3d pOrig;
   Vector3d n;
   MatrixNd Z; // constaint space, p = pOrig + Zx, x in R2
   
   public RotationAxis2D () {
      super();
      setNormal(new Vector3d(1,0,0));      
      pOrig = new Point3d();
      updateZ();
   }
   
   public RotationAxis2D (Vector3d normal, Point3d point) {
      super();
      n = new Vector3d(normal);
      Z = new MatrixNd(3,2);
      setNormal(normal);
      pnt.set(point);
      pOrig = new Point3d(point);
   }
   
   private void updateZ() {
      
      // set Z to nullspace of [n / 0 / 0]
      Matrix3d M = new Matrix3d();
      M.setRow(0, n);
      SVDecomposition3d svd = new SVDecomposition3d(M);
      Matrix3d V = svd.getV();
      for (int i=0; i<2; i++) {
         for (int j=0; j<3; j++) {
            Z.set(j, i, V.get(j, i+1));
         }
      }
      // [n Z] should now form a complete basis for R3
      
   }
   
   public void compute(Twist velocity, Vector3d p0) {
      compute(velocity.w, velocity.v, p0);
   }
   
   
   public abstract void compute(Vector3d w, Vector3d v0, Vector3d p0);
   
   public void setPlane(Vector3d normal, Point3d point) {
      setNormal(normal);
      pOrig.set(point);
   }
   
   public void getPlane(Vector3d normal, Point3d point) {
      normal.set(n);
      point.set(pOrig);
   }
   
   public void setPlanePoint(Point3d p) {
      pOrig.set(p);
   }
   
   private void setNormal(Vector3d normal) {
      dir.set(normal);
      n.set(normal);
      updateZ();
   }
   
   public abstract Object clone();
   
}
