package artisynth.tools.rotation;

import maspack.matrix.Line3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.spatialmotion.Twist;

public abstract class RotationAxis extends Line3d {
   
   protected Twist vel;   // velocity
   protected Twist acc;   // acceleration
   protected AxisValidity validity;
   
   public enum AxisValidity {
      INVALID, AXIS_VALID, CENTER_AND_AXIS_VALID
   };
   
   
   public RotationAxis() {
      vel = new Twist();
      acc = new Twist();
      validity = AxisValidity.INVALID;
   }
   
   // matrix for performing cross products
   protected static Matrix3d crossProductMatrix(Vector3d vec) {
      return new Matrix3d(0, -vec.z, vec.y, vec.z, 0, -vec.x, -vec.y, vec.x, 0);
   }
   
   public static Vector3d getAcceleration( Vector3d a0, Vector3d alpha, Vector3d w, Vector3d p0, Vector3d p ) {
      Vector3d a = new Vector3d();
      
      Vector3d dp = new Vector3d(p);
      dp.sub (p0);
      
      a.set (alpha);
      a.cross (dp);
      
      dp.cross(w);
      dp.cross(w);
      a.add (dp);
      a.add (a0);
      
      return a;
   }
   
   public static Vector3d getVelocity( Vector3d v0, Vector3d w, Vector3d p0, Vector3d p ) {
      
      Vector3d v = new Vector3d();
      v.set (p0);
      v.sub (p);
      v.cross (w);
      v.add (v0);
      
      return v;
   }
   
   public void setValidity(AxisValidity val) {
      validity = val;
   }
   
   public void invalidate() {
      validity = AxisValidity.INVALID;
   }
   
   public AxisValidity getAngularVelocity(Vector3d w) {
      w.set (this.vel.w);
      return validity;
   }
   
   public AxisValidity getLinearVelocity(Vector3d v) {
      v.set ( this.vel.v);
      return validity;
   }
   
   public AxisValidity getLinearAcceleration(Vector3d a) {
      a.set(this.acc.v);
      return validity;
   }
   
   public AxisValidity getAngularAcceleration(Vector3d alpha) {
      alpha.set(this.acc.w);
      return validity;
   }
   
   public AxisValidity getICR(Point3d p) {
      p.set (pnt);
      return validity;
   }
   
   // updates linear velocity and acceleration
   @Override
   public void setPoint(Point3d p) {
      Vector3d newVel = getVelocity(this.vel.v, this.vel.w, this.pnt, p);
      Vector3d newAcc = getAcceleration(this.acc.v, this.acc.w, this.vel.w, this.pnt, p);
      pnt.set(p);
      vel.v.set(newVel);
      acc.v.set(newAcc);
      
   }
   
   public void projectCenter(Point3d center) {
      // project onto line
      Point3d p = project(center);
      setPoint(p);
   }
   
}
