package artisynth.tools.rotation;

import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector3d;
import maspack.spatialmotion.Twist;

public class InstantaneousHelicalAxis extends RotationAxis {

   public InstantaneousHelicalAxis () {
      super();
   }

   public InstantaneousHelicalAxis (Twist velocity, Twist acceleration,
      Point3d position) {
      this();
      vel.set(velocity);
      acc.set(acceleration);
      pnt.set(position);
   }

   public InstantaneousHelicalAxis (Twist vel0, Twist vel1, Point3d position,
      double dt) {
      this();
      vel.set(vel1);
      acc.set(vel1);
      acc.sub(vel0);
      acc.scale(1.0 / dt);
      pnt.set(position);

   }

   public void compute(Twist velocity, Vector3d p0) {
      compute(velocity.w, velocity.v, null, null, p0);
   }
   
   public void compute(Twist velocity, Twist acceleration, Vector3d p0) {
      compute(velocity.w, velocity.v, acceleration.w, acceleration.v, p0);
   }

   public void compute(Vector3d w, Vector3d v0, Vector3d p0) {
      compute(w, v0, null, null, p0);
   }
   
   public void compute(Vector3d w, Vector3d v0, Vector3d alpha, Vector3d a0,
      Vector3d p0) {

      Point3d p_prev = new Point3d(this.pnt);
      Vector3d v_prev = new Vector3d(this.dir);

      // set w and alpha
      this.vel.w.set(w);
      this.acc.w.set(alpha);

      // not enough motion to compute,
      if (w.norm() == 0) {
         validity = AxisValidity.INVALID;
         return;
      }

      // instantaneous helical axis direction
      this.dir.set(w);
      this.dir.normalize();

      Matrix3d W = crossProductMatrix(w); 
      
      // compute a point on the line
      SVDecomposition3d svd = new SVDecomposition3d(W);
      
      // pseudo-inverse for solving for a point on the line
      Matrix3d Winv = new Matrix3d();
      svd.pseudoInverse(Winv);

      // IHA linear velocity
      double s = w.dot(v0) / w.dot(w);
      this.vel.v.set(w);
      this.vel.v.scale(s);

      // IHA point
      Vector3d b = new Vector3d();
      b.mul(W, p0);
      b.add(-v0.x, -v0.y, -v0.z);
      b.add(-this.vel.v.x, -this.vel.v.y, -this.vel.v.z);
      pnt.mul(Winv, b); // single solution for point

      // instantaneous axis is okay at this point
      Vector3d alphaxw = new Vector3d(alpha);
      alphaxw.cross(w);

      if (alphaxw.norm() == 0) {
         // project previous point
         pnt.set(this.project(p_prev));
         if (a0 != null && alpha != null) {
            this.acc.v.set(getAcceleration(a0, alpha, w, p0, pnt));
         }
         validity = AxisValidity.AXIS_VALID;
         return;
      }

      // Find point by minimizing acceleration
      // b = a0 + alpha x (r1-r0) + w x ( w x (r1-r0) )      
      if (alpha == null) {
         // more stable?? estimate rotation center by comparing to previous line
         closestPointBetweenLines (v_prev, p_prev, w, pnt, p_prev, pnt, 0);
      } else {
         
         Vector3d r = new Vector3d();
         r.set(pnt);
         r.sub(p0);
         b.set(r);
         b.cross(w);
         b.cross(w);
         r.cross(alpha);
         b.sub(r);
         b.add(a0);

         double t = -b.dot(alphaxw) / alphaxw.normSquared();
         r.set(w);
         r.scale(t);
         pnt.add(r);         
      }

      validity = AxisValidity.CENTER_AND_AXIS_VALID;
   }

   public AxisValidity getHelixVelocity(Vector3d v) {
      return getLinearVelocity(v);
   }

}
