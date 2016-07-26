package artisynth.tools.rotation;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.spatialmotion.Twist;
import artisynth.core.mechmodels.Frame;
import artisynth.core.modelbase.MonitorBase;
import artisynth.tools.rotation.RotationAxis.AxisValidity;

public class IHAComputer extends MonitorBase implements RotationAxisComputer {

   public double eps = 1e-10;
   InstantaneousHelicalAxis IHA;
   Twist vPrev;

   Frame myFrame = null;

   public IHAComputer (Frame frame) {
      myFrame = frame;
      IHA = new InstantaneousHelicalAxis();
      vPrev = new Twist();
   }
   
   public Frame getFrame() {
      return myFrame;
   }

   public void apply(double t0, double t1) {

      Vector3d w = myFrame.getVelocity().w;
      
      if (w.norm ()<eps) {
         IHA.invalidate();
         vPrev.set (myFrame.getVelocity());
         return;
      }
      
      Vector3d v0 = myFrame.getVelocity().v;
      Twist acc = new Twist(myFrame.getVelocity());
      acc.sub(vPrev);
      acc.scale (1.0/(t1-t0));
      
      Vector3d alpha = acc.w;
      Vector3d a0 = acc.v;
      Vector3d p0 = myFrame.getPosition();
      
      Point3d icr_prev = new Point3d(); 
      IHA.getICR(icr_prev);
      
      vPrev.set (myFrame.getVelocity());
      
      IHA.compute (w, v0, alpha, a0, p0);
      
      
      Vector3d tmp = new Vector3d(alpha);
      tmp.cross (w);
      if (tmp.norm () < eps) {
         IHA.projectCenter (icr_prev);
         IHA.setValidity (AxisValidity.AXIS_VALID);
      }

   }

   public void setThreshold(double epsilon) {
      eps = epsilon;
   }

   public AxisValidity getRotationAxis(Vector3d axis, Point3d pnt) {
      
      IHA.getDirection (axis);
      return IHA.getICR (pnt);

   }
   
   public RotationAxis getRotationAxis() {
      return IHA;
   }

}
