package artisynth.tools.rotation;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import artisynth.core.mechmodels.Frame;
import artisynth.core.modelbase.MonitorBase;
import artisynth.tools.rotation.RotationAxis.AxisValidity;

public class PlaneProjectionRotationComputer extends MonitorBase implements RotationAxisComputer {

   public double eps = 1e-10;

   public PlaneProjectionRotationAxis RA;   
   boolean follow; // not yet implemented

   Frame myFrame = null;

   public PlaneProjectionRotationComputer (Frame frame, Point3d pnt, Vector3d normal) {
      myFrame = frame;
      
      // initialize plane
      RA = new PlaneProjectionRotationAxis(normal, pnt);
      
   }
   
   public Frame getFrame() {
      return myFrame;
   }

   public void apply(double t0, double t1) {

      Vector3d w = myFrame.getVelocity().w;
      
      if (w.norm ()<eps) {
         RA.invalidate();
         return;
      }
      
      Vector3d v0 = myFrame.getVelocity().v;
      Vector3d p0 = myFrame.getPosition();
      
      Point3d icr_prev = new Point3d(); 
      RA.getICR(icr_prev);
      RA.compute (w, v0, p0);

   }

   public void setThreshold(double epsilon) {
      eps = epsilon;
   }

   public AxisValidity getRotationAxis(Vector3d axis, Point3d pnt) {
      
      RA.getDirection (axis);
      return RA.getICR (pnt);

   }
   
   public RotationAxis getRotationAxis() {
      return RA;
   }

}
