package artisynth.tools.rotation;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import artisynth.core.mechmodels.Frame;
import artisynth.core.modelbase.Monitor;
import artisynth.tools.rotation.RotationAxis.AxisValidity;

public interface RotationAxisComputer extends Monitor {
   
   public void setThreshold(double epsilon);
   public AxisValidity getRotationAxis(Vector3d axis, Point3d pnt);
   public RotationAxis getRotationAxis();
   public Frame getFrame();
   

}
