package artisynth.tools.rotation;

import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.spatialmotion.Twist;
import artisynth.core.mechmodels.Frame;
import artisynth.core.modelbase.ControllerBase;

public class RotationController extends ControllerBase {
      private Frame myFrame;
      private Point3d rotCenter;
      private Vector3d rotAxis;
      private double rotationSpeed;
      
      public RotationController(Frame frame, Vector3d axis, Point3d center) {
	 myFrame = frame;
	 rotCenter = center;
	 rotAxis = axis;
      }
      public void apply(double t0, double t1) {
	 // rotate frame about a point
	 
	 Vector3d omega =new Vector3d(rotAxis);
	 omega.normalize();
	 omega.scale(rotationSpeed);
	 Vector3d v = new Vector3d(omega);
	 
	 Vector3d r = new Vector3d(myFrame.getPosition());
	 r.sub(rotCenter);
	 v.cross(r);
	 //v.scale(-1);
	 Twist bodyTwist = new Twist(v, omega);
	 myFrame.setVelocity(bodyTwist);
	 
	 double dt = t1-t0;
	 double theta = rotationSpeed*dt;
	 AxisAngle rot = new AxisAngle(rotAxis, theta);
	 Vector3d trans = new Vector3d(rotCenter);
	 
   	 // rotates about supplied point
	 trans.scale(-1);
   	 RigidTransform3d rt = new RigidTransform3d();
   	 rt.setTranslation(trans);
   	 myFrame.transformGeometry(rt);
   	 trans.scale(-1.0);
   	 rt.setTranslation(trans);
   	 rt.setRotation(rot);
   	 myFrame.transformGeometry(rt);
   	 
      }
      
      public void setSpeed(double omega) {
	 rotationSpeed = omega;
      }
   }