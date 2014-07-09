package artisynth.models.toolbox.demos;

import java.awt.Color;
import java.io.IOException;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.GLRenderer;
import maspack.render.RenderProps;
import maspack.render.RenderProps.LineStyle;
import maspack.render.RenderProps.PointStyle;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.TimeBase;
import artisynth.demos.mech.SphericalJointDemo;
import artisynth.models.toolbox.rotation.IHAComputer;
import artisynth.models.toolbox.rotation.RotationAxis.AxisValidity;

public class AxisSphericalJointDemo extends SphericalJointDemo {

   RigidBody myRotatingBody = null;
   IHAComputer myRotationComputer;
   private Vector3d computedAxis = new Vector3d ();
   private Point3d computedPoint = new Point3d ();
   private Particle centerOfRotation;
   private RenderProps rotCenterRenderProps;
   private RenderProps rotAxisRenderProps;
   private Color iffyColour = Color.RED;
   private Color confidentColour = Color.GREEN;

   public AxisSphericalJointDemo () {
      super ();
   }

   public AxisSphericalJointDemo (String name) throws IOException {
      super (name);

      // get hand
      myRotatingBody = myHand;
      RenderProps.setAlpha (myMechMod.rigidBodyConnectors ().get (0), 0.5);
      addExtras ();
      addControllers ();
   }

   public void addExtras () {

      rotCenterRenderProps = new RenderProps ();
      rotAxisRenderProps = new RenderProps ();

      rotCenterRenderProps.setPointRadius (0.01);
      rotCenterRenderProps.setPointColor (iffyColour);
      rotCenterRenderProps.setPointStyle (PointStyle.SPHERE);

      rotAxisRenderProps.setLineRadius (0.005);
      rotAxisRenderProps.setLineStyle (LineStyle.CYLINDER);

      centerOfRotation = new Particle (0);
      centerOfRotation.setDynamic (false);

      centerOfRotation.setPosition (myRotatingBody.getCenterOfMass ());
      centerOfRotation.setRenderProps (rotCenterRenderProps);

      myMechMod.addParticle (centerOfRotation);
   }

   public void addControllers () {

      // computes rotation axis
      myRotationComputer = new IHAComputer (myRotatingBody);
      myRotationComputer.setModel (myMechMod);
      addMonitor (myRotationComputer);
   }

   @Override
   public StepAdjustment advance (double t0, double t1, int flags) {

      StepAdjustment adj = super.advance (t0, t1, flags);
      AxisValidity valid =
         myRotationComputer.getRotationAxis (computedAxis, computedPoint);
      computedAxis.normalize ();
      centerOfRotation.setPosition (computedPoint);

      // colours the axis/center based on whether the algorithm deems to
      // estimates to be valid
      if (valid == AxisValidity.AXIS_VALID) {
         rotAxisRenderProps.setLineColor (confidentColour);
         rotCenterRenderProps.setPointColor (iffyColour);
      }
      else if (valid == AxisValidity.CENTER_AND_AXIS_VALID) {
         rotAxisRenderProps.setLineColor (confidentColour);
         rotCenterRenderProps.setPointColor (confidentColour);
      }
      else {
         rotAxisRenderProps.setLineColor (iffyColour);
         rotCenterRenderProps.setPointColor (iffyColour);
      }
      
      if (TimeBase.modulo (t1, 1) == 0) {
         System.out
            .println ("Center of rotation: " + computedPoint.toString ());
         System.out.println ("Rotation axis: " + computedAxis.toString ());
         System.out.println("Validity: " + valid);
      }

      return adj;
   }

   // render a line for the computed axis and point for its estimated center
   @Override
   public void render (GLRenderer renderer, int flags) {
      super.render (renderer, flags);

      Point3d p0 = new Point3d (computedAxis);
      p0.scale (0.2);
      p0.add (computedPoint);
      Point3d p1 = new Point3d (computedAxis);
      p1.scale (-0.2);
      p1.add (computedPoint);

      float[] coords0 = new float[] { (float)p0.x, (float)p0.y, (float)p0.z };
      float[] coords1 = new float[] { (float)p1.x, (float)p1.y, (float)p1.z };
      float[] coords =
         new float[] { (float)computedPoint.x, (float)computedPoint.y,
                      (float)computedPoint.z };

      renderer.drawLine (rotAxisRenderProps, coords0, coords1, true, false);
      renderer.drawPoint (rotCenterRenderProps, coords, false);
   }

}
