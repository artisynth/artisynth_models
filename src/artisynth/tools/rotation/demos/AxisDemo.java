package artisynth.tools.rotation.demos;

import java.awt.Color;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JSeparator;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.Renderer;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.TimeBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.tools.rotation.IHAComputer;
import artisynth.tools.rotation.RotationController;
import artisynth.tools.rotation.RotationAxis.AxisValidity;

public class AxisDemo extends RootModel {

   double len = 2;

   MechModel myModel;
   Point3d rotationCenter = new Point3d ();
   Vector3d rotationAxis = new Vector3d ();
   double rotationSpeed = 3;

   public static PropertyList myProps = new PropertyList (
      AxisDemo.class, RootModel.class);

   // properties made separately so I can have smooth sliders
   static {
      myProps.add ("axisX * *", "x-coordinate of rotation axis", 1);
      myProps.add ("axisY * *", "x-coordinate of rotation axis", 0);
      myProps.add ("axisZ * *", "x-coordinate of rotation axis", 0);
      myProps.add ("centerX * *", "x-coordinate of rotation axis", 1);
      myProps.add ("centerY * *", "x-coordinate of rotation axis", 0);
      myProps.add ("centerZ * *", "x-coordinate of rotation axis", 0);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   IHAComputer myRotationComputer;
   RotationController myRotationGenerator;
   RigidBody sphere;

   private Vector3d computedAxis = new Vector3d ();
   private Point3d computedPoint = new Point3d ();
   private RenderProps rotAxisRenderProps = new RenderProps ();
   private RenderProps rotCenterRenderProps = new RenderProps ();
   Color confident = Color.GREEN;
   Color iffy = Color.RED;

   public AxisDemo () {
      super ();
   }

   public AxisDemo (String name) {
      super ();
      setName (name);

      myModel = new MechModel ("rotation_test");
      addModel (myModel);

      rotationAxis.set (1, 0, 1);
      rotationAxis.normalize ();
      rotationCenter.set (0, 1, 0);

      rotAxisRenderProps.setLineRadius (0.01 * len);
      rotAxisRenderProps.setLineStyle (LineStyle.CYLINDER);
      rotCenterRenderProps.setPointRadius (0.05 * len);
      rotCenterRenderProps.setPointStyle (PointStyle.SPHERE);

      buildModel ();
      addControllers ();
   }

   public void attach (DriverInterface driver) {
      super.attach (driver);
      if (getControlPanels ().size () == 0) {
         addPanel (driver);
      }
   }

   @Override
   public StepAdjustment advance (double t0, double t1, int flags) {

      StepAdjustment adj = super.advance (t0, t1, flags);
      AxisValidity valid =
         myRotationComputer.getRotationAxis (computedAxis, computedPoint);
      computedAxis.normalize ();

      // colours the axis/center based on whether the algorithm deems to
      // estimates to be valid
      if (valid == AxisValidity.AXIS_VALID) {
         rotAxisRenderProps.setLineColor (confident);
         rotCenterRenderProps.setPointColor (iffy);
      }
      else if (valid == AxisValidity.CENTER_AND_AXIS_VALID) {
         rotAxisRenderProps.setLineColor (confident);
         rotCenterRenderProps.setPointColor (confident);
      }
      else {
         rotAxisRenderProps.setLineColor (iffy);
         rotCenterRenderProps.setPointColor (iffy);
      }
      
      if (TimeBase.modulo (t1, 1) == 0) {
         System.out
            .println ("Center of rotation: " + computedPoint.toString ());
         System.out.println ("Rotation axis: " + computedAxis.toString ());
         System.out.println("Validity: " + valid);
      }

      return adj;
   }

   public void buildModel () {

      sphere = new RigidBody ("sphere");
      PolygonalMesh sphere_mesh = MeshFactory.createQuadSphere (len, 20);
      sphere_mesh.scale (1, 1, 1.5);
      sphere.setMesh (sphere_mesh, null);

      RenderProps.setFaceStyle (sphere, FaceStyle.NONE);
      RenderProps.setEdgeColor (sphere, Color.BLUE);
      RenderProps.setDrawEdges (sphere, true);
      RenderProps.setLineColor (sphere, Color.BLUE);
      sphere.setDynamic (false);
      myModel.addRigidBody (sphere);

   }

   public void addControllers () {

      // computes rotation axis
      myRotationComputer = new IHAComputer (sphere);
      myRotationComputer.setModel (myModel);

      // causes the object to spin in this demo
      myRotationGenerator =
         new RotationController (sphere, rotationAxis, rotationCenter);
      myRotationGenerator.setModel (myModel);
      myRotationGenerator.setSpeed (rotationSpeed);

      addController (myRotationGenerator);
      addMonitor (myRotationComputer);
   }

   // render a line for the computed axis and point for its estimated center
   @Override
   public void render (Renderer renderer, int flags) {

      Point3d p0 = new Point3d (computedAxis);
      p0.scale (2 * len);
      p0.add (computedPoint);
      Point3d p1 = new Point3d (computedAxis);
      p1.scale (-2 * len);
      p1.add (computedPoint);

      float[] coords0 = new float[] { (float)p0.x, (float)p0.y, (float)p0.z };
      float[] coords1 = new float[] { (float)p1.x, (float)p1.y, (float)p1.z };
      float[] coords =
         new float[] { (float)computedPoint.x, (float)computedPoint.y,
                      (float)computedPoint.z };

      renderer.drawLine (rotAxisRenderProps, coords0, coords1, null, true, false);
      renderer.drawPoint (rotCenterRenderProps, coords, false);
   }

   protected ControlPanel panel;

   public void addPanel (DriverInterface driver) {

      panel = new ControlPanel ("Rotation Parameter Control", "");
      panel.addWidget (new JLabel ("Rotation Axis"));
      panel.addWidget ("   x", this, "axisX", 0.0, 1.0);
      panel.addWidget ("   y", this, "axisY", 0.0, 1.0);
      panel.addWidget ("   z", this, "axisZ", 0.0, 1.0);
      panel.addWidget (new JSeparator ());
      panel.addWidget (new JLabel ("Center of Rotation"));
      panel.addWidget ("   x", this, "centerX", -len * 5, len * 5);
      panel.addWidget ("   y", this, "centerY", -len * 5, len * 5);
      panel.addWidget ("   z", this, "centerZ", -len * 5, len * 5);
      panel.addWidget (new JSeparator ());
      addControlPanel (panel);
   }

   // property setters/getters
   // done individually so I can set sliders for continuous motion
   public void setAxisX (double x) {
      rotationAxis.x = x;
      rotationAxis.normalize ();
   }

   public double getAxisX () {
      return rotationAxis.x;
   }

   public void setAxisY (double y) {
      rotationAxis.y = y;
      rotationAxis.normalize ();
   }

   public double getAxisY () {
      return rotationAxis.y;
   }

   public void setAxisZ (double z) {
      rotationAxis.z = z;
      rotationAxis.normalize ();
   }

   public double getAxisZ () {
      return rotationAxis.z;
   }

   public void setCenterX (double x) {
      rotationCenter.x = x;
   }

   public double getCenterX () {
      return rotationCenter.x;
   }

   public void setCenterY (double y) {
      rotationCenter.y = y;
   }

   public double getCenterY () {
      return rotationCenter.y;
   }

   public void setCenterZ (double z) {
      rotationCenter.z = z;
   }

   public double getCenterZ () {
      return rotationCenter.z;
   }
}
