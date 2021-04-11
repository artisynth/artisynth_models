package artisynth.models.registration.test;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.TargetFrame;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.registration.DynamicRegistrationController;
import maspack.geometry.MeshFactory;
import maspack.geometry.MeshUtilities;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;
import maspack.widgets.VectorMultiField;

public class FrameRegistrationTest extends RootModel {
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      MechModel mech = new MechModel("mech");
      mech.setFrameDamping (0.1);
      mech.setRotaryDamping (0.0);
      mech.setGravity (0, 0, 1e-10);
      // mech.setIntegrator (Integrator.StaticIncrementalStep);
      mech.setStaticIncrements (1);
      addModel(mech);
      
      Vector3d vsize = new Vector3d(0.05, 0.05, 0.02);
      RigidBody model = new RigidBody("model");
      PolygonalMesh vmesh;
      try {
         vmesh = new PolygonalMesh(ArtisynthPath.getSrcRelativeFile (this, "data/T10.obj"));
         MeshUtilities.sqrt3Subdivide (vmesh, 2);
         vmesh.transform (new RigidTransform3d(Vector3d.ZERO, AxisAngle.ROT_X_90));
      }
      catch (IOException e) {
         vmesh = MeshFactory.subdivide (MeshFactory.createBox (vsize.x, vsize.y, vsize.z), 2);
      }  
      model.addMesh (vmesh, true, true);
      model.setDensity (1000);
      mech.addRigidBody (model);
      
      RigidBody target = new TargetFrame ();
      target.setName ("target");
      target.addMesh (model.getSurfaceMesh().clone ());
      target.setPose (model.getPose ());
      target.setDynamic (false);
      addRenderable (target);
      
      RigidTransform3d trans = new RigidTransform3d(Vector3d.ZERO, new AxisAngle(0,0,1, Math.PI/3));
      model.transformPose (trans);
      
      DynamicRegistrationController rcon = new DynamicRegistrationController (mech);
      rcon.addRegistrationTarget (model, target, 1.0, 1.0);
      rcon.setName ("registration");
      
      addController (rcon);
      
      RenderProps.setShading (model, Shading.SMOOTH);
      Color boneColor = new Color( 227, 218, 201);
      RenderProps.setFaceColor (model, boneColor);
      
      RenderProps.setDrawEdges (target, true);
      RenderProps.setFaceStyle (target, FaceStyle.NONE);
      RenderProps.setFaceColor (target, Color.CYAN);
      RenderProps.setLineColor (target, Color.CYAN);
      
      setWeightFactors (1, 0.001);
      setForceScaling(10000);
   }
   
   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);
      
      addControlPanel (createControlPanel());
//      try {
//         MechSystemSolver.setLogWriter (new PrintWriter(ArtisynthPath.getSrcRelativeFile (this, "data/frame_test.log")));
//      } catch (FileNotFoundException e) {
//         e.printStackTrace();
//      }
   }
   
   @Override
   public void detach (DriverInterface driver) {
      super.detach (driver);
      MechSystemSolver.setLogWriter (null);
   }
  
   private static final double EPS = 1e-13;
   private static final double INF = Double.POSITIVE_INFINITY;
   
   /**
    * Recursively estimate radius of frame
    * @param f frame
    * @return estimated radius
    */
   private static double estimateFrameRadius(Frame f) {
      Vector3d vmin = new Vector3d(INF, INF, INF);
      Vector3d vmax = new Vector3d(-INF, -INF, -INF);
      
      ModelComponent c = f;
      double r = 0;
      do {
         if (c instanceof RenderableComponent ) {
            RenderableComponent rc = (RenderableComponent)c;
            rc.updateBounds (vmin, vmax);
            r = vmax.distance (vmin)/2;
         }
         c = c.getParent ();
      } while (r <= EPS && c != null);
      
      return r;
   }
   
   public ControlPanel createControlPanel() {
      
      ControlPanel panel = new ControlPanel("frame reg controls");
      
      final DynamicRegistrationController reg = (DynamicRegistrationController)(getControllers ().get ("registration"));
      
      panel.addWidget ("force scaling", reg, "forceScaling");
      
      
      VectorMultiField weights = new VectorMultiField ("weights", 2);
      weights.setValue (new Vector2d(1, 1));
      
      weights.addValueChangeListener (new ValueChangeListener() {

         @Override
         public void valueChange (ValueChangeEvent evt) {
            Vector w = (Vector)evt.getValue ();
            setWeightFactors (w.get (0), w.get (1));
         }
         
      });
      
      panel.addWidget (weights);
      
      panel.pack ();
      return panel;
      
   }
   
   protected void setWeightFactors(double wl, double wr) {
      final DynamicRegistrationController reg = (DynamicRegistrationController)(getControllers ().get ("registration"));
      
      for (Frame f : reg.getRegistrationFrames ()) {
         double r = estimateFrameRadius (f);
         System.out.println ("radius: " + r);
         reg.setRegistrationWeight (f, wl, wr);
      }   
   }
   
   protected void setForceScaling(double s) {
      final DynamicRegistrationController reg = (DynamicRegistrationController)(getControllers ().get ("registration"));
      reg.setForceScaling (s);
   }

}
