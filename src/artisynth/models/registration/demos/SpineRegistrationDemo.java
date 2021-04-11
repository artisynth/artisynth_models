package artisynth.models.registration.demos;

import java.awt.Color;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;

import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.TargetFrame;
import artisynth.core.materials.LinearFrameMaterial;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidMeshComp;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.PullController;
import artisynth.core.workspace.RootModel;
import artisynth.models.registration.DynamicRegistrationController;
import maspack.geometry.AABBTree;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Feature;
import maspack.geometry.MeshFactory;
import maspack.geometry.MeshUtilities;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;
import maspack.widgets.BooleanSelector;
import maspack.widgets.EnumSelector;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;

/**
 * Registration demo with a spine model that can change target types between point, 
 * curve, and body
 */
public class SpineRegistrationDemo extends RootModel {

   public static enum TargetType {
      NONE, POINT, FRAME, CURVE
   }
   
   public static class FrameRegistrationTarget extends TargetFrame {

      static TargetType DEFAULT_TARGET_TYPE = TargetType.NONE;
      static Vector2d DEFAULT_TARGET_WEIGHT = new Vector2d(1,1);

      static PropertyList myProps = new PropertyList (FrameRegistrationTarget.class);
      static {
         myProps.add ("targetType", "target type", DEFAULT_TARGET_TYPE);
         myProps.add ("targetWeight", "weight", DEFAULT_TARGET_WEIGHT);
         myProps.add ("renderProps", "render properties", new RenderProps());
         myProps.add ("axisLength", "axis length", 0);
      }

      TargetType targetType;
      Vector2d targetWeight;

      @Override
      public PropertyList getAllPropertyInfo () {
         return myProps;
      }

      public FrameRegistrationTarget (RigidBody rb) {

         targetType = DEFAULT_TARGET_TYPE;

         super.setName((rb.getName() != null ? rb.getName() : String.format(
            "f%d", rb.getNumber())) + "_target");

         // add mesh to TargetFrame
         PolygonalMesh mesh = new PolygonalMesh();
         boolean meshAdded = false;
         for (RigidMeshComp mc : rb.getMeshComps ()) {
            PolygonalMesh smesh = (PolygonalMesh)mc.getMesh ();
            if (smesh != null) {
               // shift mesh by marker location
               PolygonalMesh shiftedMesh = smesh.clone ();
               shiftedMesh.setMeshToWorld (smesh.getMeshToWorld ());
               mesh.addMesh (shiftedMesh);
               meshAdded = true;
            }
         } 

         setRenderProps (rb.getRenderProps ());
         if (meshAdded) {
            setSurfaceMesh (mesh);
         } else {
            setAxisLength (0.2);
         }

         double r = rb.getCharacteristicRadius ();
         targetWeight = new Vector2d(1, r*r);
         setPose(rb.getPose ());

      }
      @Override
      public void render (Renderer renderer, int flags) {
         if (targetType == TargetType.FRAME) {
            super.render (renderer, flags);
         } else if (targetType == TargetType.POINT ) {
            boolean highlight = (flags & Renderer.HIGHLIGHT) != 0 || isSelected ();
            // renderer.setPointColoring (getRenderProps(), highlight);
            float[] fpos = new float[3];
            getPosition().get (fpos);
            renderer.drawPoint (getRenderProps(), fpos, highlight);
         } else if (targetType == TargetType.CURVE) {

         }
      }

      public void setTargetType(TargetType type) {
         targetType = type;
      }

      public TargetType getTargetType() {
         return targetType;
      }

      public void setTargetWeight(Vector2d w) {
         targetWeight.set(w);
      }

      public Vector2d getTargetWeight() {
         return targetWeight;
      }
   }

   private static class RegistrationCorrespondenceSetter extends ControllerBase {

      DynamicRegistrationController reg;
      HashMap<Frame,FrameRegistrationTarget> targetMap;
      Polyline curve;
      AABBTree curveTree;

      public RegistrationCorrespondenceSetter(
         DynamicRegistrationController reg, 
         HashMap<Frame,FrameRegistrationTarget> targetMap,
         Polyline curve) {
         this.reg = reg;
         this.targetMap = targetMap;
         this.curve = curve;
         curveTree = new AABBTree();
         if (curve != null) {
            curveTree.build (Arrays.asList (this.curve.getSegments ()));
         }
      }
      
      public void setTargetType(TargetType tt) {
         for (Frame f : reg.getRegistrationFrames ()) {
            FrameRegistrationTarget brt = targetMap.get (f);
            brt.setTargetType (tt);
         }
      }

      @Override
      public void apply (double t0, double t1) {

         BVFeatureQuery bvq = new BVFeatureQuery ();
         Point3d near = new Point3d();

         for (Frame f : reg.getRegistrationFrames ()) {
            FrameRegistrationTarget brt = targetMap.get (f);

            if (brt != null) {
               switch (brt.targetType) {
                  case NONE:
                     reg.setRegistrationWeight (f, 0, 0);

                     // move target with model
                     brt.setPose (f.getPose ());
                     break;
                  case CURVE: {
                     if (curve != null) {
                        reg.setRegistrationWeight (f, brt.targetWeight.x, 0);

                        // project target to curve
                        Feature line = bvq.nearestFeatureToPoint (near, curveTree, f.getPosition ());
                        if (line != null) {
                           brt.setPosition (near);
                           brt.setOrientation (f.getOrientation ());
                        }
                     } else {
                        // treat as if NONE
                        reg.setRegistrationWeight (f, 0, 0);                        
                        // move target with model
                        brt.setPose (f.getPose ());
                        break;
                     }

                     break;
                  }
                  case POINT:
                     reg.setRegistrationWeight (f,  brt.targetWeight.x, 0);
                     // rotate to match model
                     brt.setOrientation (f.getOrientation ());
                     break;
                  case FRAME:
                     reg.setRegistrationWeight(f, brt.targetWeight.x, brt.targetWeight.y);
                     break;
                  default:
                     break;

               }
            }
         }

      }

   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      MechModel mech = new MechModel("mech");
      mech.setFrameDamping (0.001);
      mech.setRotaryDamping (0.02);
      mech.setGravity (0, 0, 1e-12);
      addModel(mech);

      RenderableComponentList<ModelComponent> spine = buildPseudoSpine(mech);
      addRegistrationController(mech, spine);
      setOrientationEnabled (true);
      setForceScaling(100);
      setTargetType(TargetType.FRAME);

   }

   protected void addRegistrationController(MechModel mech, RenderableComponentList<ModelComponent> spine) {

      @SuppressWarnings("unchecked")
      RenderableComponentList<RigidBody> bodies = (RenderableComponentList<RigidBody>)spine.get ("bodies");
      RenderableComponentList<FrameRegistrationTarget> targets = new RenderableComponentList<>(FrameRegistrationTarget.class, "targets");
      addRenderable(targets);

      DynamicRegistrationController rcon = new DynamicRegistrationController (mech);
      HashMap<Frame,FrameRegistrationTarget> targetMap = new HashMap<> ();
      for (RigidBody rb : bodies) {
         FrameRegistrationTarget target = new FrameRegistrationTarget (rb);
         rcon.addRegistrationTarget (rb, target, 1.0, 1.0);
         target.setTargetType (TargetType.FRAME);
         targets.add (target);
         targetMap.put (rb, target);
      }
      rcon.setName ("registration");

      addController (rcon);

      PolylineMesh pmesh = new PolylineMesh ();
      Vertex3d vtxs[] = new Vertex3d[bodies.size ()+2];
      int idx = 0;
      Point3d pos = new Point3d();
      pos.set(bodies.get (0).getPosition ());
      pos.z -= 0.025;
      pos.x += Math.sin (pos.z*20)/50;
      vtxs[idx++] = pmesh.addVertex (pos);
      for (RigidBody rb : bodies) {
         pos.set (rb.getPosition ());
         pos.x += Math.sin (pos.z*20)/50;
         vtxs[idx++] = pmesh.addVertex (pos);
      }
      pos.set(bodies.get (bodies.size()-1).getPosition ());
      pos.z += 0.025;
      pos.x += Math.sin (pos.z*20)/50;
      vtxs[idx++] = pmesh.addVertex (pos);

      pmesh.addLine (vtxs);
      FixedMeshBody spinecurve = new FixedMeshBody(pmesh);
      addRenderable (spinecurve);

      RegistrationCorrespondenceSetter corr = new RegistrationCorrespondenceSetter (rcon, targetMap, pmesh.getLine (0));
      addController (corr);

      RenderProps.setDrawEdges (rcon, true);
      RenderProps.setFaceStyle (rcon, FaceStyle.NONE);
      RenderProps.setFaceColor (rcon, Color.CYAN);
      RenderProps.setLineColor (rcon, Color.CYAN);

      RenderProps.setLineStyle (spinecurve, LineStyle.CYLINDER);
      RenderProps.setLineRadius (spinecurve, 0.001);
      RenderProps.setLineColor (spinecurve, Color.GREEN);

      RenderProps.setFaceStyle (targets, FaceStyle.NONE);
      RenderProps.setDrawEdges (targets, true);
      RenderProps.setPointStyle (targets, PointStyle.SPHERE);
      RenderProps.setPointRadius (targets, 0.02);
      RenderProps.setPointColor (targets, Color.CYAN);
      RenderProps.setLineColor (targets, Color.CYAN);
      RenderProps.setShading (targets, Shading.SMOOTH);
   }

   @Override
   public void addController (Controller controller) {
      if (controller instanceof PullController) {
         PullController pcon = (PullController)controller;
         pcon.setStiffness (100);
      }
      super.addController (controller);
   }

   protected RenderableComponentList<ModelComponent> buildPseudoSpine(MechModel mech) {

      int nseg = 5;

      Vector3d vsize = new Vector3d(0.05, 0.05, 0.02);

      // frame-spring properties
      double k = 1000;
      double krot = 0.1;
      double d = 0.0;
      double drot = 0.0;

      double dz = 1.5*vsize.z;

      RenderableComponentList<ModelComponent> spine = new RenderableComponentList<>(ModelComponent.class, "spine");
      RenderableComponentList<RigidBody> bodies = new RenderableComponentList<RigidBody>(RigidBody.class, "bodies");
      RenderableComponentList<FrameSpring> springs = new RenderableComponentList<FrameSpring>(FrameSpring.class, "springs");
      spine.add (bodies);
      spine.add (springs);
      mech.add (spine);

      // add vertebrae
      for (int i = 0; i < nseg; ++i) {
         RigidBody v = new RigidBody("v" + i);
         PolygonalMesh vmesh;
         try {
            vmesh = new PolygonalMesh(ArtisynthPath.getSrcRelativeFile (this, "data/T10.obj"));
            MeshUtilities.sqrt3Subdivide (vmesh, 2);
            vmesh.transform (new RigidTransform3d(new Vector3d(0, 0, -0.0125), AxisAngle.ROT_X_90));
         }
         catch (IOException e) {
            vmesh = MeshFactory.subdivide (MeshFactory.createBox (vsize.x, vsize.y, vsize.z), 2);
         }  
         v.addMesh (vmesh, true, true);
         v.setDensity (1000);

         v.setPose (new RigidTransform3d(new Point3d(0, 0, i*dz), AxisAngle.IDENTITY));
         bodies.add (v);
      }

      // add frame-springs between them
      for (int i = 1; i < nseg; ++i) {
         RigidBody v0 = bodies.get (i-1);
         RigidBody v1 = bodies.get (i);
         String name = "joint_" + Integer.toString (i-1) + "_" + Integer.toString(i);
         FrameSpring fs = new FrameSpring (name);
         fs.setMaterial (new LinearFrameMaterial (k, krot, d, drot));
         RigidTransform3d jpos = new RigidTransform3d(new Vector3d(0, 0, (i-0.5)*dz), AxisAngle.IDENTITY);
         fs.setFrames (v0, v1, jpos);

         springs.add (fs);
      }

      //      RigidBody v0 = bodies.get (0);
      //      v0.setDynamic (false);

      RenderProps.setShading (spine, Shading.SMOOTH);
      Color boneColor = new Color( 227, 218, 201);
      RenderProps.setFaceColor (spine, boneColor);

      return spine;

   }

   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);

      addControlPanel (createControlPanel());

      for (Controller c : getControllers() ) {
         if (c != null && c instanceof PullController) {
            PullController pcon = (PullController)c;
            pcon.setStiffness (100);
         }
      }
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

      EnumSelector esel = new EnumSelector ("target type", TargetType.values ());
      esel.addValueChangeListener (new ValueChangeListener() {
         @Override
         public void valueChange (ValueChangeEvent e) {
            TargetType tt = (TargetType)e.getValue ();
            setTargetType (tt);
         }
      });
      esel.setValue (TargetType.FRAME);
      panel.addWidget (esel);
      
      panel.addWidget ("force scaling", reg, "forceScaling");

      BooleanSelector selector = new BooleanSelector ("orientation", true);
      selector.addValueChangeListener (new ValueChangeListener() {
         @Override
         public void valueChange (ValueChangeEvent evt) {
            boolean orient = ((Boolean)(evt.getValue ())).booleanValue ();
            setOrientationEnabled(orient);
         }
      });
      panel.addWidget (selector);
    
      panel.pack ();
      return panel;

   }

   protected void setOrientationEnabled(boolean set) {
      final DynamicRegistrationController reg = (DynamicRegistrationController)(getControllers ().get ("registration"));

      int idx = 0;
      for (Frame f : reg.getRegistrationFrames ()) {
         double rw = 0;
         if (set) {
            // diameter squared
            rw = estimateFrameRadius (f);
            rw = rw*rw / Math.PI;
         }
         if (idx > 0) {
            reg.setRegistrationWeight (f, 1, rw);
         } else {
            reg.setRegistrationWeight (f, 0, 0);
         }
         ++idx;
      }
   }

   protected void setForceScaling(double s) {
      final DynamicRegistrationController reg = (DynamicRegistrationController)(getControllers ().get ("registration"));
      reg.setForceScaling (s);
   }
   
   protected void setTargetType(TargetType tt) {

      for (Controller c : getControllers ()) {
         if (c instanceof RegistrationCorrespondenceSetter) {
            RegistrationCorrespondenceSetter corr = (RegistrationCorrespondenceSetter)c;
            corr.setTargetType(tt);
         }
      }      
   }

}
