package artisynth.models.registration;

import java.awt.Color;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.swing.JSeparator;

import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.TargetFrame;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.DynamicMeshComponent;
import artisynth.core.mechmodels.ForceComponent;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameAttachment;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.MotionTarget.TargetActivity;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointAttachment;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ComponentListImpl;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.modelbase.StructureChangeEvent;
import artisynth.core.util.ScanToken;
import artisynth.models.registration.correspondences.MeshCorrespondenceComputer;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix6d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixBlockBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.HierarchyNode;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.Renderer.FaceStyle;
import maspack.spatialmotion.Wrench;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;

/**
 * Applies forces, computes forces jacobians, involved in a physics-based registration.  Allows for point,
 * frame, and mesh targets.
 * 
 * S. Khallaghi et al., "Biomechanically Constrained Surface Registration: Application to MR-TRUS Fusion 
 * for Prostate Interventions," in IEEE Transactions on Medical Imaging, vol. 34, no. 11, pp. 2404-2414, 
 * Nov. 2015. doi: 10.1109/TMI.2015.2440253
 * 
 * Thesis: 
 * 
 * Sanchez, C.A. (2018). Overcoming obstacles in biomechanical modelling: methods for dealing with 
 * discretization, data fusion, and detail. University of British Columbia.  Chapter 4.
 * https://open.library.ubc.ca/cIRcle/collections/ubctheses/24/items/1.0375814
 * 
 * Implementation note: the force scaling parameter is the reciprocal of beta described in the paper.
 **/
public class DynamicRegistrationController extends ControllerBase
   implements RenderableComponent, CompositeComponent {

   public static boolean DEFAULT_ENABLED = true;

   boolean enabled;
   MechModel mech;
   boolean forceAttaching;
   boolean forceAttached;
   
   DynamicRegistrationForce myForce;

   PointList<TargetPoint> targetPoints;
   RenderableComponentList<TargetFrame> targetFrames;
   RenderableComponentList<TargetMesh> targetMeshes;
   ComponentList<MeshCorrespondenceController> controllers;

   HashMap<Point,PointTargetInfo> pointMap;
   HashMap<Frame,FrameTargetInfo> frameMap;
   HashMap<DynamicMeshComponent,MeshTargetInfo> meshMap;
   HashMap<DynamicMeshComponent,MeshCorrespondenceController> meshCorrs;

   public static double DEFAULT_BETA = 1000; // default force scaling
   double beta;                              // force scaling

   private static class PointTargetInfo {
      Point source;
      Point target;
      double weight;

      public PointTargetInfo(Point point, Point target, double w) {
         this.source = point;
         this.target = target;
         this.weight = w;
      }

      public void setWeight(double w) {
         this.weight = w;
      }
   }

   private static class FrameTargetInfo {
      Frame source;
      Frame target;
      Vector2d weight;

      public FrameTargetInfo(Frame frame, Frame target, double w) {
         this.source = frame;
         this.target = target;
         this.weight = new Vector2d(w, w);
      }

      public void setWeight(double wl, double wr) {
         this.weight.x = wl;
         this.weight.y = wr;
      }
   }

   private static class MeshTargetInfo {
      DynamicMeshComponent source;
      TargetMesh target;
      VectorNd weight;

      public MeshTargetInfo(DynamicMeshComponent mesh, TargetMesh target, double w) {
         this.source = mesh;
         this.target = target;
         this.weight = new VectorNd(source.numVertices ());
         setWeight(w);
      }

      public void setWeight(double w) {
         for (int i=0; i<weight.size (); ++i) {
            this.weight.set(i, w);
         }
      }

      public void setWeight(int vidx, double w) {
         this.weight.set (vidx, w);
      }

      public void setWeight(VectorNd weights) {
         this.weight.set (weights);
      }
   }


   static PropertyList myProps = new PropertyList (DynamicRegistrationController.class, ControllerBase.class);
   static {
      myProps.add ("renderProps","render props", new RenderProps());
      myProps.add ("forceScaling","scaling of force", 1);
      myProps.add ("enabled isEnabled setEnabled", "enable/disable controller", DEFAULT_ENABLED);
   }

   public DynamicRegistrationController(MechModel mech) {

      this.enabled = DEFAULT_ENABLED;
      this.mech = mech;
      this.forceAttaching = false;
      this.forceAttached = false;

      myForce = new DynamicRegistrationForce ();
      
      targetPoints = new PointList<>(TargetPoint.class, "targetPoints");
      targetFrames = new RenderableComponentList<>(TargetFrame.class, "targetFrames");
      targetMeshes = new RenderableComponentList<>(TargetMesh.class, "targetMeshes");
      controllers = new ComponentList<>(MeshCorrespondenceController.class, "correspondenceControllers");
      add(targetPoints);
      add(targetFrames);
      add(targetMeshes);
      add(controllers);

      pointMap = new HashMap<> ();
      frameMap = new HashMap<> ();
      meshMap = new HashMap<> ();
      meshCorrs = new HashMap<> ();
      
      setForceScaling (DEFAULT_BETA);
      setDefaultRenderProps();
   }
   
   protected void setDefaultRenderProps() {
      RenderProps.setPointColor (targetPoints, Color.CYAN);
      RenderProps.setEdgeColor (targetFrames, Color.CYAN);
      RenderProps.setFaceColor (targetFrames, Color.CYAN);
      RenderProps.setDrawEdges (targetFrames, true);
      RenderProps.setFaceStyle (targetFrames, FaceStyle.NONE);
      RenderProps.setFaceColor (targetMeshes, Color.CYAN);
      RenderProps.setEdgeColor (targetMeshes, Color.CYAN);
      RenderProps.setDrawEdges (targetMeshes, true);
      RenderProps.setFaceStyle (targetMeshes, FaceStyle.NONE);
   }
   
   @Override
   public void setName (String name) {
      super.setName (name);
      try {
         myForce.setName (name + "_force");
      } catch (Exception e) {}
   }

   /**
    * Creates a controller with supplied set of model sources components
    * @param mech mech model containing all model components
    * @param sourcePoints source points
    * @param sourceFrames source frames
    * @param sourceMeshes source meshes
    */
   public DynamicRegistrationController(MechModel mech, 
      Collection<? extends Point> sourcePoints, 
      Collection<? extends Frame> sourceFrames,
      Collection<? extends DynamicMeshComponent> sourceMeshes) {

      this(mech);

      if (sourcePoints != null) {
         for (Point pnt : sourcePoints) {
            addRegistrationTarget (pnt);
         }
      }

      if (sourceFrames != null) {
         for (Frame f : sourceFrames) {
            addRegistrationTarget(f);
         }
      }
      
      if (sourceMeshes != null) {
         for (DynamicMeshComponent dmc : sourceMeshes) {
            addRegistrationTarget(dmc);
         }
      }
   }

   /**
    * Adds a registration source point with weight, creates a target point to register to.
    * The target point is already added internally to this controller's subcomponents.
    * 
    * @param pnt model point
    * @param w weight
    * @return target
    */
   public TargetPoint addRegistrationTarget(Point pnt, double w) {

      TargetPoint tpnt = new TargetPoint();
      tpnt.setName((pnt.getName() != null ? pnt.getName() : String.format(
         "p%d", pnt.getNumber())) + "_ref");
      tpnt.setState(pnt);
      tpnt.setTargetActivity(TargetActivity.PositionVelocity);

      targetPoints.add (tpnt);
      addRegistrationTarget(pnt, tpnt, w);

      return tpnt;
   }
   
   /**
    * Manually adds a point and registration target.
    * @param source model source point
    * @param target target point to register to
    * @param w registration weight
    */
   public void addRegistrationTarget(Point source, Point target, double w) {
      PointTargetInfo pinfo = new PointTargetInfo (source, target, w);
      pointMap.put (source, pinfo);
   }

   /**
    * Adds a registration target point with default weight of one
    * @param pnt model source point
    * @return target point
    * @see #addRegistrationTarget(Point, double)
    */
   public TargetPoint addRegistrationTarget(Point pnt) {
      return addRegistrationTarget(pnt, 1.0);
   }
   
   /**
    * Retrieves the target point component associated with a model registration point
    * @param source source point
    * @return target point if exists, null otherwise
    */
   public Point getRegistrationTarget(Point source) {
      PointTargetInfo pti = pointMap.get (source);
      if (pti != null) {
         return pti.target;
      }
      return null;
   }

   /**
    * Removes a registration source point and corresponding target point from the controller
    * @param pnt source point
    */
   public void removeRegistrationTarget(Point pnt) {
      PointTargetInfo pti = pointMap.remove (pnt);
      if (pti != null) {
         targetPoints.remove (pti.target);
      }
   }

   /**
    * Adds a registration source frame with weight, creates a target frame to register to.
    * The target frame is already added internally to this controller's subcomponents.
    * Translation and rotation weights can be specified separately.  Note that
    * the translation error has spatial units squared, whereas rotational error is in radians - 
    * it may be necessary to scale the rotational weight by sqrt(2)*r^2/PI for approximate
    * equal contribution in a body with approximate radius r.
    * 
    * @param frame model source frame
    * @param wt translational weight
    * @param wr rotational weight
    * @return target frame
    */
   public TargetFrame addRegistrationTarget(Frame frame, double wt, double wr) {

      TargetFrame tframe = new TargetFrame();
      tframe.setName((frame.getName() != null ? frame.getName() : String.format(
         "f%d", frame.getNumber())) + "_ref");
      tframe.setState(frame);
      if (frame instanceof RigidBody) {
         // add mesh to TargetFrame
         PolygonalMesh mesh = null;
         RigidBody rb = (RigidBody)frame;
         if ((mesh = rb.getSurfaceMesh()) != null) {
            tframe.setSurfaceMesh(mesh.clone (), rb.getSurfaceMeshComp().getFileName());
            tframe.setRenderProps (rb.getRenderProps ());
         }
      }
      tframe.setTargetActivity(TargetActivity.PositionVelocity);
      targetFrames.add (tframe);
      
      addRegistrationTarget(frame, tframe, wt, wr);
      return tframe;
   }
   
   /**
    * Manually add a registration model source and target frame.
    * 
    * @param source model source frame
    * @param target target frame
    * @param wt translational weight
    * @param wr rotational weight
    * @see #addRegistrationTarget(Frame, double, double)
    */
   public void addRegistrationTarget(Frame source, Frame target, double wt, double wr) {
      FrameTargetInfo finfo = new FrameTargetInfo (source, target, 1.0);
      finfo.setWeight (wt,  wr);
      frameMap.put (source, finfo);
   }

   /**
    * Add a registration source frame with unit weights.  You may want to scale the
    * rotational weight using {@link #setRegistrationWeight(Frame, double, double)}
    * to ensure balanced contributions.
    * 
    * @param frame source frame
    * @return target frame
    * @see #addRegistrationTarget(Frame, double, double)
    */
   public TargetFrame addRegistrationTarget(Frame frame) {
      // XXX should I estimate diameter of frame - or leave that to user?
      //  "ideal": 1.0, r*r*Math.sqrt(2)/Math.PI
      return addRegistrationTarget(frame, 1.0, 1.0);
   }

   /**
    * Retrieves the target frame associated with the supplied source.
    * @param source registration source frame
    * @return target frame if exists, null otherwise
    */
   public Frame getRegistrationTarget(Frame source) {
      FrameTargetInfo fti = frameMap.get (source);
      if (fti != null) {
         return fti.target;
      }
      return null;
   }
   
   /**
    * Removes a registration source frame and corresponding target from this controller.
    * 
    * @param frame source frame
    */
   public void removeRegistrationTarget(Frame frame) {
      FrameTargetInfo fti = frameMap.remove (frame);
      if (fti != null) {
         targetFrames.remove (fti.target);
      }
   }

   /**
    * Adds a registration source mesh with supplied weight and creates a target mesh to register to.
    * The target mesh is already added internally to this controller's subcomponents.  Note that
    * each vertex contributes to the registration error scaled by this weight.  To remove the
    * dependence on number of vertices, set the weight to its reciprocal.  
    * 
    * @param mesh dynamic source mesh
    * @param w registration weight
    * @return created target component
    */
   public TargetMesh addRegistrationTarget(DynamicMeshComponent mesh, double w) {

      TargetMesh tmesh = new TargetMesh ();
      tmesh.setMesh (mesh.getMesh ().copy ());
      tmesh.setName((mesh.getName() != null ? mesh.getName() : null));

      targetMeshes.add (tmesh);
     
      addRegistrationTarget(mesh, tmesh, w);

      return tmesh;
   }
   
   /**
    * Adds a registration source mesh with supplied desired target (not necessarily same number of vertices),
    * as well as a helper object for computing vertex correspondences and weights between the source and target.
    * Internally, this creates and adds a controller to the model that governs the correspondence computations.
    * 
    * @param source model source mesh
    * @param target desired target surface
    * @param mw mesh weight, scale factor applied to all vertices
    * @param correspondences helper object for computing vertex correspondences and weights
    * @return created correspondence controller
    */
   public MeshCorrespondenceController addRegistrationTarget(DynamicMeshComponent source, 
      MeshComponent target, double mw, MeshCorrespondenceComputer correspondences) {
      return addRegistrationTarget (source, target.getMesh (), mw, correspondences);
   }
   
   /**
    * Adds a registration source mesh with supplied desired target (not necessarily same number of vertices),
    * as well as a helper object for computing vertex correspondences and weights between the source and target.
    * Internally, this creates and adds a controller to the model that governs the correspondence computations.
    * 
    * @param source model source mesh
    * @param target desired target surface
    * @param mw mesh weight, scale factor applied to all vertices
    * @param correspondences helper object for computing vertex correspondences and weights
    * @return created correspondence controller
    */
   public MeshCorrespondenceController addRegistrationTarget(DynamicMeshComponent source, 
      MeshBase target, double mw, MeshCorrespondenceComputer correspondences) {
      
      // add target
      addRegistrationTarget (source, 1.0);
      
      // control target correspondences
      MeshCorrespondenceController mcc = new MeshCorrespondenceController (source, target);
      mcc.setMeshWeight (mw);
      mcc.setCorrespondenceComputer (correspondences);
      mcc.setRegistrationController (this);
      addController (source, mcc);
      return mcc;
      
   }
 
   /**
    * Adds a correspondence controller internally, maintaining map
    * @param source registration source mesh
    * @param corr correspondence controller
    */
   protected void addController(DynamicMeshComponent source, MeshCorrespondenceController corr) {
      controllers.add (corr);
      meshCorrs.put (source, corr);
   }
   
   /**
    * Removes a correspondence controller internally, maintaining map
    * @param source registration source mesh
    * @return true if removed successfully, false otherwise
    */
   protected boolean removeController(DynamicMeshComponent source) {
      MeshCorrespondenceController mcc = meshCorrs.remove (source);
      if (mcc != null) {
         return controllers.remove (mcc);
      }
      return false;
   }
   
   /**
    * Manually adds a registration source mesh and target mesh to the registration controller.  The target
    * mesh must have the same number of vertices as the source mesh, and correspondences are one-to-one.
    * Each vertex contributes to the registration error.  It may be prudent to set it to the reciprocal
    * of the number of vertices.
    * 
    * @param source source mesh
    * @param target target mesh
    * @param w mesh registration weight
    */
   public void addRegistrationTarget(DynamicMeshComponent source, TargetMesh target, double w) {
      MeshTargetInfo minfo = new MeshTargetInfo (source, target, w);
      meshMap.put (source, minfo);
   }

   /**
    * Adds a registration source mesh with unit vertex weight, creating a corresponding target mesh.
    * @param mesh source mesh
    * @return target mesh
    * @see #addRegistrationTarget(DynamicMeshComponent, double)
    */
   public TargetMesh addRegistrationTarget(DynamicMeshComponent mesh) {
      return addRegistrationTarget(mesh, 1.0);
   }
   
   /**
    * Retrieves the registration target component associated with a source mesh
    * @param source source mesh
    * @return target mesh if exists
    */
   public TargetMesh getRegistrationTarget(DynamicMeshComponent source) {
      MeshTargetInfo mti = meshMap.get (source);
      if (mti != null) {
        return mti.target;
      }
      return null;
   }

   /**
    * Removes a registration source mesh and target
    * @param mesh source mesh
    */
   public void removeRegistrationTarget(DynamicMeshComponent mesh) {

      MeshTargetInfo mti = meshMap.remove (mesh);
      if (mti != null) {
         targetMeshes.remove (mti.target);
         removeController (mesh);
      }

   }

   /**
    * Sets the point registration weight
    * @param point source point 
    * @param w registration weight
    */
   public void setRegistrationWeight(Point point, double w) {
      PointTargetInfo pti = pointMap.get (point);
      if (pti != null) {
         pti.setWeight (w);
      }
   }
   
   /**
    * Retrieves the point registration weight
    * @param point source point
    * @return weight
    */
   public double getRegistrationWeight(Point point) {
      PointTargetInfo pti = pointMap.get (point);
      if (pti != null) {
         return pti.weight;
      }
      return 0;
   }

   /**
    * Sets the frame registration weights
    * @param frame source frame
    * @param wt translational weight
    * @param wr rotational weight
    */
   public void setRegistrationWeight(Frame frame, double wt, double wr) {
      FrameTargetInfo fti = frameMap.get (frame);
      if (fti != null) {
         fti.setWeight (wt, wr);
      }
   }
 
   /**
    * Retrieves the translational and rotational registration weights associated with a frame
    * @param frame source frame
    * @return translational (x) and rotational (y) weights if they exist
    */
   public Vector2d getRegistrationWeight(Frame frame) {
      FrameTargetInfo fti = frameMap.get (frame);
      if (fti != null) {
         return fti.weight;
      }
      return null;
   }

   /**
    * Sets the registration weight associated with a particular source vertex.
    * 
    * @param mesh source mesh
    * @param vidx vertex index
    * @param w vertex weight
    */
   public void setRegistrationWeight(DynamicMeshComponent mesh, int vidx, double w) {
      MeshTargetInfo mti = meshMap.get (mesh);
      if (mti != null) {
         mti.setWeight (vidx, w);
      }
   }

   /**
    * Sets the weight for all vertices in a registration mesh source/target pair.
    * 
    * @param mesh source mesh
    * @param w vertex weight
    */
   public void setRegistrationWeight(DynamicMeshComponent mesh, double w) {
      MeshTargetInfo mti = meshMap.get (mesh);
      if (mti != null) {
         mti.setWeight (w);
      }
   }

   /**
    * Sets the weights for all vertices in a mesh source/target pair
    * @param mesh source mesh
    * @param w vector of weights
    */
   public void setRegistrationWeight(DynamicMeshComponent mesh, VectorNd w) {
      MeshTargetInfo mti = meshMap.get (mesh);
      if (mti != null) {
         mti.setWeight (w);
      }
   }
   
   /**
    * Retrieves the set of registration weights for all vertices in the source/target mesh pair
    * @param mesh source mesh
    * @return vertex weights
    */
   public VectorNd getRegistrationWeight(DynamicMeshComponent mesh) {
      MeshTargetInfo mti = meshMap.get (mesh);
      if (mti != null) {
         return mti.weight;
      }
      return null;
   }
   
   /**
    * Gets the weighted error for a source/target point
    * @param source source point
    * @return error
    */
   public double getRegistrationError(Point source) {
      PointTargetInfo pti = pointMap.get (source);
      if (pti != null) {
         double err = Math.sqrt (source.getPosition ().distanceSquared (pti.target.getPosition ())*pti.weight);
         return err;
      }
      return 0;
   }
   
   /**
    * Gets the weighted error for a source/target frame
    * @param source source frame
    * @return error
    */
   public double getRegistrationError(Frame source) {
      FrameTargetInfo fti = frameMap.get (source);
      if (fti != null) {
         RigidTransform3d xm = fti.source.getPose ();
         RigidTransform3d ym = fti.target.getPose ();
         // angle between orientations
         RotationMatrix3d R = new RotationMatrix3d();
         R.mulInverseRight (xm.R, ym.R);
         AxisAngle aa = R.getAxisAngle ();
         double err =  Math.sqrt(fti.weight.x*xm.p.distanceSquared (ym.p) + fti.weight.y*aa.angle*aa.angle);
         return err;
      }
      return 0;
   }
   
   /**
    * Gets the weighted error for the source/target mesh
    * @param source source mesh
    * @return error
    */
   public double getRegistrationError(DynamicMeshComponent source) {
      MeshTargetInfo mti = meshMap.get (source);
      if (mti != null) {
         
         double err2 = 0;
         MeshComponent target = mti.target;
         for (int i=0; i<source.numVertices (); ++i) {
            Point3d wsrc = source.getVertex (i).getWorldPoint ();
            Point3d wtgt = target.getVertex (i).getWorldPoint ();
            err2 += mti.weight.get (i)*wsrc.distanceSquared (wtgt);
         }
         
         return Math.sqrt (err2);
      }
      return 0;
   }
   
   /**
    * Retrieves the set of registration source points
    * @return source points
    */
   public Set<Point> getRegistrationPoints() {
      return pointMap.keySet ();
   }
   
   /**
    * Retrieves the set of registration source frames
    * @return source frames
    */
   public Set<Frame> getRegistrationFrames() {
      return frameMap.keySet ();
   }
   
   /**
    * Retrieves the set of registration source meshes
    * @return source meshes
    */
   public Set<DynamicMeshComponent> getRegistrationMeshes() {
      return meshMap.keySet ();
   }
   
   /**
    * Retrieves the internal set of target points.  This does not include any target points that were
    * added to the controller manually via {@link #addRegistrationTarget(Point, Point, double)}.
    * @return set of internal target points
    */
   public PointList<TargetPoint> getTargetPoints() {
      return targetPoints;
   }
   
   /**
    * Retrieves the internal set of target frames.  This does not include any target frames that were
    * added to the controller manually via {@link #addRegistrationTarget(DynamicMeshComponent, TargetMesh, double)}.
    * @return set of internal target frames
    */
   public RenderableComponentList<TargetFrame> getTargetFrames() {
      return targetFrames;
   }
   
   /**
    * Retrieves the internal set of target meshes.  This does not include any target meshes that were
    * added to the controller manually via {@link #addRegistrationTarget(DynamicMeshComponent, TargetMesh, double)}.
    * @return set of internal target meshes
    */
   public RenderableComponentList<TargetMesh> getTargetMeshes() {
      return targetMeshes;
   }

   // ENSURE FORCE IS PROPERLY ATTACHED

   @Override
   public void connectToHierarchy (CompositeComponent connector) {
      if (connector == getParent()) {
         attachForceEffector ();
      }
      super.connectToHierarchy (connector);
   }

   /**
    * Attach force effector for governing registration
    */
   protected void attachForceEffector() {
      if (!forceAttaching) {
         forceAttaching = true;
         if (mech != null) {
            mech.addForceEffector (myForce);
            forceAttached = true;
         }
         forceAttaching = false;
      }
   }

   /**
    * Detach force effector for governing registration
    */
   protected void detachForceEffector() {
      if (!forceAttaching) {
         forceAttaching = true;
         if (mech != null) {
            mech.removeForceEffector (myForce);
            forceAttached = false;
         }
         forceAttaching = false;
      }
   }

   @Override
   public void disconnectFromHierarchy (CompositeComponent connector) {
      if (connector == getParent()) {
         detachForceEffector();
      }
      super.disconnectFromHierarchy (connector);
   }


   /**
    * Gets force scaling parameter
    * @return scale
    */
   public double getForceScaling() {
      return beta;
   }

   /**
    * Sets force scaling parameter
    * @param beta scale parameter
    */
   public void setForceScaling(double beta) {
      this.beta = beta;
   }

   // PROPERTIES

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   @Override
   public void initialize (double t) {
      for (MeshCorrespondenceController mcc : controllers) {
         mcc.initialize (t);
      }
      super.initialize (t);
   }
   
   @Override
   public void apply (double t0, double t1) {
      if (isEnabled ()) {
         // try attaching now if not already attached
         if (!forceAttached) {
            attachForceEffector ();
         }
         
         for (MeshCorrespondenceController mcc : controllers) {
            mcc.apply (t0, t1);
         }
      }
   }

   /**
    * Controller is enabled
    * @return true if enabled
    */
   public boolean isEnabled() {
      return enabled;
   }

   /**
    * Set controller enabled or not
    * @param set enables controller if set
    */
   public void setEnabled(boolean set) {
      enabled = set;
   }

   /*
    * FORCE COMPONENT IMPLEMENTATION
    */

   /**
    * Registration force
    */
   private class DynamicRegistrationForce extends ModelComponentBase implements ForceComponent {
      
      public DynamicRegistrationForce () {
         super();
      }
      
      @Override
      public void applyForces (double t) {
         if (isEnabled()) {
            applyPointForces(t);
            applyFrameForces(t);
            applyMeshForces(t);
         }
      }
   
      @Override
      public void addSolveBlocks (SparseNumberedBlockMatrix M) {
         if (isEnabled ()) {
            addPointSolveBlocks (M);
            addFrameSolveBlocks (M);
            addMeshSolveBlocks (M);
         }
      }
   
      @Override
      public void addPosJacobian (SparseNumberedBlockMatrix M, double s) {
         if (isEnabled ()) {
            addPointPosJacobian (M, s);
            addFramePosJacobian (M, s);
            addMeshPosJacobian (M, s);
         }
      }
   
      @Override
      public void addVelJacobian (SparseNumberedBlockMatrix M, double s) {
         if (isEnabled ()) {
            // nothing
         }
      }
   
      @Override
      public int getJacobianType () {
         return Matrix.SYMMETRIC;
      }
   
      // FORCES
   
      private void applyPointForces(double t) {
         // f_i = beta * \sum_m w_m * dx_m/du_i^T (y_m - x_m)
   
         Vector3d f = new Vector3d();
   
         // over model points m
         for (Entry<Point,PointTargetInfo> entry : pointMap.entrySet ()) {
   
            PointTargetInfo pti = entry.getValue ();
   
            double wm = pti.weight * beta;
   
            if (wm != 0) {
               // world location of model point
               Point3d xm = pti.source.getPosition ();
               Point3d ym = pti.target.getPosition ();
               f.sub (ym, xm);
               f.scale (wm);
   
               // add force to attachment
               DynamicAttachment pa = pti.source.getAttachment ();
               if (pa != null && pa instanceof PointAttachment) {
                  AttachmentForceUtilities.addPointForce((PointAttachment)pa, f);
               } else {
                  pti.source.addForce (f);
               }
            }
         }
      }
   
      private void applyFrameForces(double t) {
   
         // f_i = \sum_m  dx_m/du_i^T Q_i^T W [ y_m - x_m  ]
         //                                   [ theta*u    ]
         // (u, theta) is axis-angle from x to y
         // W = [ wml*I      0 ]
         //     [     0  wmr*I ]
   
         Wrench f = new Wrench();
         AxisAngle rot = new AxisAngle();
         RotationMatrix3d R = new RotationMatrix3d();
   
         // over model frames m
         for (Entry<Frame,FrameTargetInfo> entry : frameMap.entrySet ()) {
   
            FrameTargetInfo fti = entry.getValue ();
   
            double wmt = fti.weight.x*beta;
            double wmr = fti.weight.y*beta;
   
            if (wmt != 0 || wmr != 0) {
   
               RigidTransform3d xm = fti.source.getPose ();
               RigidTransform3d ym = fti.target.getPose ();
   
               R.mulInverseRight (xm.R, ym.R);
               rot.set (R);
   
               // translation
               f.set (0, wmt*(ym.p.x - xm.p.x));
               f.set (1, wmt*(ym.p.y - xm.p.y));
               f.set (2, wmt*(ym.p.z - xm.p.z));
   
               // rotation
               double d = -wmr*rot.angle;
               f.set (3, d*rot.axis.x);
               f.set (4, d*rot.axis.y);
               f.set (5, d*rot.axis.z);
   
               //               System.out.println ("f: ");
               //               System.out.println (f.toString ("%.2g"));
               //               System.out.println ();
   
               // add force to attachment
               DynamicAttachment att = fti.source.getAttachment ();
               if (att != null && att instanceof FrameAttachment) {
                  AttachmentForceUtilities.addWrench((FrameAttachment)att, f);
               } else {
                  fti.source.addForce (f);
               }
            }
         }
      }
   
      private void applyMeshForces(double t) {
         // f_i = \sum_m w_m * dx_m/du_i^T Q_i^T (y_m - x_m)
   
         Vector3d f = new Vector3d();
   
         // over model points m
         for (Entry<DynamicMeshComponent,MeshTargetInfo> entry : meshMap.entrySet ()) {
   
            MeshTargetInfo pti = entry.getValue ();
   
            for (int i = 0; i < pti.source.numVertices (); ++i) {
               double wm = pti.weight.get (i) * beta;
   
               if (wm != 0) {
                  // world location of model point
                  Point3d xm = pti.source.getVertex(i).getWorldPoint ();
                  Point3d ym = pti.target.getVertex(i).getWorldPoint ();
                  //                  Point3d xm = pti.source.getVertex(i).getPosition ();
                  //                  Point3d ym = pti.target.getVertex(i).getPosition ();
                  f.sub (ym, xm);
                  f.scale (wm);
   
                  // add force to attachment
                  DynamicAttachment pa = pti.source.getVertexAttachment (i);
                  if (pa != null && pa instanceof PointAttachment) {
                     AttachmentForceUtilities.addPointForce((PointAttachment)pa, f);
                  }
                  // System.out.println ("v" + pti.source.getVertex (i).getIndex () + ", pos: " + xm.toString ("%.5f") + ", target: " + ym.toString ("%.5f") + ", force: " + f.toString ("%.5f"));
               }
            }
         }
      }
   
      private void addPointSolveBlocks (SparseNumberedBlockMatrix K) {
         // K_{i,j} = - \sum_m w_m dx_m/du_i^T Q_i^T Q_j dx_m/du_j
   
         // over model points m
         for (Entry<Point,PointTargetInfo> entry : pointMap.entrySet ()) {
   
            PointTargetInfo pti = entry.getValue ();
   
            double wm = pti.weight * beta;
   
            if (wm != 0) {
               Point pm = pti.source;
               DynamicAttachment da = pm.getAttachment ();
               if (da != null && da instanceof PointAttachment) {
                  PointAttachment pa = (PointAttachment)da;
                  AttachmentForceUtilities.addSolveBlocks (K, pa, pa);
               } else if (pti.source.isDynamic ()) {
                  // add point directly
                  int bi = pm.getSolveIndex ();
                  if (bi >= 0) {
                     MatrixBlock blk = K.getBlock (bi, bi);
                     if (blk == null) {
                        blk =  MatrixBlockBase.alloc (pm.getVelStateSize (), pm.getVelStateSize ());
                        K.addBlock (bi, bi, blk);
                     }
                  } // valid block indices
               }
   
            } // non-zero weight
   
         } // point
      }
   
      private void addFrameSolveBlocks (SparseNumberedBlockMatrix K) {
         // K_{i,j} = - \sum_m dx_m/du_i^T Q_i^T W Q_j dx_m/du_j
         // W = [ wml*I      0 ]
         //     [     0  wmr*I ]
   
         // over model frames m
         for (Entry<Frame,FrameTargetInfo> entry : frameMap.entrySet ()) {
   
            FrameTargetInfo fti = entry.getValue ();
   
            double wmt = fti.weight.x*beta;
            double wmr = fti.weight.y*beta;
   
            if (wmt != 0 || wmr != 0) {
               Frame fm = fti.source;
               DynamicAttachment da = fm.getAttachment ();
               if (da != null && da instanceof FrameAttachment) {
                  FrameAttachment fa = (FrameAttachment)da;
                  AttachmentForceUtilities.addSolveBlocks (K, fa, fa);
               } else if (fm.isDynamic ()) {
                  // add frame directly
                  int bi = fm.getSolveIndex ();
                  if (bi >= 0) {
                     MatrixBlock blk = K.getBlock (bi, bi);
                     if (blk == null) {
                        blk =  MatrixBlockBase.alloc (fm.getVelStateSize (), fm.getVelStateSize ());
                        K.addBlock (bi, bi, blk);
                     }
                  } // valid block indices
               }
            } // non-zero weight
         } // frame
      }
   
      private void addMeshSolveBlocks (SparseNumberedBlockMatrix K) {
         // K_{i,j} = - \sum_m w_m dx_m/du_i^T Q_i^T Q_j dx_m/du_j
   
         // over meshes
         for (Entry<DynamicMeshComponent,MeshTargetInfo> entry : meshMap.entrySet ()) {
   
            MeshTargetInfo pti = entry.getValue ();
            DynamicMeshComponent msource = pti.source;
   
            // over vertices m
            for (int i=0; i < msource.numVertices (); ++i) {
   
               double wm = pti.weight.get (i) * beta;
   
               if (wm != 0) {
                  DynamicAttachment da = msource.getVertexAttachment (i);
                  if (da != null && da instanceof PointAttachment) {
                     PointAttachment pa = (PointAttachment)da;
                     AttachmentForceUtilities.addSolveBlocks (K, pa, pa);
                  }
               } // non-zero weight
            } // vertex
         }// mesh
      }
   
      protected void addPointPosJacobian(SparseNumberedBlockMatrix K, double s) {
         // K_{i,j} = - \sum_m w_m dx_m/du_i^T Q_i^T Q_j dx_m/du_j 
   
         // over model points m
         for (Entry<Point,PointTargetInfo> entry : pointMap.entrySet ()) {
   
            PointTargetInfo pti = entry.getValue ();
   
            double wm = -s*pti.weight*beta;
   
            if (wm != 0) {
               // add attachment Jacobian attachment
               Point pm = pti.source;
               DynamicAttachment da = pm.getAttachment ();
               if (da != null && da instanceof PointAttachment) {
                  PointAttachment pa = (PointAttachment)da;
                  AttachmentForceUtilities.addScaledMulTranspose (K, wm, pa, pa);
               } else if (pm.isDynamic ()) {
                  // dynamic point
                  int bidx = pm.getSolveIndex ();
                  if (bidx >= 0) {
                     // diagonal for point on model
                     MatrixBlock bii = K.getBlock (bidx, bidx);
                     for (int j = 0; j < bii.colSize (); ++j) {
                        bii.set (j, j, wm);
                     }
                  }
               }
            }
         }
      }
   
      protected void addFramePosJacobian(SparseNumberedBlockMatrix K, double s) {
         // K_{i,j} = - \sum_m dx_m/du_i^T Q_i^T W Q_j dx_m/du_j 
         // W = [ wml*I      0 ]
         //     [     0  wmr*I ]
   
         Matrix6d M = new Matrix6d();
   
         // over model frames m
         for (Entry<Frame,FrameTargetInfo> entry : frameMap.entrySet ()) {
   
            FrameTargetInfo fti = entry.getValue ();
   
            double wmt = -s*fti.weight.x*beta;
            double wmr = -s*fti.weight.y*beta;
   
            if (wmt != 0 || wmr != 0) {
   
               // translation
               M.set (0, 0, wmt);
               M.set (1, 1, wmt);
               M.set (2, 2, wmt);
               // rotation
               M.set (3, 3, wmr);
               M.set (4, 4, wmr);
               M.set (5, 5, wmr);
   
               //               System.out.println("M: ");
               //               System.out.println(M.toString ("%.2g"));
               //               System.out.println();
   
               // add Jacobian term
               Frame fm = fti.source;
               DynamicAttachment da = fm.getAttachment ();
               if (da != null && da instanceof FrameAttachment) {
                  FrameAttachment fa = (FrameAttachment)da;
                  AttachmentForceUtilities.addMulTransposeLeftAndRight(K, fa, M, fa);
   
               } else {
   
                  // direct frame attachment
                  int bidx = fm.getSolveIndex ();
                  if (bidx >= 0) {
                     MatrixBlock bii = K.getBlock (bidx, bidx);
                     bii.add (M);
                  }
   
               }
            }
         }
      }
   
      protected void addMeshPosJacobian(SparseNumberedBlockMatrix K, double s) {
         // K_{i,j} = - \sum_m w_m dx_m/du_i^T Q_i^T Q_j dx_m/du_j 
   
         // over meshes
         for (Entry<DynamicMeshComponent,MeshTargetInfo> entry : meshMap.entrySet ()) {
   
            MeshTargetInfo pti = entry.getValue ();
            DynamicMeshComponent msource = pti.source;
   
            // over vertices m
            for (int i=0; i < msource.numVertices (); ++i) {
   
               double wm = -s*pti.weight.get (i)*beta;
   
               if (wm != 0) {
                  // add attachment Jacobian attachment
                  DynamicAttachment da = pti.source.getVertexAttachment (i);
                  if (da != null && da instanceof PointAttachment) {
                     PointAttachment pa = (PointAttachment)da;
                     AttachmentForceUtilities.addScaledMulTranspose (K, wm, pa, pa);
                  }
                  
                  // System.out.println ("v" + pti.source.getVertex (i).getIndex () + ", wm: " + wm + ", K: \n" + K.toString ("%.5f"));
               }
            }
         }
      }
   }
   
   // RENDER
   @Override
   public void prerender (RenderList list) {
      super.prerender (list);
      recursivelyPrerender (this, list);      
   }
   
   protected void recursivelyPrerender (
      CompositeComponent comp, RenderList list) {

       for (int i=0; i<comp.numComponents(); i++) {
         ModelComponent c = comp.get (i);
         if (c instanceof Renderable) {
            list.addIfVisible ((Renderable)c);
         }
         else if (c instanceof CompositeComponent) {
            recursivelyPrerender ((CompositeComponent)c, list);
         }
      }     
   }
   
   // COMPOSITE COMPONENT
   
   protected ComponentListImpl<ModelComponent> myComponents =
      new ComponentListImpl<ModelComponent>(ModelComponent.class, this);

   private NavpanelDisplay myDisplayMode = NavpanelDisplay.NORMAL;
   
// ========== Begin ModelComponent overrides ==========

   public Iterator<? extends HierarchyNode> getChildren() {
      return myComponents.iterator();
   }

   public Iterator<ModelComponent> iterator() {
      return myComponents.iterator();
   }

   public boolean hasChildren() {
      // hasChildren() might be called in the super() constructor, from the
      // property progagation code, before myComponents has been instantiated
      return myComponents != null && myComponents.size() > 0;
   }

   public boolean hasState() {
      return true;
   }

   // ========== End ModelComponent overrides ==========

   // ========== Begin CompositeComponent implementation ==========

   /**
    * {@inheritDoc}
    */
   public ModelComponent get (String nameOrNumber) {
      return myComponents.get (nameOrNumber);
   }

   /**
    * {@inheritDoc}
    */
   public ModelComponent get (int idx) {
      return myComponents.get (idx);
   } 

   /**
    * {@inheritDoc}
    */
   public ModelComponent getByNumber (int num) {
      return myComponents.getByNumber (num);
   }

   /**
    * {@inheritDoc}
    */
   public int numComponents() {
      return myComponents.size();
   }

   /**
    * {@inheritDoc}
    */
   public int indexOf (ModelComponent comp) {
      return myComponents.indexOf (comp);
   }

   /**
    * {@inheritDoc}
    */
   public ModelComponent findComponent (String path) {
      return ComponentUtils.findComponent (this, path);
   }

   /**
    * {@inheritDoc}
    */
   public int getNumberLimit() {
      return myComponents.getNumberLimit();
   }

   /**
    * {@inheritDoc}
    */
   public NavpanelDisplay getNavpanelDisplay() {
      return myDisplayMode;
   }
   
   /**
    * Sets the display mode for this component. This controls
    * how the component is displayed in a navigation panel. The default
    * setting is <code>NORMAL</code>.
    *
    * @param mode new display mode
    */
   public void setDisplayMode (NavpanelDisplay mode) {
      myDisplayMode = mode;
   }

   /**
    * {@inheritDoc}
    */
   public void componentChanged (ComponentChangeEvent e) {
      myComponents.componentChanged (e);
      notifyParentOfChange (e);
   }

   /**
    * {@inheritDoc}
    */
   public void updateNameMap (
      String newName, String oldName, ModelComponent comp) {
      myComponents.updateNameMap (newName, oldName, comp);
   }

   /**
    * {@inheritDoc}
    */
   public boolean hierarchyContainsReferences() {
      return false;
   }

   // ========== End CompositeComponent implementation ==========

   public void add (ModelComponent comp) {
      myComponents.add (comp);
   }
   
   public boolean remove (ModelComponent comp) {
      return myComponents.remove (comp);
   }
   
   protected void removeAll() {
      myComponents.removeAll();
   }

   protected void notifyStructureChanged (Object comp) {
      notifyStructureChanged (comp, /*stateIsChanged=*/true);
   }
        
   protected void notifyStructureChanged (Object comp, boolean stateIsChanged) {
      if (comp instanceof CompositeComponent) {
         notifyParentOfChange (
            new StructureChangeEvent ((CompositeComponent)comp,stateIsChanged));
      }
      else if (!stateIsChanged) {
         notifyParentOfChange (
            StructureChangeEvent.defaultStateNotChangedEvent);
      }
      else {
         notifyParentOfChange (
            StructureChangeEvent.defaultEvent);
      }
   }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (ScanWriteUtils.scanProperty (rtok, this, tokens)) {
         return true;
      }
      else if (myComponents.scanAndStoreComponentByName (rtok, tokens)) {
         return true;
      }
      rtok.pushBack();
      return false;
   }

   protected boolean postscanItem (
      Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {
      
      if (myComponents.postscanComponent (tokens, ancestor)) {
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }

   @Override
      public void scan (
         ReaderTokenizer rtok, Object ref) throws IOException {

      myComponents.scanBegin();
      super.scan (rtok, ref);
   }

   @Override
   public void postscan (
   Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {
      if (hierarchyContainsReferences()) {
         ancestor = this;
      }
      super.postscan (tokens, ancestor);
      myComponents.scanEnd();
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      super.writeItems (pw, fmt, ancestor);
      myComponents.writeComponentsByName (pw, fmt, ancestor);
   }

   public DynamicRegistrationController copy (
      int flags, Map<ModelComponent,ModelComponent> copyMap) {

      DynamicRegistrationController ccomp =
         (DynamicRegistrationController)super.copy (flags, copyMap);

      ccomp.myComponents =
         new ComponentListImpl<ModelComponent>(ModelComponent.class, this);
      ccomp.myDisplayMode = myDisplayMode;
      
      return ccomp;
   }

   public ComponentList<MeshCorrespondenceController> getMeshCorrespondenceControllers () {
      return controllers;
   }

   /**
    * Add registration controls to a control panel
    * @param panel control panel to populate
    * @param controller registration controller
    */
   public static void addControls (
      ControlPanel panel, DynamicRegistrationController controller) {
      
      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create (prop);
         panel.addWidget(widget);
      }
      ComponentList<MeshCorrespondenceController> mccs = controller.getMeshCorrespondenceControllers ();
      for (MeshCorrespondenceController mcc : mccs) {
         panel.addWidget (new JSeparator ());
         String name = mcc.getName ();
         if (name == null) {
            name = "C" + mcc.getNumber ();
         }
         panel.addLabel (name);
         for (PropertyInfo propInfo : mcc.getAllPropertyInfo ()) {
            Property prop = mcc.getProperty(propInfo.getName());
            LabeledComponentBase widget = PropertyWidget.create (prop);
            panel.addWidget(widget);
         }
      }
      
   }

}
