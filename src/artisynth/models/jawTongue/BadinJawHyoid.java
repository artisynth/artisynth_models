package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;

import javax.swing.JFrame;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewerFrame;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.matrix.AxisAlignedRotation;
import maspack.util.ReaderTokenizer;
import artisynth.core.driver.Main;
import artisynth.core.driver.ViewerManager;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.editorManager.RemoveComponentsCommand;
import artisynth.core.gui.selectionManager.SelectionManager;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.ForceEffector;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.SegmentedPlanarConnector;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.materials.RotAxisFrameMaterial;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.ComponentChangeEvent.Code;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.probes.TracingProbe;
import artisynth.core.probes.WayPoint;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.MDLMeshIO;
import artisynth.core.util.TimeBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.dynjaw.DigastricMuscle;
import artisynth.models.dynjaw.FoodBolus;
import artisynth.models.dynjaw.JawDemo;
import artisynth.models.dynjaw.JawLarynxDemo;
import artisynth.models.dynjaw.JawModel;
import artisynth.models.dynjaw.JawPanel;
import artisynth.models.tongue3d.HexTongueDemo;

public class BadinJawHyoid extends JawLarynxDemo {

   public static final double m2mm = 1000d;

   public static final boolean useDigsatricSling = false;
   public static final boolean fixHyoid = false;
   public static final boolean showBitePlanes = true;
   


   public static final double duration = 1.0;
   public static final String inputProbeFile = "ramp1.txt";

       String taskName = "open";
//    String taskName = "rchew";
//   String taskName = "rlat";
//   String taskName = "protrude";
//   String taskName  = "rest";
   
   VectorNd jawContour = new VectorNd ();
   VectorNd maxillaContour = new VectorNd ();
   VectorNd hyoidContour = new VectorNd ();
   
   public static final boolean useSymmetricGeometry = true;
   public static final boolean useTeethSmoothedGeometry = true;
   
   public static final String regGeomDir = ArtisynthPath.getSrcRelativePath (
      BadinJawHyoid.class, "geometry/");
   
   public static final RigidTransform3d jawRestPose_symmetric = new RigidTransform3d (
      new Vector3d (98.6508, 0, 87.5041), new AxisAngle (0, 1, 0, Math.toRadians (1.6314)));

   public static final RigidTransform3d jawIPPose_symmetric = new RigidTransform3d(
      new Vector3d (93.679, 0, 92.2072), new AxisAngle (0,1,0, Math.toRadians (8.3136)));
   
   public static final RigidTransform3d jawEEPose_symmetric = new RigidTransform3d(
      new Vector3d (93.96, 0.0, 89.47), new AxisAngle (0,1,0, Math.toRadians (5.9154)));
   
   public static final RigidTransform3d hyoidEEPose = new RigidTransform3d(
      new Vector3d (115.5, 0.0, 62.0), new AxisAngle ());

   
   public static RigidTransform3d getJawIPWorld () {

      RigidTransform3d XJawIPWorld = new RigidTransform3d (
         new Vector3d(93.1968, 1.28798, 91.7165), //mm
         new AxisAngle(0.018748, 0.99949, 0.025949, Math.toRadians (4.8778)));
      
      RigidTransform3d XJawIPWorld_symmetric = new RigidTransform3d (
         new Vector3d(93.7687, 0, 92.2142), //mm
         new AxisAngle(0, 1, 0, Math.toRadians (8.2694)));
      
      return useSymmetricGeometry ? XJawIPWorld_symmetric : XJawIPWorld;
   }
   
   protected RigidTransform3d origJawWorld = null;
   // set to true in order to call setRestPosture in the attach method
   protected boolean setRestPostureInAttach = false;
   
   public static PropertyList myProps =
      new PropertyList (BadinJawHyoid.class, JawLarynxDemo.class);

   static {
      myProps.addReadOnly (
         "jawContour", "contour of jaw mesh in mid-sagittal plane", null);
      myProps.addReadOnly (
         "hyoidContour", "contour of hyoid mesh in mid-sagittal plane", null);
      myProps.addReadOnly (
         "maxillaContour", "contour of maxilla mesh in mid-sagittal plane", null);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   public BadinJawHyoid () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
   }


   public void setupJawModel () {
      super.setupJawModel ();
      setupJoints(/*tmjsUnilateral=*/false);
      
//      makeFixedLarynx ();
      removeLarynx ();
      centerDynamicBodies(new String[]{"jaw", "hyoid"});
      
      if (fixHyoid) {
         myJawModel.rigidBodies ().get ("hyoid").setDynamic (false);
      }
      else {
         setHyoidSpringStiffness ();
      }
      
//      setDampingParams (10, 100, 0.001);
      if (useDigsatricSling)
         makeDigastricSling();

      myJawModel.setMaxStepSize (0.01);
      myJawModel.setIntegrator (Integrator.ConstrainedBackwardEuler);
      myJawModel.showMeshFaces (true);


      
      for (RigidBody body : myJawModel.rigidBodies ()) {
         if (body.getName().compareTo ("jaw") != 0.0 &&
             body.getName().compareTo ("hyoid") != 0.0 &&
             body.getName().compareTo ("maxilla") != 0.0)
            RenderProps.setVisible (body, false);
//         RenderProps.setVisible (body, body.isDynamic ());
      }
      
      for (ForceEffector e : myJawModel.forceEffectors ())
         if (e instanceof FoodBolus)
            ((FoodBolus)e).setActive (false);
      
//      resetSpringLengthProps ();

      replaceSkullMeshes();
      pruneUnnamedMarkers();
      updateMarkerPositions();

      
      if (useSymmetricGeometry) {
          alignMarkers(); // axis alignment applied to jaw,hyoid meshes
          mirrorMarkersRight();
          projectMarkers();
          
//         mirrorMarkersLeft();
//         makeComSymmetric(myJawModel.rigidBodies ().get ("jaw"));
      }
      else {
    	  alignMarkers();
      }
         
      centerSymmetricDynamicBodies (new String[]{"jaw","hyoid"});
      if (!fixHyoid) {
         // update hyoid frame spring attachment
         FrameSpring s = myJawModel.frameSprings ().get ("hyoidTissue");
         if (s != null) {
            s.setAttachFrameB (s.getFrameA ().getPose ());
         }
         setHyoidSpringStiffness ();
      }
      
//      origJawWorld = new RigidTransform3d (
//         myJawModel.rigidBodies ().get("jaw").getPose ());
//      setJawPosture (getJawIPWorld ());
//      updateJawConstraints();
//      resetSpringLengthProps();
//      setJawPosture (origJawWorld);
      
//      transformLarynx();
//      addPalate();
      
      myJawModel.setIntegrator (Integrator.Trapezoidal);
      myJawModel.setMaxStepSize (0.005);
      
//      ((FoodBolus)myJawModel.forceEffectors ().get ("rightbolus")).setActive (true);
 
   }
   
   
   public void resetSpringLengthProps() {
      for (AxialSpring s : myJawModel.axialSprings ()) {
         if (s instanceof Muscle) {
            ((Muscle)s).resetLengthProps ();
         }
         else
         {
            s.setRestLength (s.getLength ());
         }
      }
      
      for (MultiPointSpring s : myJawModel.multiPointSprings ()) {
         if (s instanceof MultiPointMuscle) {
            ((MultiPointMuscle)s).resetLengthProps();
         }
      }
   }
   
   public void setDampingParams(double td, double rd, double d)
   {
      setDamping ("jaw", td, rd);
      setDamping ("hyoid", td, rd);
      setDamping ("thyroid", td, rd);
      setDamping ("cricoid", td, rd);

      setSpringDamping (d);
   }
   
//   private MultiPointMuscle create(String name, Muscle ad, Muscle pd) {
//      MultiPointMuscle dg = new MultiPointMuscle ();
//      dg.setMaterial(ad.getMaterial());
//      dg.setName (name);
//      dg.setMaxLength (ad.getMaxLength ()+pd.getMaxLength ());
//      dg.setOptLength (ad.getOptLength ()+pd.getOptLength ());
//      double optLenChange = (dg.getOptLength ()-ad.getOptLength ())/dg.getOptLength ();
////      System.out.println("digastric opt len change = "+optLenChange);
//      dg.setTendonRatio (optLenChange); // used to match force-length behaviour to ad
//      dg.setMaxForce (ad.getMaxForce ());
//      dg.setPassiveFraction (ad.getPassiveFraction ());
//      dg.setTendonRatio (ad.getTendonRatio ());
//      dg.addPoint (ad.getFirstPoint ()); // ad origin on mandible
//      dg.addPoint (ad.getSecondPoint ()); // ad insertion on hyoid
//      dg.addPoint (pd.getFirstPoint ()); // pd origin on styloid process
//      return dg;
//   }
   
   boolean digastricSlingMade = false;
   public void makeDigastricSling() {
      if (digastricSlingMade) 
         return;
      for (String prefix : new String[]{"l","r"}) {
         Muscle ad = (Muscle)myJawModel.axialSprings().get (prefix+"ad");
         Muscle pd = (Muscle)myJawModel.axialSprings().get (prefix+"pd");
//         MultiPointMuscle dg = create (prefix+"dg", ad, pd);
         DigastricMuscle dg = new DigastricMuscle (prefix+"dg", ad, pd);
         for (MuscleExciter ex : myJawModel.getMuscleExciters ()) {
            boolean dgAlreadyAdded = false;
            int idx = ex.findTarget (ad);
            if (idx != -1) {
               System.out.println("removed ad from "+ex.getName ());
               double gain = ex.getGain (idx);
               ex.removeTarget (ad);
               ex.addTarget (dg, gain);
               dgAlreadyAdded = true;
            }
            idx = ex.findTarget (pd);
            if (idx != -1) {
               System.out.println("removed pd from "+ex.getName ());
               double gain = ex.getGain (idx);
               ex.removeTarget (pd);
               if (!dgAlreadyAdded)
                  ex.addTarget (dg, gain);
            }
         }
         
         myJawModel.getMuscleExciters ().get (
            prefix+"protracthyoid").removeTarget (dg);
         myJawModel.getMuscleExciters ().get (
            prefix+"retracthyoid").removeTarget (dg);
         myJawModel.removeAxialSpring (ad);
         myJawModel.removeAxialSpring (pd);
         myJawModel.addMultiPointSpring (dg);
         
      }
      
      myJawModel.multiPointSprings ().setRenderProps (
         new RenderProps(myJawModel.axialSprings ().getRenderProps ()));
      digastricSlingMade = true;
   }
   
   private Vector3d getExtent(RigidBody body) {
      double inf = Double.MAX_VALUE;
      Point3d pmin = new Point3d(inf, inf, inf);
      Point3d pmax = new Point3d(-inf, -inf, -inf);
      body.updateBounds (pmin, pmax);
      pmax.sub (pmin);
      return pmax; // size
   }
   
   public void setHyoidSpringStiffness() {
      // Stephanie's model 8 springs of 220 N/m
      double k = 8*220;
      Vector3d size = getExtent(myJawModel.rigidBodies ().get ("hyoid"));
//      System.out.println("size = "+size.toString ("%g"));
      double kr = k * size.maxElement () / 2.0;
      FrameSpring hyoidSpring = myJawModel.frameSprings ().get ("hyoidTissue");
      if (hyoidSpring == null) {
         addHyoidSpring (k, kr, 0, 0);
      }
      else {
         hyoidSpring.setMaterial (new RotAxisFrameMaterial (k, kr, 0, 0));
//         hyoidSpring.setStiffness (k);
//         hyoidSpring.setRotaryStiffness (kr);
      }
   }
   
   public void addHyoidSpring(
      double k, double kr, double d, double dr) {
      FrameSpring s = new FrameSpring ("hyoidTissue", k, kr, d, dr);
      s.setFrameA (myJawModel.rigidBodies ().get ("hyoid"));
      s.setFrameB (myJawModel.rigidBodies ().get ("maxilla"));
      s.setAttachFrameB (s.getFrameA ().getPose ());
      myJawModel.addFrameSpring(s);
   }
   
   protected void centerDynamicBodies(String[] bodyNames) {
      ArrayList<RigidBody> bodiesToCenter = new ArrayList<RigidBody> ();
      for (String name : bodyNames) {
         if (myJawModel.rigidBodies ().get (name) != null)
            bodiesToCenter.add (myJawModel.rigidBodies ().get (name));
      }
      
      for (RigidBody body : bodiesToCenter) {
         // compute centre-of-mass from mesh
         body.setInertiaFromDensity (body.getDensity());
         // align body-frame origin at center-of-mass
         RigidTransform3d X = BadinJawHyoid.centerBodyAtCOM (body);
         // re-compute inertia for new body pose (paranoid)
         //body.setInertiaFromMesh (density);
         for (BodyConnector con : myJawModel.bodyConnectors ())
         {
            if (con.getBodyA () == body)
            {
               System.out.println("jaw con - " + con.getName ());
               con.transformGeometry (X);
            }
         }
      }
      
      myJawModel.updateCons ();
   }
   
   private void makeFixedLarynx()
   {
      RigidBody larynx = myJawModel.rigidBodies().get ("thyroid");
      if (larynx == null)
         return;
      
//      larynx.setName ("larynx");
      larynx.setDynamic (true);
      String filename = ArtisynthPath.getSrcRelativePath (JawModel.class, "geometry/larynx_t.obj");
      try 
       { larynx.setMesh (new PolygonalMesh(new File(filename)), filename);
       }
      catch (IOException e)
       { e.printStackTrace ();
       }
      
      ArrayList<FrameMarker> newLarynxMarkers = new ArrayList<FrameMarker>();
      for (FrameMarker m : myJawModel.frameMarkers ())
      {
         if (m.getFrame ().getName ().compareTo ("cricoid")==0)
         {
            newLarynxMarkers.add(m);
         }
      }
      
      for (FrameMarker m : newLarynxMarkers)
      {
         myJawModel.removeFrameMarker (m);
         m.setFrame (larynx);
         m.getLocation ().transform (larynx.getPose());
         myJawModel.addFrameMarker (m);
      }

      LinkedList<ModelComponent> componentsToDelete = new LinkedList<ModelComponent> ();
      componentsToDelete.add (myJawModel.rigidBodies ().get("cricoid"));
      componentsToDelete.add (myJawModel.bodyConnectors ().get("cricothyroid"));
      RemoveComponentsCommand cmd =
         new RemoveComponentsCommand (
            "delete", componentsToDelete);
      cmd.execute ();

     
   }

   private void removeLarynx () {
      ArrayList<RigidBody> bodiesToRemove = new ArrayList<RigidBody> ();
      String[] bodyNamesToRemove =
         new String[] { "thyroid", "cricoid", "sternum" };
      for (String name : bodyNamesToRemove) {
         if (myJawModel.rigidBodies ().get (name) != null)
            bodiesToRemove.add (myJawModel.rigidBodies ().get (name));
      }


      // remove bodies, plus anything that depends on them
      LinkedList<ModelComponent> update = new LinkedList<ModelComponent>();
      LinkedList<ModelComponent> delete =
         ComponentUtils.findDependentComponents (update, bodiesToRemove);

      // SelectionManager sel = Main.getMain ().getSelectionManager ();

      // LinkedList<ModelComponent> componentsToDelete =
      //    new LinkedList<ModelComponent> ();

      // for (RigidBody b : bodiesToRemove) {
      //    sel.addSelected (b);
      // }
      // componentsToDelete = sel.getDependencyExpandedSelection ();
      // sel.clearSelections ();
      
      deleteComponents(delete, update);
//      deactivateComponents(componentsToDelete);

   }
   
   public void deactivateComponents(LinkedList<ModelComponent> componentsToDelete) {
      
      for (ModelComponent m : componentsToDelete) {
         if (Renderable.class.isAssignableFrom (m.getClass ())) {
            RenderProps.setVisible((Renderable)m, false);
         }
         
         if (m instanceof RigidBody) {
            ((RigidBody)m).setDynamic (false);
         }
         else if (m instanceof Muscle) {
            ((Muscle)m).setEnabled (false);
         }
         else if (m instanceof AxialSpring) {
            myJawModel.removeAxialSpring ((AxialSpring)m);
         }
      }
   }
   
   public void deleteComponents (
      LinkedList<ModelComponent> delete, LinkedList<ModelComponent> update) {
      RemoveComponentsCommand cmd =
         new RemoveComponentsCommand (
            "delete", delete, update);
      cmd.execute ();

      while (true) {
         delete = new LinkedList<ModelComponent> ();
         // clean up exciters
         for (MuscleExciter me : myJawModel.getMuscleExciters ()) {
            cleanupExciters (delete, me);
         }
         if (delete.size () > 0) {
            cmd = new RemoveComponentsCommand ("delete", delete);
            cmd.execute ();
         }
         else
            break;
      }
   }

   private void cleanupExciters (
      LinkedList<ModelComponent> toDelete, MuscleExciter me) {
      for (int i = 0; i < me.numTargets (); i++) {
         ExcitationComponent ec = me.getTarget (i);
         if (ec == null
         || (ec instanceof Muscle && !myJawModel.axialSprings ().contains (ec))
         || (ec instanceof MuscleExciter && !myJawModel
            .getMuscleExciters ().contains (ec))) {
            if (!toDelete.contains (me)) {
               toDelete.add (me);
               System.out.println ("removing muscle exciter " + me.getName ()
               + " because target " + ec.getName ());
            }
         }
         else if (ec instanceof MuscleExciter) {
            cleanupExciters (toDelete, (MuscleExciter)ec);
         }
      }
   }
   
   private void setupJoints(boolean tmjsUnilateral) {
      String[] joints = new String[]{"LTMJ", "RTMJ"};
      for (String name : joints) {
         BodyConnector con = myJawModel.bodyConnectors ().get (name);
         if (con != null) {
            if (con instanceof PlanarConnector) {
               ((PlanarConnector)con).setUnilateral (tmjsUnilateral);
            }
            else if (con instanceof SegmentedPlanarConnector) {
               ((SegmentedPlanarConnector)con).setUnilateral (tmjsUnilateral);
            }
         }
      }
      
      RenderProps.setVisible (
         myJawModel.bodyConnectors ().get ("LBITE"), showBitePlanes);
      RenderProps.setVisible (
         myJawModel.bodyConnectors ().get ("RBITE"), showBitePlanes);
   }

   public void addJawOptions (ControlPanel panel) {
      if (panel == null)
         return;
      JawPanel.createJawTonguePanel (myJawModel, panel);
   }

   public void loadControlPanel (RootModel root) {
      String panelNames[] =
         new String[] { "misc", "damping", "laryngealmuscles", "muscles"

         };
      loadControlPanel (root, panelNames);
   }

   public void attach (DriverInterface driver) {

      if (getControlPanels ().size () == 0) {
         loadControlPanel (this);
      }
      
      createProbes(duration);
//      loadProbes ();
      
      // this used to be done in build, but requires component changed event hack
      origJawWorld = new RigidTransform3d (
         myJawModel.rigidBodies ().get("jaw").getPose ());
      setJawPosture (getJawIPWorld ());
      updateJawConstraints();
      resetSpringLengthProps();
      setJawPosture (origJawWorld);
      
      
      // set rest posture in attach() so that jaw pose is changes after 
      // tongue is dynamically-attached for BadinJawHyoidTongue
      /* XXX bad hack */
      if (setRestPostureInAttach ||
          this.getClass().getPackage() == BadinJawHyoid.class.getPackage()) {
	 setRestPosture();
      }
      
//      setIntercuspalPosture();
      
      File workingDir = new File (ArtisynthPath.getSrcRelativePath (
         BadinJawHyoidTongue.class, "data/jtdata/jaw"));
      if (workingDir.exists ()) {
         ArtisynthPath.setWorkingDir (workingDir);
      }
      
//      OBB jawobb = new OBB ();
//      jawobb.set (myJawModel.rigidBodies ().get("jaw").getMesh ());
//      driver.getViewer ().addRenderable (jawobb);
      resetInitialState();
   }
   
//   protected void resetInitialState() {
//      // bit of hack: called when initial state is changed in the attach()
//      // method, requiring the initial state stored in waypoint 0 to be reset
//      getWayPoint(0).setState (this);
//   }
   
   public void loadProbes() {
      
      File inputDataDir = new File (ArtisynthPath.getSrcRelativePath (
         this, "data/"+taskName));
      if (inputDataDir.exists ()) {
         ArtisynthPath.setWorkingDir (inputDataDir);
         probesFilename="0probes.art";
         loadProbes ();
      }
      
      File outputDataDir = new File (ArtisynthPath.getSrcRelativePath (
         this, "data/jtdata/jaw/"+taskName));
      if (outputDataDir.exists ()) {
         ArtisynthPath.setWorkingDir (outputDataDir);
      }
      
      FrameMarker inc = myJawModel.frameMarkers ().get ("lowerincisor");
      RenderProps.setVisible (inc, true);
       TracingProbe p =
       addTracingProbe (inc, "position", 0, 1.0);
       p.setUpdateInterval (0.001);
       p.setRenderInterval (0.005);
       p.setName ("Incisor Trace Position");



//      createViewers (180);
      
      resetprobeformat ();

      removeAllOutputProbes ();
      removeAllWayPoints ();
//      generateWaypoints(0.01, (int)Math.round (duration/0.01));
      //Main.getTimeline ().reset ();
      
      
      NumericOutputProbe op = new NumericOutputProbe (this, "jawContour", "jawcontour.txt",0.1);
      op.setName ("jaw contour");
      op.setStartStopTimes (0, 1);
      addOutputProbe (op);
   }
   
   public void createProbes(double duration) {
      removeAllInputProbes ();
      removeAllOutputProbes ();
      removeAllWayPoints ();
      if (myJawModel != null) {
         addInProbes (this, myJawModel, duration, inputProbeFile);
         addOutProbes (this, myJawModel, duration);
         addBreakPoint (duration);
      }
      //Main.getTimeline ().reset ();
   }
   
   public static void addOutProbes(RootModel root, MechModel jaw, double duration) {
      String[] landmarks = new String[]{"lowerincisor", "ltmj", "rtmj", "hyoidRef"};
      JawDemo.addOutputProbes (root, jaw, landmarks, "position", duration);
      JawDemo.addOutputProbes (root, jaw, landmarks, "velocity", duration);
      JawDemo.addTraceProbes (root, jaw, new String[]{"lowerincisor"}, "", duration);
//      addContourProbes (root, duration);
   }
   
   public static void addInProbes(RootModel root, MechModel jaw, double duration) {
      addInProbes (root, jaw, duration, null);
   }
   
   public static void addInProbes(RootModel root, MechModel jaw, double duration, String filename) {
      String[] muscles = new String[]{"bi_dg", "bi_ad", "bi_ip", "bi_sp", "bi_close"};
      try {
         HexTongueDemo.addMuscleProbes (root, jaw, muscles, "", duration, filename);                                  
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   public static void createViewer(int x, int y, int w, int h, AxisAlignedRotation view,
	 Point3d eyePos, Point3d centerPos, Point3d gridPos, double zoom) {
      GLViewerFrame vf = Main.getMain().createViewerFrame();
      GLViewer v = vf.getViewer();

      // vf.setSize (Toolkit.getDefaultToolkit ().getScreenSize ());
      vf.setSize(w, h);
      vf.setLocation(x, y);
      v.setOrthographicView(true);
      v.setGridVisible(true);
      v.setBackgroundColor(Color.WHITE);
      v.setAxialView(view);
      v.getGrid().setMajorColor(Color.BLACK);
      v.getGrid().setMinorColor(new Color(0.6f, 0.6f, 0.6f));
      v.getGrid().setPosition(gridPos);
      v.getGrid().setLineWidth(2);

      v.setOrthogonal(zoom, 1, 1000);
      v.setEye(eyePos);
      v.setCenter(centerPos);

   }
   
   public void createViewers(double zoom)
   {
      double h = 50;
      createViewer (0, 0, 360, 500, AxisAlignedRotation.X_Z, 
         new Point3d(0,-250,h), new Point3d(0,0,h), new Point3d(0,140,0), zoom);
      createViewer (360, 0, 280, 500, AxisAlignedRotation.Y_Z, 
         new Point3d(100, -5, h), new Point3d(0,-5,h), new Point3d(-140,0,0), zoom);
      
   }
   
   public void generateWaypoints(double step, int repeatNum) {
      WayPoint w = null;
      for (int i=1; i<=repeatNum; i++) {
         w = new WayPoint (step*i);
         addWayPoint (w);
      }
      if (w != null)
         w.setBreakPoint (true);
      myWayPoints.setAttachedFileName ("waypoints.dat");
   }
   
   
   public void resetprobeformat() {
      resetprobeformat (this);
   }
   
   public static void resetprobeformat(RootModel root) {
      for (Probe ip : root.getInputProbes ()) {
         if (ip instanceof NumericInputProbe) {
            ((NumericInputProbe)ip).setFormat ("%g");
         }
      }
      for (Probe op : root.getOutputProbes ()) {
         if (op instanceof NumericOutputProbe) {
            ((NumericOutputProbe)op).setFormat ("%g");
            op.setUpdateInterval (0.01);
         }
      }
   }
   
   
   /*
    * properties for getting mesh contours in mid-sagittal plane
    */
   
   public static void addContourProbes(RootModel root, double duration) {
      if (BadinJawHyoid.class.isAssignableFrom (root.getClass ())) {
         double interval = 0.01;
         addRootOutputProbe (root, "jawContour", duration, interval);
         addRootOutputProbe (root, "hyoidContour", duration, interval);
         addRootOutputProbe (root, "maxillaContour", duration, interval);
      }
   }
   
   public static void addRootOutputProbe(RootModel root, String propname, double duration, double interval) {
      NumericOutputProbe op = new NumericOutputProbe (root, propname, propname+".txt", interval);
      op.setName (propname);
      op.setStartStopTimes (0, duration);
      root.addOutputProbe (op);
   }

   protected PolygonalMesh jawmesh = null;
   protected LinkedList<Vertex3d> jawvertices = null;
   protected Integer[] jawvertidxs =
      new Integer[] { 7, 21, 40, 19, 1, 50, 48, 62, 65, 70, 32, 25, 3, 38, 10,
                     49, 33, 59, 58, 56 };
   public VectorNd getJawContour () {
      if (jawmesh == null || jawvertices == null) {
         
         jawmesh = myJawModel.rigidBodies ().get("jaw").getMesh ();
         jawvertices = getContourVertices (jawmesh, getContourTols ()[0], getContourTols ()[1]);
         jawContour = new VectorNd (jawvertidxs.length*3);
         System.out.println(jawvertices.size ()+"jaw contour pnts, "+getContourTols ()[0]+" tol");
      }
      getMeshContour(jawContour, jawvertices, jawvertidxs);
      return jawContour;
   }

   protected PolygonalMesh maxillamesh = null;
   protected LinkedList<Vertex3d> maxillavertices = null;
   protected Integer[] maxillavertidxs =
      new Integer[] { 18, 15, 16, 7, 8, 3, 1, 21, 40, 27, 42, 24, 23, 5, 39,
                     33, 19, 9 };
   public VectorNd getMaxillaContour () {
      if (maxillamesh == null || maxillavertices == null) {
         maxillamesh = myJawModel.rigidBodies ().get("maxilla").getMesh ();
         maxillavertices = getContourVertices (maxillamesh, getContourTols ()[0], getContourTols ()[1]);
         maxillaContour = new VectorNd (maxillavertidxs.length*3);
//         System.out.println(maxillavertices.size ()+"maxilla contour pnts, "+getContourTols ()[0]+" tol");
      }
      getMeshContour(maxillaContour, maxillavertices, maxillavertidxs);
      return maxillaContour;
   }


   protected PolygonalMesh hyoidmesh = null;
   protected LinkedList<Vertex3d> hyoidvertices = null;
   protected Integer[] hyoidvertidxs =
      new Integer[] { 21, 22, 1, 24, 76, 78, 84, 51, 16, 19, 65 };
   public VectorNd getHyoidContour () {
      if (hyoidmesh == null || hyoidvertices == null) {
         hyoidmesh = myJawModel.rigidBodies ().get("hyoid").getMesh ();
         hyoidvertices = getContourVertices (hyoidmesh, getContourTols ()[0], getContourTols ()[1]);
         hyoidContour = new VectorNd (hyoidvertidxs.length*3);
//         System.out.println(hyoidvertices.size ()+"hyoid contour pnts, "+getContourTols ()[0]+" tol");
      }
      getMeshContour(hyoidContour, hyoidvertices, hyoidvertidxs);
      return hyoidContour;
   }

   
   protected LinkedList<Vertex3d> getContourVertices(PolygonalMesh mesh, double tol, double dist) {
      LinkedList<Vertex3d> vertices = new LinkedList<Vertex3d> ();

      for (Vertex3d v : mesh.getVertices ()) {
         if (distToMidSagittal(v) < tol) {
            checkDistAndAdd(vertices, v, dist);
         }
      }
      return vertices;
   }

   protected void checkDistAndAdd(LinkedList<Vertex3d> vertices, Vertex3d newv, double dist) {
      Point3d vpos = new Point3d();
      Point3d newvpos = new Point3d ();
      projectToMidSagittal (newvpos, newv.getWorldPoint ());
      boolean tooClose = false;
      for (Vertex3d v : vertices) {
         projectToMidSagittal (vpos, v.getWorldPoint ());
         if (vpos.distance (newvpos) < dist) {
            tooClose = true;
            break;
         }
      }
      if (!tooClose)
         vertices.add (newv);
   }
   
   protected void getMeshContour(VectorNd contour, LinkedList<Vertex3d> vertices, Integer[] idxs) {
      if (idxs == null)
         return;
      double[] buf = contour.getBuffer ();
      int idx = 0;
      for (Integer i : idxs) {
         Point3d pos = vertices.get (i-1).getWorldPoint (); // indices are one indexed
         buf[idx] = pos.x;
         buf[idx+1] = pos.y;
         buf[idx+2] = pos.z;
         idx += 3;
      }
   }
   
   public static RigidTransform3d centerBodyAtCOM (RigidBody body) {
      return centerBodyAtPoint(body, body.getCenterOfMass());
   }
   
   public static RigidTransform3d centerBodyAtPoint (RigidBody body, Point3d pos) {
      body.setAxisLength (5);
      Point3d posl = new Point3d(pos);
      Point3d posw = new Point3d(pos);
      posw.transform (body.getPose().R);
      body.translateCoordinateFrame (posw);
      for (FrameMarker mrk : body.getFrameMarkers ()) {
         // System.out.println("transforming " + mrk.getName ());
         Point3d loc = new Point3d ();
         mrk.getLocation (loc);
         loc.sub (posl);
         mrk.setLocation (loc);
      }
      return body.getPose();
   }
   
   public void replaceSkullMeshes() {
      String meshName;
      String[] bodyNames = {"cranium", "maxilla", "jaw", "hyoid"};
      for (String name : bodyNames) {
         if (useSymmetricGeometry) {
            if (useTeethSmoothedGeometry && (name=="jaw" || name=="maxilla")) {
               meshName = regGeomDir+"ubc"+name+"_symmetric_teethsmoothed.obj";
            }
            else {
               meshName = regGeomDir+"ubc"+name+"_symmetric.obj";
            }
         }
         else
            meshName = regGeomDir+"ubc"+name+".obj";
         replaceMesh(myJawModel.rigidBodies ().get (name), meshName);
      }
      
      // remove other bodies, XXX - should find best rigid transform instead
      String[] bodiesToRemove = new String[]{"vertebrae", "pharynx"};
      for (String name : bodiesToRemove) {
         RigidBody body = myJawModel.rigidBodies ().get(name);
         if (body != null)
            myJawModel.removeRigidBody (body);
      }
   }
   
   
   protected void centerSymmetricDynamicBodies(String[] bodyNames) {
      ArrayList<RigidBody> bodiesToCenter = new ArrayList<RigidBody> ();
      for (String name : bodyNames) {
         if (myJawModel.rigidBodies ().get (name) != null)
            bodiesToCenter.add (myJawModel.rigidBodies ().get (name));
      }
      
      for (RigidBody body : bodiesToCenter) {
         // compute centre-of-mass from mesh
         body.setInertiaFromDensity (body.getDensity());
         // make com symmetric
         Point3d com = new Point3d(body.getCenterOfMass());
         com.y = 0.0;
         // align body-frame origin at center-of-mass
         RigidTransform3d X = BadinJawHyoid.centerBodyAtCOM (body);
         // re-compute inertia for new body pose (paranoid)
         // body.setInertiaFromMesh (density); Not needed: JL
         for (BodyConnector con : myJawModel.bodyConnectors ())
         {
            if (con.getBodyA () == body)
            {
               System.out.println("jaw con - " + con.getName ());
               con.transformGeometry (X);
            }
         }
      }
      
      myJawModel.updateCons ();
   }
	
   public void alignMarkers () {
      // jaw and hyoid mesh were first axis aligned, before being mirrored
      RigidTransform3d XBadinJawAligned = new RigidTransform3d ();
      try {
         String filename = regGeomDir + "XBadinJawToSymmetric.txt";
         ReaderTokenizer rtok =
            new ReaderTokenizer ((new FileReader (filename)));
         XBadinJawAligned.scan (rtok);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

      System.out.println ("X = \n" + XBadinJawAligned.toString ("%g"));

      String[] bodyNames = new String[] { "jaw", "hyoid" };
      for (String name : bodyNames) {
         for (FrameMarker mkr : myJawModel
            .rigidBodies ().get (name).getFrameMarkers ()) {
            mkr.transformGeometry (XBadinJawAligned);
         }
      }
   }

   public void projectMarkers () {
      // only project jaw and hyoid markers as maxilla mesh has been cropped
      // exclude some interior markers
      String[] excludeMarkers = new String[] {"tmj", "Com"};
      String[] bodyNames = new String[] { "jaw", "hyoid" };
      for (String name : bodyNames) {
         RigidBody body = myJawModel.rigidBodies ().get (name);
         PolygonalMesh mesh = body.getMesh ();
         for (FrameMarker m : body.getFrameMarkers ()) {
            if (m.getName () != null) {
               boolean exclude = false;
               for (String excluded : excludeMarkers) {
                  if (m.getName ().endsWith (excluded)) {
                     exclude = true;
                     break;
                  }
               }
               if (exclude)
                  continue;
            }
            Point3d proj = new Point3d ();
            Vector2d coords = new Vector2d ();
            BVFeatureQuery query = new BVFeatureQuery();
            query.nearestFaceToPoint (proj, coords, mesh, m.getPosition ());
            proj.inverseTransform (mesh.getMeshToWorld ());
            m.setLocation (proj);
         }
      }
   }

   public void mirrorMarkersRight () {

      // set left side to mirror right side, left side is -Y axis
      for (FrameMarker m : myJawModel.frameMarkers ()) {
         Point3d pos = new Point3d ();
         Point3d loc = new Point3d ();
         if (m.getName () == null) {
            continue;
         }
         String name = m.getName ();

         if (name.startsWith ("l")) {
            FrameMarker right = findMatchingRightMarker (name);
            if (right == null) {
               // found un-paired marker
               pos.set (m.getPosition ());
               pos.y = 0.0;
               loc.inverseTransform (m.getFrame ().getPose (), pos);
               m.setLocation (loc);
               m.setRefPos (pos);
               continue;
            }
            pos.set (right.getPosition ());
            if (pos.y < 0) { // right side marker in left hemisphere, flip
               pos.y = -pos.y;
               loc.inverseTransform (right.getFrame ().getPose (), pos);
               right.setLocation (loc);
               m.setRefPos (pos);
            }
            pos.y = -pos.y; // left pos
            loc.inverseTransform (m.getFrame ().getPose (), pos);
            m.setLocation (loc);
            m.setRefPos (pos);
         }
         else if (!name.startsWith ("r")
         || findMatchingLeftMarker (name) == null) {
            // found un-paired marker

            pos.set (m.getPosition ());
            pos.y = 0.0;
            loc.inverseTransform (m.getFrame ().getPose (), pos);
            m.setLocation (loc);
            m.setRefPos (pos);
         }
      }

   }
	
   public void mirrorMarkersLeft() {
      
      // set right side to mirror left side,left side is -Y axis
      for (FrameMarker m : myJawModel.frameMarkers ())
      {
         Point3d pos = new Point3d ();
         Point3d loc = new Point3d ();
         if (m.getName() == null) {
            continue;
         }
         String name = m.getName ();
         
         if (name.startsWith ("r")) {
            FrameMarker left = findMatchingLeftMarker (name);
            if (left == null) {
               // found un-paired marker
               pos.set (m.getPosition ());
               pos.y = 0.0;
               loc.inverseTransform (m.getFrame ().getPose (), pos);
               m.setLocation (loc);
               m.setRefPos (pos);
               continue;
            }
            pos.set (left.getPosition ());
            if (pos.y > 0) { // left pos
               pos.y = -pos.y;
               loc.inverseTransform (left.getFrame ().getPose (), pos);
               left.setLocation (loc);
               m.setRefPos (pos);
            }
            pos.y = -pos.y; // right pos
            loc.inverseTransform (m.getFrame ().getPose (), pos);
            m.setLocation (loc);
            m.setRefPos (pos);
         }
         else if (!name.startsWith ("l") || findMatchingRightMarker (name) == null) {
            // found un-paired marker
            
            pos.set (m.getPosition ());
            pos.y = 0.0;
            loc.inverseTransform (m.getFrame ().getPose (), pos);
            m.setLocation (loc);
            m.setRefPos (pos);
         }
      }
      
   }
   
   private FrameMarker findMatchingLeftMarker(String name) {
      
      FrameMarker match = null;
      String prefix = (name.startsWith ("right")?"left":"l");
      String suffix = name.substring ((name.startsWith ("right")?5:1));
      for (FrameMarker m : myJawModel.frameMarkers ()) {
         if (m.getName () != null 
             && m.getName ().startsWith (prefix)
             && m.getName ().endsWith (suffix)) {
            match = m;
            break;
         }
      }
      return match;
   }
   
   
   private FrameMarker findMatchingRightMarker(String name) {
      
      FrameMarker match = null;
      String prefix = (name.startsWith ("left")?"right":"r");
      String suffix = name.substring ((name.startsWith ("left")?4:1));
      for (FrameMarker m : myJawModel.frameMarkers ()) {
         if (m.getName () != null 
             && m.getName ().startsWith (prefix)
             && m.getName ().endsWith (suffix)) {
            match = m;
            break;
         }
      }
      return match;
   }
   
   
   public void updateJawConstraints() {
      // match bite constraint to badin teeth
      double biteAngle = 0; // JBiomech model was 10.0 because different head orientation
      double biteCant = 8; // cant to simulate inner cusp surface
      myJawModel.setRBiteAngle (biteAngle);
      myJawModel.setLBiteAngle (biteAngle);
      
      myJawModel.setRBiteCant (-biteCant);
      myJawModel.setLBiteCant (biteCant);
      
      myJawModel.setGlobalConRot (new AxisAngle(0,0,1,-Math.PI/2));
      myJawModel.updateCons ();
      
      RenderProps.setVisible(myJawModel.bodyConnectors ().get ("RBITE"), false);
      RenderProps.setVisible(myJawModel.bodyConnectors ().get ("LBITE"), false);
      
      
   }
   
   public void pruneUnnamedMarkers() {
      ArrayList<FrameMarker> noname = new ArrayList<FrameMarker> ();
      for (FrameMarker m : myJawModel.frameMarkers ()) {
         if (m.getName () == null) {
            noname.add (m);
         }
      }     
      for (FrameMarker m : noname) {
         myJawModel.removeFrameMarker (m);
      }  
   }
   
   
   public FrameMarker[] getSkullMarkers() {
      String[] skullNames = new String[]{"cranium","maxilla","jaw"};
      ArrayList<RigidBody> skullBodies = new ArrayList<RigidBody> ();
      ArrayList<FrameMarker> markers = new ArrayList<FrameMarker> ();
      for (String bodyName : skullNames) {
         skullBodies.add (myJawModel.rigidBodies().get(bodyName));
      }
      
      for (FrameMarker m : myJawModel.frameMarkers ()) {
         if (skullBodies.contains (m.getFrame ()))
            markers.add (m);
      }
      return markers.toArray (new FrameMarker[markers.size ()]);
   }
   
   public void updateMarkerPositions() {
//      BadinDataDemo.readMarkerPositions (getSkullMarkers(), 
//         regGeomDir + "ubcMarkers/ubcskull_markers.txt", m2mm);
//
//      BadinDataDemo.readMarkerPositions (
//         myJawModel.rigidBodies ().get ("hyoid").getFrameMarkers (), 
//         regGeomDir + "ubcMarkers/ubchyoid_markers.txt", m2mm);
      
      
      BadinDataDemo.readMarkerPositions (
         myJawModel.frameMarkers ().toArray (new FrameMarker[0]), 
         regGeomDir+"ubcmarkers.txt", m2mm);
      
   }
   
   public static void replaceMesh (RigidBody body, String fileName) {
      // assumes mesh coordinates are valid for body at world-frame
      body.setPose (new RigidTransform3d ());
      body.setInertiaFromDensity (body.getDensity());
      setMesh (body, fileName);
   }
   
   public void transformPharynx() {
     
      RigidTransform3d vertPose = new RigidTransform3d ();
      BadinDataDemo.readTransform (vertPose, "XUbcvertFace.txt");
      vertPose.p.scale (m2mm);
      
      String[] bodiesToTransform = new String[]{"vertebrae", "pharynx"};
      for (String bodyName : bodiesToTransform) {
         RigidBody body = myJawModel.rigidBodies ().get(bodyName);
         if (body != null) {
            body.scaleDistance (0.95);
            body.transformGeometry (vertPose);
         }
      }
      
   }
   public void transformLarynx() {
      double thyroidScaling = 1.1;
      RigidTransform3d thyroidPose = new RigidTransform3d ();
      BadinDataDemo.readTransform (thyroidPose, "XUbcthyroidFace.txt");
      thyroidPose.p.scale (m2mm);
      
      String[] bodiesToTransform = new String[]{"thyroid", "cricoid", "sternum"};
      for (String bodyName : bodiesToTransform) {
         RigidBody body = myJawModel.rigidBodies ().get(bodyName);
         if (body != null) {
            body.scaleDistance (thyroidScaling);
            for (FrameMarker m : body.getFrameMarkers ()) {
               m.scaleDistance (thyroidScaling);
            }
//            body.setPose (thyroidPose);
            body.transformGeometry (thyroidPose);
         }
      }
      
      myJawModel.bodyConnectors ().get ("cricothyroid").transformGeometry (thyroidPose);

     
   }
   
//   public void addPalate() {
//      addBody("palate", regGeomDir+"badinpalate_face.mdl");
//   }

   public RigidBody addBody (String name, String fileName) {
      RigidBody body = myJawModel.rigidBodies ().get (name);
      if (body == null) {
         body = new RigidBody (name);
         body.setDynamic (false);
         setMesh (body, fileName);
         myJawModel.addRigidBody (body);
      }
      return body;
   }
   
   public static void setMesh(RigidBody body, String fileName) {
      try {
         PolygonalMesh mesh = null;
         if (fileName.endsWith (".mdl")) {
            mesh = MDLMeshIO.read (fileName, new Vector3d (m2mm,m2mm,m2mm));
         }
         else { // obj file
            mesh = new PolygonalMesh (new File(fileName));
            mesh.scale (m2mm);
         }
         if (mesh != null) {
            AffineTransform3d Xscaling = new AffineTransform3d();
            Xscaling.setScaling (m2mm, m2mm, m2mm);
            body.setMesh (mesh, fileName, Xscaling);
            if (mesh.getTextureCoords() == null || 
                mesh.getTextureCoords().size() == 0) {
               RenderProps.setColorMapEnabled (body, false);
            }
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   public void setRestPosture () {
      setRestPosture ("tongue_rest_il01.txt");
   }
   
   public void setRestPosture (String tongueRestNodesFilename) {
      System.out.println("setting jaw rest posture");
      setJawPosture (jawRestPose_symmetric);

      if (myJawModel.models ().size () > 0  && myJawModel.models ().get (0) instanceof FemMuscleModel) {
	      System.out.println("setting tongue rest posture");
	 FemMuscleModel tongue = (FemMuscleModel)myJawModel.models ().get (0);
	 HexTongueDemo.setTonguePosture (tongue, regGeomDir + tongueRestNodesFilename, /*isRestPosture= */true);
      }
   }
   
   public void setIntercuspalPosture () {
//      setRestPosture ();
      setJawPosture(jawIPPose_symmetric);
      

      if (myJawModel.models ().size () > 0  && myJawModel.models ().get (0) instanceof FemMuscleModel) {
	 FemMuscleModel tongue = (FemMuscleModel)myJawModel.models ().get (0);
	 HexTongueDemo.setTonguePosture (tongue, regGeomDir + "tongue_ip.txt", false/*dont reset rest pos*/);
      }
   }

   
   public void setEEPosture () {
//    setRestPosture ();
    setJawPosture(jawEEPose_symmetric);
    setBodyPosture (hyoidEEPose, "hyoid");

    if (myJawModel.models ().size () > 0  && myJawModel.models ().get (0) instanceof FemMuscleModel) {
       FemMuscleModel tongue = (FemMuscleModel)myJawModel.models ().get (0);
       HexTongueDemo.setTonguePosture (tongue, regGeomDir + "tongue_ee.txt", false/*dont reset rest pos*/);
    }
 }
   
   
   public void setJawPosture(RigidTransform3d XJawToWorld) {
      setBodyPosture (XJawToWorld, "jaw");
   }
   
   public void setBodyPosture(RigidTransform3d XJawToWorld, String bodyName) {
      RigidBody body = myJawModel.rigidBodies ().get (bodyName);
      if (body != null && XJawToWorld != null) {
         body.setPose (XJawToWorld);
         myJawModel.updatePosState ();
      }
   }
   
   public double[] getContourTols() {
      return tols;
   }
   double[] tols = new double[]{5.0, 0.1};

   // assume mid-sagittal plane is y=0.0
   protected double distToMidSagittal(Vertex3d v) {
      return Math.abs (v.getPosition ().y);
   }
   
   protected void projectToMidSagittal(Point3d mspos, Point3d pos) {
      mspos.set (pos);
      mspos.y = 0.0;
   }
   
}
