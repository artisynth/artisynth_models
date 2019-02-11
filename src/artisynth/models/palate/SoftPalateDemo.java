package artisynth.models.palate;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.RenderableUtils;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.UCDReader;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.Model;
import artisynth.core.util.AmiraLandmarkReader;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

public class SoftPalateDemo extends RootModel {

   boolean addMusclesAuto = false;
   MechModel myMechMod;
   FemMuscleModel myFemMod;
   double myDensity = 5.0;
   double muscleMaxForce = 1000;
   
   ArrayList<String> bundleNames = new ArrayList<String> ();
   
   public static final String palateGeomPath = ArtisynthPath.getSrcRelativePath (
      SoftPalateDemo.class, "geometry/");

   public SoftPalateDemo () throws IOException {
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      
      
      myMechMod = new MechModel ("mech");
      myFemMod = new FemMuscleModel ("palate");
         UCDReader.read (myFemMod, palateGeomPath + "softPalate.inp", myDensity);

      myFemMod.setSurfaceRendering (SurfaceRender.Shaded);
      myFemMod.setLinearMaterial (100000, 0.33, true); // should use 10 kPa from Payan BBE2001 paper
      myFemMod.setStiffnessDamping (0.1);
      
      // fix the top of the palate
      Point3d min = new Point3d ();
      Point3d max = new Point3d ();
      RenderableUtils.getBounds (myFemMod, min, max);
      double tol = (max.z - min.z) * 0.05;

      for (FemNode3d n : myFemMod.getNodes ()) {
         if (n.getPosition ().z >= max.z - tol) {
            n.setDynamic (false);
            RenderProps.setPointStyle (n, PointStyle.SPHERE);
            RenderProps.setPointColor (n, Color.LIGHT_GRAY);
         }
      }

      myMechMod.addModel (myFemMod);
      myMechMod.setIntegrator (Integrator.BackwardEuler);
      addModel (myMechMod);
      
      RigidBody airway = addBody("airway", "airwayIan.obj");
      RenderProps.setFaceColor (airway, new Color(0.6f,0.6f,0.9f));
      
      RigidBody tongue = addBody("tongue", "tongueIan.obj");
      RenderProps.setFaceColor (tongue, new Color(0.6f,0.9f,0.6f));
      
      if (addMusclesAuto)
         addMusclesFromAmiraLandmarks ();
      else {
         addMarkersFromAmiraLandmarks ();
         addMuscleBundleFromSavedIndices ();
      }
      
      for (MuscleBundle b : myFemMod.getMuscleBundles()) {
	 b.setFibresActive(true);
      }

      RenderProps.setFaceColor (myFemMod, Color.PINK);
       RenderProps.setFaceStyle (myFemMod, FaceStyle.FRONT);
       RenderProps.setDrawEdges (myFemMod, true);
      // RenderProps.setPointStyle (myFemMod, PointStyle.SPHERE);
       RenderProps.setPointStyle (myFemMod.markers (), PointStyle.SPHERE);
      RenderProps.setPointRadius (myFemMod, 0.3);
      RenderProps.setPointColor (myFemMod, Color.BLUE);
      RenderProps.setLineStyle (myFemMod, LineStyle.SPINDLE);
      RenderProps.setLineColor (myFemMod, Color.DARK_GRAY);
      RenderProps.setLineRadius (myFemMod, 0.5);

      ControlPanel panel = new ControlPanel ("options");
      FemControlPanel.addFemControls (panel, myFemMod, myMechMod);
      addControlPanel (panel);
      
      ControlPanel musclePanel = new ControlPanel ("muscles");
      FemControlPanel.addBundleControls(musclePanel, myFemMod);
      addControlPanel (musclePanel);
   }
   
   public RigidBody addBody (String name, String fileName) {
      return addBody (name, fileName, 1d);
   }
   
   public RigidBody addBody (String name, String fileName, double scale) {
      RigidBody body = new RigidBody (name);
      body.setDynamic (false);
      String fullMeshFileName = palateGeomPath + fileName;
      addMesh (body, fullMeshFileName, scale);
      myMechMod.addRigidBody (body);
      return body;
   }
   
   private void addMesh (RigidBody body, String meshname, double scale) {
      try {
         PolygonalMesh mesh = new PolygonalMesh (new File (meshname));
         mesh.scale (scale);
         body.setMesh (mesh, meshname);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   private void addMusclesFromAmiraLandmarks () {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (this, "landmarks"));
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            if (fileName.endsWith (".landmarkAscii")) {
               addMuscleFromFile (directory + "/" + fileName);
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }
   }

   private FemMarker createAndAddMarker (FemMuscleModel fem, Point3d pnt) {
      FemMarker marker = new FemMarker();

      // add the marker to the model
      FemElement3dBase elem = fem.findContainingElement (pnt);
      if (elem == null) {
         Point3d newLoc = new Point3d();
         elem = fem.findNearestSurfaceElement (newLoc, pnt);
         pnt.set (newLoc);
      }
      marker.setPosition (pnt);
      marker.setFromElement (elem);
      fem.addMarker (marker, elem);
      return marker;
   }
   
   private Muscle createAndAddFibre ( 
      Point pointA, Point pointB, MuscleBundle bundle) {
      Muscle fibre = new Muscle();
      fibre.setConstantMuscleMaterial(1, 1);
      fibre.setFirstPoint (pointA);
      fibre.setSecondPoint (pointB);
      bundle.addFibre (fibre);
      return fibre;
   }
   
   private MuscleBundle createAndAddMuscleBundle(FemMuscleModel fem, Point3d[] pts) {
      MuscleBundle bundle = new MuscleBundle ();
      FemMarker cur, prev = null;
      for (int i = 0; i < pts.length; i++) {
         cur = createAndAddMarker (fem, pts[i]);
         if (i > 0)
            createAndAddFibre (prev, cur, bundle);
         prev = cur;
      }
      fem.addMuscleBundle (bundle);
      return bundle;  
   }
   
   private void addMuscleFromFile (String fileName) {
      try {
         Point3d[] pts = AmiraLandmarkReader.read (fileName);
         MuscleBundle b = createAndAddMuscleBundle (myFemMod, pts);
         b.setMaxForce (muscleMaxForce);
         Vector3d c = new Vector3d();
         c.setRandom (0, 1);
         RenderProps.setLineColor (b, new Color((float)c.x,(float)c.y,(float)c.z));
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   private void addMarkersFromAmiraLandmarks () {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (this, "landmarks"));
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            if (fileName.endsWith (".landmarkAscii")) {
               addMarkersFromFile (directory + "/" + fileName);
               bundleNames.add (fileName.substring (
                  0, fileName.length ()-".landmarkAscii".length ()));
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }
   }
   
   private void addMuscleBundleFromSavedIndices () {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (this, "landmarks"));
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            if (fileName.startsWith ("bundle") && fileName.endsWith (".txt")) {
               addMuscleBundleFromIndexFile (directory + "/" + fileName);
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }
   }
   
   private void addMuscleBundleFromIndexFile (String fileName) {
      try {
         Integer[][] indices = readMuscleIndices(fileName);
         MuscleBundle bundle = new MuscleBundle ();
         int bundleNum = myFemMod.getMuscleBundles ().size();
         if (bundleNum < bundleNames.size ()) {
            bundle.setName (bundleNames.get (bundleNum));
         }
         myFemMod.addMuscleBundle (bundle);
         for (Integer[] ptidx : indices) {
            Muscle fibre = new Muscle ();
            fibre.setFirstPoint (myFemMod.markers ().get (ptidx[0]));
            fibre.setSecondPoint (myFemMod.markers ().get (ptidx[1]));
            bundle.addFibre (fibre);
         }
         bundle.setMaxForce (muscleMaxForce);
         Vector3d c = new Vector3d();
         c.setRandom (0, 1);
         RenderProps.setLineColor (bundle, new Color((float)c.x,(float)c.y,(float)c.z));
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   private Integer[][] readMuscleIndices(String fileName) throws IOException {
      ArrayList<Integer[]> indices = new ArrayList<Integer[]> ();
      
      ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (fileName));
      while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
         rtok.pushBack ();
         int p0 = rtok.scanInteger ();
         int p1 = rtok.scanInteger ();
         indices.add (new Integer[]{p0,p1});
      }
      return indices.toArray (new Integer[indices.size ()][]);      
   }

   private void addMarkersFromFile (String fileName) {
      try {
         Point3d[] pts = AmiraLandmarkReader.read (fileName);
         for (Point3d pt : pts) {
            FemMarker m = new FemMarker (pt);
            myFemMod.addMarker (m);
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   public void createVisibilityPanel() {
      
      if (myMechMod == null)
         return;
      ControlPanel panel = new ControlPanel ("Show");
      for (Model mod : myMechMod.models ()) {
         panel.addWidget (mod.getName (), mod, "renderProps.visible");
      }
      for (RigidBody body : myMechMod.rigidBodies ()) {
         panel.addWidget (body.getName (), body, "renderProps.visible");
      }
      addControlPanel (panel);
   }
   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);
      Main.getMain ().arrangeControlPanels (this);
      ArtisynthPath.setWorkingDir (new File(
         ArtisynthPath.getSrcRelativePath (SoftPalateDemo.class, "data/")));
   }

}
