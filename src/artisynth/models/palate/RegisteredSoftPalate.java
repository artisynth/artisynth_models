package artisynth.models.palate;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JFrame;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.TetGenReader;
import artisynth.core.femmodels.UCDReader;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.util.AmiraLandmarkReader;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.template.ModelTemplate;
import artisynth.models.tongue3d.FemMuscleTongueDemo;
import artisynth.models.tongue3d.HexTongueDemo;

public class RegisteredSoftPalate extends RootModel {
   static boolean addMusclesAuto = false;
   public static final double defaultMaxStepSizeSec = 0.05;
   public static final Integrator defaultIntegrator =
      Integrator.BackwardEuler;
   static double muscleMaxForce = 3;
   
   static ArrayList<String> bundleNames;
   MechModel myMechMod;
   FemMuscleModel myFemMod;
   
   public static final String softPalateGeometryDir =
      ArtisynthPath.getSrcRelativePath (
 	      ModelTemplate.class, "geometry/fem/palate/");
   
   public RegisteredSoftPalate () {
      super ();
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      

      // create mech model
      myMechMod = new MechModel ("mech");
      myMechMod.setMaxStepSize (defaultMaxStepSizeSec);
      myMechMod.setIntegrator (defaultIntegrator);
      addModel (myMechMod);
      
      myFemMod = createSoftPalate(softPalateGeometryDir, "ianMesh_mod_simp", "landmarks_registered",false);
      myFemMod.setName ("ianSoftPalate");
      myMechMod.addModel (myFemMod);
      
      setSoftPalateRendering (myFemMod);
      
   }
   public static FemMuscleModel createSoftPalate(String meshDirectory,
	 String meshFilePrefix, String landmarkPath, boolean linearMaterial) {
      return createSoftPalate(meshDirectory, meshFilePrefix, landmarkPath, 1d, linearMaterial);
   }
   public static FemMuscleModel createSoftPalate(String meshDirectory,
	 String meshFilePrefix, String landmarkPath, double scale, boolean linearMaterial) {

      FemMuscleModel fem = new FemMuscleModel (meshFilePrefix);
      bundleNames = new ArrayList<String> ();
      try {
	 TetGenReader.read (
               fem, fem.getDensity(), meshDirectory + meshFilePrefix + ".node", meshDirectory + meshFilePrefix + ".ele",
               new Vector3d (1, 1, 1));
      } catch (Exception e) {
	 try {
		 AnsysReader.read (
		       fem, meshDirectory + meshFilePrefix +".node", meshDirectory + meshFilePrefix + ".elem", 
	               1, null, /*options=*/0);
	 } catch (Exception e2) {
	    try {
	       	 UCDReader.read (fem, meshDirectory + meshFilePrefix + ".inp", 1);
	    } catch (Exception e3) {
	       e.printStackTrace ();
	       e2.printStackTrace ();
	       e3.printStackTrace ();
	    }
	 }
      }
      
      if (linearMaterial) {
         fem.setMaterial (new LinearMaterial ());
      }
      else {
         fem.setMaterial (new MooneyRivlinMaterial (1.0370,0,0,0.486,0,10.370));
      }
      
      setSoftPalateProperties (fem);
      
      if(!landmarkPath.isEmpty()) {
	 if (addMusclesAuto)
	         addMusclesFromAmiraLandmarks (fem, landmarkPath);
	 else {
	         addMarkersFromAmiraLandmarks (fem, landmarkPath);
	         addMuscleBundleFromSavedIndices (fem, landmarkPath);
	 }
         for (MuscleBundle b : fem.getMuscleBundles()) {
   	 	b.setFibresActive(true);
         }
      }
      return fem;
   }
   public static void setSoftPalateProperties (FemModel3d fem) {
//      fem.setYoungsModulus (500);
//      fem.setPoissonsRatio(0.33000);
      fem.setLinearMaterial (500, 0.33, true);
      fem.setDensity(0.000001040);
      fem.setParticleDamping(40.00);
      fem.setStiffnessDamping (0.03);
      
      fem.setIncompressible (FemModel.IncompMethod.AUTO);
      fem.setIntegrator (defaultIntegrator);
      fem.setMaxStepSize (defaultMaxStepSizeSec);
      //fem.setImplicitIterations (10);
      //fem.setImplicitPrecision (0.001);
   }
   public static void setSoftPalateRendering (FemModel3d fem) {
      // Palate Rendering
      RenderProps.setVisible (fem.getNodes (), true);
      RenderProps.setVisible(fem.getElements(), false);
      RenderProps.setVisible(fem.getNodes(), false);
      
      fem.setSurfaceRendering (SurfaceRender.Shaded);
      fem.setElementWidgetSize (1.0);
      RenderProps.setFaceColor (fem, Color.PINK);
      RenderProps.setFaceStyle (fem, FaceStyle.NONE);
      RenderProps.setDrawEdges (fem, true);
      RenderProps.setPointStyle (fem, PointStyle.SPHERE);
      RenderProps.setLineWidth(fem, 1);
      RenderProps.setPointRadius (fem, 0.3);
      RenderProps.setPointColor (fem, Color.BLUE);
      RenderProps.setLineStyle (fem, LineStyle.SPINDLE);
      RenderProps.setLineColor (fem, Color.LIGHT_GRAY);
      RenderProps.setLineRadius (fem, 0.8);
   }
   private static void addMusclesFromAmiraLandmarks (FemMuscleModel fem, String landmarkPath) {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (RegisteredSoftPalate.class, landmarkPath));
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            if (fileName.endsWith (".landmarkAscii")) {
               bundleNames.add (fileName.substring (
                     0, fileName.length ()-".landmarkAscii".length ()));
               addMuscleFromFile (fem, directory + "/" + fileName);
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }
   }
   private static void addMuscleFromFile (FemMuscleModel fem, String fileName) {
      try {
         Point3d[] pts = AmiraLandmarkReader.read (fileName);
         MuscleBundle b = createAndAddMuscleBundle (fem, pts);
         b.setMaxForce (muscleMaxForce);
         Vector3d c = new Vector3d();
         c.setRandom (0, 1);
         RenderProps.setLineColor (b, new Color((float)c.x,(float)c.y,(float)c.z));
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   private static MuscleBundle createAndAddMuscleBundle(FemMuscleModel fem, Point3d[] pts) {
      MuscleBundle bundle = new MuscleBundle ();
      FemMarker cur, prev = null;
      for (int i = 0; i < pts.length; i++) {
         cur = createAndAddMarker (fem, pts[i]);
         if (i > 0)
            createAndAddFibre (prev, cur, bundle);
         prev = cur;
      }
      int bundleNum = fem.getMuscleBundles ().size();
      if (bundleNum < bundleNames.size ()) {
         bundle.setName (bundleNames.get (bundleNum));
      }
      fem.addMuscleBundle (bundle);
      return bundle;  
   }
   private static FemMarker createAndAddMarker (FemMuscleModel fem, Point3d pnt) {
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
   
   private static Muscle createAndAddFibre ( 
      Point pointA, Point pointB, MuscleBundle bundle) {
      Muscle fibre = new Muscle();
      fibre.setFirstPoint (pointA);
      fibre.setSecondPoint (pointB);
      ConstantAxialMuscle mat = new ConstantAxialMuscle();
      mat.setForceScaling(muscleMaxForce); // XXX should this be setMaxForce ??
      fibre.setMaterial(mat);
      bundle.addFibre (fibre);
      return fibre;
   }
   private static void addMarkersFromAmiraLandmarks (FemMuscleModel fem, String landmarkPath) {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (RegisteredSoftPalate.class, landmarkPath));
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            if (fileName.endsWith (".landmarkAscii")) {
               bundleNames.add (fileName.substring (
                  0, fileName.length ()-".landmarkAscii".length ()));
               addMarkersFromFile (fem, directory + "/" + fileName);
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }
   }
   private static void addMarkersFromFile (FemMuscleModel fem, String fileName) {
      try {
         Point3d[] pts = AmiraLandmarkReader.read (fileName);
         String bundleName = bundleNames.get (bundleNames.size()-1);
         int i=0;
         for (Point3d pt : pts) {
            FemMarker m = new FemMarker (pt);
            m.setName(bundleName+ "_" + i);
            fem.addMarker (m);
            i++;
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   private static void addMuscleBundleFromSavedIndices (FemMuscleModel fem, String landmarkPath) {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (RegisteredSoftPalate.class, landmarkPath));
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            for(String bundleName: bundleNames) {
               if (fileName.startsWith (bundleName) && fileName.endsWith (".bundle")) {
                  addMuscleBundleFromIndexFile (fem, directory + "/" + fileName);
               }
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }
   }
   
   private static void addMuscleBundleFromIndexFile (FemMuscleModel fem, String fileName) {
      try {
         Integer[][] indices = readMuscleIndices(fileName);
         MuscleBundle bundle = new MuscleBundle ();
         int bundleNum = fem.getMuscleBundles ().size();
         if (bundleNum < bundleNames.size ()) {
            bundle.setName (bundleNames.get (bundleNum));
         }
         fem.addMuscleBundle (bundle);
         String bundleName = bundle.getName();
         for (Integer[] ptidx : indices) {
            Muscle fibre = new Muscle ();
            fibre.setFirstPoint (fem.markers ().get(bundleName + "_" + ptidx[0]));
            fibre.setSecondPoint (fem.markers ().get (bundleName + "_" + ptidx[1]));
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
   
   private static Integer[][] readMuscleIndices(String fileName) throws IOException {
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
   public static ControlPanel createMusclePanel(RootModel root,
	 FemMuscleModel fem) {
      ControlPanel controlPanel = new ControlPanel("Palate Muscles", "LiveUpdate");
      controlPanel.setScrollable(true);
      FemControlPanel.addBundleControls(controlPanel, fem);
      root.addControlPanel(controlPanel);
      return controlPanel;
   }
   public static ControlPanel createControlPanel(RootModel root,
	 FemMuscleModel fem, ModelComponent topModel) {
      ControlPanel controlPanel = new ControlPanel("Palate options", "LiveUpdate");
      controlPanel.setScrollable(true);
      FemControlPanel.addMuscleControls(controlPanel, fem, topModel);
      controlPanel.addWidget("elements visible", fem,  "elements:renderProps.visible");
      controlPanel.addWidget("muscles visisble", fem, "bundles:renderProps.visible");
      root.addControlPanel(controlPanel);
      return controlPanel;
   }
   ControlPanel myControlPanel = null;
   public void attach (DriverInterface driver) {
      super.attach (driver);
      
      myControlPanel = RegisteredSoftPalate.createControlPanel (this, myFemMod, myMechMod);
      if (myFemMod.getMuscleBundles().size()>0)
	 RegisteredSoftPalate.createMusclePanel(this, myFemMod);
   }
   public static final void anchorPalate(FemModel3d myFemMod, MechModel myMechMod)
   {
      // the tensor veli palatini attaches to the hamulus
      Particle hamulus_R = new Particle(1.0, 111.0, 25.0, 127.0);
      Particle hamulus_L = new Particle(1.0, 111.0, -25.0, 127.0);
      hamulus_R.setDynamic(false);
      hamulus_L.setDynamic(false);
      myMechMod.addParticle(hamulus_R);
      myMechMod.addParticle(hamulus_L);
      
      // the levator veli palatini --> for now I just make all nodes above a cutoff point static, thus anchoring levator
      for (FemNode3d node: myFemMod.getNodes()) 
      {
	 double zValue = 150.0;	// all nodes above the plane z=115.0 are inactive for now
	 if (node.getPosition().z > zValue) 
	    node.setDynamic(false);
      }
      
      // the palatoglossus should attach right at the tongue
      	// perhaps the nodes (R: node 425,412,413 ; L: node 56,84,70) should be attached to the tongue?
      
      // the palatopharyngeus attaches to the pharyngeal wall
      	// for now just anchored statically
      Particle pharAnchor_R = new Particle(1.0, 140.0, 18.0, 75.0);
      Particle pharAnchor_L = new Particle(1.0, 140.0, -18.0, 75.0);
      pharAnchor_R.setDynamic(false);
      pharAnchor_L.setDynamic(false);
      myMechMod.addParticle(pharAnchor_R);
      myMechMod.addParticle(pharAnchor_L);
      
      // finally, the soft palate should be attached to the hard palate
      	// for now, if we just anchor all nodes for x<98.0, that will do a decent job of anchoring the front edge
      for (FemNode3d node: myFemMod.getNodes()) 
      {
	 double xValue_front = 96.5;
	 double zValue = 121.0;
	 if ((node.getPosition().x < xValue_front && node.getPosition().z > zValue)) 
	    node.setDynamic(false);
      }
      
      for (FemNode3d node: myFemMod.getNodes()) 
      {
	 if (!node.isDynamic()) {
	    RenderProps.setPointColor(node, Color.GRAY);
	 }
      }
   }
}
