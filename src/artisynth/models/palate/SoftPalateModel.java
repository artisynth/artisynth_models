package artisynth.models.palate;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JFrame;

import maspack.geometry.*;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.*;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.mechmodels.*;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.*;
import artisynth.core.util.*;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.face.BadinFaceDemo;
import artisynth.models.jawTongue.JawHyoidFemMuscleTongue;

public class SoftPalateModel extends JawHyoidFemMuscleTongue 
{
   public static final double defaultMaxStepSizeSec = 0.01; // DBG
   public static final Integrator defaultIntegrator = Integrator.ConstrainedBackwardEuler;
   
   public MechModel myMechModel;
   public FemMuscleModel softPalate;
   public FemMuscleModel tongue;
   
   public String geometryName = "softPalate_v2_smoothed";
   public String geometryPath = ArtisynthPath.getSrcRelativePath ( SoftPalateModel.class, "geometry/");
   public String landmarkPath = "landmarks_v4/";
   public String dataPath     = ArtisynthPath.getSrcRelativePath ( SoftPalateModel.class, "data_v3/");
   public String probesFilename = "probes.art";
   
   public static final String Material_linear     = "lnMat";
   public static final String Material_neohookean = "nhMat";
   public static final String Material_mooney     = "mrMat";
   public String materialType = Material_linear;
   
   public ArrayList<String> bundleNames;
   public double muscleMaxForce = 3;
   public boolean useMuscles = true;
   public boolean addMusclesAuto = false;
   public boolean anchorPalate = true;
   public boolean useControls = false;
   public boolean useProbes = true;
   
   public SoftPalateModel () 
   {
      super ();
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      super.workingDirname = "data_v3/";
      super.setWorkingDir();

      // create mech model
      //myMechMod = new MechModel ("mech");
      myMechModel = this.myJawModel;
      myMechModel.setMaxStepSize (defaultMaxStepSizeSec);
      myMechModel.setIntegrator (defaultIntegrator);
      
      tongue = (FemMuscleModel)myMechModel.models().get("tongue");
      
      softPalate = createSoftPalate();
      setRendering();
      
      //loadSkull();
      
      myMechModel.setCollisionBehavior(tongue, softPalate, true);
      
      //myMechModel.scaleDistance(1.0/1000.0);		// crash! fix this!
      // DBG
      myMechModel.setProfiling (true);
   }
   
   public FemMuscleModel createSoftPalate() 
   {
      // TODO: this needs to be reworked
      FemMuscleModel fem = readFemMesh(geometryPath, geometryName);
      fem.setName("softPalate");
      myMechModel.addModel (fem);
      
      setSoftPalateProperties (fem);
      
      if (useMuscles == true) {
	 addInternalMuscles(fem);
      }
      if (anchorPalate == true) {
	 anchorPalate(fem, tongue, myMechModel);
      }
      return fem;
   }
   
   public static FemMuscleModel readFemMesh(String meshDir, String meshName)
   {
      FemMuscleModel fem = new FemMuscleModel ();
      try 
      {
	 TetGenReader.read ( fem, fem.getDensity(), meshDir + meshName + ".node", meshDir + meshName + ".ele", new Vector3d (1, 1, 1));
      } 
      catch (Exception e) 
      {
	 try 
	 {
	    AnsysReader.read ( fem, meshDir + meshName +".node", meshDir + meshName + ".elem", 1, null, /*options=*/0);
	 } 
	 catch (Exception e2) 
	 {
	    try 
	    {
	       	 UCDReader.read (fem, meshDir + meshName + ".inp", 1);
	    } 
	    catch (Exception e3) 
	    {
	       e.printStackTrace ();
	       e2.printStackTrace ();
	       e3.printStackTrace ();
	    }
	 }
      }
      return fem;
   }
   
   public void setSoftPalateProperties (FemModel3d fem) 
   {
      double E = 15000.0/1000.0;			// Youngs modulus --> 15000 Pa = 15000 kg/(m*s^2) * (1m/1000mm) = 15000/1000 = 15 kg/(mm*s^2)
      double nu = 0.49;					// poisson ratio
      double K = E/(3.0*(1.0-2.0*nu));			// bulk modulus  --> from wiki
      double G = E/(2.0*(1.0+nu));			// shear modulus --> from wiki
      double rho = 1040.0/(1000.0*1000.0*1000.0);	// density --> 1040 kg/m^3 * (1m/1000mm)^3 = 1040/1,000,000,000 = 0.000001040 kg/mm^3
      boolean isIncompressible = false;			//
      
      FemMaterial mat;
      if      ( materialType.equalsIgnoreCase(SoftPalateModel.Material_linear) == true )
      {
	 LinearMaterial lnMat = new LinearMaterial();
	 lnMat.setPoissonsRatio(nu);
	 lnMat.setYoungsModulus(E);
	 mat = lnMat;
      }
      else if ( materialType.equalsIgnoreCase(SoftPalateModel.Material_neohookean) == true )
      {
	 NeoHookeanMaterial nhMat = new NeoHookeanMaterial();
	 nhMat.setPoissonsRatio(nu);
	 nhMat.setYoungsModulus(E);
	 mat = nhMat;
      }
      else
      {
	 MooneyRivlinMaterial mrMat = new MooneyRivlinMaterial();	// c10=1037, c20=486, bm=10*c10
	 mrMat.setBulkModulus (K);
	 mrMat.setC01(G/2.0 - K/10.0);
	 mrMat.setC10 (K/10.0);                                   // Pa  [Buchaillard 2009],[Duck 1990]
	 //mrMat.setC20 (486);                                    // Pa
	 //mrMat.setC11();
	 //mrMat.setC02();
	 mat = mrMat;
      }

      fem.setMaterial(mat);
      fem.setDensity(rho);
      if (isIncompressible) {
         fem.setIncompressible (IncompMethod.AUTO);         
      }
      else {
         fem.setIncompressible (IncompMethod.OFF);
      }      
      
      // TODO: the below doesn't belong here
      fem.setParticleDamping(1.00);
      fem.setStiffnessDamping (0.03);
      
      
      //fem.setIntegrator (defaultIntegrator);
      //fem.setMaxStepSizeSec (defaultMaxStepSizeSec);
      //fem.setImplicitIterations (10);
      //fem.setImplicitPrecision (0.001);
   }
   
   
   public void setRendering()
   {
      for(BodyConnector c: myMechModel.bodyConnectors()) {
	 RenderProps.setVisible(c, false);
      }
      RenderProps.setAlpha(myMechModel.rigidBodies().get("maxilla"), 0.2);
      RenderProps.setAlpha(myMechModel.rigidBodies().get("jaw"), 0.2);
      
      RenderProps.setVisible(myMechModel.rigidBodies().get("maxilla"), false);
      RenderProps.setVisible(myMechModel.rigidBodies().get("jaw"), true);
      RenderProps.setVisible(myMechModel.axialSprings(), false);
      RenderProps.setVisible(myMechModel.multiPointSprings(), true);
      RenderProps.setVisible(myMechModel.particles(), true);
      myMechModel.multiPointSprings().get(0).getRenderProps().setVisible(false);
      myMechModel.multiPointSprings().get(1).getRenderProps().setVisible(false);
      myMechModel.multiPointSprings().getRenderProps().setLineRadius(0.3);
      
      setSoftPalateRendering(softPalate);
   }
   
   public void setSoftPalateRendering (FemModel3d fem) {
      // Palate Rendering
      fem.getNodes().getRenderProps().setVisible(true);
      fem.getElements().getRenderProps().setVisible(false);
      fem.markers().getRenderProps().setVisible(true);
      fem.markers().getRenderProps().setPointColor(Color.green);
      
      fem.setSurfaceRendering   (SurfaceRender.Shaded);
      fem.setElementWidgetSize  (1.0);
      RenderProps.setFaceColor  (fem, Color.PINK);
      RenderProps.setFaceStyle  (fem, FaceStyle.NONE);
      RenderProps.setDrawEdges  (fem, true);
      RenderProps.setPointStyle (fem, PointStyle.SPHERE);
      RenderProps.setLineWidth  (fem, 1);
      RenderProps.setPointRadius(fem, 0.3);
      RenderProps.setPointColor (fem, Color.BLUE);
      RenderProps.setLineStyle  (fem, LineStyle.SPINDLE);
      RenderProps.setLineColor  (fem, Color.LIGHT_GRAY);
      RenderProps.setLineRadius (fem, 0.8);
      
      for (FemNode3d node: fem.getNodes()) 
      {
	 if (!node.isDynamic()) {
	    RenderProps.setPointColor(node, Color.cyan);
	 }
      }
   }
   
   public void anchorPalate(FemMuscleModel softPalate, FemModel3d tongue, MechModel mechMod)
   {
      // This is absolutely un-general code: it only works for the softPalate_v2_smoothed and hex tongue
      
      // the tensor veli palatini attaches to the hamulus
      FemMarker hmR1 = new FemMarker(117.0, 15.0, 125.0);
      FemMarker hmR2 = new FemMarker(112.0, 15.0, 125.0);
      FemMarker hmL1 = new FemMarker(117.0, -15.0, 125.0);
      FemMarker hmL2 = new FemMarker(112.0, -15.0, 125.0);
      Particle hamulus_R = new Particle(1.0, 109.0,  21.0, 117.0); // old 111.0, 25.0, 127.0
      Particle hamulus_L = new Particle(1.0, 109.0, -21.0, 117.0);
      hamulus_R.setDynamic(false);
      hamulus_L.setDynamic(false);
      softPalate.addMarker(hmR1);
      softPalate.addMarker(hmR2);
      softPalate.addMarker(hmL1);
      softPalate.addMarker(hmL2);
      mechMod.addParticle(hamulus_R);
      mechMod.addParticle(hamulus_L);
      MultiPointMuscle vpmR = createMultiPointMuscle("velipalatini_R", hmR1, hamulus_R, hmR2);
      MultiPointMuscle vpmL = createMultiPointMuscle("velipalatini_L", hmL1, hamulus_L, hmL2);
      mechMod.addMultiPointSpring(vpmR);
      mechMod.addMultiPointSpring(vpmL);
      
      // Levator veli palatini --> for now I just make all nodes above a cutoff point static, thus anchoring levator
      for (FemNode3d node: softPalate.getNodes()) 
      {
	 double zValue = 150.0;	// all nodes above the plane z=150.0 are inactive for now
	 if (node.getPosition().z > zValue) 
	    node.setDynamic(false);
      }

      // Palatoglossus --> should attach right at the tongue 
      if (tongue != null)
      {
	 int nR1 = 51;		// attachment nodes on the soft palate
	 int nR2 = 50;
	 int nL1 = 1237;
	 int nL2 = 1206;
	 
	 mechMod.attachPoint(softPalate.getNode(nR1), tongue);
	 mechMod.attachPoint(softPalate.getNode(nR2), tongue);
	 mechMod.attachPoint(softPalate.getNode(nL1), tongue);
	 mechMod.attachPoint(softPalate.getNode(nL2), tongue);

	 // TODO: this isn't a nice solution
	 softPalate.getMuscleBundles().get("LM_palatoglossus_R").getFibres().get(0).setFirstPoint(softPalate.getNode(nR1));
	 softPalate.getMuscleBundles().get("LM_palatoglossus_R").getFibres().get(10).setFirstPoint(softPalate.getNode(nR2));
	 softPalate.getMuscleBundles().get("LM_palatoglossus_L").getFibres().get(0).setFirstPoint(softPalate.getNode(nL1));
	 softPalate.getMuscleBundles().get("LM_palatoglossus_L").getFibres().get(10).setFirstPoint(softPalate.getNode(nL2));
      }

      // need to put this here because attaching the soft palate to 
      // the tongue caused some nodes to move, which will invalidate
      // some internal structures like the bounding volume hierarchy
      softPalate.updatePosState();
      
      // the palatopharyngeus attaches to the pharyngeal wall
      	// for now just anchored statically and connected with a spring
      Particle pharAnchor_R = new Particle(1.0, 140.0, 18.0, 75.0);
      Particle pharAnchor_L = new Particle(1.0, 140.0, -18.0, 75.0);
      pharAnchor_R.setDynamic(false);
      pharAnchor_L.setDynamic(false);
      MultiPointMuscle ppmR = createMultiPointMuscle("palatopharyngeus_R", 
	    softPalate.markers().findComponent("LM_palatopharyngeus_R_7"), 
	    pharAnchor_R, 
	    softPalate.markers().findComponent("LM_palatopharyngeus_R_15"));
      MultiPointMuscle ppmL = createMultiPointMuscle("palatopharyngeus_L", 
	    softPalate.markers().findComponent("LM_palatopharyngeus_L_7"), 
	    pharAnchor_L, 
	    softPalate.markers().findComponent("LM_palatopharyngeus_L_15"));
      mechMod.addParticle(pharAnchor_R);
      mechMod.addParticle(pharAnchor_L);
      mechMod.addMultiPointSpring(ppmR);
      mechMod.addMultiPointSpring(ppmL);
      //MuscleBundle mb = ((MuscleBundleList)softPalate.get("bundles")).findComponent("LM_palatopharyngeus_R");
      
      // finally, the soft palate should be attached to the hard palate
      	// for now, if we just anchor all nodes for x<98.0, that will do a decent job of anchoring the front edge
      for (FemNode3d node: softPalate.getNodes()) 
      {
	 double xValue_front = 96.5;
	 double zValue = 121.0;
	 if ((node.getPosition().x < xValue_front &&
              node.getPosition().z > zValue)) 
	    node.setDynamic(false);
      }
      
   }
   
   public MultiPointMuscle createMultiPointMuscle(String name, Point p1, Point p2, Point p3)
   {
      MultiPointMuscle muscle = new MultiPointMuscle (name);
      muscle = MultiPointMuscle.createPai (muscleMaxForce, 0, 1.0, 0);
      muscle.addPoint (p1);
      muscle.addPoint (p2);
      muscle.addPoint (p3);
      return muscle;
   }
   
   public void addInternalMuscles(FemMuscleModel fem)
   {
      bundleNames = new ArrayList<String> ();
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
   }
   private void addMusclesFromAmiraLandmarks (FemMuscleModel fem, String landmarkPath) {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (SoftPalateModel.class, landmarkPath));
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
   private void addMuscleFromFile (FemMuscleModel fem, String fileName) {
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
   private MuscleBundle createAndAddMuscleBundle(FemMuscleModel fem, Point3d[] pts) {
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
      fibre.setFirstPoint (pointA);
      fibre.setSecondPoint (pointB);
      ConstantAxialMuscle mat = new ConstantAxialMuscle();
      mat.setForceScaling(muscleMaxForce); // XXX should this be setMaxForce ??
      fibre.setMaterial(mat);
      bundle.addFibre (fibre);
      return fibre;
   }
   private void addMarkersFromAmiraLandmarks (FemMuscleModel fem, String landmarkPath) {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (SoftPalateModel.class, landmarkPath));
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
   private void addMarkersFromFile (FemMuscleModel fem, String fileName) {
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
   private void addMuscleBundleFromSavedIndices (FemMuscleModel fem, String landmarkPath) {
      File directory =
         new File (ArtisynthPath.getSrcRelativePath (SoftPalateModel.class, landmarkPath));
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            for(String bundleName: bundleNames) {
               if (fileName.startsWith (bundleName) && fileName.endsWith (".bundle")) {
                  addMuscleBundleFromIndexFile (fem, directory + "/" + fileName, bundleName);
               }
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }
   }
   private void addMuscleBundleFromIndexFile (FemMuscleModel fem, String fileName, String bundleName) {
      try {
         Integer[][] indices = readMuscleIndices(fileName);
         MuscleBundle bundle = new MuscleBundle ();
         bundle.setName(bundleName);
//         int bundleNum = fem.getMuscleBundles ().size();	// bad! assumes file read order
//         if (bundleNum < bundleNames.size ()) {
//            bundle.setName (bundleNames.get (bundleNum));
//         }
         fem.addMuscleBundle (bundle);
         //String bundleName = bundle.getName();
         for (Integer[] ptidx : indices) {
            Muscle fibre = new Muscle ();
            fibre.setFirstPoint  (fem.markers ().get(bundleName + "_" + ptidx[0]));
            fibre.setSecondPoint (fem.markers ().get(bundleName + "_" + ptidx[1]));
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
   
   public void loadSkull()
   {
      String skullPath = ArtisynthPath.getSrcRelativePath (BadinFaceDemo.class, "geometry/");
      String geometryName = "badinskull.obj";
      PolygonalMesh skullMesh;
      
      double scale = 1000.0;
      
      try
      {
	 skullMesh =  new PolygonalMesh(new File(skullPath + geometryName));
      }
      catch (Exception e)
      {
	 e.printStackTrace();
	 skullMesh = null;
      }

      skullMesh.scale(scale);
      
      RigidBody skull = new RigidBody();
      skull.setName("skull");
      skull.setMesh(skullMesh, null);
      skull.setDynamic(false);
      
      myMechModel.addRigidBody(skull);

   }
   
   public void loadProbes()
   {
      if (probesFilename == null || !myInputProbes.isEmpty () || !myOutputProbes.isEmpty ())
         return;

      //String probeFileFullPath = ArtisynthPath.getWorkingDir().getPath() + probesFilename;
      String probeFileFullPath = dataPath + probesFilename;
      try
      {
	 scanProbes( ArtisynthIO.newReaderTokenizer(probeFileFullPath));
	 System.out.println("Loaded Probes from File: " + probeFileFullPath);
      }
      catch (Exception e)
      {
	 if (debug)
	    System.out.println("Error reading probe file");
	 //e.printStackTrace();
      }
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
   public void attach (DriverInterface driver) 
   {
      if (useProbes == true)
	 loadProbes();
      
      if (useControls == true)
      {
	 super.attach (driver);

	 myControlPanel = SoftPalateModel.createControlPanel (this, softPalate, myMechModel);
	 if (softPalate.getMuscleBundles().size()>0)
	    SoftPalateModel.createMusclePanel(this, softPalate);
	 
	 driver.getViewer().setGridVisible(false);
      }
      setWayPoints(1.0,0.01);
   }
   public void setWayPoints(double stopPoint,double wayPointStep) {
      removeAllWayPoints();
      addWayPoints(this,stopPoint,wayPointStep);
   }
   public static void addWayPoints(RootModel root, double duration,
	 double waypointstep) {
      for (int i = 1; i < duration / waypointstep; i++) {
	 root.addWayPoint(i * waypointstep);
      }
      root.addBreakPoint(duration);
   }
   
}
