package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JSeparator;

import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.render.RenderProps;
import maspack.render.RenderProps.LineStyle;
import maspack.util.ReaderTokenizer;
import maspack.widgets.LabeledControl;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;
import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.PointForce;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidBodyConnector;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.TracingProbe;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScalableUnits;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

public class JawDemo extends RootModel implements ScalableUnits {
   
   public String getAbout() {
      return"A multi-body jaw model for studying chewing biomechanics.\n\n"+
	    "The model was developed by Ian Stavness and Alan G Hannam, please cite: \n" +
	    "Alan G Hannam, Ian Stavness, John E Lloyd, and Sidney Fels. "+
	    "A Dynamic Model of Jaw and Hyoid Biomechanics during Chewing. "+
	    "Journal of Biomechanics, 41(5):1069-1076, 2008.";
   }
   
   
   public static boolean debug = false;

   protected JawModel myJawModel;

   protected final double MIN_SLIDER = -180.0;

   protected final double MAX_SLIDER = 180.0;

   protected String[] pokesToControl = new String[] { "lCondylarAngle",
	 "rCondylarAngle", "lCondylarCant", "rCondylarCant", "lMedWallAngle",
	 "rMedWallAngle", "lPostWallAngle", "rPostWallAngle", "lBiteAngle",
	 "rBiteAngle", "lBiteCant", "rBiteCant" };

   protected String workingDirname = "data/incisorForce";

   protected String probesFilename = "incisorDispProbes.art";

   public JawDemo() {
      super(null);
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      loadModel();
      myJawModel.setLateralWallOffset (0.0);
      addModel(myJawModel);
//      setIncisorVisible();
      setupJawModel();
      
      RenderProps.setLineColor (myJawModel.axialSprings (), Color.WHITE);
      for (AxialSpring s : myJawModel.axialSprings ()) {
         if (s instanceof Muscle) {
            ((Muscle)s).setExcitationColor (Color.RED);
            ((Muscle)s).setMaxColoredExcitation (0.1);
         }
      }
   }

   protected void loadModel() {
      // create jaw model
      try {
	 myJawModel = new JawModel("jawmodel", /* fixedLaryngeal */true, /* useComplexJoint */
	       true, /* curvJoint */false);
      } catch (Exception e) {
	 e.printStackTrace();
	 System.err.println("RootJaw() - error to instantiating JawModel");
      }
   }

   public void setWorkingDir() {
      if (workingDirname == null) return;
      // set default working directory to repository location
      File workingDir = new File (
         ArtisynthPath.getSrcRelativePath(this, workingDirname));
      ArtisynthPath.setWorkingDir(workingDir);
      if (debug) {
	 System.out.println("Set working directory to "
	       + ArtisynthPath.getWorkingDir().getAbsolutePath());
      }
   }

   public boolean bolusesLoaded = false;

   public void loadBoluses() {
      if (bolusesLoaded) return;
      createBoluses();
      for (FoodBolus fb : myFoodBoluses) {
	 myJawModel.addForceEffector(fb);
	 // System.out.println(fb.getName() + " P = "
	 // + fb.getPlane().toString("%8.2f"));
	 if (fb.getName().equals("rightbolus")) fb.setActive(true);
	 else
	    fb.setActive(false);
      }
      bolusesLoaded = true;
   }

   public void loadProbes() {
      if (probesFilename == null || !myInputProbes.isEmpty()
	    || !myOutputProbes.isEmpty()) return;
      String probeFileFullPath = ArtisynthPath.getWorkingDir().getPath() + "/"
	    + probesFilename;
      System.out.println("Loading Probes from File: " + probeFileFullPath);

      try {
	 scanProbes(
	       ArtisynthIO.newReaderTokenizer(probeFileFullPath));
      } catch (Exception e) {
	 System.out.println("Error reading probe file");
	 e.printStackTrace();
      }
   }

   public void addIncisorProbe(double duration, double interval, String propName) {
      String name = "incisor_" + propName;
      NumericOutputProbe p = new NumericOutputProbe(new Property[] { myJawModel
	    .frameMarkers().get("lowerincisor").getProperty(propName) },
	    interval);
      p.setStartStopTimes(0.0, duration);
      p.setName(name);
      p.setAttachedFileName(name + "_output.txt");
      p.setDefaultDisplayRange(-0.1, 0.1);
      addOutputProbe(p);

   }

   public void addConForceProbe(double duration, double interval) {
      ArrayList<Property> props = new ArrayList<Property>();
      for (RigidBodyConnector rbc : myJawModel.rigidBodyConnectors()) {
	 if (rbc.isEnabled() && rbc.getProperty("activation") != null) {
	    props.add(rbc.getProperty("activation"));
	 }
      }

      Property[] proparray = new Property[props.size()];
      for (int i = 0; i < props.size(); i++) {
	 proparray[i] = props.get(i);
      }

      String name = "conforce";

      NumericOutputProbe p = new NumericOutputProbe(proparray, interval);
      p.setStartStopTimes(0, duration);
      p.setName(name);
      p.setAttachedFileName(name + "_output.txt");
      p.setDefaultDisplayRange(-0.1, 0.1);
      addOutputProbe(p);

   }

   public void addMuscleProbe(double duration, double interval,
	 String propName, String name) {
      ArrayList<Property> props = new ArrayList<Property>();
      for (AxialSpring m : myJawModel.axialSprings()) {
	 if (m instanceof Muscle) if (((Muscle) m).isEnabled()) {
	    props.add(m.getProperty(propName));
	 }
      }

      Property[] proparray = new Property[props.size()];
      for (int i = 0; i < props.size(); i++) {
	 proparray[i] = props.get(i);
      }

      NumericOutputProbe p = new NumericOutputProbe(proparray, interval);
      p.setStartStopTimes(0, duration);
      p.setName(name);
      p.setAttachedFileName(name + "_output.txt");
      p.setDefaultDisplayRange(-0.1, 0.1);
      addOutputProbe(p);
   }

   public void setupJawModel() {
      // createIncisorPointForce();

      // show only jaw rigidbody
      myJawModel.showMeshFaces(false);
      RenderProps.setVisible(myJawModel.rigidBodies().get("jaw"), true);

      // integrator settings
      myJawModel.setMaxStepSize(0.001);
      myJawModel.setIntegrator(Integrator.BackwardEuler);

      // damping settings
      setDampingParams(10, 100, 0.0);

      RenderProps.setVisible(myJawModel.rigidBodies().get("hyoid"), true);
      addIncisorForce();

   }

   boolean addIncisorTrace = false;
   
   
   public void addIncisorForce() {
//      Point3d incPt = new Point3d(0.0, -47.118954, -1.2163778);
//      FrameMarker inc = new FrameMarker(myJawModel.rigidBodies().get("lower"), incPt);
//      myJawModel.addFrameMarker(inc);
      
      FrameMarker inc = myJawModel.frameMarkers().get("lowerincisor");
      
      PointForce pf = new PointForce(new Vector3d(0,0,-1), inc);
      pf.setName("AppliedForce_Incisor");
      pf.setAxisLength(15);
      pf.setMagnitude(0);
      myJawModel.addForceEffector(pf);
      
      RenderProps.setLineColor(pf, new Color(0.4f, 0.7f, 0.4f));
      RenderProps.setLineStyle(pf, LineStyle.CYLINDER);
      RenderProps.setLineRadius(pf, 1.5);
   }

   public void setIncisorVisible() {
      FrameMarker inc = myJawModel.frameMarkers().get("lowerincisor");
      if (inc == null) return;
      RenderProps.setLineColor(inc, Color.CYAN);
      RenderProps.setPointColor(inc, Color.CYAN);
      Point3d loc = new Point3d();
      inc.getLocation(loc);
      inc.setLocation(loc);
      enableTracing (inc);
      addIncisorTrace = true;
   }

   public PointForce createIncisorPointForce() {
      FrameMarker inc = myJawModel.frameMarkers().get("lowerincisor");
      if (inc == null) return null;

      double r = inc.getRenderProps().getLineRadius();
      PointForce pf = new PointForce(inc);
      pf.setName("incForce");
      pf.setMagnitude(0.0);
      pf.setDirection(new Vector3d(0, 0, -1));
      pf.setAxisLength(r * 10);
      RenderProps.setLineStyle(pf, LineStyle.CYLINDER);
      RenderProps.setLineRadius(pf, r / 2);
      RenderProps.setLineColor(pf, Color.GREEN);

      myJawModel.addForceEffector(pf);
      return pf;
   }

   public void setDampingParams(double td, double rd, double d) {
      // set damping values in RB damper
      setJawDamping(td, rd);
      setLarynxDamping(td, rd);

      // set spring damping values
      setSpringDamping(d);
   }

//   public void setMuscleDamping(double d) {
//      for (AxialSpring s : myJawModel.axialSprings()) {
//	 if (s instanceof Muscle) s.setDamping(d);
//      }
//   }

   public void setSpringDamping(double d) {
      for (AxialSpring s : myJawModel.axialSprings()) {
	 AxialSpring.setDamping (s, d);
      }
   }

   public void setJawDamping(double td, double rd) {
      setDamping("jaw", td, rd);
   }

   public void setLarynxDamping(double td, double rd) {
      setDamping("hyoid", td, rd);
      setDamping("thyroid", td, rd);
      setDamping("cricoid", td, rd);
   }

   public void setDamping(String bodyName, double td, double rd) {
      RigidBody body = myJawModel.rigidBodies().get(bodyName);
      if (body != null) {
	 body.setRotaryDamping(rd);
	 body.setFrameDamping(td);
      }
   }

   /*
    * Probe helper methods
    */

   public static void addMuscleProbes(RootModel root, JawModel jaw,
	 String[] muscleNames) throws IOException {
      addMuscleProbes(root, jaw, muscleNames, "");
   }

   public static void addMuscleProbes(RootModel root, JawModel jaw,
	 String[] muscleNames, String namePrefix) throws IOException {
      NumericInputProbe inprobe;

      for (int i = 0; i < muscleNames.length; i++) {
	 String compPath = "axialSprings/" + muscleNames[i];
	 if (jaw.findComponent(compPath) == null) {
	    // not muscle, try excitors list
	    compPath = "exciters/" + muscleNames[i];
	    if (jaw.findComponent(compPath) == null) continue;
	 }

	 inprobe = new NumericInputProbe(jaw, compPath + ":excitation", null);
	 inprobe.setName(namePrefix + muscleNames[i]);
	 inprobe.setActive(true);
	 inprobe.setStartStopTimes(0, 1.0);
	 inprobe.loadEmpty();
	 root.addInputProbe(inprobe);
      }
   }

   /*
    * Food bolus helper methods
    */

   ArrayList<FoodBolus> myFoodBoluses = new ArrayList<FoodBolus>();

   protected double bolusDiameter = 10.0; // mm

   protected double bolusMaxResistance = 35.0; // N

   protected double bolusStiffness = bolusMaxResistance / (bolusDiameter);

   protected double bolusDamping = 0.01;

   public void createBoluses() {
      // TODO: create bolus using occlusal plane angle
      Point3d rightbitePos = myJawModel.frameMarkers().get("rbite")
	    .getPosition();
      Point3d leftbitePos = myJawModel.frameMarkers().get("lbite")
	    .getPosition();
      createFoodBolus("rightbolus", rightbitePos, (PlanarConnector) myJawModel
	    .rigidBodyConnectors().get("rbite"));
      createFoodBolus("leftbolus", leftbitePos, (PlanarConnector) myJawModel
	    .rigidBodyConnectors().get("lbite"));
      updateBoluses();

   }

   public void updateBoluses() {
      System.out.println("bolus dirs updated");
      if (myFoodBoluses.size() >= 2) {
	 updateBolusDirection("RBITE", myFoodBoluses.get(0));
	 updateBolusDirection("LBITE", myFoodBoluses.get(1));
      }
   }

   public void updateBolusDirection(String constraintName, FoodBolus bolus) {
      PlanarConnector bite = (PlanarConnector) myJawModel.rigidBodyConnectors()
	    .get(constraintName);
      if (bite != null && bolus != null) {
	 bolus.setPlane(bite);
	 // RigidTransform3d XPB = bite.getXDB();
	 // // System.out.println(constraintName + " X =\n" +
	 // XPB.toString("%8.2f"));
	 // bolus.setPlane( getPlaneFromX (XPB));
	 // // System.out.println(bolus.getName() + "plane =\n" +
	 // bolus.myPlane.toString("%8.2f"));
      }
   }

   private Plane getPlaneFromX(RigidTransform3d XPB) {
      Plane myPlaneB = new Plane();
      Vector3d nrm = new Vector3d(XPB.R.m02, XPB.R.m12, XPB.R.m22);
      myPlaneB.set(nrm, nrm.dot(XPB.p));
      return myPlaneB;
   }

   public void createFoodBolus(String bolusName, Point3d location,
	 PlanarConnector plane) {
      FoodBolus fb = new FoodBolus(bolusName, plane, bolusDiameter,
	    bolusMaxResistance, bolusDamping);

      RenderProps bolusPtProps = new RenderProps(myJawModel.getRenderProps());
      bolusPtProps.setPointRadius(0.0);
      bolusPtProps.setPointColor(Color.BLACK);

      RigidBody jaw = myJawModel.rigidBodies().get("jaw");
      FrameMarker bolusContactPt = new FrameMarker();
      myJawModel.addFrameMarker(bolusContactPt, jaw, location);
      bolusContactPt.setName(bolusName + "ContactPoint");
      bolusContactPt.setRenderProps(new RenderProps(bolusPtProps));

      fb.setCollidingPoint(bolusContactPt);
      myFoodBoluses.add(fb);
   }

   /*
    * Jaw Control Panel helper methods:
    */

   public void addJawOptions(ControlPanel panel) {
      if (panel == null) return;
      JawPanel.createJawPanel(myJawModel, panel);
   }

   public void addWidgets(ControlPanel panel, String[] propNames) {
      if (panel == null) return;
      panel.addWidget(new JSeparator());
      LabeledControl slider;
      for (int i = 0; i < propNames.length; i++) {
	 slider = (LabeledControl) panel.addWidget(myJawModel, propNames[i],
	       MIN_SLIDER, MAX_SLIDER);
	 if (propNames[i].contains("Bite")) {
	    slider.addValueChangeListener(new ValueChangeListener() {
	       public void valueChange(ValueChangeEvent e) {
		  ((JawDemo) Main.getMain().getRootModel())
			.updateBoluses();
	       }
	    });
	 }
      }
   }

   public void createControlPanel(RootModel root, JFrame refFrame) {
      ControlPanel panel = new ControlPanel("Jaw Controls", "LiveUpdate");
      addJawOptions(panel);
      // addSliderControls(panel, pokesToControl);
      addWidgets(panel, pokesToControl);
      panel.pack();
      panel.setVisible(true);
      java.awt.Point loc = refFrame.getLocation();
      panel.setLocation(loc.x + refFrame.getWidth(), loc.y);
      panel.enableLiveUpdating(true);
      root.addControlPanel(panel);
   }

   public void loadControlPanel(RootModel root) {
      String panelNames[] = new String[] { "miscJawonly", "dampingJawonly",
	    "muscles", "jointsFlatTmj" };
      loadControlPanel(root, panelNames);

   }

   public void loadControlPanel(RootModel root, String[] filenames) {
      for (int i = 0; i < filenames.length; i++) {
	 File file = new File(ArtisynthPath.getSrcRelativePath(JawDemo.class,
	       "controlpanels/" + filenames[i] + ".art"));
	 if (file == null) continue;
	 ControlPanel panel = null;
	 try {
	    panel = (ControlPanel) ComponentUtils.loadComponent(file, root,
		  ControlPanel.class);
	 } catch (Exception e) {
	    System.out.println(
               "Error reading control panel file "+file+": "+e.getMessage());
	 }
	 if (panel != null) {
	    root.addControlPanel(panel);
	 }
      }
   }

   public void attach(DriverInterface driver) {
      setWorkingDir();
      loadProbes();

      if (getControlPanels().size() == 0) { // createControlPanel (this,
					    // driver.getFrame());
	 loadControlPanel(this);
      }

   }

   public void detach(DriverInterface driver) {
      super.detach(driver);
   }

   public void scaleDistance(double s) {
      myJawModel.scaleDistance(s);
      for (FoodBolus fb : myFoodBoluses) {
	 fb.scaleDistance(s);
      }
   }

   public void scaleMass(double s) {
      myJawModel.scaleMass(s);
      for (FoodBolus fb : myFoodBoluses) {
	 fb.scaleMass(s);
      }
   }

   public void scan(ReaderTokenizer rtok, Object ref) throws IOException {
      super.scan(rtok, ref);
      myJawModel = (JawModel) myModels.get(0);
   }

   @Override
   public void initialize (double t) {
      if (myFoodBoluses != null && t == 0) {
         for (FoodBolus f : myFoodBoluses) {
            f.setCrushed(true);
            f.applyForces (t); // set bolus force to zero, because crushed
         }
      }
      super.initialize(t);
   }

   private static FrameMarker[] getMarkers(MechModel mech, String[] markerNames) {
      ArrayList<FrameMarker> markers = new ArrayList<FrameMarker>();
      for (String name : markerNames) {
	 FrameMarker mkr = mech.frameMarkers().get(name);
	 if (mkr != null) markers.add(mkr);
      }
      return markers.toArray(new FrameMarker[0]);
   }

   public static void addOutputProbes(RootModel root, MechModel mech,
	 String[] markerNames, String prop, double duration) {
      addOutputProbes(root, getMarkers(mech, markerNames), prop, duration);
   }

   public static void addOutputProbes(RootModel root, FrameMarker[] markers,
	 String prop, double duration) {
      for (FrameMarker mkr : markers) {
	 NumericOutputProbe outprobe = new NumericOutputProbe(mkr, prop, null,
	       0.01);
	 String name = prop + "_"
	       + (mkr.getName() != null ? mkr.getName() : mkr.getNumber());
	 outprobe.setName(name);
	 outprobe.setAttachedFileName("out_" + name + ".txt");
	 outprobe.setActive(true);
	 outprobe.setStartStopTimes(0, duration);
	 root.addOutputProbe(outprobe);
      }
   }

   public static void addTraceProbes(RootModel root, MechModel mech,
	 String[] markerNames, String namePrefix, double duration) {
      addTraceProbes(root, getMarkers(mech, markerNames), namePrefix, duration);
   }

   public static void addTraceProbes(RootModel root, FrameMarker[] markers,
	 String namePrefix, double duration) {

      for (FrameMarker mkr : markers) {
	 String name = namePrefix
	       + (mkr.getName() != null ? mkr.getName() : mkr.getNumber());
	 TracingProbe traceprobe = root.addTracingProbe(mkr, "position", 0,
	       duration);
	 traceprobe.setUpdateInterval(0.01);
	 traceprobe.setRenderInterval(0.05);
	 traceprobe.setName("trace_" + name);
      }
   }

}
