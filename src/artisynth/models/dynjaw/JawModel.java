package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

//import org.python.modules.types;



//import com.sun.org.apache.bcel.internal.generic.LMUL;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix4d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import maspack.render.Renderer;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.ColorMapProps;
import maspack.render.Renderer.LineStyle;
import maspack.spatialmotion.SpatialInertia;
import maspack.spatialmotion.Wrench;
import maspack.util.DoubleInterval;
import maspack.util.ReaderTokenizer;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.SegmentedPlanarConnector;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.Traceable;
import artisynth.core.materials.*;
import artisynth.core.probes.TracingProbe;
import artisynth.core.probes.VectorTracingProbe;
import artisynth.core.util.AmiraLandmarkReader;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScalableUnits;
import artisynth.core.util.TimeBase;

public class JawModel extends MechModel implements ScalableUnits,
      Traceable {

   public boolean debug = false; // set to true for debug printlns

   public static final double CM_TO_MM = 10.0; // conversion factor

   public static final double NEWTONS_TO_FORCEUNITS = 1000.0; // kg*mm*s^-2

   public static final String muscleListFilename = "muscleList.txt";

   public static final String bodyListFilename = "bodyList.txt";

   public static final String muscleListAllFilename = "muscleListAll.txt";

   public static final String bodyListAllFilename = "bodyListAll.txt";

   public static final String muscleInfoFilename = "muscleInfo.txt";

   public static final String muscleGroupInfoFilename = "muscleGroupsInfo.txt";

   protected boolean addAllGeometry = false;

   protected ArrayList<BodyInfo> bodyInfoList = new ArrayList<BodyInfo>();

   protected ArrayList<String> muscleList = new ArrayList<String>();

   protected HashMap<String, MuscleInfo> muscleInfo;

   protected ArrayList<Muscle> myMuscles = new ArrayList<Muscle>();

   protected ArrayList<MuscleGroupInfo> muscleGroupInfo;

   protected HashMap<String, String> muscleGroupNames = new HashMap<String, String>();

   protected HashMap<String, String> muscleAbbreviations = new HashMap<String, String>();

   protected boolean myRenderEdgesP;

   protected boolean myRenderFacesP;

   protected boolean myTransparencyP;

   static final double transparentAlpha = 0.5;

   static final double opaqueAlpha = 1.0;

   static final double FIXED_PT_RADIUS = 0.0;

   static final double CONTACT_PT_RADIUS = 1.0;

   static final double MEMBRANE_CYL_RADIUS = 0.6;

   static final double MEMBRANE_PT_RADIUS = 0.75;

   static public final Point3d c1Point = new Point3d(0.0, 54.3478, 63.5381);

   protected double myHeadRotation = 0.0; // degrees

   protected double jawCentricRotation = 0.0; // degrees of jaw opening in

   // centric relation

   static final double CLOSE_TO_SAGGITAL = 0.5; // threshold in x-axis for

   // mid-saggital plane

   // members used by output probes for jaw
   public static final Point3d leftBiteLocation = new Point3d(24.8, -20.4, 47.0);

   public static final Point3d hyoidRefPos = new Point3d(0.0, 1.6876, 7.12046);

   protected FrameMarker hyoidRefMarker;

   static final double VERTICAL_TMJ_OFFSET = -4.0;

   protected boolean memShow = true;

   protected boolean musShow = true;

   protected boolean enableMuscles = true;

   // old damping parameters...
   // protected double muscleDamping = 0.005;
   // protected double genLinDamping = 0.01;
   // protected double genRotDamping = 20;
   // protected double initExcitation;

   protected double myMuscleDamping = 0.00;

   protected double myJawDampingR = 1089.0; // rotational damping for jaw

   protected double myJawDampingT = 0.403; // translational damping for jaw

   protected double myHyoidDampingR = 10.0; // rotational damping for hyoid

   protected double myHyoidDampingT = 0.05; // translational damping for hyoid

   static final double VERTICAL_TRACHEA_OFFSET = 10.0;

   protected double thMemStiff = 6000.0; // stiffness of thyro-hyoid membrane

   protected double thMemDamp = 0.1; // damping of thyro-hyoid membrane

   protected double ctrMemStiff = 6000.0; // stiffness of thyro-hyoid membrane

   protected double ctrMemDamp = 0.1; // damping of thyro-hyoid membrane

   protected double hyoidMass = 0.01;

   protected double thyroCricoidMass = 0.047; // combined thyroid + cricoid mass

   protected Point3d[] jointPoint; // left and right joint points in jaw-frame

   protected double myRestTone = 0.0;

   protected double defaultPharnyxStiffness = 200.0;

   // XXX - unitConversion must match NEWTONS_TO_FORCE in Muscle model
   // scaling factor for force conversion (1000 = real)
   protected double unitConversion = 1000;

   protected double gravityVal = 9.8;

   protected ArrayList<Muscle> closerMuscleList;

   protected ArrayList<AxialSpring> memList = new ArrayList<AxialSpring>();

   protected ArrayList<FrameMarker> memPts = new ArrayList<FrameMarker>();

   Wrench jawMuscleWrench = new Wrench();

   Point3d jawComPosition = new Point3d();

   protected Point3d myTmpPos = new Point3d();

   protected RigidTransform3d XBodyToCom = new RigidTransform3d();

   protected Wrench myTmpWrench = new Wrench();

   protected RigidTransform3d XWorldToCom = new RigidTransform3d();

   public static final int NUM_CON = 10;

   ArrayList<PlanarConnector> con = new ArrayList<PlanarConnector>(NUM_CON);

   ArrayList<RigidTransform3d> conPose = new ArrayList<RigidTransform3d>(
	 NUM_CON);

   ArrayList<FrameMarker> conPt = new ArrayList<FrameMarker>(NUM_CON);

   AxisAngle globalConRot = new AxisAngle();

   RigidTransform3d globalConX = new RigidTransform3d();

   protected boolean planesVisible = true;

   RigidBody constrainedBody;

   Point3d cricothryroidArticulation = new Point3d(0.0, 23.0, -24.0);

   boolean useComplexJoint = true;

   boolean useCurvJoint = true;

   ArrayList<JawPlanes> conOrder = new ArrayList<JawPlanes>();

   public void createConstraintOrder() {
      conOrder.add(JawPlanes.LTMJ);
      conOrder.add(JawPlanes.RTMJ);
      conOrder.add(JawPlanes.LBITE);
      conOrder.add(JawPlanes.RBITE);
      if (useComplexJoint) {
	 conOrder.add(JawPlanes.LMED);
	 conOrder.add(JawPlanes.RMED);
	 conOrder.add(JawPlanes.LPOST);
	 conOrder.add(JawPlanes.RPOST);
	 conOrder.add(JawPlanes.LLTRL);
	 conOrder.add(JawPlanes.RLTRL);

      }

   }

   public void createSimpleConstraintOrder() {
      conOrder.add(JawPlanes.LTMJ);
      conOrder.add(JawPlanes.RTMJ);
      conOrder.add(JawPlanes.LBITE);
      conOrder.add(JawPlanes.RBITE);
   }

   // indexes of angle arrays
   public static final int LEFT = 0;

   public static final int RIGHT = 1;

   double[] condylarAngle = new double[] { 40.0, 40.0 };// {left, right}

   double[] condylarCant = new double[] { 0.0, -0.0 }; // {left, right}

   double[] medWallAngle = new double[] { -1.0, 1.0 }; // {left, right}

   double[] ltrlWallAngle = new double[] { -1.0, 1.0 }; // {left, right}

   double[] postWallAngle = new double[] { 0.0, 0.0 }; // {left, right}

   // double occlusalAngle = 6.5; // bite plane
   double[] biteAngle = new double[] { 10, 10 };// {left, right}

   double[] biteCant = new double[] { 0.0, 0.0 }; // {left, right}

   // medial wall offset (negative = lateral outward)
   double[] medWallOffset = new double[] { 0, 0 };

   // lateral wall offset (negative = lateral outward)
   double[] ltrlWallOffset = new double[] { -0.1, -0.1 };

   // posterior wall offset (positive = forward);
   double[] postWallOffset = new double[] { -0.0, -0.0 };

   // constraint force norm properties (for output probes)
   VectorNd tmjForceNorms = new VectorNd(NUM_CON); // 8 jaw constraint planes

   double tmjForceNorm = 0.0;

   double[] curvParams = new double[] { 0.0, // x0
	 0.0, // y0
	 12.0, // xf
	 -5.0, // yf
	 -40.0, // initial slope
	 -10.0 }; // final slope

   int numSegments = 20;

   public enum JawPlanes {
      LTMJ("ltmj", false, 40.0), RTMJ("rtmj", false, 40.0), LBITE("lbite",
	    true, 25.0), RBITE("rbite", true, 25.0), LMED("ltmj", true, 25.0), RMED(
	    "rtmj", true, 25.0), LPOST("ltmj", true, 25.0), RPOST("rtmj", true,
	    25.0), LLTRL("ltmj", true, 25.0), RLTRL("rtmj", true, 25.0);

      String contactName;

      boolean unilateralFlag;

      double planeSize;

      private JawPlanes(String name, boolean unilateral, double size) {
	 contactName = name;
	 unilateralFlag = unilateral;
	 planeSize = size;
      }

      public String getContactName() {
	 return contactName;
      }

      public boolean isUnilateral() {
	 return unilateralFlag;
      }

      public double getPlaneSize() {
	 return planeSize;
      }

   }

   private class BodyInfo {
      public String name;

      public String meshName;

      public void scan(ReaderTokenizer rtok) throws IOException {
	 name = rtok.sval;
	 rtok.nextToken();
	 meshName = rtok.sval;
      }
   }

   private class MuscleInfo {
      public String name;

      public String origin;

      public String insertion;

      public String fullName;

      boolean pairedFlag; // true == left-right paired muscle

      public boolean isPaired() {
	 return pairedFlag;
      }

      public void scan(ReaderTokenizer rtok) throws IOException {
	 name = rtok.sval;
	 rtok.nextToken();
	 origin = rtok.sval;
	 rtok.nextToken();
	 insertion = rtok.sval;
	 rtok.nextToken();
	 pairedFlag = (rtok.sval.compareTo("paired") == 0.0);
	 rtok.nextToken();
	 fullName = rtok.sval;
      }

   }

   protected class MuscleGroupInfo {
      public String name;

      public String fullName;

      public ArrayList<String> coactivators = new ArrayList<String>();

      public void scan(ReaderTokenizer rtok) throws IOException {
	 rtok.eolIsSignificant(true);
	 name = rtok.sval;
	 rtok.nextToken();
	 fullName = rtok.sval;
	 rtok.nextToken();
	 while (rtok.ttype != ReaderTokenizer.TT_EOL
	       && rtok.ttype != ReaderTokenizer.TT_EOF) {
	    coactivators.add(rtok.sval);
	    rtok.nextToken();
	 }
      }
   }

   public static PropertyList myProps = new PropertyList(JawModel.class,
	 MechModel.class);

   static {
      myProps.add("ctrMemStiffness * *", "stiffness of CTr membrane", 100.0);
      myProps.add("thMemStiffness * *", "stiffness of TH membrane", 100.0);
      myProps.add("jawCentricRotation * *",
	    "degrees of jaw rotation in centric relation", 0.0);
      myProps.add("headRotation * *",
	    "degrees of head rotation about C1 point", 0.0);
      myProps.add("renderFaces * showMeshFaces",
	    "flag for redering mesh faces", false);
      myProps.add("renderEdges * showMeshEdges",
	    "flag for redering mesh faces", false);
      myProps
	    .add("transparency * *", "transparency of rigid body meshes", true);
      myProps.add("memShow getShowMembrane setShowMembrane",
	    "visibility of laryngeal connective tissue", true);
      myProps.add("musShow getShowMuscles setShowMuscles",
	    "visibility of all muscles", true);
      myProps.add("enableMuscles * *", "enables all muscles", true);
      myProps.add("muscleColor * *", "color of closer muscles", Color.CYAN);
      myProps.add("rigidBodyColor * *", "color of jaw and skull", Color.WHITE);
      myProps.addReadOnly("jawMuscleWrench",
	    "total wrench applied to jaw by muscles");
      myProps
	    .addReadOnly("jawMuscleForce",
		  "translational force component of total wrench applied to jaw by muscles");
      myProps.addReadOnly("jawMuscleMoment",
	    "moment component of total wrench applied to jaw by muscles");

      myProps.addReadOnly("jawComPosition",
	    "jaw centre-of-mass in world-coordinates");
      myProps.add("planesVisible isPlanesVisible *",
	    "visibility of constraint plane", true);
      myProps.add("lCondylarAngle * *",
	    "forward and downward angle of left tmj plane", 0.0, "[-180,180]");
      myProps.add("lCondylarCant * *", "medial cant angle of left tmj plane",
	    0.0, "[-180,180]");
      myProps.add("lMedWallAngle * *",
	    "horizontal cant angle of left med wall", 0.0, "[-180,180]");
      myProps.add("lLtrlWallAngle * *",
	    "horizontal cant angle of left lateral wall", 0.0, "[-180,180]");
      myProps.add("lPostWallAngle * *",
	    "horizontal cant angle of left post wall", 0.0, "[-180,180]");
      myProps.add("rCondylarAngle * *",
	    "forward and downward angle of right tmj plane", 0.0, "[-180,180]");
      myProps.add("rCondylarCant * *", "medial cant angle of right tmj plane",
	    0.0, "[-180,180]");
      myProps.add("rMedWallAngle * *",
	    "horizontal cant angle of right med wall", 0.0, "[-180,180]");
      myProps.add("rLtrlWallAngle * *",
	    "horizontal cant angle of right lateral wall", 0.0, "[-180,180]");
      myProps.add("rPostWallAngle * *",
	    "horizontal cant angle of right post wall", 0.0, "[-180,180]");
      // myProps.add("occlusalAngle * *",
      // "forward and downward angle of occlusal plane", 0.0);
      myProps.add("lBiteAngle * *",
	    "forward and downward angle of left bite plane", 0.0, "[-180,180]");
      myProps.add("lBiteCant * *", "medial cant angle of left bite plane", 0.0,
	    "[-180,180]");
      myProps
	    .add("rBiteAngle * *",
		  "forward and downward angle of right bite plane", 0.0,
		  "[-180,180]");
      myProps.add("rBiteCant * *", "medial cant angle of right bite plane",
	    0.0, "[-180,180]");

      myProps.addReadOnly("constraintForceNorms *",
	    "vector of force norms for eight jaw constraints");
      myProps.addReadOnly("tmjForceNorm *",
	    "force norms for left tmj constraints");

   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public JawModel() throws IOException {
      super();
   }

   public JawModel(String name) throws IOException {
      this(name, true, true, true);
   }

   public JawModel(String name, boolean fixedLaryngeal,
	 boolean useComplexJoint, boolean useCurvJoint) throws IOException {
      super(name);

      setGravity(0, 0, -gravityVal * unitConversion);
      setupRenderProps();

      if (!fixedLaryngeal) {
	 muscleList = readStringList(muscleListAllFilename);
	 bodyInfoList = readBodyInfoList(bodyListAllFilename);
      } else {
	 muscleList = readStringList(muscleListFilename);
	 bodyInfoList = readBodyInfoList(bodyListFilename);
      }

      muscleInfo = readMuscleInfo(muscleInfoFilename);
      muscleGroupInfo = readMuscleGroupsInfo(ArtisynthPath.getSrcRelativePath(
	    JawModel.class, "geometry/" + muscleGroupInfoFilename));
      // setMuscleAbbreviations();

      assembleRigidBodies();
      initHeadRotationVertebrae();
      setPharynxTexture();

      attachMarkers();
      assembleMuscles();
      pruneMuscleList(); // removes muscles from myMuscles if not in
      // muscleList
      attachMuscles();
      // printMuscleMaxForces();

      closerMuscleList = createMuscleList(readStringList("closerMuscleList.txt"));

      // setCoactivatingMuscles();
      assembleBilateralExcitors();
      assembleMuscleGroups();

      updateMuscleLengthProps();

      showMasseterMarkers();

      if (myRigidBodies.get("cricoid") != null && !fixedLaryngeal) {
	 attachMembraneMesh("hyoid", "thyroid", "thMem"); // thyrohyoid membrane
	 attachMembraneMesh("cricoid", "sternum", "ctrMem"); // cricotracheal
	 // membrane
	 addPharynxSprings();
      } else {
	 setLaryngealBodiesFixed();
      }

      addFixedMarkers();

      showMeshFaces(true);
      showMeshEdges(false);
      setTransparency(false);

      setMaxStepSize(0.0001);
      setIntegrator(Integrator.RungeKutta4);

      this.useComplexJoint = useComplexJoint;

      constrainedBody = myRigidBodies.get("jaw");
      if (constrainedBody == null) // no jaw body - error
      {
	 System.err.println("JawModel: unable to get jaw rigidbody");
	 return;
      }

      initCons();

      for (PlanarConnector pc : con) {
	 addBodyConnector(pc);
      }

      // do not use medial constraints in default jaw model
      bodyConnectors().get("LMED").setEnabled(false);
      bodyConnectors().get("RMED").setEnabled(false);
      RenderProps.setVisible(bodyConnectors().get("LMED"), false);
      RenderProps.setVisible(bodyConnectors().get("RMED"), false);

      // add cricothyroid revolute joint
      if (!fixedLaryngeal) {
	 addCricothyroidJoint();
      }

      this.useCurvJoint = useCurvJoint;

      if (useCurvJoint) {
	 // remove planar tmj constraints (first two in list)
	 removeBodyConnector(con.get(0));
	 removeBodyConnector(con.get(1));

	 addCurvilinearTmjs();
      }
   }

   private void setupRenderProps() {
      RenderProps props = createRenderProps();

      // Particle RenderProps
      props.setPointRadius(1.0);
      props.setPointStyle(Renderer.PointStyle.SPHERE);
      //props.setPointSlices(12);
      props.setPointColor(Color.PINK);

      // Line RenderProps
      props.setLineRadius(2.0);
      //props.setLineSlices(8);
      props.setLineWidth(3);
      props.setLineStyle(Renderer.LineStyle.LINE);
      props.setLineColor(Color.WHITE);

      // Mesh RenderProps
      props.setShading(Renderer.Shading.SMOOTH);
      props.setFaceColor(new Color(1f, 0.8f, 0.6f));
      props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);

      ColorMapProps tp = new ColorMapProps();
      tp.setFileName(ArtisynthPath.getSrcRelativePath(JawModel.class,
	    "skull.jpg"));
      //tp.setSphereMappingEnabled(false);
      //tp.setAutomatic(false);
      tp.setColorMixing(Renderer.ColorMixing.MODULATE);
      tp.setEnabled(true);

      props.setColorMap(tp);
      setRenderProps(props);

      // Spring Render Props
      RenderProps.setLineRadius(myAxialSprings, 2.0);
      //RenderProps.setLineSlices(myAxialSprings, 8);
      RenderProps.setLineStyle(myAxialSprings, Renderer.LineStyle.SPINDLE);
      RenderProps.setLineColor(myAxialSprings, Color.RED);
   }

   public void setPharynxTexture() {
      RigidBody body = myRigidBodies.get("pharynx");
      if (body == null) return;

      RenderProps.setColorMapFileName(body, ArtisynthPath.getSrcRelativePath(
	    JawModel.class, "flesh.jpg"));
   }

   private HashMap<String, MuscleInfo> readMuscleInfo(String filename)
	 throws IOException {
      HashMap<String, MuscleInfo> infoList = new HashMap<String, MuscleInfo>();
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(ArtisynthPath
	    .getSrcRelativePath(JawModel.class, "geometry/" + filename)));

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 MuscleInfo mi = new MuscleInfo();
	 mi.scan(rtok);
	 infoList.put(mi.name, mi);
      }
      return infoList;
   }

   protected ArrayList<MuscleGroupInfo> readMuscleGroupsInfo(String filename)
	 throws IOException {
      ArrayList<MuscleGroupInfo> infoList = new ArrayList<MuscleGroupInfo>();
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));
      rtok.eolIsSignificant(true); // read end of lines
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 MuscleGroupInfo mgi = new MuscleGroupInfo();
	 mgi.scan(rtok);
	 infoList.add(mgi);
      }
      return infoList;
   }

   private ArrayList<BodyInfo> readBodyInfoList(String filename)
	 throws IOException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(ArtisynthPath
	    .getSrcRelativePath(JawModel.class, "geometry/" + filename)));
      rtok.wordChars(".");
      ArrayList<BodyInfo> bodyInfoList = new ArrayList<BodyInfo>();
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 BodyInfo bi = new BodyInfo();
	 bi.scan(rtok);
	 bodyInfoList.add(bi);
      }
      return bodyInfoList;
   }

   private ArrayList<String> readStringList(String filename) throws IOException {
      ArrayList<String> stringList = new ArrayList<String>();
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(ArtisynthPath
	    .getSrcRelativePath(JawModel.class, "geometry/" + filename)));

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 if (rtok.ttype != ReaderTokenizer.TT_WORD) { throw new IOException(
	       "readMarkerList Expecting word, got " + rtok.tokenName()); }
	 stringList.add(rtok.sval);
      }
      return stringList;
   }

   public ArrayList<Muscle> createMuscleList(
	 ArrayList<String> muscleAbbreviations) {
      ArrayList<Muscle> list = new ArrayList<Muscle>();
      for (String name : muscleAbbreviations) {
	 if (muscleInfo.get(name).isPaired()) {
	    list.add(findMuscle("l" + name));
	    list.add(findMuscle("r" + name));
	 } else {
	    list.add(findMuscle(name));
	 }
      }
      return list;
   }

   public boolean cricoidExists() {
      if (bodyInfoList == null) { return false; }
      return bodyInfoList.contains("cricoid");
   }

   public void setLaryngealBodiesFixed() {
      String[] larNames = new String[] { "hyoid", "thyroid", "cricoid" };
      RigidBody body;
      for (int i = 0; i < larNames.length; i++) {
	 if ((body = rigidBodies().get(larNames[i])) != null) {
	    body.setDynamic(false);
	 }
      }

   }

   public RigidBody createAndAddBody(String name, String meshName) {

      RigidBody body = myRigidBodies.get (name);
      if (body == null) {
         body = new RigidBody();
         body.setName(name);
         addRigidBody(body);
      }
      body.setInertia(SpatialInertia.createBoxInertia(1, 1, 1, 1));
      setBodyDynamicProps(body);
      if (meshName.compareTo("none") != 0) setBodyMesh(body, meshName);
      RenderProps.setVisible (body, true);
      return body;
   }

   public void setBodyMesh(RigidBody body, String meshName) {
      setBodyMesh(body, meshName, 1.0);
   }

   public void setBodyMesh(RigidBody body, String meshName, double scale) {
      String meshFilename = ArtisynthPath.getSrcRelativePath(JawModel.class,
	    "geometry/" + meshName);
      PolygonalMesh mesh = new PolygonalMesh();
      try {
	 mesh.read(new BufferedReader(new FileReader(meshFilename)));
      } catch (IOException e) {
	 e.printStackTrace();
	 return;
      }
      mesh.scale(scale);
      mesh.setFixed(true);
      mesh.triangulate ();
      body.setMesh(mesh, meshFilename);
   }

   public void setBodyDynamicProps(RigidBody body) {
      if (body == null || body.getName() == null) return;

      String name = body.getName();
      if (name.compareTo("jaw") == 0.0) setJawDynamicProps(body);
      else if (name.compareTo("hyoid") == 0.0) setHyoidDynamicProps(body);
      else if (name.compareTo("thyroid") == 0.0) setThyroidDynamicProps(body);
      else if (name.compareTo("cricoid") == 0.0) setCricoidDynamicProps(body);
      else
	 setBodyFixed(body);

   }

   public void setBodyFixed(RigidBody body) {
      if (body != null) body.setDynamic(false);
   }

   public void setJawDynamicProps(RigidBody jaw) {
      if (jaw == null) { return; }
      // Ineria Properties from ADAMS model - kg*mm*mm
      jaw.setDynamic(true);
      jaw.setRotationalInertia(new SymmetricMatrix3d(92.19, // (0,0)
	    182.2, // (1,1)
	    125.2, // (2,2)
	    -1.122, // (0,1)
	    -10.7, // (0,2)
	    1.345)); // (1,2)
      jaw.setCenterOfMass(new Point3d(9.53, 0.0, 36.08)); // mm (uncapped obj)
      // jaw.setCenterOfMass(new Point3d(0.0, -13.73, 31.97)); //mm (capped obj)
      jaw.setMass(0.2); // kg
      // rotate to AmiraJaw coordinate frame (Rot about z-axis -PI/2)
      RotationMatrix3d R = new RotationMatrix3d();
      R.setAxisAngle(0, 0, 1, -Math.PI / 2);
      SpatialInertia M = new SpatialInertia();
      jaw.getInertia(M);
      M.transform(R);
      jaw.setInertia(M);
   }

   public void setThyroidDynamicProps(RigidBody thyroid) {
      if (thyroid == null) { return; }
      // Inertia Properties from ADAMS model - kg*mm*mm
      thyroid.setDynamic(true);
      thyroid.setRotationalInertia(new SymmetricMatrix3d(2.1537, // (0,0)
	    1.6109, // (1,1)
	    7.4320, // (2,2)
	    0.0, // (0,1)
	    0.0, // (0,2)
	    1.9663)); // (1,2)
      thyroid.setCenterOfMass(new Point3d(0.0, 14.92, -14.24)); // mm (from obj)
      thyroid.setMass(thyroCricoidMass); // kg
   }

   public void setCricoidDynamicProps(RigidBody cricoid) {
      if (cricoid == null) { return; }
      // Inertia Properties from ADAMS model - kg*mm*mm
      cricoid.setDynamic(true);
      cricoid.setRotationalInertia(new SymmetricMatrix3d(2.1537, // (0,0)
	    1.6109, // (1,1)
	    7.4320, // (2,2)
	    0.0, // (0,1)
	    0.0, // (0,2)
	    1.9663)); // (1,2)s
      cricoid.setCenterOfMass(new Point3d(0.0, 20.92, -23.24)); // mm (from obj)
      cricoid.setMass(thyroCricoidMass); // kg
   }

   public void setHyoidDynamicProps(RigidBody hyoid) {
      if (hyoid == null) { return; }
      // Inertia Properties from ADAMS model - kg*mm*mm
      hyoid.setDynamic(true);
      hyoid.setRotationalInertia(new SymmetricMatrix3d(2.1537, // (0,0)
	    1.6109, // (1,1)
	    7.4320, // (2,2)
	    0.0, // (0,1)
	    0.0, // (0,2)
	    1.9663)); // (1,2)
      hyoid.setCenterOfMass(new Point3d(0.0, 6.03, 4.48)); // mm (from obj)
      // hyoid.setMass(0.2); //kg
      hyoid.setMass(hyoidMass); // kg
   }

   private void transformVertebrae() {
      RigidTransform3d Xvert = new RigidTransform3d();
      RigidBody vertebrae = myRigidBodies.get("vertebrae");
      if (vertebrae == null) {
	 if (debug) {
	    System.out.println("transformVertebrae: Vertebrae is null");
	 }
	 return;
      }
      PolygonalMesh vertMesh = vertebrae.getMesh();
      if (vertMesh == null) {
	 if (debug) {
	    System.out.println("transformVertebrae: Vertebrae mesh is null");
	 }
	 return;
      }
      double amiraXRotDegrees = 94.3078;
      // Translation from Amira is for local coordinate; need additional
      // translation
      // Point3d amiraTrans = new Point3d(0.104019, 1.54776, 1.6985);
      // Point3d amiraTrans = new Point3d(0.104019, 6.02563, -7.26386);
      Point3d amiraTrans = new Point3d(0, 6.02563, -7.26386); // symmetric
      double amiraUniformScale = 1.2;

      amiraTrans.scale(CM_TO_MM);
      Xvert.p.set(amiraTrans);
      Xvert.R.setAxisAngle(1, 0, 0, Math.PI * (amiraXRotDegrees / 180.0));
      vertMesh.scale(amiraUniformScale);
      vertMesh.transform(Xvert);
      // vertebrae.setPose(Xvert);
   }

   private RigidTransform3d readAnatomyTransformInfo(String name)
	 throws IOException {
      // apply tranformations to rigid bodies that require it
      // if "name.transform" exists, read file and apply transform
      Matrix4d M = null;
      RigidTransform3d X;
      ReaderTokenizer rtok;
      try {
	 rtok = new ReaderTokenizer(new FileReader(ArtisynthPath
	       .getSrcRelativePath(JawModel.class, "geometry/" + name
		     + ".transform")));
      } catch (Exception e) {
	 if (debug) {
	    System.out.println("Can't find transform file for " + name);
	    System.out.println("assuming no transform required");
	 }
	 return null;
      }

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 if (rtok.ttype == ReaderTokenizer.TT_WORD) {
	    if (rtok.sval.compareTo("setTransform") == 0) {
	       if (debug) {
		  System.out.println(name + " parser found: " + rtok.sval);
	       }
	       while (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
		  if (debug) {
		     System.out.println("reading " + rtok.sval);
		  }
	       // flush the "setTransform" flag in Amira landmark file
	       if (rtok.ttype == ReaderTokenizer.TT_NUMBER) {
		  rtok.pushBack();
		  M = new Matrix4d();
		  M.scan(rtok);
		  M.transpose(); // amira uses opposite row/col convention for
		  // matrices
		  if (true) {
		     System.out.println(name + " pose = " + M.toString());
		  }
	       }
	       break;
	    }
	 }
      }
      if (M == null) throw new IOException("Error reading transform info from "
	    + name + " file");
      else {
	 X = new RigidTransform3d();
	 X.set((Matrix) M);
	 return X;
      }

   }

   public void assembleRigidBodies() throws IOException {
      for (BodyInfo bodyInfo : bodyInfoList) {
	 createAndAddBody(bodyInfo.name, bodyInfo.meshName);
      }

      // transformed vertebrae mesh has been saved to vertebrae_t.obj
      // therefore no need to transform here
      // transformVertebrae();

      // Rigidbody dampers for dynamic components
      setBodyDamping("jaw", myJawDampingT * unitConversion, myJawDampingR
	    * unitConversion);
      setBodyDamping("hyoid", myHyoidDampingT * unitConversion, myHyoidDampingR
	    * unitConversion);
      setBodyDamping("thyroid", myHyoidDampingT * unitConversion,
	    myHyoidDampingR * unitConversion);
      setBodyDamping("cricoid", 0.1, 10);
   }

   public void setBodyDamping(String name, double td, double rd) {
      RigidBody body = myRigidBodies.get(name);
      if (body == null) { return; }
      body.setFrameDamping(td);
      body.setRotaryDamping(rd);
   }

   private void readLandmarkPoints(String filename, ArrayList<Point3d> pointList)
	 throws IOException {
      if (pointList == null) {
	 System.err
	       .println("readLandmarkPoints Error: pointList argument is null");
	 return;
      }
      Point3d point;
      ReaderTokenizer rtok;
      try {
	 rtok = new ReaderTokenizer(new FileReader(ArtisynthPath
	       .getSrcRelativePath(JawModel.class, "geometry/" + filename
		     + ".landmarkAscii")));
      } catch (Exception e) {
	 System.err.println("Error opening landmark file: " + filename);
	 e.printStackTrace();
	 return;
      }

      rtok.wordChars("./@");

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 if (rtok.ttype == ReaderTokenizer.TT_WORD
	       && rtok.sval.compareTo("@1") == 0) {
	    if (debug) {
	       System.out.println(filename + ": found " + rtok.sval);
	    }
	    rtok.nextToken();
	    if (rtok.ttype != ReaderTokenizer.TT_NUMBER) {
	       rtok.pushBack(); // not at data yet, look for next "@1" word
	       continue;
	    } else {
	       rtok.pushBack(); // we have found landmark data - scan into
	       // Point3ds until EOF

	       while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
		  rtok.pushBack();
		  point = new Point3d();
		  try {
		     point.scan(rtok);
		  } catch (Exception e) {
		     System.err
			   .println("readLandmarkPoints: error scanning Point3d as expected.");
		     e.printStackTrace();
		     return;
		  }
		  point.scale(CM_TO_MM); // amira geometry are in cm (convert to
		  // mm)
		  pointList.add(point);
		  if (debug) {
		     System.out.println("Read point (" + point.toString()
			   + ") from " + filename + ".landmarkAscii");
		  }
	       }
	       break;
	    }
	 }
      }
      if (pointList.isEmpty())
	 throw new IOException("readLandmarkPoints: error no points found in "
	       + filename + ".landmarkAscii file");
   }

   public void attachMarkers() {
      Point3d[] pts;
      for (int k = 0; k < muscleList.size(); k++) {
	 String name = muscleList.get(k);
	 MuscleInfo info = muscleInfo.get(name);
	 RigidBody origin = myRigidBodies.get(info.origin);
	 RigidBody insertion = myRigidBodies.get(info.insertion);

	 try {
	    if (origin == null || insertion == null) { throw new Exception(
		  "muscle attached to non-existent body"); }
	    if (info.isPaired()) {
	       pts = AmiraLandmarkReader.read(ArtisynthPath.getSrcRelativePath(
		     JawModel.class, "geometry/l" + name + ".landmarkAscii"),
		     CM_TO_MM);
	       checkOriginInsertion(pts);

	       addFrameMarker(new FrameMarker("l" + name + "_origin"), origin,
		     pts[0]);
	       addFrameMarker(new FrameMarker("l" + name + "_insertion"),
		     insertion, pts[1]);
	       // right side muscle is mirrored in x direction
	       addFrameMarker(new FrameMarker("r" + name + "_origin"),
		     myRigidBodies.get(info.origin),
		     createRightSidePoint(pts[0]));
	       addFrameMarker(new FrameMarker("r" + name + "_insertion"),
		     myRigidBodies.get(info.insertion),
		     createRightSidePoint(pts[1]));
	    } else {
	       pts = AmiraLandmarkReader.read(ArtisynthPath.getSrcRelativePath(
		     JawModel.class, "geometry/" + name + ".landmarkAscii"),
		     CM_TO_MM);
	       addFrameMarker(new FrameMarker(name + "_origin"), origin, pts[0]);
	       // right side attachment is mirrored in x direction
	       addFrameMarker(new FrameMarker(name + "_insertion"), insertion,
		     createRightSidePoint(pts[0]));

	    }
	 } catch (Exception e) {
	    System.out.println(e.getMessage());
	    System.out.println("unable to add markers for muscle, removing: "
		  + info.fullName);
	    muscleList.remove(k);
	    k--; // update index to reflect purged kth muscle
	    continue;
	 }
      }
   }

   private void checkOriginInsertion(Point3d[] markerPts) {
      if (markerPts[0].z < markerPts[1].z) { // origin and insertion point are
	 // mixed up, do swap
	 Point3d tmp = markerPts[0];
	 markerPts[0] = markerPts[1];
	 markerPts[1] = tmp;
      }
   }

   private Muscle createPeckMuscle (
	 String name, double maxForce, double optLen, double maxLen, double ratio) {
      Muscle m = new Muscle(name);
      m.setPeckMuscleMaterial(maxForce, optLen, maxLen, ratio);
      return m;
   }
   
   public void assembleMuscles() {
      /*
       * all muscle CSA values and 40 N/cm^2 constant taken from Peck 2000 Arch
       * Oral Biology
       */
      double shlpMaxForce = 66.9 / 0.7 * 0.3; // ihlp reported as 70% of muscle
      // [Peck 2000]
      double mylohyoidMaxForce = 177.0 / 100 / 2.0 * 40.0; // mylohyoid 177 mm^2
      // from
      // Buchilliard2009
      // JASA, divided
      // equally into
      // anterior and
      // posterior parts
      double geniohyoidMaxForce = 80.0 / 100 * 40.0; // geniohyoid 80 mm^2 from
      // Buchilliard2009 JASA
      double postdigMaxForce = 40.0; // same CSA as antdig from vanEijden 1997
      // Anat Rec
      double stylohyoidMaxForce = 0.39 * 40; // 0.39 cm^2 from van Eijden 1997
      // Anat Rec

      // NB - max length and opt length get overwritten in
      // updateMuscleLengthProps()
      // Skull - Jaw Muscles
      myMuscles.add(createPeckMuscle("lat", 158.0, 75.54, 95.92, 0.5)); // lat
      myMuscles.add(createPeckMuscle("ldm", 81.6, 29.07, 44.85, 0.29)); // ldm
      myMuscles.add(createPeckMuscle("lip", 66.9, 31.5, 41.5, 0.0)); // lip
      // (opener)
      myMuscles.add(createPeckMuscle("lmp", 174.8, 40.51, 50.63, 0.64)); // lmp
      myMuscles.add(createPeckMuscle("lmt", 95.6, 65.81, 93.36, 0.48)); // lmt
      myMuscles.add(createPeckMuscle("lpt", 75.6, 77.11, 101.08, 0.51)); // lpt
      myMuscles.add(createPeckMuscle("lsm", 190.4, 51.46, 66.88, 0.46)); // lsm
      myMuscles.add(createPeckMuscle("lsp", shlpMaxForce, 27.7, 37.7, 0.0)); // lsp
      // (opener)
      myMuscles.add(createPeckMuscle("rat", 158.0, 75.54, 95.92, 0.5)); // rat
      myMuscles.add(createPeckMuscle("rdm", 81.6, 29.07, 44.85, 0.29)); // rdm
      myMuscles.add(createPeckMuscle("rip", 66.9, 31.5, 41.5, 0.0)); // rip
      // (opener)
      myMuscles.add(createPeckMuscle("rmp", 174.8, 40.51, 50.63, 0.64)); // rmp
      myMuscles.add(createPeckMuscle("rmt", 95.6, 65.81, 93.36, 0.48)); // rmt
      myMuscles.add(createPeckMuscle("rpt", 75.6, 77.11, 101.08, 0.51)); // rpt
      myMuscles.add(createPeckMuscle("rsm", 190.4, 51.46, 66.88, 0.46)); // rsm
      myMuscles.add(createPeckMuscle("rsp", shlpMaxForce, 27.7, 37.7, 0.0)); // rsp
      // (opener)

      // Laryngeal Muscles (jaw-hyoid, skull-hyoid)
      myMuscles.add(createPeckMuscle("lad", 40.0, 35.1, 45.1, 0.0)); // lad
      // (opener)
      myMuscles.add(createPeckMuscle("lpd", postdigMaxForce, 35.1, 45.1,
	    0.0)); // lpd
      myMuscles.add(createPeckMuscle("lam", mylohyoidMaxForce, 35.1, 45.1,
	    0.0));// Left Anterior Mylohyoid
      myMuscles.add(createPeckMuscle("lpm", mylohyoidMaxForce, 35.1, 45.1,
	    0.0));// Left Posterior Mylohyoid
      myMuscles.add(createPeckMuscle("lgh", geniohyoidMaxForce, 35.1, 45.1,
	    0.0));// Left Geniohyoid
      myMuscles.add(createPeckMuscle("lsh", stylohyoidMaxForce, 35.1, 45.1,
	    0.0));// Left Stylohyoid
      myMuscles.add(createPeckMuscle("rad", 40.0, 35.1, 45.1, 0.0)); // rad
      // (opener)
      myMuscles.add(createPeckMuscle("rpd", postdigMaxForce, 35.1, 45.1,
	    0.0)); // rpd
      myMuscles.add(createPeckMuscle("ram", mylohyoidMaxForce, 35.1, 45.1,
	    0.0));// Right Anterior Mylohyoid
      myMuscles.add(createPeckMuscle("rpm", mylohyoidMaxForce, 35.1, 45.1,
	    0.0));// Right Posterior Mylohyoid
      myMuscles.add(createPeckMuscle("rgh", geniohyoidMaxForce, 35.1, 45.1,
	    0.0));// Right Geniohyoid
      myMuscles.add(createPeckMuscle("rsh", stylohyoidMaxForce, 35.1, 45.1,
	    0.0));// Right Stylohyoid

      // hyoid depressors
      myMuscles.add(createPeckMuscle("lth", 20.0, 35.1, 45.1, 0.5));// Left
      // Thyrohyoid
      myMuscles.add(createPeckMuscle("lsteh", 50.0, 35.1, 45.1, 0.5));// Left
      // Sternohyoid
      myMuscles.add(createPeckMuscle("loh", 50.0, 35.1, 45.1, 0.5));// Left
      // Omohyoid
      myMuscles.add(createPeckMuscle("rth", 20.0, 35.1, 45.1, 0.5));// Right
      // Thyrohyoid
      myMuscles.add(createPeckMuscle("rsteh", 50.0, 35.1, 45.1, 0.5));// Right
      // Sternohyoid
      myMuscles.add(createPeckMuscle("roh", 50.0, 35.1, 45.1, 0.5));// Right
      // Omohyoid

      // Laryngeal Muscles (thyroid-cricoid, crico-arytenoid, sternum)
      myMuscles.add(createPeckMuscle("lpc", 20.0, 35.1, 45.1, 0.5));// Left
      // Posterior
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("llc", 20.0, 35.1, 45.1, 0.5));// Left
      // Lateral
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("lpct", 20.0, 35.1, 45.1, 0.5));// Left
      // Posterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("lact", 20.0, 35.1, 45.1, 0.5));// Left
      // Anterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("lstet", 20.0, 35.1, 45.1, 0.5));// Left
      // Sternothyroid
      myMuscles.add(createPeckMuscle("rpc", 20.0, 35.1, 45.1, 0.5));// Right
      // Posterior
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("rlc", 20.0, 35.1, 45.1, 0.5));// Right
      // Lateral
      // Cricoarytenoid
      myMuscles.add(createPeckMuscle("rpct", 20.0, 35.1, 45.1, 0.5));// Right
      // Posterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("ract", 20.0, 35.1, 45.1, 0.5));// Right
      // Anterior
      // Cricothyroid
      myMuscles.add(createPeckMuscle("rstet", 20.0, 35.1, 45.1, 0.5));// Right
      // Sternothyroid

      myMuscles.add(createPeckMuscle("ta", 20.0, 35.1, 45.1, 0.5));// Transverse
      // Arytenoid
      // (midline
      // between
      // left
      // and
      // right
      // arytenoids)

      // get length ratios to update muscle lengths for new Amira geometry
      Muscle m;
      for (int k = 0; k < myMuscles.size(); k++) {
	 m = myMuscles.get(k);
      }

   }

   public void printMuscleMaxForces() {

      for (AxialSpring s : myAxialSprings) {
	 if (s instanceof Muscle) {
	    Muscle m = (Muscle) s;
	    String fullname = "";
	    if (muscleInfo.containsKey(m.getName().substring(1))) {
	       fullname = (m.getName().startsWith("l") ? "Left " : "Right ")
		     + muscleInfo.get(m.getName().substring(1)).fullName;
	    }
	    System.out.printf("%s %g -- %s\n", m.getName().toUpperCase(), 
	       Muscle.getMaxForce (m), fullname);

	 }
      }

   }

   public void assembleMuscleGroups() {

      for (MuscleGroupInfo info : muscleGroupInfo) {
	 // bilateral excitor
	 MuscleExciter bilateral = new MuscleExciter("bi_" + info.name);

	 // add groups for left and right sides
	 for (int i = 0; i < 2; i++) // add groups for left and right sides
	 {
	    String prefix = (i == 0 ? "l" : "r");
	    String fullPrefix = (i == 0 ? "Left " : "Right ");
	    MuscleExciter exciter = new MuscleExciter(prefix + info.name);
	    for (String target : info.coactivators) {
	       ExcitationComponent c = (Muscle) myAxialSprings.get(prefix
		     + target);
	       if (c == null) { // look for target in excitors list
		  c = myExciterList.get(prefix + target);
	       }
	       if (c == null) continue;

	       exciter.addTarget(c, 1.0);
	    }
	    addMuscleExciter(exciter);
	    bilateral.addTarget(exciter, 1.0);
	    muscleAbbreviations.put(prefix + info.name, fullPrefix
		  + info.fullName);
	 }

	 addMuscleExciter(bilateral);
	 muscleAbbreviations.put("bi_" + info.name, "Bilateral "
	       + info.fullName);

      }

   }

   public void updateMuscleLengthProps() {
      for (Muscle m : myMuscles)
	 m.resetLengthProps();
   }

   public void pruneMuscleList() {
      for (int k = 0; k < myMuscles.size(); k++) {
	 Muscle m = myMuscles.get(k);
	 String name = m.getName();
	 boolean inMuscleList = false;
	 for (int i = 0; i < muscleList.size(); i++) {
	    if (name.endsWith(muscleList.get(i))) {
	       inMuscleList = true;
	       break;
	    }
	 }
	 if (!inMuscleList) {
	    myMuscles.remove(k);
	    k--; // update index to reflect purged kth muscle
	 }
      }
   }

   /*
    * add FrameMarkers and Muscles to model
    */
   public void attachMuscles() {
      for (int k = 0; k < muscleList.size(); k++) {
	 String name = muscleList.get(k);
	 if (muscleInfo.get(muscleList.get(k)).isPaired()) {
	    addMuscle("l" + name);
	    muscleAbbreviations.put("l" + name, "Left "
		  + muscleInfo.get(name).fullName);
	    addMuscle("r" + name);
	    muscleAbbreviations.put("r" + name, "Right "
		  + muscleInfo.get(name).fullName);
	 } else {
	    addMuscle(name);
	 }
      }
   }

   public void assembleBilateralExcitors() {
      for (int k = 0; k < muscleList.size(); k++) {
	 String name = muscleList.get(k);
	 if (muscleInfo.get(muscleList.get(k)).isPaired()) {
	    Muscle left = (Muscle) myAxialSprings.get("l" + name);
	    Muscle right = (Muscle) myAxialSprings.get("r" + name);
	    if (left != null && right != null) {
	       String excitorName = "bi_" + name;
	       MuscleExciter bilateral = new MuscleExciter(excitorName);
	       bilateral.addTarget(left, 1.0);
	       bilateral.addTarget(right, 1.0);
	       addMuscleExciter(bilateral);
	       String fullName = muscleInfo.get(name).fullName;
	       muscleAbbreviations.put(excitorName, "Bilateral " + fullName);
	    }
	 }
      }

   }

   private Muscle findMuscle(String name) {
      for (Muscle m : myMuscles) {
	 if (name.compareTo(m.getName()) == 0) return m;
      }
      return null;
   }

   private void addMuscle(String name) {
      Muscle m = findMuscle(name);
      if (m == null) {
	 System.err.println(name + " muscle not found.");
	 return;
      }
      m.setFirstPoint(myFrameMarkers.get(name + "_origin"));
      m.setSecondPoint(myFrameMarkers.get(name + "_insertion"));
      AxialSpring.setDamping (m, myMuscleDamping);
      addAxialSpring(m);
   }

   public void detachMuscles() {
      for (Muscle m : myMuscles) {
	 removeAxialSpring(m);
      }
   }

   public void setPharynxStiffness(double k) {
      for (int i = 0; i < 2; i++) {
	 String name = (i == 0 ? "l" : "r") + "phc";
	 AxialSpring.setStiffness (myAxialSprings.get(name), k);
      }
   }

   public void addPharynxSprings() {
      RigidBody sternum = myRigidBodies.get("sternum");
      RigidBody hyoid = myRigidBodies.get("hyoid");

      if (hyoid == null || sternum == null) return;

      // add spring for pharyngeal constrictor

      for (int i = 0; i < 2; i++) {
	 // i == 0 is left-side, i == 1 is right-side
	 FrameMarker origin = new FrameMarker();
	 FrameMarker insertion = new FrameMarker();
	 addFrameMarker(origin, hyoid, new Point3d(i == 0 ? 16.88 : -16.88,
	       8.16, 5.29));
	 addFrameMarker(insertion, sternum, new Point3d(
	       i == 0 ? 17.92 : -17.92, 32.71, 18.46));
	 String name = (i == 0 ? "l" : "r") + "phc";
	 origin.setName(name + "_origin");
	 insertion.setName(name + "_insertion");
	 // JawMuscle constrictor = createPeckMuscle(name, 50, 24, 26, 0.0);
	 // constrictor.setPassiveFraction(0.025);
	 AxialSpring constrictor = new AxialSpring(name,
	       defaultPharnyxStiffness, 0.0, 26);
	 constrictor.setFirstPoint(origin);
	 constrictor.setSecondPoint(insertion);
	 RenderProps.setLineStyle(constrictor, LineStyle.CYLINDER);
	 RenderProps.setLineRadius(constrictor,
	       JawModel.MEMBRANE_CYL_RADIUS);
	 RenderProps.setLineColor(constrictor, Color.WHITE);
	 addAxialSpring(constrictor);
      }

   }

   private void addMembraneStrand(String name, Point3d originPt,
	 Point3d insertionPt, RigidBody oBody, RigidBody iBody) {
      FrameMarker origin;
      FrameMarker insertion;
      AxialSpring membraneStrand;
      if (oBody == null || iBody == null) {
	 System.err.println("addMembraneStrand Error: rigidbody args null");
	 return;
      }
      Vector3d disp = new Vector3d();

      // render props for markers
      RenderProps memPointProps = RenderProps.createPointProps(null);
      memPointProps.setPointColor(Color.WHITE);
      memPointProps.setPointRadius(MEMBRANE_PT_RADIUS);

      origin = new FrameMarker(originPt);
      origin.setName(name + "_origin");
      origin.setRenderProps(memPointProps.clone());
      origin.setFrame(oBody);
      myFrameMarkers.add(origin);
      insertion = new FrameMarker(insertionPt);
      insertion.setName(name + "_insertion");
      insertion.setRenderProps(memPointProps.clone());
      insertion.setFrame(iBody);
      myFrameMarkers.add(insertion);

      disp.sub(origin.getPosition(), insertion.getPosition());
      if (name.contains("thMem")) { // System.out.println("adding thMem: stiffness = "
	 // + thMemStiff);
	 membraneStrand = new AxialSpring(name, thMemStiff, thMemDamp, disp
	       .norm());
      } else { // System.out.println("adding ctrMem: stiffness = " +
	 // ctrMemStiff);
	 membraneStrand = new AxialSpring(name, ctrMemStiff, ctrMemDamp, disp
	       .norm());
      }

      RenderProps.setLineStyle(membraneStrand, LineStyle.CYLINDER);
      RenderProps.setLineRadius(membraneStrand, MEMBRANE_CYL_RADIUS);
      RenderProps.setLineColor(membraneStrand, Color.WHITE);
      membraneStrand.setFirstPoint(origin);
      membraneStrand.setSecondPoint(insertion);
      addAxialSpring(membraneStrand);
      memPts.add(origin);
      memPts.add(insertion);
      memList.add(membraneStrand);

   }

   public void attachMembraneMesh(String oBodyName, String iBodyName,
	 String memName) {
      RigidBody oBody = myRigidBodies.get(oBodyName);
      RigidBody iBody = myRigidBodies.get(iBodyName);
      if (oBody == null || iBody == null) {
	 System.err.println("attachMembraneMesh: " + memName
	       + " not attached because body null.");
	 return;
      }

      Point3d[] pts = null;
      try {
	 pts = AmiraLandmarkReader.read(ArtisynthPath.getSrcRelativePath(
	       JawModel.class, "geometry/l" + memName + ".landmarkAscii"),
	       CM_TO_MM);
      } catch (IOException e) {
	 e.printStackTrace();
      }

      // NB - landmarks ordered in origin, insertion pairs
      // add left membrane strands in a spring mesh (for stabilizing the hyoid
      // bone)

      // TODO - need to fix multiple frame markers attached to thyroid and hyoid
      // at
      // same positions - should work for now...

      if (memName.compareTo("ctrMem") == 0.0) {
	 if (debug) {
	    System.out
		  .println("Applying vertical offset to trachea markers for ctrMem");
	 }
	 doTracheaOffset(pts);
      }

      int num = 0;
      for (int k = 0; k < pts.length; k += 2) {
	 addMembraneStrand("l" + memName + num++, pts[k], pts[k + 1], oBody,
	       iBody);
	 if (k + 2 < pts.length) {
	    addMembraneStrand("l" + memName + num++, pts[k], pts[k + 3], oBody,
		  iBody);
	    addMembraneStrand("l" + memName + num++, pts[k + 2], pts[k + 1],
		  oBody, iBody);
	 }
      }

      // NB - landmarks ordered in origin, insertion pairs
      // add left membrane strands in a spring mesh (for stabilizing the hyoid
      // bone)
      num = 0;
      for (int k = 0; k < pts.length; k += 2) {
	 addMembraneStrand("r" + memName + num++, createRightSidePoint(pts[k]),
	       createRightSidePoint(pts[k + 1]), oBody, iBody);
	 if (k + 2 < pts.length) {
	    addMembraneStrand("r" + memName + num++,
		  createRightSidePoint(pts[k]),
		  createRightSidePoint(pts[k + 3]), oBody, iBody);
	    addMembraneStrand("r" + memName + num++,
		  createRightSidePoint(pts[k + 2]),
		  createRightSidePoint(pts[k + 1]), oBody, iBody);
	 }
      }
   }

   public Point3d createRightSidePoint(Point3d leftSidePt) {
      Point3d rightSidePt = new Point3d();
      if (leftSidePt != null) {
	 rightSidePt.set(leftSidePt);
	 rightSidePt.x = -rightSidePt.x; // right-left mirrored in x-axis
      }
      return rightSidePt;
   }

   public void doTracheaOffset(Point3d[] pts) {
      // add vertical offset to all odd points
      int count = 0;
      for (Point3d p : pts) {
	 if (count % 2 == 1) {
	    p.z = p.z + VERTICAL_TRACHEA_OFFSET;
	 }
	 count++;
      }
   }

   private int findLargestZValue(Point3d[] pointList) {
      if (pointList.length < 1) return 0;
      double maxZ = pointList[0].z;
      int maxZindex = 0, k = 0;
      for (Point3d p : pointList) {
	 if (p.z > maxZ) {
	    maxZ = p.z;
	    maxZindex = k;
	 }
	 k++;
      }
      return maxZindex;
   }

   public FrameMarker addFixedMarker(String bodyName, Point3d location,
	 String markerName) {
      // add marker at location for body
      RigidBody body = myRigidBodies.get(bodyName);
      if (body == null) {
	 System.err.println("addFixedMarker Error: "
	       + "body specified don't exist, can't create marker");
	 return null;
      }
      FrameMarker marker = new FrameMarker();
      addFrameMarker(marker, body, location);
      marker.setName(markerName);
      RenderProps fixedProps = marker.createRenderProps();
      fixedProps.setPointColor(Color.BLUE);
      fixedProps.setPointRadius(FIXED_PT_RADIUS);
      marker.setRenderProps(fixedProps);
      // myFrameMarkers.add(marker);
      return marker;
   }

   public void addComMarker(String bodyName) {
      // add marker at COM
      Point3d com = new Point3d();
      RigidBody body = myRigidBodies.get(bodyName);
      if (body == null) {
	 // System.err.println("addComMarker Error: "
	 // + "body specified don't exist, can't create com marker");
	 return;
      }
      body.getCenterOfMass(com);
      FrameMarker comMarker = new FrameMarker();
      addFrameMarker(comMarker, body, com);
      comMarker.setName(bodyName + "Com");
      RenderProps comProps = RenderProps.createPointProps(null);
      comProps.setPointColor(Color.ORANGE);
      comProps.setPointRadius(FIXED_PT_RADIUS);
      comMarker.setRenderProps(comProps);
      // myFrameMarkers.add(comMarker);
   }

   public void addFixedMarkers() {
      // get fixed markers for jaw
      FrameMarker m;
      boolean hasRightPair;
      RigidBody body;
      String name;
      Point3d[] pts;
      String[] list = new String[] { "jaw", "maxilla", "hyoid", "thyroid" };
      ArrayList<String> bodyList = new ArrayList<String>();
      for (int i = 0; i < list.length; i++)
	 bodyList.add(list[i]);
      RenderProps fixedProps = RenderProps.createPointProps(null);
      fixedProps.setPointRadius(FIXED_PT_RADIUS);
      fixedProps.setPointColor(Color.WHITE);
      for (int i = 0; i < bodyList.size(); i++) {
	 name = bodyList.get(i);
	 body = myRigidBodies.get(name);
	 try {
	    pts = AmiraLandmarkReader.read(ArtisynthPath.getSrcRelativePath(
		  JawModel.class, "geometry/" + name
			+ "FixedMarkers.landmarkAscii"), CM_TO_MM);
	 } catch (IOException e) {
	    e.printStackTrace();
	    return;
	 }

	 // find tmj point index for jaw
	 int tmjPointIndex = -1;
	 if (name.compareTo("jaw") == 0.0) {
	    tmjPointIndex = findLargestZValue(pts);
	 }

	 for (int k = 0; k < pts.length; k++) {
	    m = new FrameMarker();
	    m.setRenderProps(fixedProps.clone());
	    hasRightPair = false;

	    // find incisor point (if pt is in sagittal plane)
	    if (Math.abs(pts[k].x) < CLOSE_TO_SAGGITAL) {
	       pts[k].x = 0.0; // fix to sagittal plane
	       if (name.compareTo("maxilla") == 0.0) {
		  m.setName("upperincisor");
	       } else if (name.compareTo("jaw") == 0.0) {
		  m.setName("lowerincisor");
		  RenderProps.setPointRadius(m, CONTACT_PT_RADIUS);
		  pts[k].y -= 1.0;
	       } else if (name.compareTo("hyoid") == 0.0 && k == 0) {
		  m.setName("hyoidRefMarker");
	       } else if (name.compareTo("thyroid") == 0.0 && k == 0) {
		  m.setName("thyroidRef");
	       } else
		  RenderProps.setVisible(m, false);
	    } else {
	       hasRightPair = true;
	    }

	    if (k == tmjPointIndex) { // lower tmj point (by -4.0) from top
	       // surface of mesh to approx centre of
	       // condyle
	       pts[k].z = pts[k].z + VERTICAL_TMJ_OFFSET;
	       m.setName("ltmj");
	       RenderProps.setPointRadius(m, CONTACT_PT_RADIUS);
	       RenderProps.setPointColor(m, Color.BLUE);
	    }
	    addFrameMarker(m, body, pts[k]);

	    if (hasRightPair == true) {
	       m = new FrameMarker();
	       addFrameMarker(m, body, createRightSidePoint(pts[k]));
	       m.setRenderProps(fixedProps.clone());
	       if (k == tmjPointIndex) {
		  m.setName("rtmj");
		  RenderProps.setPointRadius(m, CONTACT_PT_RADIUS);
		  RenderProps.setPointColor(m, Color.BLUE);
	       }
	    }

	 }
      }

      // add C1 marker
      body = myRigidBodies.get("sternum");
      if (body != null) {
	 m = new FrameMarker();
	 addFrameMarker(m, body, c1Point);
	 m.setName("C1point");
	 m.setRenderProps(fixedProps.clone());
	 RenderProps.setPointColor(m, Color.MAGENTA);
      }

      // add COM markers
      addComMarker("jaw");
      addComMarker("hyoid");
      addComMarker("thyroid");
      addComMarker("cricoid");

      // add bite points
      FrameMarker fm;
      fm = addFixedMarker("jaw", leftBiteLocation, "lbite");
      RenderProps.setPointRadius(fm, CONTACT_PT_RADIUS);
      Point3d rightBiteLocation = new Point3d(leftBiteLocation);
      rightBiteLocation.x *= -1;
      fm = addFixedMarker("jaw", rightBiteLocation, "rbite");
      RenderProps.setPointRadius(fm, CONTACT_PT_RADIUS);

      // add reference point for hyoid
      RigidBody hyoid = myRigidBodies.get("hyoid");
      if (hyoid != null) {
	 m = new FrameMarker();
	 addFrameMarker(m, hyoid, hyoidRefPos);
	 m.setName("hyoidRef");
      }

   }

   public double getUnitConversion() {
      return unitConversion;
   }

   public void setUnitConversion(double uc) {
      unitConversion = uc;
   }

   public int getNumMuscles() {
      return myMuscles.size();
   }
   
   public ArrayList<Muscle> getMuscles() {
    return myMuscles;
   }


   public Point3d getJointPoint(int idx) {
      return jointPoint[idx];
   }

   public boolean getTransparency() {
      return myTransparencyP;
   }

   public void setTransparency(boolean transparent) {
      myTransparencyP = transparent;
      for (RigidBody body : myRigidBodies) {
	 if (body.getMesh() == null) {
	    continue;
	 }
	 RenderProps.setAlpha(body, transparent ? transparentAlpha
	       : opaqueAlpha);
      }
   }

   /**
    * Iterate through all rigidbodies associated with the MechModel and change
    * mesh render setting for faces to reflect show parameter
    * 
    * @param facesVisible visibility flag
    */
   public void showMeshFaces(boolean facesVisible) {
      myRenderFacesP = facesVisible;
      for (RigidBody body : myRigidBodies) {
	 RenderProps.setVisible(body, facesVisible);
      }
      return;
   }

   public boolean getRenderFaces() {
      return myRenderFacesP;
   }

   public void showMeshEdges(boolean edgesVisible) {
      myRenderEdgesP = edgesVisible;
      for (RigidBody body : myRigidBodies) {
	 RenderProps.setDrawEdges(body, edgesVisible);
	 RenderProps.setLineColor(body, Color.WHITE);
      }
      return;
   }

   public boolean getRenderEdges() {
      return myRenderEdgesP;
   }

   public void setMuscleColor(Color newColor) {
      RenderProps.setLineColor(this, newColor);
   }

   public Color getMuscleColor() {
      return getRenderProps().getLineColor();
   }

   public void setRigidBodyColor(Color newColor) {
      for (RigidBody rb : myRigidBodies) {
	 RenderProps.setFaceColor(rb, newColor);
      }
   }

   public Color getRigidBodyColor() {
      // assume all bodies have same color as first in list
      return myRigidBodies.get(0).getRenderProps().getFaceColor();
   }

   public void scaleDistance(double s) {
      super.scaleDistance(s);
      updateMuscleLengthProps();
      c1Point.scale(s);
      initHeadRotationVertebrae();
      medWallOffset[0] *= s;
      medWallOffset[1] *= s;
      ltrlWallOffset[0] *= s;
      ltrlWallOffset[1] *= s;
      postWallOffset[0] *= s;
      postWallOffset[1] *= s;
      curvParams[0] *= s;
      curvParams[1] *= s;
      curvParams[2] *= s;
      curvParams[3] *= s;
   }

   public double getJawCentricRotation() {
      return jawCentricRotation;
   }

   protected RigidTransform3d XCondyleToWorld = null;

   protected RigidTransform3d XCentricRotation = new RigidTransform3d();

   public void setJawCentricRotation(double jawCentricRotation) {
      this.jawCentricRotation = jawCentricRotation;
      if (XCondyleToWorld == null) {
	 FrameMarker condyle = myFrameMarkers.get("ltmj");
	 if (condyle == null) return;
	 Point3d condylePt = new Point3d(condyle.getLocation()); // body coords
	 // ?
	 condylePt.x = 0; // assume standard jaw position
	 XCondyleToWorld = new RigidTransform3d();
	 XCondyleToWorld.p.negate(condylePt);
      }
      XCentricRotation.p.setZero();
      XCentricRotation.R.setAxisAngle(1, 0, 0, Math
	    .toRadians(jawCentricRotation));
      XCentricRotation.mul(XCentricRotation, XCondyleToWorld);
      XCentricRotation.mulInverseLeft(XCondyleToWorld, XCentricRotation);

      myRigidBodies.get("jaw").transformGeometry(XCentricRotation);
   }

   public double getHeadRotation() {
      return myHeadRotation;
   }

   protected RigidTransform3d Xbw;

   protected RigidTransform3d Xsr;

   public void initHeadRotationVertebrae() {
      RigidBody vert = myRigidBodies.get("vertebrae");
      if (vert == null) {
	 System.out.println("setHeadRotationVertebrae: no vertebrae");
	 return;
      }
      Xbw = new RigidTransform3d();
      Xbw.p.set(c1Point); // constant

      RigidTransform3d Xsw = new RigidTransform3d();
      vert.getPose(Xsw);

      Xsr = new RigidTransform3d();
      Xsr.mulInverseLeft(Xbw, Xsw);

   }

   public void setHeadRotation(double degrees) {
      RigidBody body;
      String[] bodiesToRot = new String[] { "hyoid", "thyroid", "cricoid",
	    "pharynx", "sternum", "vertebrae" };
      // RigidTransform3d Xbw = new RigidTransform3d();
      // Xbw.p.set(c1Point); // constant

      if (Xbw == null || Xsr == null) initHeadRotationVertebrae();

      RigidTransform3d newXrot = new RigidTransform3d();
      newXrot.R.setAxisAngle(1, 0, 0, -Math.PI * (degrees / 180.0));

      RigidTransform3d newTvr = new RigidTransform3d();

      // System.out.println("newTvr = \n" + newTvr.toString("%8.2f"));

      for (int k = 0; k < bodiesToRot.length; k++) {
	 body = myRigidBodies.get(bodiesToRot[k]);
	 if (body != null) {
	    if (bodiesToRot[k].compareTo("vertebrae") == 0.0) {
	       newTvr.mul(newXrot, Xsr);
	       newTvr.mul(Xbw, newTvr);
	    } else {
	       newTvr.mulInverseRight(newXrot, Xbw);
	       newTvr.mul(Xbw, newTvr);
	    }
	    // body.setPose(newTvr);
	    body.transformGeometry(newTvr);
	 } else {
	    if (debug) {
	       System.out.println("setHeadRotation: body null "
		     + bodiesToRot[k]);
	    }
	 }
      }
      myHeadRotation = degrees;

      // for (FrameMarker m : frameMarkers ())
      // {
      // m.updateState ();
      // }
   }

   public double getThMemStiffness() {
      return thMemStiff;
   }

   public void setThMemStiffness(double k) {
      for (AxialSpring l : myAxialSprings) {
	 if (l.getName().contains("thMem")) {
	    AxialSpring.setStiffness(l, k);
	 }
      }
      thMemStiff = k;
   }

   public double getCtrMemStiffness() {
      return ctrMemStiff;
   }

   public void setCtrMemStiffness(double k) {
      for (AxialSpring l : myAxialSprings) {
	 if (l.getName().contains("ctrMem")) {
	    AxialSpring.setStiffness (l, k);
	 }
      }
      ctrMemStiff = k;
   }

   public String getMuscleName(String shortName) {
      if (muscleInfo.get(shortName) != null) return muscleInfo.get(shortName).fullName;
      else if (muscleAbbreviations.get(shortName) != null) return muscleAbbreviations
	    .get(shortName);
      else
	 return "unknown";
   }

   public void setShowMembrane(boolean show) {
      memShow = show;
      for (AxialSpring l : myAxialSprings) {
	 if (l.getName().contains("Mem")) {
	    RenderProps.setVisible(l, show);
	    RenderProps.setVisible(l.getFirstPoint(), show);
	    RenderProps.setVisible(l.getSecondPoint(), show);
	 }
      }
   }

   public boolean getShowMembrane() {
      return memShow;
   }

   public void setShowMuscles(boolean show) {
      musShow = show;
      for (AxialSpring m : myAxialSprings) {
	 if (m instanceof Muscle) {
	    RenderProps.setVisible(m, show);
	    RenderProps.setVisible(m.getFirstPoint(), show);
	    RenderProps.setVisible(m.getSecondPoint(), show);
	 }
      }
      return;
   }

   public boolean getShowMuscles() {
      return musShow;
   }

   public void setEnableMuscles(boolean enabled) {
      enableMuscles = enabled;
      for (Muscle m : myMuscles) {
	 m.setEnabled(enabled);
      }
   }

   public boolean getEnableMuscles() {
      return enableMuscles;
   }

   public void showMarkers(String[] markersToShow) {
      FrameMarker m;
      for (int i = 0; i < markersToShow.length; i++) {
	 // System.out.println("Show marker: " + markersToShow[i]);
	 m = myFrameMarkers.get(markersToShow[i]);
	 if (m != null) {
	    RenderProps props = m.getRenderProps();
	    if (props == null) {
	       props = RenderProps.createLineProps(null);
	    }
	    props.setPointRadius(1.0);
	    //props.setPointSlices(20);
	    props.setPointColor(Color.PINK);
	    m.setRenderProps(props);
	 }
      }
   }

   public void showMasseterMarkers() {
      String markersToShow[] = new String[] { "lsm_insertion", "rsm_insertion",
	    "ldm_insertion", "rdm_insertion", "lsm_origin", "rsm_origin",
	    "ldm_origin", "rdm_origin" };
      showMarkers(markersToShow);
   }

   /*
    * getJawMuscleWrench -- computes wrench due to markers that are muscle
    * endpoints. XXX - note that if markers are also attached to non-muscle
    * force effectors that force will also be added to the returned wrench. For
    * jaw model all muscle end-point markers are not attached to additional
    * force effectors.
    */
   public Wrench getJawMuscleWrench() {
      jawMuscleWrench.setZero();
      RigidBody body = myRigidBodies.get("jaw");

      myTmpPos.transform(body.getPose(), body.getCenterOfMass());
      XBodyToCom.R.setIdentity();
      XBodyToCom.p.negate(myTmpPos); // XWorldToCom
      XBodyToCom.mul(body.getPose()); // XBodyToCom = XWorldToCom * XBodyToWorld

      for (FrameMarker m : myFrameMarkers) {
	 if (m.getFrame() == body) {
	    boolean muscleAttachedToMarker = false;
	    for (int i = 0; i < myAxialSprings.size(); i++) {
	       AxialSpring s = myAxialSprings.get(i);
	       if (s instanceof Muscle) {
		  if (s.getFirstPoint() == m || s.getSecondPoint() == m) {
		     muscleAttachedToMarker = true;
		     break;
		  }
	       }
	    }
	    if (muscleAttachedToMarker) {
	       // get wrench in coordinate frame at COM point (aligned with
	       // world-frame)
	       myTmpPos.transform(XBodyToCom, m.getLocation());
	       jawMuscleWrench.f.add(m.getForce(), jawMuscleWrench.f);
	       jawMuscleWrench.m.crossAdd(myTmpPos, m.getForce(),
		     jawMuscleWrench.m);

	       // System.out.println(m.getName () + " -- f = " + m.getForce ());
	    }
	 }
      }
      return jawMuscleWrench;
   }

   public Vector3d getJawMuscleForce() {
      return getJawMuscleWrench().f;
   }

   public Vector3d getJawMuscleMoment() {
      return getJawMuscleWrench().m;
   }

   public Point3d getJawComPosition() {
      RigidBody jaw = myRigidBodies.get("jaw");
      jawComPosition.transform(jaw.getPose(), jaw.getCenterOfMass());
      return jawComPosition;
   }

   /*
    * Traceable interface
    */

   public String[] getTraceables() {
      return new String[] { "jawMuscleForce", "jawMuscleMoment"};
   }
   
   public String getTraceablePositionProperty (String traceableName) {
      return "+jawComPosition";
   }

   public TracingProbe getTracingProbe(String traceableName) {
      VectorTracingProbe vecProbe = null;
      Property jawCom = getProperty("jawComPosition");
      if (traceableName.equals("jawMuscleForce")) {
	  vecProbe = new VectorTracingProbe(this, getProperty("jawMuscleForce"),
	     jawCom, 1.0);
      } else if (traceableName.equals("jawMuscleMoment")) {
         vecProbe =  new VectorTracingProbe(this, getProperty("jawMuscleMoment"),
            jawCom, 1.0);
      }
      
      if (vecProbe !=  null) {
         vecProbe.setRenderAsPush (false);
         return vecProbe;
      }
      else {
         throw new IllegalArgumentException(
            "Unknown traceable '" + traceableName + "'");
      }
   }

   public void resetInertia(RigidBody body) {
      // System.out.println( b.getName () + " old inertia:");
      // System.out.println( b.getCenterOfMass ().toString ("%8.2f") + "\n");
      // System.out.println( b.getRotationalInertia ().toString ("%8.2f"));

      if (body.getMesh() == null) return;

      double density = body.getMass() / body.getMesh().computeVolume();
      body.setInertiaFromDensity (density);

      // System.out.println( b.getName () + " new inertia:");
      // System.out.println( b.getCenterOfMass ().toString ("%8.2f") + "\n");
      // System.out.println( b.getRotationalInertia ().toString ("%8.2f"));

   }

   public RigidTransform3d centerAtCOM(MechModel mech, String bodyName) {
      RigidBody body = mech.rigidBodies().get(bodyName);
      body.setAxisLength(100);

      RigidTransform3d XComToBody = new RigidTransform3d();
      XComToBody.p.set(body.getCenterOfMass());

      RigidTransform3d XBodyToWorld = new RigidTransform3d();
      body.getPose(XBodyToWorld);

      RigidTransform3d XComToWorld = new RigidTransform3d();
      XComToWorld.mul(XBodyToWorld, XComToBody);
      body.setPose(XComToWorld);

      RigidTransform3d XMeshToCom = new RigidTransform3d();
      if (body.getMesh() != null) {
	 PolygonalMesh mesh = body.getMesh();
	 // XMeshToCom.mulInverseRight (mesh.getMeshToWorld (), XComToWorld);
	 XMeshToCom.invert(XComToWorld);
	 mesh.transform(XMeshToCom);
	 body.setMesh(mesh, null);
      }

      for (FrameMarker mrk : mech.frameMarkers()) {
	 if (mrk.getFrame() == body) {
	    // System.out.println("transforming " + mrk.getName ());
	    Point3d loc = new Point3d();
	    mrk.getLocation(loc);
	    loc.transform(XMeshToCom);
	    mrk.setLocation(loc);
	 }
      }

      for (BodyConnector con : mech.bodyConnectors()) {
	 if (con.getBodyA() == body) {
	    System.out.println("jaw con - " + con.getName());
	    con.transformGeometry(XComToWorld);
	 }
      }

      return XMeshToCom;
   }

   public void setGlobalConRot(AxisAngle a) {
      globalConRot.set(a);
      globalConX.R.setAxisAngle(a);
   }

   public void initCons() {
      constrainedBody = myRigidBodies.get("jaw");
      createConstraintOrder();

      // set initial marker position as origin for tmj planes
      for (JawPlanes jp : conOrder) {
	 PlanarConnector pc;
	 conPt.add(myFrameMarkers.get(jp.getContactName()));
	 conPose.add(new RigidTransform3d());
	 // try to find existing constraints
	 BodyConnector rbc = bodyConnectors().get(jp.name());
	 if (rbc != null && rbc instanceof PlanarConnector) {
	    pc = (PlanarConnector) rbc;
	    System.out.println("initCons - found pc = " + rbc.getName());
	 } else { // didn't find it, so create a new one
	    if (jp.isUnilateral()) {
	       pc = new PlanarConnector();
	       pc.setUnilateral(true);
	    } else {
	       pc = new PlanarConnector();
	    }
	    pc.setPlaneSize(jp.getPlaneSize());
	    pc.setName(jp.name());
	 }
	 con.add(pc);

      }

      updateCons();

      for (PlanarConnector pc : con) {
	 RenderProps props = new RenderProps(myRenderProps);
	 props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);
	 props.setAlpha(0.8);
	 pc.setRenderProps(props);
	 if (pc.getName().endsWith("BITE")) {
	    // pc.setRenderNormalReversed(true);
	    // pc.setPlaneSize(AmiraJaw.leftBiteLocation.x*2.0);
	 }
      }

   }

   public void addCricothyroidJoint() {
      RigidBody thyroid = rigidBodies().get("thyroid");
      RigidBody cricoid = rigidBodies().get("cricoid");
      if (thyroid == null || cricoid == null) {
	 System.err.println("Unable to add cricothyroid joint.");
	 return;
      }

      RevoluteJoint ctJoint = new RevoluteJoint();
      ctJoint.setName("cricothyroid");

      Point3d pmin = new Point3d();
      Point3d pmax = new Point3d();
      thyroid.updateBounds(pmin, pmax);
      ctJoint.setAxisLength(1.0 * (pmax.x - pmin.x));

      if (ctJoint != null) {
	 //RenderProps.setLineSlices(ctJoint, 12);
	 RenderProps.setLineColor(ctJoint, Color.ORANGE);
	 RenderProps.setLineRadius(ctJoint, 0.6);
      }

      // connector transforms for both bodies are the same
      // as they are coincident at simulation start
      RigidTransform3d TCA = new RigidTransform3d();
      TCA.p.set(cricothryroidArticulation);
      TCA.R.setAxisAngle(0, 1, 0, Math.PI / 2);
      // ctJoint.set(cricoid, X, thyroid, X);

      RigidTransform3d TCW = new RigidTransform3d();
      // System.out.println("cricoid pose = \n" +
      // cricoid.getPose().toString("%8.2f"));
      // System.out.println("TCA     pose = \n" + TCA.toString("%8.2f"));

      ctJoint.setBodies(cricoid, TCA, thyroid, TCA);
      addBodyConnector(ctJoint);

   }

   public void updateCons() {
      updateCon(JawPlanes.LTMJ);
      updateCon(JawPlanes.RTMJ);
      updateCon(JawPlanes.LBITE);
      updateCon(JawPlanes.RBITE);
      updateCon(JawPlanes.LMED);
      updateCon(JawPlanes.RMED);
      updateCon(JawPlanes.LPOST);
      updateCon(JawPlanes.RPOST);
      updateCon(JawPlanes.LLTRL);
      updateCon(JawPlanes.RLTRL);
      updateCurvCons();
   }

   public void updateCon(JawPlanes plane) {
      RigidTransform3d Xpw;
      if (!conOrder.contains(plane)) return;
      switch (plane) {
      case LTMJ:
      case RTMJ: {
	 Xpw = updateTmjPose(plane);
	 break;
      }
      case LBITE:
      case RBITE: {
	 Xpw = updateBitePose(plane);
	 break;
      }
      case LMED:
      case RMED: {
	 Xpw = updateMedWallPose(plane);
	 break;
      }
      case LLTRL:
      case RLTRL: {
	 Xpw = updateLtrlWallPose(plane);
	 break;
      }
      case LPOST:
      case RPOST: {
	 Xpw = updatePostWallPose(plane);
	 break;
      }
      default: {
	 System.err.println("JawModel.updateCon: " + "unknown JawPlane type: "
	       + plane.name());
	 return;
      }
      }
      Vector3d Pca = conPt.get(conOrder.indexOf(plane)).getLocation();
      con.get(conOrder.indexOf(plane)).set(constrainedBody, Pca, Xpw);
   }

   private RigidTransform3d updateTmjPose(JawPlanes plane) {
      RigidTransform3d Xpw = conPose.get(conOrder.indexOf(plane));
      Xpw.R.setAxisAngle(globalConRot);
      Xpw.R.mulAxisAngle(1, 0, 0, Math.toRadians(180 + condylarAngle[conOrder
	    .indexOf(plane)]));
      Xpw.R.mulAxisAngle(0, 1, 0, Math.toRadians(condylarCant[conOrder
	    .indexOf(plane)]));
      // Xpw.R.mulAxisAngle(0,0,1,
      // Math.toRadians(medWallAngle[conOrder.indexOf(plane)]));
      Xpw.p.set(conPt.get(conOrder.indexOf(plane)).getPosition());
      return Xpw;
   }

   private RigidTransform3d updateMedWallPose(JawPlanes plane) {
      RigidTransform3d Xpw;
      Point3d medWallOrigin;
      if (plane.equals(JawPlanes.LMED)) {
	 Xpw = new RigidTransform3d(conPose.get(conOrder
	       .indexOf(JawPlanes.LTMJ)));
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.toRadians(medWallAngle[LEFT]));
	 // additional y-axis +90 degree flip for left medial wall
	 Xpw.R.mulAxisAngle(0, 1, 0, Math.toRadians(90.0));
	 medWallOrigin = new Point3d(conPt.get(conOrder.indexOf(plane))
	       .getPosition());
	 medWallOrigin.x -= medWallOffset[LEFT];
      } else if (plane.equals(JawPlanes.RMED)) {
	 Xpw = new RigidTransform3d(conPose.get(conOrder
	       .indexOf(JawPlanes.RTMJ)));
	 // additional y-axis -90 degree flip for right medial wall
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.toRadians(medWallAngle[RIGHT]));
	 Xpw.R.mulAxisAngle(0, 1, 0, Math.toRadians(-90.0));
	 medWallOrigin = new Point3d(conPt.get(conOrder.indexOf(plane))
	       .getPosition());
	 medWallOrigin.x += medWallOffset[RIGHT];
      } else {
	 System.err.println("updateMedWallPose called with bad arg:"
	       + plane.name());
	 return new RigidTransform3d();
      }

      Xpw.p.set(medWallOrigin);
      return Xpw;
   }

   private RigidTransform3d updateLtrlWallPose(JawPlanes plane) {
      RigidTransform3d Xpw;
      Point3d ltrlWallOrigin;
      if (plane.equals(JawPlanes.LLTRL)) {
	 Xpw = new RigidTransform3d(conPose.get(conOrder
	       .indexOf(JawPlanes.LTMJ)));
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.toRadians(ltrlWallAngle[LEFT]));
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.PI);
	 // additional y-axis +90 degree flip for left lateral wall
	 Xpw.R.mulAxisAngle(0, 1, 0, Math.toRadians(90.0));
	 ltrlWallOrigin = new Point3d(conPt.get(conOrder.indexOf(plane))
	       .getPosition());
	 Point3d offset = new Point3d();
	 offset.x -= ltrlWallOffset[LEFT];
	 offset.transform(globalConX);
	 ltrlWallOrigin.add(offset);
      } else if (plane.equals(JawPlanes.RLTRL)) {
	 Xpw = new RigidTransform3d(conPose.get(conOrder
	       .indexOf(JawPlanes.RTMJ)));
	 // additional y-axis -90 degree flip for right lateral wall
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.toRadians(ltrlWallAngle[RIGHT]));
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.PI);
	 Xpw.R.mulAxisAngle(0, 1, 0, Math.toRadians(-90.0));
	 ltrlWallOrigin = new Point3d(conPt.get(conOrder.indexOf(plane))
	       .getPosition());
	 Point3d offset = new Point3d();
	 offset.x += ltrlWallOffset[RIGHT];
	 offset.transform(globalConX);
	 ltrlWallOrigin.add(offset);
      } else {
	 System.err.println("updateLtrlWallPose called with bad arg:"
	       + plane.name());
	 return new RigidTransform3d();
      }

      Xpw.p.set(ltrlWallOrigin);
      return Xpw;
   }

   private RigidTransform3d updatePostWallPose(JawPlanes plane) {
      Point3d postWallOrigin;
      RigidTransform3d Xpw;
      if (plane.equals(JawPlanes.LPOST)) {
	 Xpw = new RigidTransform3d(conPose.get(conOrder
	       .indexOf(JawPlanes.LTMJ)));
	 Xpw.R.mulAxisAngle(1, 0, 0, Math.PI);
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.toRadians(postWallAngle[LEFT]));
	 // additional x-axis +90 degree flip for post wall
	 Xpw.R.mulAxisAngle(1, 0, 0, Math.toRadians(90.0));
	 postWallOrigin = new Point3d(conPt.get(conOrder.indexOf(plane))
	       .getPosition());
	 postWallOrigin.y -= postWallOffset[LEFT];
      } else if (plane.equals(JawPlanes.RPOST)) {
	 Xpw = new RigidTransform3d(conPose.get(conOrder
	       .indexOf(JawPlanes.RTMJ)));
	 Xpw.R.mulAxisAngle(1, 0, 0, Math.PI);
	 Xpw.R.mulAxisAngle(0, 0, 1, Math.toRadians(postWallAngle[RIGHT]));
	 // additional x-axis +90 degree flip for post wall
	 Xpw.R.mulAxisAngle(1, 0, 0, Math.toRadians(90.0));
	 postWallOrigin = new Point3d(conPt.get(conOrder.indexOf(plane))
	       .getPosition());
	 postWallOrigin.y -= postWallOffset[RIGHT];
      } else {
	 System.err.println("updateBackWallPose called with bad arg:"
	       + plane.name());
	 return new RigidTransform3d();
      }
      Xpw.p.set(postWallOrigin);
      return Xpw;
   }

   private RigidTransform3d updateBitePose(JawPlanes plane) {
      RigidTransform3d Xpw = conPose.get(conOrder.indexOf(plane));
      if (plane.equals(JawPlanes.LBITE)) {
	 // add 180 degrees to biteAngle to that unilateral constraint points
	 // downward
	 Xpw.R.setAxisAngle(globalConRot);
	 Xpw.R.mulAxisAngle(1, 0, 0, Math.toRadians(180 + biteAngle[LEFT]));
	 Xpw.R.mulAxisAngle(0, 1, 0, Math.toRadians(biteCant[LEFT]));
      } else if (plane.equals(JawPlanes.RBITE)) {
	 Xpw.R.setAxisAngle(globalConRot);
	 Xpw.R.mulAxisAngle(1, 0, 0, Math.toRadians(180 + biteAngle[RIGHT]));
	 Xpw.R.mulAxisAngle(0, 1, 0, Math.toRadians(biteCant[RIGHT]));
      } else {
	 System.err.println("updateBitePose called with bad arg:"
	       + plane.name());
	 return new RigidTransform3d();
      }
      Xpw.p.set(conPt.get(conOrder.indexOf(plane)).getPosition());
      return Xpw;
   }

   public double getLCondylarAngle() {
      return condylarAngle[LEFT];
   }
   
   public void setLateralWallOffset (double offset) {
      if (ltrlWallOffset[0] != offset && ltrlWallOffset[1] != offset) {
         ltrlWallOffset[0] = offset;
         ltrlWallOffset[1] = offset;
         updateCon (JawPlanes.LLTRL);
         updateCon (JawPlanes.RLTRL);
      }
   }

   public void setMedialWallOffset (double offset) {
      if (medWallOffset[0] != offset && medWallOffset[1] != offset) {
         medWallOffset[0] = offset;
         medWallOffset[1] = offset;
         updateCon (JawPlanes.LMED);
         updateCon (JawPlanes.RMED);
      }
   }
   
   public void setLCondylarAngle(double angle) {
      if (condylarAngle[LEFT] != angle) {
         condylarAngle[LEFT] = angle;
         updateCon(JawPlanes.LTMJ);
         updateCon(JawPlanes.LMED);
         updateCon(JawPlanes.LLTRL);
         updateCon(JawPlanes.LPOST);
      }
   }

   public double getLCondylarCant() {
      return condylarCant[LEFT];
   }

   public void setLCondylarCant(double angle) {
      if (condylarCant[LEFT] != angle) {
         condylarCant[LEFT] = angle;
         updateCon(JawPlanes.LTMJ);
         updateCon(JawPlanes.LMED);
         updateCon(JawPlanes.LLTRL);
         updateCon(JawPlanes.LPOST);
      }
   }

   public double getRCondylarAngle() {
      return condylarAngle[RIGHT];
   }

   public void setRCondylarAngle(double angle) {
      if (condylarAngle[RIGHT] != angle) {
         condylarAngle[RIGHT] = angle;
         updateCon(JawPlanes.RTMJ);
         updateCon(JawPlanes.RMED);
         updateCon(JawPlanes.RPOST);
      }
   }

   public double getRCondylarCant() {
      return condylarCant[RIGHT];
   }

   public void setRCondylarCant(double angle) {
      if (condylarCant[RIGHT] != angle) {
         condylarCant[RIGHT] = angle;
         updateCon(JawPlanes.RTMJ);
         updateCon(JawPlanes.RMED);
         updateCon(JawPlanes.RPOST);
      }
   }

   public double getLMedWallAngle() {
      return medWallAngle[LEFT];
   }

   public void setLMedWallAngle(double angle) {
      if (medWallAngle[LEFT] != angle) {
         medWallAngle[LEFT] = angle;
         updateCon(JawPlanes.LTMJ);
         updateCon(JawPlanes.LMED);
      }
   }

   public double getRMedWallAngle() {
      return medWallAngle[RIGHT];
   }

   public void setRMedWallAngle(double angle) {
      if (medWallAngle[RIGHT] != angle) {
         medWallAngle[RIGHT] = angle;
         updateCon(JawPlanes.RTMJ);
         updateCon(JawPlanes.RMED);
      }
   }

   public double getLPostWallAngle() {
      return postWallAngle[LEFT];
   }

   public void setLPostWallAngle(double angle) {
      if (postWallAngle[LEFT] != angle) {
         postWallAngle[LEFT] = angle;
         updateCon(JawPlanes.LTMJ);
         updateCon(JawPlanes.LPOST);
      }
   }

   public double getRPostWallAngle() {
      return postWallAngle[RIGHT];
   }

   public void setRPostWallAngle(double angle) {
      if (postWallAngle[RIGHT] != angle) {
         postWallAngle[RIGHT] = angle;
         updateCon(JawPlanes.RTMJ);
         updateCon(JawPlanes.RPOST);
      }
   }

   public double getLLtrlWallAngle() {
      return ltrlWallAngle[LEFT];
   }

   public void setLLtrlWallAngle(double angle) {
      if (ltrlWallAngle[LEFT] != angle) {
         ltrlWallAngle[LEFT] = angle;
         updateCon(JawPlanes.LTMJ);
         updateCon(JawPlanes.LLTRL);
      }
   }

   public double getRLtrlWallAngle() {
      return ltrlWallAngle[RIGHT];
   }

   public void setRLtrlWallAngle(double angle) {
      if (ltrlWallAngle[RIGHT] != angle) {
         ltrlWallAngle[RIGHT] = angle;
         updateCon(JawPlanes.RTMJ);
         updateCon(JawPlanes.RLTRL);
      }
   }

   // public double getOcclusalAngle()
   // {
   // return occlusalAngle;
   // }
   //
   // public void setOcclusalAngle(double angle)
   // {
   // occlusalAngle = angle;
   // updateCon(JawPlanes.LBITE);
   // updateCon(JawPlanes.RBITE);
   // }

   public double getLBiteAngle() {
      return biteAngle[LEFT];
   }

   public void setLBiteAngle(double angle) {
      if (biteAngle[LEFT] != angle) {
         biteAngle[LEFT] = angle;
         updateCon(JawPlanes.LBITE); // will zero any existing constraint distance
      }
   }

   public double getLBiteCant() {
      return biteCant[LEFT];
   }

   public void setLBiteCant(double angle) {
      if (biteCant[LEFT] != angle) {
         biteCant[LEFT] = angle;
         updateCon(JawPlanes.LBITE); // will zero any existing constraint distance
      }
   }

   public double getRBiteAngle() {
      return biteAngle[RIGHT];
   }

   public void setRBiteAngle(double angle) {
      if (biteAngle[RIGHT] != angle) {
         biteAngle[RIGHT] = angle;
         updateCon(JawPlanes.RBITE); // will zero any existing constraint distance
      }
   }

   public double getRBiteCant() {
      return biteCant[RIGHT];
   }

   public void setRBiteCant(double angle) {
      if (biteCant[RIGHT] != angle) {
         biteCant[RIGHT] = angle;
         updateCon(JawPlanes.RBITE); // will zero any existing constraint distance
      }
   }

   public boolean isPlanesVisible() {
      return planesVisible;
   }

   public void setPlanesVisible(boolean visible) {
      for (PlanarConnector pc : con) {
	 RenderProps.setVisible(pc, visible);
      }
      planesVisible = visible;
   }

   public double getTmjForceNorm() {
      updateConstraintForceNorms();
      // System.out.println("tmj activation = " +
      // tmjForceNorms.toString("%8.2f"));
      return tmjForceNorms.get(0);
   }

   public VectorNd getConstraintForceNorms() {
      updateConstraintForceNorms();
      // System.out.println("constraint activations (N) = " +
      // tmjForceNorms.toString("%8.2f"));
      return tmjForceNorms;
   }

   public void updateConstraintForceNorms() {
      for (int i = 0; i < con.size(); i++) {
	 // planar constrain has only one constraint get activation for
	 // constraint 0
	 // Note - convert activation from force units to Newtons
	 tmjForceNorms.set(i, con.get(i).getActivation(0)
	       / NEWTONS_TO_FORCEUNITS);
      }

      if (useCurvJoint) {
	 // manually get constraint forces for new curvilinear tmjs
	 tmjForceNorms.set(0, bodyConnectors().get("LTMJ")
	       .getActivation(0)
	       / NEWTONS_TO_FORCEUNITS);
	 tmjForceNorms.set(1, bodyConnectors().get("RTMJ")
	       .getActivation(0)
	       / NEWTONS_TO_FORCEUNITS);
      }
   }

   public void addCurvilinearTmjs() {
      RigidBody jaw = myRigidBodies.get("jaw");
      if (jaw == null) return;
      addCurvConstraint("LTMJ", "ltmj", jaw);
      addCurvConstraint("RTMJ", "rtmj", jaw);

   }

   public void addCurvConstraint(String name, String contactName, RigidBody jaw) {
      SegmentedPlanarConnector tmj = new SegmentedPlanarConnector();
      tmj.setName(name);
      tmj.setPlaneSize(40);
      tmj.setUnilateral(true);
      updateCurvCon(tmj, myFrameMarkers.get(contactName), jaw);
      addBodyConnector(tmj);
   }

   public void updateCurvCon(SegmentedPlanarConnector tmj,
	 FrameMarker contactPoint, RigidBody jaw) {
      RigidTransform3d XTmjToWorld = new RigidTransform3d();
      XTmjToWorld.p.set(contactPoint.getPosition()); // in world-coordinates
      // XTmjToWorld.p.transform(jaw.getPose());
      XTmjToWorld.R.mulAxisAngle(globalConRot);
      XTmjToWorld.R.mulAxisAngle(0, 0, 1, Math.PI / 2); // rotate x-z to y-z

      tmj.set(jaw, contactPoint.getLocation(), // in body-coordinates
	    XTmjToWorld, JawModel.getCurvPts(curvParams, numSegments));
   }

   public void updateCurvCon(String name, String contactName, RigidBody jaw) {
      BodyConnector con = myConnectors.get(name);
      if (con instanceof SegmentedPlanarConnector) {
	 SegmentedPlanarConnector tmj = (SegmentedPlanarConnector) con;
	 FrameMarker contactPoint = myFrameMarkers.get(contactName);
	 if (tmj == null || contactPoint == null || jaw == null) return;
	 updateCurvCon(tmj, contactPoint, jaw);
      }
   }

   public void updateCurvCons() {
      RigidBody jaw = myRigidBodies.get("jaw");
      updateCurvCon("LTMJ", "ltmj", jaw);
      updateCurvCon("RTMJ", "rtmj", jaw);
   }

   /*
    * params = [ x0, y0, xf, yf, initial slope, final slope ]
    */
   public static double[] getCurvPts(double[] params, int numSegments) {
      // specify cubic by (xf, yf, initial slope, final slope), x0=0, y0=0
      double[] a = JawModel.getCubicParams(params[2], params[3], params[4],
	    params[5]);
      double[] pts = new double[(numSegments + 1) * 2];
      double x, y;
      double deltax = params[2] - params[0]; // xf-x0
      double step = deltax / (double) numSegments;
      int count = 0;
      for (int i = 0; i < numSegments + 1; i++) {
	 x = params[0] + step * i;
	 y = params[1] + a[3] * Math.pow(x, 3) + a[2] * Math.pow(x, 2) + a[1]
	       * x + a[0];
	 pts[count++] = -x;
	 pts[count++] = y;
	 // System.out.printf("TMJ pt %d: %8.2f, %8.2f \n", i, x, pts[count-1]);
      }
      return pts;
   }

   public static double[] getCubicParams(double Xf, double Yf,
	 double slopeInit, double slopeFinal) {
      VectorNd a = new VectorNd(4);
      MatrixNd M = new MatrixNd(4, 4);
      M.set(new double[] { 0, 0, 0, 1, Math.pow(Xf, 3), Math.pow(Xf, 2), Xf, 1,
	    0, 0, 1, 0, 3.0 * Math.pow(Xf, 2), 2.0 * Xf, 1, 0 });
      // System.out.println("M = \n" + M.toString("%8.4f"));
      VectorNd y = new VectorNd(new double[] { 0.0, Yf,
	    Math.toRadians(slopeInit), Math.toRadians(slopeFinal) });
      if (M.invert() != false) M.mul(a, y);

      // System.out.println("a vector (a0, a1, a2, a3) = " +
      // a.toString("%8.4f"));
      // reverse order of a so that rval[0]=a0, rval[1]=a1, ...
      return new double[] { a.get(3), a.get(2), a.get(1), a.get(0) };
   }

}
