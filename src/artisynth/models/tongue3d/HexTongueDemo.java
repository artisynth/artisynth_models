package artisynth.models.tongue3d;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

import javax.swing.JSeparator;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.AxisAngle;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyList;
import maspack.render.LineRenderProps;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;
import maspack.util.ReaderTokenizer;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.LabeledControl;
import maspack.widgets.PropertyWidget;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemMuscleStiffener;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.gui.editorManager.RemoveComponentsCommand;
import artisynth.core.materials.InactiveMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.ParticlePlaneConstraint;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponentBase;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.MDLMeshIO;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.jawTongue.BadinDataDemo;
import artisynth.models.jawTongue.BadinJawHyoid;
import artisynth.models.jawTongue.BadinJawHyoidTongue;

public class HexTongueDemo extends RootModel {

   public String getAbout() {
      return "A 3D FEM tongue model for studying speech biomechanics.\n\n"
         +
         "The model was developed by Ian Stavness based on a reference model by Gipsa-Lab (Grenoble, France), please cite: \n"
         +

         "Ian Stavness, John E Lloyd, Yohan Payan, and Sidney Fels. "
         +
         "Coupled Hard-Soft Tissue Simulation with Contact and Constraints Applied to Jaw-Tongue-Hyoid Dynamics. "
         +
         "International Journal for Numerical Methods in Biomedical Engineering, 27:367-390, 2011."
         +
         "\n\n and \n\n"
         +
         "S. Buchaillard, P. Perrier, and Y. Payan. "
         +
         "A biomechanical model of cardinal vowel production: Muscle activations and the impact of gravity on tongue positioning. "
         +
         "Journal of the Acoustical Society of America, 126(4):2033-2051, 2009.";
   }

   protected static boolean addBilateralExciters = true;
   boolean leftSideOnly = false;
   boolean linearMaterial = false;
   boolean showJawMaxilla = false;
   boolean useIcpMuscleDefs = true;
   boolean useIncompConstraint = false;
   public static boolean DEFAULT_ACTIVATION_STIFFENING = true;
   boolean doActivationStiffening = DEFAULT_ACTIVATION_STIFFENING;
   static boolean scaleByFascicle = true;
   public static double midsagittalTol = 1e-4;
   public static double defaultExcitationProbeMagnitude = 0.3;

   public static final String geometrypath = ArtisynthPath.getSrcRelativePath(
      HexTongueDemo.class, "geometry/");

   protected FemMuscleModel tongue;
   protected MechModel mech;
   protected FemMuscleStiffener stiffener = null;

   public static final NodeName[] tongueNodes = new NodeName[] {
      new NodeName("tip", 919), new NodeName("top", 869),
      new NodeName("base", 927) };

   public static final double mm2m = 1d / 1000d;

   public static PropertyList myProps =
      new PropertyList (HexTongueDemo.class, RootModel.class);

   static {
      myProps.add (
         "activationStiffening",
         "apply activation stiffening to the FEM", 
         DEFAULT_ACTIVATION_STIFFENING);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public boolean getActivationStiffening() {
      return doActivationStiffening;
   }

   public void setActivationStiffening (boolean enable) {
      doActivationStiffening = enable;
   }

   /*
    * attachment and sty nodes from old HexTongueDemo.java, not used in
    * Jaw-Tongue-Hyoid model XXX - should be removed
    */

   public static final Integer[] JawAttachmentNodeIdxs = new Integer[] { 362,
      363, 364, 365, 366, 367, 368, 370, 371, 394, 395, 397, 399, 401, 403,
      405, 775, 776, 777, 778, 779, 780, 781, 783, 784, 807, 808, 810, 812,
      814, 816, 818, 930, 931, 932, 933, 934, 935, 936, 17, 22, 44, 45, 53,
      57, 87, 90, 280, 283, 369, 372, 384, 387, 430, 435, 457, 458, 466,
      470, 500, 503, 693, 696, 782, 785, 797, 800 };

   public static final Integer[] HyoidAttachmentNodeIdxs = new Integer[] { 6,
      14, 21, 29, 39, 43, 174, 180, 181, 182, 419, 427, 434, 442, 452, 456,
      587, 593, 594, 595, 8, 10, 23, 111, 113, 169, 170, 421, 423, 436, 524,
      526, 582, 583, 25, 438 };

   public static final Point3d[] styNodePositions = new Point3d[] {
      new Point3d(0.144246, 0.041761, 0.130712),
      new Point3d(0.144246, -0.041761, 0.130712) };

   public static final Point3d[] styNodeLowerPositions = new Point3d[] {
      new Point3d(0.14984029, 0.035891287, 0.125),
      new Point3d(0.14984029, -0.035891287, 0.125) };

   /*
    * end attachment and sty nodes
    */

   public static class NodeName {
      public String nodeName;

      public Integer nodeIdx;

      public NodeName (String name, Integer idx) {
         nodeName = name;
         nodeIdx = idx;
      }
   }

   @Override
   public StepAdjustment advance(double t0, double t1, int flags) {
      if (stiffener != null && doActivationStiffening) {
         stiffener.updateElemStiffnesses();
      }
      return super.advance(t0, t1, flags);
   }

   public HexTongueDemo () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      mech = new MechModel("mech");
      mech.setIntegrator(Integrator.ConstrainedBackwardEuler);
      mech.setMaxStepSize(0.01);
      addModel(mech);

      tongue = createTongue();
      // addModel(tongue);
      mech.addModel(tongue);

      stiffener = new FemMuscleStiffener(tongue);

      for (FemElement elem : tongue.getElements()) {
         if (elem.getRestVolume() < 0) {
            System.out.println("elem " + elem.myNumber + " degenerate");
         }
      }

      if (leftSideOnly) {
         setupSymmetricSimulation(mech, tongue, new Plane(Vector3d.Y_UNIT, 0),
            midsagittalTol);
      }

      addMuscleExciterProbes (this, tongue);
      setActivationColor(tongue);

      RenderProps.setVisible (tongue.getMuscleBundles (), true);
      // tongue.loadFibreEx ("r/i.excitations");
      
//      splitTongueAlongMidline();
   }
   
   public void splitTongueAlongMidline()  {
     
      ArrayList<FemNode3d> midlineNodes = new ArrayList<FemNode3d> ();
      for (FemNode3d n : tongue.getNodes ()) {
         if (Math.abs (n.getPosition ().y) < 1e-6) {
            midlineNodes.add (n);
            RenderProps.setPointColor (n, Color.magenta);
         }
      }
      int idx = -1;
      Vector3d centroid = new Vector3d ();
      for (FemNode3d n : midlineNodes) {
         FemNode3d newNode = new FemNode3d (n.getPosition ());
//         double newMass = n.getMass ()/2d;
//         n.setMass(newMass);
//         newNode.setMass (newMass);
         newNode.setMass(n.getMass ());
         tongue.addNode (newNode);
         for (FemElement3d e : n.getElementDependencies ()) {
            e.computeCentroid (centroid);
            if (centroid.y < 0) {
               for (int i = 0; i < e.getNodes ().length; i++) {
                  if (e.getNodes ()[i] == n) {
                     idx = i;
                     break;
                  }
               }
               e.getNodes ()[idx] = newNode;
            }
         }
      }

   }

   /*
    * create tongue method is used in TetTongue sub-class
    */
   public FemMuscleModel createTongue() {
      return tongue = createHexTongue(linearMaterial, useIcpMuscleDefs);
   }

   public static void createHexTongue(FemMuscleModel tongue,
      boolean linearMaterial, boolean useIcpMuscles) {
      readFromAnsysReader(tongue, "tongue");
      setupTongue(tongue, linearMaterial, useIcpMuscles, true /* scaleMuscleForceByVolume */);
   }

   public static FemMuscleModel createHexTongue(boolean linearMaterial,
      boolean useIcpMuscles) {
      FemMuscleModel tongue = new FemMuscleModel("tongue");
      tongue.setMuscleMaterial(new InactiveMuscle());
      createHexTongue(tongue, linearMaterial, useIcpMuscles);
      return tongue;
   }

   public static void setupTongue(FemMuscleModel tongue,
      boolean linearMaterial, boolean useIcpMuscles,
      boolean scaleMuscleForceByVolume) {
      addStyNodes(tongue);
      setAttachmentNodes(tongue);
      if (linearMaterial)
         tongue.setMaterial(new LinearMaterial());
      else
         tongue.setMaterial(new MooneyRivlinMaterial());

      if (useIcpMuscles) {
         loadBundlesFromFile(tongue);
         loadMusclesFromFile(tongue);
         createMuscleExciters(tongue);
      } else {
         // loadComponentsFromFile(tongue, geometrypath + "tongueJapandemo.art");
      }

      HexTongueDemo.setTongueProperties(tongue);
      setMuscleProps(tongue, scaleMuscleForceByVolume);
      setupRenderProps(tongue);
      setNodeNames(tongue);
   }

   public static void setTongueProperties(FemModel3d tongue) {
//      tongue.setGravity(0, 0, -9.8); // gravity should be inherited from mech model
      tongue.setDensity(1000);
      tongue.setParticleDamping(1.22);
      tongue.setStiffnessDamping(0.05);

      if (tongue.getMaterial() instanceof LinearMaterial) {
         //tongue.setWarping(true);
         tongue.setLinearMaterial(6912, 0.49, true);
      } else if (tongue.getMaterial() instanceof MooneyRivlinMaterial) {
         MooneyRivlinMaterial mrmat = (MooneyRivlinMaterial)tongue
            .getMaterial();
         // [Gerard 2005] measurement on fresh cadavar tongue
         // mrmat.setC10 (192); // Pa
         // mrmat.setC20 (90); // Pa

         // [Buchaillard 2009] scaled params to match
         // values for human muscle tissue at rest [Duck 1990]
         mrmat.setC10(1037); // Pa
         mrmat.setC20(486); // Pa

         // mrmat.setBulkModulus (100*mrmat.getC10 ()); // 100x c10 ~
         // possion=0.49
         mrmat.setBulkModulus(10 * mrmat.getC10());

         // [Buchaillard 2009]
         // Rayleigh damping C = a M + b K
         // a = 40 s^-1
         // b = 0.03 s
         tongue.setParticleDamping(40);
         tongue.setStiffnessDamping(0.03);

         // [Buchaillard 2009]
         // density = 1040 kg m^-3
         tongue.setDensity(1040);
      }

      tongue.setIncompressible(IncompMethod.AUTO);
      tongue.setIntegrator(Integrator.ConstrainedBackwardEuler);
      tongue.setMaxStepSize(0.01);
      tongue.setImplicitIterations(10);
      tongue.setImplicitPrecision(0.001);

      if (tongue instanceof FemMuscleModel) {
         double maxforce = 2;
         for (MuscleBundle b : ((FemMuscleModel)tongue).getMuscleBundles()) {
            b.setMaxForce(maxforce);
         }
      }
   }

   public static void
      loadComponentsFromFile(FemMuscleModel fem, String filename) {

      try {
         ReaderTokenizer rtok = ArtisynthIO.newReaderTokenizer(new File(
            filename));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            if (rtok.ttype == ReaderTokenizer.TT_WORD
               && rtok.sval.equals("markers")) {
               rtok.scanToken('=');
               ScanWriteUtils.scanfull (rtok, fem.markers(), fem);
            }
            if (rtok.ttype == ReaderTokenizer.TT_WORD
               && rtok.sval.equals("bundles")) {
               rtok.scanToken('=');
               ScanWriteUtils.scanfull (rtok, fem.getMuscleBundles(), fem);
            }
            if (rtok.ttype == ReaderTokenizer.TT_WORD
               && rtok.sval.equals("exciters")) {
               rtok.scanToken('=');
               ScanWriteUtils.scanfull (rtok, fem.getMuscleExciters(), fem);
               break;
            }
         }
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public static void loadBundlesFromFile(FemMuscleModel fem) {

      try {
         AnsysMuscleElemReader.read(fem, new FileReader(geometrypath
            + "CreateMuscles.mac"));
      } catch (IOException e) {
         e.printStackTrace();
      }

   }

   public static void loadMusclesFromFile(FemMuscleModel fem) {

      try {
         AnsysMuscleFiberReader.read(fem, new FileReader(geometrypath
            + "Fibers.mac"));
      } catch (IOException e) {
         e.printStackTrace();
      }

   }

   public static void createMuscleExciters(FemMuscleModel fem) {

      if (addBilateralExciters) {
         for (int i = 0; i < 20; i += 2) {
            MuscleBundle left = fem.getMuscleBundles().get(i);
            MuscleBundle right = fem.getMuscleBundles().get(i + 1);
            String[] name = left.getName().split("_");
            MuscleExciter ex = new MuscleExciter(name[0]);
            ex.addTarget(left, 1.0);
            ex.addTarget(right, 1.0);
            fem.addMuscleExciter(ex);
         }
         // add exciter for unpaired muscle group (SL)
         MuscleBundle unpaired = fem.getMuscleBundles().get(20);
         MuscleExciter ex = new MuscleExciter(unpaired.getName());
         ex.addTarget(unpaired, 1.0);
         fem.addMuscleExciter(ex);
      }
      else {

         for (MuscleBundle b : fem.getMuscleBundles()) {
            String name = b.getName();
            if (name.endsWith("_L") || name.endsWith("_R")) {
               MuscleExciter ex = new MuscleExciter(name);
               ex.addTarget(b, 1);
               fem.addMuscleExciter(ex);
            }
         }

         // explicitly add exciters for SL that have been manually separated
         // original fibers.mac file did not divide SL into bilateral groups
         FemMuscleTongueDemo.addExciterFromFile(fem,
            new File(ArtisynthPath.getSrcRelativePath(HexTongueDemo.class,
               "exciters/SL_L.art")));
         FemMuscleTongueDemo.addExciterFromFile(fem,
            new File(ArtisynthPath.getSrcRelativePath(HexTongueDemo.class,
               "exciters/SL_R.art")));
      }

   }

   public static void readFromAnsysReader(FemModel3d fem, String filename) {
      // System.out.println("reading tongue file: " + name);
      try {
         AnsysReader.read(fem, geometrypath + filename + ".node", geometrypath
            + filename + ".elem", 1, null, /* options= */0);
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public static void addStyNodes(FemModel3d fem) {
      for (Point3d p : HexTongueDemo.styNodeLowerPositions) {
         FemNode3d n = new FemNode3d(p);
         n.setDynamic(false);
         fem.addNode(n);
         System.out.println("number=" + n.getNumber());
      }
   }

   public static FemNode3d[] getHyoidAttachments(FemModel3d fem) {
      Integer[] attachedNodeIndices = readIntList(ArtisynthPath
         .getSrcRelativePath(HexTongueDemo.class,
            "geometry/hyoidAttachments.txt"));
      return getAttachments(fem, attachedNodeIndices);
   }

   public static FemNode3d[] getJawAttachments(FemModel3d fem) {
      Integer[] attachedNodeIndices = readIntList(ArtisynthPath
         .getSrcRelativePath(HexTongueDemo.class,
            "geometry/jawAttachments.txt"));
      return getAttachments(fem, attachedNodeIndices);
   }

   private static FemNode3d[] getAttachments(FemModel3d fem,
      Integer[] attachedNodeIndices) {
      ArrayList<FemNode3d> attachedNodes = new ArrayList<FemNode3d>();
      for (Integer idx : attachedNodeIndices) {
         attachedNodes.add(fem.getNode(idx));
      }
      return attachedNodes.toArray(new FemNode3d[0]);
   }

   public static void setAttachmentNodes(FemModel3d fem) {

      for (FemNode3d n : getJawAttachments(fem)) {
         n.setDynamic(false);
         RenderProps.setPointColor(n, Color.RED);
      }

      for (FemNode3d n : getHyoidAttachments(fem)) {
         n.setDynamic(false);
         RenderProps.setPointColor(n, Color.BLUE);
      }
   }

   private static Integer[] findNodeIdxs(FemModel3d fem,
      String nodePosFilename, double tol) {
      ArrayList<Integer> idxs = new ArrayList<Integer>();

      try {
         PolygonalMesh verts = MDLMeshIO.read(nodePosFilename, null);
         // ReaderTokenizer rtok = new ReaderTokenizer (new FileReader
         // (nodePosFilename));
         for (Vertex3d v : verts.getVertices()) {
            for (int i = 0; i < fem.numNodes(); i++) {
               FemNode3d n = fem.getNode(i);
               if (v.getPosition().distance(n.getPosition()) < tol) {
                  idxs.add(i);
                  break;
               }
            }
         }
         if (idxs.size() != verts.numVertices()) {
            System.err.printf(
               "error finding attachment node indices, %d points, "
                  + "but %d nodes found\n", verts.numVertices(), idxs
                  .size());
         }
      } catch (IOException e) {
         e.printStackTrace();
      }

      return idxs.toArray(new Integer[0]);
   }

   public static void setMuscleProps(FemMuscleModel tongue,
      boolean scaleMuscleForceByVolume) {
      VectorNd maxForces = getMaxForces(tongue);
      for (int i = 0; i < tongue.getMuscleBundles().size(); i++) {
         MuscleBundle b = tongue.getMuscleBundles().get(i);
         if (scaleMuscleForceByVolume) {
            setVolumeDepMaxForce(b, maxForces.get(i), /* scaleByFascicle= */
               scaleByFascicle);
         } else {
            b.setMaxForce(maxForces.get(i) / b.getFibres().size());
         }

         b.setFibresActive(true);
      }

      // using fibers, therefore inactivate fem muscle material
      tongue.setMuscleMaterial(new InactiveMuscle());
   }

   public static VectorNd getMaxForces(FemMuscleModel tongue) {
      HashMap<String,Double> musMaxForce = readMuscleMaxForce(ArtisynthPath
         .getSrcRelativePath(HexTongueDemo.class,
            "geometry/MuscleMaxForce.txt"));
      VectorNd maxForces = new VectorNd(tongue.getMuscleBundles().size());
      for (int i = 0; i < tongue.getMuscleBundles().size(); i++) {
         MuscleBundle b = tongue.getMuscleBundles().get(i);
         String[] splitname = b.getName().split("_"); // assuming bilateral
         // muscles named NAME_R or
         // NAME_L
         double maxforce = musMaxForce.get(splitname[0]);
         if (splitname.length > 1) // muscle is bilateral, divide equally b/w
            // left and right sides
            maxforce /= 2.0;
         maxForces.set(i, maxforce);
      }
      return maxForces;
   }

   private static HashMap<String,Double> readMuscleMaxForce(String fileName) {
      HashMap<String,Double> nameForceMap = new HashMap<String,Double>();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(fileName));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            rtok.pushBack();
            String name = rtok.scanWord();
            Double maxForce = rtok.scanNumber();
            nameForceMap.put(name, maxForce);
         }
      } catch (IOException e) {
         e.printStackTrace();
      }
      return nameForceMap;
   }

   public static void setNodeNames(FemModel3d tongue) {
      for (NodeName nn : HexTongueDemo.tongueNodes) {
         tongue.getNode(nn.nodeIdx).setName(nn.nodeName);
      }
   }

   public static void setupRenderProps(FemModel3d tongue) {

      tongue.invalidateSurfaceMesh();
      tongue.setSurfaceRendering(SurfaceRender.None);
      tongue.setElementWidgetSize(1.0);
      RenderProps.setFaceColor(tongue, new Color(.8f, .5f, .5f));
      RenderProps.setLineColor(tongue, new Color(.2f, .2f, .2f)); // dark grey
      RenderProps.setFaceStyle(tongue, FaceStyle.FRONT_AND_BACK);
      RenderProps.setDrawEdges(tongue, false);
      RenderProps.setVisibleMode(tongue.getNodes(), PropertyMode.Inherited);
      RenderProps.setPointRadius(tongue.getNodes(), 0.00035);
      RenderProps.setPointStyle(tongue.getNodes(), PointStyle.SPHERE);
      RenderProps.setVisibleMode(tongue.getElements(), PropertyMode.Inherited);

      if (tongue instanceof FemMuscleModel) {
         RenderableComponentList<MuscleBundle> bundles =
            ((FemMuscleModel)tongue).getMuscleBundles();
         RenderProps.setLineWidth(bundles, 2);

         // RenderProps.setLineStyle(bundles, LineStyle.CYLINDER);
         // RenderProps.setLineRadius(bundles, 0.0005);

         RenderProps.setLineStyle(bundles, LineStyle.LINE);
         RenderProps.setLineWidth(bundles, 3);

         tongue.setElementWidgetSize(1);
         tongue.setSurfaceRendering(SurfaceRender.None);
         RenderProps.setVisible(tongue.getElements(), true);
         RenderProps.setVisible(tongue.getNodes(), false);

         // make muscle bundle visibility inherited
         for (MuscleBundle b : bundles) {
            RenderProps.setVisibleMode(b, PropertyMode.Inherited);
            RenderProps.setLineColor(b, FemControlPanel.getMuscleColor(b.getNumber()));
            b.setElementWidgetSize(0);
         }
      }
   }

   public void fillHole() {
      if (tongue == null)
         return;

      int rightHole[] = { 700, 706, 701, 825, 710 };
      int leftHole[] = { 287, 293, 288, 412, 297 };
      ArrayList<TetElement> filltets = new ArrayList<TetElement>(4);

      filltets.add(new TetElement(tongue.getNode(rightHole[0]), tongue
         .getNode(rightHole[1]), tongue.getNode(rightHole[3]), tongue
         .getNode(rightHole[2])));
      filltets.add(new TetElement(tongue.getNode(rightHole[1]), tongue
         .getNode(rightHole[4]), tongue.getNode(rightHole[3]), tongue
         .getNode(rightHole[2])));
      filltets.add(new TetElement(tongue.getNode(leftHole[0]), tongue
         .getNode(leftHole[1]), tongue.getNode(leftHole[2]), tongue
         .getNode(leftHole[3])));
      filltets.add(new TetElement(tongue.getNode(leftHole[1]), tongue
         .getNode(leftHole[2]), tongue.getNode(leftHole[3]), tongue
         .getNode(leftHole[4])));

      for (TetElement tet : filltets) {
         tongue.addElement(tet);
      }
      tongue.invalidateSurfaceMesh();
   }

   public static void addControls(ControlPanel controlPanel,
      FemMuscleModel tongue, ModelComponent topModel) {
      FemControlPanel.addMuscleControls(controlPanel, tongue, topModel);

      LabeledControl checkBox, checkBox1, checkBox2, checkBox3;

      controlPanel.addWidget(tongue, "surfaceRendering");
      controlPanel.addWidget(tongue, "stressPlotRanging");
      controlPanel.addWidget(tongue, "stressPlotRange");

      // Muscle Exciters

      controlPanel.addWidget(new JSeparator());

      for (int i = 0; i < tongue.getMuscleExciters().size(); ++i) {
         MuscleExciter b = tongue.getMuscleExciters().get(i);
         LabeledComponentBase slider = controlPanel.addWidget(b
            .getProperty("excitation"), 0.0, 1.0);
         // DoubleFieldSlider slider = (DoubleFieldSlider)
         // PropertyWidget.create(
         // b, "excitation", 0, 1.0);
         String label = b.getName() != null ? b.getName() : new Integer(b
            .getNumber()).toString();
         slider.setLabelText(label);
         slider.setLabelFontColor(Color.BLACK);
         // myControlPanel.addWidget(slider);
      }
      //
      controlPanel.addWidget(new JSeparator());
      // Muscle Activations
      for (int i = 0; i < tongue.getMuscleBundles().size(); ++i) {
         MuscleBundle b = tongue.getMuscleBundles().get(i);
         LabeledComponentBase slider = controlPanel.addWidget(b
            .getProperty("excitation"), 0.0, 1.0);
         // DoubleFieldSlider slider = (DoubleFieldSlider)
         // PropertyWidget.create(
         // b, "excitation", 0, 1.0);
         String label = b.getName() != null ? b.getName() : new Integer(b
            .getNumber()).toString();
         slider.setLabelText(label);
         if (b.getRenderProps() == null)
            slider.setLabelFontColor(Color.WHITE);
         else
            slider.setLabelFontColor(b.getRenderProps().getLineColor());

         if (b.getRenderProps().getLineColor().getRed() > 170
            && b.getRenderProps().getLineColor().getGreen() > 170
            && b.getRenderProps().getLineColor().getBlue() > 170) {
            slider.setBackgroundColor(new Color(50, 50, 50));
         }
         checkBox = (LabeledControl)PropertyWidget.create(b,
            "renderProps.visible");
         checkBox.setLabelText("");
         checkBox.addValueChangeListener(new ValueChangeListener() {
            public void valueChange(ValueChangeEvent e) {
               Main.getMain().rerender();
            }
         });
         slider.add(checkBox);
         // myControlPanel.addWidget(slider);
      }

   }

   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      if (myControlPanels.size() == 0 && tongue != null) {
         FemControlPanel.createControlPanel(this, tongue, myModels.get(0));
         FemControlPanel.createMuscleExcitersPanel(this, tongue);
      }
   }

   public void loadProbes() {
      File workingDir = new File(ArtisynthPath.getSrcRelativePath(
         HexTongueDemo.class, "data/sty/"));
      if (!workingDir.exists())
         return;
      String probeFileName = "/0probes.art";
      ArtisynthPath.setWorkingDir(workingDir);
      try {
         scanProbes(
            ArtisynthIO.newReaderTokenizer(workingDir.getAbsolutePath()
               + probeFileName));
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public static void addMuscleProbes(RootModel root,
      CompositeComponent model, String[] muscleNames, String namePrefix,
      double duration) throws IOException {
      addMuscleProbes(root, model, muscleNames, namePrefix, duration, null);
   }

   public static void addMuscleProbes(RootModel root,
      CompositeComponent model, String[] muscleNames, String namePrefix,
      double duration, String attachfilename) throws IOException {
      NumericInputProbe inprobe;

      for (int i = 0; i < muscleNames.length; i++) {
         String compPath = "axialSprings/" + muscleNames[i];
         if (model.findComponent(compPath) == null) {
            compPath = "bundles/" + muscleNames[i];
            if (model.findComponent(compPath) == null) {
               // not a bundle, try excitors list
               compPath = "exciters/" + muscleNames[i];
               if (model.findComponent(compPath) == null)
                  continue;
            }
         }
         inprobe = new NumericInputProbe(model, compPath + ":excitation", null);
         String name = namePrefix + muscleNames[i];
         inprobe.setName(name);
         if (attachfilename == null) {
            inprobe.setAttachedFileName(name + ".txt");
         } else {
            inprobe.setAttachedFileName(attachfilename);
         }
         inprobe.setActive(true);
         inprobe.setStartStopTimes(0, duration);
         inprobe.loadEmpty();
         root.addInputProbe(inprobe);
      }
   }

   public static void addPositionProbes(RootModel root, FemMuscleModel tongue,
      String[] nodeNames, String namePrefix, double duration)
      throws IOException {
      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
      for (int i = 0; i < nodeNames.length; i++) {
         String compPath = "nodes/" + nodeNames[i];
         ModelComponent node = tongue.findComponent(compPath);
         if (node != null && node instanceof FemNode3d) {
            nodes.add((FemNode3d)node);
         }
      }
      addPositionProbes(root, nodes.toArray(new FemNode3d[nodes.size()]),
         namePrefix, duration);
   }

   public static void addPositionProbes(RootModel root, FemNode3d[] nodes,
      String namePrefix, double duration) throws IOException {

      NumericOutputProbe outprobe;
      for (FemNode3d n : nodes) {
         outprobe = new NumericOutputProbe(n, "position", null, 0.01);
         String name = namePrefix
            + (n.getName() != null ? n.getName() : n.getNumber());
         outprobe.setName(name);
         outprobe.setAttachedFileName("probe_" + name + ".txt");
         outprobe.setActive(true);
         outprobe.setStartStopTimes(0, duration);
         root.addOutputProbe(outprobe);
      }
   }

   public static void addOutProbes(RootModel root, FemMuscleModel tongue,
      double duration) {
      System.out.println("add tongue output probes");
      // addPositionProbeForNamedNodes (root, tongue, duration);
      addTongueContourProbePos(root, tongue, duration);
      addTongueContourProbeVel(root, tongue, duration);
   }

   public static Integer[] readIntList(String fileName) {
      ArrayList<Integer> intList = new ArrayList<Integer>();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(fileName));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            rtok.pushBack();
            intList.add(rtok.scanInteger());
         }
      } catch (IOException e) {
         e.printStackTrace();
      }
      return intList.toArray(new Integer[intList.size()]);
   }

   public static void addTongueContourProbeVel(RootModel root,
      FemMuscleModel tongue, double duration) {
      addTongueContourProbe(root, tongue, "velocity", duration);
   }

   public static void addTongueContourProbePos(RootModel root,
      FemMuscleModel tongue, double duration) {
      addTongueContourProbe(root, tongue, "position", duration);
   }

   private static void addTongueContourProbe(RootModel root,
      FemMuscleModel tongue, String prop, double duration) {
      Integer[] contourIdxs = readIntList(ArtisynthPath.getSrcRelativePath(
         HexTongueDemo.class, "geometry/tongueContourNodes.txt"));
      Property[] contourPosProps = new Property[contourIdxs.length];
      for (int i = 0; i < contourIdxs.length; i++) {
         contourPosProps[i] = tongue.getProperty("nodes/" + contourIdxs[i]
            + ":" + prop);
      }

      NumericOutputProbe outprobe = new NumericOutputProbe(contourPosProps,
         0.01);
      String name = prop + "_tongueContour";
      outprobe.setName(name);
      outprobe.setAttachedFileName("out_" + name + ".txt");
      outprobe.setActive(true);
      outprobe.setStartStopTimes(0, duration);
      root.addOutputProbe(outprobe);
   }

   public static void addPositionProbeForNamedNodes(RootModel root,
      FemMuscleModel tongue, double duration) {
      ArrayList<FemNode3d> namedNodes = new ArrayList<FemNode3d>();
      for (FemNode3d n : tongue.getNodes()) {
         if (n.getName() != null) {
            namedNodes.add(n);
         }
      }
      try {
         HexTongueDemo.addPositionProbes(root, namedNodes
            .toArray(new FemNode3d[0]), "", duration);
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public static void addInProbes(RootModel root, FemMuscleModel tongue,
      double duration) {
      ArrayList<String> exciterNames = new ArrayList<String>();
      for (MuscleExciter ex : tongue.getMuscleExciters()) {
         if (ex.getName() != null)
            exciterNames.add(ex.getName());
      }
      try {
         HexTongueDemo.addMuscleProbes(root, tongue, exciterNames
            .toArray(new String[0]), "", duration);
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public void createProbes(double duration) {
      if (tongue != null) {
         addOutProbes(this, tongue, duration);
         addInProbes(this, tongue, duration);
      }
   }

   

   /*
    * from MuscleBundle
    */

   public static void setVolumeDepMaxForce(MuscleBundle b,
      double bundleMaxForce, boolean scaleByFascicle) {

      double totalMaxForce = 0;
      for (Muscle f : b.getFibres()) {
         ArrayList<FemElement> surroundingElems = new ArrayList<FemElement>();
         double vol = getSurroundingElemsVol(b, surroundingElems, f);
         // if (surroundingElems.size () == 0) {
         // System.err.println("no surrounding elems, f"+f.getNumber()+" in "+myName+", vol = "+vol);
         // }
         Muscle.setMaxForce(f, bundleMaxForce * vol);
         totalMaxForce += Muscle.getMaxForce(f);
      }

      double S = 1.0;
      if (scaleByFascicle) {
         double fascicleMaxForce = getTotalFascicleForce(b);
         System.out.println(b.getName() + " total/fascicle = " + totalMaxForce
            / fascicleMaxForce);
         S = bundleMaxForce / fascicleMaxForce;
      } else {
         S = bundleMaxForce / totalMaxForce;
      }

      for (Muscle f : b.getFibres()) {
         Muscle.setMaxForce(f, Muscle.getMaxForce(f) * S);
      }
   }

   private static double getTotalFascicleForce(MuscleBundle b) {
      // a fascile is a serial group of muscle fibers
      // fasciles are arranged in parallel to form a muscle bundle
      // the sum of fascile max forces should equal bundleMaxForce
      double totalMaxFascicleForce = 0;
      for (int i = 0; i < b.getFascicles().size(); i++) {
         double maxMaxForce = -1;
         for (Muscle f : b.getFascicles().get(i)) {
            if (Muscle.getMaxForce(f) > maxMaxForce) {
               maxMaxForce = Muscle.getMaxForce(f);
            }
         }
         totalMaxFascicleForce += maxMaxForce;
      }
      return totalMaxFascicleForce;
   }

   /*
    * returns volume of elements added to elems
    */
   public static double getSurroundingElemsVol(MuscleBundle b,
      ArrayList<FemElement> surroundingElems, Muscle fiber) {
      HashSet<FemElement3d> elems0 = new HashSet<FemElement3d>();
      HashSet<FemElement3d> elems1 = new HashSet<FemElement3d>();
      double elemsVolume = 0.0;
      if (!(fiber.getFirstPoint() instanceof FemNode3d)
         || !(fiber.getSecondPoint() instanceof FemNode3d)) {
         System.err.println("found fiber with non-node end-point");
         return elemsVolume;
      }

      FemNode3d n0 = (FemNode3d)fiber.getFirstPoint();
      FemNode3d n1 = (FemNode3d)fiber.getSecondPoint();

      elems0.addAll(n0.getElementDependencies());
      elems1.addAll(n1.getElementDependencies());
      for (MuscleElementDesc desc : b.getElements()) {
         FemElement e = desc.getElement();
         if (elems0.contains(e) && elems1.contains(e)) {
            surroundingElems.add(e);
            e.computeVolumes();
            elemsVolume += e.getVolume();
         }
      }

      if (surroundingElems.size() == 0) {
         // fiber connected to non-adjacent elements,
         // therefore compute average volume of all elements
         int cnt = 0;
         for (FemElement e : elems0) {
            if (contains(b, e)) {
               e.computeVolumes();
               elemsVolume += e.getVolume();
               cnt++;
            }
         }
         elemsVolume /= cnt;
      }

      return elemsVolume;
   }

   public static boolean contains(MuscleBundle b, FemElement e) {
      boolean contains = false;
      for (MuscleElementDesc desc : b.getElements()) {
         if (desc.getElement() == e) {
            contains = true;
            break;
         }
      }
      return contains;
   }

   public void setupStaticBadinJawTongue() {
      setTonguePosture(tongue, BadinJawHyoid.regGeomDir
         + "tongue_rest_il01.txt", /* isRestPosture= */true, mm2m);
      addBadinSkull(mech);
      attachJawTongue(mech, tongue);
      enableContactSkullTongue(mech, tongue);
      // setJawPose(mech, jawOpen_a);
      // setTonguePosture(tongue, BadinJawHyoid.regGeomDir +
      // "tongue_posture_jawOpen_a.txt", /*isRestPosture =*/false);
   }

   public static final RigidTransform3d jawOpen_a = new RigidTransform3d(
      new Point3d(
         3.3059119600747096 * mm2m, -0.01769186171122445 * mm2m,
         -7.594895092878403 * mm2m),
      new AxisAngle(
         -0.002002947186028977, -0.999990881781903, 0.0037715455216803763,
         0.04424490551491432));

   public static void setJawPose(MechModel mech, RigidTransform3d pose) {
      mech.rigidBodies().get("badinjaw").setPose(pose);
   }

   public static void addBadinSkull(MechModel mech) {
      RigidTransform3d skullForward = new RigidTransform3d();
      skullForward.p.x = -2.0 * mm2m;

      File directory = new File(geometrypath + "meshesBadinJawHyoidTongue/");
      if (directory != null && directory.isDirectory()) {
         for (String fileName : directory.list()) {
            if (fileName.endsWith(".obj")) {
               RigidBody body =
                  BadinDataDemo.addBody(mech,
                     BadinDataDemo.getBasename(fileName), directory + "/"
                        + fileName);
               body.scaleDistance(mm2m);
               // body.setPose(skullForward);
               RenderProps.setFaceColor(body, new Color(1f, 0.8f, 0.6f));
               RenderProps.setFaceStyle(body, FaceStyle.FRONT_AND_BACK);
               RenderProps.setShading(body, Shading.SMOOTH);
            }
         }
      }
      else {
         System.err.println("cannot read skull meshes from " + directory);
      }
   }

   public static void attachJawTongue(MechModel mech, FemMuscleModel tongue) {
      RigidBody jaw = mech.rigidBodies().get("badinjaw");
      RigidBody hyoid = mech.rigidBodies().get("badinhyoid");
      BadinJawHyoidTongue.attachTongueToJaw(mech, tongue, jaw, hyoid);
   }

   public static void enableContactSkullTongue(MechModel mech,
      FemMuscleModel tongue) {
      RigidBody jaw = mech.rigidBodies().get("badinjaw");
      RigidBody maxilla = mech.rigidBodies().get("badinmaxilla");
      mech.setCollisionBehavior(tongue, jaw, true);
      mech.setCollisionBehavior(tongue, maxilla, true);
      showCollisions(mech, true);
   }

   public static void showCollisions(MechModel mech, boolean visible) {
      CollisionManager collisions = mech.getCollisionManager();
      RenderProps.setVisible(collisions, visible);
      RenderProps.setEdgeWidth(collisions, 2);
      RenderProps.setEdgeColor(collisions, new Color(0f, 0.4f, 0.4f));
      RenderProps.setLineWidth(collisions, 2);
      RenderProps.setLineColor(collisions, new Color(0f, 0f, 0.5f));
      collisions.setContactNormalLen(-5.0 * mm2m);
      collisions.setDrawContactNormals(true);
   }

   public static void setTonguePosture(FemMuscleModel tongue, String filename,
      boolean isRestPosture) {
      setTonguePosture(tongue, filename, isRestPosture, 1d);
   }

   public static void setTonguePosture(FemMuscleModel tongue, String filename,
      boolean isRestPosture, double scale) {
      ArrayList<Point3d> nodePositions = new ArrayList<Point3d>();

      try {
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            rtok.pushBack();
            Point3d pnt = new Point3d();
            pnt.scan(rtok);
            pnt.scale(scale);
            nodePositions.add(pnt);
         }
      } catch (IOException e) {
         e.printStackTrace();
      }

      if (nodePositions.size() == tongue.getNodes().size()) {
         for (int i = 0; i < nodePositions.size(); i++) {
            tongue.getNode(i).setPosition(nodePositions.get(i));
            if (isRestPosture)
               tongue.getNode(i).resetRestPosition();
         }
      }
      else {
         System.out.printf(
            "cannot set tongue rest pos, found %d, expecting %d\n",
            nodePositions.size(), tongue.getNodes().size());
      }
   }

   public static void setupSymmetricSimulation(MechModel mech, FemModel3d fem,
      Plane midsagittalPlane, double midsagittalTol) {

      LinkedList<FemNode3d> midsagittalNodes = new LinkedList<FemNode3d>();
      LinkedList<Point> rightsidePoints = new LinkedList<Point>();

      for (FemNode3d n : fem.getNodes()) {
         Point3d pos = n.getPosition();
         double dist = midsagittalPlane.distance(pos);
         if (Math.abs(dist) < midsagittalTol) {
            Point3d proj = new Point3d();
            midsagittalPlane.project(proj, pos);
            n.setPosition(proj);
            midsagittalNodes.add(n);
         } else if (dist > 0) { // right side is removed
            rightsidePoints.add(n);
            n.setDynamic(false);
         }
      }

      for (FemMarker m : fem.markers()) {
         double dist = midsagittalPlane.distance(m.getPosition());
         if (dist > 0) {
            rightsidePoints.add(m);
         }
      }

      for (FemNode3d n : midsagittalNodes) {
         if (n.isActive()) {
            ParticlePlaneConstraint ppc = new ParticlePlaneConstraint(n,
               midsagittalPlane);
            mech.addConstrainer(ppc);
         }
      }

      // remove rightside points, plus anything that depends on them
      LinkedList<ModelComponent> update = new LinkedList<ModelComponent>();
      LinkedList<ModelComponent> delete =
         ComponentUtils.findDependentComponents (update, rightsidePoints);
      RemoveComponentsCommand cmd =
         new RemoveComponentsCommand ("delete", delete, update);
      Main.getMain().getUndoManager().saveStateAndExecute (cmd);      
      Main.getMain().rerender();

      // Main.getMain().getSelectionManager().clearSelections();
      // for (Point p : rightsidePoints) {
      //    Main.getMain().getSelectionManager().addSelected(p);
      // }

      // LinkedList<ModelComponent> dependentSelection = Main.getMain()
      //    .getSelectionManager().getDependencyExpandedSelection();

      // Main.getMain().getSelectionManager().clearSelections();
      // RemoveComponentsCommand cmd = new RemoveComponentsCommand("delete",
      //    dependentSelection);
      //rerender();

   }

   public static void addMuscleExciterProbes (
      RootModel root, FemMuscleModel tongue) {
      addMuscleExciterProbes (root, tongue, Order.Cubic);
   }
    
   public static void addMuscleExciterProbes (
         RootModel root, FemMuscleModel tongue, Order interpolationOrder) {

      double duration = 1;
      double totalDuration = 1.2 * duration;

      for (MuscleExciter ex : tongue.getMuscleExciters ()) {
         addMuscleExciterProbe (root, ex, interpolationOrder, duration);
      }
      
      root.addBreakPoint (totalDuration);
   }
   
   public static void addMuscleExciterProbe (RootModel root, MuscleExciter ex, Order interpolationOrder, double duration) {
      addMuscleExciterProbe (root, ex, interpolationOrder, duration, defaultExcitationProbeMagnitude);
   }
      
   public static void addMuscleExciterProbe (RootModel root, MuscleExciter ex, Order interpolationOrder, double duration, double excitation) {
      if (ex==null) {
         return;
      }
      
      try {
         NumericInputProbe inprobe = new NumericInputProbe (ex, "excitation", null);
         String name = "ex_" + ex.getName ();
         inprobe.setName (name);
         inprobe.setAttachedFileName (name + ".txt");
         inprobe.setActive (true);
         inprobe.setStartStopTimes (0, duration);
         inprobe.loadEmpty ();
         root.addInputProbe (inprobe);
         inprobe.setActive (false);
         inprobe.getNumericList ().clear ();
         inprobe.addData (
            new double[] { 0, excitation, 0 },
            duration / 2);
         inprobe.setInterpolationOrder (interpolationOrder);

         // double ex = defaultExcitationProbeMagnitude;
         // inprobe.addData(new double[]{0,ex, ex, ex ,0}, duration/4);
         // inprobe.setInterpolationOrder(Order.Linear);
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace ();
      }
   }


   public static void setActivationColor(FemMuscleModel tongue) {
      for (MuscleBundle b : tongue.getMuscleBundles()) {
         if (b.getExcitationColor() != null)
            continue; // don't set excitation color multiple times
         Color exColor = b.getRenderProps().getLineColor();
         b.setExcitationColor(exColor);
         for (Muscle m : b.getFibres()) {
            m.setExcitationColor(exColor);
            m.setRenderProps(new LineRenderProps());
         }
         RenderProps.setLineColor(b, Color.WHITE);
      }

      tongue.setMaxColoredExcitation(defaultExcitationProbeMagnitude);

      // tongue.setSurfaceRendering(SurfaceRender.Shaded);
      // RenderProps.setAlpha(tongue, 0.5);
      // tongue.setMaxColoredExcitation(0.8);
   }
}
