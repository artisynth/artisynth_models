package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.GL.GLViewer;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemMuscleStiffener;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.tongue3d.FemMuscleTongueDemo;
import artisynth.models.tongue3d.HexTongueDemo;
import artisynth.models.tongue3d.TetTongueDemo;

public class BadinJawHyoidTongue extends BadinJawHyoid {

   public String getAbout() {
      return "A 3D dynamically coupled rigid-body jaw and FEM tongue model for studying speech and chewing biomechanics.\n\n"
         +
         "The model was developed by Ian Stavness, please cite: \n"
         +
         "Ian Stavness, John E Lloyd, Yohan Payan, and Sidney Fels. "
         +
         "Coupled Hard-Soft Tissue Simulation with Contact and Constraints Applied to Jaw-Tongue-Hyoid Dynamics. "
         +
         "International Journal for Numerical Methods in Biomedical Engineering, 27:367-390, 2011.";
   }

   boolean drawContacts = false;
   boolean collideTongueMaxilla = true;
   boolean collideTongueJaw = true;
   boolean addStaticBadinGeometry = false;
   boolean addSoftPalateModel = false;

   boolean useLinearMaterial = false;
   boolean useIcpMuscleDefs = true;
   boolean useIncompressibleConstraint = true;
   boolean doActivationStiffening = true;
   
   public static final TongueType DEFAULT_TONGUE_TYPE = TongueType.HexLineMuscles;
   
   public enum TongueType {
      TetLineMuscles,
      HexLineMuscles,
      HexElementMuscles,
   }

   protected FemMuscleModel tongue;
   protected FemMuscleStiffener stiffener = null;
   VectorNd tongueContour = new VectorNd();

   public static PropertyList myProps =
      new PropertyList(BadinJawHyoidTongue.class, BadinJawHyoid.class);

   static {
      myProps.addReadOnly(
         "tongueContour", "contour of tongue mesh in mid-sagittal plane", null);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   @Override
   public StepAdjustment advance(
      double t0, double t1, int flags) {
      if (stiffener != null && doActivationStiffening) {
         stiffener.updateElemStiffnesses();
      }
      return super.advance(t0, t1, flags);
   }

   public BadinJawHyoidTongue () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      TongueType tongueType = DEFAULT_TONGUE_TYPE;
      try {
       tongueType = TongueType.valueOf (args[0]);
      } catch (Exception e) {
         // use default
      }
      
      addTongueToJaw(tongueType);

      attachTongueToJaw();

      if (collideTongueMaxilla) {
         addContact("maxilla"); // palate
         myJawModel.setPenetrationTol(0.002);
      }

      if (collideTongueJaw) {
         addContact("jaw");
      }

      if (drawContacts) {
         showCollisions(collideTongueJaw || collideTongueMaxilla);
      }

      if (fixHyoid) {
         myJawModel.rigidBodies().get("hyoid").setDynamic(false);
      }

      if (addStaticBadinGeometry) {
         String[] meshNames = new String[] { "badinskin", "badinpalate" };
         Color[] meshColor = new Color[] { new Color(.3f, .3f, .4f), null };
         for (int i = 0; i < meshNames.length; i++) {
            RigidBody body =
               addBody("badinskin", regGeomDir + meshNames[i] + ".obj");
            RenderProps.setVisible(body, true);
            Color color =
               (meshColor[i] == null ? new Color(1f, .8f, .6f) : meshColor[i]);
            RenderProps.setFaceColor(body, color);
         }
      }

      // if (addSoftPalateModel)
      // addSoftPalate ();

      String[] redundantMuscles =
         new String[] { "lgh", "lam", "lpm", "rgh", "ram", "rpm" };
      for (String muscleName : redundantMuscles) {
         if (myJawModel.axialSprings().get(muscleName) != null) {
            Muscle m =
               (Muscle)myJawModel.axialSprings().get(muscleName);
            m.setEnabled(false);
            RenderProps.setVisible(m, false);
         }
      }

      RenderProps.setVisible(myJawModel.frameMarkers(), false);
   }

   public void addTongueToJaw(TongueType tt) {
      System.out.println("adding tongue of type "+tt.toString ());
      switch(tt) {
         case TetLineMuscles:
            tongue = TetTongueDemo.createTetTongue(useLinearMaterial, useIcpMuscleDefs);
            tongue.scaleDistance(m2mm);
            
            stiffener = new FemMuscleStiffener(tongue);

            RenderProps.setPointStyle(tongue, PointStyle.SPHERE);
            RenderProps.setPointRadius(tongue, 1);
            RenderProps.setLineWidth(tongue, 1);
            RenderProps.setLineStyle(tongue.getMuscleBundles(), LineStyle.CYLINDER);
            RenderProps.setLineRadius(tongue.getMuscleBundles(), 0.3);
            RenderProps.setVisible(tongue.getNodes(), false);
            RenderProps.setVisible(tongue.getElements(), true);
            RenderProps.setLineWidth(tongue.getElements(), 1);
            
            break;
         case HexLineMuscles:
            tongue = HexTongueDemo.createHexTongue(useLinearMaterial, useIcpMuscleDefs);
            tongue.scaleDistance(m2mm);
            
            stiffener = new FemMuscleStiffener(tongue);

            RenderProps.setPointStyle(tongue, PointStyle.SPHERE);
            RenderProps.setPointRadius(tongue, 1);
            RenderProps.setLineWidth(tongue, 1);
            RenderProps.setLineStyle(tongue.getMuscleBundles(), LineStyle.CYLINDER);
            RenderProps.setLineRadius(tongue.getMuscleBundles(), 0.3);
            RenderProps.setVisible(tongue.getNodes(), false);
            RenderProps.setVisible(tongue.getElements(), true);
            RenderProps.setLineWidth(tongue.getElements(), 1);

            break;
         case HexElementMuscles:
            tongue = FemMuscleTongueDemo.createFemMuscleTongue (useLinearMaterial);
            tongue.scaleDistance(m2mm);
            
//            GenericMuscle mat = new GenericMuscle();
            // mat.setMaxStress(60000);
//            tongue.setMuscleMaterial(mat);
            
            useIncompressibleConstraint = false; // hard incompressibility unstable with element muscles

            FemMuscleTongueDemo.addExciters(tongue);
            
            tongue.setElementWidgetSize(0);
            tongue.setDirectionRenderLen(0.5);
            // tongue.setSurfaceRendering (SurfaceRender.Strain);
            tongue.setSurfaceRendering(SurfaceRender.None);
            // tongue.setSurfaceRendering(SurfaceRender.Shaded);
            RenderProps.setVisible(tongue.getElements(), true);
            RenderProps.setVisible(tongue.getNodes(), false);
            RenderProps.setLineStyle(tongue.getMuscleBundles(), LineStyle.LINE);
            RenderProps.setLineWidth(tongue.getMuscleBundles(), 2);
            RenderProps.setVisible(tongue.getMuscleBundles(), true);
            RenderProps.setLineWidth(tongue.getElements(), 1);

            break;         
         default: 
            System.err.println("unknown tongue type "+tt);
            return;
      }


      if (useIncompressibleConstraint) {
         tongue.setIncompressible(IncompMethod.AUTO);
      }
      else {
         tongue.setIncompressible(IncompMethod.OFF);
      }

      RigidTransform3d tongueBackward = new RigidTransform3d();
      tongueBackward.p.x = 2.0; // mm
      tongue.transformGeometry(tongueBackward);
      
      myJawModel.addModel(tongue);

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

   public static FemNode3d[] getHyoidAttachments(FemModel3d fem) {
      Integer[] attachedNodeIndices =
         readIntList(BadinJawHyoid.regGeomDir + "hyoidAttachmentsPartial.txt");
      return getAttachments(fem, attachedNodeIndices);
   }

   public static FemNode3d[] getJawAttachments(FemModel3d fem) {
      Integer[] attachedNodeIndices =
         readIntList(BadinJawHyoid.regGeomDir + "jawAttachmentsStephanie.txt");
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

   public void attachTongueToJaw() {
      RigidBody jaw = myJawModel.rigidBodies().get("jaw");
      RigidBody hyoid = myJawModel.rigidBodies().get("hyoid");
      FemMuscleModel tongue = (FemMuscleModel)myJawModel.models().get(0);
      attachTongueToJaw(myJawModel, tongue, jaw, hyoid);
   }

   public static void attachTongueToJaw(MechModel mech, FemMuscleModel tongue,
      RigidBody jaw, RigidBody hyoid) {
      for (FemNode n : tongue.getNodes()) {
         n.setDynamic(true);
         n.setRenderProps(null);
      }
      // set styloglossus insertion points non-dynamic
      tongue.getNode(tongue.getNodes().size() - 1).setDynamic(false);
      tongue.getNode(tongue.getNodes().size() - 2).setDynamic(false);

      for (FemNode3d n : getJawAttachments(tongue)) {
         attachNode(mech, jaw, n, Color.RED);
      }

      for (FemNode3d n : getHyoidAttachments(tongue)) {
         attachNode(mech, hyoid, n, Color.BLUE);
      }

   }

   public static void attachNode(MechModel mech, RigidBody body, FemNode node,
      Color color) {
      mech.attachPoint(node, body);
      RenderProps.setPointColor(node, color);
      RenderProps.setVisible(node, true);
   }

   public void addContact(String bodyName) {
      FemMuscleModel tongue =
         (FemMuscleModel)myJawModel.findComponent("models/0");
      if (tongue == null)
         return;
      RigidBody body = myJawModel.rigidBodies().get(bodyName);
      RenderProps.setVisible(body, true);
      myJawModel.setCollisionBehavior(tongue, body, true);
   }

   public void showCollisions(boolean visible) {
      CollisionManager collisions = myJawModel.getCollisionManager();
      RenderProps.setVisible(collisions, visible);
      RenderProps.setEdgeWidth(collisions, 2);
      RenderProps.setEdgeColor(collisions, new Color(0f, 0.4f, 0.4f));
      RenderProps.setLineWidth(collisions, 2);
      RenderProps.setLineColor(collisions, new Color(0f, 0f, 0.5f));
      collisions.setContactNormalLen(-5.0);
      collisions.setDrawContactNormals(true);
   }

   // public void addSoftPalate () {
   // try {
   // SoftPalateDemo palate = new SoftPalateDemo ("softpalate");
   // MechModel spMech = (MechModel)palate.models ().get (0);
   // spMech.scaleDistance (BadinDataDemo.mm2m);
   // RigidTransform3d X = new RigidTransform3d ();
   // X.p.set (0.00571704, -0.12648, 0.0327923);
   // X.R.setAxisAngle (0, 0, 1, Math.PI / 2);
   // spMech.transformGeometry (X);
   // for (RigidBody body : spMech.rigidBodies ()) {
   // myJawModel.addRigidBody (body);
   // }
   // myJawModel.addModel ((FemModel3d)spMech.models ().get (0));
   // }
   // catch (IOException e) {
   // e.printStackTrace ();
   // }
   // }

   ControlPanel myControlPanel = null;

   public void attach(DriverInterface driver) {
      super.attach(driver);

      if (tongue != null) {
         myControlPanel =
            FemControlPanel.createControlPanel(this, tongue, myJawModel);
         FemControlPanel.createMusclePanel(
            this, tongue, /* exciters */true);
      }

      // HexTongueDemo.addInProbes (this, tongue, duration);
      // HexTongueDemo.addOutProbes (this, tongue, duration);
      removeAllInputProbes();
      removeAllOutputProbes();

      File workingDir = new File(ArtisynthPath.getSrcRelativePath(
         BadinJawHyoidTongue.class, "data/jtdata/jawtongue"));
      if (workingDir.exists()) {
         ArtisynthPath.setWorkingDir(workingDir);
      }

      /*
       * setup camera for video
       */

      // GLViewerFrame vf = (GLViewerFrame)driver.getFrame();
      GLViewer v = driver.getViewer();
      if (v != null) {
         v.setOrthographicView(true);
         v.setGridVisible(true);
         // vm.setBackgroundColor (Color.WHITE);
         // vc.setAxialView (AxisAlignedRotation.Front);
         // vc.getGrid ().setColor (Color.BLACK);
         // vc.getGrid ().setDivisionColor (new Color (0.6f, 0.6f, 0.6f));
         // vc.getGrid ().setPosition (gridPos);
         // vc.getGrid ().setLineWidth (2);

         // v.setOrthogonal (6, 1, 1000);
         v.setEye(new Point3d(100, -700, 95));
         v.setCenter(new Point3d(100, 0, 95));
      }
      // RigidTransform3d sagittalClip = new RigidTransform3d(
      // new Vector3d(100, 0, 95),
      // new AxisAngle(1, 0, 0, Math.PI/2));

      // GLClipPlane clip = vc.getToolBar ().addClipPlane ();
      // clip.setOffset (0.25);
      // clip.setGridVisible (false);
      // clip.setDragger (DraggerType.None);

   }

   public void loadProbes() {
      String modelDir;
      if (useLinearMaterial)
         modelDir = "lin/";
      else {
         if (useIncompressibleConstraint)
            modelDir = "hyp/";
         else
            modelDir = "bulk/";
      }

      File outputDataDir =
         new File(ArtisynthPath.getSrcRelativePath(this, "data/jtdata/"
            + modelDir + taskName));
      if (outputDataDir.exists()) {
         ArtisynthPath.setWorkingDir(outputDataDir);
      }

      NumericOutputProbe op =
         new NumericOutputProbe(this, "jawContour", "jawcontour.txt", 0.1);
      op.setName("jaw contour");
      op.setStartStopTimes(0, 1);
      addOutputProbe(op);

      op =
         new NumericOutputProbe(
            this, "tongueContour", "tonguecontour.txt", 0.1);
      op.setName("tongue contour");
      op.setStartStopTimes(0, 1);
      addOutputProbe(op);
   }

   public double[] getTongueContourTols() {
      return new double[] { 0.1, 0.1 };
   }

   protected PolygonalMesh tonguemesh = null;
   protected LinkedList<Vertex3d> tonguevertices = null;

   public VectorNd getTongueContour() {
      if (tonguemesh == null || tonguevertices == null) {
         tonguemesh = ((FemModel3d)myJawModel.models().get(0)).getSurfaceMesh();
         tonguevertices =
            getContourVertices(
               tonguemesh, getTongueContourTols()[0], getTongueContourTols()[1]);
         tongueContour = new VectorNd(tonguevertices.size() * 2);
         System.out.println(tonguevertices.size() + "tongue contour pnts, "
            + getTongueContourTols()[0] + " tol");
      }
      getMeshContour(tongueContour, tonguevertices, null);
      return tongueContour;
   }

}
