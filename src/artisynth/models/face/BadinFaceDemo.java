package artisynth.models.face;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.GenericMeshReader;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderable;
import maspack.util.ReaderTokenizer;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.AnsysWriter;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.Model;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.WayPoint;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.tongue3d.HexTongueDemo;

public class BadinFaceDemo extends RootModel {

   public static final boolean leftSideOnly = false;
   public static final boolean drawContactNormals = false;
   public static final boolean splitMusclesBilaterally = false;
   public static final boolean linearMaterial = false;
   public static final double defaultMaxStepSizeSec = 0.005;
   public static final Integrator defaultIntegrator =
   Integrator.ConstrainedBackwardEuler;

   public static final String faceGeometryDir =
   ArtisynthPath.getSrcRelativePath (BadinFaceDemo.class, "geometry/");

   public static LinkedHashMap<Integer,Integer> nodeIdToIndex = null;

   public static double midsagittalTol = 1e-4;
   public static Plane midSagittalPlane = new Plane(Vector3d.Y_UNIT, 0);

   double myIntraoralPressure = 0;

   MechModel mech;
   FemMuscleModel face;
   
   // Use the internal muscles and probes defined by Flynn and Chiu
   String workingDir = ArtisynthPath.getSrcRelativePath(BadinFaceDemo.class, "data_chenhao/");
   public static String muscleNodesFile   = faceGeometryDir + "face_muscles_CH_with_LLS.node";
   public static String muscleConnectFile = faceGeometryDir + "face_muscles_topology_flynn.mac";
   static String[] faceMuscleNames = new String[] {
         "DAO",   // depressor anguli oris
         "BUC",   // buccinator
         "DLI",   // depressor labii inferioris
         "MENT",  // mentalis
         "MAS",   // masseter (not used)
         "OOM",   // obicularis oris middle
         "OOP",   // obicularis oris peripheral
         "LLSAN", // levator labii superioris alaeque nasi
         "LAO",   // levator anguli oris
         "RIS",   // risorius
         "ZYG",   // zygomatic
         "LLS"    // levator labii superioris
   };
   boolean origModel = false; // The updates by Flynn and Chiu should generally be used (origModel=false) 

   /*
    * set up properties for face demo
    */
   public static PropertyList myProps =
   new PropertyList (BadinFaceDemo.class, RootModel.class);

   static {
      myProps.add ("jawHinge * *", "hinge angle for jaw", 0.0, "[0,45]");
      myProps.add ("intraoralPressure * *", "external force applied normal to dynamic nodes on inner surface of the face", 0);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   /*
    * assemble face model in constructor
    */
   public BadinFaceDemo() {
      super ();
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      // create mech model
      mech = new MechModel ("mech");
      mech.setMaxStepSize (defaultMaxStepSizeSec);
      mech.setIntegrator (defaultIntegrator);
      addModel (mech);
      
      if (origModel==true)
         setOrigModel();
      
      ArtisynthPath.setWorkingDir(new File( workingDir ));
      
      // create dynamic face model
      if (leftSideOnly)
         face = createFace (faceGeometryDir, "face_leftside_jawconformed", linearMaterial);
      else
         face = createFace (faceGeometryDir, "badinface_oop_csa_midsagittal", linearMaterial);
      //	 face = createFace (faceGeometryDir, "badinface_newteeth_conformed_upperfixed_Quality", linearMaterial);
      //	 face = createFace (faceGeometryDir, "face_skullconformed_quality", linearMaterial);


      face.setName ("badinface");
      mech.addModel (face);

      // create static rigid bodies for skull meshes
      addStaticSkull ();  

      RigidBody maxilla = mech.rigidBodies().get("badinmaxilla");
      RigidBody jaw = mech.rigidBodies ().get ("badinjaw");

      addCondylePoint ();

      /* setup face for conforming inner-surface with jaw */
      //      conformFaceToBody(face, jaw);
      //      conformFaceToBody(face, maxilla);

      /* attach face nodes that are close to the jaw surface */
      //    attachAdjacentFaceNodesToJaw(jaw, face);

      if (origModel == true)
      {
         /* fix all inner-surface face nodes, except those listed in lip_free_nodes.node */
         attachFaceToSkull (face);
      }
      else
      {
         /* fix all inner-surface nose nodes, and zygomatic ligament nodes, and edge of domain nodes */
         setNodesNonDynamicByNumber(face, "edge_nodes.nodenum");
         setNodesNonDynamicByNumber(face, "nose_nodes_to_fix.txt");
         setNodesNonDynamicByNumber(face, "zygomatic_ligament_attachments.txt");
      }

      // attach face nodes to jaw from list of indices
      attachFaceToJaw (mech, jaw, face, "face_jaw_attachments.txt");

      setupRenderProps ();

      enableLipLipContact();
      enableLipSkullContact();
      //      showMiddleLayer();



      if (leftSideOnly) {
         addMuscles(face, faceGeometryDir + "face_leftside_muscles.node", midSagittalPlane);
         HexTongueDemo.setupSymmetricSimulation(mech, face, midSagittalPlane, midsagittalTol);
      }
      else {
         addMuscles(face, muscleNodesFile, midSagittalPlane);
      }

      //      editFaceMeshMode();
      //      fixOuterNodes(face);
      //      face.setGravity(Vector3d.ZERO);
      //      mech.setGravity(Vector3d.ZERO);
      //      face.setIncompressible(FemModel3d.IncompMethod.OFF);

      //      showIntraoralNodes();
      //      showIntraoralFaces();
      //      System.out.println("intraoralSurfaceArea = "+getIntraoralSurfaceArea() + " m^2");

      if (origModel == false)
      {
         for (Integer ni : readIntList (faceGeometryDir + "cheek_area_nodes.nodenum")) {
            FemNode3d n = face.getByNumber(ni);
            RenderProps.setPointColor(n, Color.BLUE);
         }
         RenderProps.setVisible(face.getNodes(), true);
      }
   }


   ArrayList<Point> leftpts = new ArrayList<Point>();
   ArrayList<Point> rightpts = new ArrayList<Point>();
   double tol = 1e-4;
   Point3d reflect = new Point3d();

   public void editFaceMeshMode() {
      for (FemNode3d p : face.getNodes()) {
         if (midSagittalPlane.distance(p.getPosition()) < -midsagittalTol) {
            leftpts.add(p);
            RenderProps.setPointColor(p, Color.MAGENTA.brighter());
         }
         else {
            RenderProps.setPointColor(p, Color.CYAN);
         }
         RenderProps.setVisible(p, true);

      }

      ArrayList<Point> nomatch = new ArrayList<Point>();
      for (Point lp : leftpts) {
         Point rp = findRightSidePoint(lp);
         if (rp == null) {
            System.out.println("no right side point found for p"+lp.getNumber());
            RenderProps.setPointColor(lp, Color.RED);
            //	    RenderProps.setPointRadius(lp, 0.001);
            nomatch.add(lp);
         }
         else { 
            if (rightpts.contains(rp)) {
               System.err.println("found duplicate right-side point");
               RenderProps.setPointColor(rp, Color.ORANGE.brighter());

            }
            else {
               rightpts.add(rp);
               RenderProps.setPointColor(rp, Color.MAGENTA.darker());
            }
         }

      }

      leftpts.removeAll(nomatch);
   }

   private Point findRightSidePoint(Point p) {
      Point rightPt = null;
      Point3d pos = p.getPosition();
      midSagittalPlane.reflect(reflect, pos);
      for (FemNode3d n : face.getNodes()) {
         if (reflect.distance(n.getPosition()) < tol) {
            rightPt = n;
            break;
         }
      }
      return rightPt;
   }

   public void mirror() {
      assert rightpts.size() == leftpts.size();
      for (int i = 0; i < rightpts.size(); i++) {
         midSagittalPlane.reflect(reflect, rightpts.get(i).getPosition());
         leftpts.get(i).setPosition(reflect);
      }
      rerender();
   }


   public static FemMuscleModel createFace(String meshDirectory,
      String meshFilePrefix, boolean linearMaterial) {
      return createFace(meshDirectory, meshFilePrefix, 1d, linearMaterial);
   }

   public static FemMuscleModel createFace(String meshDirectory,
      String meshFilePrefix, double scale, boolean linearMaterial) {

      FemMuscleModel fem = new FemMuscleModel (meshFilePrefix);
      try {
         AnsysReader.read(fem, meshDirectory + meshFilePrefix + ".node",
            meshDirectory + meshFilePrefix + ".elem", 1d, new Vector3d(scale,scale,scale),
            /* options= */AnsysReader.ONE_BASED_NUMBERING);

         //       AnsysWriter.writeNodeFile (fem, faceGeometryDir + meshFilePrefix +"_new.node");
         //       AnsysWriter.writeElemFile (fem, faceGeometryDir + meshFilePrefix +"_new.elem");

      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      fem.setSurfaceRendering (SurfaceRender.None);
      fem.setElementWidgetSize (1.0);
      if (linearMaterial) {
         fem.setMaterial (new LinearMaterial ());
      }
      else {
         fem.setMaterial (new MooneyRivlinMaterial ());
      }
      RenderProps.setVisible (fem.getNodes (), true);
      setFaceProperties (fem);

      return fem;
   }

   public static void setFaceProperties (FemModel3d fem) {
      fem.setGravity (0, 0, -9.8);
      fem.setDensity (1000);
      fem.setParticleDamping (1.22);
      fem.setStiffnessDamping (0.05);

      if (fem.getMaterial() instanceof LinearMaterial) {
         //fem.setWarping (true);
         fem.setLinearMaterial (15000, 0.49, true); // Nazari 2010 Motor Control paper
      }
      else if (fem.getMaterial () instanceof MooneyRivlinMaterial) {
         MooneyRivlinMaterial mrmat = (MooneyRivlinMaterial)fem.getMaterial ();

         // Nazari 2010 Motor Control paper
         mrmat.setC10 (2500); // Pa
         mrmat.setC20 (1175); // Pa

         // mrmat.setBulkModulus (100*mrmat.getC10 ()); // 100x c10 ~
         // possion=0.49
         mrmat.setBulkModulus (10 * mrmat.getC10 ());

         // [Buchaillard 2009]
         // Rayleigh damping C = a M + b K
         // a = 40 s^-1
         // b = 0.03 s
         fem.setParticleDamping (19);
         fem.setStiffnessDamping (0.055);

         // [Buchaillard 2009]
         // density = 1040 kg m^-3
         fem.setDensity (1040);
      }

      fem.setIncompressible (IncompMethod.AUTO);
      fem.setIntegrator (defaultIntegrator);
      fem.setMaxStepSize (defaultMaxStepSizeSec);
      fem.setImplicitIterations (10);
      fem.setImplicitPrecision (0.001);
   }

   public void addStaticSkull () {
      String[] meshNames =
      new String[] { "badinskull", "badinskin", "badintongue",
                     "badinjaw", "badinmaxilla",
                     "badincranium", "upperteeth", "lowerteeth" };

      for (String meshName : meshNames) {
         String[] names = meshName.split ("_");
         addBody (mech, names[0], faceGeometryDir + meshName + ".obj");
      }
   }

   public static RigidBody addBody (MechModel mech, String name, String fullMeshFileName) {
      return addBody(mech, name, fullMeshFileName, 1d);
   }

   public static RigidBody addBody (MechModel mech, String name, String fullMeshFileName, double scale) {
      RigidBody body = new RigidBody (name);
      body.setDynamic (false);
      try {
         PolygonalMesh mesh = new PolygonalMesh ();
         mesh.read (new FileReader (fullMeshFileName));
         mesh.scale(scale);
         body.setMesh (mesh, null);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      mech.addRigidBody (body);
      RenderProps.setFaceColor (body, Color.LIGHT_GRAY);
      return body;
   }

   public void conformFaceToBody (FemModel3d face, RigidBody body) {
      fixOuterNodes (face);
      mech.setCollisionBehavior (face, body, true);
      mech.setGravity(0, 0, 0);

      // add probe to push jaw into inside of face
      //      Point3d initBodyPos = new Point3d (0.01, 0, 0.005); // jaw
      Point3d initBodyPos = new Point3d (0.005, 0, 0.002); // maxilla
      body.setPosition (initBodyPos);
      double duration = 0.1;
      NumericInputProbe posprobe =
      new NumericInputProbe (body, "position", 0, duration);
      posprobe.addData (0, initBodyPos);
      posprobe.addData (duration, new Vector3d ());
      addInputProbe (posprobe);
      addBreakPoint (duration + 2 * defaultMaxStepSizeSec);
   }

   public static void attachFaceToSkull (FemModel3d face) {
      fixInnerNodes (face);

      // un-fix nodes around the lips and cheeks

      setNodesDynamicByNumber(face, "lip_free_nodes.nodenum");
      setNodesDynamicByNumber(face, "cheek_free_nodes.nodenum");
      setNodesDynamicByNumber(face, "nose_free_nodes.nodenum");


      //      setNodesDynamicAnsys(face, "lip_free_nodes.node");
      //      setNodesDynamicByIndex (face, "cheek_free_nodes.txt");
      //      setNodesDynamicByNumber (face, "more_free_nodes_ansys.txt");
   }

   public static void addMuscles(FemMuscleModel face, String muscleNodeFilename, Plane midSagittalPlane) {
      addMuscles(face, muscleNodeFilename, 1d, midSagittalPlane);
   }

   public static void addMuscles(FemMuscleModel face, String muscleNodeFilename, double scale, Plane midSagittalPlane) {
      try {

         nodeIdToIndex = new LinkedHashMap<Integer,Integer> ();

         LinkedHashMap<Integer, Point3d> nodeMap = AnsysReader.readNodeFile(
            new FileReader(muscleNodeFilename), true);

         for (int markerId : nodeMap.keySet ()) {
            Point3d pos = nodeMap.get (markerId);
            pos.scale(scale);
            FemMarker marker = face.addNumberedMarker (pos, markerId);
            nodeIdToIndex.put (markerId, marker.getNumber ());
         }

         AnsysFaceMuscleFiberReader.read (face, nodeIdToIndex, new FileReader ( muscleConnectFile ));
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

      if (splitMusclesBilaterally) {
         nameColorMuscles (face, "_L");
      }
      else {
         nameColorMuscles(face);
      }

      //      closeOOP();

      if (!leftSideOnly) {
         addRightSideMuscles(face, midSagittalPlane, splitMusclesBilaterally);
         if (splitMusclesBilaterally)
            addBilateralExciters(face);
      }
   }

   public static void addRightSideMuscles(FemMuscleModel face, Plane midSagittalPlane, boolean addRightBundles) {
      System.out.println("start add rightside muscles");
      deleteRightSideMarkers(face, midSagittalPlane);
      ArrayList<Muscle> toAdd = new ArrayList<Muscle>();
      ArrayList<MuscleBundle> bundlesToAdd = new ArrayList<MuscleBundle>();
      for (MuscleBundle leftBundle : face.getMuscleBundles()) {
         String[] str = leftBundle.getName().split("_");
         String muscleName = str[0];
         for (Muscle m : leftBundle.getFibres()) {
            Point p1 = createRightSideMarker(face, m.getFirstPoint(), midSagittalPlane);
            Point p2 = createRightSideMarker(face, m.getSecondPoint(), midSagittalPlane);
            if (p1 == null || p2 == null) {
               System.err.println("no right side match");
               continue;
            }
            Muscle rightMuscle = (Muscle)m.copy(0, null);
            rightMuscle.setFirstPoint(p2);
            rightMuscle.setSecondPoint(p1);
            toAdd.add(rightMuscle);
         }

         MuscleBundle bundle;
         if (addRightBundles) {
            bundle = new MuscleBundle(muscleName + "_R");
            RenderProps.setLineColor(bundle, leftBundle.getRenderProps()
               .getLineColor().darker());
            bundlesToAdd.add(bundle);
         } else {
            bundle = leftBundle;
         }

         for (Muscle m : toAdd) {
            bundle.addFibre(m);
         }
         toAdd.clear();
      }
      for (MuscleBundle bundle : bundlesToAdd) {
         face.addMuscleBundle(bundle);
      }
   }

   public static void addBilateralExciters(FemMuscleModel face) {
      
      String[] lr = new String[]{"_L", "_R"};
      for (String name : faceMuscleNames) {
         if (name.equalsIgnoreCase("MAS") == true)
            continue; // masseter muscle not used
         
         MuscleExciter ex = new MuscleExciter(name);
         for (String suffix : lr) {
            ex.addTarget(face.getMuscleBundles().get(name+suffix), 1d);
         }
         face.addMuscleExciter(ex);
      }
   }

   private static void deleteRightSideMarkers(FemMuscleModel face, Plane midSagittalPlane) {
      LinkedList<FemMarker> toDelete = new LinkedList<FemMarker>();
      for (FemMarker m : face.markers()) {
         double dist = midSagittalPlane.distance(m.getPosition());
         if (dist > midsagittalTol)
            toDelete.add(m);
      }

      face.markers().removeAll(toDelete);
   }

   private static Point createRightSideMarker(FemMuscleModel face, Point p, Plane midSagittalPlane) {
      Point rightPt = null;
      Point3d pos = p.getPosition();
      double dist = midSagittalPlane.distance(pos);
      if (dist < -midsagittalTol) {
         Point3d reflect = new Point3d();
         midSagittalPlane.reflect(reflect, pos);
         rightPt = createMarker(face, reflect);
         face.addMarker((FemMarker)rightPt);
      } else
         rightPt = p; // right-side or mid-sagittal point

      return rightPt;
   }


   public static void closeOOP(FemMuscleModel face) {
      int[][] endptidxs = new int[][]{
                                      // right-side, left-side
                                      {134121, 114121}, // upper part
                                      {134136, 114136} // lower part
      };

      int[] midptnodeidx = new int[] {
                                      521, // upper
                                      3583 // lower
      };

      for (int i = 0; i < 2; i++) {
         FemMarker[] endpts = new FemMarker[2];

         endpts[1] = face.markers().getByNumber(endptidxs[i][1]);

         FemNode3d midnode = face.getByNumber(midptnodeidx[i]);
         Point3d midpos = new Point3d();
         midpos = midnode.getPosition();
         FemMarker midmarker = createMarker(face, midpos);
         face.addMarker(midmarker);
         face.getMuscleBundles().get("OOP").addFibre(new Muscle(midmarker, endpts[1]));
      }
   }

   private static FemMarker createMarker(FemMuscleModel face, Point3d pos) {
      FemElement3dBase elem = face.findContainingElement(pos);
      if (elem == null) {
         Point3d origpos = new Point3d(pos);
         elem = face.findNearestSurfaceElement(pos, origpos);
      }
      FemMarker m = new FemMarker(elem, pos);
      return m;
   }


   public static void nameColorMuscles (FemMuscleModel face) {
      nameColorMuscles(face, "");
   }

   public static void nameColorMuscles (FemMuscleModel face, String suffix) {
      
      for (int i = 0; i < face.getMuscleBundles ().size (); i++) {
         MuscleBundle b = face.getMuscleBundles ().get(i);
         b.setName (faceMuscleNames[i]+suffix);
         System.out.println(b.getName());
         RenderProps.setVisibleMode (b, PropertyMode.Inherited);
         RenderProps.setLineColor (b, FemControlPanel.getMuscleColor (b.getNumber ()));
      }

      // masseter muscle not used
      face.removeMuscleBundle(face.getMuscleBundles().get("MAS"));
   }

   public void writeMuscleNodeFile (String fileName) {
      if (nodeIdToIndex == null) {
         System.err.println ("cannot write face msucle node file, nodeIdToIndex map is null");
         return;
      }

      FemModel3d face =
      (FemModel3d)((MechModel)this.models ().get (0)).models ().get (0);

      try {
         PrintWriter pw =
         new PrintWriter (new File (faceGeometryDir + fileName));
         for (Integer nodeId : nodeIdToIndex.keySet ()) {
            FemMarker mkr = face.markers ().get (nodeIdToIndex.get (nodeId));
            pw.printf ("%d %s\n", nodeId, mkr.getPosition ().toString ("%14.9f"));
         }
         pw.close ();
      }
      catch (FileNotFoundException e) {
         e.printStackTrace ();
      }

   }

   public static void setNodesDynamicAnsys (FemModel3d fem, String ansysNodeFilename) {
      for (FemNode3d n : getNodes (fem, faceGeometryDir + ansysNodeFilename)) {
         n.setDynamic (true);
         n.setRenderProps (null);
      }
   }

   public static void setNodesDynamicByNumber (FemModel3d fem, String nodeIndicesFilename) {
      for (Integer ni : readIntList (faceGeometryDir + nodeIndicesFilename)) {
         FemNode3d n = fem.getByNumber(ni);
         if (n != null) {
            n.setDynamic (true);
            n.setRenderProps (null);
         }
      }
   }

   public static void setNodesDynamicByIndex (FemModel3d fem, String nodeIndicesFilename) {
      for (Integer ni : readIntList (faceGeometryDir + nodeIndicesFilename)) {
         if (ni < fem.numNodes()) {
            FemNode3d n = fem.getNode(ni);
            n.setDynamic (true);
            n.setRenderProps (null);
         }
      }
   }

   public static Integer[] readIntList (String fullPathFileName) {
      ArrayList<Integer> idxs = new ArrayList<Integer> ();
      try {
         ReaderTokenizer rtok =
         new ReaderTokenizer (new FileReader (fullPathFileName));
         while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
            idxs.add ((int)rtok.lval);
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      return idxs.toArray (new Integer[0]);
   }

   public static void attachFaceToJaw (MechModel mech, RigidBody jaw, FemModel3d face, String fileName) {
      for (Integer idx : readIntList (faceGeometryDir + fileName)) {
         if (idx < face.numNodes()) {
            FemNode3d n = face.getNode(idx);
            mech.attachPoint(n, jaw);
            RenderProps.setPointSize(n, 5);
            RenderProps.setPointColor(n, Color.RED);
         }
      }
   }


   public void attachAdjacentFaceNodesToJaw (RigidBody jaw, FemModel3d face) {

      PolygonalMesh mesh = jaw.getMesh ();
      //OBBTree obbt = mesh.getObbtree ();
      Point3d proj = new Point3d ();
      Vector2d coords = new Vector2d ();
      BVFeatureQuery query = new BVFeatureQuery();
      Vector3d dist = new Vector3d ();

      for (FemNode3d n : face.getNodes ()) {
         if (n.isDynamic ()) {
            continue; // node has already been excluded from attachment (e.g.
            // nodes near the lips)
         }
         query.nearestFaceToPoint (proj, coords, mesh, n.getPosition ());
         dist.sub (n.getPosition (), proj);
         if (dist.norm () < 1e-3) {
            proj.inverseTransform (mesh.getMeshToWorld ());
            mech.attachPoint (n, jaw, proj);
            RenderProps.setPointSize (n, 5);
            RenderProps.setPointColor (n, Color.RED);
         }
      }
   }

   public static void fixInnerNodes (FemModel3d face) {
      for (FemNode3d n : getInnerNodes (face)) {
         n.setDynamic (false);
         RenderProps.setPointSize (n, 5);RenderProps.setVisible (n, true);
         RenderProps.setPointColor (n, Color.GREEN);
      }
   }

   public static void fixOuterNodes (FemModel3d face) {
      for (FemNode3d n : getOuterNodes (face)) {
         n.setDynamic (false);
         RenderProps.setPointSize (n, 5);
         RenderProps.setPointColor (n, Color.BLUE);
      }
   }

   public static FemNode3d[] getInnerNodes (FemModel3d face) {
      return getNodes (face, faceGeometryDir + "atlas_internal.node");
   }

   public static FemNode3d[] getOuterNodes (FemModel3d face) {
      return getNodes (face, faceGeometryDir + "atlas_external.node");
   }

   public static FemNode3d[] getNodes (FemModel3d fem, String nodeListFileName) {
      ArrayList<FemNode3d> extNodes = new ArrayList<FemNode3d> ();
      try {
         Integer[] extNodeIdxs =
         AnsysReader.readNodeIdxs (new FileReader (nodeListFileName));
         for (int i = 0; i < extNodeIdxs.length; i++) {
            FemNode3d n = fem.getByNumber (extNodeIdxs[i]);
            if (n != null)
               extNodes.add (n);
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      return extNodes.toArray (new FemNode3d[0]);
   }

   public void writeFaceNodes (String fileName) {
      AnsysWriter.writeNodeFile (
         (FemModel3d)mech.models ().get (0), faceGeometryDir + fileName);
   }

   public void addCondylePoint () {
      RigidBody jaw = mech.rigidBodies ().get ("badinjaw");
      FrameMarker tmj = new FrameMarker ("tmj");
      mech.addFrameMarker (tmj, jaw, new Point3d (0.13, 0, 0.13));
   }

   double jawHingeAngle = 0.0;
   RigidTransform3d rot = new RigidTransform3d ();
   RigidTransform3d tmjTrans = new RigidTransform3d ();
   RigidTransform3d XJawHinge = new RigidTransform3d ();

   public void setJawHinge (double angle) {
      jawHingeAngle = angle;

      Point3d tmjPos = mech.frameMarkers ().get ("tmj").getPosition ();

      tmjTrans.p.negate (tmjPos);
      rot.R.setAxisAngle (0, -1, 0, Math.toRadians (angle));

      XJawHinge.mul (rot, tmjTrans);
      tmjTrans.p.set (tmjPos);
      XJawHinge.mul (tmjTrans, XJawHinge);

      RigidBody jaw = mech.rigidBodies ().get ("badinjaw");
      jaw.setPose (XJawHinge);
   }

   public double getJawHinge () {

      return jawHingeAngle;
   }

   public void setupRenderProps () {
      RenderProps.setFaceStyle (mech, FaceStyle.FRONT_AND_BACK);

      for (RigidBody body : mech.rigidBodies ()) {
         if (body.getName () == null)
            continue;
         else if (body.getName ().startsWith ("badin"))
            RenderProps.setFaceColor (body, new Color (0.4f, 0.6f, 0.8f));
         else if (body.getName ().equals ("badinskin"))
            RenderProps.setFaceColor (body, Color.LIGHT_GRAY);
         else if (body.getName ().startsWith ("badinskull")
         || body.getName ().startsWith ("amira"))
            RenderProps.setFaceColor (body, new Color (0f, 0.7f, 0f));
         else if (body.getName ().startsWith ("badin"))
            RenderProps.setFaceColor (body, new Color (0f, 0f, 1f));
      }

      for (Model mod : mech.models ()) {
         if (mod.getName () == null || !(mod instanceof Renderable)) {
            continue;
         }
         else if (mod.getName ().equals ("badinface")) {
            RenderProps.setFaceColor ((Renderable)mod, new Color (
               0.8f, 0.6f, 0.4f));
            if (mod instanceof FemMuscleModel) {
               FemMuscleModel face = (FemMuscleModel)mod;
               RenderProps.setLineWidth (face.getMuscleBundles (), 2);
               RenderProps.setPointStyle (face.markers (), PointStyle.SPHERE);
               RenderProps.setPointRadius (face.markers (), 0.0002);
               RenderProps.setPointColor (face.markers (), Color.LIGHT_GRAY);
               RenderProps.setVisible (face.getNodes (), false);
               face.setElementWidgetSize (0);
            }
         }
         else if (mod.getName ().equals ("badintongue"))
            RenderProps.setFaceColor ((Renderable)mod, new Color (
               0.8f, 0.5f, 0.5f));
      }

      String[] visible = new String[]{"badinjaw", "badinmaxilla"};
      for (RigidBody r : mech.rigidBodies ()) {
         RenderProps.setVisible (r, false);
      }
      for (String name : visible) {
         RenderProps.setVisible(mech.rigidBodies ().get (name), true);
      }

      face.setElementWidgetSize(1);

      for (FemNode3d n : face.getNodes()) {
         RenderProps.setVisibleMode(n, PropertyMode.Inherited);
      }
      RenderProps.setVisible(face.getNodes(), false);
      RenderProps.setPointRadius(face.getNodes(), 0.0005);
      RenderProps.setPointStyle(face.getNodes(), PointStyle.SPHERE);

      if (drawContactNormals) {
         CollisionManager collisions = mech.getCollisionManager();
         collisions.setContactNormalLen(-0.002);
         collisions.setDrawContactNormals(true);
         RenderProps.setLineColor(collisions, new Color(0.2f,0.2f,0.5f));
         RenderProps.setVisible(collisions, true);
      }
   }

   public static String stripSuffix (String str) {
      String[] substr = str.split ("_");
      return substr[0];
   }

   public static ControlPanel createVisibilityPanel (RootModel root, MechModel mech) {

      if (mech == null)
         return null;
      ControlPanel panel = new ControlPanel ("Show");
      for (Model mod : mech.models ()) {
         panel.addWidget (
            stripSuffix (mod.getName ()), mod, "renderProps.visible");
      }
      for (RigidBody body : mech.rigidBodies ()) {
         panel.addWidget (
            stripSuffix (body.getName ()), body, "renderProps.visible");
      }

      root.addControlPanel (panel);
      return panel;
   }

   public static void selLL(FemMuscleModel face) {

      try {
         scanAnsysContactSurfaceQuads(face, faceGeometryDir+"lower_lip_surface_Ian.elem.txt");
      } catch (FileNotFoundException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public static void selUL(FemMuscleModel face) {

      try {
         scanAnsysContactSurfaceQuads(face, faceGeometryDir+"upper_lip_surface_Ian.elem.txt");
      } catch (FileNotFoundException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public void enableLipSkullContact() {
      mech.setCollisionBehavior(face, mech.rigidBodies().get("badinjaw"), true);
      mech.setCollisionBehavior(face, mech.rigidBodies().get("badinmaxilla"), true);
   } 

   public void enableLipLipContact() {
      enableLipLipContact(mech, face, leftSideOnly);
   }

   public static void enableLipLipContact(MechModel mech, FemMuscleModel face, boolean leftSideOnly) {

      PolygonalMesh lowerlipMesh, upperlipMesh;
      try {
         if (leftSideOnly) {
            face.addMeshComp (
               face.scanMesh(faceGeometryDir + "lowerlip_leftside.smesh"));
            face.addMeshComp (
               face.scanMesh(faceGeometryDir + "upperlip_leftside.smesh"));
         }
         else {
//            lowerlipMesh = face.scanMesh(faceGeometryDir + "badinface_lowerlip_filled.smesh");
//            upperlipMesh = face.scanMesh(faceGeometryDir + "badinface_upperlip_filled.smesh");
            // these collision surfaces do not share vertices, and do not cause perturbed pivots with Pardiso
            upperlipMesh = (PolygonalMesh)GenericMeshReader.readMesh (faceGeometryDir + "face_collisionSurf_upperLip_reduced2.ply");
            lowerlipMesh = (PolygonalMesh)GenericMeshReader.readMesh (faceGeometryDir + "face_collisionSurf_lowerLip_reduced.ply");
            face.addMesh("lowerlip", lowerlipMesh);
            face.addMesh("upperlip", upperlipMesh);
         }


      } catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      mech.setCollisionBehavior(face, face, true);
   }

   public static void scanAnsysContactSurfaceQuads(FemMuscleModel face, String fileName)
   throws FileNotFoundException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(fileName));
      int[] idxs = new int[4];
      FemNode3d[] nodes = new FemNode3d[4];
      LinkedList<FemElement3d> contactElems = new LinkedList<FemElement3d>();
      LinkedList<FemElement3d> neighborElems = new LinkedList<FemElement3d>();

      try {
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            rtok.pushBack();
            neighborElems.clear();
            for (int i = 0; i < 4; i++) {
               idxs[i] = rtok.scanInteger();
               nodes[i] = face.getByNumber(idxs[i]);
               if (nodes[i] == null) break;
               //	       RenderProps.setVisible(nodes[i], true);
               neighborElems.addAll(face.getElementNeighbors(nodes[i]));
               for (FemElement3d e : neighborElems) {
                  if (e.containsFace(nodes[0], nodes[1], nodes[2], nodes[3])) {
                     contactElems.add(e);
                     break;
                  }
               }
            }
         }
      } catch (IOException e) {
         e.printStackTrace();
      }

      Main.getMain().getSelectionManager().clearSelections();
      for (FemElement3d e : contactElems) {
         Main.getMain().getSelectionManager().addSelected(e);
      }
   }



   public static void showAllLayers(FemMuscleModel face) {
      RenderProps.setVisible(face.getElements(), true);
      for (FemElement3d e : face.getElements()) {
         RenderProps.setVisibleMode(e, PropertyMode.Inherited);
      }
   }


   public void showsup () {
      showSupLayer(face);
   }

   public void showmid () {
      showMiddleLayer(face);
   }

   public void showdeep () {
      showDeepLayer(face);
   }

   public void setIntraoralPressure(double p) {
      if (intraoralNodes == null) {
         intraoralNodes = getIntraoralNodes();
      }
      double forcePerNode = p*getIntraoralSurfaceArea()/intraoralNodes.size();

      if (p == 0) {
         clearIntraoralPressure();
      } else {
         for (int i = 0; i < intraoralNodes.size(); i++) {
            FemNode3d n = intraoralNodes.get(i);
            face.getSurfaceVertex(n).computeNormal(tmp);
            tmp.scale(-forcePerNode);
            n.setExternalForce(tmp);
         }
      }
      myIntraoralPressure = p;
   }

   public double getIntraoralPressure() {
      return myIntraoralPressure;
   }

   public void clearIntraoralPressure() {
      for (int i = 0; i < intraoralNodes.size(); i++) {
         intraoralNodes.get(i).setExternalForce(Vector3d.ZERO);
      }
   }

   Vector3d tmp = new Vector3d();
   ArrayList<FemNode3d> intraoralNodes = null;
   ArrayList<Face> intraoralFaces = null;

   public ArrayList<FemNode3d> getIntraoralNodes() {
      ArrayList<FemNode3d> innerDynamicNodes = new ArrayList<FemNode3d>();
      if (origModel == true)
      {
         for (FemNode3d n : getInnerNodes(face)) {
            if (n.isActive() && !n.isAttached()) {
               innerDynamicNodes.add(n);
            }
         }
      }
      else
      {
         for (Integer ni : readIntList (faceGeometryDir + "cheek_area_nodes.nodenum")) {
            FemNode3d n = face.getByNumber(ni);
            if (n.isActive() && !n.isAttached()) {
               innerDynamicNodes.add(n);
            }
         }
      }
      return innerDynamicNodes;
   }

   public void showIntraoralNodes() {
      if (intraoralNodes == null) {
         intraoralNodes = getIntraoralNodes();
      }

      for (FemNode3d n : intraoralNodes) {
         RenderProps.setVisible(n, true);
         RenderProps.setPointColor(n, Color.CYAN.darker());

      }
   }

   public ArrayList<Face> getIntraoralFaces() {
      if (intraoralNodes == null) {
         intraoralNodes = getIntraoralNodes();
      }
      //      PolygonalMesh faceMesh = face.getSurfaceMesh();
      ArrayList<Face> faces = new ArrayList<Face>();

      //      ArrayList<FemMeshVertex> vertices = new ArrayList<FemMeshVertex>(intraoralNodes.length);
      //      FemMeshVertex[] vertices = new FemMeshVertex[intraoralNodes.length];
      for (FemNode3d n : intraoralNodes) {
         Vertex3d v = face.getSurfaceVertex(n);
         Iterator<HalfEdge> itr = v.getIncidentHalfEdges();
         while (itr.hasNext()) {
            Face f = itr.next().getFace();
            if (!faces.contains(f)) {
               faces.add(f);
            }
         }
      }

      return faces;
   }

   public double getIntraoralSurfaceArea() {
      if (intraoralFaces == null) {
         intraoralFaces = getIntraoralFaces();
      }
      double area = 0;
      for (int i = 0; i < intraoralFaces.size(); i++) {
         area += intraoralFaces.get(i).computeArea();
      }
      return area;
   }

   public void showIntraoralFaces() {
      if (intraoralFaces == null) {
         intraoralFaces = getIntraoralFaces();
      }

      ArrayList<Face> toRemove = new ArrayList<Face>();
      PolygonalMesh mesh = face.getSurfaceMesh();
      for (Face face : mesh.getFaces())  {
         if (!intraoralFaces.contains(face)) {
            toRemove.add(face);
         }
      }

      for (Face face : toRemove) {
         mesh.removeFace(face);
      }
   }

   public static void showSupLayer (FemMuscleModel face) {
      RenderProps.setVisible(face.getElements(), true);
      for (FemElement3d e : face.getElements()) {
         RenderProps.setVisible(e, false);
      }
      for (FemNode3d n : getOuterNodes (face)) {
         for (FemElement3d e : face.getElementNeighbors(n)) {
            RenderProps.setVisibleMode(e, PropertyMode.Inherited);
         }
      }
   }

   public static void showDeepLayer(FemMuscleModel face) {
      RenderProps.setVisible(face.getElements(), true);
      for (FemElement3d e : face.getElements()) {
         RenderProps.setVisible(e, false);
      }
      for (FemNode3d n : getInnerNodes (face)) {
         for (FemElement3d e : face.getElementNeighbors(n)) {
            RenderProps.setVisibleMode(e, PropertyMode.Inherited);
         }
      }
   }

   public static void showMiddleLayer(FemMuscleModel face) {
      RenderProps.setVisible(face.getElements(), true);
      for (FemNode3d n : getInnerNodes (face)) {
         for (FemElement3d e : face.getElementNeighbors(n)) {
            RenderProps.setVisible(e, false);
         }
      }
      for (FemNode3d n : getOuterNodes (face)) {
         for (FemElement3d e : face.getElementNeighbors(n)) {
            RenderProps.setVisible(e, false);
         }
      }
   }

   //   public enum FaceLayer {
   //      INNER,
   //      MIDDLE,
   //      OUTER
   //   };
   //   
   //   public LinkedList<FemElement3d> getLayer(FaceLayer layer) {
   //      LinkedList<FemElement3d> elems = new LinkedList<FemElement3d>();
   //      
   //      return elems;
   //   }

   public static ControlPanel createMusclePanel(RootModel root,
      FemMuscleModel fem) {
      ControlPanel controlPanel = new ControlPanel(fem.getName() + " Muscles",
      "LiveUpdate");
      controlPanel.setScrollable(true);
      FemControlPanel.addBundlesToPanel(controlPanel, fem, /*reset colors=*/false);

      root.addControlPanel(controlPanel);
      return controlPanel;
   }


   ControlPanel options;
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      
      if (myControlPanels.size() == 0) {
         createVisibilityPanel(this, mech);
         options = FemControlPanel.createControlPanel(this, face, mech);
         createMusclePanel(this, face);
      }
      addWayPoints(this, 1d, 0.01);
      addMuscleProbes(this, face, 1d, "OOP");

   }

   public static void addWayPoints(RootModel root, double duration,
      double waypointstep) {
      root.removeAllWayPoints();
      for (int i = 1; i < duration / waypointstep; i++) {
         root.addWayPoint(i * waypointstep);
      }
      root.addBreakPoint(duration);
   }

   public void fastfwd() {
      WayPoint lastValid = myWayPoints.getLastValid();
      if (lastValid != null) {
         Main.getMain().getScheduler().reset(lastValid);
      }
   }


   public static void addMuscleProbes(RootModel root, FemMuscleModel face,
      double duration, String muscleName) {
      addMuscleProbes(root, face, duration, new String[]{muscleName});
   }

   public static void addMuscleProbes(RootModel root, FemMuscleModel face,
      double duration, String[] muscleNames) {
      for (String name : muscleNames) {
         Property prop;
         if ((prop = face.getProperty("exciters/" + name + ":excitation")) == null) {
            if ((prop = face.getProperty("bundles/" + name + ":excitation")) == null) {
               System.err.println("addMuscleProbe -- cannot find muscle "+name);
               return;
            }
         }

         NumericInputProbe oop = new NumericInputProbe(prop, face);
         oop.setStartStopTimes(0, duration);
         oop.setName(name);
         oop.setAttachedFileName(name.toLowerCase()+".txt");
         try {
            oop.load();
         } catch (IOException e) {
         }
         root.addInputProbe(oop);
      }
   }

   public static void showCollisions(MechModel mech, boolean visible) {
      CollisionManager collisions = mech.getCollisionManager();
      RenderProps.setVisible(collisions, visible);
      RenderProps.setEdgeWidth (collisions, 2);      
      RenderProps.setEdgeColor (collisions, new Color (0f, 0.4f, 0.4f));
      RenderProps.setLineWidth (collisions, 2);      
      RenderProps.setLineColor (collisions, new Color (0f, 0f, 0.5f));
      collisions.setContactNormalLen (-0.005);
      collisions.setDrawContactNormals(true);
   }

   public static void setNodesNonDynamicByNumber (FemModel3d fem, String nodeIndicesFilename) {
      for (Integer ni : readIntList (faceGeometryDir + nodeIndicesFilename)) {
         FemNode3d n = fem.getByNumber(ni);
         if (n != null) {
            n.setDynamic (false);
            RenderProps.setPointColor (n, Color.GREEN);
         }
      }
   }
   
   public void setOrigModel()
   {
      //Use the internal muscles and probes of original face model
      workingDir = ArtisynthPath.getSrcRelativePath(BadinFaceDemo.class, "data/");
      muscleNodesFile   = faceGeometryDir + "face_muscles_yohan.node";
      muscleConnectFile = faceGeometryDir + "face_muscles_topology.mac";
      faceMuscleNames = new String[] {
                                      "DAO", // depressor anguli oris
                                      "BUC", // buccinator
                                      "DLI", // depressor labii inferioris
                                      "MENT", // mentalis
                                      "MAS", // masseter (not used)
                                      "OOM", // obicularis oris middle
                                      "OOP", // obicularis oris peripheral
                                      "LLSAN", // levator labii superioris alaeque nasi
                                      "LAO", // levator anguli oris
                                      "RIS", // risorius
                                      "ZYG" // zygomatic
      };
   }

}
