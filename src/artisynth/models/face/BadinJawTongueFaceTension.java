package artisynth.models.face;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Locale;
import java.util.Scanner;
import java.util.ArrayList;

import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyMode;
import maspack.render.GL.GLViewer;
import maspack.render.RenderProps;
import maspack.render.Renderer.Shading;
import artisynth.core.driver.Main;
import artisynth.core.driver.ViewerManager;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.FungMaterial;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.jawTongue.JawHyoidFemMuscleTongue;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.femmodels.MuscleElementDesc;

public class BadinJawTongueFaceTension extends JawHyoidFemMuscleTongue {

   public static final double scaleFactor   = 1.0d;
   public static final double timeToStretch = .06; // s
   public static final double timeToOpenJaw = .07;
   public static final double openTime      = 2.0; 
   public static final double timeToFinish  = timeToOpenJaw + openTime + 0.01; 
   public static double muscleThickness = 6;

   boolean collideTongueFace = false;
   boolean openJaw           = false;
   boolean smile             = true;
   boolean scar              = false;
   public static boolean contactEnabled = false;
   public static boolean faceMusclesActivated = false;

   public static final String dataDir =
      ArtisynthPath.getSrcRelativePath (BadinJawTongueFaceTension.class, "data/");
   public static final String resultsDir =
      ArtisynthPath.getSrcRelativePath (BadinJawTongueFaceTension.class, "results/");
   public static final String geometryDir =
      ArtisynthPath.getSrcRelativePath (BadinJawTongueFaceTension.class, "geometry/");

   int listLength = readIntList (geometryDir + "internal_face_nodes.nodenum").length; // + 
   //      readIntList (geometryDir + "face_jaw_attachments.txt").length;
   double[][] reactionForce = new double[listLength][3]; 

   FemMuscleModel face;

   public BadinJawTongueFaceTension () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      face = createScaledFace();
      myJawModel.addModel (face);

      face.setStiffnessDamping(0.02);
      face.setParticleDamping(80);

      try {
         //         readRSTLFile(dataDir, "langer_rotMat_badin");
         readRSTLFile(dataDir, "rstl_rotMat_badin_old");
      } catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

//    for (FemNode3d n : BadinFaceDemo.getInnerNodes (face)) {
      for (FemNode3d n : BadinFaceDemo.getNodes(face, geometryDir + "nodes_to_displace.node")) {
         n.setDynamic(false);
         RenderProps.setPointColor (n, Color.CYAN);
      }
      RigidBody hyoid = myJawModel.rigidBodies ().get ("hyoid");
      hyoid.setDynamic (false);

      RigidBody jaw = myJawModel.rigidBodies().get("jaw");
//      RigidBody maxilla = myJawModel.rigidBodies().get("maxilla");
      RigidBody cranium = myJawModel.rigidBodies().get("cranium");

//      myJawModel.removeRigidBody(maxilla);
      
//      replaceMesh(cranium, BadinFaceDemo.faceGeometryDir + "badinskullEditChoppedTeeth.obj");
//      replaceMesh(maxilla, BadinFaceDemo.faceGeometryDir + "badinmaxilla.obj");
      BadinJawFaceDemo.transformAndReplaceMesh(jaw, BadinFaceDemo.faceGeometryDir + "badinjaw.obj");

      if (smile) { 
        BadinFaceDemo.addMuscles(face, BadinFaceDemo.faceGeometryDir + "face_muscles_flynn.node", m2mm, BadinFaceDemo.midSagittalPlane);
//        BadinFaceDemo.addMuscles(face, BadinFaceDemo.faceGeometryDir + "face_muscles_flynn_scaled010.node", m2mm, BadinFaceDemo.midSagittalPlane);
      
        for (MuscleBundle b : face.getMuscleBundles()) {
            b.setFibresActive(false);
        }
        setDefaultOOP();
      }
      face.setGravity(Vector3d.ZERO);

      setFaceRenderProps (face);
      MyMonitor StressFieldMonitor = new MyMonitor();
      addMonitor(StressFieldMonitor);

      myJawModel.setStabilization (PosStabilization.GlobalStiffness);
//      System.out.println(myJawModel.getPenetrationTol());
//      myJawModel.setPenetrationTol(0.01d); 
//      System.out.println("Hello there " + myJawModel.getPenetrationTol());
    
      
   }

   public void addNodeProbe(FemNode3d node, double duration, double interval, String landmarkName) {

      NumericOutputProbe p = new NumericOutputProbe (node, "position", 0.d, duration, 0.1d);
      p.setName(landmarkName);
      p.setAttachedFileName(resultsDir + landmarkName + "_output.txt");
//      p.setDefaultDisplayRange(-0.1, 0.1);
//      p.setActive(true);
      addOutputProbe(p);
      
      RenderProps.setPointColor(node,Color.GREEN);

   }

   public FemMuscleModel createScaledFace () {
      FemMuscleModel face;

      face = BadinFaceDemo.createFace(BadinFaceDemo.faceGeometryDir,
         "badinface_adjusted", 1d / scaleFactor, /*linearMaterial=*/false);
//      "badinface_oop_csa_midsagittal", 1d / scaleFactor, /*linearMaterial=*/false);

      MooneyRivlinMaterial mrmat = (MooneyRivlinMaterial) face.getMaterial();
      mrmat.setC10(400); // Pa  Taken from Barbarino et al (2011) for SMAS and superficial fat
      mrmat.setC20(1400); // Pa
      mrmat.setBulkModulus(50000); // Pa

      for (FemNode3d n : BadinFaceDemo.getOuterNodes (face)) {
         for (FemElement3d e : face.getElementNeighbors(n)) {
          // Parameters estimated from avg foredata params using end_index=35
          e.setMaterial (new FungMaterial (17773d,5931d,5931d,1000d, 1000d, 1000d, 
             1000d, 1000d, 1000d, 3784d, 250000));
// younger skin parameters
//            e.setMaterial (new FungMaterial (0.5*17773d,0.5*5931d,0.5*5931d,0.5*1000d, 0.5*1000d, 0.5*1000d, 
//               0.5*1000d, 0.5*1000d, 0.5*1000d, 0.5*3784d, 250000));
// Older skin parameters
//            e.setMaterial (new FungMaterial (2.0*17773d,2.0*5931d,2.0*5931d,2.0*1000d, 2.0*1000d, 2.0*1000d, 
//               2.0*1000d, 2.0*1000d, 2.0*1000d, 2.0*3784d, 250000));
            
        }
      }
//      
      face.scaleDistance (m2mm);

      return face;
   }

   public void setFaceRenderProps (FemModel3d face) {
      RenderProps.setVisible (face.getNodes (), true);
      RenderProps.setVisible (face.getElements (), true);
      RenderProps.setVisible (face.markers (), false);
      RenderProps.setLineWidth (face, 1);

      face.setSurfaceRendering (SurfaceRender.Stress);

      RenderProps.setFaceColor(face, new Color(0.9f, 0.8f, 0.75f));
      if (scar){
         for (Integer ei : readIntList (geometryDir + "jawScarElements90.txt")) {
            FemElement3d escar = face.getElementByNumber(ei);
            if (escar != null) {
               RenderProps.setFaceColor (escar, Color.PINK);
            }
         }
      }
      RenderProps.setShading (myJawModel, Shading.FLAT);

      RenderProps.setLineWidth(face.getElements(), 1);
      RenderProps.setLineColor(face, new Color(.2f, .2f, .2f));
      // face.setSubSurfaceRendering(false);

      for (RigidBody body : myJawModel.rigidBodies()) {
         RenderProps.setFaceColor(body, new Color(0.55f, 0.55f, 0.65f));
         RenderProps.setVisible(body, true);
      }

      for (FemNode3d n : face.getNodes()) {
         RenderProps.setVisibleMode(n, PropertyMode.Inherited);
      }

      for (RigidBody rb : myJawModel.rigidBodies()) {
         rb.setAxisLength(0);
      }

      RenderProps.setVisible(myJawModel.getCollisionManager(), false);
//      for (FemNode3d n : BadinFaceDemo.getInnerNodes (face)) {
//         RenderProps.setVisible (n, true);
//      }

   }

   @Override
   public void attach(DriverInterface driver) {
      // TODO Auto-generated method stub
      super.attach(driver);

      ControlPanel panel;
      panel = FemControlPanel.createControlPanel(this, face, myJawModel);
      panel.setName("Face Controls");

      GLViewer v = driver.getViewer();
      v.setGridVisible (false);

      addWayPoints(this, timeToFinish, 0.2);
      //      this.addBreakPoint(timeToStretch / 1e9);

      MyController mycon = new MyController(myJawModel, "stretch skin");
      addController(mycon);
      if (openJaw)
         addOpenJawProbe(this, myJawModel, 0.0d, timeToFinish, "bi_open");

      if (smile) {
      //    String[] purseMuscles = new String[] {"OOP", "OOM", "BUC"};
//    String[] poutMuscles = new String[] {"DAO", "DLI", "MENT"};
        String[] smileMuscles = new String[] {"ZYG", "LAO", "RIS", "LLSAN", "OOPu", "OOPl", "OOM", "BUC", "MENT", "DAO", "DLI"};
//    String[] openSmileMuscles = new String[] {"ZYG", "LAO", "RIS", "LLSAN", "DLI", "DAO"};
        addFaceMuscleProbes(Main.getMain().getRootModel(), face, 0.0d, timeToFinish, smileMuscles);
      }
      
      FrameMarker incisorMarker = myJawModel.frameMarkers().get("lowerincisor");
      RenderProps.setVisible(incisorMarker, true);
      addIncisorProbe(timeToFinish, 0.1d, "displacement");
      
      FemNode3d node = face.getNode(2953);
      addNodeProbe(node, timeToFinish, 0.1d, "pg");

      node = face.getNode(2943);
      addNodeProbe(node, timeToFinish, 0.1d, "sl");

      node = face.getNode(6384);
      addNodeProbe(node, timeToFinish, 0.1d, "rmc");

      node = face.getNode(1996);
      addNodeProbe(node, timeToFinish, 0.1d, "lmc");

      node = face.getNode(6044);
      addNodeProbe(node, timeToFinish, 0.1d, "rml");

      node = face.getNode(1637);
      addNodeProbe(node, timeToFinish, 0.1d, "lml");

      node = face.getNode(6072);
      addNodeProbe(node, timeToFinish, 0.1d, "rmu");

      node = face.getNode(1665);
      addNodeProbe(node, timeToFinish, 0.1d, "lmu");

      node = face.getNode(1727);
      addNodeProbe(node, timeToFinish, 0.1d, "ph");

      node = face.getNode(6897);
      addNodeProbe(node, timeToFinish, 0.1d, "ran");

      node = face.getNode(2510);
      addNodeProbe(node, timeToFinish, 0.1d, "lan");

      node = face.getNode(1224);
      addNodeProbe(node, timeToFinish, 0.1d, "prn");

      node = face.getNode(1260);
      addNodeProbe(node, timeToFinish, 0.1d, "cn");

      node = face.getNode(5708);
      addNodeProbe(node, timeToFinish, 0.1d, "ror");

      node = face.getNode(1300);
      addNodeProbe(node, timeToFinish, 0.1d, "lor");

      node = face.getNode(5556);
      addNodeProbe(node, timeToFinish, 0.1d, "rtr");

      node = face.getNode(1128);
      addNodeProbe(node, timeToFinish, 0.1d, "ltr");

      node = face.getNode(5793);
      addNodeProbe(node, timeToFinish, 0.1d, "rle");

      node = face.getNode(1386);
      addNodeProbe(node, timeToFinish, 0.1d, "lle");

      node = face.getNode(5814);
      addNodeProbe(node, timeToFinish, 0.1d, "rc");

      node = face.getNode(1407);
      addNodeProbe(node, timeToFinish, 0.1d, "lc");

      node = face.getNode(5804);
      addNodeProbe(node, timeToFinish, 0.1d, "rue");

      node = face.getNode(1397);
      addNodeProbe(node, timeToFinish, 0.1d, "lue");

      node = face.getNode(4722);
      addNodeProbe(node, timeToFinish, 0.1d, "rbp");

      node = face.getNode(245);
      addNodeProbe(node, timeToFinish, 0.1d, "lbp");

      node = face.getNode(2922);
      addNodeProbe(node, timeToFinish, 0.1d, "n");
      
      node = face.getNode(1208);
      addNodeProbe(node, timeToFinish, 0.1d, "sn");

      node = face.getNode(6733);
      addNodeProbe(node, timeToFinish, 0.1d, "j1");

      node = face.getNode(5885);
      addNodeProbe(node, timeToFinish, 0.1d, "j2");
      
      node = face.getNode(7084);
      addNodeProbe(node, timeToFinish, 0.1d, "j3");

      if (scar){
         MyStiffnessController mystiffcon = new MyStiffnessController(myJawModel, "create scar");
         addController(mystiffcon);
      }


   }

   public static void addFaceMuscleProbes(RootModel root, FemMuscleModel face,
      double startTime, double stopTime, String[] muscleNames) {

      for (String name : muscleNames) {
         Property prop;
         if ((prop = face.getProperty("exciters/" + name + ":excitation")) == null) {
            if ((prop = face.getProperty("bundles/" + name + ":excitation")) == null) {
               System.err.println("addMuscleProbe -- cannot find muscle "+name);
               return;
            }
         }

         NumericInputProbe oop = new NumericInputProbe(prop, face);
         oop.setStartStopTimes(startTime, stopTime);
         oop.setName(name);
         //         oop.setAttachedFileName(dataDir+name.toLowerCase()+".txt");
         oop.addData(new double[] { 0, 0, timeToOpenJaw, 0, timeToOpenJaw + openTime, 
		   0.0, timeToFinish, 0.0 },NumericInputProbe.EXPLICIT_TIME);

         try {
            oop.load();
         } catch (IOException e) {
         }
         root.addInputProbe(oop);
      }
      
   }


   public static void addOpenJawProbe(RootModel root, MechModel jaw,
      double startTime, double stopTime, String muscleName) {
      addMuscleProbes(root, jaw, startTime, stopTime, new String[]{muscleName});
   }

   public static void addMuscleProbes(RootModel root, MechModel jaw,
      double startTime, double stopTime, String[] muscleNames) {
      for (String name : muscleNames) {
         Property prop;
         if ((prop = jaw.getProperty("exciters/" + name + ":excitation")) == null) {
            if ((prop = jaw.getProperty("bundles/" + name + ":excitation")) == null) {
               System.err.println("addMuscleProbe -- cannot find muscle "+name);
               return;
            }
         }

         NumericInputProbe openJaw = new NumericInputProbe(prop, jaw);
         openJaw.setStartStopTimes(startTime, stopTime);
         openJaw.setName(name);
         openJaw.addData(new double[] { 0, 0, timeToOpenJaw, 0, timeToOpenJaw + 
		   openTime, 1d, timeToFinish, 1d },NumericInputProbe.EXPLICIT_TIME);

         try {
            openJaw.load();
         } catch (IOException e) {
         }
         root.addInputProbe(openJaw);
      }
   }

   public void readRSTLFile(String dataDir, String name) throws IOException {
      Scanner s = null;
      double[] rotMatComp = new double[9];

      String fileName = dataDir + name + ".txt";
      try {
         s = new Scanner(
            new BufferedReader(new FileReader(fileName)));
         s.useLocale(Locale.US);

         for (FemElement3d e : face.getElements()) {
            for (int i=0; i<9; i++) {
               rotMatComp[i] = s.nextDouble();
            }
            e.setFrame (new Matrix3d (rotMatComp));
         }
      }
      catch (Exception e) {
         System.out.println ("Error reading file " + fileName + ": "+e);
      } 
      finally {
         if (s != null) s.close();
      }
   }

   public class MyController extends ControllerBase {
      public MyController(MechModel m, String name) {

         setModel(m);
         setName(name);

      }

      public void apply(double t0, double t1) {

         if ( t0 < timeToStretch ) {
            Point3d finalPosition = new Point3d();
            double interpScaleFactor = 0.0;

//          for (FemNode3d n : BadinFaceDemo.getInnerNodes (face)) {
            for (FemNode3d n : BadinFaceDemo.getNodes(face, geometryDir + 
            "nodes_to_displace.node")) {

               interpScaleFactor = (1d + t1 * (scaleFactor - 1d) / timeToStretch) / 
                  (1d + t0 * (scaleFactor - 1d) / timeToStretch);

               finalPosition.scale(interpScaleFactor, n.getPosition());

               n.setPosition(finalPosition);
            }
         }
         else if (contactEnabled == false) {
            RigidBody jaw     = myJawModel.rigidBodies().get("jaw");
//            RigidBody maxilla = myJawModel.rigidBodies().get("maxilla");
            RigidBody cranium = myJawModel.rigidBodies().get("cranium");

            myJawModel.setCollisionBehavior(face, jaw, true);
//            myJawModel.setCollisionBehavior(face, maxilla, true);
            myJawModel.setCollisionBehavior(face, cranium, true);
            if (collideTongueFace) {
               myJawModel.setCollisionBehavior (face, tongue, true);
            }

            BadinFaceDemo.enableLipLipContact(myJawModel, face, false);
            //            enableLipSkullContact();
            BadinFaceDemo.showCollisions(myJawModel, true);
            contactEnabled = true;

            // Set all face nodes to dynamic
            for (FemNode3d n : face.getNodes()) {
               n.setDynamic (true);
               n.setRenderProps (null);
            }

            /* fix all inner-surface nose nodes, and zygomatic ligament nodes, and edge of domain nodes */
            setNodesNonDynamicByNumber(face, "edge_nodes.nodenum");
            setNodesNonDynamicByNumber(face, "nose_nodes_to_fix.txt");
            setNodesNonDynamicByNumber(face, "eye_nodes_to_fix.txt");
            setNodesNonDynamicByNumber(face, "zygomatic_ligament_attachments.txt");

            //            attachFaceToSkull (face);
//            for (FemNode3d n : BadinFaceDemo.getInnerNodes (face)) {
//               n.setDynamic (false);
//            }

            // attach face nodes to jaw from list of indices
            BadinFaceDemo.attachFaceToJaw (myJawModel, myJawModel.rigidBodies ().get (
               "jaw"), face, "mandibular_ligament_attachments.txt");
            //               "jaw"), face, "face_jaw_attachments.txt");

            //            BadinFaceDemo.setNodesDynamicByNumber (face, "submental_free_nodes.nodenum");
            //            BadinFaceDemo.setNodesDynamicByNumber (face, "zygomatic_free_nodes.nodenum");

            //            tongue.setGravity(0, 0, -9800);
            //            face.setGravity(0, 0, -9800);
            //            myJawModel.setGravity(0, 0, -9800);

            // Store the reaction force on the lip free nodes in order to release them gradually later
            int index = 0;
//          for (FemNode3d n : face.getNodes ()) {
            for (Integer ni : readIntList (geometryDir + "internal_face_nodes.nodenum")) {
               FemNode3d n = face.getByNumber(ni);

               // Store the node reaction force
               reactionForce[index][0] = n.getForce().x;
               reactionForce[index][1] = n.getForce().y;
               reactionForce[index][2] = n.getForce().z;
               index++;
            }

         }

         // Relax the reaction force on the lip free nodes and jaw nodes linearly to zero over time
         if ( t0 >= timeToStretch && t0 <= timeToOpenJaw ) {
            int index = 0;
            double scale = 0.0;
//            for (FemNode3d n : face.getNodes ()) {
             for (Integer ni : readIntList (geometryDir + "internal_face_nodes.nodenum")) {
                FemNode3d n = face.getByNumber(ni);

               // Apply interpolated force to node
               scale = -(1d - (t0 - timeToStretch ) / (timeToOpenJaw - timeToStretch));
               double[] externalForce = new double[] {reactionForce[index][0]*scale, reactionForce[index][1]*scale, reactionForce[index][2]*scale};
               n.setExternalForce(new Vector3d(externalForce[0], externalForce[1], externalForce[2]));
               index++;
            }
         }

//         // Add face muscles and probes
//         if (t0 >= timeToOpenJaw && !faceMusclesActivated && smile) {
//            BadinFaceDemo.addMuscles(face, BadinFaceDemo.faceGeometryDir + "face_muscles_yohan.node", m2mm, BadinFaceDemo.midSagittalPlane);
//            ((GenericMuscle)face.getMuscleMaterial()).setMaxStress(100000);
//            face.setDirectionRenderLen(0.002);
//            face.setElementWidgetSize(1);
//            //
//            for (MuscleBundle b : face.getMuscleBundles()) {
//               b.scaleDistance (m2mm);
//               //              b.setFibresActive(false);
//            }
//            ((GenericMuscle)face.getMuscleMaterial()).scaleDistance(m2mm);
//            faceMusclesActivated = true;

//            String[] smileMuscles = new String[] {"ZYG", "LAO", "RIS", "LLSAN"};
//            addFaceMuscleProbes(Main.getMain().getRootModel(), face, 0.0d, timeToFinish / 1e9, smileMuscles);
//         }

         // Add face muscles and probes
         if (t0 >= timeToStretch && !faceMusclesActivated && smile) {
//            BadinFaceDemo.addMuscles(face, BadinFaceDemo.faceGeometryDir + "face_muscles_flynn.node", m2mm, BadinFaceDemo.midSagittalPlane);
//
//           for (MuscleBundle b : face.getMuscleBundles()) {
//               b.setFibresActive(false);
//           }

           setMuscleElements(muscleThickness);
//           setMuscleThickness(face, "DAO", 7.d);
//           setDefaultOOP();

           ((GenericMuscle)face.getMuscleMaterial()).setMaxStress(100000);
           face.setDirectionRenderLen(0.5);
           face.setElementWidgetSize(1);
           RenderProps.setLineWidth(face.getElements(), 0);
           RenderProps.setLineColor(face, new Color(.2f, .2f, .2f));
           // face.setSubSurfaceRendering(false);
           RenderProps.setVisible(face.markers(), false);
           RenderProps.setVisible(face.getMuscleBundles(), true);

           for (MuscleBundle b : face.getMuscleBundles()) {
              b.scaleDistance (m2mm);
            }
           ((GenericMuscle)face.getMuscleMaterial()).scaleDistance(m2mm);

           // Attach mandible to nodes, which are close to where the Mentalis attaches to the mandible
           // Marker nos. 75, 58, 114051, 114196
           
//           attachNodesNearMuscleInsertionPoint(75, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(58, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(114051, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(114196, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           
//           // Attach end of DLI to mandible
//           attachNodesNearMuscleInsertionPoint(50, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(40, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(114041, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(114182, myJawModel.rigidBodies ().get ("jaw"), 3d);
//
           // Attach end of DAO to mandible
//           attachNodesNearMuscleInsertionPoint(19, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(0, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(114011, myJawModel.rigidBodies ().get ("jaw"), 3d);
//           attachNodesNearMuscleInsertionPoint(114205, myJawModel.rigidBodies ().get ("jaw"), 3d);

           // Attach end of LAO to maxilla
//           attachNodesNearMuscleInsertionPoint(166, myJawModel.rigidBodies ().get ("cranium"), 5d);
//           attachNodesNearMuscleInsertionPoint(156, myJawModel.rigidBodies ().get ("cranium"), 5d);
//           attachNodesNearMuscleInsertionPoint(114151, myJawModel.rigidBodies ().get ("cranium"), 5d);
//           attachNodesNearMuscleInsertionPoint(114221, myJawModel.rigidBodies ().get ("cranium"), 5d);
//
//           // Attach end of LLSAN to maxilla
//           attachNodesNearMuscleInsertionPoint(128, myJawModel.rigidBodies ().get ("cranium"), 5d);
//           attachNodesNearMuscleInsertionPoint(144, myJawModel.rigidBodies ().get ("cranium"), 5d);
//           attachNodesNearMuscleInsertionPoint(114141, myJawModel.rigidBodies ().get ("cranium"), 5d);
//           attachNodesNearMuscleInsertionPoint(114211, myJawModel.rigidBodies ().get ("cranium"), 5d);
//
//           // Attach end of ZYG to maxilla
//           attachNodesNearMuscleInsertionPoint(186, myJawModel.rigidBodies ().get ("cranium"), 5d);
//           attachNodesNearMuscleInsertionPoint(114171, myJawModel.rigidBodies ().get ("cranium"), 5d);

           faceMusclesActivated = true;
           

         }
         if (t0 >= timeToStretch )
            face.setGravity(0d, 0d, -9800.d);
         if (t0 >= timeToOpenJaw && myJawModel.getMaxStepSize() > 0.001)
            myJawModel.setMaxStepSize(0.001);


      }
   }

   public class MyStiffnessController extends ControllerBase {
      public MyStiffnessController(MechModel m, String name) {

         setModel(m);
         setName(name);

      }

      public void apply(double t0, double t1) {

         if ( t0 >= timeToStretch && t1 <= timeToOpenJaw) {
            double iSF  = 0.0;
            double iSF2 = 0.0;

            for (Integer ei : readIntList (geometryDir + "jawScarElements90.txt")) {
               FemElement3d escar = face.getElementByNumber(ei);
               if (escar != null) {
                  
                  iSF  = 1d + (t1 - timeToStretch) / (timeToOpenJaw - timeToStretch) * 9d;
                  iSF2 = 1d + (t1 - timeToStretch) / (timeToOpenJaw - timeToStretch) * 28.966d;

//                  escar.setMaterial (new MooneyRivlinMaterial ( iSF*2.5d, 0d, 0d, iSF*1.175, 0d,
//                     25d));
                  escar.setMaterial (new FungMaterial (iSF*17.773d,iSF2*5.931d,iSF2*5.931d,iSF*1.000d, iSF*1.000d, iSF*1.000d, 
                     iSF*1.000d, iSF*1.000d, iSF*1.000d, iSF*3.784d, 250.000));
                  
               }
            }

         }

      }
   }
   public void attachNodesNearMuscleInsertionPoint(int markerNumber, RigidBody rigidBody, double distance) {
      Point3d markerPosition = face.markers().getByNumber(markerNumber).getPosition();
      for (FemNode3d n : face.getNodes()) {
         if (n.getPosition().distance(markerPosition) < distance && !n.isAttached()){
            myJawModel.attachPoint(n, rigidBody);
            RenderProps.setPointSize(n, 5);
            RenderProps.setPointColor(n, Color.BLUE);
         }
      }
      
   }
   public void setMuscleElements(double muscleThickness) {
      setMuscleElements(face, muscleThickness);
   }
   public static void setMuscleElements(FemMuscleModel face,
      double muscleThickness) {
      for (MuscleBundle b : face.getMuscleBundles()) {
         b.clearElements();
         b.addElementsNearFibres(muscleThickness);
         b.computeElementDirections();
      }
   }

   public static void setMuscleThickness(FemMuscleModel face,
      String faceMuscle, double muscleThickness) {
      MuscleBundle b = face.getMuscleBundles().get(faceMuscle);
      b.clearElements();
      b.addElementsNearFibres(muscleThickness);
      b.computeElementDirections();
   }

//   /**
//    * Old method for creating OOP. See comments in setDefaultOOP(), below.
//    */
//   public void setDefaultOOPOld() {
//      // use 7M fiber directions
//      // add elements for 7th upper, 6th lower ring
//
//      MuscleBundle oop = face.getMuscleBundles().get("OOP");
//      System.out.println ("num elems=" + oop.getElements().size());
//      
//      loadoop(oop, "7M");
//      oop.clearElements();
//
//      MuscleExciter ex = new MuscleExciter("OOPu");
//      face.addMuscleExciter(ex);
//      addToExciter(ex, addoop("7Du"));
//      addToExciter(ex, addoop("7Mu"));
//      addToExciter(ex, addoop("7Su"));
//
//      ex = new MuscleExciter("OOPl");
//      face.addMuscleExciter(ex);
//      addToExciter(ex, addoop("6Dl"));
//      addToExciter(ex, addoop("6Ml"));
//      addToExciter(ex, addoop("6Sl"));
//
//      oop.setElementWidgetSize(1);
//      RenderProps.setVisible(oop, true);
//   }
   
   /**
    * John Lloyd, Jul 13, 2019.
    * 
    * New method for default OOP. Creates two new muscle groups, OOPu
    * and OOPl, for separate upper and lower excitation. The original 
    * method created a single group, OOP, and controlled subelements
    * within this using separate exciters, something which is not
    * longer supported.
    */
   public void setDefaultOOP() {
      // use 7M fiber directions
      // add elements for 7th upper, 6th lower ring
      MuscleBundle oopu = new MuscleBundle ("OOPu");
      face.addMuscleBundle (oopu);
      MuscleExciter exu = new MuscleExciter("OOPu");
      face.addMuscleExciter(exu);
      exu.addTarget (oopu);
      
      loadoop(oopu, "7M"); // temporarily load 7M elems just to create fibers 
      oopu.clearElements();     
      addoop (oopu, "7Du");
      addoop (oopu, "7Mu");
      addoop (oopu, "7Su");
      //RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, oopu);
      oopu.computeElementDirections();
      
      MuscleBundle oopl = new MuscleBundle ("OOPl");
      face.addMuscleBundle (oopl);
      MuscleExciter exl = new MuscleExciter("OOPl");
      face.addMuscleExciter(exl);
      exl.addTarget (oopl);
      
      loadoop(oopl, "7M"); // temporarily load 7M elems just to create fibers 
      oopl.clearElements();          
      addoop (oopl, "6Dl");
      addoop (oopl, "6Ml");
      addoop (oopl, "6Sl");
      //RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, oopl);
      oopl.computeElementDirections();

      oopu.setElementWidgetSize(1);
      RenderProps.setVisible(oopu, true);
      oopl.setElementWidgetSize(1);
      RenderProps.setVisible(oopl, true);
   }

//   public void addToExciter(MuscleExciter ex, ArrayList<MuscleElementDesc> elemsToAdd) {
//      for (MuscleElementDesc desc : elemsToAdd) {
//         ex.addTarget(desc, 1);
//      }
//   }
   
   public void addoop(MuscleBundle mus, String name) { 
      RefFemMuscleFaceDemo.addMuscleElements(
         face, mus, geometryDir+"oop/"+name+"_elements.txt");
   }   
   
   public void loadoop(MuscleBundle mus, String name) {
      RefFemMuscleFaceDemo.loadMuscleElements(
         face, mus, geometryDir+"oop/"+name+"_elements.txt");

      RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, mus);
   }

//   public ArrayList<MuscleElementDesc> addoop(String name) {
//      MuscleBundle oop = face.getMuscleBundles().get("OOP");
//      ArrayList<MuscleElementDesc> muscleElems = new ArrayList<MuscleElementDesc>();
//      String filename = geometryDir+"oop/"+name+"_elements.txt";
//      for (Integer id : BadinFaceDemo.readIntList(filename)) {
//         MuscleElementDesc desc = new MuscleElementDesc();
//         FemElement3d elem = face.getElements().getByNumber(id);
//         desc.setElement(elem);
//         oop.addElement(desc);
//         muscleElems.add(desc);
//      }
//      oop.computeElementDirections();
//      return muscleElems;
//   }

   public static void setNodesDynamicByIndex (FemModel3d fem, String nodeIndicesFilename) {
      for (Integer ni : readIntList (geometryDir + nodeIndicesFilename)) {
         if (ni < fem.numNodes()) {
            FemNode3d n = fem.getNode(ni);
            n.setDynamic (true);
            n.setRenderProps (null);
         }
      }
   }


   public static void setNodesNonDynamicByIndex (FemModel3d fem, String nodeIndicesFilename) {
      for (Integer ni : readIntList (geometryDir + nodeIndicesFilename)) {
         if (ni < fem.numNodes()) {
            FemNode3d n = fem.getNode(ni);
            n.setDynamic (false);
            RenderProps.setPointColor (n, Color.GREEN);
         }
      }
   }


   public static void setNodesNonDynamicByNumber (FemModel3d fem, String nodeIndicesFilename) {
      for (Integer ni : readIntList (geometryDir + nodeIndicesFilename)) {
         FemNode3d n = fem.getByNumber(ni);
         if (n != null) {
            n.setDynamic (false);
            RenderProps.setPointColor (n, Color.GREEN);
         }
      }
   }

   public void enableLipSkullContact() {
      myJawModel.setCollisionBehavior(face, myJawModel.rigidBodies().get("badinjaw"), true);
      myJawModel.setCollisionBehavior(face, myJawModel.rigidBodies().get("badinmaxilla"), true);
   } 

   public static void addWayPoints(RootModel root, double duration,
      double waypointstep) {
      root.removeAllWayPoints();
      for (int i = 1; i < duration / waypointstep; i++) {
         root.addWayPoint(i * waypointstep);
      }
      root.addBreakPoint(duration);
   }
   
   @Override
   public void addTongueToJaw(TongueType tt) {   
      // do not add tongue
   }

   @Override
   public void attachTongueToJaw() {
      // do not attach tongue
   }

   public class MyMonitor extends MonitorBase {
      public void apply (double t0, double t1) {
         if (timeToOpenJaw >= t0 && timeToOpenJaw <= t1){

            try {
               writeTensionFieldFile(resultsDir, "initialTensionField.txt");
            } catch (IOException e) {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }

         }
         else if (timeToOpenJaw + openTime / 2d >= t0 && timeToOpenJaw + openTime / 2d <= t1){

            try {
               writeTensionFieldFile(resultsDir, "midTensionField.txt");
            } catch (IOException e) {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }

         }
         else if (timeToOpenJaw + openTime >= t0 && timeToOpenJaw + openTime <= t1){

            try {
               writeTensionFieldFile(resultsDir, "finalTensionField.txt");
               writeNodalCoordsFile(resultsDir, "finalNodePositions.node");
            } catch (IOException e) {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
            // Save all the probe data
            for ( Probe b : myOutputProbes ) {
               try {
                  b.save();
               } catch (Exception e) {
                  System.out.println(e.getMessage());
               }
            }
         }
      }
   }
   public void writeTensionFieldFile(String dir, String name) throws IOException {

      try {
         String fn = dir + "/" + name;

         FileOutputStream fo = new FileOutputStream(fn);
         PrintStream      ps = new PrintStream(fo);

         for (FemNode3d n : BadinFaceDemo.getOuterNodes(face)) {
            ps.println (n.myNumber + " " + n.getStress ().m00 + " " + n.getStress ().m01 + " " + 
               n.getStress ().m02 + " " + n.getStress ().m11 + " " + 
               n.getStress ().m12 + " " + n.getStress ().m22);
         }

         ps.close();
         fo.close();

      } finally {
      }
   }

   public void writeNodalCoordsFile(String dir, String name) throws IOException {

      try {
         String fn = dir + "/" + name;

         FileOutputStream fo = new FileOutputStream(fn);
         PrintStream      ps = new PrintStream(fo);
         
         for (FemNode3d n : face.getNodes()) {
            ps.println (n.myNumber + " " + n.getPosition ().x + " " + n.getPosition ().y + " " + 
               n.getPosition ().z);
         }

         ps.close();
         fo.close();

      } finally {
      }
   }
   public void writeStrainFieldFile(String dir, String name) throws IOException {

      try {
         String fn = dir + "/" + name;

         FileOutputStream fo = new FileOutputStream(fn);
         PrintStream ps = new PrintStream(fo);

         for (FemNode3d n : BadinFaceDemo.getOuterNodes(face)) {
            ps.println (n.myNumber + " " + n.getStrain ().m00 + " " + n.getStrain ().m01 + " " + 
               n.getStrain ().m02 + " " + n.getStrain ().m11 + " " + 
               n.getStrain ().m12 + " " + n.getStrain ().m22);
         }

         ps.close();
         fo.close();

      } finally {
      }
   }

   public static void attachFaceToSkull (FemModel3d face) {
      BadinFaceDemo.fixInnerNodes (face);

      // un-fix nodes around the lips and cheeks

      BadinFaceDemo.setNodesDynamicByNumber(face, "lip_free_nodes.nodenum");
      BadinFaceDemo.setNodesDynamicByNumber(face, "cheek_free_nodes.nodenum");
      BadinFaceDemo.setNodesDynamicByNumber(face, "nose_free_nodes.nodenum");
      setNodesDynamicByIndex(face, "face_jaw_attachments.txt");
   }



}
