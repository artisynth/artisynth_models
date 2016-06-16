package artisynth.models.face;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.Shading;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.MDLMeshIO;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.jawTongue.JawHyoidFemMuscleTongue;

public class BadinJawTongueFaceDemo extends JawHyoidFemMuscleTongue {

   boolean collideTongueFace = true;
   protected FemMuscleModel face;
   
   double myIntraoralPressure = 0;

   public static PropertyList myProps =
      new PropertyList (BadinJawTongueFaceDemo.class, RootModel.class);

   static {
      myProps.add ("intraoralPressure * *", "external force applied normal to dynamic nodes on inner surface of the face", 0);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   public BadinJawTongueFaceDemo () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      face = createScaledFace();
      myJawModel.addModel (face);
      
      RigidBody jaw = myJawModel.rigidBodies().get("jaw");
      RigidBody maxilla = myJawModel.rigidBodies().get("maxilla");
//      replaceMesh(maxilla, BadinFaceDemo.faceGeometryDir + "ubcmaxilla_symmetric_chop_smooth.obj");
      replaceMesh(maxilla, BadinFaceDemo.faceGeometryDir + "badinmaxilla.obj");
      transformAndReplaceMesh(jaw, BadinFaceDemo.faceGeometryDir + "badinjaw.obj");
//      resetTonguePosition();
//      setBadinTmjPosition();

      BadinFaceDemo.attachFaceToSkull (face);
      BadinFaceDemo.attachFaceToJaw (myJawModel, myJawModel.rigidBodies ().get (
         "jaw"), face, "face_jaw_attachments.txt");

//      attachFaceToTongue(myJawModel, face, tongue,
//	    "badinface_submental_face_attachnodes.txt",
//	    "badinface_submental_tongue_attachnodes.txt");
      
      
      BadinFaceDemo.setNodesDynamicByNumber (face, "submental_free_nodes.nodenum");
      BadinFaceDemo.setNodesDynamicByNumber (face, "zygomatic_free_nodes.nodenum");
            
      BadinFaceDemo.enableLipLipContact(myJawModel, face, false);
      myJawModel.setCollisionBehavior(face, jaw, true);
      myJawModel.setCollisionBehavior(face, maxilla, true);
      if (collideTongueFace && tongue != null) {
         myJawModel.setCollisionBehavior (face, tongue, true);
      }
      
      setFaceRenderProps (face);
      

      /*
       * set OOP muscle
       * 
       */
      loadoop("7M"); // for fibers
      face.getMuscleBundles().get("OOP").clearElements();
      addoop("7Mu");
      addoop("5Ml");

      
   }
   
   public static void transformAndReplaceMesh(RigidBody body, String fileName) {
      try {
         PolygonalMesh mesh = null;
         if (fileName.endsWith (".mdl")) {
            mesh = MDLMeshIO.read (fileName, new Vector3d (m2mm,m2mm,m2mm));
         }
         else { // obj file
            mesh = new PolygonalMesh (new File(fileName));
            mesh.scale (m2mm);
            mesh.inverseTransform(body.getPose());
         }
         
         if (mesh != null) {
            body.setMesh (mesh, fileName);
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
   
   public void resetTonguePosition() {
      if (tongue == null)
	 return;
      /* tongue was moved backward by 2mm in BadinJawHyoidTongue
       * which is not necessary with new badin jaw/maxilla mesh
       */
      RigidTransform3d tongueForward = new RigidTransform3d ();
      tongueForward.p.x = -2.0; //mm
      tongue.transformGeometry (tongueForward);
   }
   
   public void setBadinTmjPosition() {
      Point3d rtmj = new Point3d(137, -49, 137);
      Point3d ltmj = new Point3d(137, 49, 137);
      
      myJawModel.frameMarkers().get("rtmj").setPosition(rtmj);
      myJawModel.frameMarkers().get("ltmj").setPosition(ltmj);
      
      myJawModel.updateCons ();
   }
   
   
   public FemMuscleModel createScaledFace () {
      FemMuscleModel face;
//      face = BadinFaceDemo.createFace(BadinFaceDemo.faceGeometryDir,
//	    "face_skullconformed_quality", /*linearMaterial=*/false);
      
      face = BadinFaceDemo.createFace(BadinFaceDemo.faceGeometryDir,
	    "badinface_oop_csa_midsagittal", /*linearMaterial=*/false);
      face.setName ("badinface");
      
      BadinFaceDemo.addMuscles(face, BadinFaceDemo.muscleNodesFile, BadinFaceDemo.midSagittalPlane);
//      BadinFaceDemo.addMuscles(face, BadinFaceDemo.faceGeometryDir + "face_muscles_yohan.node", BadinFaceDemo.midSagittalPlane);
//    BadinFemMuscleFaceDemo.setMuscleElements(face, BadinFemMuscleFaceDemo.muscleThickness);
//    RefFemMuscleFaceDemo.loadMuscleElements(face, face.getMuscleBundles().get("OOP"), 
//	    BadinFaceDemo.faceGeometryDir+"OOP_7thRing_elements.txt");
      
      
      ((GenericMuscle)face.getMuscleMaterial()).setMaxStress(100000);
      face.setDirectionRenderLen(0.5);
      face.setElementWidgetSize(1);
      

      
      
      face.scaleDistance (m2mm);
      
      return face;
   }

   
   public static void attachFaceToTongue(MechModel mech,
	 FemModel3d face, FemModel3d tongue, String faceAttachNodesFile,
	 String tongueAttachNodesFile) {
      
      Integer[] faceNodeIdxs = BadinFaceDemo.readIntList (BadinFaceDemo.faceGeometryDir + faceAttachNodesFile);
      Integer[] tongueNodeIdxs = BadinFaceDemo.readIntList (BadinFaceDemo.faceGeometryDir + tongueAttachNodesFile);
      
      assert faceNodeIdxs.length == tongueNodeIdxs.length;
      
      for (int i = 0; i < faceNodeIdxs.length; i++) {
	 mech.attachPoint(face.getByNumber(faceNodeIdxs[i]), tongue.getByNumber(tongueNodeIdxs[i]));
      }
   }
   
   public static void projectAndAttachFaceToTongue (
      MechModel mech, FemModel3d tongue, FemModel3d face, String fileName) {
      for (Integer idx : BadinFaceDemo
         .readIntList (BadinFaceDemo.faceGeometryDir + fileName)) {
         FemNode3d faceNode = face.getNode (idx);
         // mech.attachPoint (faceNode, tongueNode);
         Point tonguePoint =
            getClosestSurfacePoint (tongue, faceNode.getPosition ());
         mech.attachPoint (tonguePoint, faceNode);
      }
   }

   private static Point getClosestSurfacePoint (FemModel3d fem, Point3d pos) {
      PolygonalMesh mesh = fem.getSurfaceMesh ();
      //OBBTree obbt = mesh.getObbtree ();
      Point3d proj = new Point3d ();
      Vector2d coords = new Vector2d ();
      BVFeatureQuery query = new BVFeatureQuery();

      query.nearestFaceToPoint (proj, coords, mesh, pos);
      FemElement elem = fem.findContainingElement (proj);
      FemMarker fmkr = new FemMarker (proj);
      fem.addMarker (fmkr);

      return fmkr;
   }

   public void setFaceRenderProps (FemModel3d face) {
      RenderProps.setVisible (face.getNodes (), false);
      RenderProps.setVisible (face.getElements (), true);
      RenderProps.setVisible (face.markers (), false);
      RenderProps.setLineWidth (face, 1);

      face.setSurfaceRendering (SurfaceRender.None);

//      RenderProps.setFaceColor (face, new Color (0.75f, 0.75f, 0.95f));
      RenderProps.setFaceColor(face, new Color(0.9f, 0.8f, 0.75f));
      RenderProps.setShading (myJawModel, Shading.FLAT);
      
      RenderProps.setLineWidth(face.getElements(), 0);
      RenderProps.setLineColor(face, new Color(.2f, .2f, .2f));
      // face.setSubSurfaceRendering(false);
      
      for (RigidBody body : myJawModel.rigidBodies()) {
	RenderProps.setFaceColor(body, new Color(0.55f, 0.55f, 0.65f));
      }
      
//      RenderProps.setVisible(tongue.getMuscleBundles(), true);
//      tongue.setSurfaceRendering(SurfaceRender.Shaded);
      
      for (FemNode3d n : face.getNodes()) {
	 RenderProps.setVisibleMode(n, PropertyMode.Inherited);
      }
      
      for (RigidBody rb : myJawModel.rigidBodies()) {
	 rb.setAxisLength(0);
      }
      
      RenderProps.setVisible(myJawModel.getCollisionManager(), false);
   }

   public void makeExciterProbes() {
      removeAllInputProbes();
      
      if (tongue != null) {
      for (MuscleExciter ex : tongue.getMuscleExciters()) {
	 if (ex.getExcitation() > 0) {
	    addExcitationProbe(ex);
	 }
      }
      }
      
      for (MuscleExciter ex : myJawModel.getMuscleExciters()) {
	 if (ex.getExcitation() > 0) {
	    addExcitationProbe(ex);
	 }
      }
   }
   
   public void addExcitationProbe(MuscleExciter ex) {
      double duration = 1.0, rampDuration = 0.4;
      NumericInputProbe ip = new NumericInputProbe(ex.getProperty("excitation"), ex);
      ip.setStartStopTimes(0, duration);
      ip.setInterpolationOrder(Order.Linear);
      ip.setName(ex.getName()+" excitation");
      ip.addData(0, new VectorNd(1));
      ip.addData(rampDuration, new VectorNd(new double[]{ex.getExcitation()}));
      ip.addData(duration, new VectorNd(new double[]{ex.getExcitation()}));
      ip.setAttachedFileName(ex.getName()+"txt");
      addInputProbe(ip);
   }

   
   @Override
   public void attach(DriverInterface driver) {
      // TODO Auto-generated method stub
      super.attach(driver);
      
      ControlPanel panel;
      panel = FemControlPanel.createControlPanel(this, face, myJawModel);
      panel.setName("Face Controls");
      
      panel = FemControlPanel.createMuscleBundlesPanel(this, face);
      panel.setName("Face Muscles");
      
      File workingDir = 
         new File (ArtisynthPath.getRootRelativePath(this, "issp/u/"));
      if (workingDir.exists ()) {
	 ArtisynthPath.setWorkingDir (workingDir);
	 try {
	    Main.getMain().loadProbesFile(new File(workingDir+"/0probes.art"));
	 } catch (IOException e) {
	    e.printStackTrace();
	 }
      }
      
      
//      myJawModel.scaleDistance(1d/m2mm);
      
//      RigidTransform3d X = BadinFemMuscleFaceDemo.model2ct();
//      face.transformGeometry(X);
//      tongue.transformGeometry(X);
//      for (RigidBody rb : myJawModel.rigidBodies()) {
//	 rb.transformGeometry(X);
////	 if (rb.getMesh() != null) {
////	    rb.getMesh().transform(X);
////	 }
//      }
   }
   
   
   /*
    * from BadinFemMuscleFace
    */
   public void addoop(String name) {
      MuscleBundle oop = face.getMuscleBundles().get("OOP");
      String filename = BadinFaceDemo.faceGeometryDir+"oop/"+name+"_elements.txt";
      for (Integer id : BadinFaceDemo.readIntList(filename)) {
	 MuscleElementDesc desc = new MuscleElementDesc();
	 FemElement3d elem = face.getElements().getByNumber(id);
	 desc.setElement(elem);
	 oop.addElement(desc);
      }
      
      oop.computeElementDirections();
   }
   
   public void diroop() {
      RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, face.getMuscleBundles().get("OOP"));
   }
   
   public void loadoop(String name) {
      RefFemMuscleFaceDemo.loadMuscleElements(face, face.getMuscleBundles().get("OOP"),
	    BadinFaceDemo.faceGeometryDir+"oop/"+name+"_elements.txt");
      
      RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, face.getMuscleBundles().get("OOP"));
   }
   
   
   /*
    * intraoral pressure code, copied from BadinFaceDemo
    */
   
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
      for (FemNode3d n : BadinFaceDemo.getInnerNodes(face)) {
         if (n.isActive() && !n.isAttached()) {
            innerDynamicNodes.add(n);
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


}
