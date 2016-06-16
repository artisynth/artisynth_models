package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.Model;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.MDLMeshIO;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.dynjaw.JawModel;
import artisynth.models.tongue3d.HexTongueDemo;

public class BadinDataDemo extends RootModel {


   protected MechModel mech;
   protected FemMuscleModel tongue = null;
   
   
   public static final String badinGeometryDir =
      ArtisynthPath.getSrcRelativePath (BadinDataDemo.class, "geometry/");
   
   public static final String badinModelsDir =
      ArtisynthPath.getSrcRelativePath (BadinDataDemo.class, "models/");
   public static final double mm2m = 1d / 1000d;
   public static final double cm2m = 1d / 100d;
   public static final double data2faceZdist = -0.0078;
   
   public static double faceTransY = -0.12648; // from Yohan
   public static double faceRotY = Math.toRadians (3.5); // found by manually aligning to original tongue posture
   public static final Point3d tongueToThinJaw_face = new Point3d(-0.0025, 0, -0.001); // found my manually aligning tongue to fit with jaw from ctdata (in face reference frame)
   public static final Point3d tongueToThinJaw_tongue = new Point3d(-0.004, 0, -0.001); // found my manually aligning tongue to fit with jaw from ctdata (in tongue reference frame)
   
   public BadinDataDemo () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      
          
      if (!(new File(badinGeometryDir)).exists () /*||
          !(new File(badinGeometryDir+"ubcMarkers")).exists ()*/) {
         System.err.println("could not load BadinFaceDemo -- no data in directory:\n"+badinGeometryDir);
         return;
      }
      

      mech = new MechModel ("mech");
      RenderProps.setFaceStyle (mech, FaceStyle.FRONT_AND_BACK);
      addModel (mech);
      


      boolean useUbcAffine = false;
      addTransformedMeshes ();
//      addMeshesAndTransform (/*writeMeshesToFile=*/true); 
//      addMeshesAndTransform (/*writeMeshesToFile=*/false);
//      addUbcOrigSkull ();
      
//      addUbcThyroid();
//      addUbcVert ();
      addUbcMarkers();
      
//      transformUbcMarkers(useUbcAffine);
//      writeUbcMarkerPositions();
      readUbcMarkerPositions ();
      
      // uncomment for _face mdl meshes
      registerHyoids();
      
//       addFemFace();
       addTongue ();
       transformTongueToFace ();

       
//       addSoftPalate();
      
      
//       FemModel3d newface = addFemModel("newface", "face");
//       RenderProps.setFaceColor (newface, new Color (0.4f, 0.4f, 1f));

//       double len = 0.118384; // from manual scaling of ct image to fit badinskull mesh
//       addImagePlane(mech, badinGeometryDir+"badin_sagittal256.png", len,
//          new RigidTransform3d(new Vector3d(0.0910441, 0.127461, 0.0736404), 
//             new AxisAngle (1,0,0,Math.PI/2.0)));
       
//       translateFaceToSymmetric();
//       translateThinJawToTongue ();

       
//     addPBMeshes();
  
//       writeAllMeshes("../geometry_tricia");
//      writeObjMeshes ();
       setupRenderProps ();
       
//       FrameMarker[] mkrs = new FrameMarker[mech.frameMarkers ().size ()];
//       mech.frameMarkers ().toArray (mkrs);      
//       writeMarkersToMdl ("ubcmarkers_reg_tongue.mdl", mkrs);
      
   }
   
   public void translateFaceToSymmetric () {
      RigidTransform3d XBadinFace = getXBadinFace ();
      Point3d translation = new Point3d ();
      translation.negate (XBadinFace.p);
      translation.y = faceTransY;
      RigidTransform3d XFaceToTongue = new RigidTransform3d ();
      XFaceToTongue.R.setAxisAngle (0, 1, 0, faceRotY);
      XFaceToTongue.p.add (translation);
      if (mech.models ().get ("badinface") != null) {
         ((FemModel3d)mech.models ().get ("badinface"))
            .transformGeometry (XFaceToTongue);
      }
      for (RigidBody body : mech.rigidBodies ()) {
         body.transformGeometry (XFaceToTongue);
      }

   }

   public void translateThinJawToTongue() {

      RigidTransform3d XThinJawToTongue = new RigidTransform3d ();
      XThinJawToTongue.p.negate (tongueToThinJaw_tongue);

      for (RigidBody body : mech.rigidBodies ()) {
         if (!body.getName ().startsWith ("tongue"))
            body.transformGeometry (XThinJawToTongue);
      }

      // also translate face
      if (mech.models ().get ("badinface") != null) {
         ((FemModel3d)mech.models ().get ("badinface"))
            .transformGeometry (XThinJawToTongue);
      }
   }
   

   
   public void addUbcThyroid() {
      RigidBody thyroid = addUbcBody ("ubcthyroid", "thyroid_t.obj");
      thyroid.scaleDistance (mm2m);
      thyroid.scaleDistance (1.1);
      
      RigidTransform3d thyroidPose = new RigidTransform3d ();
      readTransform (thyroidPose, "XUbcthyroidFace.txt");
      
      thyroid.setPose (thyroidPose);
   }
   
   public void addUbcVert() {
      RigidBody vert = addUbcBody ("ubcvertebrae", "vertebrae_t.obj");
      vert.scaleDistance (mm2m);
      vert.scaleDistance (0.95);
      
      RigidTransform3d vertPose = new RigidTransform3d ();
      readTransform (vertPose, "XUbcvertFace.txt");
      
      vert.setPose (vertPose);
   }

//   public void addSoftPalate () {
//      try {
//         SoftPalateDemo palate = new SoftPalateDemo ("softpalate");
//         MechModel spMech = (MechModel)palate.models ().get (0);
//         spMech.scaleDistance (mm2m);
//         RigidTransform3d X = new RigidTransform3d ();
//         // X.p.set(0.00571704, -0.12648, 0.0327923);
//         X.R.setAxisAngle (0, 0, 1, Math.PI / 2);
//         spMech.transformGeometry (X);
//         for (RigidBody body : spMech.rigidBodies ()) {
//            mech.addRigidBody (body);
//         }
//         mech.addModel ((FemModel3d)spMech.models ().get (0));
//      }
//      catch (IOException e) {
//         e.printStackTrace ();
//      }
//   }

   public void addPBMeshes () {

      RigidTransform3d XPbData = new RigidTransform3d ();
      readTransform (XPbData, "XPbData.txt");

      RigidTransform3d XDataFace = new RigidTransform3d ();
      XDataFace.p.z = data2faceZdist;

      RigidTransform3d XPbFace = new RigidTransform3d ();
      XPbFace.mul (XDataFace, XPbData);

      
      File directory = new File (badinGeometryDir+"PB_3D_meshes/");
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            if (fileName.endsWith (".obj")) {
               RigidBody body = addBody(mech,
                  getBasename(fileName),directory+"/"+fileName);
               body.scaleDistance (cm2m);
               body.transformGeometry (XPbFace);
               RenderProps.setFaceColor (body, new Color (189, 68, 68)); //rust red
            }
         }
      }
      else {
         System.err.println("cannot read PB meshes from " + directory);
      }
   }
   
   public static RigidBody addImagePlane (
      MechModel mech, String imageFileName, double len, RigidTransform3d pose) {
      RigidBody image = new RigidBody ("ctimage");
      image.setDynamic (false);
      image.setPose (pose);
      image.setMesh (
         MeshFactory.createRectangle (len, len, /*textureCoords=*/true), null);
      RenderProps.setColorMapFileName (image, imageFileName);
      RenderProps.setColorMapEnabled (image, true);
      RenderProps.setFaceColor (image, Color.WHITE);
      mech.addRigidBody (image);
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(len/2,-len/2,0));
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(len/2,len/2,0));
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(-len/2,-len/2,0));
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(-len/2,len/2,0));
      return image;
   }
   
   public void addTransformedMeshes() {
      
      File directory = new File (badinGeometryDir);
      if (directory != null && directory.isDirectory ()) {
         for (String fileName : directory.list ()) {
            if (fileName.endsWith (".mdl")) {
               addBodyFromMdl(getBasename(fileName),directory+"/"+fileName);
            }
         }
      }
      else {
         System.err.println("cannot read landmarks from directory " + directory);
      }

   }
   
   /*
    * returns the filename without extension suffix
    */
   public static String getBasename(String fileName) {
//      return fileName.substring (0, fileName.length ()-4);
      String[] names = fileName.split ("_");
      if (names.length > 1)
         return names[0];
      else 
         return fileName.substring (0, fileName.length ()-4); //remove extension
   }
   
   public void addMeshesAndTransform(boolean writeMeshesToFile) {
      
//      addBadinJaw ();
//      addBadinHyoid ();
//      addBadinCT ();
//      addBadinPalate();
      
      addUbcSkull ();
      
//      transformBadinMeshes ();
      transformUbcMeshes (/*useAffineXform=*/false);
//      resetBodyPoses();
//      
//
//      if (writeMeshesToFile) {
////         String[] meshesToWrite =
////            new String[] { "badinskull", "badinskin", "badinjaw",
////                          "badinhyoid", "badinpalate", "ubcjaw", "ubcmaxilla",
////                          "ubccranium", };
//         String[] meshesToWrite =
//            new String[] { "ubcjaw", "ubcmaxilla",
//                          "ubccranium", };
//         String suffix = "_face";
//         writeMdlMeshes (meshesToWrite, suffix);
//      }
   }
   
   public void transformBadinMeshes() {

//      RigidTransform3d XJawData = new RigidTransform3d ();
////      getBadinJawToBadinData (XJawData);
////      System.out.println ("XJawData = \n" + XJawData.toString ("%4.2f"));
////      writeTransform (XJawData, "XBadinJawData.txt");
//      readTransform (XJawData, "XBadinJawData.txt");
//      
//      String[] badinToTransform = new String[]{"badinjaw", "badinhyoid", "badinpalate"};
//      for (String name : badinToTransform)
//         mech.rigidBodies ().get (name).transformGeometry (XJawData);
//      
//      RigidTransform3d XDataFace = new RigidTransform3d();
//      XDataFace.p.z = data2faceZdist;
//      String[] meshesDataToFace =
//         new String[] { "badinjaw", "badinhyoid", "badinpalate" };
      
//      for (String name : meshesDataToFace) {
//         mech.rigidBodies ().get(name).transformGeometry (XDataFace);
//      }
      
      RigidTransform3d XJawFace = getXBadinFace ();
      String[] badinToTransform = new String[]{"badinjaw", "badinhyoid", "badinpalate"};
      for (String name : badinToTransform)
         mech.rigidBodies ().get (name).transformGeometry (XJawFace);

   }
   
   public void registerHyoids() {
      String[] hyoids = new String[]{"amirahyoid", "ubchyoid", "ubcthyroid"};
      RigidTransform3d XAmirahyoidBadinhyoid = new RigidTransform3d ();
      readTransform (XAmirahyoidBadinhyoid, "XAmirahyoidAnsyshyoid.txt");
      
      for (String hyoidname : hyoids) {
         RigidBody hyoid = mech.rigidBodies ().get (hyoidname);
         if (hyoid == null) {
            System.err.println("registerHyoids: no "+hyoidname+" found, ignoring");
            continue;
         }
         hyoid.transformGeometry (XAmirahyoidBadinhyoid);
      }

   }
   
   public void transformUbcMeshes(boolean useUbcAffineMeshes) {

      String[] mmMeshes = new String[]{"ubcjaw", "ubcmaxilla", "ubccranium", "ubchyoid"};
      for (String name : mmMeshes) {
         mech.rigidBodies ().get (name).scaleDistance (mm2m);
      }
      
      AffineTransform3dBase XUbcjawFace, XUbcmaxFace;
      
      if (useUbcAffineMeshes) {
         XUbcjawFace = new AffineTransform3d();
         readTransform (XUbcjawFace, "XUbcjawFaceAffine.txt");
         
         XUbcmaxFace = new AffineTransform3d();
         readTransform (XUbcmaxFace, "XUbcmaxFaceAffine.txt");
         
      }
      else {
         RigidTransform3d XUbcjawData = new RigidTransform3d ();
         // getUbcJawToBadinData (XUbcjawData);
         // System.out.println ("XUbcjawData = \n" + XUbcjawData.toString
         // ("%4.2f"));
         // writeTransform (XUbcjawData, "XUbcjawData.txt");
         readTransform (XUbcjawData, "XUbcjawData.txt");

         RigidTransform3d XUbcmaxillaData = new RigidTransform3d ();
         // getUbcMaxillaToBadinData (XUbcmaxillaData);
         // System.out.println ("XUbcmaxillaData = \n" +
         // XUbcmaxillaData.toString ("%4.2f"));
         // writeTransform (XUbcmaxillaData, "XUbcmaxData.txt");
         readTransform (XUbcmaxillaData, "XUbcmaxData.txt");

         RigidTransform3d XDataFace = new RigidTransform3d ();
         XDataFace.p.z = data2faceZdist;
         
         XUbcjawFace = new RigidTransform3d ();
         XUbcmaxFace = new RigidTransform3d ();

         ((RigidTransform3d)XUbcjawFace).mul (XDataFace, XUbcjawData);
         ((RigidTransform3d)XUbcmaxFace).mul (XDataFace, XUbcmaxillaData);
      }
      
      RigidTransform3d XUbchyoidFace = new RigidTransform3d ();
      readTransform (XUbchyoidFace, "XUbchyoidFace.txt");
//      RigidTransform3d XAmirahyoidBadinhyoid = new RigidTransform3d ();
//      readTransform (XAmirahyoidBadinhyoid, "XAmirahyoidAnsyshyoid.txt");
//      XUbchyoidFace.mul(XAmirahyoidBadinhyoid, XUbchyoidFace);
      
      mech.rigidBodies ().get ("ubcjaw").transformGeometry (XUbcjawFace);
      mech.rigidBodies ().get ("ubchyoid").transformGeometry (XUbchyoidFace);
      mech.rigidBodies ().get ("ubcmaxilla").transformGeometry (XUbcmaxFace);
      mech.rigidBodies ().get ("ubccranium").transformGeometry (XUbcmaxFace);

   }
   
   public void resetBodyPoses() {
      for (RigidBody body : mech.rigidBodies ()) {
         if (body.getMesh () != null)
            body.getMesh ().transform (body.getPose ());
         body.setPose (new RigidTransform3d());
      }
   }

   public void addUbcOrigSkull () {
      RigidBody jaw = addUbcBody ("ubcjaw", "jaw_smooth.obj");
      RigidBody max = addUbcBody ("ubcmaxilla", "maxilla_smooth.obj");
      RigidBody cra = addUbcBody ("ubccranium", "cranium_topcrop2.obj");
      RigidBody hyoid = addUbcBody ("ubchyoid", "hyoid_t.obj");
   }
   
   public void addUbcSkull () {
      RigidBody jaw = addUbcBody ("ubcjaw", "jaw_smooth.obj");
      RigidBody max = addUbcBody ("ubcmaxilla", "maxilla_smooth.obj");
      RigidBody cra = addUbcBody ("ubccranium", "cranium_topcrop2.obj");
      RigidBody hyoid = addUbcBody ("ubchyoid", "hyoid_t.obj");
   }

   public void addTongue () {
      
      tongue = HexTongueDemo.createHexTongue (true, true);

      mech.addModel (tongue);
      tongue.setName ("badintongue");
   }
   
   public static RigidTransform3d getXBadinFace() {
      RigidTransform3d XJawData = new RigidTransform3d ();
      readTransform (XJawData, "XBadinJawData.txt");
    
      RigidTransform3d XDataFace = new RigidTransform3d();
      XDataFace.p.z = data2faceZdist;

      RigidTransform3d XBadinjawFace = new RigidTransform3d ();
      XBadinjawFace.mul (XDataFace, XJawData);
      
      return XBadinjawFace;
   }
   
   public void transformTongueToFace () {
      if (tongue == null)
         return;
      
      tongue.transformGeometry (getXBadinFace ());
//      RigidTransform3d XJawData = new RigidTransform3d ();
////      getBadinJawToBadinData (XJawData);
////      writeRigidTransform (XJawData, "XBadinJawData.txt");
//      readTransform (XJawData, "XBadinJawData.txt");
//      tongue.transformGeometry (XJawData);
//      
//      RigidTransform3d XDataFace = new RigidTransform3d();
//      XDataFace.p.z = data2faceZdist;
//      tongue.transformGeometry (XDataFace);
   }
   
   public void addBadinPalate () {
      RigidBody palate = addBadinBody ("badinpalate", "palate.obj");
      RenderProps.setFaceColor (palate, new Color (1f, 0.2f, 0.2f));
      palate.getMesh ().triangulate ();
   }
   
   public void addBadinJaw () {
      RigidBody jaw = addBadinBody ("badinjaw", "jaw.obj");
      RenderProps.setFaceColor (jaw, new Color (0.6f, 0.6f, 0.8f));
      jaw.getMesh ().triangulate ();
   }

   public void addBadinHyoid () {
      RigidBody hyoid = addBadinBody ("badinhyoid", "hyoid.obj");
      RenderProps.setFaceColor (hyoid, new Color (0.8f, 0.8f, 0.6f));
   }

   public void addFemBadinHyoid () {
      FemModel3d hyoid = addFemModel ("badinhyoid", "hyoid");
      RenderProps.setFaceColor (hyoid, new Color (0.8f, 0.8f, 0.6f));
   }

   public void addFemFace () {
      FemModel3d face = addFemModel ("badinface", "face.orig");
      RenderProps.setFaceColor (face, new Color (0.8f, 0.6f, 0.6f));
   }
   
   public void addBadinCT () {
      RigidBody ct = addBodyFromMdl ("badinskull", badinGeometryDir+"badinskull_amira_face.mdl");
      RenderProps.setFaceStyle (ct, FaceStyle.FRONT_AND_BACK);
      RigidBody skin = addBodyFromMdl ("badinskin", badinGeometryDir+"badinskin_face.mdl");
      RenderProps.setFaceStyle (skin, FaceStyle.FRONT_AND_BACK);
   }
   
   public RigidBody addBodyFromMdl(String name, String fullMeshFileName) {
      RigidBody body = new RigidBody (name);
      body.setDynamic (false);
      try {
         PolygonalMesh mesh =
            MDLMeshIO.read (fullMeshFileName, null/* no scaling */);
         body.setMesh (mesh, null);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      mech.addRigidBody (body);
      RenderProps.setFaceColor (body, Color.LIGHT_GRAY);
      return body;
   }

   public FemModel3d addFemModel (String name, String meshFilePrefix) {
      FemModel3d fem = new FemModel3d (name);
      try {
         AnsysReader.read (fem, 
            badinGeometryDir + meshFilePrefix + ".node",
            badinGeometryDir + meshFilePrefix + ".elem",
            1, null, /*options=*/AnsysReader.TETRAHEDRALIZE_HEXES);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      fem.setSurfaceRendering (SurfaceRender.Shaded);
      mech.addModel (fem);
      return fem;
   }

   public RigidBody addBadinBody (String name, String meshFileName) {
      String fullMeshFileName = badinGeometryDir + meshFileName;
      return addBody (mech, name, fullMeshFileName);
   }
   
   public RigidBody addUbcBody (String name, String meshFileName) {
      String fullMeshFileName =
         ArtisynthPath.getSrcRelativePath (JawModel.class, "geometry/"
         + meshFileName);
      return addBody(mech, name, fullMeshFileName);
   }
   
   public static RigidBody addBody (MechModel mech, String name, String fullMeshFileName) {
      RigidBody body = new RigidBody (name);
      body.setDynamic (false);
      addMesh (body, fullMeshFileName);
      mech.addRigidBody (body);
      return body;
   }
   


   private static void addMesh (RigidBody body, String meshname) {
      try {
         PolygonalMesh mesh = new PolygonalMesh (new File (meshname));
         body.setMesh (mesh, meshname);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   public static void setMarkerPositions (
      List<FrameMarker> markerList, String fileName) {
      readMarkerPositions (markerList.toArray (
         new FrameMarker[markerList.size ()]), fileName);
   }

   public static void readMarkerPositions (
      FrameMarker[] markers, String fileName) {
      readMarkerPositions (markers, fileName, 1d);
   }
   
   public static void readMarkerPositions (
      FrameMarker[] markers, String fileName, double s) {
      ArrayList<Point3d> pnts = new ArrayList<Point3d> ();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (fileName));
         while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
            rtok.pushBack ();
            Point3d p = new Point3d ();
            p.x = rtok.scanNumber ();
            p.y = rtok.scanNumber ();
            p.z = rtok.scanNumber ();
            p.scale (s);
            pnts.add (p);
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }


      if (pnts.size () > markers.length) {
         System.out.printf (
            "warning read %d positions for %d markers\n",
            pnts.size (), markers.length);
      }
      else if (pnts.size () < markers.length) {
         System.out.printf (
            "warning read %d positions for %d markers\n",
            pnts.size (), markers.length);
      }

      Point3d loc = new Point3d();
      Point3d pos = new Point3d();
      int num = Math.min (pnts.size (), markers.length);
      for (int i = 0; i < num; i++) {
         // assume positions specified in world-coordinate   
         pos.set (pnts.get (i));
         loc.inverseTransform (markers[i].getFrame ().getPose (), pos);
         markers[i].setLocation (loc);
         markers[i].setRefPos (pos);
      }

   }
   
   public void transformUbcMarkers(boolean useUbcAffineMeshes) {
      
      RigidBody cranium = null, maxilla = null, jaw = null,  hyoid = null;
      for (RigidBody body : mech.rigidBodies ()) {
         if (body.getName () != null && body.getName ().startsWith ("ubccranium"))
            cranium = body;
         else if (body.getName () != null && body.getName ().startsWith ("ubcmaxilla"))
            maxilla = body;         
         else if (body.getName () != null && body.getName ().startsWith ("ubcjaw"))
            jaw = body;
         else if (body.getName () != null && body.getName ().startsWith ("ubchyoid"))
            hyoid = body;
      }
      if (cranium == null || maxilla == null || jaw == null) {
         System.err.println("cannot find ubc bodies");
         return;
      }
      
      AffineTransform3dBase XUbcjawFace, XUbcmaxFace;
      
      if (useUbcAffineMeshes) {
         XUbcjawFace = new AffineTransform3d();
         readTransform (XUbcjawFace, "XUbcjawFaceAffine.txt");
         
         XUbcmaxFace = new AffineTransform3d();
         readTransform (XUbcmaxFace, "XUbcmaxFaceAffine.txt");
         
      }
      else {
         RigidTransform3d XUbcjawData = new RigidTransform3d ();
         // getUbcJawToBadinData (XUbcjawData);
         // System.out.println ("XUbcjawData = \n" + XUbcjawData.toString
         // ("%4.2f"));
         // writeTransform (XUbcjawData, "XUbcjawData.txt");
         readTransform (XUbcjawData, "XUbcjawData.txt");

         RigidTransform3d XUbcmaxillaData = new RigidTransform3d ();
         // getUbcMaxillaToBadinData (XUbcmaxillaData);
         // System.out.println ("XUbcmaxillaData = \n" +
         // XUbcmaxillaData.toString ("%4.2f"));
         // writeTransform (XUbcmaxillaData, "XUbcmaxData.txt");
         readTransform (XUbcmaxillaData, "XUbcmaxData.txt");

         RigidTransform3d XDataFace = new RigidTransform3d ();
         XDataFace.p.z = data2faceZdist;
         
         XUbcjawFace = new RigidTransform3d ();
         XUbcmaxFace = new RigidTransform3d ();

         ((RigidTransform3d)XUbcjawFace).mul (XDataFace, XUbcjawData);
         ((RigidTransform3d)XUbcmaxFace).mul (XDataFace, XUbcmaxillaData);
      }
      
      
      RigidTransform3d XUbchyoidFace = new RigidTransform3d ();
      readTransform (XUbchyoidFace, "XUbchyoidFace.txt");
//      RigidTransform3d XAmirahyoidBadinhyoid = new RigidTransform3d ();
//      readTransform (XAmirahyoidBadinhyoid, "XAmirahyoidAnsyshyoid.txt");
//      XUbchyoidFace.mul(XAmirahyoidBadinhyoid, XUbchyoidFace);
      
      if (cranium.getPose ().isIdentity ()) {
         for (FrameMarker m : cranium.getFrameMarkers ()) {
            m.transformGeometry (XUbcmaxFace);
         }
      }
      
      if (maxilla.getPose ().isIdentity ()) {
         for (FrameMarker m : maxilla.getFrameMarkers ()) {
            m.transformGeometry (XUbcmaxFace);
         }
      }
      
      if (jaw.getPose ().isIdentity ()) {
         for (FrameMarker m : jaw.getFrameMarkers ()) {
            m.transformGeometry (XUbcjawFace);
         }
      }
      
      if (hyoid.getPose ().isIdentity ()) {
         for (FrameMarker m : hyoid.getFrameMarkers ()) {
            m.transformGeometry (XUbchyoidFace);
         }
      }

      
   }
   
   public void addUbcMarkers() {

      String fileshortname = "UbcSkullHyoidMarkers.art";
      RigidBody cranium = null, maxilla = null, jaw = null,  hyoid = null;
      for (RigidBody body : mech.rigidBodies ()) {
         if (body.getName () != null && body.getName ().equals ("ubccranium"))
            cranium = body;
         else if (body.getName () != null && body.getName ().equals ("ubcmaxilla"))
            maxilla = body;         
         else if (body.getName () != null && body.getName ().equals ("ubcjaw"))
            jaw = body;
         else if (body.getName () != null && body.getName ().equals ("ubchyoid"))
            hyoid = body;
      }
      if (cranium == null || maxilla == null || jaw == null || hyoid == null) {
         System.err.println("cannot find ubc bodies");
         return;
      }
      // for original jaw model, cranium=r/0, maxilla=r/1, jaw=r/2
      

      
      String fileName = badinGeometryDir+fileshortname;
      
      try {
         ReaderTokenizer rtok = ArtisynthIO.newReaderTokenizer (fileName);
         rtok.wordChar ('_');
         Point3d loc = new Point3d();
         String name = null;
         ArrayList<FrameMarker> hyoidMarkers = new ArrayList<FrameMarker>();
         while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
            if (rtok.ttype == ReaderTokenizer.TT_WORD
            && rtok.sval.startsWith ("r/")) {
               int bodyIdx = Integer.parseInt (rtok.sval.substring (2, 3));
               String strtok = rtok.scanWord ();
               if (strtok.equals ("name")) {
                  rtok.scanToken ('=');
                  rtok.nextToken ();
                  name = rtok.sval;
               }
               else
                  name = null;
                  
               
               while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
                  if (rtok.ttype == ReaderTokenizer.TT_WORD
                  && rtok.sval.equals ("location")) {
                     rtok.scanToken ('=');
                     rtok.scanToken ('[');
                     loc.scan (rtok);
                     FrameMarker mkr = new FrameMarker (name);
                     switch(bodyIdx) {
                        case 0: // cranium 
                           mech.addFrameMarker (mkr, cranium, loc);
                           break;
                        case 1: // maxilla
                           mech.addFrameMarker (mkr, maxilla, loc);
                           break;
                        case 2: // jaw
                           mech.addFrameMarker (mkr, jaw, loc);
                           break;
                        case 3: // hyoid
                           mech.addFrameMarker (mkr, hyoid, loc);
                           break;
                     }
                     break;
                  }
               }
            }
         }
         
      }
      catch(IOException e) {
         e.printStackTrace ();
      }
      
      RenderProps.setPointStyle (mech.frameMarkers (), PointStyle.SPHERE);
      RenderProps.setPointRadius (mech.frameMarkers(), 0.001);
      RenderProps.setPointColor (mech.frameMarkers (), Color.BLUE);
      
   }

   
   public void writeMarkersToMdl(String filename, FrameMarker[] markers) {

      Point3d[] pnts = new Point3d[markers.length];
      for (int i = 0; i < pnts.length; i++) {
         pnts[i] = markers[i].getPosition ();
      }
      try {
         PrintWriter pw = new PrintWriter (new File(filename));
         MDLMeshIO.writePoints (pnts, filename, pw, new NumberFormat ("%g"));
         pw.close ();
      }
      catch(FileNotFoundException e) {
         e.printStackTrace ();
      }
      
   }
   
   
   
   public FrameMarker[] getUbcSkullMarkers() {
      String[] skullNames = new String[]{"ubccranium","ubcmaxilla","ubcjaw"};
      ArrayList<RigidBody> skullBodies = new ArrayList<RigidBody> ();
      ArrayList<FrameMarker> markers = new ArrayList<FrameMarker> ();
      for (String bodyName : skullNames) {
         skullBodies.add (mech.rigidBodies().get(bodyName));
      }
      
      for (FrameMarker m : mech.frameMarkers ()) {
         if (skullBodies.contains (m.getFrame ()))
            markers.add (m);
      }
      
      return markers.toArray (new FrameMarker[markers.size ()]);
   }
  
   public void readUbcMarkerPositions() {
      readMarkerPositions (getUbcSkullMarkers (), 
         badinGeometryDir+"ubcMarkers/ubcskull_markers_REGISTERED.txt");
      
      readMarkerPositions ( 
         mech.rigidBodies ().get("ubchyoid").getFrameMarkers (),
         badinGeometryDir+"ubcMarkers/ubchyoid_markers_REGISTERED.txt");
   }
   
   public void writeUbcMarkerPositions() {
      writeMarkerPositions (badinGeometryDir+"ubcMarkers/ubcskull_markers.txt", 
         getUbcSkullMarkers ());
      writeMarkerNames (badinGeometryDir+"ubcMarkers/ubcskull_markerNames.txt", 
         getUbcSkullMarkers ());
      
      writeMarkerPositions (badinGeometryDir+"ubcMarkers/ubchyoid_markers.txt", 
         mech.rigidBodies ().get("ubchyoid").getFrameMarkers ());
      writeMarkerNames (badinGeometryDir+"ubcMarkers/ubchyoid_markerNames.txt", 
         mech.rigidBodies ().get("ubchyoid").getFrameMarkers ());
   }
   
   public void writeMarkerPositions(String filename, FrameMarker[] markers) {
      try {
         PrintWriter pw = new PrintWriter (new File(filename));
         for (FrameMarker m : markers) {
            m.getPosition ().write (pw, new NumberFormat ("%g"));
            pw.println ();
         }
         pw.close ();
      }
      catch(IOException e) {
         e.printStackTrace ();
      }
   }
   
   public void writeMarkerNames(String filename, FrameMarker[] markers) {
      try {
         PrintWriter pw = new PrintWriter (new File(filename));
         for (FrameMarker m : markers) {
            pw.println(m.getName ());
            }
         pw.close ();
      }
      catch(IOException e) {
         e.printStackTrace ();
      }
   }
   
   public static double getBadinJawToBadinData (RigidTransform3d X) {

      return fitLandmarks (
         X, badinModelsDir+"BadinDataLandmarksMaxilla.art", 
         badinModelsDir  + "BadinJawLandmarks.art");

   }
   
   public static double getUbcJawToBadinData (RigidTransform3d X) {

      String badinData = badinModelsDir+"BadinDataLandmarksMaxilla.art";
      String ubcJaw = badinModelsDir+"ArtisynthJawLandmarks.art";

      //  use real distance scaling
      return fitLandmarks (X, badinData, ubcJaw, 1d, mm2m); 
     
      // compute scaling for best fit
//      double scaling = fitLandmarks (new RigidTransform3d(), badinData, ubcJaw, 1d, 1d);
//      return fitLandmarks (X, badinData, ubcJaw, 1d, scaling);
   }
   
   public static double getUbcMaxillaToBadinData (RigidTransform3d X) {

      String badinData =badinModelsDir+"BadinDataLandmarksMaxilla.art";
      String ubcJaw =badinModelsDir+"ArtisynthMaxillaLandmarks.art";

      //  use real distance scaling
      return fitLandmarks (X, badinData, ubcJaw, 1d, mm2m); 
     
      // compute scaling for best fit
//      double scaling = fitLandmarks (new RigidTransform3d(), badinData, ubcJaw, 1d, 1d);
//      return fitLandmarks (X, badinData, ubcJaw, 1d, scaling);
   }
   
   public static double fitLandmarks (
      RigidTransform3d X, String destModelFile, String srcModelFile) {
      return fitLandmarks (X, destModelFile, srcModelFile, 1d, 1d);
   }
   
   public static double fitLandmarks (
      RigidTransform3d X, String destModelFile, String srcModelFile,
      double scaleDest, double scaleSrc) {

      MechModel srcModel =
         getMechModel (srcModelFile);
      srcModel.scaleDistance (scaleSrc);

      MechModel destModel =
         getMechModel (destModelFile);
      destModel.scaleDistance (scaleDest);

      ArrayList<Point3d> srcPnts = new ArrayList<Point3d> ();
      ArrayList<Point3d> destPnts = new ArrayList<Point3d> ();

      for (FrameMarker srcMarker : srcModel.frameMarkers ()) {
         if (srcMarker.getName () == null) 
            continue;
         FrameMarker destMarker =
            destModel.frameMarkers ().get (srcMarker.getName ());
         if (destMarker == null) 
            continue;
         
         srcPnts.add (srcMarker.getPosition ());
         destPnts.add (destMarker.getPosition ());
      }
      return X.fit (destPnts, srcPnts);
   }
   

   public static MechModel getMechModel (String modelFileName) {
      RootModel root = new RootModel ();
      try {
         ReaderTokenizer rtok =
            ArtisynthIO.newReaderTokenizer (new File (modelFileName));
         root.scan (rtok, null);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

      for (Model mod : root.models ()) {
         if (MechModel.class.isAssignableFrom (mod.getClass ()))
            return (MechModel)mod;
      }

      return null;
   }

   
   public void writeAllMeshes(String relDirName) {
      for (RigidBody body : mech.rigidBodies ()) {
         
         PolygonalMesh mesh = body.getMesh ();
         if (mesh != null && body.getName () != null) {
            if (!body.getPose ().isIdentity ()) {
               mesh.transform (body.getPose ());
               body.setPose (new RigidTransform3d());
            }
            
            writeObjMesh (mesh, relDirName + body.getName () + ".obj");
            writeMdlMesh (mesh, relDirName + body.getName () + ".mdl");
         }
      }
      
      for (Model mod : mech.models ()) {
         if (mod instanceof FemModel3d) {
            FemModel3d fem = (FemModel3d)mod;
            PolygonalMesh mesh = fem.getSurfaceMesh ();
            
            if (mesh != null && mod.getName () != null) {
               writeObjMesh (mesh, relDirName + mod.getName () + ".obj");
               writeMdlMesh (mesh, relDirName + mod.getName () + ".mdl");
         }
         }
      }
   }

   public void writeMdlMeshes (String[] names, String suffix) {
      for (String name : names) {
         PolygonalMesh mesh = mech.rigidBodies ().get (name).getMesh ();
         writeMdlMesh (mesh, name + suffix +".mdl");
      }

   }

   public static void writeMdlMesh (PolygonalMesh mesh, String shortname) {
      try {
         String filename = badinGeometryDir+shortname;
         PrintWriter pw = new PrintWriter (new File (filename));
         MDLMeshIO.write (mesh, filename, pw, new NumberFormat ("%g"));
         pw.close ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   public void writeObjMeshes () {
      for (RigidBody body : mech.rigidBodies ()) {
         
         PolygonalMesh mesh = body.getMesh ();
         if (mesh != null && body.getName () != null) {
            writeObjMesh (mesh, body.getName () + "_face.obj");
         }
      }
   }
   
   public static void writeObjMesh (PolygonalMesh mesh, String shortname) {
      try {
         String filename =badinGeometryDir+shortname;
         PrintWriter pw = new PrintWriter (new File (filename));
         mesh.write (pw, new NumberFormat ("%g"), /*zeroIndexed=*/false);
         pw.close ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   public static void writeTransform (AffineTransform3dBase X, String shortname) {
      try {
         String filename =badinGeometryDir+shortname;
         PrintWriter pw = new PrintWriter (new File (filename));
         pw.println("[");
         X.write (pw, new NumberFormat ("%g"));
         pw.println("]");
         pw.close ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   public static void readTransform (AffineTransform3dBase X, String shortname) {
      if (X == null)
         return;
      try {
         String filename =badinGeometryDir+shortname;
         ReaderTokenizer rtok = new ReaderTokenizer ((new FileReader (filename)));
         X.scan (rtok); 
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   public void setupRenderProps() {
      RenderProps.setFaceStyle (mech, FaceStyle.FRONT_AND_BACK);
      
      for (RigidBody body : mech.rigidBodies ()) {
         if (body.getName () == null)
            continue;
         else if (body.getName ().startsWith ("ubc")) 
            RenderProps.setFaceColor (body, new Color (1f, 0.8f, 0.6f));
         else if (body.getName ().equals ("badinskin"))
            RenderProps.setFaceColor (body, Color.LIGHT_GRAY);
         else if (body.getName ().startsWith ("badinskull") ||
                  body.getName ().startsWith ("amira"))
            RenderProps.setFaceColor (body, new Color (0.3f, 0.3f, 0.6f));
         else if (body.getName ().startsWith ("badin"))
            RenderProps.setFaceColor (body, new Color (0f, 0f, 1f));
      }
      
      for (Model mod : mech.models ()) {
         if (mod.getName () == null || !(mod instanceof Renderable)) {
            continue;
         }
         else if (mod.getName ().equals ("badinface"))
            RenderProps.setFaceColor ((Renderable)mod, new Color (0.8f, 0.6f, 0.6f));
         else if (mod.getName ().equals ("badintongue"))
            RenderProps.setFaceColor ((Renderable)mod, new Color (0.8f,0.5f,0.5f));
      }
   }
   
   public void setupRenderPropsX() {
      RenderProps.setFaceStyle (mech, FaceStyle.FRONT_AND_BACK);
      
      for (RigidBody body : mech.rigidBodies ()) {
         if (body.getName () == null)
            continue;
         else if (body.getName ().startsWith ("ubc")) 
            RenderProps.setFaceColor (body, new Color (0f, 0.7f, 0.7f));
         else if (body.getName ().startsWith ("badinjaw"))
            RenderProps.setFaceColor (body, new Color (0.2f, 0.2f, 0.8f));
         else if (body.getName ().startsWith ("badinhyoid"))
            RenderProps.setFaceColor (body, new Color (1f, 0.8f, 0.2f));
         else if (body.getName ().startsWith ("badinpalate"))
            RenderProps.setFaceColor (body, new Color (1f, 0.2f, 0.2f));
         else if (body.getName ().startsWith ("badinskull"))
            RenderProps.setFaceColor (body, new Color (0.2f, 0.5f, 0.2f));
      }
      
      for (Model mod : mech.models ()) {
         if (mod.getName () == null || !(mod instanceof Renderable)) {
            continue;
         }
         else if (mod.getName ().equals ("badinhyoid"))
            RenderProps.setFaceColor ((Renderable)mod, new Color (1f, 1f, 0.6f));
         else if (mod.getName ().equals ("badinface"))
            RenderProps.setFaceColor ((Renderable)mod, new Color (0.8f, 0.6f, 0.6f));
         else if (mod.getName ().equals ("badintongue"))
            RenderProps.setFaceColor ((Renderable)mod, new Color (0.3f, 0.1f, 0.4f));
      }
   }

   public static String stripSuffix (String str) {
      String[] substr = str.split ("_");
      return substr[0];
   }
   
   
   public static void createVisibilityPanel(RootModel root, MechModel mech) {
   
      if (mech == null)
         return;
      ControlPanel panel = new ControlPanel ("Show");
      for (Model mod : mech.models ()) {
         panel.addWidget (stripSuffix(mod.getName ()), mod, "renderProps.visible");
      }
      for (RigidBody body : mech.rigidBodies ()) {
         panel.addWidget (stripSuffix(body.getName ()), body, "renderProps.visible");
      }

      root.addControlPanel (panel);
   }
   
   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);
      if (myControlPanels.size() == 0) {
         createVisibilityPanel(this, mech);
      }
   }
   
   public static void writeNodesToMdl(String filename, FemModel3d fem) {
      
      Point3d[] pnts = new Point3d[fem.numNodes ()];
      for (int i = 0; i < pnts.length; i++) {
         pnts[i] = fem.getNode (i).getPosition ();
      }
      try {
         PrintWriter pw = new PrintWriter (new File(filename));
         MDLMeshIO.writePoints (pnts, filename, pw, new NumberFormat ("%g"));
         pw.close ();
      }
      catch(FileNotFoundException e) {
         e.printStackTrace ();
      }
      
   }
   

}
