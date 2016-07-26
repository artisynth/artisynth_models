package artisynth.tools.exReader.tests;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.tools.exReader.CmissFileManager;
import artisynth.tools.exReader.ExMeshGenerator;
import artisynth.tools.exReader.ExParser;
import artisynth.tools.exReader.ExRegion;
import artisynth.tools.exReader.ExWriter;
import artisynth.tools.exReader.TangentFlipper;

public class SurfaceMeshes extends RootModel {

   private static int[] defaultResolution = { 5, 5, 5 };
   private static final String localPath = ArtisynthPath.getSrcRelativePath(
      SurfaceMeshes.class, "data/cmiss/");
   private static final String wavefrontPath = localPath + "wavefront/";
   private static final String fixedPath = localPath + "fixed/";
   
   CmissFileManager cmiss = new CmissFileManager(localPath);
   
   
   private static String armMusclesFilename = "armmuscles.list";
   private static String armBonesFilename = "armbones.list";
   private static String armPath  = "left_arm/";
   
   private static String legMusclesFilename = "legmuscles.list";
   private static String legBonesFilename = "legbones.list";
   private static String legPath  = "right_leg/";
   
   String objSuffix = ".obj";
   
   ExParser myParser;
   MechModel myModel = new MechModel("cmiss");

   RenderProps boneRenderProps;
   RenderProps muscleRenderProps;

   boolean armVisible = true;
   boolean legVisible = true;
   boolean bonesVisible = true;
   boolean musclesVisible = true;

   ArrayList<RigidBody> armBones;
   ArrayList<RigidBody> armMuscles;
   ArrayList<RigidBody> legBones;
   ArrayList<RigidBody> legMuscles;

   boolean writeStuff = false;
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public SurfaceMeshes () {

      myParser = new ExParser();

      armBones = new ArrayList<RigidBody>();
      armMuscles = new ArrayList<RigidBody>();
      legBones = new ArrayList<RigidBody>();
      legMuscles = new ArrayList<RigidBody>();

      boneRenderProps = new RenderProps();
      boneRenderProps.setFaceColor(new Color(1, 204f/255f, 153f/255f));
      boneRenderProps.setVisible(true);
      boneRenderProps.setFaceStyle(FaceStyle.FRONT_AND_BACK);

      muscleRenderProps = new RenderProps();
      muscleRenderProps.setFaceColor(new Color(165, 66, 56));
      muscleRenderProps.setVisible(true);
      muscleRenderProps.setFaceStyle(FaceStyle.FRONT_AND_BACK);

   }

   public SurfaceMeshes (String name) {
      this();
      setName(name);
      buildModel();
      addModel(myModel);
   }

   public void buildModel() {
      addArmBones();
      addArmMuscles();
       addLegBones();
       addLegMuscles();

      transformGeometry(armBones);
      transformGeometry(armMuscles);
      transformGeometry(legBones);
      transformGeometry(legMuscles);
      
      if (writeStuff) {
         writeMeshes();
      }

   }
   
   public void writeMeshes() {
      
      String resString = ""+defaultResolution[0];
      for (int i=1; i<3; i++) {
         resString += "x" + defaultResolution[i];
      }
      
      String out = wavefrontPath + resString + File.separator;
      
      
      String armMuscleDir = out + armPath;
      File armMuscleDirFile = new File(armMuscleDir);
      armMuscleDirFile.mkdirs(); // make directories if don't exist
      
      for (int i=0; i<armMuscles.size(); i++){
         PolygonalMesh mesh = armMuscles.get(i).getMesh();
         String strFile = armMuscleDir + mesh.getName() + objSuffix;
         System.out.println("Writing " + strFile);
         File file = new File(strFile);
         try {
            mesh.write(new PrintWriter(file), "%g");
         } catch (FileNotFoundException e) {
            e.printStackTrace();
         } catch (IOException e) {
            e.printStackTrace();
         }
      }
      
      String armBoneDir = wavefrontPath + armPath;
      File armBoneDirFile = new File(armBoneDir);
      armBoneDirFile.mkdirs(); // make directories if don't exist
      
      for (int i=0; i<armBones.size(); i++) {
         PolygonalMesh mesh = armBones.get(i).getMesh();
         String strFile = armBoneDir + mesh.getName() + objSuffix;
         System.out.println("Writing " + strFile);
         File file = new File(strFile);
         try {
            mesh.write(new PrintWriter(file), "%g");
         } catch (FileNotFoundException e) {
            e.printStackTrace();
         } catch (IOException e) {
            e.printStackTrace();
         }
      }
      
      
   }

   public void transformGeometry(ArrayList<RigidBody> rbList) {

      // scale down geometry
      double scale = 0.1;
      // center of mass when scaled
      Vector3d CoM =
         new Vector3d(
            -192.75370819098796, -47.06579202423347, 1403.7163262310887);
      CoM.scale(-1.0); // subtract CoM
      CoM.scale(scale); // scale down by scale

      Vector3d translate = new Vector3d(0, 15, 0);
      CoM.add(translate);

      AffineTransform3d trans = new AffineTransform3d();
      trans.applyScaling(scale, scale, scale);
      trans.setTranslation(CoM); // add translation

      RigidTransform3d rigid = new RigidTransform3d();
      rigid.setRotation(new AxisAngle(0.4, 0, 1, 2.5));

      // CoM.transform(trans); // rotate translation vector too

      for (RigidBody rb : rbList) {
         rb.getMesh().transform(trans);
         rb.getMesh().transform(rigid);
      }

   }

   public void addLegBones() {
      try {
         File bonesFile = cmiss.get(legBonesFilename);
         addExSurfaceFromFile(
            bonesFile, legPath,defaultResolution, legBones);
      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }
   
   public void addArmBones() {
      try {
         File bonesFile = cmiss.get(armBonesFilename);
         addExSurfaceFromFile(
            bonesFile, armPath, defaultResolution, armBones);
      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }

   public void addLegMuscles() {
      try {
         File musclesFile = cmiss.get(legMusclesFilename);
         addExSurfaceFromFile(
            musclesFile, legPath, defaultResolution, legMuscles);
      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }
   
   public void addArmMuscles() {
      try {
         File musclesFile = cmiss.get(armMusclesFilename);
         addExSurfaceFromFile(
            musclesFile, armPath, defaultResolution, armMuscles);
      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }

   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);

      for (RigidBody rb : armBones) {
         rb.setRenderProps(boneRenderProps);
      }
      for (RigidBody rb : armMuscles) {
         rb.setRenderProps(muscleRenderProps);
      }

      for (RigidBody rb : legBones) {
         rb.setRenderProps(boneRenderProps);
      }
      for (RigidBody rb : legMuscles) {
         rb.setRenderProps(muscleRenderProps);
      }
      
   }

   public RigidBody generateCmissSurface(String meshName, String meshPath, int[] resolution)
      throws IOException {

      File exNodeFile = cmiss.get(meshPath + meshName + ".exnode");
      File exElemFile = cmiss.get(meshPath + meshName + ".exelem");

      ExParser parser = new ExParser();

      // parse file
      System.out.println("================================");
      System.out.println("     Parsing " + meshName);
      System.out.println("================================");

      parser.parseExNode(exNodeFile);
      parser.parseExElem(exElemFile);

      // get last region
      ExRegion myRegion = parser.getLastParsedRegion();
      String name = myRegion.getName().replace("/", "");
      RigidBody rb = new RigidBody(name);
      
      // fix derivatives
      TangentFlipper flipper = new TangentFlipper();
      //flipper.fixTangents(myRegion.getElements(), -0.75);
      
      flipper.flipAllTangents(myRegion.getElements(), 2);      
  
      if (writeStuff) {
         File out = new File(fixedPath + meshPath + meshName +".exelem");
         out.getParentFile().mkdirs();
         
         PrintWriter elemWriter = new PrintWriter(out);
         ExWriter.writeElements(myRegion.getElements(), name, elemWriter, null);
         elemWriter.close();
         
         out = new File(fixedPath + meshPath + meshName + ".exnode");
         out.getParentFile().mkdirs();
         
         PrintWriter nodeWriter = new PrintWriter(out);
         ExWriter.writeNodes(myRegion.getNodes(), name, nodeWriter, null);
         nodeWriter.close();
      }

      PolygonalMesh mesh;
      try {
         ExMeshGenerator.useArclength = false;
         mesh =
            ExMeshGenerator.generateSurfaceMesh(
               myRegion.getElements(), resolution);
         mesh.setName(name);
         if (!mesh.isClosed()) {
           System.out.println("Warning: mesh "+rb.getName()+" is not closed");  
         }
         rb.setMesh(mesh, null);
         rb.setDynamic(false);
         myModel.addRigidBody(rb);         
      }
      catch (Exception e) {
         System.out.println("Error splitting element: " + rb.getName());
         // e.printStackTrace();
      }
      
      return rb;
   }

   public RigidBody addSurfaceFromObjFile(String objpath, String objname)
      throws IOException {

      String meshfilename = objpath + objname + ".obj";
      RigidBody rb = new RigidBody(objname);

      PolygonalMesh mesh = new PolygonalMesh(new File(meshfilename));
      mesh.setName(objname);
      rb.setMesh(mesh, meshfilename);
      rb.setDynamic(false);
      myModel.addRigidBody(rb);

      return rb;
   }

   public void addExSurfaceFromFile(File bodyList, String exPath,
      int[] resolution, ArrayList<RigidBody> rbList) throws IOException {

      ReaderTokenizer rtok = null;
      try {
         rtok = new ReaderTokenizer(new FileReader(bodyList));
         if (rtok.ttype == ReaderTokenizer.TT_EOF) {
            return;
         }
            
      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            rbList.add(generateCmissSurface(rtok.sval, exPath, resolution));
         }
      }
   }

   public void setArmVisible(boolean visible) {
      if (armVisible != visible) {
         armVisible = visible;
         updateVisibility();
      }
   }

   public boolean getArmVisible() {
      return armVisible;
   }

   public void setLegVisible(boolean visible) {
      if (legVisible != visible) {
         legVisible = visible;
         updateVisibility();
      }
   }

   public boolean getLegVisible() {
      return legVisible;
   }
   
   public void setBonesVisible(boolean visible) {
      if (bonesVisible != visible) {
         bonesVisible = visible;
         updateVisibility();
      }
   }

   public boolean getBonesVisible() {
      return bonesVisible;
   }

   public void updateVisibility() {
      for (RigidBody rb : armBones) {
         RenderProps.setVisible(rb, armVisible & bonesVisible);
      }
      for (RigidBody rb : armMuscles) {
         RenderProps.setVisible(rb, armVisible & musclesVisible);
      }
      for (RigidBody rb : legBones) {
         RenderProps.setVisible(rb, legVisible & bonesVisible);
      }
      for (RigidBody rb : legMuscles) {
         RenderProps.setVisible(rb, legVisible & musclesVisible);
      }
      
   }

}
