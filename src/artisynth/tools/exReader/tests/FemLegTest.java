package artisynth.tools.exReader.tests;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.tools.exReader.CmissFileManager;
import artisynth.tools.exReader.ExMeshGenerator;
import artisynth.tools.exReader.ExParser;
import artisynth.tools.exReader.ExRegion;
import artisynth.tools.femtool.FemTools;

public class FemLegTest extends RootModel {

   private static final String localPath = ArtisynthPath.getSrcRelativePath(
      FemLegTest.class, "data/cmiss/");
   CmissFileManager cmiss = new CmissFileManager(localPath);
   
   private static String legMusclesFilename = "legmuscles.list";
   private static String legBonesFilename = "legbones.list";
   private static String legPath = "right_leg/";

   private static int[] defaultRBres = { 3, 3, 3 };
   private static int[] defaultFEMres = { 1, 1, 1 };

   ArrayList<RigidBody> legBones;
   ArrayList<FemModel3d> muscleList;
   AffineTransform3d myTransform;

   ExParser myParser;

   RenderProps muscleRenderProps;
   RenderProps boneRenderProps;

   public static String rbpath =
      ArtisynthPath.getHomeRelativePath(
         "src/maspack/geometry/sampleData/", ".");

   RigidBody box0;
   double mu = 0.1;
   MechModel myModel;

   public FemLegTest () {
      super(null);
   }

   public FemLegTest (String name) throws Exception {

      super(name);
      myParser = new ExParser();
      myModel = new MechModel("model");

      boneRenderProps = new RenderProps();
      boneRenderProps.setFaceColor(new Color(208, 215, 185));
      boneRenderProps.setVisible(true);
      boneRenderProps.setFaceStyle(FaceStyle.FRONT_AND_BACK);

      muscleRenderProps = new RenderProps();
      muscleRenderProps.setFaceColor(new Color(165, 66, 56));
      muscleRenderProps.setVisible(true);
      muscleRenderProps.setFaceStyle(FaceStyle.FRONT_AND_BACK);

      legBones = new ArrayList<RigidBody>();

      myModel.setIntegrator(Integrator.ConstrainedBackwardEuler);

      // disable adaptive time step
      this.setAdaptiveStepping(true);
      addTable();
      setTransform();
      addMuscles();
      addBones();

      addModel(myModel);

      // mechmod.setIntegrator (Integrator.BackwardEuler);
      // addBreakPoint (0.29);
      // reset();

   }

   public void addBones() {
      try {
         File bonesFile = cmiss.get(legBonesFilename); 
         addMeshesFromFile(bonesFile, legBones, boneRenderProps);
      } catch (IOException e) {
         e.printStackTrace();
      }

      // transform
      for (RigidBody rb : legBones) {
         rb.transformGeometry(myTransform);
      }

      setCollisions(legBones, 0);

   }

   public void setTransform() {
      Vector3d CoM = new Vector3d(-192.75370819098796, -47.06579202423347,
         1403.7163262310887);
      myTransform = new AffineTransform3d();
      double scale = 0.01;
      myTransform.applyScaling(scale, scale, scale);
      CoM.scale(scale);
      myTransform.setTranslation(new Vector3d(-CoM.x, -CoM.y, -CoM.z + 8));
   }

   public void addMuscles() {

      try {
         File musclesFile = cmiss.get(legMusclesFilename);
         muscleList = addFemFromFile(musclesFile, muscleRenderProps);
      } catch (IOException e) {
         e.printStackTrace();
         return;
      }

      // center
      // Vector3d CoM = computeCoM(muscleList);
      for (FemModel3d fem : muscleList) {
         fem.transformGeometry(myTransform);
      }

      setCollisions(muscleList);

   }

   public Vector3d computeCoM(ArrayList<FemModel3d> femList) {
      Vector3d CoM = new Vector3d(0, 0, 0);
      double mass = 0;
      for (FemModel3d fem : femList) {
         fem.updateVolume();
         for (FemElement3d elem : fem.getElements()) {
            Point3d pnt = FemTools.getCentreOfMass(elem);
            mass += elem.getMass();
            CoM.scaledAdd(elem.getMass(), pnt);
         }
      }
      CoM.scale(1.0 / mass);

      return CoM;
   }

   public void setCollisions(ArrayList<FemModel3d> femList) {

      RigidBody table = myModel.rigidBodies().get("table");

      for (int i = 0; i < femList.size(); i++) {
         myModel.setCollisionBehavior(femList.get(i), table, true, mu);
      }

   }

   // dummy used so doesn't have same signature
   public void setCollisions(ArrayList<RigidBody> rbList, int dummy) {

      RigidBody table = myModel.rigidBodies().get("table");

      for (int i = 0; i < rbList.size(); i++) {
         myModel.setCollisionBehavior(rbList.get(i), table, true, mu);

         // for (int j = 0; j<i-1; j++) {
         // myModel.setCollisionBehavior(femList.get(i), femList.get(j), true);
         // }

      }

   }

   public void addMeshesFromFile(File surfaceFile, ArrayList<RigidBody> rbList,
      RenderProps props) throws IOException {

      ReaderTokenizer rtok = null;
      try {
         rtok = new ReaderTokenizer(new FileReader(surfaceFile));
      } catch (FileNotFoundException e) {
         e.printStackTrace();
      }

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {

            String name = rtok.sval;
            System.out.println("   Creating Surface: " + name);

            File nodeFile = cmiss.get(legPath + name + ".exnode");
            File elemFile = cmiss.get(legPath + name + ".exelem");

            myParser.parseExNode(nodeFile);
            myParser.parseExElem(elemFile);

            // get the recently read region
            ExRegion lastRegion = myParser.getLastParsedRegion();

            // build RB
            RigidBody rb = new RigidBody(rtok.sval);
            PolygonalMesh mesh;
            try {
               mesh = ExMeshGenerator.generateSurfaceMesh(
                  lastRegion.getElements(), defaultRBres);
            } catch (Exception e) {
               e.printStackTrace();
               continue;
            }
            mesh.triangulate();

            if (!mesh.isClosed()) {
               System.out.println("Warning: mesh " + rb.getName() + " is open");
            }

            rb.setMesh(mesh, rtok.sval);
            rb.setDynamic(true);
            rbList.add(rb);

            // set properties
            rb.setRenderProps(props);
            myModel.addRigidBody(rb);
         }
      }
   }

   public ArrayList<FemModel3d> addFemFromFile(File femList,
      RenderProps props) throws IOException {

      ArrayList<FemModel3d> myFEMs = new ArrayList<FemModel3d>();

      ReaderTokenizer rtok = null;
      try {
         rtok = new ReaderTokenizer(new FileReader(femList));
      } catch (FileNotFoundException e) {
         e.printStackTrace();
         return myFEMs;
      }

      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            File nodeFile = cmiss.get(legPath + rtok.sval + ".exnode");
            File elemFile = cmiss.get(legPath + rtok.sval + ".exelem");

            myParser.parseExNode(nodeFile);
            myParser.parseExElem(elemFile);

            // get the recently read region
            ExRegion lastRegion = myParser.getLastParsedRegion();

            // build FEM
            String name = lastRegion.getName();
            name = name.replace("/", "");
            System.out.println("   Creating FEM: " + name);

            try {

               ExMeshGenerator.useArclength = true;
               ExMeshGenerator.useLinearInternally = false;
               FemModel3d fem =
                  ExMeshGenerator.generateLinearFEM(
                     lastRegion.getElements(), defaultFEMres, 1.0);

               // FemModel3d fem =
               // ExMeshGenerator.generateLinearFEM(
               // lastRegion.getElements(), 1.0);
               fem.setName(name);

               for (FemElement3d elem : fem.getElements()) {
                  if (elem.isInvertedAtRest()) {
                     System.out.println("Warning: " + fem.getName()
                        + " has an inverted element at rest, " + elem.getName()
                        + "(" + elem.getNumber() + ")");
                  }
               }

               myFEMs.add(fem);

               // set properties
               fem.setRenderProps(props);
               fem.setSurfaceRendering(SurfaceRender.Shaded);
               RenderProps.setDrawEdges(fem, false);
               RenderProps.setVisible(fem.getElements(), true);
               RenderProps.setVisible(fem.getNodes(), true);
               RenderProps.setVisible(fem, true);

               fem.setLinearMaterial (1000, 0.33, true);
               fem.setParticleDamping(0.1);

               myModel.addModel(fem);

            } catch (Exception e) {
               e.printStackTrace();
               return myFEMs;
            }

         }
      }

      return myFEMs;
   }

   public void addTable() {
      RigidBody table = new RigidBody("table");
      table.setDynamic(false);
      try {
         table.setMesh(new PolygonalMesh(new File(rbpath + "box.obj")), null);
      } catch (IOException e) {
         e.printStackTrace();
      }
      AffineTransform3d trans = new AffineTransform3d();
      trans.setIdentity();
      trans.applyScaling(10, 5, 0.5);
      table.transformGeometry(trans);
      table.setPose(
         new RigidTransform3d(
            new Vector3d(1, 0, -7),
            new AxisAngle(1, 0, 0, Math.toRadians(mu == 0 ? 0.0 : 1.5))));
      myModel.addRigidBody(table);
   }

   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);

//      setViewerEye(new Point3d(1, 25, 10));
//      setViewerCenter(new Point3d(1, 0, 5));
   }

   public synchronized StepAdjustment advance(
      double t0, double t1, int flags) {
      StepAdjustment rcode = super.advance(t0, t1, flags);
      return rcode;
   }

}
