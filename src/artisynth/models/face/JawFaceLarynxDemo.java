package artisynth.models.face;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.NumericProbePanel;
import artisynth.core.materials.AxialMuscleMaterial;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearAxialMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MuscleMaterial;
import artisynth.core.materials.PaiAxialMuscle;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.AmiraLandmarkReader;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.vkhUpperAirway.VKHUpperAirwayDemo;

public class JawFaceLarynxDemo extends BadinJawTongueFaceTension {

   public String rigidBodyPath = ArtisynthPath.getSrcRelativePath(
         VKHUpperAirwayDemo.class, "geometry/rigidBodies/");
   public String femPath = ArtisynthPath.getSrcRelativePath(
         VKHUpperAirwayDemo.class, "geometry/fem/");
   public String femListPath = ArtisynthPath.getSrcRelativePath(
         JawFaceLarynxDemo.class, "geometry/");
   public String otherPath = ArtisynthPath.getSrcRelativePath (
         VKHUpperAirwayDemo.class, "geometry/other/");
   public String femListAllFilename = "femList.txt";
   public String femAttachParticleFilename = "";
   public String autoAttachListFilename = "autoAttachList.txt";
   public String collisionListFilename = "collision.txt";
   protected String frameMarkerListFilename = "";
   protected String muscleSpringListFilename = "";
   protected String muscleSpringPropertyListFilename = "";
   protected String springListFilename = "";
   protected String springPropertyListFilename = "";
   protected String femBundleSpringListFilename = "femBundleSpringList.txt";

   protected String femMarkerListFilename = "";

   protected ArrayList<MuscleInfo> muscleList = new ArrayList<MuscleInfo>();
   protected ArrayList<AutoAttachInfo> autoAttachInfoList = new ArrayList<AutoAttachInfo>();
   protected ArrayList<MuscleSpringInfo> muscleSpringInfoList =
         new ArrayList<MuscleSpringInfo>();
   protected ArrayList<SpringInfo> springInfoList = new ArrayList<SpringInfo>();
   protected ArrayList<String> muscleSpringList = new ArrayList<String>();
   protected ArrayList<String> muscleSpringGroupList = new ArrayList<String>();
   protected ArrayList<MuscleExciterInfo> muscleExciterList =
         new ArrayList<MuscleExciterInfo>();
   protected ArrayList<CollisionInfo> collisionInfoList =
         new ArrayList<CollisionInfo>();

   protected int bundleCount;
   protected boolean useElementsInsteadOfFibres = false;
   protected boolean fiberDefinedAcrossSets = true;
   protected boolean SetCollision = true;

   public double MUSCLE_DENSITY = 1.112E-6;
   public double MUSCLE_PARTICLE_DAMPING = 0.5;
   public double MUSCLE_STIFFNESS_DAMPING = 0.5;
   public double MUSCLE_YOUNGS_MODULUS = 50000;
   public double MUSCLE_DAMPING = 0;
   public double MUSCLE_MAXLAMBDA = 2.1;
   public double MUSCLE_MAXSTRESS = 3e5;
   public double MUSCLE_MAX_FORCE = 5;
   public double MUSCLE_PASSIVE_FRACTION = 0;
   public double MUSCLE_FORCE_SCALING = 1000;
   public double MUSCLE_MAX_FORCE_SCALING = 1;
   public double MUSCLE_TENDON_RATIO = 0;
   public double SPRING_MUSCLE_FORCE_SCALING = 1000;
   public double SPRING_MUSCLE_DAMPING = 0;
   public double SPRING_DAMPING = 0;
   public double SPRING_STIFFNESS = 0;
   public double SPRING_MUSCLE_MAX_FORCE = 1;
   public double SPRING_MUSCLE_PASSIVE_FRACTION = 0;
   public double SPRING_MUSCLE_TENDON_RATIO = 0;
   public double COLLISION_FRICTION_COEFF = 0.0;

   public String MUSCLE_FIBRE_TYPE = "Peck";

   public boolean IncompressOption = false;
   protected boolean centerRigidBodies = false;
   protected RigidBody block;

   public FemMaterial FEM_MATERIAL = new LinearMaterial(24.7, 0.47);

   public MuscleMaterial MUSCLE_MATERIAL = new BlemkerMuscle(MUSCLE_MAXLAMBDA,
         1.0, MUSCLE_MAXSTRESS, 0.00005, 6.6);

   public double elementWedgeSize = 0;

   public boolean muscleControlPanel = false;

   public boolean drawBundle = true;
   public boolean drawBundleExcitation = false;
   public boolean drawNodes = false;
   public boolean drawAttachedNodes = false;
   public double pointRadius = 0.5;
   public double elementDirectionRenderLen = 0.5;
   public double addMidElementsWithin = 5;
   public int lineWidth = 1;
   public double lineRadius = 0.5;
   public boolean muscleShaded = true;
   protected boolean groupExciters = true;

   protected HashMap<RigidBody, Vector3d> bodyTranslationMap= new HashMap<RigidBody, Vector3d>();

   public double xAngle = 90.0;
   public double yAngle = -10.0;

   RigidTransform3d X = new RigidTransform3d(0.0, 0.0, 0.0,
         Math.toRadians(xAngle), 0.0, Math.toRadians(yAngle));

   RigidTransform3d Y = new RigidTransform3d(202.547, 133.893, 286.134, 0.0,
         0.0, 0.0);

   public JawFaceLarynxDemo() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      addRigidBody("Thyroid", "Thyroid.obj");
      addRigidBody("Cricoid", "Cricoid.obj");
      addRigidBody("Vertebrae", "Vertebrae.obj");

      RigidBody body = myJawModel.rigidBodies().get("Thyroid");
      body.transformGeometry(X);
      body.transformGeometry(Y);

      body = myJawModel.rigidBodies().get("Cricoid");
      body.transformGeometry(X);
      body.transformGeometry(Y);

      body = myJawModel.rigidBodies().get("Vertebrae");
      body.transformGeometry(X);
      body.transformGeometry(Y);

      muscleList = readStringList(femListAllFilename);

      for (MuscleInfo mu : muscleList) {
         // Add mesh
         addMuscle(mu);
        FemMuscleModel femMuscle = (FemMuscleModel) myJawModel.models().get(
               mu.name);
         femMuscle.transformGeometry(X);
         femMuscle.transformGeometry(Y);
         attachMuscle(mu);
         attachMuscleToMuscle(mu);
//         defineMuscleFibre(mu);
      }

      // ###################################################################
      // ###### Set attachments ##############################################
      autoAttachInfoList = readAutoAttachInfoList(autoAttachListFilename);
      autoAttach();
      // ###################################################################

      //###################################################################
      //###### Add springs ################################################
      addFemBundleSpring (femBundleSpringListFilename);
      
      //###################################################################

      if(groupExciters) {
         groupFemExciters();
      }
      
      //###################################################################
      //###### Set collision ##############################################
      if (SetCollision) {
         collisionInfoList = readCollisionInfoList(collisionListFilename);
         setCollision();
      }
            
   }

   private void addMuscle(MuscleInfo mu) throws IOException {

      String name = mu.name;
      String meshName = mu.meshName;
      Color color = mu.color;

      FemMuscleModel femMuscle = new FemMuscleModel(name);
      // Read mesh from files
      AnsysReader.read(femMuscle, femPath + meshName + ".node", femPath
            + meshName + ".elem", 1, null, /* options= */0);
      
      // Set material properties
      femMuscle.scaleDistance(mu.scale);
      if (!Double.isNaN(mu.density)) {
         femMuscle.setDensity(mu.density);
      } else if (!Double.isNaN(mu.mass)) {
         femMuscle.updateVolume();
         femMuscle.setDensity(mu.mass / femMuscle.getVolume());
      } else {
         femMuscle.setDensity(MUSCLE_DENSITY);
      }
      if (!Double.isNaN(mu.particleDamping)) {
         femMuscle.setParticleDamping(mu.particleDamping);
      } else {
         femMuscle.setParticleDamping(MUSCLE_PARTICLE_DAMPING);
      }
      if (!Double.isNaN(mu.stiffnessDamping)) {
         femMuscle.setStiffnessDamping(mu.stiffnessDamping);
      } else {
         femMuscle.setStiffnessDamping(MUSCLE_STIFFNESS_DAMPING);
      }
//      if (!Double.isNaN(mu.youngsModulus)) {
//         femMuscle.setYoungsModulus(mu.youngsModulus);
//      } else {
//         femMuscle.setYoungsModulus(MUSCLE_YOUNGS_MODULUS);
//      }
      // Set material type
      if (mu.material != null) {
         femMuscle.setMaterial(mu.material);
      } else {
         femMuscle.setMaterial(FEM_MATERIAL);
         // femMuscle.setMaterial (new LinearMaterial());
      }
      // Set muscle material properties
      if (mu.muscleMaterial != null) {
         femMuscle.setMuscleMaterial(mu.muscleMaterial);
      } else {
         /*
          * GenericMuscle mm = new GenericMuscle(); mm.setMaxStress
          * (MUSCLE_MAXSTRESS); mm.setMaxLambda(MUSCLE_MAXLAMBDA);
          */
         femMuscle.setMuscleMaterial(MUSCLE_MATERIAL);
      }

      if (mu.incompressible) {
         femMuscle.setIncompressible(IncompMethod.AUTO);
      } else {
         femMuscle.setIncompressible(IncompMethod.OFF);
      }

      femMuscle.setImplicitIterations(100);
      femMuscle.setImplicitPrecision(0.001);
      // femMuscle.setMaxStepSize (10 * TimeBase.MSEC);

      // Set Rendering properties

      if (muscleShaded && elementWedgeSize == 0) {
         femMuscle.setSurfaceRendering(SurfaceRender.Shaded);
      } else {
         if (muscleShaded && elementWedgeSize > 0) {
            femMuscle.setElementWidgetSize(elementWedgeSize);
         }
         femMuscle.setSurfaceRendering(SurfaceRender.None);
      }
      RenderProps.setFaceStyle(femMuscle, Renderer.FaceStyle.FRONT);
      RenderProps.setShading(femMuscle, Renderer.Shading.FLAT);
      // RenderProps.setShading (femMuscle, Renderer.Shading.SMOOTH);
      RenderProps.setFaceColor(femMuscle, color);
      RenderProps.setLineWidth(femMuscle, lineWidth);
      RenderProps.setLineColor(femMuscle, Color.PINK);
      RenderProps.setPointSize(femMuscle, 0);
      RenderProps.setPointRadius(femMuscle, pointRadius);
      RenderProps.setPointColor(femMuscle, Color.WHITE);
      if (!drawNodes) {
         RenderProps.setPointStyle(femMuscle, Renderer.PointStyle.POINT);
      } else {
         RenderProps.setPointStyle(femMuscle, Renderer.PointStyle.SPHERE);
      }

      // highlight inverted elements
      for (FemElement3d e : femMuscle.getElements()) {
         if (e.isInvertedAtRest()) {
            e.computeVolumes();
            System.out.println(" -Inverted Element " + name + "/elements/"
                  + e.getNumber() + ", v(" + e.getVolume() + ")");
            RenderProps.setLineWidth(e, lineWidth * 5);
            RenderProps.setLineColor(e, Color.RED);
         }
      }

      myJawModel.addModel(femMuscle);

   }

   private void addRigidBody(String name, String meshName) {

      RigidBody body = new RigidBody(name);

      String fullMeshFileName = rigidBodyPath + meshName;
      setBodyMesh(body, fullMeshFileName, 1.0);
      body.setDynamic(false);
      myJawModel.addRigidBody(body);

   }

   private void setBodyMesh(RigidBody body, String meshname, double scale) {
      try {
         PolygonalMesh mesh = new PolygonalMesh(new File(meshname));
         mesh.scale(scale);
         body.setMesh(mesh, meshname);

      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   private ArrayList<MuscleInfo> readStringList(String filename) {
      ArrayList<MuscleInfo> muscleList = new ArrayList<MuscleInfo>();
      MuscleInfo mu;
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(femListPath
               + filename));

         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            if (rtok.ttype != ReaderTokenizer.TT_WORD) {
               throw new IOException("readMarkerList Expecting word, got "
                     + rtok.tokenName());
            }
            mu = new MuscleInfo();
            mu.scan(rtok);
            muscleList.add(mu);
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + femPath + filename
                  + " does not exist!");
      }
      return muscleList;
   }

   private class MuscleInfo {
      public String name;

      public String meshName;

      public double scale;

      public Color color;
      public double mass = Double.NaN, density = Double.NaN,
            particleDamping = Double.NaN, stiffnessDamping = Double.NaN,
            youngsModulus = Double.NaN;

      public FemMaterial material;

      public MuscleMaterial muscleMaterial;

      public boolean incompressible = IncompressOption;
      
      public void scan(ReaderTokenizer rtok) throws IOException {
         name = rtok.sval;
         rtok.nextToken();
         meshName = rtok.sval;
         rtok.nextToken();
         rtok.nextToken();
         scale = rtok.nval;
         rtok.nextToken();
         int r = (int) rtok.nval;
         rtok.nextToken();
         int g = (int) rtok.nval;
         rtok.nextToken();
         int b = (int) rtok.nval;
         color = new Color(r, g, b);
      }
   }

   // ########################################################################
   // #### Subfunction: attach muscle to rigidBody
   // ########################################################################
   private void attachMuscle(MuscleInfo mu) throws IOException {
      String name = mu.name;
      String meshName = mu.meshName;
      FemMuscleModel muscleModel = (FemMuscleModel) myJawModel.models().get(
            name);
      if (debug)
         System.out.println("Attaching " + name + " Nodes");
      // Read list from file
      ArrayList<MuscleAttachInfo> attachList = readMuscleAttachmentList(meshName);
      // Attach nodes
      for (MuscleAttachInfo targetAttach : attachList) {
         RigidBody bi = myJawModel.rigidBodies().get(targetAttach.target);
         if (bi == null || targetAttach.target.equals("fix")) {
            if (!targetAttach.target.equals("fix") && debug)
               System.out
               .println("Warning: Cannot attach nodes to RigidBody \""
                     + targetAttach.target + "\", it does not exist.");
            if (debug)
               System.out.println("Fixing Nodes, # nodes = "
                     + targetAttach.list.length);
            for (int i : targetAttach.list) {
               FemNode3d n = muscleModel.getNodes().get(i);
               n.setDynamic(false);
               if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
                  RenderProps.setPointStyle(muscleModel.getNode(i),
                        Renderer.PointStyle.SPHERE);
                  RenderProps.setPointRadius(muscleModel.getNode(i),
                        pointRadius * 1.5);
                  RenderProps.setPointColor(muscleModel.getNode(i),
                        Color.DARK_GRAY);
               }
            }
         } else {
            if (debug)
               System.out.println("Attaching Nodes to " + bi.getName()
                     + ", # nodes = " + targetAttach.list.length);

            for (int i : targetAttach.list) {
               FemNode3d n = muscleModel.getNodes().get(i);
               if (debug)
                  System.out.println("\tattaching node " + n.getNumber()
                        + " to " + bi.getName());
               
               myJawModel.attachPoint(n, bi);
               if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
                  RenderProps.setPointStyle(muscleModel.getNode(i),
                        Renderer.PointStyle.SPHERE);
                  RenderProps.setPointRadius(muscleModel.getNode(i),
                        pointRadius * 1.5);
                  RenderProps.setPointColor(muscleModel.getNode(i), Color.BLUE);
               }
            }
         }
      }
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunction: attach muscle to rigidBody
   // ########################################################################
   private void attachMuscleToMuscle(MuscleInfo mu) throws IOException {
      String name = mu.name;
      String meshName = mu.meshName;
      FemMuscleModel muscleModel = (FemMuscleModel) myJawModel.models().get(
            name);
      if (debug)
         System.out.println("Attaching " + name + " Nodes");
      // Read list from file
      ArrayList<MuscleAttachInfo> attachList = readMuscleToMuscleAttachmentList(meshName);
      // Attach nodes
      for (MuscleAttachInfo targetAttach : attachList) {
         FemModel3d bi = (FemModel3d) myJawModel.models().get(
               targetAttach.target);
         if (bi == null || targetAttach.target.equals("fix")) {
            if (!targetAttach.target.equals("fix") && debug)
               System.out
               .println("Warning: Cannot attach nodes to RigidBody \""
                     + targetAttach.target + "\", it does not exist.");
            if (debug)
               System.out.println("Fixing Nodes, # nodes = "
                     + targetAttach.list.length);
            for (int i : targetAttach.list) {
               FemNode3d n = muscleModel.getNodes().get(i);
               n.setDynamic(false);
               if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
                  RenderProps.setPointStyle(muscleModel.getNode(i),
                        Renderer.PointStyle.SPHERE);
                  RenderProps.setPointRadius(muscleModel.getNode(i),
                        pointRadius * 1.5);
                  RenderProps.setPointColor(muscleModel.getNode(i),
                        Color.DARK_GRAY);
               }
            }
         } else {
            if (debug)
               System.out.println("Attaching Nodes to " + bi.getName()
                     + ", # nodes = " + targetAttach.list.length);

            boolean invert = false;
            FemNode3d node;
            Point3d orig_pos = new Point3d();

            for (int i : targetAttach.list) {
               node = muscleModel.getNodes().get(i);
               orig_pos.set(node.getPosition());
               invert = false;

               if (debug)
                  System.out.println("\tattaching node " + node.getNumber()
                        + " to " + bi.getName());

               myJawModel.attachPoint(node, bi);

               // Reverse node attachment if results in inverted elements
               for (FemElement3d el : node.getElementDependencies()) {
                  if (el.computeVolumes() < 0) {
                     invert = true;
                  }
               }
               if (invert) {
                  myJawModel.detachPoint(node);
                  node.setPosition(orig_pos);
                  for (FemElement3d el : node.getElementDependencies()) {
                     if (el.computeVolumes() < 0) {
                        System.out.println("Warning: inverted element "
                              + el.getNumber());
                     }
                  }
               } else if (drawAttachedNodes) {
                  RenderProps.setPointRadius(node, pointRadius);
                  RenderProps
                  .setPointStyle(node, Renderer.PointStyle.SPHERE);
                  RenderProps.setPointColor(node, Color.GREEN);
               }
            }
         }
      }
      muscleModel.resetRestPosition();
   }

   // ########################################################################
   private ArrayList<MuscleAttachInfo> readMuscleAttachmentList(String filename) {
      ArrayList<MuscleAttachInfo> muscleAttachList = new ArrayList<MuscleAttachInfo>();
      try {
         MuscleAttachInfo mu;
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(femListPath
               + filename + ".attach"));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            if (rtok.ttype == ReaderTokenizer.TT_WORD) {
               mu = new MuscleAttachInfo();
               mu.target = rtok.sval;
               if (debug)
                  System.out.println("Reading attachments to " + mu.target);
               if (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
                  throw new IOException(
                        "Muscle Attachment File incorrect format: " + filename
                        + ".attach");
               mu.list = new int[(int) rtok.nval];
               int i = 0;
               while (rtok.nextToken() == ReaderTokenizer.TT_NUMBER) {
                  int n = (int) rtok.nval;
                  mu.list[i] = n;
                  i++;
               }
               muscleAttachList.add(mu);
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + femListPath + filename
                  + ".attach does not exist!");
      }
      return muscleAttachList;
   }

   private class MuscleAttachInfo {
      public String target;

      public int[] list;
   }

   private ArrayList<MuscleAttachInfo> readMuscleToMuscleAttachmentList(
         String filename) {
      ArrayList<MuscleAttachInfo> muscleAttachList = new ArrayList<MuscleAttachInfo>();
      try {
         MuscleAttachInfo mu;
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(femListPath
               + filename + ".femAttach"));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            if (rtok.ttype == ReaderTokenizer.TT_WORD) {
               mu = new MuscleAttachInfo();
               mu.target = rtok.sval;
               if (debug)
                  System.out.println("Reading attachments to " + mu.target);
               if (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
                  throw new IOException(
                        "Muscle Attachment File incorrect format: " + filename
                        + ".attach");
               mu.list = new int[(int) rtok.nval];
               int i = 0;
               while (rtok.nextToken() == ReaderTokenizer.TT_NUMBER) {
                  int n = (int) rtok.nval;
                  mu.list[i] = n;
                  i++;
               }
               muscleAttachList.add(mu);
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + femPath + filename
                  + ".attach does not exist!");
      }
      return muscleAttachList;
   }

   // ########################################################################
   // #### Subfunction: attach fem particles
   // ########################################################################
   // private void attachParticles(String filename) throws IOException {
   // try {
   // ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(otherPath
   // + filename));
   // FemMuscleModel fem1, fem2;
   // int node1, node2;
   // String femName1, femName2;
   // while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
   // if (rtok.ttype != ReaderTokenizer.TT_WORD)
   // throw new IOException("Frame Marker File incorrect format: "
   // + filename);
   // femName1 = rtok.sval;
   // rtok.nextToken();
   // femName2 = rtok.sval;
   // fem1 = (FemMuscleModel) myMechMod.models().get(femName1);
   // fem2 = (FemMuscleModel) myMechMod.models().get(femName2);
   // if (femName1 != "end" && femName2 != "end") {
   // if (debug)
   // System.out.println("Attaching nodes between \"" + femName1
   // + "\" and \"" + femName2 + "\":");
   // while (rtok.nextToken() != ReaderTokenizer.TT_WORD) {
   // node1 = (int) rtok.nval;
   // rtok.nextToken();
   // node2 = (int) rtok.nval;
   // if (fem1 != null && fem2 != null) {
   // myMechMod.attachPoint(fem1.getNode(node1),
   // fem2.getNode(node2));
   // if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
   // RenderProps.setPointRadius(fem1.getNode(node1),
   // pointRadius);
   // RenderProps.setPointStyle(fem1.getNode(node1),
   // Renderer.PointStyle.SPHERE);
   // RenderProps.setPointColor(fem1.getNode(node1),
   // Color.GREEN);
   // RenderProps.setPointRadius(fem2.getNode(node2),
   // pointRadius);
   // RenderProps.setPointStyle(fem2.getNode(node2),
   // Renderer.PointStyle.SPHERE);
   // RenderProps.setPointColor(fem2.getNode(node2),
   // Color.GREEN);
   // }
   // if (debug)
   // System.out.println("\tNode " + node1 + " ==> Node "
   // + node2);
   // }
   // }
   // }
   // }
   //
   // } catch (IOException e) {
   // if (debug)
   // System.out.println("Warning: File " + otherPath + filename
   // + " does not exist!");
   // }
   // }
   // ########################################################################
   private class AutoAttachInfo {
      public String[] type = new String[2];
      public String[] name = new String[2];
      public double distance = 0;

      public void scan(ReaderTokenizer rtok) throws IOException {
         type[0] = rtok.sval;
         rtok.nextToken();
         name[0] = rtok.sval;
         rtok.nextToken();
         type[1] = rtok.sval;
         rtok.nextToken();
         name[1] = rtok.sval;
         rtok.nextToken();
         distance = rtok.nval;
      }
   }

   // ########################################################################
   // #### Subfunctions: read lists from files
   // ########################################################################
   // private ArrayList<CollisionInfo> readCollisionInfoList (String filename)
   // {
   // ArrayList<CollisionInfo> collisionInfoList =
   // new ArrayList<CollisionInfo>();
   // try {
   // ReaderTokenizer rtok = new ReaderTokenizer(
   // new FileReader(otherPath + filename));
   // rtok.wordChars (".");
   // CollisionInfo bi;
   // while (rtok.nextToken () != ReaderTokenizer.TT_EOF)
   // {
   // bi = new CollisionInfo ();
   // bi.scan (rtok);
   // collisionInfoList.add (bi);
   // }
   // rtok.close();
   // } catch (IOException e) {
   // if(debug)
   // System.out.println("Warning: File " + otherPath + filename
   // + " does not exist!");
   // }
   // return collisionInfoList;
   // }
   private ArrayList<AutoAttachInfo> readAutoAttachInfoList(String filename) {
      ArrayList<AutoAttachInfo> AutoAttachInfoList = new ArrayList<AutoAttachInfo>();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(femListPath
               + filename));
         rtok.wordChars(".");
         AutoAttachInfo bi;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            bi = new AutoAttachInfo();
            bi.scan(rtok);
            AutoAttachInfoList.add(bi);
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + femListPath + filename
                  + " does not exist!");
      }
      return AutoAttachInfoList;
   }

   // ########################################################################
   // #### Subfunction: attach objects automatically
   // ########################################################################
   private void autoAttach() throws IOException {
      for (AutoAttachInfo autoAttachInfo : autoAttachInfoList) {
         // Find first model
         if (autoAttachInfo.type[0].matches("fem")
               && autoAttachInfo.type[1].matches("fem")) {
            FemModel3d model1 = (FemModel3d) myJawModel.models().get(
                  autoAttachInfo.name[0]);
            FemModel3d model2 = (FemModel3d) myJawModel.models().get(
                  autoAttachInfo.name[1]);
            if (model1 != null && model2 != null) {
               attachFemToFem(model1, model2, autoAttachInfo.distance);
               if (debug == true)
                  System.out.println("Automatically attaching ("
                        + autoAttachInfo.name[0] + ", "
                        + autoAttachInfo.name[1] + ")");
            }
         } else if (autoAttachInfo.type[0].matches("fem")
               && autoAttachInfo.type[1].matches("rigidBody")) {
            FemModel3d model1 = (FemModel3d) myJawModel.models().get(
                  autoAttachInfo.name[0]);
            RigidBody model2 = myJawModel.rigidBodies().get(
                  autoAttachInfo.name[1]);
            if (model1 != null && model2 != null) {
               attachFemToRigidBody(model1, model2, autoAttachInfo.distance);
               if (debug == true)
                  System.out.println("Automatically attaching ("
                        + autoAttachInfo.name[0] + ", "
                        + autoAttachInfo.name[1] + ")");
            }
         } else if (autoAttachInfo.type[0].matches("rigidBody")
               && autoAttachInfo.type[1].matches("fem")) {
            RigidBody model1 = myJawModel.rigidBodies().get(
                  autoAttachInfo.name[0]);
            FemModel3d model2 = (FemModel3d) myJawModel.models().get(
                  autoAttachInfo.name[1]);
            if (model1 != null && model2 != null) {
               attachFemToRigidBody(model2, model1, autoAttachInfo.distance);
               if (debug == true)
                  System.out.println("Automatically attaching ("
                        + autoAttachInfo.name[0] + ", "
                        + autoAttachInfo.name[1] + ")");
            }
         } else {
            System.out.println("Warning: Incorrect file format ("
                  + autoAttachListFilename + "), attachments not set.");
         }
      }
   }

   // ########################################################################
   // ########################################################################
   // #### Subfunction: attach fem1 to fem2 automatically
   // ########################################################################
   private void attachFemToFem(FemModel3d fem1, FemModel3d fem2, double distance) {
      BVFeatureQuery query = new BVFeatureQuery();
      Point3d orig_pos = new Point3d();
      boolean invert = false;

      for (FemElement3d el : fem1.getElements()) {
         if (el.computeVolumes() < 0) {
            System.out.println("Warning: inverted element " + el.getNumber());
         }
      }

      for (FemNode3d node : fem1.getNodes()) {
         if (query.isInsideOrientedMesh (
                fem2.getSurfaceMesh(), node.getPosition(), distance)) {
            if (!node.isAttached()) {
               invert = false;
               orig_pos.set(node.getPosition());

               // Attach node to fem2
               myJawModel.attachPoint(node, fem2);

               // Reverse node attachment if results in inverted elements
               for (FemElement3d el : node.getElementDependencies()) {
                  if (el.computeVolumes() < 0) {
                     invert = true;
                  }
               }
               if (invert) {
                  myJawModel.detachPoint(node);
                  node.setPosition(orig_pos);
                  for (FemElement3d el : node.getElementDependencies()) {
                     if (el.computeVolumes() < 0) {
                        System.out.println("Warning: inverted element "
                              + el.getNumber());
                     }
                  }
               } else if (drawAttachedNodes) {
                  RenderProps.setPointRadius(node, pointRadius);
                  RenderProps
                  .setPointStyle(node, Renderer.PointStyle.SPHERE);
                  RenderProps.setPointColor(node, Color.GREEN);
               }
            }
         }
      }
      fem1.resetRestPosition();
   }

   // ########################################################################
   // #### Subfunction: attach fem to a rigidBody automatically
   // ########################################################################
   private void attachFemToRigidBody(FemModel3d fem, RigidBody rb,
         double distance) {
      Point3d proj = new Point3d();
      Vector2d coords = new Vector2d();
      BVFeatureQuery query = new BVFeatureQuery();

      for (FemNode3d node : fem.getNodes()) {
         query.nearestFaceToPoint(proj, coords, rb.getMesh(), node.getPosition());

         if (proj.distance(node.getPosition()) < distance) {
            if (!node.isAttached()) {
               myJawModel.attachPoint(node, rb);
               // System.out.println("\t"+node.myNumber);
               if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
                  RenderProps
                  .setPointStyle(node, Renderer.PointStyle.SPHERE);
                  RenderProps.setPointColor(node, Color.BLUE);
                  RenderProps.setPointRadius(node, pointRadius * 1.5);
               }
            }
         }
      }
   }

   // ========================================================================

   private class MuscleSpringInfo
   {
      public String name;
      public String muscleType;
      public double damping = Double.NaN, maxForce = Double.NaN, 
       optLength = Double.NaN, maxLength = Double.NaN,
       tendonRatio = Double.NaN, passiveFraction = Double.NaN,
       muscleForceScaling = Double.NaN;

      public MuscleMaterial muscleMaterial;

      public Color bundleColor;
      
      public MuscleSpringInfo(String argName) {
    name = argName;
      }
   }
   private class SpringInfo
   {
   }

   //########################################################################
   //#### Subfunction: add fem bundle springs
   //########################################################################
   private void addFemBundleSpring(String filename) throws IOException {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
               new FileReader(femListPath + filename));
         rtok.wordChars (".");
         rtok.wordChars ("-");
         String bundleName, landmarkFileName;
         FemMuscleModel muscleModel;
         String individualBundleControl;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype == ReaderTokenizer.TT_WORD)
            {
               muscleModel = (FemMuscleModel) myJawModel.models().get(rtok.sval);
               while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
                  bundleName = rtok.sval;
                  if(muscleModel!=null && !bundleName.matches("(?i:end)")) {
                     rtok.nextToken ();
                     landmarkFileName = rtok.sval;
                     rtok.nextToken ();
                     individualBundleControl = rtok.sval;
                     if(individualBundleControl.matches ("-individualBundleControl")) {
                        addFemBundleFromAmiraFile(
                           muscleModel, bundleName, landmarkFileName, true);
                     } else {
                        rtok.pushBack ();
                        addFemBundleFromAmiraFile(
                           muscleModel, bundleName, landmarkFileName, false);
                     }
                  } else {
                     break;
                  }
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if(debug)
            System.out.println("Warning: File " + femListPath + filename
               + " does not exist!");
      }
   }
   //########################################################################
   
   // ########################################################################
   // #### Subfunction: add FemMarker from Amira file
   // ########################################################################
   public void addFemBundleFromAmiraFile(FemMuscleModel fem, String bundleName,
         String landmarkFileName, boolean individualBundleControl) {
      try {
         Point3d[][] pts = AmiraLandmarkReader.readSets(otherPath
               + landmarkFileName);
         if (pts == null) {
            return;
         }

         // Transform the coordinates of the points
         for (int n = 0; n < pts[0].length; n++) {
            for (int m = 0; m < pts.length; m++) {
               // Transform point first
               pts[m][n].transform(X);
               pts[m][n].transform(Y);
           }
         }
         
         Point3d[] curSetPts;
         MuscleBundle bundle = new MuscleBundle();
         FemMarker cur = null, prev = null;

         MuscleSpringInfo ms = null;
         for (MuscleSpringInfo muscleSpringInfo : muscleSpringInfoList) {
            if (muscleSpringInfo.name.matches(bundleName)) {
               ms = muscleSpringInfo;
            }
         }
         if (ms == null) {
            ms = new MuscleSpringInfo(bundleName);
            muscleSpringInfoList.add(ms);
         }
         if (fiberDefinedAcrossSets) {
            for (int j = 0; j < pts[0].length; j++) {
               if (individualBundleControl) {
                  bundle = new MuscleBundle();
                  // Set muscle material properties
                  if (ms.muscleMaterial != null) {
                     bundle.setMuscleMaterial(ms.muscleMaterial);
                  } else {
                     bundle.setMuscleMaterial(MUSCLE_MATERIAL);
                  }
                  bundle.setName(bundleName + "_" + j);
                  fem.addMuscleBundle(bundle);
                  bundle.setFibresActive(true);

                  Color color = null;
                  String name = bundleName.replaceAll("_L", "");
                  name = name.replaceAll("_R", "");
                  name = name + "_" + j;
                  for (int k = 0; k < muscleExciterList.size(); k++) {
                     if (name.matches(muscleExciterList.get(k).name)) {
                        color = muscleExciterList.get(k).color;
                     }
                  }
                  if (color == null || !groupExciters) {
                     if (bundleCount >= (NumericProbePanel.colorList.length))
                        ms.bundleColor = NumericProbePanel.colorList[bundleCount
                              % NumericProbePanel.colorList.length];
                     else
                        ms.bundleColor = NumericProbePanel.colorList[bundleCount];
                     bundleCount++;

                     if (groupExciters) {
                        muscleExciterList.add(new MuscleExciterInfo(name,
                              ms.bundleColor));
                        MuscleExciter exciter = new MuscleExciter(name);
                        exciter.addTarget(bundle, 1.0);
                        myJawModel.addMuscleExciter(exciter);
                     }
                  } else {
                     MuscleExciter exciter = myJawModel.getMuscleExciters().get(
                           name);
                     exciter.addTarget(bundle, 1.0);
                     ms.bundleColor = color;
                  }

                  if (useElementsInsteadOfFibres) {
                     RenderProps.setLineWidth(bundle, (int) (lineWidth * 3));
                     bundle.addElementsNearFibres(addMidElementsWithin);
                     bundle.setDirectionRenderLen(elementDirectionRenderLen);
                     bundle.getFibres().clear();
                     // bundle.setFibresActive(false);
                  } else {
                     RenderProps.setLineStyle(bundle, LineStyle.SPINDLE);
                     RenderProps.setLineRadius(bundle, lineRadius);
                  }
                  if (drawBundleExcitation) {
                     RenderProps.setLineColor(bundle, Color.WHITE);
                     bundle.setExcitationColor(ms.bundleColor);
                  } else {
                     RenderProps.setLineColor(bundle, ms.bundleColor);
                  }
                  if (drawBundle == false) {
                     RenderProps.setVisible(bundle, drawBundle);
                  }
               }
               for (int i = 0; i < pts.length; i++) {
                  if (pts[i][j] != null) {
                     cur = createAndAddMarker(fem, pts[i][j]);
                     if (i > 0)
                        createAndAddFibre(prev, cur, bundle, ms);
                     prev = cur;
                     if (cur != null && drawAttachedNodes != drawNodes
                           && drawAttachedNodes) {
                        RenderProps.setPointRadius(cur, pointRadius);
                        RenderProps.setPointStyle(cur,
                              Renderer.PointStyle.SPHERE);
                        RenderProps.setPointColor(cur, Color.GREEN);
                     }
                  }
               }
            }
         } else {
            for (int i = 0; i < pts.length; i++) {
               curSetPts = pts[i];
               if (individualBundleControl) {
                  bundle = new MuscleBundle();
                  // Set muscle material properties
                  if (ms.muscleMaterial != null) {
                     bundle.setMuscleMaterial(ms.muscleMaterial);
                  } else {
                     bundle.setMuscleMaterial(MUSCLE_MATERIAL);
                  }
                  bundle.setName(bundleName + "_" + i);
                  fem.addMuscleBundle(bundle);
                  bundle.setFibresActive(true);

                  Color color = null;
                  String name = bundleName.replaceAll("_L", "");
                  name = name.replaceAll("_R", "");
                  name = name + "_" + i;
                  for (int j = 0; j < muscleExciterList.size(); j++) {
                     if (name.matches(muscleExciterList.get(j).name)) {
                        color = muscleExciterList.get(j).color;
                     }
                  }
                  if (color == null || !groupExciters) {
                     if (bundleCount >= (NumericProbePanel.colorList.length))
                        ms.bundleColor = NumericProbePanel.colorList[bundleCount
                              % NumericProbePanel.colorList.length];
                     else
                        ms.bundleColor = NumericProbePanel.colorList[bundleCount];
                     bundleCount++;

                     if (groupExciters) {
                        muscleExciterList.add(new MuscleExciterInfo(name,
                              ms.bundleColor));
                        MuscleExciter exciter = new MuscleExciter(name);
                        exciter.addTarget(bundle, 1.0);
                        myJawModel.addMuscleExciter(exciter);
                     }
                  } else {
                     MuscleExciter exciter = myJawModel.getMuscleExciters().get(
                           name);
                     exciter.addTarget(bundle, 1.0);
                     ms.bundleColor = color;
                  }

                  if (useElementsInsteadOfFibres) {
                     RenderProps.setLineWidth(bundle, (int) (lineWidth * 3));
                     bundle.addElementsNearFibres(addMidElementsWithin);
                     bundle.setDirectionRenderLen(elementDirectionRenderLen);
                     bundle.getFibres().clear();
                     // bundle.setFibresActive(false);
                  } else {
                     RenderProps.setLineStyle(bundle, LineStyle.SPINDLE);
                     RenderProps.setLineRadius(bundle, lineRadius);
                  }
                  if (drawBundleExcitation) {
                     RenderProps.setLineColor(bundle, Color.WHITE);
                     bundle.setExcitationColor(ms.bundleColor);
                  } else {
                     RenderProps.setLineColor(bundle, ms.bundleColor);
                  }
                  if (drawBundle == false) {
                     RenderProps.setVisible(bundle, drawBundle);
                  }
               }
               for (int j = 0; j < curSetPts.length; j++) {
                  if (curSetPts[j] != null) {
                     cur = createAndAddMarker(fem, curSetPts[j]);
                     if (j > 0)
                        createAndAddFibre(prev, cur, bundle, ms);
                     prev = cur;
                     if (cur != null && drawAttachedNodes != drawNodes
                           && drawAttachedNodes) {
                        RenderProps.setPointRadius(cur, pointRadius);
                        RenderProps.setPointStyle(cur,
                              Renderer.PointStyle.SPHERE);
                        RenderProps.setPointColor(cur, Color.GREEN);
                     }
                  }
               }
            }
         }

         if (!individualBundleControl) {
            // Set muscle material properties
            if (ms.muscleMaterial != null) {
               bundle.setMuscleMaterial(ms.muscleMaterial);
            } else {
               bundle.setMuscleMaterial(MUSCLE_MATERIAL);
            }
            bundle.setName(bundleName);
            fem.addMuscleBundle(bundle);
            bundle.setFibresActive(true);

            Color color = null;
            String name = bundleName.replaceAll("_L", "");
            name = name.replaceAll("_R", "");
            for (int j = 0; j < muscleExciterList.size(); j++) {
               if (name.matches(muscleExciterList.get(j).name)) {
                  color = muscleExciterList.get(j).color;
               }
            }
            if (color == null || !groupExciters) {
               if (bundleCount >= (NumericProbePanel.colorList.length))
                  ms.bundleColor = NumericProbePanel.colorList[bundleCount
                        % NumericProbePanel.colorList.length];
               else
                  ms.bundleColor = NumericProbePanel.colorList[bundleCount];
               bundleCount++;

               if (groupExciters) {
                  muscleExciterList.add(new MuscleExciterInfo(name,
                        ms.bundleColor));
                  MuscleExciter exciter = new MuscleExciter(name);
                  exciter.addTarget(bundle, 1.0);
                  myJawModel.addMuscleExciter(exciter);
               }
            } else {
               MuscleExciter exciter = myJawModel.getMuscleExciters().get(name);
               exciter.addTarget(bundle, 1.0);
               ms.bundleColor = color;
            }

            if (useElementsInsteadOfFibres) {
               RenderProps.setLineWidth(bundle, (int) (lineWidth * 3));
               bundle.addElementsNearFibres(addMidElementsWithin);
               bundle.setDirectionRenderLen(elementDirectionRenderLen);
               bundle.getFibres().clear();
               // bundle.setFibresActive(false);
            } else {
               RenderProps.setLineStyle(bundle, LineStyle.SPINDLE);
               RenderProps.setLineRadius(bundle, lineRadius);
            }
            if (drawBundleExcitation) {
               RenderProps.setLineColor(bundle, Color.WHITE);
               bundle.setExcitationColor(ms.bundleColor);
            } else {
               RenderProps.setLineColor(bundle, ms.bundleColor);
            }
            if (drawBundle == false) {
               RenderProps.setVisible(bundle, drawBundle);
            }
         }
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + landmarkFileName
                  + " does not exist!");
      }
   }

   private class MuscleExciterInfo
   {
      public String name;
      public Color color;
      public MuscleExciterInfo(String argName, Color argColor) {
    name = argName;
    color = argColor;
      }
   }

   private Muscle createAndAddFibre ( 
         Point pointA, Point pointB, 
         MuscleBundle bundle, MuscleSpringInfo ms) {
         Muscle spring = new Muscle();
         spring.setFirstPoint (pointA);
         spring.setSecondPoint (pointB);
         
         AxialMuscleMaterial mat = null;
         String mtype = null;
         
         if (ms.muscleType == null) {
            mtype = MUSCLE_FIBRE_TYPE;
         } else {
            mtype = ms.muscleType;
         }

         if (mtype.matches("Linear")) {
            mat = new LinearAxialMuscle();
         } else if (mtype.matches("Constant")) {
            mat = new ConstantAxialMuscle();
         } else if (mtype.matches("Peck")) {
            mat = new PeckAxialMuscle();
         } else if (mtype.matches("Pai")) {
            mat = new PaiAxialMuscle();
         } else
            mat = new ConstantAxialMuscle();
         
             // Set properties
             if(!Double.isNaN(ms.damping)) {
                     mat.setDamping(ms.damping);
             } else {
                     mat.setDamping(MUSCLE_DAMPING);
             }
             if(!Double.isNaN(ms.maxForce)) {
                     mat.setMaxForce(ms.maxForce*MUSCLE_MAX_FORCE_SCALING);
             } else {
                     mat.setMaxForce(MUSCLE_MAX_FORCE*MUSCLE_MAX_FORCE_SCALING);
             }
             if(!Double.isNaN(ms.optLength)) {
                     mat.setOptLength(ms.optLength);
             } else {
                     mat.setOptLength(spring.getLength());
             }
             if(!Double.isNaN(ms.maxLength)) {
                     mat.setMaxLength(ms.maxLength);
             } else {
                     mat.setMaxLength(spring.getLength()*1.5);
             }
             if(!Double.isNaN(ms.tendonRatio)) {
                     mat.setTendonRatio(ms.tendonRatio);
             } else {
                     mat.setTendonRatio(MUSCLE_TENDON_RATIO);
             }
             if(!Double.isNaN(ms.passiveFraction)) {
                     mat.setPassiveFraction(ms.passiveFraction);
             } else {
                     mat.setPassiveFraction(MUSCLE_PASSIVE_FRACTION);
             }
             if(!Double.isNaN(ms.muscleForceScaling)) {
                     mat.setForceScaling(ms.muscleForceScaling);
            } else {
                     mat.setForceScaling(MUSCLE_FORCE_SCALING);
            }
             spring.setMaterial(mat);
         
         bundle.addFibre (spring);
         return spring;
      }

   private static FemMarker createAndAddMarker (FemMuscleModel fem, Point3d pnt) {
      FemMarker marker = new FemMarker();

      // add the marker to the model
      FemElement3dBase elem = fem.findContainingElement (pnt);
      if (elem == null) {
         Point3d newLoc = new Point3d();
         elem = fem.findNearestSurfaceElement (newLoc, pnt);
         pnt.set (newLoc);
      }
      marker.setPosition (pnt);
      marker.setFromElement (elem);
      fem.addMarker (marker, elem);
      return marker;
   }

   //########################################################################
   //#### Subfunction: group fem as exciter
   //########################################################################  
   public void groupFemExciters() {
      String exciterName, name, femName;

      for(int i=0;i<muscleExciterList.size();i++) {
         exciterName = muscleExciterList.get(i).name;
         MuscleExciter exciter = new MuscleExciter(exciterName);
         MuscleBundle b;

         for(int j=0;j<muscleList.size();j++) {
            femName = muscleList.get(j).name;
            name = femName.replaceAll("_L", "");
            name = name.replaceAll("_R", "");
            if(name.matches(exciterName)) {
               FemMuscleModel fem =
                     (FemMuscleModel)myJawModel.models().get(femName);
               for(int k=0;k<fem.getMuscleBundles().size();k++) {
                  b = fem.getMuscleBundles().get(k);
                  exciter.addTarget((ExcitationComponent)b, 1.0);
               }
            }
         }
         if(exciter.numTargets()>0) {
            myJawModel.addMuscleExciter(exciter);
         }
      }
   }

   //########################################################################
   //#### Subfunctions: set collision between rigidbodies and fem
   //########################################################################
   public void setCollision()
   {
      for (CollisionInfo collisionInfo: collisionInfoList) {
    // Find first model
         if (collisionInfo.type[0].matches("fem")
            && collisionInfo.type[1].matches("fem")) {
            FemModel3d model1 =
               (FemModel3d)myJawModel.models().get(collisionInfo.name[0]);
            FemModel3d model2 =
               (FemModel3d)myJawModel.models().get(collisionInfo.name[1]);
       if(model1!=null && model2!=null) {
          myJawModel.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
          if(true)
                  System.out.println("Collision set between ("
                     + collisionInfo.name[0] + ", " + collisionInfo.name[1]
                     + ")");
       }
         } else if (collisionInfo.type[0].matches("fem")
            && collisionInfo.type[1].matches("rigidBody")) {
            FemModel3d model1 =
               (FemModel3d)myJawModel.models().get(collisionInfo.name[0]);
            RigidBody model2 =
                  myJawModel.rigidBodies().get(collisionInfo.name[1]);
       if(model1!=null && model2!=null) {
          myJawModel.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
          if(true)
                  System.out.println("Collision set between ("
                     + collisionInfo.name[0] + ", " + collisionInfo.name[1]
                     + ")");
       }
         } else if (collisionInfo.type[0].matches("rigidBody")
            && collisionInfo.type[1].matches("rigidBody")) {
            RigidBody model1 =
                  myJawModel.rigidBodies().get(collisionInfo.name[0]);
            RigidBody model2 =
                  myJawModel.rigidBodies().get(collisionInfo.name[1]);
       if(model1!=null && model2!=null) {
          myJawModel.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
          if(true)
                  System.out.println("Collision set between ("
                     + collisionInfo.name[0] + ", " + collisionInfo.name[1]
                     + ")");
       }
         } else if (collisionInfo.type[0].matches("rigidBody")
            && collisionInfo.type[1].matches("fem")) {
            RigidBody model1 =
               myJawModel.rigidBodies().get(collisionInfo.name[0]);
            FemModel3d model2 =
               (FemModel3d)myJawModel.models().get(collisionInfo.name[1]);
       if(model1!=null && model2!=null) {
          myJawModel.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
          if(true)
                  System.out.println("Collision set between ("
                     + collisionInfo.name[0] + ", " + collisionInfo.name[1]
                     + ")");
       }
    } else {
            System.out.println("Warning: Incorrect file format ("
               + collisionListFilename + "), collision not set.");
    }
      }
   }
   //########################################################################

   private class CollisionInfo
   {
      public String[] type = new String[2];
      public String[] name = new String[2];

      public void scan(ReaderTokenizer rtok) throws IOException
      {
    type[0] = rtok.sval;
    rtok.nextToken ();
    name[0] = rtok.sval;
    rtok.nextToken ();
    type[1] = rtok.sval;
    rtok.nextToken ();
    name[1] = rtok.sval;
      }
   }

   //########################################################################
   //#### Subfunctions: read lists from files
   //########################################################################
   private ArrayList<CollisionInfo> readCollisionInfoList (String filename)
   {
      ArrayList<CollisionInfo> collisionInfoList =
         new ArrayList<CollisionInfo>();
      try {
    ReaderTokenizer rtok = new ReaderTokenizer(
          new FileReader(otherPath + filename));
    rtok.wordChars (".");
    CollisionInfo bi;
    while (rtok.nextToken () != ReaderTokenizer.TT_EOF)
    {
       bi = new CollisionInfo ();
       bi.scan (rtok);
       collisionInfoList.add (bi);
    }
    rtok.close();
      } catch (IOException e) {
    if(debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
      return collisionInfoList;
   }

//   @Override
//   public void addTongueToJaw() {   
//      // do not add tongue
//   }
//
//   @Override
//   public void attachTongueToJaw() {
//      // do not attach tongue
//   }

}
