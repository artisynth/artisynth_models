package artisynth.models.template;

/**
 * 
 */

import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;

import javax.swing.JFrame;
import javax.swing.JSeparator;

import maspack.fileutil.FileManager;
import maspack.fileutil.uri.URIx;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.ReaderTokenizer;
import maspack.util.Logger.LogLevel;
import maspack.widgets.BooleanSelector;
import maspack.widgets.DoubleFieldSlider;
import maspack.widgets.PropertyWidget;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.TetGenReader;
import artisynth.core.femmodels.UCDReader;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.gui.NumericProbePanel;
import artisynth.core.materials.AxialMuscleMaterial;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.materials.InactiveMuscle;
import artisynth.core.materials.IncompNeoHookeanMaterial;
import artisynth.core.materials.LinearAxialMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.MuscleMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.materials.PaiAxialMuscle;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.materials.StVenantKirchoffMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Marker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SolidJoint;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.Model;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.AmiraLandmarkReader;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

/**
 * @author tsoul
 * 
 */
public class ModelTemplate extends RootModel {

   // SANCHEZ, added so we can download files from a zip file or remote source
   FileManager fileManager = null;

   // #####################################################################
   // CONTROLS
   protected boolean debug = false;

   protected boolean disableNodes = false;
   protected boolean IncompressOption = false;
   protected boolean SetCollision = false;
   protected boolean SetAutoAttach = false;
   protected boolean groupExciters = false;
   protected boolean muscleControlPanel = false;
   protected boolean SingleMusclePanels = false;
   protected boolean useElementsInsteadOfFibres = false;
   protected boolean fiberDefinedAcrossSets = false;
   protected boolean centerRigidBodies = false;

   protected String basePath = ArtisynthPath.getSrcRelativePath(ModelTemplate.class, "/");
   protected String rigidBodyPath = basePath + "geometry/rigidBodies/";
   protected String femPath = basePath + "geometry/fem/";
   protected String otherPath = basePath + "geometry/other/";
   protected String probesPath = null;
   protected String rigidBodyDest = null;
   protected String femDest = null;
   protected String otherDest = null;
   protected String probesDest = null;

   protected String remoteDataSource = "file://" + basePath; // default remote source
   protected boolean downloadFiles = true;   // if true, download/extract file, 
   // otherwise just get stream

   protected String bodyListAllFilename = "";
   protected String bodyPropertyListFilename = "";
   protected String bodyTransformListAllFilename = "";
   protected String femListAllFilename = "";
   protected String femPropertyListFilename = "";
   protected String femTransformListAllFilename = "";
   protected String femAttachParticleFilename = "";
   protected String frameMarkerListFilename = "";
   protected String femMarkerListFilename = "";
   protected String femBundleSpringListFilename = "";
   protected String muscleSpringListFilename = "";
   protected String muscleSpringPropertyListFilename = "";
   protected String springListFilename = "";
   protected String springPropertyListFilename = "";
   protected String workingDirname = "src/artisynth/models/template/data";
   protected String probesFilename = "";
   protected String collisionListFilename = "";
   protected String autoAttachListFilename = "";
   protected String aboutFilename =
      "src/artisynth/models/template/about_ModelTemplate.txt";

   // Parameter settings
   public double MAX_STEP_SIZE_SEC = 10e-3; // 10 msec
   public double GRAVITY = 0;
   public double COLLISION_FRICTION_COEFF = 0.0;
   public double BODY_DENSITY = 1.040;
   public double BODY_FRAME_DAMPING = 0.001;
   public double BODY_ROTARY_DAMPING = 0.001;
   public double MUSCLE_DENSITY = 1.040;
   public double MUSCLE_PARTICLE_DAMPING = 0.5;
   public double MUSCLE_STIFFNESS_DAMPING = 0.5;
   public double MUSCLE_YOUNGS_MODULUS = 50000;
   public double MUSCLE_MAXLAMBDA = 2.1;
   public double MUSCLE_MAXSTRESS = 3e5;
   public double MUSCLE_MAX_FORCE = 1;
   public double MUSCLE_MAX_FORCE_SCALING = 1;
   public double MUSCLE_FORCE_SCALING = 1;
   public double MUSCLE_PASSIVE_FRACTION = 0;
   public double MUSCLE_DAMPING = 0;
   public double MUSCLE_TENDON_RATIO = 0;
   public double SPRING_MUSCLE_MAX_FORCE = 1;
   public double SPRING_MUSCLE_FORCE_SCALING = 1;
   public double SPRING_MUSCLE_PASSIVE_FRACTION = 0;
   public double SPRING_MUSCLE_TENDON_RATIO = 0;
   public double SPRING_MUSCLE_DAMPING = 0;
   public double SPRING_DAMPING = 0;
   public double SPRING_STIFFNESS = 0;
   public FemMaterial FEM_MATERIAL = new LinearMaterial();
   public MuscleMaterial MUSCLE_MATERIAL = new InactiveMuscle();
   public String MUSCLE_FIBRE_TYPE = "Constant";

   public double OverallScaling = 1;
   public RigidTransform3d OverallTrans = new RigidTransform3d(
      0, 0, 0, 0, 0, 0, 0);

   // Rendering settings
   protected boolean drawMuscleVector = false;
   protected boolean drawBundle = true;
   protected boolean drawBundleExcitation = false;
   protected boolean muscleShaded = true;
   public boolean drawNodes = false;
   public boolean drawAttachedNodes = true;
   public double pointRadius = 0.2;
   public double lineRadius = 0.1;
   public int lineWidth = 1;
   public double addMidElementsWithin = 5;
   public double elementDirectionRenderLen = 0.5;
   public double elementWedgeSize = 0;
   // Timeline options
   public boolean includeWayPoints = false;
   public double wayPointStep = 0.01;
   public double stopPoint = 1d;
   // #####################################################################

   protected MechModel myMechMod;
   protected ArrayList<BodyInfo> bodyInfoList = new ArrayList<BodyInfo>();
   protected ArrayList<MuscleInfo> muscleList = new ArrayList<MuscleInfo>();
   protected ArrayList<MuscleExciterInfo> muscleExciterList =
      new ArrayList<MuscleExciterInfo>();
   protected ArrayList<TransformInfo> bodyTransformList =
      new ArrayList<TransformInfo>();
   protected ArrayList<TransformInfo> muscleTransformList =
      new ArrayList<TransformInfo>();
   protected ArrayList<CollisionInfo> collisionInfoList =
      new ArrayList<CollisionInfo>();
   protected ArrayList<AutoAttachInfo> autoAttachInfoList =
      new ArrayList<AutoAttachInfo>();
   protected ArrayList<String> muscleSpringList = new ArrayList<String>();
   protected ArrayList<String> muscleSpringGroupList = new ArrayList<String>();
   protected ArrayList<MuscleSpringInfo> muscleSpringInfoList =
      new ArrayList<MuscleSpringInfo>();
   protected ArrayList<SpringInfo> springInfoList = new ArrayList<SpringInfo>();
   protected HashMap<RigidBody,Vector3d> bodyTranslationMap =
      new HashMap<RigidBody,Vector3d>();

   protected ControlPanel myControlPanel;
   protected RigidBody block;
   private static final double inf = Double.POSITIVE_INFINITY;
   protected int bundleCount;

   // GLViewer v = Main.getMain().getViewer();

   public ModelTemplate () {
   }

   public ModelTemplate (String name) throws IOException {
      super(name);
   }

   public void createModel() throws IOException
   {
      // ###################################################################
      // ###### Initialization #############################################

      fileManager = new FileManager(basePath, remoteDataSource);
      // Disable most logging since this class is designed to look for
      // non-existent files, such as .femAttach, on the off-chance that they
      // exist
      fileManager.setVerbosityLevel(LogLevel.FATAL);  

      myMechMod = new MechModel("mech");
      myMechMod.setIntegrator(Integrator.ConstrainedBackwardEuler);
      myMechMod.setMaxStepSize(MAX_STEP_SIZE_SEC);
      // v.myUnClippedRenderList.clear();
      bundleCount = 0;
      // ###################################################################

      // ###################################################################
      // ###### Add rigid bodies ###########################################
      bodyInfoList = readBodyInfoList(bodyListAllFilename);
      readBodyProperties(bodyPropertyListFilename);
      bodyTransformList =
         readTransList(rigidBodyPath, bodyTransformListAllFilename, rigidBodyDest);
      assembleRigidBodies();
      // ###################################################################

      // ###################################################################
      // ###### Add muscles ################################################
      // Read in mesh
      muscleList = readStringList(femListAllFilename);
      readMuscleProperty(femPropertyListFilename);
      muscleTransformList =
         readTransList(femPath, femTransformListAllFilename, femDest);
      assembleMuscles();
      attachParticles(femAttachParticleFilename);
      // ###################################################################

      // ###################################################################
      // ###### Set collision ##############################################
      if (SetAutoAttach) {
         autoAttachInfoList = readAutoAttachInfoList(autoAttachListFilename);
         autoAttach();
      }
      // ###################################################################

      // ===================================================================
      // EDIT: Add Fem markers, (Sanchez, Nov 29, 2011)
      addAllFemMarkers(femMarkerListFilename);
      // ===================================================================

      // ###################################################################
      // ###### Add frame markers ##########################################
      addAllFrameMarker(frameMarkerListFilename);
      RenderProps.setVisible(myMechMod.frameMarkers(), drawAttachedNodes);
      // ###################################################################

      // ###################################################################
      // ###### Add springs ################################################
      readMuscleSpringProperty(muscleSpringPropertyListFilename);
      addMuscleSpring(muscleSpringListFilename);
      addFemBundleSpring(femBundleSpringListFilename);

      readSpringProperty(springPropertyListFilename);
      addPassiveSpring(springListFilename);
      // ###################################################################

      if (groupExciters) {
         groupFemExciters();
      }

      // ###################################################################
      // ###### Set collision ##############################################
      if (SetCollision) {
         collisionInfoList = readCollisionInfoList(collisionListFilename);
         setCollision();
      }
      // ###################################################################

      // ###################################################################
      // ###### set the location of ref_block to the centre ################
      // setRefBlock();
      // ###################################################################

      // ###################################################################
      addModel(myMechMod);
      myMechMod.setGravity(0, 0, -GRAVITY);
      // Apply overall scaling
      myMechMod.scaleDistance(OverallScaling);
      // Apply overall transform
      myMechMod.transformGeometry(OverallTrans);

      if (fileManager != null) {
         fileManager.closeStreams();
      }

   }

   // ########################################################################
   // #### Utility functions
   // ########################################################################
   //   private Color getComplementaryColor(Color origColor)
   //   {
   //      float[] hsbColor =
   //         Color
   //            .RGBtoHSB(
   //               origColor.getRed(), origColor.getGreen(), origColor.getBlue(),
   //               null);
   //      if (hsbColor[0] < 0.5) {
   //         hsbColor[0] = (float)(hsbColor[0] + 0.5);
   //      } else {
   //         hsbColor[0] = (float)(hsbColor[0] - 0.5);
   //      }
   //      return (new Color(Color.HSBtoRGB(hsbColor[0], hsbColor[1], hsbColor[2])));
   //   }

   // ########################################################################

   // ########################################################################
   // #### Subfunctions: read lists from files
   // ########################################################################
   private ArrayList<CollisionInfo> readCollisionInfoList(String filename)
   {
      ArrayList<CollisionInfo> collisionInfoList =
         new ArrayList<CollisionInfo>();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(getInputStream(otherPath, filename, otherDest)));
         rtok.wordChars(".");
         CollisionInfo bi;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            bi = new CollisionInfo();
            bi.scan(rtok);
            collisionInfoList.add(bi);
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
      return collisionInfoList;
   }

   private ArrayList<AutoAttachInfo> readAutoAttachInfoList(String filename)
   {
      ArrayList<AutoAttachInfo> AutoAttachInfoList =
         new ArrayList<AutoAttachInfo>();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         rtok.wordChars(".");
         AutoAttachInfo bi;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            bi = new AutoAttachInfo();
            bi.scan(rtok);
            AutoAttachInfoList.add(bi);
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
      return AutoAttachInfoList;
   }

   private ArrayList<BodyInfo> readBodyInfoList(String filename)
   {
      ArrayList<BodyInfo> bodyInfoList = new ArrayList<BodyInfo>();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(rigidBodyPath, filename, rigidBodyDest)));
         rtok.wordChars(".");
         BodyInfo bi;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            bi = new BodyInfo();
            bi.scan(rtok);
            bodyInfoList.add(bi);
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + rigidBodyPath + filename
               + " does not exist!");
      }
      return bodyInfoList;
   }

   private void readBodyProperties(String filename)
   {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         rtok.wordChars(".");
         rtok.wordChars("-");
         BodyInfo bi;
         String name;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            name = rtok.sval;
            if (name != null) {
               for (BodyInfo bodyInfo : bodyInfoList) {
                  if (bodyInfo.name.matches(name)) {
                     bi = bodyInfo;
                     while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
                        if (rtok.sval.matches("-density")) {
                           bi.scanDensity(rtok);
                        } else if (rtok.sval.matches("-mass")) {
                           bi.scanMass(rtok);
                        } else {
                           rtok.pushBack();
                           break;
                        }
                     }
                     break;
                  }
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      } catch (Exception e) {
      }
   }

   private ArrayList<TransformInfo> readTransList(String source, String relPath, String destPath)
   {
      ArrayList<TransformInfo> transformList = new ArrayList<TransformInfo>();
      if (relPath == null || relPath.equals("")) {
         return transformList;
      }
      try {
         InputStream in = getInputStream(source, relPath, destPath);
         ReaderTokenizer rtok = new ReaderTokenizer(new InputStreamReader(in));
         TransformInfo trans;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            trans = new TransformInfo();
            trans.scan(rtok);
            transformList.add(trans);
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out
            .println("Warning: cannot read transform file '" + source + relPath + "'");
      }
      return transformList;
   }

   private ArrayList<MuscleInfo> readStringList(String filename)
   {
      ArrayList<MuscleInfo> muscleList = new ArrayList<MuscleInfo>();
      MuscleInfo mu;
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(femPath, filename, femDest)));

         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype != ReaderTokenizer.TT_WORD)
            {
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

   private void readMuscleProperty(String filename)
   {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         rtok.wordChars(".");
         rtok.wordChars("-");
         MuscleInfo m;
         String name;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            name = rtok.sval;
            if (name != null) {
               for (MuscleInfo muscleInfo : muscleList) {
                  if (muscleInfo.name.matches(name)) {
                     m = muscleInfo;
                     while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
                        if (rtok.sval.matches("-density")) {
                           m.scanDensity(rtok);
                        } else if (rtok.sval.matches("-mass")) {
                           m.scanMass(rtok);
                        } else if (rtok.sval.matches("-particleDamping")) {
                           m.scanParticleDamping(rtok);
                        } else if (rtok.sval.matches("-stiffnessDamping")) {
                           m.scanStiffnessDamping(rtok);
                        } else if (rtok.sval.matches("-youngsModulus")) {
                           m.scanYoungsModulus(rtok);
                        } else if (rtok.sval.matches("-materialType")) {
                           m.scanMaterial(rtok);
                        } else if (rtok.sval.matches("-muscleMaterial")) {
                           m.scanMuscleMaterial(rtok);
                        } else if (rtok.sval.matches("-incompressible")) {
                           m.scanIncompressible(rtok);
                        } else if (rtok.sval.matches("-muscleMaxForce")) {
                           m.scanMuscleMaxForce(rtok);
                        } else if (rtok.sval.matches("-muscleForceScaling")) {
                           m.scanMuscleForceScaling(rtok);
                        } else if (rtok.sval.matches("-muscleDamping")) {
                           m.scanMuscleDamping(rtok);
                        } else if (rtok.sval.matches("-optLength")) {
                           m.scanOptLength(rtok);
                        } else if (rtok.sval.matches("-maxLength")) {
                           m.scanMaxLength(rtok);
                        } else if (rtok.sval.matches("-tendonRatio")) {
                           m.scanTendonRatio(rtok);
                        } else if (rtok.sval.matches("-passiveFraction")) {
                           m.scanPassiveFraction(rtok);
                        } else if (rtok.sval.matches("-muscleType")) {
                           m.scanMuscleType(rtok);
                        } else {
                           rtok.pushBack();
                           break;
                        }
                     }
                     break;
                  }
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   private ArrayList<MuscleFibreInfo> readMuscleFibreList(String filename)
   {
      ArrayList<MuscleFibreInfo> muscleFibreList =
         new ArrayList<MuscleFibreInfo>();
      try {
         MuscleFibreInfo mu;
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(femPath, filename + ".fibre", femDest)));

         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype == ReaderTokenizer.TT_WORD)
            {
               mu = new MuscleFibreInfo();
               mu.name = rtok.sval;
               if (debug)
                  System.out.println("Reading attachments to " + mu.name);
               if (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
                  throw new IOException("Muscle Fibre File incorrect format: "
                     + filename + ".fibre");
               mu.list = new int[(int)rtok.nval][2];
               int i = 0;
               while (rtok.nextToken() == ReaderTokenizer.TT_NUMBER) {
                  mu.list[i][0] = (int)rtok.nval;
                  if (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
                     throw new IOException(
                        "Muscle Fibre File incorrect format: " + filename
                        + ".fibre");
                  mu.list[i][1] = (int)rtok.nval;
                  i++;
               }
               muscleFibreList.add(mu);
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + femPath + filename
               + ".fibre does not exist!");
      }
      return muscleFibreList;
   }

   private ArrayList<MuscleAttachInfo>
   readMuscleAttachmentList(String filename)
   {
      ArrayList<MuscleAttachInfo> muscleAttachList =
         new ArrayList<MuscleAttachInfo>();
      try {
         MuscleAttachInfo mu;
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(femPath, filename + ".attach", femDest)));

         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype == ReaderTokenizer.TT_WORD)
            {
               mu = new MuscleAttachInfo();
               mu.target = rtok.sval;
               if (debug)
                  System.out.println("Reading attachments to " + mu.target);
               if (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
                  throw new IOException(
                     "Muscle Attachment File incorrect format: " + filename
                     + ".attach");
               mu.list = new int[(int)rtok.nval];
               int i = 0;
               while (rtok.nextToken() == ReaderTokenizer.TT_NUMBER) {
                  int n = (int)rtok.nval;
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

   private ArrayList<MuscleAttachInfo> readMuscleToMuscleAttachmentList(
      String filename)
      {
      ArrayList<MuscleAttachInfo> muscleAttachList =
         new ArrayList<MuscleAttachInfo>();
      try {
         MuscleAttachInfo mu;
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(femPath, filename + ".femAttach", femDest)));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype == ReaderTokenizer.TT_WORD)
            {
               mu = new MuscleAttachInfo();
               mu.target = rtok.sval;
               if (debug)
                  System.out.println("Reading attachments to " + mu.target);
               if (rtok.nextToken() != ReaderTokenizer.TT_NUMBER)
                  throw new IOException(
                     "Muscle Attachment File incorrect format: " + filename
                     + ".attach");
               mu.list = new int[(int)rtok.nval];
               int i = 0;
               while (rtok.nextToken() == ReaderTokenizer.TT_NUMBER) {
                  int n = (int)rtok.nval;
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
               + ".femAttach does not exist!");
      }
      return muscleAttachList;
      }

   private void readMuscleSpringProperty(String filename)
   {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         rtok.wordChars(".");
         rtok.wordChars("-");
         String name;
         MuscleSpringInfo ms;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            name = rtok.sval;
            ms = new MuscleSpringInfo(name);
            muscleSpringInfoList.add(ms);
            while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
               if (rtok.sval.matches("-damping")) {
                  ms.scanDamping(rtok);
               } else if (rtok.sval.matches("-maxForce")) {
                  ms.scanMaxForce(rtok);
               } else if (rtok.sval.matches("-optLength")) {
                  ms.scanOptLength(rtok);
               } else if (rtok.sval.matches("-maxLength")) {
                  ms.scanMaxLength(rtok);
               } else if (rtok.sval.matches("-tendonRatio")) {
                  ms.scanTendonRatio(rtok);
               } else if (rtok.sval.matches("-passiveFraction")) {
                  ms.scanPassiveFraction(rtok);
               } else if (rtok.sval.matches("-muscleType")) {
                  ms.scanMuscleType(rtok);
               } else if (rtok.sval.matches("-muscleMaterial")) {
                  ms.scanMuscleMaterial(rtok);
               } else if (rtok.sval.matches("-muscleForceScaling")) {
                  ms.scanMuscleForceScaling(rtok);
               } else {
                  rtok.pushBack();
                  break;
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   private void readSpringProperty(String filename)
   {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         rtok.wordChars(".");
         rtok.wordChars("-");
         String name;
         SpringInfo ms;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            name = rtok.sval;
            ms = new SpringInfo(name);
            springInfoList.add(ms);
            while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
               if (rtok.sval.matches("-damping")) {
                  ms.scanDamping(rtok);
               } else if (rtok.sval.matches("-stiffness")) {
                  ms.scanStiffness(rtok);
               } else if (rtok.sval.matches("GLRendererstLength")) {
                  ms.scanRestLength(rtok);
               } else {
                  rtok.pushBack();
                  break;
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   private class BodyInfo
   {
      public String name;
      public String meshName;
      public boolean dynamic = true;
      public double scale;
      public Color color;
      public double mass = Double.NaN, density = Double.NaN;
      //      public double linDamping;
      //      public double rotDamping;

      public void scan(ReaderTokenizer rtok) throws IOException
      {
         name = rtok.sval;
         rtok.nextToken();
         meshName = rtok.sval;
         rtok.nextToken();
         if (rtok.sval.matches("(?i:fix)"))
            dynamic = false;
         else
            dynamic = true;
         rtok.nextToken();
         scale = rtok.nval;
         rtok.nextToken();
         int r = (int)rtok.nval;
         rtok.nextToken();
         int g = (int)rtok.nval;
         rtok.nextToken();
         int b = (int)rtok.nval;
         color = new Color(r, g, b);
      }

      public void scanMass(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         mass = rtok.nval;
      }

      public void scanDensity(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         density = rtok.nval;
      }
   }

   private class MuscleExciterInfo
   {
      public String name;
      public Color color;

      public MuscleExciterInfo (String argName, Color argColor) {
         name = argName;
         color = argColor;
      }
   }

   private class MuscleInfo
   {
      public String name;
      public String meshName;
      public String format;
      public double scale;
      public Color color;
      public Color bundleColor;
      public String muscleType;
      // public boolean hasBaseBundle = false;

      public double mass = Double.NaN, density = Double.NaN,
         particleDamping = Double.NaN,
         stiffnessDamping = Double.NaN, youngsModulus = Double.NaN,
         muscleMaxForce = Double.NaN, muscleForceScaling = Double.NaN,
         muscleDamping = Double.NaN,
         optLength = Double.NaN, maxLength = Double.NaN,
         tendonRatio = Double.NaN, passiveFraction = Double.NaN;
      public FemMaterial material;

      public MuscleMaterial muscleMaterial;
      public boolean incompressible = IncompressOption;

      public void scan(ReaderTokenizer rtok) throws IOException
      {
         name = rtok.sval;
         rtok.nextToken();
         meshName = rtok.sval;
         rtok.nextToken();
         format = rtok.sval;
         rtok.nextToken();
         scale = rtok.nval;
         rtok.nextToken();
         int r = (int)rtok.nval;
         rtok.nextToken();
         int g = (int)rtok.nval;
         rtok.nextToken();
         int b = (int)rtok.nval;
         color = new Color(r, g, b);
      }

      public void scanMass(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         mass = rtok.nval;
      }

      public void scanDensity(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         density = rtok.nval;
      }

      public void scanParticleDamping(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         particleDamping = rtok.nval;
      }

      public void scanStiffnessDamping(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         stiffnessDamping = rtok.nval;
      }

      public void scanYoungsModulus(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         youngsModulus = rtok.nval;
      }

      public void scanIncompressible(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         if (rtok.sval.matches("(?i:true)")) {
            incompressible = true;
         } else {
            incompressible = false;
         }
      }

      public void scanMuscleDamping(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         muscleDamping = rtok.nval;
      }

      public void scanOptLength(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         optLength = rtok.nval;
      }

      public void scanMaxLength(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         maxLength = rtok.nval;
      }

      public void scanTendonRatio(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         tendonRatio = rtok.nval;
      }

      public void scanPassiveFraction(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         passiveFraction = rtok.nval;
      }

      public void scanMuscleType(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         muscleType = rtok.sval;
      }

      public void scanMaterial(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         String materialType = rtok.sval;
         if (materialType.matches("(?i:Linear)")) {
            rtok.nextToken();
            double YoungsModulus = rtok.nval;
            rtok.nextToken();
            double PoissonsRatio = rtok.nval;
            material = new LinearMaterial(YoungsModulus, PoissonsRatio);
         } else if (materialType.matches("(?i:StVenanKirchoff)")) {
            rtok.nextToken();
            double YoungsModulus = rtok.nval;
            rtok.nextToken();
            double PoissonsRatio = rtok.nval;
            material = new StVenantKirchoffMaterial();
            ((StVenantKirchoffMaterial)material)
            .setYoungsModulus(YoungsModulus);
            ((StVenantKirchoffMaterial)material)
            .setPoissonsRatio(PoissonsRatio);
         } else if (materialType.matches("(?i:NeoHookean)")) {
            rtok.nextToken();
            double YoungsModulus = rtok.nval;
            rtok.nextToken();
            double PoissonsRatio = rtok.nval;
            material = new NeoHookeanMaterial();
            ((NeoHookeanMaterial)material).setYoungsModulus(YoungsModulus);
            ((NeoHookeanMaterial)material).setPoissonsRatio(PoissonsRatio);
         } else if (materialType.matches("(?i:MooneyRivlin)")) {
            rtok.nextToken();
            double bulkModulus, c10, c01, c11, c20, c02;
            bulkModulus = rtok.nval;
            rtok.nextToken();
            c10 = rtok.nval;
            rtok.nextToken();
            c01 = rtok.nval;
            rtok.nextToken();
            c11 = rtok.nval;
            rtok.nextToken();
            c20 = rtok.nval;
            rtok.nextToken();
            c02 = rtok.nval;
            material =
               new MooneyRivlinMaterial(c10, c01, c11, c20, c02, bulkModulus);
         } else if (materialType.matches("(?i:IncompNeoHookean)")) {
            rtok.nextToken();
            double bulkModulus = rtok.nval;
            rtok.nextToken();
            double shearModulus = rtok.nval;
            material = new IncompNeoHookeanMaterial(shearModulus, bulkModulus);
         }
      }

      public void scanMuscleMaxForce(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         muscleMaxForce = rtok.nval;
      }

      public void scanMuscleForceScaling(ReaderTokenizer rtok)
         throws IOException
         {
         rtok.nextToken();
         muscleForceScaling = rtok.nval;
         }

      public void scanMuscleMaterial(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         String muscleMaterialType = rtok.sval;
         if (muscleMaterialType.matches("(?i:GenericMuscle)")) {
            rtok.nextToken();
            double maxLambda, maxStress, expStressCoeff, uncrimpingFactor;
            maxLambda = rtok.nval;
            rtok.nextToken();
            maxStress = rtok.nval;
            rtok.nextToken();
            expStressCoeff = rtok.nval;
            rtok.nextToken();
            uncrimpingFactor = rtok.nval;
            rtok.nextToken();
            // fibreModulus = rtok.nval;
            muscleMaterial = new GenericMuscle();
            ((GenericMuscle)muscleMaterial).setMaxLambda(maxLambda);
            ((GenericMuscle)muscleMaterial).setMaxStress(maxStress);
            ((GenericMuscle)muscleMaterial).setExpStressCoeff(expStressCoeff);
            ((GenericMuscle)muscleMaterial)
            .setUncrimpingFactor(uncrimpingFactor);
            // ((GenericMuscle)muscleMaterial).setFibreModulus(fibreModulus);
         } else if (muscleMaterialType.matches("(?i:BlemkerMuscle)")) {
            rtok.nextToken();
            double maxLambda, optLambda, maxStress, expStressCoeff, uncrimpingFactor;
            maxLambda = rtok.nval;
            rtok.nextToken();
            optLambda = rtok.nval;
            rtok.nextToken();
            maxStress = rtok.nval;
            rtok.nextToken();
            expStressCoeff = rtok.nval;
            rtok.nextToken();
            uncrimpingFactor = rtok.nval;
            muscleMaterial = new BlemkerMuscle();
            ((BlemkerMuscle)muscleMaterial).setMaxLambda(maxLambda);
            ((BlemkerMuscle)muscleMaterial).setOptLambda(optLambda);
            ((BlemkerMuscle)muscleMaterial).setMaxStress(maxStress);
            ((BlemkerMuscle)muscleMaterial).setExpStressCoeff(expStressCoeff);
            ((BlemkerMuscle)muscleMaterial)
            .setUncrimpingFactor(uncrimpingFactor);
         } else if (muscleMaterialType.matches("(?i:InactiveMuscle)")) {
            muscleMaterial = new InactiveMuscle();
         }
      }
   }

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

      public MuscleSpringInfo () {
      }

      public MuscleSpringInfo (String argName) {
         name = argName;
      }

      public void scanDamping(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         damping = rtok.nval;
      }

      public void scanMaxForce(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         maxForce = rtok.nval;
      }

      public void scanOptLength(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         optLength = rtok.nval;
      }

      public void scanMaxLength(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         maxLength = rtok.nval;
      }

      public void scanTendonRatio(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         tendonRatio = rtok.nval;
      }

      public void scanPassiveFraction(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         passiveFraction = rtok.nval;
      }

      public void scanMuscleType(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         muscleType = rtok.sval;
      }

      public void scanMuscleForceScaling(ReaderTokenizer rtok)
         throws IOException
         {
         rtok.nextToken();
         muscleForceScaling = rtok.nval;
         }

      public void scanMuscleMaterial(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         String muscleMaterialType = rtok.sval;
         if (muscleMaterialType.matches("(?i:GenericMuscle)")) {
            rtok.nextToken();
            double maxLambda, maxStress, expStressCoeff, uncrimpingFactor;
            maxLambda = rtok.nval;
            rtok.nextToken();
            maxStress = rtok.nval;
            rtok.nextToken();
            expStressCoeff = rtok.nval;
            rtok.nextToken();
            uncrimpingFactor = rtok.nval;
            rtok.nextToken();
            // fibreModulus = rtok.nval;
            muscleMaterial = new GenericMuscle();
            ((GenericMuscle)muscleMaterial).setMaxLambda(maxLambda);
            ((GenericMuscle)muscleMaterial).setMaxStress(maxStress);
            ((GenericMuscle)muscleMaterial).setExpStressCoeff(expStressCoeff);
            ((GenericMuscle)muscleMaterial)
            .setUncrimpingFactor(uncrimpingFactor);
            // ((GenericMuscle)muscleMaterial).setFibreModulus(fibreModulus);
         } else if (muscleMaterialType.matches("(?i:BlemkerMuscle)")) {
            rtok.nextToken();
            double maxLambda, optLambda, maxStress, expStressCoeff, uncrimpingFactor;
            maxLambda = rtok.nval;
            rtok.nextToken();
            optLambda = rtok.nval;
            rtok.nextToken();
            maxStress = rtok.nval;
            rtok.nextToken();
            expStressCoeff = rtok.nval;
            rtok.nextToken();
            uncrimpingFactor = rtok.nval;
            muscleMaterial = new BlemkerMuscle();
            ((BlemkerMuscle)muscleMaterial).setMaxLambda(maxLambda);
            ((BlemkerMuscle)muscleMaterial).setOptLambda(optLambda);
            ((BlemkerMuscle)muscleMaterial).setMaxStress(maxStress);
            ((BlemkerMuscle)muscleMaterial).setExpStressCoeff(expStressCoeff);
            ((BlemkerMuscle)muscleMaterial)
            .setUncrimpingFactor(uncrimpingFactor);
         } else if (muscleMaterialType.matches("(?i:InactiveMuscle)")) {
            muscleMaterial = new InactiveMuscle();
         }
      }
   }

   private class SpringInfo
   {
      public String name;
      public double stiffness = Double.NaN, damping = Double.NaN,
         restLength = Double.NaN;

      public SpringInfo () {
      }

      public SpringInfo (String argName)
      {
         name = argName;
      }

      public void scanStiffness(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         stiffness = rtok.nval;
      }

      public void scanDamping(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         damping = rtok.nval;
      }

      public void scanRestLength(ReaderTokenizer rtok) throws IOException
      {
         rtok.nextToken();
         restLength = rtok.nval;
      }
   }

   private class TransformInfo
   {
      public RigidTransform3d transform;
      public String name;

      public void scan(ReaderTokenizer rtok) throws IOException
      {
         double tx, ty, tz, rx, ry, rz, deg;
         name = rtok.sval;
         rtok.nextToken();
         tx = rtok.nval;
         rtok.nextToken();
         ty = rtok.nval;
         rtok.nextToken();
         tz = rtok.nval;
         rtok.nextToken();
         rx = rtok.nval;
         rtok.nextToken();
         ry = rtok.nval;
         rtok.nextToken();
         rz = rtok.nval;
         rtok.nextToken();
         deg = rtok.nval;
         transform =
            new RigidTransform3d(tx, ty, tz, rx, ry, rz, Math.toRadians(deg));
         if (debug)
            System.out.println(name + " Transform(" + tx + " " + ty + " " + tz
               + " " + rx + " " + ry + " " + rz + " " + deg + ")");
      }
   }

   private class CollisionInfo
   {
      public String[] type = new String[2];
      public String[] name = new String[2];

      public void scan(ReaderTokenizer rtok) throws IOException
      {
         type[0] = rtok.sval;
         rtok.nextToken();
         name[0] = rtok.sval;
         rtok.nextToken();
         type[1] = rtok.sval;
         rtok.nextToken();
         name[1] = rtok.sval;
      }
   }

   private class AutoAttachInfo
   {
      public String[] type = new String[2];
      public String[] name = new String[2];
      public double distance = 0;

      public void scan(ReaderTokenizer rtok) throws IOException
      {
         type[0] = rtok.sval;
         rtok.nextToken();
         name[0] = rtok.sval;
         rtok.nextToken();
         type[1] = rtok.sval;
         rtok.nextToken();
         name[1] = rtok.sval;

         if (type[0].equals("fem") || type[1].equals("fem")) {
            rtok.nextToken();
            distance = rtok.nval;
         } else {
            distance = 0;
         }
      }
   }

   private class MuscleAttachInfo
   {
      public String target;
      public int[] list;
   }

   private class MuscleFibreInfo
   {
      public String name;
      public int[][] list;
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunctions: set collision between rigidbodies and fem
   // ########################################################################
   public void setCollision()
   {
      for (CollisionInfo collisionInfo : collisionInfoList) {
         // Find first model
         if (collisionInfo.type[0].matches("fem")
            && collisionInfo.type[1].matches("fem")) {
            FemModel3d model1 =
               (FemModel3d)myMechMod.models().get(collisionInfo.name[0]);
            FemModel3d model2 =
               (FemModel3d)myMechMod.models().get(collisionInfo.name[1]);
            if (model1 != null && model2 != null) {
               myMechMod.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
               if (true)
                  System.out.println("Collision set between ("
                     + collisionInfo.name[0] + ", " + collisionInfo.name[1]
                        + ")");
            }
         } else if (collisionInfo.type[0].matches("fem")
            && collisionInfo.type[1].matches("rigidBody")) {
            FemModel3d model1 =
               (FemModel3d)myMechMod.models().get(collisionInfo.name[0]);
            RigidBody model2 =
               myMechMod.rigidBodies().get(collisionInfo.name[1]);
            if (model1 != null && model2 != null) {
               myMechMod.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
               if (true)
                  System.out.println("Collision set between ("
                     + collisionInfo.name[0] + ", " + collisionInfo.name[1]
                        + ")");
            }
         } else if (collisionInfo.type[0].matches("rigidBody")
            && collisionInfo.type[1].matches("rigidBody")) {
            RigidBody model1 =
               myMechMod.rigidBodies().get(collisionInfo.name[0]);
            RigidBody model2 =
               myMechMod.rigidBodies().get(collisionInfo.name[1]);
            if (model1 != null && model2 != null) {
               myMechMod.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
               if (true)
                  System.out.println("Collision set between ("
                     + collisionInfo.name[0] + ", " + collisionInfo.name[1]
                        + ")");
            }
         } else if (collisionInfo.type[0].matches("rigidBody")
            && collisionInfo.type[1].matches("fem")) {
            RigidBody model1 =
               myMechMod.rigidBodies().get(collisionInfo.name[0]);
            FemModel3d model2 =
               (FemModel3d)myMechMod.models().get(collisionInfo.name[1]);
            if (model1 != null && model2 != null) {
               myMechMod.setCollisionBehavior(
                  model1, model2, true, COLLISION_FRICTION_COEFF);
               if (true)
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

   // ########################################################################

   // ########################################################################
   // #### Subfunctions: add rigid body to the model from a file
   // ########################################################################
   public void assembleRigidBodies() throws IOException
   {
      for (BodyInfo bodyInfo : bodyInfoList) {
         addBody(bodyInfo);
      }

      // add a reference body
      setRefBlock();
   }

   public void setRefBlock() {
      block = new RigidBody("ref_block");
      block.setMesh(MeshFactory.createBox(
         OverallScaling, OverallScaling, OverallScaling), null);
      block.setDynamic(false);
      RenderProps.setVisible(block, false);
      Point3d com = getCenter();
      block.getMesh().transform(new RigidTransform3d(com.x, com.y, com.z));
      myMechMod.addRigidBody(block);
   }

   public RigidBody addBody(BodyInfo bi) {

      String name = bi.name;
      String meshName = bi.meshName;
      Color color = bi.color;
      double scale = bi.scale;

      RigidBody body = new RigidBody(name);

      setBodyMesh(body, rigidBodyPath, meshName, rigidBodyDest, scale);

      // Set parameters
      if (!Double.isNaN(bi.mass)) {
         body.setMass(bi.mass);
      } else if (!Double.isNaN(bi.density)) {
         body.setDensity(bi.density);
      } else {
         body.setDensity(BODY_DENSITY);
      }

      body.setFrameDamping(BODY_FRAME_DAMPING);
      body.setRotaryDamping(BODY_ROTARY_DAMPING);

      // Set dynamic
      body.setDynamic(bi.dynamic);

      // Apply individual transform
      for (TransformInfo trans : bodyTransformList) {
         if (name.matches(trans.name)) {
            body.transformGeometry(trans.transform);
         }
      }

      // Rendering
      RenderProps.setFaceColor(body, color);
      RenderProps.setShading(body, Renderer.Shading.FLAT);
      // RenderProps.setShading (body, Renderer.Shading.SMOOTH);
      RenderProps.setFaceStyle(body, Renderer.FaceStyle.FRONT_AND_BACK);

      myMechMod.addRigidBody(body);

      // exclude for cut plane
      /*
       * if(!name.matches("hyoid") && !name.matches("thyroid") &&
       * !name.matches("cricoid") && !name.matches("Arytenoid_L")) {
       * v.myUnClippedRenderList.addIfVisible(body); v.removeRenderable(body); }
       */

      return body;
   }

   private void setBodyMesh(RigidBody body, String source, String filename, 
      String fileDest, double scale) {

      try {
         PolygonalMesh mesh = new PolygonalMesh();
         mesh.read(
            new InputStreamReader(
               getInputStream(source, filename, fileDest)));

         mesh.scale(scale);
         body.setMesh(mesh, null);

         if (centerRigidBodies) {
            // adjust center of mass for stability reasons
            Point3d com = new Point3d();
            body.getCenterOfMass(com);
            bodyTranslationMap.put(body, com);
            body.setPose(new RigidTransform3d(com.x, com.y, com.z, 0, 0, 0));
            mesh.transform(
               new RigidTransform3d(-com.x, -com.y, -com.z, 0, 0, 0));
            body.setMesh(mesh, null);
         }

      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunctions: add fem muscles, attach nodes, and define fibres
   // ########################################################################
   public void assembleMuscles() throws IOException
   {
      assignBundleColor();
      for (MuscleInfo mu : muscleList)
      {
         // Add mesh
         addMuscle(mu);
         attachMuscle(mu);
         attachMuscleToMuscle(mu);
         defineMuscleFibre(mu);
      }
   }

   // ########################################################################
   // #### Subfunction: add fem muscle to the model from a file
   // ########################################################################
   private void addMuscle(MuscleInfo mu) throws IOException {

      String name = mu.name;
      String meshName = mu.meshName;
      String format = mu.format;
      Color color = mu.color;

      FemMuscleModel femMuscle = new FemMuscleModel(name);
      // Read mesh from files
      try {

         if (format.matches("ansys")) {
            InputStreamReader nodeReader = new InputStreamReader(getInputStream(femPath, meshName + ".node", femDest));
            InputStreamReader elemReader = new InputStreamReader(getInputStream(femPath, meshName + ".elem", femDest));
            AnsysReader.read(
               femMuscle, nodeReader, elemReader,
               1, null, /* options= */0);
            if (debug)
               System.out.println("Reading in mesh: " + femPath + meshName
                  + ".node|elem");
         } else if (format.matches("tetgen")) {
            InputStreamReader nodeReader = new InputStreamReader(getInputStream(femPath, meshName + ".node", femDest));
            InputStreamReader elemReader = new InputStreamReader(getInputStream(femPath, meshName + ".ele", femDest));
            TetGenReader.read(
               femMuscle, femMuscle.getDensity(), null, nodeReader, elemReader);
            if (debug)
               System.out.println("Reading in mesh: " + femPath + meshName
                  + ".node|ele");
         } else if (format.matches("ucd")) {
            InputStreamReader reader = new InputStreamReader(getInputStream(femPath, meshName + ".inp", femDest));
            UCDReader.read(femMuscle, reader, 10.0, null);
            if (debug)
               System.out.println("Reading in mesh: " + femPath + meshName
                  + ".inp");
         } else {
            throw new IOException("Warning: Mesh file format not supported: "
               + format);
         }
      } catch (Exception e) {
         if (format.matches("ansys")) {
            throw new RuntimeException(" Can't create Ansys FEM from "
               + femPath + meshName + ".node|.elem", e);
         } else if (format.matches("ansys")) {
            throw new RuntimeException(" Can't create TetGen FEM from "
               + femPath + meshName + ".node|.ele", e);
         } else if (format.matches("ucd")) {
            throw new RuntimeException(" Can't create UCD FEM from "
               + femPath + meshName + ".inp", e);
         }
      }

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
      }
      else {
         femMuscle.setIncompressible(IncompMethod.OFF);
      }
      // femMuscle.setIncompressible(mu.incompressible);

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

      // Apply transform
      for (TransformInfo trans : muscleTransformList) {
         if (name.matches(trans.name))
            femMuscle.transformGeometry(trans.transform);
      }

      // disable nodes if necessary
      if (disableNodes) {
         for (FemNode3d node : femMuscle.getNodes()) {
            node.setDynamic(false);
         }
      }

      /*
       * // subdivide if(name.startsWith("Mylohyoid")) {
       * subdivideFem(femMuscle,1); subdivideFem(femMuscle,2); }
       */
      if (name.startsWith("Sternohyoid_")) {
         // subdivideFem(femMuscle,2);
      }
      if (name.endsWith("_tendon")) {
         // subdivideFem(femMuscle,3);
      }

      // highlight inverted elements
      for (FemElement3d e : femMuscle.getElements()) {
         if (e.isInvertedAtRest()) {
            e.computeVolumes();
            System.out.println(" -Inverted Element " + name + "/elements/" +
               e.getNumber() + ", v(" + e.getVolume() + ")");
            RenderProps.setLineWidth(e, lineWidth * 5);
            RenderProps.setLineColor(e, Color.RED);
         }
      }

      myMechMod.addModel(femMuscle);

      // exclude for cut plane
      /*
       * if(name.matches("Thyroepiglottic_ligament")) {
       * v.myUnClippedRenderList.addIfVisible(femMuscle);
       * v.removeRenderable(femMuscle); }
       */
   }

   /*
    * public void subdivideFem(FemModel3d femMuscle, int divideCase) { int
    * numElem = femMuscle.getElements().size(); int curNum = 0; for(int
    * i=0;i<numElem;i++) { HexElement hex = (HexElement)
    * femMuscle.getElement(curNum);
    * if(!femMuscle.subdivideHexToTwo(hex,divideCase)) { curNum++;
    * //System.out.println(" -Inverted Element " + hex.getNumber() + ", v(" +
    * hex.getVolume() + ")"); RenderProps.setLineWidth (hex, lineWidth*5);
    * RenderProps.setLineColor (hex, Color.RED); } } }
    */
   // ########################################################################

   // ########################################################################
   // #### Subfunction: Add muscle fibres
   // ########################################################################
   private void defineMuscleFibre(MuscleInfo mu) throws IOException
   {
      String name = mu.name;
      // Color color = getComplementaryColor(mu.color);
      FemMuscleModel muscleModel;
      muscleModel = (FemMuscleModel)myMechMod.models().get(name);
      if (debug)
         System.out.println("Defining " + name + " Muscle Fibres");
      // Read list from file
      ArrayList<MuscleFibreInfo> fibreList = readMuscleFibreList(name);
      MuscleBundle bundle;
      Muscle fibre;
      if (debug)
         System.out.println("Number of muscle bundle: " + fibreList.size());
      int count = 0;
      for (MuscleFibreInfo targetFibre : fibreList) {
         if (debug)
            System.out.println("Adding Bundle " + targetFibre.name);
         bundle = new MuscleBundle(muscleModel.getName() + "_" + count);
         for (int[] i : targetFibre.list) {
            fibre = new Muscle();
            fibre.setFirstPoint(muscleModel.getNode(i[0]));
            fibre.setSecondPoint(muscleModel.getNode(i[1]));

            AxialMuscleMaterial mat = null;
            String mtype = null;

            if (mu.muscleType == null) {
               mtype = MUSCLE_FIBRE_TYPE;
            } else {
               mtype = mu.muscleType;
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

            // fibre.resetLengthProps();

            if (!Double.isNaN(mu.muscleMaxForce)) {
               mat.setMaxForce(mu.muscleMaxForce * MUSCLE_MAX_FORCE_SCALING);
            } else {
               mat.setMaxForce(MUSCLE_MAX_FORCE * MUSCLE_MAX_FORCE_SCALING);
            }
            if (!Double.isNaN(mu.muscleForceScaling)) {
               mat.setForceScaling(mu.muscleForceScaling);
            } else {
               mat.setForceScaling(MUSCLE_FORCE_SCALING);
            }
            if (!Double.isNaN(mu.muscleDamping)) {
               mat.setDamping(mu.muscleDamping);
            } else {
               mat.setDamping(MUSCLE_DAMPING);
            }
            if (!Double.isNaN(mu.optLength)) {
               mat.setOptLength(mu.optLength);
            } else {
               mat.setOptLength(fibre.getLength());
            }
            if (!Double.isNaN(mu.maxLength)) {
               mat.setMaxLength(mu.maxLength);
            } else {
               mat.setMaxLength(fibre.getLength() * 1.5);
            }
            if (!Double.isNaN(mu.tendonRatio)) {
               mat.setTendonRatio(mu.tendonRatio);
            } else {
               mat.setTendonRatio(MUSCLE_TENDON_RATIO);
            }
            if (!Double.isNaN(mu.passiveFraction)) {
               mat.setPassiveFraction(mu.passiveFraction);
            } else {
               mat.setPassiveFraction(MUSCLE_PASSIVE_FRACTION);
            }
            fibre.setMaterial(mat);
            bundle.addFibre(fibre);
            if (debug)
               System.out.println("\tFibre(" + i[0] + "," + i[1] + ")");
         }
         muscleModel.addMuscleBundle(bundle);
         bundle.setFibresActive(true);
         // mu.hasBaseBundle = true;
         bundleCount++;

         if (useElementsInsteadOfFibres) {
            RenderProps.setLineWidth(bundle, (int)(lineWidth * 3));
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
            bundle.setExcitationColor(mu.bundleColor);
         } else {
            RenderProps.setLineColor(bundle, mu.bundleColor);
         }
         if (drawBundle == false) {
            RenderProps.setVisible(bundle, drawBundle);
         }
      }
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunction: add FemMarker from Amira file
   // ########################################################################
   public void addFemBundleFromAmiraFile(FemMuscleModel fem,
      String bundleName, String landmarkFileName,
      boolean individualBundleControl) {
      try {
         InputStreamReader reader = 
            new InputStreamReader(
               getInputStream(otherPath, landmarkFileName, otherDest));
         Point3d[][] pts =
            AmiraLandmarkReader.readSets(reader, 1);
         if (pts == null) {
            return;
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
                        ms.bundleColor =
                        NumericProbePanel.colorList[bundleCount
                                                    % NumericProbePanel.colorList.length];
                     else
                        ms.bundleColor =
                        NumericProbePanel.colorList[bundleCount];
                     bundleCount++;

                     if (groupExciters) {
                        muscleExciterList.add(new MuscleExciterInfo(
                           name, ms.bundleColor));
                        MuscleExciter exciter = new MuscleExciter(name);
                        exciter.addTarget(bundle, 1.0);
                        myMechMod.addMuscleExciter(exciter);
                     }
                  } else {
                     MuscleExciter exciter =
                        myMechMod.getMuscleExciters().get(name);
                     exciter.addTarget(bundle, 1.0);
                     ms.bundleColor = color;
                  }

                  if (useElementsInsteadOfFibres) {
                     RenderProps.setLineWidth(bundle, (int)(lineWidth * 3));
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
                        RenderProps.setPointStyle(
                           cur, Renderer.PointStyle.SPHERE);
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
                        ms.bundleColor =
                        NumericProbePanel.colorList[bundleCount
                                                    % NumericProbePanel.colorList.length];
                     else
                        ms.bundleColor =
                        NumericProbePanel.colorList[bundleCount];
                     bundleCount++;

                     if (groupExciters) {
                        muscleExciterList.add(new MuscleExciterInfo(
                           name, ms.bundleColor));
                        MuscleExciter exciter = new MuscleExciter(name);
                        exciter.addTarget(bundle, 1.0);
                        myMechMod.addMuscleExciter(exciter);
                     }
                  } else {
                     MuscleExciter exciter =
                        myMechMod.getMuscleExciters().get(name);
                     exciter.addTarget(bundle, 1.0);
                     ms.bundleColor = color;
                  }

                  if (useElementsInsteadOfFibres) {
                     RenderProps.setLineWidth(bundle, (int)(lineWidth * 3));
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
                        RenderProps.setPointStyle(
                           cur, Renderer.PointStyle.SPHERE);
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
                  ms.bundleColor =
                  NumericProbePanel.colorList[bundleCount
                                              % NumericProbePanel.colorList.length];
               else
                  ms.bundleColor = NumericProbePanel.colorList[bundleCount];
               bundleCount++;

               if (groupExciters) {
                  muscleExciterList.add(new MuscleExciterInfo(
                     name, ms.bundleColor));
                  MuscleExciter exciter = new MuscleExciter(name);
                  exciter.addTarget(bundle, 1.0);
                  myMechMod.addMuscleExciter(exciter);
               }
            } else {
               MuscleExciter exciter = myMechMod.getMuscleExciters().get(name);
               exciter.addTarget(bundle, 1.0);
               ms.bundleColor = color;
            }

            if (useElementsInsteadOfFibres) {
               RenderProps.setLineWidth(bundle, (int)(lineWidth * 3));
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
            System.out.println("Warning: File " + otherPath
               + landmarkFileName + " does not exist!");
      }
   }

   private static FemMarker createAndAddMarker(FemMuscleModel fem, Point3d pnt) {
      FemMarker marker = new FemMarker();

      // add the marker to the model
      FemElement3dBase elem = fem.findContainingElement(pnt);
      if (elem == null) {
         Point3d newLoc = new Point3d();
         elem = fem.findNearestSurfaceElement(newLoc, pnt);
         pnt.set(newLoc);
      }
      marker.setPosition(pnt);
      marker.setFromElement(elem);
      fem.addMarker(marker, elem);
      return marker;
   }

   private Muscle createAndAddFibre(
      Point pointA, Point pointB,
      MuscleBundle bundle, MuscleSpringInfo ms) {
      Muscle spring = new Muscle();
      spring.setFirstPoint(pointA);
      spring.setSecondPoint(pointB);

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
      if (!Double.isNaN(ms.damping)) {
         mat.setDamping(ms.damping);
      } else {
         mat.setDamping(MUSCLE_DAMPING);
      }
      if (!Double.isNaN(ms.maxForce)) {
         mat.setMaxForce(ms.maxForce * MUSCLE_MAX_FORCE_SCALING);
      } else {
         mat.setMaxForce(MUSCLE_MAX_FORCE * MUSCLE_MAX_FORCE_SCALING);
      }
      if (!Double.isNaN(ms.optLength)) {
         mat.setOptLength(ms.optLength);
      } else {
         mat.setOptLength(spring.getLength());
      }
      if (!Double.isNaN(ms.maxLength)) {
         mat.setMaxLength(ms.maxLength);
      } else {
         mat.setMaxLength(spring.getLength() * 1.5);
      }
      if (!Double.isNaN(ms.tendonRatio)) {
         mat.setTendonRatio(ms.tendonRatio);
      } else {
         mat.setTendonRatio(MUSCLE_TENDON_RATIO);
      }
      if (!Double.isNaN(ms.passiveFraction)) {
         mat.setPassiveFraction(ms.passiveFraction);
      } else {
         mat.setPassiveFraction(MUSCLE_PASSIVE_FRACTION);
      }
      if (!Double.isNaN(ms.muscleForceScaling)) {
         mat.setForceScaling(ms.muscleForceScaling);
      } else {
         mat.setForceScaling(MUSCLE_FORCE_SCALING);
      }
      spring.setMaterial(mat);

      bundle.addFibre(spring);
      return spring;
   }

   // ########################################################################
   // #### Subfunction: attach muscle to rigidBody
   // ########################################################################
   private void attachMuscle(MuscleInfo mu) throws IOException
   {
      String name = mu.name;
      String meshName = mu.meshName;
      FemMuscleModel muscleModel = (FemMuscleModel)myMechMod.models().get(name);
      if (debug)
         System.out.println("Attaching " + name + " Nodes");
      // Read list from file
      ArrayList<MuscleAttachInfo> attachList =
         readMuscleAttachmentList(meshName);
      // Attach nodes
      for (MuscleAttachInfo targetAttach : attachList) {
         RigidBody bi = myMechMod.rigidBodies().get(targetAttach.target);
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
                  RenderProps.setPointStyle(
                     muscleModel.getNode(i), Renderer.PointStyle.SPHERE);
                  RenderProps.setPointRadius(
                     muscleModel.getNode(i), pointRadius * 1.5);
                  RenderProps.setPointColor(
                     muscleModel.getNode(i), Color.DARK_GRAY);
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

               myMechMod.attachPoint(n, bi);
               if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
                  RenderProps.setPointStyle(
                     muscleModel.getNode(i), Renderer.PointStyle.SPHERE);
                  RenderProps.setPointRadius(
                     muscleModel.getNode(i), pointRadius * 1.5);
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
   private void attachMuscleToMuscle(MuscleInfo mu) throws IOException
   {
      String name = mu.name;
      String meshName = mu.meshName;
      FemMuscleModel muscleModel = (FemMuscleModel)myMechMod.models().get(name);
      if (debug)
         System.out.println("Attaching " + name + " Nodes");
      // Read list from file
      ArrayList<MuscleAttachInfo> attachList =
         readMuscleToMuscleAttachmentList(meshName);
      // Attach nodes
      for (MuscleAttachInfo targetAttach : attachList) {
         FemModel3d bi =
            (FemModel3d)myMechMod.models().get(targetAttach.target);
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
                  RenderProps.setPointStyle(
                     muscleModel.getNode(i), Renderer.PointStyle.SPHERE);
                  RenderProps.setPointRadius(
                     muscleModel.getNode(i), pointRadius * 1.5);
                  RenderProps.setPointColor(
                     muscleModel.getNode(i), Color.DARK_GRAY);
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

               myMechMod.attachPoint(node, bi);

               // Reverse node attachment if results in inverted elements
               for (FemElement3d el : node.getElementDependencies()) {
                  if (el.computeVolumes() < 0) {
                     invert = true;
                  }
               }
               if (invert) {
                  myMechMod.detachPoint(node);
                  node.setPosition(orig_pos);
                  for (FemElement3d el : node.getElementDependencies()) {
                     if (el.computeVolumes() < 0) {
                        System.out.println(
                           "Warning: inverted element " + el.getNumber());
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

   // ########################################################################
   // #### Subfunction: attach objects automatically
   // ########################################################################
   private void autoAttach() throws IOException {

      for (AutoAttachInfo autoAttachInfo: autoAttachInfoList) {
         // Find first model
         if (autoAttachInfo.type[0].matches("fem")
            && autoAttachInfo.type[1].matches("fem")) {
            FemModel3d model1 =
               (FemModel3d)myMechMod.models().get(autoAttachInfo.name[0]);
            FemModel3d model2 =
               (FemModel3d)myMechMod.models().get(autoAttachInfo.name[1]);
            if(model1!=null && model2!=null) {
               attachFemToFem(model1, model2, autoAttachInfo.distance);
               if(debug==true)
                  System.out.println("Automatically attaching ("
                     + autoAttachInfo.name[0] + ", " + autoAttachInfo.name[1]
                        + ")");
            }
         } else if (autoAttachInfo.type[0].matches("fem")
            && autoAttachInfo.type[1].matches("rigidBody")) {
            FemModel3d model1 =
               (FemModel3d)myMechMod.models().get(autoAttachInfo.name[0]);
            RigidBody model2 =
               myMechMod.rigidBodies().get(autoAttachInfo.name[1]);
            if(model1!=null && model2!=null) {
               attachFemToRigidBody(model1, model2, autoAttachInfo.distance);
               if(debug==true)
                  System.out.println("Automatically attaching ("
                     + autoAttachInfo.name[0] + ", " + autoAttachInfo.name[1]
                        + ")");
            }
         } else if (autoAttachInfo.type[0].matches("rigidBody")
            && autoAttachInfo.type[1].matches("fem")) {
            RigidBody model1 =
               myMechMod.rigidBodies().get(autoAttachInfo.name[0]);
            FemModel3d model2 =
               (FemModel3d)myMechMod.models().get(autoAttachInfo.name[1]);
            if(model1!=null && model2!=null) {
               attachFemToRigidBody(model2, model1, autoAttachInfo.distance);
               if(debug==true)
                  System.out.println("Automatically attaching ("
                     + autoAttachInfo.name[0] + ", " + autoAttachInfo.name[1]
                        + ")");
            }
         } else if ( autoAttachInfo.type[0].matches("rigidBody") 
            && autoAttachInfo.type[1].matches("rigidBody") ) {
            RigidBody model1 =
               myMechMod.rigidBodies().get(autoAttachInfo.name[0]);
            RigidBody model2 =
               myMechMod.rigidBodies().get(autoAttachInfo.name[1]);

            if(model1!=null && model2!=null) {
               attachRigidBodyToRigidBody(model1, model2);
               if(debug==true)
                  System.out.println("Automatically attaching ("
                     + autoAttachInfo.name[0] + ", " + autoAttachInfo.name[1]
                        + ")");
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
   private void
   attachFemToFem(FemModel3d fem1, FemModel3d fem2, double distance) {
      PolygonalMesh mesh = fem2.getSurfaceMesh();
      BVFeatureQuery query = new BVFeatureQuery();
      Point3d orig_pos = new Point3d();
      boolean invert = false;

      for (FemElement3d el : fem1.getElements()) {
         if (el.computeVolumes() < 0) {
            System.out.println(
               "Warning: inverted element " + el.getNumber());
         }
      }

      for (FemNode3d node : fem1.getNodes()) {
         if (query.isInsideOrientedMesh (mesh, node.getPosition(), distance)) {
            if (!node.isAttached()) {
               invert = false;
               orig_pos.set(node.getPosition());

               // Attach node to fem2
               myMechMod.attachPoint(node, fem2);

               // Reverse node attachment if results in inverted elements
               for (FemElement3d el : node.getElementDependencies()) {
                  if (el.computeVolumes() < 0) {
                     invert = true;
                  }
               }
               if (invert) {
                  myMechMod.detachPoint(node);
                  node.setPosition(orig_pos);
                  for (FemElement3d el : node.getElementDependencies()) {
                     if (el.computeVolumes() < 0) {
                        System.out.println(
                           "Warning: inverted element " + el.getNumber());
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
         query.nearestFaceToPoint(
            proj, coords, rb.getMesh(), node.getPosition());

         if (proj.distance(node.getPosition()) < distance) {
            if (!node.isAttached()) {
               myMechMod.attachPoint(node, rb);
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

   // ########################################################################

   // ########################################################################
   // #### Subfunction: attach two RigidBodys together
   // ########################################################################
   private void attachRigidBodyToRigidBody(RigidBody rb1, RigidBody rb2) {

      SolidJoint jnt = new SolidJoint(rb1, rb2);
      jnt.setName(rb1.getName()+"_to_"+rb2.getName());
      myMechMod.addBodyConnector(jnt);

   }

   // ########################################################################

   // ########################################################################
   // #### Subfunction: assign fem bundle color
   // ########################################################################
   public void assignBundleColor() {
      String name1, name2, femName1, femName2;
      Color color;
      int n = 0;
      boolean flag;
      for (int i = 0; i < muscleList.size(); i++) {
         flag = true;
         femName1 = muscleList.get(i).name;
         name1 = femName1.replaceAll("_L", "");
         name1 = name1.replaceAll("_R", "");
         for (int j = 0; j < muscleExciterList.size(); j++) {
            if (name1.matches(muscleExciterList.get(j).name)) {
               flag = false;
            }
         }
         if (flag) {
            if (n >= (NumericProbePanel.colorList.length))
               color =
               NumericProbePanel.colorList[n
                                           % NumericProbePanel.colorList.length];
            else
               color = NumericProbePanel.colorList[n];
            n++;
            muscleList.get(i).bundleColor = color;
            for (int j = i + 1; j < muscleList.size(); j++) {
               femName2 = muscleList.get(j).name;
               name2 = femName2.replaceAll("_L", "");
               name2 = name2.replaceAll("_R", "");
               if (name1.matches(name2)) {
                  muscleList.get(j).bundleColor = color;
                  MuscleExciterInfo mu = new MuscleExciterInfo(name1, color);
                  muscleExciterList.add(mu);
               }
            }
         }
      }
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunction: group fem as exciter
   // ########################################################################
   public void groupFemExciters() {
      String exciterName, name, femName;

      for (int i = 0; i < muscleExciterList.size(); i++) {
         exciterName = muscleExciterList.get(i).name;
         MuscleExciter exciter = new MuscleExciter(exciterName);
         MuscleBundle b;

         for (int j = 0; j < muscleList.size(); j++) {
            femName = muscleList.get(j).name;
            name = femName.replaceAll("_L", "");
            name = name.replaceAll("_R", "");
            if (name.matches(exciterName)) {
               FemMuscleModel fem =
                  (FemMuscleModel)myMechMod.models().get(femName);
               for (int k = 0; k < fem.getMuscleBundles().size(); k++) {
                  b = fem.getMuscleBundles().get(k);
                  exciter.addTarget((ExcitationComponent)b, 1.0);
               }
            }
         }
         if (exciter.numTargets() > 0) {
            myMechMod.addMuscleExciter(exciter);
         }
      }
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunction: attach fem particles
   // ########################################################################
   private void attachParticles(String filename) throws IOException {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         FemMuscleModel fem1, fem2;
         int node1, node2;
         String femName1, femName2;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype != ReaderTokenizer.TT_WORD)
               throw new IOException("Frame Marker File incorrect format: "
                  + filename);
            femName1 = rtok.sval;
            rtok.nextToken();
            femName2 = rtok.sval;
            fem1 = (FemMuscleModel)myMechMod.models().get(femName1);
            fem2 = (FemMuscleModel)myMechMod.models().get(femName2);
            if (femName1 != "end" && femName2 != "end") {
               if (debug)
                  System.out.println("Attaching nodes between \"" + femName1
                     + "\" and \"" + femName2 + "\":");
               while (rtok.nextToken() != ReaderTokenizer.TT_WORD) {
                  node1 = (int)rtok.nval;
                  rtok.nextToken();
                  node2 = (int)rtok.nval;
                  if (fem1 != null && fem2 != null) {
                     myMechMod.attachPoint(
                        fem1.getNode(node1), fem2.getNode(node2));
                     if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
                        RenderProps.setPointRadius(
                           fem1.getNode(node1), pointRadius);
                        RenderProps.setPointStyle(
                           fem1.getNode(node1), Renderer.PointStyle.SPHERE);
                        RenderProps.setPointColor(
                           fem1.getNode(node1), Color.GREEN);
                        RenderProps.setPointRadius(
                           fem2.getNode(node2), pointRadius);
                        RenderProps.setPointStyle(
                           fem2.getNode(node2), Renderer.PointStyle.SPHERE);
                        RenderProps.setPointColor(
                           fem2.getNode(node2), Color.GREEN);
                     }
                     if (debug)
                        System.out.println("\tNode " + node1 + " ==> Node "
                           + node2);
                  }
               }
            }
         }

      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   // ########################################################################

   // ========================================================================
   // EDIT: Add Fem markers (Sanchez, Nov 29,2011)
   // ========================================================================
   private void addAllFemMarkers(String filename) throws IOException {

      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         double[] pointLoc = new double[3];
         MechSystemModel model;
         FemMarker marker;
         String pName, bodyName;
         FemModel3d fem;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype != ReaderTokenizer.TT_WORD)
               throw new IOException("Fem Marker File incorrect format: "
                  + filename);
            fem = null;
            marker = new FemMarker();
            pName = rtok.sval;
            rtok.nextToken();
            bodyName = rtok.sval;
            rtok.nextToken();
            pointLoc[0] = rtok.nval;
            rtok.nextToken();
            pointLoc[1] = rtok.nval;
            rtok.nextToken();
            pointLoc[2] = rtok.nval;

            model = myMechMod.models().get(bodyName);

            if (model instanceof FemModel3d) {
               fem = (FemModel3d)model;

               marker.setPosition(new Point3d(
                  pointLoc[0], pointLoc[1], pointLoc[2]));
               marker.setName(pName);

               if (debug)
                  System.out.println("Setting frame marker \"" + pName
                     + "\": (" + pointLoc[0] + ", " + pointLoc[1] + ", "
                     + pointLoc[2] + ")");

               if (drawAttachedNodes != drawNodes && drawAttachedNodes) {
                  RenderProps.setPointRadius(marker, pointRadius);
                  RenderProps.setPointStyle(
                     marker, Renderer.PointStyle.SPHERE);
                  RenderProps.setPointColor(marker, Color.GREEN);
               }

               fem.addMarker(marker);
            } else {
               if (debug) {
                  System.out
                  .println("Warning: Invalid FEM model, cannot attach nodes!");
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   // ========================================================================

   // ########################################################################
   // #### Subfunction: Add frame markers
   // ########################################################################
   private void addAllFrameMarker(String filename) throws IOException {

      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         double[] pointLoc = new double[3];
         FrameMarker marker;
         String pName, bodyName;
         RigidBody body;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype != ReaderTokenizer.TT_WORD)
               throw new IOException("Frame Marker File incorrect format: "
                  + filename);
            marker = new FrameMarker();
            pName = rtok.sval;
            rtok.nextToken();
            bodyName = rtok.sval;
            rtok.nextToken();
            pointLoc[0] = rtok.nval;
            rtok.nextToken();
            pointLoc[1] = rtok.nval;
            rtok.nextToken();
            pointLoc[2] = rtok.nval;
            body = myMechMod.rigidBodies().get(bodyName);
            marker.setName(pName);
            if (body != null) {
               if (debug)
                  System.out.println("Setting frame marker \"" + pName
                     + "\": (" + pointLoc[0] + ", " + pointLoc[1] + ", "
                     + pointLoc[2] + ")");

               if (drawAttachedNodes) {
                  RenderProps.setPointRadius(marker, pointRadius * 1.5);
                  RenderProps.setPointStyle(
                     marker, Renderer.PointStyle.SPHERE);
                  RenderProps.setPointColor(marker, Color.BLUE);
               }

               // XXX Sanchez: I'm guessing location is in world coordinates, since
               //              originally was applied before setting the frame.
               Point3d pos = new Point3d(pointLoc[0], pointLoc[1], pointLoc[2]);
               if (centerRigidBodies) {
                  pos.sub(bodyTranslationMap.get(body)); // subtract adjusted position
               }             
               marker.setFrame(body);
               marker.setWorldLocation(pos); // XXX maybe use setLocation(pos)
               myMechMod.addFrameMarker(marker);
            } else if (body == null && myMechMod.rigidBodies().size() > 0) {
               body = block;
               if (debug) {
                  System.out
                  .println("Warning: Cannot attach nodes to RigidBody \""
                     + bodyName + "\", it does not exist.");
                  System.out
                  .println("         attaching nodes to a reference RigidBody \""
                     + body.getName() + "\" instead.");
               }
               if (debug)
                  System.out.println("Setting frame marker \"" + pName
                     + "\": (" + pointLoc[0] + ", " + pointLoc[1] + ", "
                     + pointLoc[2] + ")");

               if (drawAttachedNodes) {
                  RenderProps.setPointRadius(marker, pointRadius * 1.5);
                  RenderProps.setPointStyle(
                     marker, Renderer.PointStyle.SPHERE);
                  RenderProps.setPointColor(marker, Color.DARK_GRAY);
               }

               marker.setFrame(body);
               Point3d pos = new Point3d(pointLoc[0], pointLoc[1], pointLoc[2]);
               marker.setWorldLocation(pos);
               myMechMod.addFrameMarker(marker);
            } else if (debug) {
               System.out.println("Warning: Cannot attach nodes!");
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunction: add fem bundle springs
   // ########################################################################
   private void addFemBundleSpring(String filename) throws IOException {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         rtok.wordChars(".");
         rtok.wordChars("-");

         String bundleName, landmarkFileName;
         FemMuscleModel muscleModel;
         String individualBundleControl;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype == ReaderTokenizer.TT_WORD)
            {
               muscleModel = (FemMuscleModel)myMechMod.models().get(rtok.sval);
               while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
                  bundleName = rtok.sval;
                  if (muscleModel != null && !bundleName.matches("(?i:end)")) {
                     rtok.nextToken();
                     landmarkFileName = rtok.sval;
                     rtok.nextToken();
                     individualBundleControl = rtok.sval;
                     if (individualBundleControl
                        .matches("-individualBundleControl")) {
                        addFemBundleFromAmiraFile(
                           muscleModel, bundleName, landmarkFileName, true);
                     } else {
                        rtok.pushBack();
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
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   // ########################################################################

   // ########################################################################
   // #### Subfunction: add muscle springs
   // ########################################################################
   private void addMuscleSpring(String filename) throws IOException {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         RenderableComponentList<FrameMarker> markers =
            myMechMod.frameMarkers();
         String springName, pointName1, pointName2, ownerName1, ownerName2;
         Marker currP1, currP2;
         Muscle spring;
         MuscleSpringInfo ms;
         FemModel3d fem;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype == ReaderTokenizer.TT_WORD)
            {
               int fibreCount = 0;
               ms = new MuscleSpringInfo();
               springName = rtok.sval;
               muscleSpringList.add(springName);

               for (MuscleSpringInfo muscleSpringInfo : muscleSpringInfoList) {
                  if (muscleSpringInfo.name.matches(springName)) {
                     ms = muscleSpringInfo;
                  }
               }

               if (debug)
                  System.out.println("Defining muscle spring " + springName);
               while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
                  ownerName1 = rtok.sval;
                  if (!ownerName1.matches("(?i:end)")) {
                     rtok.nextToken();
                     pointName1 = rtok.sval;
                     rtok.nextToken();
                     ownerName2 = rtok.sval;
                     rtok.nextToken();
                     pointName2 = rtok.sval;
                     currP1 = null;
                     currP2 = null;
                     if (ownerName1.matches("frameMarker")) {
                        currP1 = markers.get(pointName1);
                     } else {
                        fem = (FemModel3d)myMechMod.models().get(ownerName1);
                        if (fem != null) {
                           currP1 = fem.markers().get(pointName1);
                        }
                     }
                     if (ownerName2.matches("frameMarker")) {
                        currP2 = markers.get(pointName2);
                     } else {
                        fem = (FemModel3d)myMechMod.models().get(ownerName2);
                        if (fem != null) {
                           currP2 = fem.markers().get(pointName2);
                        }
                     }
                     if (currP1 != null && currP2 != null) {
                        spring = new Muscle(currP1, currP2);
                        spring.setName(springName + "_" + fibreCount);

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
                        if (!Double.isNaN(ms.damping)) {
                           mat.setDamping(ms.damping);
                        } else {
                           mat.setDamping(SPRING_MUSCLE_DAMPING);
                        }
                        if (!Double.isNaN(ms.maxForce)) {
                           mat.setMaxForce(ms.maxForce
                              * MUSCLE_MAX_FORCE_SCALING);
                        } else {
                           mat.setMaxForce(SPRING_MUSCLE_MAX_FORCE
                              * MUSCLE_MAX_FORCE_SCALING);
                        }
                        if (!Double.isNaN(ms.optLength)) {
                           mat.setOptLength(ms.optLength);
                        } else {
                           mat.setOptLength(spring.getLength());
                        }
                        if (!Double.isNaN(ms.maxLength)) {
                           mat.setMaxLength(ms.maxLength);
                        } else {
                           mat.setMaxLength(spring.getLength() * 1.5);
                        }
                        if (!Double.isNaN(ms.tendonRatio)) {
                           mat.setTendonRatio(ms.tendonRatio);
                        } else {
                           mat.setTendonRatio(SPRING_MUSCLE_TENDON_RATIO);
                        }
                        if (!Double.isNaN(ms.passiveFraction)) {
                           mat.setPassiveFraction(ms.passiveFraction);
                        } else {
                           mat
                           .setPassiveFraction(SPRING_MUSCLE_PASSIVE_FRACTION);
                        }
                        if (!Double.isNaN(ms.muscleForceScaling)) {
                           mat.setForceScaling(ms.muscleForceScaling);
                        } else {
                           mat.setForceScaling(SPRING_MUSCLE_FORCE_SCALING);
                        }
                        spring.setMaterial(mat);

                        // Set rendering
                        RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
                        RenderProps.setLineRadius(spring, lineRadius * 3);
                        if (drawBundleExcitation) {
                           RenderProps.setLineColor(spring, Color.WHITE);
                           spring.setExcitationColor(Color.RED);
                        } else {
                           RenderProps.setLineColor(spring, Color.RED);
                        }

                        myMechMod.addAxialSpring(spring);
                        fibreCount++;
                     }
                  } else {
                     // Group exciters
                     if (fibreCount > 0) {
                        System.out.println("Grouping " + springName);
                        MuscleExciter exciter = new MuscleExciter(springName);
                        for (int i = 0; i < fibreCount; i++) {
                           ExcitationComponent c =
                              (ExcitationComponent)myMechMod
                              .axialSprings().get(springName + "_" + i);
                           if (c != null) {
                              exciter.addTarget(c, 1.0);
                           }
                        }
                        myMechMod.addMuscleExciter(exciter);
                        fibreCount = 0;
                     }
                     break;
                  }
               }
            }
         }
         if (groupExciters) {
            // Group left and right spring muscles
            int numExciters = myMechMod.getMuscleExciters().size();
            MuscleExciter ex, ex1, ex2;
            String exciterName, exciterName1, exciterName2;
            for (int i = 0; i < numExciters; i++) {
               exciterName1 = myMechMod.getMuscleExciters().get(i).getName();
               if (exciterName1.indexOf("_L") > -1) {
                  exciterName2 = exciterName1.replaceAll("_L", "_R");
                  ex1 = myMechMod.getMuscleExciters().get(exciterName1);
                  ex2 = myMechMod.getMuscleExciters().get(exciterName2);
                  if (ex1 != null && ex2 != null) {
                     exciterName = exciterName1.replaceAll("_L", "");
                     ex = new MuscleExciter(exciterName);
                     ex.addTarget((ExcitationComponent)ex1, 1.0);
                     ex.addTarget((ExcitationComponent)ex2, 1.0);
                     myMechMod.addMuscleExciter(ex);
                     muscleSpringGroupList.add(exciterName);
                  }
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   // ########################################################################
   // #### Subfunction: add passive springs
   // ########################################################################
   private void addPassiveSpring(String filename) throws IOException {
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new InputStreamReader(
               getInputStream(otherPath, filename, otherDest)));
         RenderableComponentList<FrameMarker> markers =
            myMechMod.frameMarkers();
         String springName, pointName1, pointName2, ownerName1, ownerName2;
         AxialSpring spring;
         SpringInfo ms;
         Marker currP1, currP2;
         FemModel3d fem;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            if (rtok.ttype == ReaderTokenizer.TT_WORD)
            {
               int fibreCount = 0;
               ms = new SpringInfo();
               springName = rtok.sval;

               for (SpringInfo springInfo : springInfoList) {
                  if (springInfo.name.matches(springName)) {
                     ms = springInfo;
                  }
               }

               if (debug)
                  System.out.println("Defining muscle spring " + springName);
               while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
                  ownerName1 = rtok.sval;
                  if (!ownerName1.matches("(?i:end)")) {
                     rtok.nextToken();
                     pointName1 = rtok.sval;
                     rtok.nextToken();
                     ownerName2 = rtok.sval;
                     rtok.nextToken();
                     pointName2 = rtok.sval;
                     currP1 = null;
                     currP2 = null;
                     if (ownerName1.matches("frameMarker")) {
                        currP1 = markers.get(pointName1);
                     } else {
                        fem = (FemModel3d)myMechMod.models().get(ownerName1);
                        if (fem != null) {
                           currP1 = fem.markers().get(pointName1);
                        }
                     }
                     if (ownerName2.matches("frameMarker")) {
                        currP2 = markers.get(pointName2);
                     } else {
                        fem = (FemModel3d)myMechMod.models().get(ownerName2);
                        if (fem != null) {
                           currP2 = fem.markers().get(pointName2);
                        }
                     }
                     if (currP1 != null && currP2 != null) {
                        spring = new AxialSpring(springName + "_" + fibreCount);
                        spring.setFirstPoint(currP1);
                        spring.setSecondPoint(currP2);

                        double k, d, l;
                        // Set properties
                        if (!Double.isNaN(ms.damping)) {
                           d = ms.damping;
                        } else {
                           d = SPRING_DAMPING;
                        }
                        if (!Double.isNaN(ms.stiffness)) {
                           k = ms.stiffness;
                        } else {
                           k = SPRING_STIFFNESS;
                        }
                        if (!Double.isNaN(ms.restLength)) {
                           l = ms.restLength;
                        } else {
                           l = spring.getLength();
                        }
                        spring.setRestLength(l);
                        spring.setLinearMaterial(k, d);
                        // Set rendering
                        RenderProps.setLineStyle(spring, LineStyle.CYLINDER);
                        RenderProps.setLineRadius(spring, lineRadius);
                        RenderProps.setLineColor(spring, Color.LIGHT_GRAY);
                        myMechMod.addAxialSpring(spring);

                        fibreCount++;
                     }
                  } else {
                     fibreCount = 0;
                     break;
                  }
               }
            }
         }
         rtok.close();
      } catch (IOException e) {
         if (debug)
            System.out.println("Warning: File " + otherPath + filename
               + " does not exist!");
      }
   }

   /*
    * private void addMultiPointSpring(String filename) throws IOException { try
    * { ReaderTokenizer rtok = new ReaderTokenizer( new FileReader(otherPath +
    * filename)); RenderableComponentList<FrameMarker> markers =
    * myMechMod.frameMarkers(); String springName, pointName; FrameMarker currP;
    * MultiPointSpring spring; SpringInfo ms; while (rtok.nextToken() !=
    * ReaderTokenizer.TT_EOF) { if (rtok.ttype == ReaderTokenizer.TT_WORD) {
    * springName = rtok.sval; spring = new MultiPointSpring(springName, 0,0,0);
    * ms = new SpringInfo(); springName = rtok.sval; for(SpringInfo
    * springInfo:springInfoList) { if(springInfo.name.matches(springName)) { ms
    * = springInfo; } } // Set properties if(!Double.isNaN(ms.damping)) {
    * spring.setDamping(ms.damping); } else {
    * spring.setDamping(MUSCLE_PARTICLE_DAMPING); }
    * if(!Double.isNaN(ms.stiffness)) { spring.setStiffness(ms.stiffness); }
    * else { spring.setStiffness(MUSCLE_STIFFNESS_DAMPING); }
    * if(!Double.isNaN(ms.restLength)) { spring.setRestLength(ms.restLength); }
    * if(debug) System.out.println("Defining multi-point spring " + springName);
    * while (rtok.nextToken() != ReaderTokenizer.TT_EOF) { pointName =
    * rtok.sval; if(!pointName.matches("(?i:end)")) { currP =
    * markers.get(pointName); if(currP!=null) { spring.addPoint(currP);
    * RenderProps.setPointColor(currP, Color.DARK_GRAY);
    * RenderProps.setPointRadius (currP, pointRadius*1.5); } } else {
    * if(spring.numPoints()>0) { if(Double.isNaN(ms.restLength))
    * spring.setRestLength(spring.getActiveLength());
    * RenderProps.setLineStyle(spring, LineStyle.CYLINDER);
    * RenderProps.setLineRadius (spring, lineRadius); RenderProps.setLineColor
    * (spring, Color.LIGHT_GRAY); myMechMod.addMultiPointSpring(spring); }
    * break; } } } } rtok.close(); } catch (IOException e) { if(debug)
    * System.out.println("Warning: File " + otherPath + filename +
    * " does not exist!"); } }
    */
   // ########################################################################

   // ########################################################################
   // #### Subfunctions: Control Panel
   // ########################################################################
   public void attach(DriverInterface driver) {
      setWorkingDir();
      loadProbes();

      super.attach(driver);
      if (getControlPanels().size() == 0 && SingleMusclePanels) {

         // Add a control panel for each fem
         // int i = 0;
         for (MuscleInfo mu : muscleList) {
            FemModel3d muscle;
            muscle = (FemModel3d)myMechMod.models().get(mu.name);
            createControlPanel(
               mu, this, myModels.get(0), muscle);
            // i++;
         }

      }
      if (muscleControlPanel) {
         if (groupExciters) {
            createAllExcitersPanel();
         } else {
            createAllMusclesPanel();
         }
      }
      // Add a control panel for viewing options
      createVisibilityPanel();
      if (includeWayPoints)
         setWayPoints();
   }

   public ControlPanel createControlPanel(
      MuscleInfo mu, RootModel root, ModelComponent topModel,
      FemModel3d muscle) {
      ControlPanel controlPanel = null;
      controlPanel = new ControlPanel(mu.name, "LiveUpdate");
      controlPanel.setScrollable(true);
      addControls(controlPanel, muscle, mu.bundleColor);
      root.addControlPanel(controlPanel);

      return controlPanel;
   }

   protected void
   addControls(ControlPanel panel, FemModel3d muscle, Color color) {
      FemControlPanel.addMuscleControls(panel, muscle, muscle);
      panel.addWidget(muscle, "profile");
      if (((FemMuscleModel)muscle).getMuscleBundles().size() > 0) {
         ComponentList<MuscleBundle> muscles =
            ((FemMuscleModel)muscle).getMuscleBundles();
         for (int i = 0; i < muscles.size(); ++i) {
            DoubleFieldSlider slider =
               (DoubleFieldSlider)panel.addWidget(
                  "activation", myMechMod, "models/" + muscle.getName()
                  + "/bundles/" + i + ":excitation", 0,
                  1);
            slider.setRoundingTolerance(0.001);
            slider.getLabel().setForeground(color);
            BooleanSelector selector =
               (BooleanSelector)PropertyWidget.create(
                  "active", muscles.get(i),
                  "fibresActive");
            slider.getLabel().setForeground(color);
            slider.add(selector);
            BooleanSelector checkBox =
               (BooleanSelector)PropertyWidget.create(
                  "visible", muscles.get(i), "renderProps.visible");
            slider.add(checkBox);
            checkBox.addValueChangeListener(new ValueChangeListener() {
               public void valueChange(ValueChangeEvent e) {
                  rerender();
               }
            });
            slider.add(checkBox);
         }
      }
      panel.addWidget(new JSeparator());
   }

   public void createVisibilityPanel() {

      if (myMechMod == null)
         return;
      ControlPanel panel = new ControlPanel("Show", "LiveUpdate");

      panel.addWidget(
         "FrameMarkers", myMechMod.frameMarkers(), "renderProps.visible");
      panel.addWidget(new JSeparator());
      panel.addWidget(
         "AxialSprings", myMechMod.axialSprings(), "renderProps.visible");
      panel.addWidget(new JSeparator());
      for (RigidBody body : myMechMod.rigidBodies()) {
         if (!body.getName().matches("ref_block"))
            panel.addWidget(body.getName(), body, "renderProps.visible");
      }
      panel.addWidget(new JSeparator());
      for (Model mod : myMechMod.models()) {
         panel.addWidget(mod.getName(), mod, "renderProps.visible");
      }
      addControlPanel(panel);
   }

   public void createAllExcitersPanel() {
      if (myMechMod == null)
         return;
      ControlPanel panel = new ControlPanel("Muscle Controls", "LiveUpdate");
      String name;
      Color color;

      for (int i = 0; i < muscleExciterList.size(); i++) {
         name = muscleExciterList.get(i).name;
         color = muscleExciterList.get(i).color;
         if (myMechMod.getMuscleExciters().get(name) != null) {
            DoubleFieldSlider slider =
               (DoubleFieldSlider)panel.addWidget(
                  name, myMechMod,
                  "exciters/" + name + ":excitation",
                  0, 1);
            slider.setRoundingTolerance(0.001);
            slider.getLabel().setForeground(color);
         }
      }
      for (String springName : muscleSpringGroupList) {
         if (myMechMod.getMuscleExciters().get(springName) != null) {
            DoubleFieldSlider slider =
               (DoubleFieldSlider)panel.addWidget(
                  springName, myMechMod,
                  "exciters/" + springName + ":excitation",
                  0, 1);
            slider.setRoundingTolerance(0.001);
         }
      }
      addControlPanel(panel);
   }

   public void createAllMusclesPanel() {
      if (myMechMod == null)
         return;
      ControlPanel panel = new ControlPanel("Muscle Controls", "LiveUpdate");
      FemModel3d muscle;
      String name;
      Color color;
      for (int k = 0; k < muscleList.size(); k++)
      {
         name = muscleList.get(k).name;
         muscle = (FemModel3d)myMechMod.models().get(name);
         MuscleSpringInfo ms;
         String bundleName;

         if (((FemMuscleModel)muscle).getMuscleBundles().size() > 0) {
            ComponentList<MuscleBundle> muscles =
               ((FemMuscleModel)muscle).getMuscleBundles();
            for (int i = 0; i < muscles.size(); ++i) {
               ms = null;
               bundleName = muscles.get(i).getName();
               DoubleFieldSlider slider =
                  (DoubleFieldSlider)panel.addWidget(
                     bundleName, myMechMod,
                     "models/" + name + "/bundles/" + i + ":excitation",
                     0, 1);
               slider.setRoundingTolerance(0.001);
               for (MuscleSpringInfo muscleSpringInfo : muscleSpringInfoList) {
                  if (muscleSpringInfo.name.matches(bundleName)) {
                     ms = muscleSpringInfo;
                  }
               }
               if (ms != null && ms.bundleColor != null) {
                  color = ms.bundleColor;
               } else {
                  color = muscleList.get(k).bundleColor;
               }
               slider.getLabel().setForeground(color);
               BooleanSelector selector =
                  (BooleanSelector)PropertyWidget.create(
                     "active", muscles.get(i),
                     "fibresActive");
               slider.getLabel().setForeground(color);
               slider.add(selector);
               BooleanSelector checkBox =
                  (BooleanSelector)PropertyWidget.create(
                     "visible", muscles.get(i), "renderProps.visible");
               slider.add(checkBox);
               checkBox.addValueChangeListener(new ValueChangeListener() {
                  public void valueChange(ValueChangeEvent e) {
                     rerender();
                  }
               });
               slider.add(checkBox);
            }
         }
      }
      for (String springName : muscleSpringList) {
         // MultiPointSpring spring =
         // myMechMod.multiPointSprings().get(springName);
         // AxialSpring spring = myMechMod.axialSprings().get(springName);
         // DoubleFieldSlider slider =
         // (DoubleFieldSlider)panel.addWidget (spring.getName(), myMechMod,
         // "multiPointSprings/" + spring.getName() +
         panel.addWidget(
            springName, myMechMod, "exciters/" + springName +
            ":excitation", 0.0, 1.0);
         /*
          * BooleanSelector selector = (BooleanSelector)PropertyWidget.create (
          * "enabled", spring, "enabled"); slider.add (selector);
          * BooleanSelector checkBox = (BooleanSelector)PropertyWidget.create (
          * "visible", spring, "renderProps.visible"); slider.add (checkBox);
          * checkBox.addValueChangeListener (new ValueChangeListener() { public
          * void valueChange (ValueChangeEvent e) {
          * rerender(); } }); slider.add (checkBox);
          */
      }
      addControlPanel(panel);
   }

   // ########################################################################
   // #### Subfunctions: Probes
   // ########################################################################
   public void setWorkingDir()
   {
      if (workingDirname == null)
         return;
      // set default working directory to repository location
      File workingDir = new File(workingDirname);
      if (!workingDir.isAbsolute()) {
         workingDir = ArtisynthPath.getSrcRelativeFile(
            this, workingDirname);
      }
      ArtisynthPath.setWorkingDir(workingDir);
      if (debug)
      {
         System.out.println("Set working directory to " +
            ArtisynthPath.getWorkingDir().getAbsolutePath());
      }
   }

   private String uriFriendly(String str) {

      // replace slashes
      str = str.replace('\\', '/');
      return str;

   }

   private String uriCat(String a, String b) {
      String out = "";
      if (a != null) {
         out = out + a;
      }
      if (b != null & !b.equals("")) {
         if (a.endsWith("/") && b.startsWith("/")) {
            out = out.substring(0, out.length()-1) + b;
         } else if (!out.endsWith("/") && !b.startsWith("/")) {
            out = out + "/" + b;
         } else {
            out = out + b;
         }
      }
      return out;
   }

   protected InputStream getInputStream(String source, String fileName, String destFolder) throws IOException {
      // first check for local file

      if (source == null) {
         source = "file://" + ArtisynthPath.getWorkingDirPath() + "/";
      }
      if (destFolder == null) {
         destFolder = "";
      }

      source = uriFriendly(source);
      fileName = uriFriendly(fileName);
      String uriStr = uriCat(source,fileName);      
      
      URIx uri = new URIx(uriStr);

      // check uri, if it is relative, then we can try to create a file
      if (uri.isRelative()) {
         String fullPath = uri.getPath();
         File f = new File(fullPath);
         if (f.canRead()) {
            if (debug) {
               System.out.println("Reading file " + f.getAbsolutePath());
            }
            return new FileInputStream(f);
         }

         // could be an absolute file name, so first check that
         // and assign file scheme
         if (f.isAbsolute()) {
            uri.setScheme("file");
         }         
      }

      try {

         // if we are downloading/extracting files, make a local copy
         if (downloadFiles) {
            File f = fileManager.get(new File(destFolder, fileName), uri);
            if (debug) {
               System.out.println("Reading file " + f.getAbsolutePath());
            }
            return new FileInputStream(f);
         } else {
            // otherwise, just return a stream
            File f = new File(destFolder, fileName);
            if (debug) {
               System.out.println("Reading file " + f.getAbsolutePath());
            }
            return fileManager.getInputStream(f, uri, fileManager.getOptions());
         }
      } catch (Exception e) {
         throw new IOException("Cannot read file: " + fileName, e);
      }

   }

   public void closeStreams() {
      if (fileManager != null) {
         fileManager.closeStreams();
      }
   }

   public void loadProbes()
   {
      if (probesFilename == null || !myInputProbes.isEmpty()
         || !myOutputProbes.isEmpty())
         return;

      try
      {
         InputStream in = getInputStream(probesPath, probesFilename, probesDest);
         scanProbes(
            ArtisynthIO.newReaderTokenizer(new InputStreamReader(in)));
         System.out.println("Loaded Probes from File: " + probesFilename);
      } catch (Exception e)
      {
         if (debug) {
            System.out.println("Error reading probe file");
            e.printStackTrace();
         }
      }
   }

   public String getAbout()
   {
      return artisynth.core.util.TextFromFile.getTextOrError(
         ArtisynthPath.getSrcRelativeFile(
            this, "about_ModelTemplate.txt"));
   }

   public void setWayPoints() {
      removeAllWayPoints();
      addWayPoints(this, stopPoint, wayPointStep);
   }

   public static void addWayPoints(RootModel root, double duration,
      double waypointstep) {
      for (int i = 1; i < duration / waypointstep; i++) {
         root.addWayPoint(i * waypointstep);
      }
      root.addBreakPoint(duration);
   }

   public Point3d getCenter() {
      // find the center of the components
      Point3d max = new Point3d(-inf, -inf, -inf);
      Point3d min = new Point3d(inf, inf, inf);
      myMechMod.updateBounds(min, max);

      Point3d center = new Point3d();
      center.add(min, max);
      center.scale(0.5);
      if (Double.isNaN(center.x) || Double.isNaN(center.x)
         || Double.isNaN(center.z)) {
         center.setZero();
      }
      return center;
   }

   public Point3d getCenter(RigidBody body) {
      // find the center of the components
      Point3d max = new Point3d(-inf, -inf, -inf);
      Point3d min = new Point3d(inf, inf, inf);
      body.updateBounds(min, max);

      Point3d center = new Point3d();
      center.add(min, max);
      center.scale(0.5);
      return center;
   }

   public void setFrameMarkerTracing(String name) {
      FrameMarker fm = myMechMod.frameMarkers().get(name);
      if (fm != null) {
         enableTracing(fm);
         RenderProps.setPointStyle(fm, PointStyle.SPHERE);
         RenderProps.setLineStyle(fm, LineStyle.CYLINDER);
         RenderProps.setLineColor(fm, Color.GREEN.darker());
         RenderProps.setPointColor(fm, Color.GREEN.darker());
         RenderProps.setLineRadius(fm, lineRadius);
         RenderProps.setPointRadius(fm, pointRadius * 5);
         RenderProps.setVisible(fm, true);
      }
   }

   public void setParticleTracing(String name) {
      Particle fm = myMechMod.particles().get(name);
      if (fm != null) {
         enableTracing(fm);
         RenderProps.setPointStyle(fm, PointStyle.SPHERE);
         RenderProps.setLineStyle(fm, LineStyle.CYLINDER);
         RenderProps.setLineColor(fm, Color.CYAN);
         RenderProps.setPointColor(fm, Color.CYAN);
         RenderProps.setLineRadius(fm, lineRadius);
         RenderProps.setPointRadius(fm, pointRadius * 4);
      }
   }
}
