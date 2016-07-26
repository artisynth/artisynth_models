package artisynth.models.larynx;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.TriangleIntersector;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLGridResolution;
import maspack.render.GL.GLViewer;
import maspack.render.Dragger3d.DraggerType;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.driver.Main;
import maspack.matrix.AxisAlignedRotation;
import artisynth.core.driver.ViewerManager;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechSystemBase;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.TimeBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.template.ModelTemplate;
import artisynth.tools.femtool.HybridFemFactory;
import artisynth.tools.femtool.HybridFemGenerator;

public class VHLarynxDemo extends ModelTemplate {
   public VHLarynxDemo() {
   }
   
   public VHLarynxDemo(String name) throws IOException{
      super(name);
      
      MechSystemBase.setDefaultStabilization(PosStabilization.GlobalStiffness);
      
      //#####################################################################
      // CONTROLS
      super.debug = false;
      
      super.disableNodes = false;
      super.IncompressOption = false;
      super.SetCollision = false;
      super.groupExciters = true;
      super.muscleControlPanel = true;
      super.SingleMusclePanels = false;
      super.drawNodes = false;
      super.drawAttachedNodes = false;
      super.drawBundle = true;
      super.drawBundleExcitation = true;
      super.muscleShaded = true;
      super.useElementsInsteadOfFibres = false;
      
      super.downloadFiles = false;  // prevent extracting geometry
      String basePath = ArtisynthPath.getSrcRelativePath(this, "");
      super.rigidBodyPath = "zip:file://" + basePath + "geometry/rigidBodies/meshes.zip!";;
      super.rigidBodyDest = basePath + "geometry/rigidBodies/";
      super.femPath = basePath + "geometry/fem/";
      super.otherPath = basePath + "geometry/other/";
      
      //super.bodyListAllFilename = "bodyListAll.txt";
      super.bodyListAllFilename = "bodyList_extrinsic.txt";
      super.bodyPropertyListFilename = "bodyPropertyList.txt";
      super.bodyTransformListAllFilename = "";
      super.femListAllFilename = "femList_extrinsic.txt";
      //super.femListAllFilename = "femList_extrinsic_supra.txt";
      //super.femListAllFilename = "femListAll.txt";
      super.femPropertyListFilename = "femMusclePropertyList.txt";
      super.femTransformListAllFilename = "";
      super.femAttachParticleFilename = "femAttachAll.txt";
      super.frameMarkerListFilename = "frameMarker.txt";
      //super.muscleSpringListFilename = "frameMuscle.fibre";
      super.muscleSpringListFilename = "frameMuscle_stylo.fibre";
      super.muscleSpringPropertyListFilename = "springMuscleProperty.txt";
      super.springListFilename = "spring.fibre";
      super.springPropertyListFilename = ""; //"springProperty.txt";
      //super.collisionListFilename = "collision.txt";
      
      // For swallowing motion
      super.probesPath = basePath + "data/ninds/";
      super.probesFilename = "probeMuscleInput.art";
      super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_108/trial12";
      //super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_110/trial16";
      //super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_111/trial09";
      
      // For hyoid range
      //super.workingDirname = "src/artisynth/models/larynx/data";
      //super.probesFilename = "probes_exciters_stepUp_range.art";
      
      super.aboutFilename = "src/artisynth/models/larynx/about_VHLarynxDemo.txt";
      //#####################################################################
      
      // Step size
      super.MAX_STEP_SIZE_SEC = 1.0e-3; // 1 msec
      // all units in mm
      super.addMidElementsWithin = 5;
      super.elementDirectionRenderLen = 0.5;
      super.GRAVITY = 9800;
      super.BODY_DENSITY = 0.00000185;
      super.MUSCLE_DENSITY = 0.000000588;
      super.MUSCLE_PARTICLE_DAMPING = 40;
      super.MUSCLE_STIFFNESS_DAMPING = 0.0005;
      super.MUSCLE_YOUNGS_MODULUS = 10;
      super.MUSCLE_MAXLAMBDA = 2.1;
      super.MUSCLE_MAXSTRESS = 300;
      super.MUSCLE_MAX_FORCE = 2.0;
      super.MUSCLE_FORCE_SCALING = 1000;
      super.MUSCLE_MAX_FORCE_SCALING = 1.0;
      super.MUSCLE_DAMPING = 0.005;
      super.MUSCLE_PASSIVE_FRACTION = 0.0;
      //super.MUSCLE_TENDON_RATIO = 0.03;
      super.SPRING_MUSCLE_MAX_FORCE = 10;
      super.SPRING_MUSCLE_FORCE_SCALING = 1000;
      super.SPRING_MUSCLE_PASSIVE_FRACTION = 0.012;
      super.SPRING_MUSCLE_TENDON_RATIO = 0.5;
      super.SPRING_MUSCLE_DAMPING = 0.05;
      super.SPRING_DAMPING = 0.5;
      super.SPRING_STIFFNESS = 50.0;
      // Book: "Skeletal muscle mechanics: from mechanisms to function", By Walter Herzog
      // 	Section 12: FEM-Simulation of Skeletal Muscle:
      //		The Influence of Inertia During Activation and Deactivation
      //		By P. MEIER AND R. BLICKHAN
      // "Bulk Modulus". In: Encyclopedia of Biomedical Engineering
      //super.FEM_MATERIAL = new MooneyRivlinMaterial(10.000,0,0,10.000,0,48.3);
      // Badin tongue
      super.FEM_MATERIAL = new MooneyRivlinMaterial(1.037,0,0,0.486,0,10.370);
      // Badin face
      //super.FEM_MATERIAL = new MooneyRivlinMaterial(2.50,0,0,1.175,0,25.00);
      //super.FEM_MATERIAL = new LinearMaterial(25,0.49);
      
      super.MUSCLE_MATERIAL = new BlemkerMuscle();
      	((BlemkerMuscle)super.MUSCLE_MATERIAL).setMaxStress (MUSCLE_MAXSTRESS);
      	((BlemkerMuscle)super.MUSCLE_MATERIAL).setMaxLambda(MUSCLE_MAXLAMBDA);
      	((BlemkerMuscle)super.MUSCLE_MATERIAL).setExpStressCoeff(0.00005);
      /*
      super.MUSCLE_MATERIAL = new GenericMuscle();
    	((GenericMuscle)super.MUSCLE_MATERIAL).setMaxStress (MUSCLE_MAXSTRESS);
    	((GenericMuscle)super.MUSCLE_MATERIAL).setMaxLambda(MUSCLE_MAXLAMBDA);
    	((GenericMuscle)super.MUSCLE_MATERIAL).setExpStressCoeff(0.00005);
      */
      
      super.MUSCLE_FIBRE_TYPE = "Peck";
      super.COLLISION_FRICTION_COEFF = 0.0;
      	
      super.OverallTrans.set(new RigidTransform3d (0, 0, 0, 1, 0, 0, Math.toRadians(180)));
      super.OverallScaling = 1; // in meters
      super.pointRadius = 0.5;
      super.lineWidth = 2;
      super.lineRadius = 0.5;
      super.elementWedgeSize = 0.0;
      super.includeWayPoints = true;
      super.wayPointStep = 0.05;
      super.stopPoint = 3.1d;
      
      
      createModel();
      
      
      /*
      for(int i=0;i<myMechMod.models().size();i++) {
	 String femName = myMechMod.models().get(i).getName();
	 if(femName.endsWith("tendon")) {
	    subdivideFem((FemModel3d) myMechMod.models().get(femName),3);
	 }
      }
      */
      
      /*
      // subdivide
      String[] femToDivide = {
	    //"Sternohyoid_L", "Sternohyoid_R",
	    //"Digastric_L_anterior_belly", "Digastric_R_anterior_belly",
	    "Mylohyoid_L", "Mylohyoid_R"
	    };
      for(String femName: femToDivide) {
	 subdivideFem((FemModel3d) myMechMod.models().get(femName),1);
	 subdivideFem((FemModel3d) myMechMod.models().get(femName),2);
      }
      */
     
      Particle ref;
      String[] targetMarkers = {
	    "Hyoid_anterior",
//	    "Hyoid_posterior",
//	    "SubglotticAirColumn_superior_posterior",
//	    "Cricoid_superior_anterior",
//	    "Hyoid_posterior_L",
//	    "Hyoid_posterior_R",
      };
      for(String markerName: targetMarkers) {
	 ref = new Particle();
	 ref.setName(markerName+"_target");
	 ref.setDynamic(false);
	 ref.setState(myMechMod.frameMarkers().get(markerName));
	 myMechMod.addParticle(ref);
	 setFrameMarkerTracing(markerName);
	 enableTracing(ref);
	 RenderProps.setPointStyle(ref, PointStyle.SPHERE);
	 RenderProps.setPointRadius(ref, pointRadius*4);
	 RenderProps.setLineStyle(ref, LineStyle.CYLINDER);
	 RenderProps.setLineRadius(ref, lineRadius*0.8);
	 RenderProps.setLineColor(ref, Color.BLUE);
	 RenderProps.setPointColor(ref, Color.BLUE);
	 //RenderProps.setAlpha(ref, 0.5);
	 
	 if(ref.getName().contains("Hyoid_anterior_target")) {
	    ref.setPosition(100, -148.84135, -234.38899);
	 }
      }
      
      
      /*
      FemModel3d adipose = new FemModel3d("Adipose1");
      FemFactory.createHexGrid(adipose, 15, 14, 17, 3, 3, 3);
      adipose.transformGeometry(new RigidTransform3d(100, -140, -237));
      myMechMod.addModel(adipose);
      */
      
      /*
      try {
	 String fempath =
	      ArtisynthPath.getHomeRelativePath (
	         "src/artisynth/core/femmodels/meshes/", ".");
	 FemModel3d adipose = TetGenReader.read (
            "fem0", 5000, fempath + "sphere2" + ".1.node", fempath
            + "sphere2" + ".1.ele", new Vector3d(10,7.5,9));
	 adipose.transformGeometry(new RigidTransform3d(100, 140, 237));
	 myMechMod.addModel(adipose);
      } catch (Exception e) {
	 System.out.println("Warning: Cannot open file sphere2.1.node|ele");
      }
      */
      
      /*
      //##################################################################
      //###### Mylohyoid raph'e ##########################################
      String[] mylohyoidNames = {"Mylohyoid_L", "Mylohyoid_R"};
      double mylohyoid_damping = super.SPRING_DAMPING;
      double mylohyoid_stiffness = super.SPRING_STIFFNESS;
      for(String femName: mylohyoidNames) {
	 if(myMechMod.models().get(femName)!=null) {
	    FemModel3d fem = (FemModel3d) myMechMod.models().get(femName);
	    int count = 0;
	    for(int i=0;i<4;i++) {
	       for(int j=0;j<4;j++) {
		  AxialSpring spring = new AxialSpring(femName+"_raphe_"+count);
		  spring.setFirstPoint(fem.getNode(200+4*i+j));
		  spring.setSecondPoint(fem.getNode(200+4*i+j+4));
		  spring.setRestLength(spring.getLength());
		  spring.setDamping(mylohyoid_damping);
		  spring.setStiffness(mylohyoid_stiffness);
		  RenderProps.setLineStyle(spring, LineStyle.CYLINDER);
		  RenderProps.setLineRadius(spring, lineRadius);
		  RenderProps.setLineColor (spring, Color.WHITE);
		  myMechMod.addAxialSpring(spring);
	       }
	    }
	 }
      }
      */
      
      //##################################################################
      //###### Omohyoid_L_tendon attachment ##############################
      int[] Omohyoid_tendon_nodes = {
	       0,2,4,10,16,22,20,18,12,6,
	       1,3,5,11,17,23,21,19,13,7
	 };
      addFemSprings("Omohyoid_L_tendon", Omohyoid_tendon_nodes, 24, 55, "Clavicle_L", "Omohyoid_L_tendon", 200, 0.5);
      addFemSprings("Omohyoid_R_tendon", Omohyoid_tendon_nodes, 24, 55, "Clavicle_R", "Omohyoid_R_tendon", 200, 0.5);
      
      //###################################################################
      //###### Digastric_L_tendon attachment ##############################
      int[] Digastric_tendon_nodes = {
	       5,9,13,17,35,53,71,67,63,59,41,23,
	       4,8,12,16,34,52,70,66,62,58,40,22,
	       3,9,11,15,33,51,69,65,61,57,39,21
	 };
      addFemSprings("Digastric_L_tendon", Digastric_tendon_nodes, 72, 119, "hyoid", "Digastric_L_tendon", 200, 0.5);
      addFemSprings("Digastric_R_tendon", Digastric_tendon_nodes, 72, 119, "hyoid", "Digastric_R_tendon", 200, 0.5);

      //###################################################################
      //###### Add cricothyroid joint #####################################
      addCricothyroidJoint();
      //###################################################################
      
      //###################################################################
      //###### Add thyroarytenoid joint ###################################
      addThyroarytenoidJoint();
      //###################################################################
      
      //###################################################################
      //###### Add trachea connection #####################################
      addTrachea(0.00000114,50,0.5); // cartilage density [Harper2001]
      //addTracheaAttachment(1, 0.005);
      //###################################################################
      
      //###################################################################
      //###### Add Monitor ################################################
      //MyMonitor MechMonitor = new MyMonitor();
      //addMonitor(MechMonitor);
      //MechMonitor.setModel(super.myMechMod);
      //###################################################################
      
      /*
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Thyrohyoid_membrane_L"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Thyrohyoid_membrane_R"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Cricothyroid_ligament_median_L"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Cricothyroid_ligament_median_R"), Renderer.FaceStyle.FRONT);
      
      
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Digastric_L_tendon"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Digastric_R_tendon"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Omohyoid_L_tendon"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Omohyoid_R_tendon"), Renderer.FaceStyle.FRONT);
      */
      
      //myMechMod.scaleDistance(1/1000d);
   }
   
   public void attach(DriverInterface driver)
   {
      super.attach(driver);
      //setSagittalView(1.5);
      setSagittalView(3.0);
   }
   
   //###################################################################
   //###### Add cricothyroid joint #####################################
   private void addCricothyroidJoint() {
      RigidBody thyroid = myMechMod.rigidBodies().get("thyroid");
      RigidBody cricoid = myMechMod.rigidBodies().get("cricoid");
      if (thyroid != null && cricoid != null) {
	 RevoluteJoint ctJoint = new RevoluteJoint();
	 ctJoint.setName("cricothyroid");

	 Point3d pmin = new Point3d();
	 Point3d pmax = new Point3d();
	 thyroid.updateBounds(pmin, pmax);
	 ctJoint.setAxisLength(0.4 * (pmax.x - pmin.x));
	 
         if (ctJoint != null) {
            //RenderProps.setLineSlices(ctJoint, 12);
            RenderProps.setLineColor(ctJoint, Color.ORANGE);
            RenderProps.setLineRadius(ctJoint, 0.6);
         }

	 // connector transforms for both bodies are the same
	 // as they are coincident at simulation start
	 RigidTransform3d TCA = new RigidTransform3d();
	 TCA.p.set(new Point3d(100, 119.5, 268));
	 TCA.R.setAxisAngle(0, 1, 0, Math.PI / 2);
	 TCA.mul(OverallTrans);

	 ctJoint.setBodies(cricoid, TCA, thyroid, TCA);
	 myMechMod.addBodyConnector(ctJoint);
      }
   }
   //###################################################################
   
   //###################################################################
   //###### Add thyroarytenoid joint ###################################   
   private void addThyroarytenoidJoint() {
      RigidBody thyroid = myMechMod.rigidBodies().get("thyroid");
      RigidBody Arytenoid_L = myMechMod.rigidBodies().get("Arytenoid_L");
      RigidBody Arytenoid_R = myMechMod.rigidBodies().get("Arytenoid_R");
      if (thyroid != null && Arytenoid_L != null) {
	 RevoluteJoint taJoint = new RevoluteJoint();
	 taJoint.setName("thyroarytenoid_L");

	 Point3d pmin = new Point3d();
	 Point3d pmax = new Point3d();
	 thyroid.updateBounds(pmin, pmax);
	 taJoint.setAxisLength(0.08 * (pmax.x - pmin.x));
	 
	 RenderProps pp = taJoint.getRenderProps();
	      if (taJoint != null) {
		 //pp.setLineSlices(12);
		 pp.setLineColor(Color.ORANGE);
		 pp.setLineRadius(0.6);
	      }

	 // connector transforms for both bodies are the same
	 // as they are coincident at simulation start
	 RigidTransform3d TCA = new RigidTransform3d();
	 TCA.p.set(new Point3d(92, -121, -261));
	 TCA.p.transform(OverallTrans);
	 TCA.R.setAxisAngle(0, 1, 0, Math.PI / 2);

	 taJoint.setBodies(Arytenoid_L, TCA, thyroid, TCA);
	 myMechMod.addBodyConnector(taJoint);
      }
      if (thyroid != null && Arytenoid_R != null) {
	 RevoluteJoint taJoint = new RevoluteJoint();
	 taJoint.setName("thyroarytenoid_R");

	 Point3d pmin = new Point3d();
	 Point3d pmax = new Point3d();
	 thyroid.updateBounds(pmin, pmax);
	 taJoint.setAxisLength(0.08 * (pmax.x - pmin.x));
	 
	 RenderProps pp = taJoint.getRenderProps();
	      if (taJoint != null) {
		 //pp.setLineSlices(12);
		 pp.setLineColor(Color.ORANGE);
		 pp.setLineRadius(0.6);
	      }

	 // connector transforms for both bodies are the same
	 // as they are coincident at simulation start
	 RigidTransform3d TCA = new RigidTransform3d();
	 TCA.p.set(new Point3d(108, -121, -261));
	 TCA.p.transform(OverallTrans);
	 TCA.R.setAxisAngle(0, 1, 0, Math.PI / 2);

	 taJoint.setBodies(Arytenoid_R, TCA, thyroid, TCA);
	 myMechMod.addBodyConnector(taJoint);
      }
   }
   //###################################################################
   
   //###################################################################
   //###### Trachea ####################################################
   private void addTrachea(double density, double stiffness, double damping) {
      String fileName = "ring.obj"; // Dimensions from [Eckel1994]
      String bodyName = "TrachealRing";
      double scale = 1;
      int numRing = 10;
      int n_in = 8;
      String shortMeshName = rigidBodyPath + fileName;
      // String fullMeshFileName = shortMeshName + fileName;
      
      try {
	      RigidBody refBody = myMechMod.rigidBodies().get("ref_block");
	      RigidBody cricoid = myMechMod.rigidBodies().get("cricoid");
	      if(cricoid==null || refBody==null) {
		 return;
	      }
	      FemModel3d myFemMod = HybridFemFactory.createHexCylinder(3,10.5,12,3,n_in,1);
	      RigidTransform3d transform = new RigidTransform3d(100,128.7,288,0,1,0,Math.PI*3/2);
	      myFemMod.transformGeometry(transform);
	      
	      int[] nodes;
	      int[] nodesTemp = new int [0];
	      int[] nodesAll = new int [0];
	      for(int i=0;i<2;i++) {
		 for(int j=2;j<1+((int)n_in/2)+n_in%2;j++) {
		    nodes = HybridFemGenerator.roundedBeamCircleNodes(j,i,3,n_in,1);
		    nodesAll = new int[nodes.length+nodesTemp.length];
		    System.arraycopy(nodes, 0, nodesAll, 0, nodes.length);
		    System.arraycopy(nodesTemp, 0, nodesAll, nodes.length, nodesTemp.length);
		    nodesTemp = nodesAll;
		 }
	      }
	      myFemMod = HybridFemGenerator.reudceHexToWedgeAndTet(myFemMod,nodesAll);
	      // Project to cricoid
	      //TriangleIntersector isect = new TriangleIntersector();
	      BVFeatureQuery query = new BVFeatureQuery();
	      PolygonalMesh mesh = cricoid.getMesh().copy();
	      RigidBody temp = new RigidBody();
	      temp.setMesh(mesh, null);
	      
	      // Define particles for Cricotracheal_ligament_at_cricoid
	      ArrayList<FrameMarker> Cricotracheal_ligament_at_cricoid1 = new ArrayList<FrameMarker>();
	      ArrayList<FrameMarker> Cricotracheal_ligament_at_cricoid2 = new ArrayList<FrameMarker>();
	      Face face;
	      Vector3d duv = new Vector3d();
	      Vector3d rayDir = new Vector3d(0,0,-1);
	      Point3d p_orig, p_proj;
	      FrameMarker marker;
	      int count = 0;
	      for(int j=0;j<2;j++) {
		 nodes = HybridFemGenerator.roundedBeamCircleNodes(j,0,3,n_in,1);
		 for(int k=0;k<nodes.length;k++) {
		    p_orig = myFemMod.getNode(nodes[k]).getPosition();
		    face = query.nearestFaceAlongRay (
		       null, duv, mesh, p_orig, rayDir);
		    //face = obbt.intersect(p_orig, rayDir, duv, isect);
		    if(face!=null) {
		       p_proj = projToSurface(p_orig, rayDir, face);
		       marker = new FrameMarker();
		       marker.setName("Cricotracheal_ligament_at_cricoid_" + count);
		       marker.setFrame(cricoid);
		       marker.setLocation(p_proj);
		       if(drawAttachedNodes) {
		  		RenderProps.setPointStyle (marker, Renderer.PointStyle.SPHERE);
		  		RenderProps.setPointRadius (marker, pointRadius*1.5);
		  		RenderProps.setPointColor (marker, Color.BLUE);
		       }
		       myMechMod.addFrameMarker(marker);
		       if(j==0) {
			  Cricotracheal_ligament_at_cricoid1.add(marker);
		       } else {
			  Cricotracheal_ligament_at_cricoid2.add(marker);
		       }
		       count++;
		    }
		 }
	      }
	 
	 mesh = new PolygonalMesh ();
	 mesh.read(new InputStreamReader(getInputStream(rigidBodyPath, fileName, rigidBodyDest)));
	 closeStreams();
	 mesh.scale (scale);

	 ComponentList<RigidBody> trachealRings = 
	    new ComponentList<RigidBody> (RigidBody.class, "rigidBodies", "r");
	 ArrayList<Vertex3d> vertices = mesh.getVertices();
	 ArrayList<FrameMarker> tracheal_top_1;
	 ArrayList<FrameMarker> tracheal_top_2;
	 ArrayList<FrameMarker> tracheal_down_1 = new ArrayList<FrameMarker>();
	 ArrayList<FrameMarker> tracheal_down_2 = new ArrayList<FrameMarker>();
	 ArrayList<FrameMarker> tracheal_down_bottom_1 = Cricotracheal_ligament_at_cricoid1;
	 ArrayList<FrameMarker> tracheal_down_bottom_2 = Cricotracheal_ligament_at_cricoid2;
	 RigidBody body_top;
	 RigidBody body_down = myMechMod.rigidBodies().get("cricoid");
	 
	 for(int i=0;i<numRing;i++) {
	    tracheal_top_1 = tracheal_down_bottom_1;
	    tracheal_top_2 = tracheal_down_bottom_2;
	    body_top = body_down;
	    tracheal_down_1 = new ArrayList<FrameMarker>();
	    tracheal_down_2 = new ArrayList<FrameMarker>();
	    tracheal_down_bottom_1 = new ArrayList<FrameMarker>();
	    tracheal_down_bottom_2= new ArrayList<FrameMarker>();
	    
	    transform = new RigidTransform3d(100,128.7,284+i*6,1,0,0,Math.toRadians(0));
	    
	    body_down = new RigidBody (bodyName+"_"+i);
	    body_down.setMesh (mesh.copy(), null);
	    body_down.transformGeometry(transform);
	    body_down.transformGeometry(OverallTrans);
	    trachealRings.add(body_down);
	    myMechMod.addRigidBody (body_down);
	    
	    // Set properties
	    body_down.setDensity(density);
	    if(i==numRing-1) {
	       body_down.setDynamic(false);
	    }
	    // Set rendering
	    RenderProps.setFaceColor(body_down, new Color(238, 221, 130));
	    RenderProps.setShading (body_down, Renderer.Shading.FLAT);
	    RenderProps.setFaceStyle (body_down, Renderer.FaceStyle.FRONT_AND_BACK);
	    
	    
	    count = 0;
	    for(Vertex3d v: vertices) {
	       //if(v.getPosition().z>0) {
		  marker = new FrameMarker();
		  marker.setName("tracheal_ligament_"+ i + "_" + count); count++;
		  marker.setFrame(body_down);
		  marker.setLocation(v.getWorldPoint());
		  myMechMod.addFrameMarker(marker);
		  if(drawAttachedNodes) {
	  		RenderProps.setPointStyle (marker, Renderer.PointStyle.SPHERE);
	  		RenderProps.setPointRadius (marker, pointRadius*1.5);
	  		RenderProps.setPointColor (marker, Color.BLUE);
		  }
		  
		  if(v.getPosition().z<0) {
		  // Inner markers
		  if(v.getPosition().distance(new Vector3d(0,0,0))>10) {
		     tracheal_down_1.add(marker);
		  // Outer markers
		  } else {
		     tracheal_down_2.add(marker);
		  }
		  
		  // Bottom 
		  } else {
		  // Inner markers
		  if(v.getPosition().distance(new Vector3d(0,0,0))>10) {
		     tracheal_down_bottom_1.add(marker);
		     // Outer markers
		  } else {
		     tracheal_down_bottom_2.add(marker);
		  }
		  }
	    }
	    
	    // Create springs
	    //if(i>0) {
	    addFrameMarkerSprings("tracheal_ligament_" + i + "_inner",
		  body_top, tracheal_top_1,
		  body_down, tracheal_down_2,1,
		  stiffness, damping);
	    addFrameMarkerSprings("tracheal_ligament_" + i + "_outer",
		  body_top, tracheal_top_2,
		  body_down, tracheal_down_1,1,
		  stiffness, damping);
	   // }
	 }
	 
	 // Shape
	 int i=0;
	 for(RigidBody body:trachealRings) {
	    if(i>1) {
	       transform = new RigidTransform3d(0,(i-1)*3,0);
	       body.transformGeometry(transform);
	       //AxisAngle ori = new AxisAngle (OverallTrans.R.getAxisAngle());
	       //body.setOrientation()
	    }
	    i++;
	 }
	    
      }
      catch (IOException e) {
	 e.printStackTrace ();
      }
      
      
   }
   
   //   private void addTracheaAttachment(double stiffness, double damping) {
   //      RigidBody refBody = myMechMod.rigidBodies().get("ref_block");
   //      RigidBody cricoid = myMechMod.rigidBodies().get("cricoid");
   //      if(cricoid==null || refBody==null) {
   //         return;
   //      }
   //      FemModel3d myFemMod = HybridFemFactory.createHexCylinder(3,3,12,3,4,8);
   //      RigidTransform3d transform = new RigidTransform3d(100,128.7,288,0,1,0,Math.PI*3/2);
   //      myFemMod.transformGeometry(transform);
   //      int[] nodes;
   //      int[] nodesTemp = new int [0];
   //      int[] nodesAll = new int [0];
   //      for(int i=0;i<=3;i++) {
   //         for(int j=2;j<=(4/2+8);j++) {
   //            nodes = HybridFemGenerator.roundedBeamCircleNodes(j,i,3,4,8);
   //            nodesAll = new int[nodes.length+nodesTemp.length];
   //            System.arraycopy(nodes, 0, nodesAll, 0, nodes.length);
   //            System.arraycopy(nodesTemp, 0, nodesAll, nodes.length, nodesTemp.length);
   //            nodesTemp = nodesAll;
   //         }
   //      }
   //      myFemMod = HybridFemGenerator.reudceHexToWedgeAndTet(myFemMod,nodesAll);
   //      // Project to cricoid
   //      Intersector isect = new Intersector();
   //      PolygonalMesh mesh = cricoid.getMesh().copy();
   //      RigidBody temp = new RigidBody();
   //      temp.setMesh(mesh, null);
   //      OBBTree obbt = mesh.getObbtree();
   //
   //      // Define particles for Cricotracheal_ligament_at_cricoid
   //      ArrayList<FrameMarker> Cricotracheal_ligament_at_cricoid1 = new ArrayList<FrameMarker>();
   //      ArrayList<FrameMarker> Cricotracheal_ligament_at_cricoid2 = new ArrayList<FrameMarker>();
   //      Face face;
   //      Vector3d duv = new Vector3d();
   //      Vector3d rayDir = new Vector3d(0,0,-1);
   //      Point3d p_orig, p_proj;
   //      FrameMarker marker;
   //      int count = 0;
   //      for(int j=0;j<2;j++) {
   //         nodes = HybridFemGenerator.roundedBeamCircleNodes(j,0,3,4,8);
   //         for(int k=0;k<nodes.length;k++) {
   //            p_orig = myFemMod.getNode(nodes[k]).getPosition();
   //            face = obbt.intersect(p_orig, rayDir, duv, isect);
   //            if(face!=null) {
   //               p_proj = projToSurface(p_orig, rayDir, face);
   //               marker = new FrameMarker();
   //               marker.setName("Cricotracheal_ligament_at_cricoid_" + count);
   //               marker.setFrame(cricoid);
   //               marker.setLocation(p_proj);
   //               if(drawAttachedNodes) {
   //                  RenderProps.setPointStyle (marker, Renderer.PointStyle.SPHERE);
   //                  RenderProps.setPointRadius (marker, pointRadius*1.5);
   //                  RenderProps.setPointColor (marker, Color.BLUE);
   //               }
   //               myMechMod.addFrameMarker(marker);
   //               if(j==0) {
   //                  Cricotracheal_ligament_at_cricoid1.add(marker);
   //               } else {
   //                  Cricotracheal_ligament_at_cricoid2.add(marker);
   //               }
   //               count++;
   //            }
   //         }
   //      }
   //      // Define particles for Cricotracheal_ligament_at_trachea
   //      ArrayList<FrameMarker> Cricotracheal_ligament_at_trachea1 = new ArrayList<FrameMarker>();
   //      ArrayList<FrameMarker> Cricotracheal_ligament_at_trachea2 = new ArrayList<FrameMarker>();
   //      count = 0;
   //      double scaleFactor;
   //      for(int j=0;j<2;j++) {
   //         nodes = HybridFemGenerator.roundedBeamCircleNodes(j,0,3,4,8);
   //         if(j==0)
   //            scaleFactor = 3;
   //         else
   //            scaleFactor = -3;
   //         for(int k=0;k<nodes.length;k++) {
   //            marker = new FrameMarker();
   //            marker.setName("Cricotracheal_ligament_at_trachea_" + count);
   //            marker.setFrame(refBody);
   //            Point3d nLoc = myFemMod.getNode(nodes[k]).getPosition();
   //            Vector3d toCentre = new Vector3d(nLoc);
   //            toCentre.sub(transform.p);
   //            toCentre.normalize();
   //            nLoc.scaledAdd(scaleFactor, toCentre);
   //            marker.setLocation(nLoc);
   //            if(drawAttachedNodes) {
   //               RenderProps.setPointStyle (marker, Renderer.PointStyle.SPHERE);
   //               RenderProps.setPointRadius (marker, pointRadius*1.5);
   //               RenderProps.setPointColor (marker, Color.GRAY);
   //            }
   //            myMechMod.addFrameMarker(marker);
   //            if(j==0) {
   //               Cricotracheal_ligament_at_trachea1.add(marker);
   //            } else {
   //               Cricotracheal_ligament_at_trachea2.add(marker);
   //            }
   //            count++;
   //         }
   //      }
   //
   //      addFrameMarkerSprings("Cricotracheal_ligament_1_",
   //         cricoid, Cricotracheal_ligament_at_cricoid1,
   //         refBody, Cricotracheal_ligament_at_trachea2,1,
   //         stiffness, damping);
   //      addFrameMarkerSprings("Cricotracheal_ligament_2_",
   //         cricoid, Cricotracheal_ligament_at_cricoid2,
   //         refBody, Cricotracheal_ligament_at_trachea1,1,
   //         stiffness, damping);
   //      /*
   //      myFemMod.transformGeometry(OverallTrans);
   //      myFemMod = HybridFemFactory.removeUnwantedNodes(myFemMod);
   //      myMechMod.addModel(myFemMod);
   //       */
   //   }
   //###################################################################
   
//   private void addFemSprings(int[] nodes1, int[] nodes2, String femName, double stiffness, double damping) {
//      FemMuscleModel femMuscle = (FemMuscleModel)myMechMod.models().get(femName);
//      
//      AxialSpring spring;
//      for(int i=0; i<nodes1.length; i++) {
//	 for(int j=0; j<nodes2.length; j++) {
//	    spring = new AxialSpring();
//	    spring.setFirstPoint(femMuscle.getNode(nodes1[i]));
//	    spring.setSecondPoint(femMuscle.getNode(nodes2[j]));
//	    spring.setRestLength(spring.getLength());
//	    spring.setStiffness(stiffness);
//	    spring.setDamping(damping);
//	    RenderProps.setLineStyle(spring, LineStyle.LINE);
//	    RenderProps.setLineWidth(spring, 3);
//	    RenderProps.setLineColor (spring, Color.CYAN);
//	    myMechMod.addAxialSpring(spring);
//	 }
//      }
//   }
   //########################################################################
   //#### Subfunctions: Monitor
   //########################################################################   
   public class MyMonitor extends MonitorBase {
	      public void apply (double t0, double t1) {
		 double step = 0.5;
		 double ex = TimeBase.modulo(t0,step)/step;
		 FemMuscleModel muscleModel1 = (FemMuscleModel) myMechMod.models().get("Geniohyoid_L");
		 FemMuscleModel muscleModel2 = (FemMuscleModel) myMechMod.models().get("Geniohyoid_R");
		 RenderableComponentList<MuscleBundle> ml1 = muscleModel1.getMuscleBundles();
		 RenderableComponentList<MuscleBundle> ml2 = muscleModel2.getMuscleBundles();
		 if (TimeBase.modulo(t0,4*step)>=step) {
		    //ex = 1-ex;
		    ex = 0;
		 }
		 ml1.get(0).setExcitation(ex);
		 ml2.get(0).setExcitation(ex);
		 System.out.println("step = " + t0 + ", excitation = " + ex);
	      }
	   }
   //########################################################################
   
   private Point3d projToSurface (Point3d origP, Vector3d origV, Face face_ray) {
      Point3d proj = new Point3d();
      Point3d surP = new Point3d();
      Point3d surP_origP = new Point3d();
      Vector3d normal = face_ray.getNormal();
      
      face_ray.computeCentroid(surP);
      surP_origP.sub(surP, origP);
      proj.set(origV);
      proj.scale(normal.dot(surP_origP)/normal.dot(origV));
      proj.add(origP);
      
      return proj;
   }
   
   private void addFemSprings(
	 String name, int[] nodes, int startingNodeIdx, int endingNodeIdx, 
	 String bodyName, String femName, double stiffness, double damping) {
      FemMuscleModel femMuscle = (FemMuscleModel)myMechMod.models().get(femName);
      RigidBody body = myMechMod.rigidBodies().get(bodyName);
      if(femMuscle!=null) {
	 if(body!=null) {
	    for(int i=startingNodeIdx;i<=endingNodeIdx;i++) {
	       myMechMod.attachPoint (femMuscle.getNode(i), body);
	       if(drawAttachedNodes) {
  		RenderProps.setPointStyle (femMuscle.getNode(i), Renderer.PointStyle.SPHERE);
  		RenderProps.setPointRadius (femMuscle.getNode(i), pointRadius*1.5);
  		RenderProps.setPointColor (femMuscle.getNode(i), Color.BLUE);
  	     }
	    }
	 } else {
	    for(int i=startingNodeIdx;i<=endingNodeIdx;i++) {
	       femMuscle.getNode(i).setDynamic(false);
	       if(drawAttachedNodes) {
  		RenderProps.setPointStyle (femMuscle.getNode(i), Renderer.PointStyle.SPHERE);
  		RenderProps.setPointRadius (femMuscle.getNode(i), pointRadius*1.5);
  		RenderProps.setPointColor (femMuscle.getNode(i), Color.GRAY);
  	     }
	    }
	 }
	 
	 int count = 0;
	 for(int n: nodes) {
	    Point3d p = femMuscle.getNode(n).getPosition();
	    int [] closest_nodes = {0,0,0,0};
	    double [] closest_dis = {Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE};
	    for(int i=startingNodeIdx;i<=endingNodeIdx;i++) {
	       Point3d m = femMuscle.getNode(i).getPosition();
	       double dis = p.distance(m);
	       if(dis<=closest_dis[0]) {
		  closest_nodes[3] = closest_nodes[2];
		  closest_nodes[2] = closest_nodes[1];
		  closest_nodes[1] = closest_nodes[0];
		  closest_nodes[0] = i;
		  closest_dis[3] = closest_dis[2];
		  closest_dis[2] = closest_dis[1];
		  closest_dis[1] = closest_dis[0];
		  closest_dis[0] = dis;
	       } else if(dis<=closest_dis[1]) {
		  closest_nodes[3] = closest_nodes[2];
		  closest_nodes[2] = closest_nodes[1];
		  closest_nodes[1] = i;
		  closest_dis[3] = closest_dis[2];
		  closest_dis[2] = closest_dis[1];
		  closest_dis[1] = dis;
	       } else if(dis<=closest_dis[2]) {
		  closest_nodes[3] = closest_nodes[2];
		  closest_nodes[2] = i;
		  closest_dis[3] = closest_dis[2];
		  closest_dis[2] = dis;
	       } else if(dis<=closest_dis[3]) {
		  closest_nodes[3] = i;
		  closest_dis[3] = dis;
	       }
	    }
	 
	    for(int i=0;i<4;i++) {
	       AxialSpring spring = new AxialSpring(name+"_"+count);
	       spring.setFirstPoint(femMuscle.getNode(n));
	       spring.setSecondPoint(femMuscle.getNode(closest_nodes[i]));
	       spring.setLinearMaterial (stiffness, damping);
	       spring.setRestLength(spring.getLength());
	       RenderProps.setLineStyle(spring, LineStyle.CYLINDER);
	       RenderProps.setLineRadius(spring, lineRadius);
	       RenderProps.setLineColor (spring, Color.LIGHT_GRAY);
	       myMechMod.addAxialSpring(spring);
	       count++;
	    }
	 }
      }
   }
   private void addFrameMarkerSprings(
	 String name,
	 RigidBody body1, ArrayList<FrameMarker> markerList1, 
	 RigidBody body2, ArrayList<FrameMarker> markerList2,
	 int numStringPerPoint,
	 double stiffness, double damping) {

	 int count = 0;
	 for(FrameMarker marker1: markerList1) {
	    Point3d p = marker1.getPosition();
	    int [] closest_nodes = {0,0,0,0};
	    double [] closest_dis = {Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE};
	    for(int i=0;i<markerList2.size();i++) {
	       Point3d m = markerList2.get(i).getPosition();
	       double dis = p.distance(m);
	       if(dis<=closest_dis[0]) {
		  closest_nodes[3] = closest_nodes[2];
		  closest_nodes[2] = closest_nodes[1];
		  closest_nodes[1] = closest_nodes[0];
		  closest_nodes[0] = i;
		  closest_dis[3] = closest_dis[2];
		  closest_dis[2] = closest_dis[1];
		  closest_dis[1] = closest_dis[0];
		  closest_dis[0] = dis;
	       } else if(dis<=closest_dis[1]) {
		  closest_nodes[3] = closest_nodes[2];
		  closest_nodes[2] = closest_nodes[1];
		  closest_nodes[1] = i;
		  closest_dis[3] = closest_dis[2];
		  closest_dis[2] = closest_dis[1];
		  closest_dis[1] = dis;
	       } else if(dis<=closest_dis[2]) {
		  closest_nodes[3] = closest_nodes[2];
		  closest_nodes[2] = i;
		  closest_dis[3] = closest_dis[2];
		  closest_dis[2] = dis;
	       } else if(dis<=closest_dis[3]) {
		  closest_nodes[3] = i;
		  closest_dis[3] = dis;
	       }
	    }
	 
	    for(int i=0;i<numStringPerPoint;i++) {
	       AxialSpring spring = new AxialSpring(name+"_"+count);
	       spring.setFirstPoint(marker1);
	       spring.setSecondPoint(markerList2.get(closest_nodes[i]));
	       spring.setRestLength(spring.getLength());
	       spring.setLinearMaterial (stiffness, damping);
	       RenderProps.setLineStyle(spring, LineStyle.CYLINDER);
	       RenderProps.setLineRadius(spring, lineRadius);
	       RenderProps.setLineColor (spring, Color.LIGHT_GRAY);
	       myMechMod.addAxialSpring(spring);
	       count++;
	    }
	 }
   }
   public void setSagittalView(double gridOffset) {
      GLViewer v = Main.getMain().getViewer();
      
      //vc.autoFit();
      v.setAxialView(AxisAlignedRotation.Y_Z);
      v.setEye(new Point3d(800, -100, -195));
      
      if (v.getNumClipPlanes() < 1) {
	 v.addClipPlane();
      }
      GLClipPlane clip = v.getClipPlane (0);
      
      clip.setResolution(new GLGridResolution(100,10));
      clip.setPosition(getCenter());
      clip.setOrientation(new AxisAngle (0, 1, 0, Math.PI / 2));
      clip.setOffset (gridOffset);
      clip.setGridVisible (true);
      clip.setDragger (DraggerType.None);
   }
}
