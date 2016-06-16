package artisynth.models.larynx;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.IOException;

import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLGridResolution;
import maspack.render.GL.GLViewer;
import maspack.render.Dragger3d.DraggerType;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import artisynth.core.driver.Main;
import maspack.matrix.AxisAlignedRotation;
import artisynth.core.driver.ViewerManager;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.TimeBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.template.ModelTemplate;

public class VHLarynxDemo_intrinsic extends ModelTemplate {
   public VHLarynxDemo_intrinsic() {
   }
   
   public VHLarynxDemo_intrinsic(String name) throws IOException{
      super(name);
      
      boolean INCLUDE_FEMSPRING = false;
      
      //#####################################################################
      // CONTROLS
      super.debug = false;
      
      super.disableNodes = false;
      super.IncompressOption = false;
      super.SetCollision = true;
      super.muscleControlPanel = true;
      super.SingleMusclePanels = false;
      super.drawNodes = false;
      super.drawAttachedNodes = false;
      super.drawBundle = true;
      super.muscleShaded = true;
      super.useElementsInsteadOfFibres = false;
      
      super.downloadFiles = false;  // prevent extracting geometry
      String basePath = ArtisynthPath.getSrcRelativePath(this, "");
      super.rigidBodyPath = "zip:file://" + basePath + "geometry/rigidBodies/meshes.zip!";
      super.rigidBodyDest = basePath + "geometry/rigidBodies/";
      super.femPath = basePath + "geometry/fem/";
      super.otherPath = basePath + "geometry/other/";
      
      super.bodyListAllFilename = "bodyList_intrinsic.txt";
      super.bodyPropertyListFilename = "bodyPropertyList.txt";
      super.bodyTransformListAllFilename = "";
      super.femListAllFilename = "femList_intrinsic.txt";
      super.femPropertyListFilename = "femMusclePropertyList.txt";
      super.femTransformListAllFilename = "";
      super.femAttachParticleFilename = "femAttachAll.txt";
      super.frameMarkerListFilename = "frameMarker.txt";
      //super.muscleSpringListFilename = "frameMuscle.fibre";
      super.muscleSpringListFilename = "frameMuscle_stylo.fibre";
      super.muscleSpringPropertyListFilename = "springMuscleProperty.txt";
      super.springListFilename = "spring.fibre";
      super.springPropertyListFilename = ""; //"springProperty.txt";
      super.collisionListFilename = "collision.txt";
      super.workingDirname = "src/artisynth/models/larynx/data";
      super.probesPath = this.workingDirname;
      super.probesFilename = "probes_hyoid3.art";
      super.aboutFilename = "src/artisynth/models/larynx/about_VHLarynxDemo.txt";
      //#####################################################################

      // Step size
      super.MAX_STEP_SIZE_SEC = 10.0e-3; // 1 msec
      // all units in mm
      super.addMidElementsWithin = 5;
      super.elementDirectionRenderLen = 0.5;
      //super.GRAVITY = 9800;
      super.MUSCLE_DENSITY = 0.000001040;
      super.MUSCLE_PARTICLE_DAMPING = 40;
      super.MUSCLE_STIFFNESS_DAMPING = 0.03;
      super.MUSCLE_YOUNGS_MODULUS = 500;
      super.MUSCLE_MAXLAMBDA = 2.1;
      super.MUSCLE_MAXSTRESS = 4;
      super.MUSCLE_MAX_FORCE = 20;
      super.MUSCLE_FORCE_SCALING = 1;
      super.SPRING_MUSCLE_MAX_FORCE = 200;
      super.SPRING_MUSCLE_FORCE_SCALING = 1;
      super.SPRING_DAMPING = 0.5;
      super.SPRING_STIFFNESS = 50.0;
      super.FEM_MATERIAL = new MooneyRivlinMaterial(1.037,0,0,0.486, 0,10.370);
      super.MUSCLE_MATERIAL = new GenericMuscle();
      	((GenericMuscle)super.MUSCLE_MATERIAL).setMaxStress (MUSCLE_MAXSTRESS);
      	((GenericMuscle)super.MUSCLE_MATERIAL).setMaxLambda(MUSCLE_MAXLAMBDA);
      	((GenericMuscle)super.MUSCLE_MATERIAL).setExpStressCoeff(0.00005);
      	
      super.COLLISION_FRICTION_COEFF = 0.0;
      	
      super.OverallTrans.set(new RigidTransform3d (0, 0, 0, 1, 0, 0, Math.toRadians(180)));
      super.OverallScaling = 1; // in meters
      super.pointRadius = 0.5;
      super.lineWidth = 1;
      super.lineRadius = 0.5;
      super.elementWedgeSize = 1.0;
      super.includeWayPoints = true;
      super.wayPointStep = 0.1;
      super.stopPoint = 5.5d;
      
      createModel();
      
      Particle p = new Particle(1,100,-150.5,-229);
      p.setName("Hyoid_anterior");
      RenderProps.setPointStyle (p, Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius (p, pointRadius*1.5);
      RenderProps.setPointColor (p, Color.CYAN);
      myMechMod.addParticle(p);
      //###################################################################
      
      /*
      FemModel3d adipose = new FemModel3d("Adipose1");
      FemFactory.createHexGrid(adipose, 30, 13.5, 18, 8, 6, 7);
      adipose.transformGeometry(new RigidTransform3d(100, -140, -237));
      myMechMod.addModel(adipose);
      */
      
      //###################################################################
      //###### Add cricothyroid joint #####################################
      RigidBody thyroid = myMechMod.rigidBodies().get("thyroid");
      RigidBody cricoid = myMechMod.rigidBodies().get("cricoid");
      if (thyroid != null && cricoid != null) {
	 RevoluteJoint ctJoint = new RevoluteJoint();
	 ctJoint.setName("cricothyroid");

	 Point3d pmin = new Point3d();
	 Point3d pmax = new Point3d();
	 thyroid.updateBounds(pmin, pmax);
	 ctJoint.setAxisLength(0.4 * (pmax.x - pmin.x));
	 
	 RenderProps pp = ctJoint.getRenderProps();
	      if (ctJoint != null) {
		 //pp.setLineSlices(12);
		 pp.setLineColor(Color.ORANGE);
		 pp.setLineRadius(0.6);
	      }

	 // connector transforms for both bodies are the same
	 // as they are coincident at simulation start
	 RigidTransform3d TCA = new RigidTransform3d();
	 TCA.p.set(new Point3d(100, -119.5, -268));
	 TCA.p.transform(OverallTrans);
	 TCA.R.setAxisAngle(0, 1, 0, Math.PI / 2);

	 ctJoint.setBodies(cricoid, TCA, thyroid, TCA);
	 myMechMod.addBodyConnector(ctJoint);
      }
      //###################################################################
      //###### Add thyroarytenoid joint ###################################
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
      //###################################################################
      
      
      //###################################################################
      //###### Add Monitor ################################################
      //MyMonitor MechMonitor = new MyMonitor();
      //addMonitor(MechMonitor);
      //MechMonitor.setModel(super.myMechMod);
      //###################################################################
      
      
      if(super.myMechMod.rigidBodies ().get ("bone")!=null)
         super.myMechMod.rigidBodies ().get ("bone").setDynamic (false);
      
      /*
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Thyrohyoid_membrane_L"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Thyrohyoid_membrane_R"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Cricothyroid_ligament_L"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Cricothyroid_ligament_R"), Renderer.FaceStyle.FRONT);
      
      
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Digastric_L_tendon"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Digastric_R_tendon"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Omohyoid_L_tendon"), Renderer.FaceStyle.FRONT);
      RenderProps.setFaceStyle ((FemMuscleModel)super.myMechMod.models().get("Omohyoid_R_tendon"), Renderer.FaceStyle.FRONT);
      */
      
      if(super.myMechMod.rigidBodies ().get ("Skin")!=null) {
	 RenderProps.setVisible(super.myMechMod.rigidBodies ().get ("Skin"), false);
	 super.myMechMod.rigidBodies ().get ("Skin").setDynamic(false);
      }

      /*
      GLViewer v = Main.getMain().getViewer();
      v.myUnClippedRenderList.clear();
      for(RigidBody bi: super.myMechMod.rigidBodies()) {
	 bi.setDynamic(false);
	 if(!bi.getName().matches("Skin")){// && !bi.getName().matches("Skull") &&
	       //!bi.getName().matches("Maxilla") && !bi.getName().matches("Mandible")) {
	    v.myUnClippedRenderList.addIfVisible(bi);
	    v.removeRenderable(bi);
	 }
      }
      for(MechSystemModel mu: super.myMechMod.models()) {
	 v.myUnClippedRenderList.addIfVisible((FemMuscleModel)mu);
	 v.removeRenderable((FemMuscleModel)mu);
      }
      
      for(MultiPointSpring sp: super.myMechMod.multiPointSprings()) {
	 v.myUnClippedRenderList.addIfVisible(sp);
	 v.removeRenderable(sp);
      }
      for(FrameMarker fm: super.myMechMod.frameMarkers()) {
	 v.myUnClippedRenderList.addIfVisible(fm);
	 v.removeRenderable(fm);
      }
	*/
      
      /*
      RigidBody bi = myMechMod.rigidBodies().get("Quadrangular_membrane");
      FemMuscleModel fem = new FemMuscleModel ("Quadrangular_membrane");
      FemFactory.createHexExtrusion(fem, 1, 1, bi.getMesh());
      myMechMod.addModel(fem);
      bi = myMechMod.rigidBodies().get("Cricothyroid_ligament");
      fem = new FemMuscleModel ("Cricothyroid_ligament");
      FemFactory.createHexExtrusion(fem, 1, 1, bi.getMesh());
      myMechMod.addModel(fem);
      */
      
      /*
      // Reflect
      FemMuscleModel fem = (FemMuscleModel) myMechMod.models().get("Cricothyroid_ligament_R");
      for(FemNode3d node: fem.getNodes()) {
	 node.setPosition(200-node.getPosition().x,node.getPosition().y,node.getPosition().z);
      }
      */
      
      /*
      //####################################################
      //####################################################
      FemMuscleModel femMuscle = new FemMuscleModel ("Epiglottis");
      FemNode3d node;
      String filePath = ArtisynthPath.getSrcRelativePath (
	      VHLarynxDemo.class, "geometry/");
      double x,y,z;
      FrameMarker v;
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new FileReader(filePath + "v_top.txt"));
   
         System.out.println("FrameMarker!!!: ");
         
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            x = rtok.nval; rtok.nextToken();
            y = rtok.nval; rtok.nextToken();
            z = rtok.nval;
            node = new FemNode3d(x,y,z);
            femMuscle.addNode(node);
            
            RenderProps.setPointRadius (node, pointRadius);
            RenderProps.setPointStyle (node, Renderer.PointStyle.SPHERE);
            RenderProps.setPointColor (node, Color.BLUE);
            
            System.out.println("Node: " + x + ", " + y + ", " + z);
         }
         rtok.close();
      } catch (IOException e) {
	    System.out.println("Warning: File " + filePath + "v_top.txt" + " does not exist!");
      }
      
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new FileReader(filePath + "v_down.txt"));
   
         System.out.println("FrameMarker!!!: ");
         
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF)
         {
            x = rtok.nval; rtok.nextToken();
            y = rtok.nval; rtok.nextToken();
            z = rtok.nval;
            
            node = new FemNode3d(x,y,z);
            femMuscle.addNode(node);
            
            System.out.println("FrameMarker: " + x + ", " + y + ", " + z);
            
            RenderProps.setPointRadius (node, pointRadius);
            RenderProps.setPointStyle (node, Renderer.PointStyle.SPHERE);
            RenderProps.setPointColor (node, Color.GREEN);
         }
         
         rtok.close();
      } catch (IOException e) {
	    System.out.println("Warning: File " + filePath + "v_down.txt" + " does not exist!");
      }
      
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new FileReader(filePath + "epiglottis.elem"));
         HexElement el;
         int n, i=0;
         int[] nodes = new int[8];
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            n = (int)rtok.nval;
            nodes[i] = n;
            i++;
            if(i==8) {
               i=0;
               el = new HexElement(
                     femMuscle.getNode(nodes[0]),
                     femMuscle.getNode(nodes[1]),
                     femMuscle.getNode(nodes[2]),
                     femMuscle.getNode(nodes[3]),
                     femMuscle.getNode(nodes[4]),
                     femMuscle.getNode(nodes[5]),
                     femMuscle.getNode(nodes[6]),
                     femMuscle.getNode(nodes[7]));
               	System.out.println("Elem: " +
                  	nodes[0] + " " +
                  	nodes[1] + " " +
                  	nodes[2] + " " +
                  	nodes[3] + " " +
                  	nodes[4] + " " +
                  	nodes[5] + " " +
                  	nodes[6] + " " +
                  	nodes[7]);
               femMuscle.addElement(el);
            }
         }
         rtok.close();
      } catch (IOException e) {
	    System.out.println("Warning: File " + filePath + "v_down.txt" + " does not exist!");
      }
      
      myMechMod.addModel(femMuscle);
      */
      double stiffness = 350; 
      double damping = 0.1;
      int[] thyroepiglottic_nodes_epi = {2,3,1,0,338,339};
      int[] thyroepiglottic_nodes_thy = {340,341,342,343,344,345};      
      int[] hyoepiglottic_nodes_hyo = {346,347,348,349};
      int[] hyoepiglottic_nodes_epi = {
	    118,119,120,121,122,123,
	    113,112,111,110,109,108,
	    97,98,99,100,101,102,
	    90,89,88,87,86,85,
	    73,74,75,76,77,78,
	    //65,64,63,62,
	    //50,51,52,53
	    };
      
      if(INCLUDE_FEMSPRING) {
	 addFemSprings("Thyroepiglottic_ligament", thyroepiglottic_nodes_epi, thyroepiglottic_nodes_thy, "Epiglottis", stiffness, damping);
	 addFemSprings("Hyoepiglottic_ligament", hyoepiglottic_nodes_hyo, hyoepiglottic_nodes_epi, "Epiglottis", stiffness, damping);
      }
      
      /*
      //####################################################
      // Generate Hyoepiglottic_ligament fem
      FemMuscleModel Hyoepiglottic_ligament = new FemMuscleModel("Hyoepiglottic_ligament");
      FemMuscleModel epiglottis = (FemMuscleModel)myMechMod.models().get("Epiglottis");
      PointList<FemNode3d> epi_nodes = epiglottis.getNodes();
      Point3d centreP = new Point3d();
      for(int n:hyoepiglottic_nodes_hyo) {
	 centreP.add(epi_nodes.get(n).getPosition());
      }
      centreP.scale(1/(double)hyoepiglottic_nodes_hyo.length);
      centreP.x = 100;
      FemMuscleModel grid = new FemMuscleModel("grid");
      FemFactory.createHexGrid(grid, 5/2, 1, 4/2, 5, 1, 4);
      System.out.println("centreP: " + centreP.x + "," + centreP.y + "," + centreP.z);
      grid.transformGeometry(new RigidTransform3d(centreP.x,centreP.y+0.5,centreP.z,1,0,0,Math.toRadians(-35)));
      RenderProps.setPointRadius (grid, 0.2*pointRadius);
      RenderProps.setPointStyle (grid, Renderer.PointStyle.SPHERE);
      myMechMod.addModel(grid);
      PointList<FemNode3d> grid_nodes = grid.getNodes();

      // Nodes
      int incr = 5;
      int count = 0;
      Point3d origP,endP,midP = new Point3d();
      Vector3d step = new Vector3d();
      //System.out.println("Fem attachment:");
      //System.out.println("Epiglottis Hyoepiglottic_ligament");
      for(int i=0;i<5;i++) {
	 for(int j=0;j<6;j++) {
	    origP = epi_nodes.get(hyoepiglottic_nodes_epi[count]).getPosition();
	    endP = grid_nodes.get(i*6*2+j).getPosition();
	    step.sub(endP,origP);
	    step.scale(1/(double)incr);
	    //System.out.println(hyoepiglottic_nodes_epi[count] + " " + Hyoepiglottic_ligament.getNodes().size());
	    count++;
	    
	    Hyoepiglottic_ligament.addNode(new FemNode3d(origP));
	    for(int k=1;k<=incr;k++) {
	       midP.scaledAdd(k, step, origP);
	       Hyoepiglottic_ligament.addNode(new FemNode3d(midP));
	    }
	    System.out.println(Hyoepiglottic_ligament.getNodes().size()-1);
	 }
      }
      //System.out.println("end");
      
      // Elements
      PointList<FemNode3d> hyoepi_nodes = Hyoepiglottic_ligament.getNodes();
      for(int i=0;i<4;i++) {
	 for(int j=0;j<5;j++) {
	    for(int k=0;k<incr;k++) {
	       
	       HexElement el = new HexElement(
		     hyoepi_nodes.get(i*(incr+1)*6+j*(incr+1)+k),hyoepi_nodes.get(i*(incr+1)*6+j*(incr+1)+k+1),
			   hyoepi_nodes.get((i+1)*(incr+1)*6+j*(incr+1)+k+1),hyoepi_nodes.get((i+1)*(incr+1)*6+j*(incr+1)+k),
				 hyoepi_nodes.get(i*(incr+1)*6+(j+1)*(incr+1)+k),hyoepi_nodes.get(i*(incr+1)*6+(j+1)*(incr+1)+k+1),
				       hyoepi_nodes.get((i+1)*(incr+1)*6+(j+1)*(incr+1)+k+1),hyoepi_nodes.get((i+1)*(incr+1)*6+(j+1)*(incr+1)+k)
		     );
	       Hyoepiglottic_ligament.addElement(el);
	       
	       System.out.println("Element: " +
		     (i*(incr+1)*6+j*(incr+1)+k) + " " +
		     (i*(incr+1)*6+j*(incr+1)+k+1) + " " +
		     ((i+1)*(incr+1)*6+j*(incr+1)+k+1) + " " +
		     ((i+1)*(incr+1)*6+j*(incr+1)+k) + " " +
             	(i*(incr+1)*6+(j+1)*(incr+1)+k) + " " +
             	(i*(incr+1)*6+(j+1)*(incr+1)+k+1) + " " +
             	((i+1)*(incr+1)*6+(j+1)*(incr+1)+k+1) + " " +
             	((i+1)*(incr+1)*6+(j+1)*(incr+1)+k));
	    }
	 }
      }
      RenderProps.setPointRadius (Hyoepiglottic_ligament, 0.2*pointRadius);
      RenderProps.setPointStyle (Hyoepiglottic_ligament, Renderer.PointStyle.SPHERE);
      myMechMod.addModel(Hyoepiglottic_ligament);
      //####################################################
      */
      
      /*
      //####################################################
      // Generate Thyroepiglottic_ligament fem
      FemMuscleModel Thyroepiglottic_ligament = new FemMuscleModel("Thyroepiglottic_ligament");
      FemMuscleModel epiglottis = (FemMuscleModel)myMechMod.models().get("Epiglottis");
      PointList<FemNode3d> epi_nodes = epiglottis.getNodes();
      Point3d centreP = new Point3d();
      for(int n:thyroepiglottic_nodes_thy) {
	 centreP.add(epi_nodes.get(n).getPosition());
      }
      centreP.scale(1/(double)thyroepiglottic_nodes_thy.length);
      centreP.x = 100;
      FemMuscleModel grid = new FemMuscleModel("grid");
      FemFactory.createHexGrid(grid, 2.4, 1, 2.8, 1, 1, 2);
      System.out.println("centreP: " + centreP.x + "," + centreP.y + "," + centreP.z);
      grid.transformGeometry(new RigidTransform3d(centreP.x,centreP.y+1.2,centreP.z,1,0,0,Math.toRadians(15)));
      RenderProps.setPointRadius (grid, 0.2*pointRadius);
      RenderProps.setPointStyle (grid, Renderer.PointStyle.SPHERE);
      myMechMod.addModel(grid);
      PointList<FemNode3d> grid_nodes = grid.getNodes();
      
      // Nodes
      int incr = 4;
      int count = 0;
      Point3d origP,endP,midP = new Point3d();
      Vector3d step = new Vector3d();
      //System.out.println("Fem attachment:");
      System.out.println("Epiglottis Thyroepiglottic_ligament");
      for(int i=0;i<3;i++) {
	 for(int j=0;j<2;j++) {
	    origP = epi_nodes.get(thyroepiglottic_nodes_epi[count]).getPosition();
	    endP = grid_nodes.get(i*2*2+j).getPosition();
	    step.sub(endP,origP);
	    step.scale(1/(double)incr);
	    System.out.println(thyroepiglottic_nodes_epi[count] + " " + Thyroepiglottic_ligament.getNodes().size());
	    count++;
	    
	    Thyroepiglottic_ligament.addNode(new FemNode3d(origP));
	    for(int k=1;k<=incr;k++) {
	       midP.scaledAdd(k, step, origP);
	       Thyroepiglottic_ligament.addNode(new FemNode3d(midP));
	    }
	    //System.out.println(Thyroepiglottic_ligament.getNodes().size()-1);
	 }
      }
      System.out.println("end");
      
      // Elements
      PointList<FemNode3d> thyepi_nodes = Thyroepiglottic_ligament.getNodes();
      for(int i=0;i<2;i++) {
	 for(int j=0;j<1;j++) {
	    for(int k=0;k<incr;k++) {
	       
	       HexElement el = new HexElement(
		     thyepi_nodes.get(i*(incr+1)*2+j*(incr+1)+k),thyepi_nodes.get(i*(incr+1)*2+j*(incr+1)+k+1),
		     thyepi_nodes.get((i+1)*(incr+1)*2+j*(incr+1)+k+1),thyepi_nodes.get((i+1)*(incr+1)*2+j*(incr+1)+k),
		     thyepi_nodes.get(i*(incr+1)*2+(j+1)*(incr+1)+k),thyepi_nodes.get(i*(incr+1)*2+(j+1)*(incr+1)+k+1),
		     thyepi_nodes.get((i+1)*(incr+1)*2+(j+1)*(incr+1)+k+1),thyepi_nodes.get((i+1)*(incr+1)*2+(j+1)*(incr+1)+k)
		     );
	       Thyroepiglottic_ligament.addElement(el);
	       
	       System.out.println("Element: " +
		     (i*(incr+1)*2+j*(incr+1)+k) + " " +
		     (i*(incr+1)*2+j*(incr+1)+k+1) + " " +
		     ((i+1)*(incr+1)*2+j*(incr+1)+k+1) + " " +
		     ((i+1)*(incr+1)*2+j*(incr+1)+k) + " " +
             	(i*(incr+1)*2+(j+1)*(incr+1)+k) + " " +
             	(i*(incr+1)*2+(j+1)*(incr+1)+k+1) + " " +
             	((i+1)*(incr+1)*2+(j+1)*(incr+1)+k+1) + " " +
             	((i+1)*(incr+1)*2+(j+1)*(incr+1)+k));
	    }
	 }
      }
      RenderProps.setPointRadius (Thyroepiglottic_ligament, 0.2*pointRadius);
      RenderProps.setPointStyle (Thyroepiglottic_ligament, Renderer.PointStyle.SPHERE);
      myMechMod.addModel(Thyroepiglottic_ligament);
      //####################################################
      */
      
      
      /*
      int incr = 4;
      Vector3d rayDir = new Vector3d(0,1,0);
      Point3d origP,endP,hitP,midP;
      double dis, curDis;
      Vector3d duv = new Vector3d();
      Intersector isect = new Intersector();
      Face face_hit;
      
      ArrayList<OBBTree> bi_list = new ArrayList<OBBTree>();
      bi_list.add(myMechMod.rigidBodies().get("hyoid").getMesh().getObbtree());
      bi_list.add(myMechMod.rigidBodies().get("thyroid").getMesh().getObbtree());
      bi_list.add(myMechMod.rigidBodies().get("Thy_L").getMesh().getObbtree());
      bi_list.add(myMechMod.rigidBodies().get("Thy_R").getMesh().getObbtree());
      FemMuscleModel epiglottis = (FemMuscleModel)myMechMod.models().get("Epiglottis");
      
      FemMuscleModel adipose = new FemMuscleModel("adipose");
      
      // Nodes
      for(FemNode3d n: epiglottis.getNodes()) {
	 //Nodes: 0-169
	 if(n.getNumber()<170) {
	    origP = n.getPosition();
	    endP = null;
	    dis = Double.MAX_VALUE;
	    for(OBBTree obbt: bi_list) {
	       face_hit = obbt.intersect(origP, rayDir, duv, isect);
	       if(face_hit!=null) {
		  hitP = projToSurface (origP, rayDir, face_hit);
		  curDis = hitP.distance(origP);
		  if(dis>curDis) {
		     dis = curDis;
		     endP = hitP;
		  }
	       }
	    }
	    if(endP==null) {
	       endP = new Point3d(origP);
	       dis = 20;
	       endP.scaledAdd(dis, rayDir);
	    }
	    if(endP!=null) {
	       Vector3d step = new Vector3d(rayDir);
	       step.normalize();
	       step.scale(dis/incr);
	       midP = new Point3d();
	       
	       adipose.addNode(new FemNode3d(origP));
	       
	       for(int i=1; i<=incr; i++) {
		  midP.scaledAdd(i, step, origP);
		  adipose.addNode(new FemNode3d(midP));
	       }
	    }
	    
	 }
      }
      
      // Elements
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(
            new FileReader(femPath + "Epiglottis.elem"));
         int n;
         int[] nodes = new int[4];
         int i=0;
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            n = (int)rtok.nval;
            nodes[i] = n-1;
            i++;
            if(i==4) {
               i=0;
               for(int j=0;j<10;j++) {
        	  rtok.nextToken();
               }
              	System.out.println("Line: " +
                	nodes[0] + " " +
                	nodes[1] + " " +
                	nodes[2] + " " +
                	nodes[3] + " " +
                	nodes[0] + " " +
                	nodes[1] + " " +
                	nodes[2] + " " +
                	nodes[3]);
              	
               for(int k=0;k<incr;k++) {
        	  
        	  //System.out.println("k = " + k);
                  System.out.println("K(" + k + ") Elem: " +
                	(nodes[0]*(incr+1)+k) + " " +
                     	(nodes[1]*(incr+1)+k) + " " +
                     	(nodes[2]*(incr+1)+k) + " " +
                     	(nodes[3]*(incr+1)+k) + " " +
                     	(nodes[0]*(incr+1)+k+1) + " " +
                     	(nodes[1]*(incr+1)+k+1) + " " +
                     	(nodes[2]*(incr+1)+k+1) + " " +
                     	(nodes[3]*(incr+1)+k+1));
                  
                  HexElement el = new HexElement(
                        adipose.getNode(nodes[0]*(incr+1)+k),
                        adipose.getNode(nodes[1]*(incr+1)+k),
                        adipose.getNode(nodes[2]*(incr+1)+k),
                        adipose.getNode(nodes[3]*(incr+1)+k),
                        adipose.getNode(nodes[0]*(incr+1)+k+1),
                        adipose.getNode(nodes[1]*(incr+1)+k+1),
                        adipose.getNode(nodes[2]*(incr+1)+k+1),
                        adipose.getNode(nodes[3]*(incr+1)+k+1));
                  adipose.addElement(el);
               }
            }
         }
         
         rtok.close();
      } catch (IOException e) {
	    System.out.println("Warning: File " + femPath + "Epiglottis.elem" + " does not exist!");
      }

      RenderProps.setPointRadius (adipose, pointRadius);
      RenderProps.setPointStyle (adipose, Renderer.PointStyle.SPHERE);
      myMechMod.addModel(adipose);
      */
      
      /*
      MultiPointMuscle spring = new MultiPointMuscle("test1");
      spring.addPoint(((FemMuscleModel)myMechMod.models().get("Epiglottis")).getNode(23));
      spring.addPoint(((FemMuscleModel)myMechMod.models().get("Epiglottis")).getNode(16));
      spring.setMuscleType(MuscleType.Linear);
      spring.setMaxForce(1);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*3);
      RenderProps.setLineColor (spring, Color.RED);
      myMechMod.addMultiPointSpring(spring);
      spring = new MultiPointMuscle("test2");
      spring.addPoint(((FemMuscleModel)myMechMod.models().get("Epiglottis")).getNode(11));
      spring.addPoint(((FemMuscleModel)myMechMod.models().get("Epiglottis")).getNode(14));
      spring.setMuscleType(MuscleType.Linear);
      spring.setMaxForce(1);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*3);
      RenderProps.setLineColor (spring, Color.RED);
      myMechMod.addMultiPointSpring(spring);
      */
      
      /*
      FemMuscleModel adipose = new FemMuscleModel("adipose");
      FemFactory.createHexGrid(adipose, 20, 15, 25, 4, 3, 5);
      adipose.setMaterial (new LinearMaterial(3250,0.33));
      adipose.setIncompressible(FemModel3d.IncompMethod.OFF);
      adipose.transformGeometry(new RigidTransform3d(100,135,235));
      adipose.transformGeometry(OverallTrans);
      RenderProps.setPointRadius (adipose, pointRadius);
      RenderProps.setPointStyle (adipose, Renderer.PointStyle.SPHERE);
      RenderProps.setPointColor (adipose.getNodes(), Color.WHITE);
      myMechMod.addModel (adipose);
      */
      /*
      // Attach nodes
      RigidBody hyoid = myMechMod.rigidBodies().get("hyoid");
      int[] nodesAttach = {8,9,10,11,12,13,14,15};
      for(int n: nodesAttach){
	 FemNode3d nod = adipose.getNode(n);
	 myMechMod.attachPoint (nod,hyoid);
	 RenderProps.setPointStyle (nod, Renderer.PointStyle.SPHERE);
	 RenderProps.setPointRadius (nod, pointRadius*1.5);
	 RenderProps.setPointColor (nod, Color.BLUE);
      }
      */

      /*
      // Set collision
      RigidBody hyoid = myMechMod.rigidBodies().get("hyoid");
      FemMuscleModel adipose = (FemMuscleModel)myMechMod.models().get("Adipose");
      FemMuscleModel epiglottis = (FemMuscleModel)myMechMod.models().get("Epiglottis");
      myMechMod.setCollisionBehavior(adipose, hyoid, true);
      myMechMod.setCollisionBehavior(adipose, thyroid, true);
      myMechMod.setCollisionBehavior(adipose, epiglottis, true);
      */
      
      /*
      int i=0;
      for(FemNode3d n: adipose.getNodes()){
	 n.setNumber(i);
	 i++;
      }
      i=0;
      for(FemElement3d e: adipose.getElements()) {
	 e.setNumber(i);
	 i++;
      }
      //AnsysWriter.writeNodeFile (adipose, femPath + "adipose_trimmed_fixed" + ".node");
      //AnsysWriter.writeElemFile (adipose, femPath + "adipose_trimmed_fixed" + ".elem");
      */
      
      //myMechMod.attachPoint (((FemMuscleModel)myMechMod.models().get("Epiglottis")).getNode(28), thyroid);
      //myMechMod.attachPoint (((FemMuscleModel)myMechMod.models().get("Epiglottis")).getNode(29), thyroid);
   }
   
   public void attach(DriverInterface driver)
   {
      super.attach(driver);
      //setSagittalView(1.5);
      setSagittalView(3.0);
   }
   
   private void addFemSprings(String name, int[] nodes1, int[] nodes2, String femName, double stiffness, double damping) {
      FemMuscleModel femMuscle = (FemMuscleModel)myMechMod.models().get(femName);
      
      AxialSpring spring;
      int k = 0;
      for(int i=0; i<nodes1.length; i++) {
	 for(int j=0; j<nodes2.length; j++) {
	    spring = new AxialSpring(name+"_"+k);
	    k++;
	    spring.setFirstPoint(femMuscle.getNode(nodes1[i]));
	    spring.setSecondPoint(femMuscle.getNode(nodes2[j]));
	    spring.setRestLength(spring.getLength());
//	    spring.setStiffness(stiffness);
//	    spring.setDamping(damping);
	    spring.setLinearMaterial (stiffness, damping);
	    RenderProps.setLineStyle(spring, LineStyle.LINE);
	    RenderProps.setLineWidth(spring, 3);
	    RenderProps.setLineColor (spring, Color.CYAN);
	    myMechMod.addAxialSpring(spring);
	 }
      }
   }
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
	 if(TimeBase.modulo(t0,4*step) >=step) {
	    //ex = 1-ex;
	    ex = 0;
	 }
	 ml1.get(0).setExcitation(ex);
	 ml2.get(0).setExcitation(ex);
	 System.out.println("step = " + t0 + ", excitation = " + ex);
      }
   }
   //########################################################################
   
   //   private Point3d projToSurface (Point3d origP, Vector3d origV, Face face_ray) {
   //      Point3d proj = new Point3d();
   //      Point3d surP = new Point3d();
   //      Point3d surP_origP = new Point3d();
   //      Vector3d normal = face_ray.getNormal();
   //      
   //      face_ray.computeCentroid(surP);
   //      surP_origP.sub(surP, origP);
   //      proj.set(origV);
   //      proj.scale(normal.dot(surP_origP)/normal.dot(origV));
   //      proj.add(origP);
   //      
   //      return proj;
   //   }
   public void setSagittalView(double gridOffset) {
      GLViewer v = Main.getMain().getViewer();
      
      //vc.autoFit();
      v.setAxialView(AxisAlignedRotation.Y_Z);
      v.setEye(new Point3d(800, -100, -195));
      
      if (v.getNumClipPlanes() < 1) {
	 v.addClipPlane();
      }
      GLClipPlane clip  = v.getClipPlane (0);
      
      clip.setResolution(new GLGridResolution(100,10));
      clip.setPosition(getCenter());
      clip.setOrientation(new AxisAngle (0, 1, 0, Math.PI / 2));
      clip.setOffset (gridOffset);
      clip.setGridVisible (true);
      clip.setDragger (DraggerType.None);
   }
}
