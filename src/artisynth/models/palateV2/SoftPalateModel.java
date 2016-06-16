package artisynth.models.palateV2;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JFrame;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.*;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.*;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.NeoHookeanMaterial;
import artisynth.core.mechmodels.*;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.*;
import artisynth.core.util.*;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.face.BadinFaceDemo;
import artisynth.models.jawTongue.JawHyoidFemMuscleTongue;

public class SoftPalateModel extends JawHyoidFemMuscleTongue 
{
   public static final double dt = 0.001;
   public static final Integrator defaultIntegrator = Integrator.ConstrainedBackwardEuler;

   public MechModel myMechModel;
   public FemMuscleModel softPalate;
   public FemMuscleModel tongue;
   public RigidBody pharWall;

   //public String geometryName = "softPalate_v6.2q_tetgenWhole.1";	// high res palate
   //public String geometryName = "softPalate_v6.2_tetgenWhole.1";	// low res palate
   //public String geometryName = "softPalate_v8.3_tetgen";		// low res palate with uvula
   //public String geometryName = "softPalate_v8.3q_tetgen";		// high res palate with uvula
   //public static String geometryName = "softPalate_v9.3_tetgen";		  // low res palate with uvula
   public static final String geometryName = "softPalate_v9.3q_tetgen";         // high res palate with uvula
   //public static final String geometryName = "softPalate_v10.0_mixedFine";      // mixed elem, high res palate (from Claudio)
   //public static final String geometryName = "softPalate_v10.0_mixedCoarse";    // mixed elem, low res palate (from Claudio)
   //public static final String geometryName = "softPalate_v10.1_mixed";          // mixed elem, low res palate (from Claudio)
   

   public static final String geometryPath = ArtisynthPath.getSrcRelativePath ( SoftPalateModel.class, "geometry/");
   public static final String dataPath     = ArtisynthPath.getSrcRelativePath ( SoftPalateModel.class, "");

   public enum MaterialType {Linear, Neohookean, MooneyRivlin};
   public MaterialType materialType = MaterialType.Linear;

   public ArrayList<String> bundleNames;
   public double muscleMaxForce = 3;
   public boolean useMuscles = true;
   public boolean addMusclesAuto = false;
   public boolean anchorPalate = true;
   public boolean useControls = false;
   public boolean useProbes = true;
   public boolean usePharWall = false;
   
   private boolean noInit = false;

//   public SoftPalateModel (String name, boolean noInit) 
//   {
//      this.noInit = noInit;
//   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      if (!noInit) {
         createModel();
      }
   }
   
   public void createModel()
   {
      ArtisynthPath.setWorkingDir(new File(dataPath));

      // create mech model
      //myMechMod = new MechModel ("mech");
      myMechModel = this.myJawModel;
      myMechModel.setIntegrator (defaultIntegrator);
      myMechModel.setMaxStepSize(dt);
      setMaxStepSize(dt);

      tongue = (FemMuscleModel)myMechModel.models().get("tongue");
//      for (FemElement elem : tongue.getElements())
//      {
//	 if (elem.getClass() == HexElement.class)
//	    ((HexElement)elem).tesselate(0);      
//      }

      softPalate = createAndAddSoftPalate(materialType, useMuscles, anchorPalate, myMechModel, tongue);
      setupRenderProps(myMechModel, tongue, softPalate);

      //loadSkull();
      if (usePharWall)
         pharWall = createAndAddPharyngealWall(myMechModel, tongue, softPalate);

      myMechModel.setCollisionBehavior(tongue, softPalate, true);
      
      //myMechModel.scaleDistance(1.0/1000.0);
   }

   public static FemMuscleModel createAndAddSoftPalate(MaterialType materialType, 
      boolean addMuscles, boolean anchorPalate, 
      MechModel myMechModel, FemMuscleModel tongue) 
   {
      FemMuscleModel fem = createSoftPalate (materialType, addMuscles);
      myMechModel.addModel (fem);

      if (anchorPalate) {
         anchorSoftPalate(fem, tongue, myMechModel);
      }

      return fem;
   }
   
   public static FemMuscleModel createSoftPalate() {
      return createSoftPalate (MaterialType.Linear, /*addMuscles=*/true);
   }
   
   public static FemMuscleModel createSoftPalate(MaterialType materialType, boolean addMuscles) {
      FemMuscleModel fem = readFemMesh (geometryPath, geometryName, 1.0);
      fem.setName ("softPalate");
      setSoftPalateProperties (fem, materialType);
      if (addMuscles) {
         SoftPalateMuscles.addSoftPalateMuscles (fem);
      }
      return fem;
   }
   
   public static FemMuscleModel createSoftPalate(MaterialType materialType, boolean addMuscles, String geometryName) {
      FemMuscleModel fem = readFemMesh (geometryPath, geometryName, 1.0);
      fem.setName ("softPalate");
      setSoftPalateProperties (fem, materialType);
      if (addMuscles) {
         SoftPalateMuscles.addSoftPalateMuscles (fem);
      }
      return fem;
   }

   public static void setSoftPalateProperties (FemModel3d fem, MaterialType materialType) 
   {
      double E = 15000.0/1000.0;			// Youngs modulus --> 15000 Pa = 15000 kg/(m*s^2) * (1m/1000mm) = 15000/1000 = 15 kg/(mm*s^2)
      double nu = 0.49;					// poisson ratio
      double K = E/(3.0*(1.0-2.0*nu));			// bulk modulus  --> from wiki
      double G = E/(2.0*(1.0+nu));			// shear modulus --> from wiki
      double rho = 1040.0/(1000.0*1000.0*1000.0);	// density --> 1040 kg/m^3 * (1m/1000mm)^3 = 1040/1,000,000,000 = 0.000001040 kg/mm^3
      boolean isIncompressible = false;			//

      FemMaterial mat;
      if      ( materialType == MaterialType.Linear )
      {
	 LinearMaterial lnMat = new LinearMaterial();
	 lnMat.setPoissonsRatio(nu);
	 lnMat.setYoungsModulus(E);
	 mat = lnMat;
      }
      else if ( materialType == MaterialType.Neohookean )
      {
	 NeoHookeanMaterial nhMat = new NeoHookeanMaterial();
	 nhMat.setPoissonsRatio(nu);
	 nhMat.setYoungsModulus(E);
	 mat = nhMat;
      }
      else
      {
	 MooneyRivlinMaterial mrMat = new MooneyRivlinMaterial();	// c10=1037, c20=486, bm=10*c10
	 mrMat.setBulkModulus (K);
	 mrMat.setC01(G/2.0 - K/10.0);
	 mrMat.setC10 (K/10.0);                                   // Pa  [Buchaillard 2009],[Duck 1990]
	 //mrMat.setC20 (486);                                    // Pa
	 //mrMat.setC11();
	 //mrMat.setC02();
	 mat = mrMat;
      }

      fem.setMaterial(mat);
      fem.setDensity(rho);
      if (isIncompressible) {
         fem.setIncompressible (IncompMethod.AUTO);         
      }
      else {
         fem.setIncompressible (IncompMethod.OFF);
      }  

      // Set Numerical Properties
      fem.setParticleDamping(1.00);
      fem.setStiffnessDamping (0.03);

      //fem.setIntegrator (defaultIntegrator);
      //fem.setMaxStepSizeSec (defaultMaxStepSizeSec);
      //fem.setImplicitIterations (10);
      //fem.setImplicitPrecision (0.001);
   }


   public static void setupRenderProps(MechModel myMechModel, FemMuscleModel tongue, FemModel3d softPalate)
   {
      for(BodyConnector c: myMechModel.bodyConnectors())
	 RenderProps.setVisible(c, false);

      myMechModel.rigidBodies().get("maxilla").getRenderProps().setAlpha(0.2);
      myMechModel.rigidBodies().get("jaw"    ).getRenderProps().setAlpha(0.2);
      myMechModel.rigidBodies().get("maxilla").getRenderProps().setVisible(false);
      myMechModel.rigidBodies().get("jaw"    ).getRenderProps().setVisible(false);
      myMechModel.particles().getRenderProps().setVisible(true);
      myMechModel.axialSprings().getRenderProps().setVisible(false);
      myMechModel.multiPointSprings().getRenderProps().setVisible(false);
      tongue.getMuscleBundles ().getRenderProps ().setVisible (false);
      tongue.markers().getRenderProps().setPointColor(Color.green);
      tongue.markers().getRenderProps().setPointRadius(0.0001);

      setupSoftPalateRenderProps(softPalate);
   }

   public static void setupSoftPalateRenderProps (FemModel3d softPalate) {
      // Palate Rendering
      softPalate.getNodes().getRenderProps().setVisible(true);
      softPalate.getElements().getRenderProps().setVisible(true);
      softPalate.markers().getRenderProps().setVisible(true);
      softPalate.markers().getRenderProps().setPointColor(Color.green);

      softPalate.setSurfaceRendering   (SurfaceRender.Shaded);
      softPalate.setElementWidgetSize  (1.0);
      RenderProps.setFaceColor  (softPalate, new Color(153, 153, 255));	// 153,153,255
      RenderProps.setFaceStyle  (softPalate, FaceStyle.NONE);
      RenderProps.setDrawEdges  (softPalate, true);
      RenderProps.setPointStyle (softPalate, PointStyle.SPHERE);
      RenderProps.setLineWidth  (softPalate, 1);
      RenderProps.setPointRadius(softPalate, 0.1);
      RenderProps.setPointColor (softPalate, Color.BLUE);
      RenderProps.setLineStyle  (softPalate, LineStyle.SPINDLE);
      RenderProps.setLineColor  (softPalate, Color.LIGHT_GRAY);
      RenderProps.setLineRadius (softPalate, 0.8);

      for (FemNode3d node: softPalate.getNodes()) 
      {
	 if (!node.isDynamic()) {
	    RenderProps.setPointColor(node, Color.cyan);
	 }
      }
   }

   public static void anchorSoftPalate(FemMuscleModel softPalate, FemModel3d tongue, MechModel mechMod)
   {
      /*
       * The Soft Palate should be anchored in the following ways:
       * 1) the palatoglossus attaches to the tongue
       * 2) the palatopharyngeus attaches to the pharyngeal wall
       * 3) the tensor veli palitini wraps around the hamulus and attaches to the skull
       * 4) the levator veli palitini attaches to the skull
       * 5) the front of the soft palate attaches to the hard palate
       */


      /*/		Levator veli palitini --> Just make all nodes above a cutoff point static 
      for (FemNode3d node: softPalate.getNodes()) 
      {
	 double zValue = 126.0;
	 if (node.getPosition().z > zValue) 
	    node.setDynamic(false);
      }
      //*/


      //*/		Palatoglossus --> attach to nearest elements on the tongue 
      if (tongue != null)
      {
	 Point pR1 = softPalate.getMuscleBundles().get(SoftPalateMuscles.palatoglossus_R_name).getFibres().get( 9).getSecondPoint();
	 Point pR2 = softPalate.getMuscleBundles().get(SoftPalateMuscles.palatoglossus_R_name).getFibres().get(19).getSecondPoint();
	 Point pL1 = softPalate.getMuscleBundles().get(SoftPalateMuscles.palatoglossus_L_name).getFibres().get( 9).getSecondPoint();
	 Point pL2 = softPalate.getMuscleBundles().get(SoftPalateMuscles.palatoglossus_L_name).getFibres().get(19).getSecondPoint();

	 mechMod.attachPoint(pR1, tongue);
	 mechMod.attachPoint(pR2, tongue);
	 mechMod.attachPoint(pL1, tongue);
	 mechMod.attachPoint(pL2, tongue);
	 
	 // attaching the posterior palatoglossus
	 Point pR3 = softPalate.getMuscleBundles().get(SoftPalateMuscles.palatoglossus_post_R_name).getFibres().get( 9).getSecondPoint();
	 Point pL3 = softPalate.getMuscleBundles().get(SoftPalateMuscles.palatoglossus_post_L_name).getFibres().get( 9).getSecondPoint();
	 mechMod.attachPoint(pR3, tongue);
	 mechMod.attachPoint(pL3, tongue);
      }
      //*/    


      //*/		Attach the soft palate to the hard palate --> Make the front edge of the soft palate static
      for (FemNode3d node: softPalate.getNodes()) 
      {
	 double xValue_front = 105.0;
	 double zValue = 110.0;
	 if ((node.getPosition().x < xValue_front && node.getPosition().z > zValue)) 
	    node.setDynamic(false);
      }
      //*/

   }
   
   public static RigidBody createAndAddPharyngealWall(MechModel myMechModel, FemModel3d tongue, FemModel3d softPalate)
   {
      PolygonalMesh mesh;
      
      try
      {
	 mesh = new PolygonalMesh (new File (geometryPath + "pharyngealWall_thick,smooth.obj"));	 
	 mesh.scale(1000.0);		// scale to mm
	 //mesh.scale(1.0/1000.0);		// scale to m
	 //mesh.transform( new RigidTransform3d ( new Vector3d (0.0, 0.0, 0.0),     new AxisAngle (0.0, 0.0, 1.0, 180.0*Math.PI/180.0)) );
      }
      catch(Exception e)
      {
	 mesh = new PolygonalMesh();
      }
      RigidBody pharWall = new RigidBody();
      pharWall.setMesh(mesh, null);
      pharWall.setDensity(1.0);
      pharWall.setDynamic(false);
      pharWall.setName("pharyngealWall");

      pharWall.getRenderProps().setDrawEdges(false);
      pharWall.getRenderProps().setLineWidth(1);
      pharWall.getRenderProps().setFaceStyle(maspack.render.Renderer.FaceStyle.FRONT_AND_BACK);
      pharWall.getRenderProps().setFaceColor(java.awt.Color.blue);

      myMechModel.addRigidBody(pharWall);
      myMechModel.setCollisionBehavior(tongue,     pharWall, true);
      myMechModel.setCollisionBehavior(softPalate, pharWall, true);
      return pharWall;
   }

   public void loadSkull()
   {
      String skullPath = ArtisynthPath.getSrcRelativePath (BadinFaceDemo.class, "geometry/");
      String geometryName = "badinskull.obj";
      PolygonalMesh skullMesh;

      double scale = 1000.0;

      try
      {
	 skullMesh =  new PolygonalMesh(new File(skullPath + geometryName));
      }
      catch (Exception e)
      {
	 e.printStackTrace();
	 skullMesh = null;
      }

      skullMesh.scale(scale);

      RigidBody skull = new RigidBody();
      skull.setName("skull");
      skull.setMesh(skullMesh, null);
      skull.setDynamic(false);

      myMechModel.addRigidBody(skull);

   }

   public static FemMuscleModel readFemMesh(String meshDir, String meshBasename, double scale)
   {
       boolean isRead = false;
       Vector3d scaleVec = new Vector3d (scale, scale, scale);
       FemMuscleModel fem = new FemMuscleModel();

       // Tetgen reader
       if (isRead == false)
       {
           try 
           {
               String nodeString = meshDir + meshBasename + ".node";
               String elemString = meshDir + meshBasename + ".ele";
               TetGenReader.read ( fem, 1.0, nodeString, elemString, scaleVec );
               isRead = true;
           } 
           catch (Exception e) 
           {
               //e.printStackTrace ();
               isRead = false;
           }
       }
       // Ansys reader
       if (isRead == false)
       {
           try 
           {
               String nodeString = meshDir + meshBasename + ".node";
               String elemString = meshDir + meshBasename + ".elem";
               AnsysReader.read ( fem, nodeString, elemString, 1.0, scaleVec, /*options=*/0);
               isRead = true;
           } 
           catch (Exception e) 
           {
               //e.printStackTrace ();
               isRead = false;
           }
       }
       // UCD reader
       if (isRead == false)
       {
           try 
           {
               String filename = meshDir + meshBasename + ".inp";
               //AbaqusReader.read(fem, filename, 1.0);
               UCDReader.read(fem, filename, 1.0, scaleVec);
               isRead = true;
           } 
           catch (Exception e) 
           {
               //e.printStackTrace ();
               isRead = false;
           }
       }
       if (isRead == false)
       {
          try 
          {
             String filename = meshDir + meshBasename + ".vtk";
             VtkInputOutput.readUnstructuredMesh_volume(fem, filename);
             //VTK_IO_copy.readUnstructuredMesh_volume(filename, fem);
             //fem = (FemMuscleModel)VtkAsciiFemReader.read(filename);
             isRead = true;
          } 
          catch (Exception e) 
          {
              isRead = false;
          }
       }

       if (isRead == false)
           System.out.println("Failed to load " + meshBasename);

       return fem;
   }
   
   /*//
   public static FemMuscleModel readFemMesh(String meshDir, String meshName)
   {
      FemMuscleModel fem = new FemMuscleModel ();
      try 
      {
	 TetGenReader.read ( fem, fem.getDensity(), meshDir + meshName + ".node", meshDir + meshName + ".ele", new Vector3d (1, 1, 1));
      } 
      catch (Exception e) 
      {
	 try 
	 {
	    AnsysReader.read ( fem, meshDir + meshName + ".node", meshDir + meshName + ".elem", 1, null, 0);
	 } 
	 catch (Exception e2) 
	 {
	    try 
	    {
	       UCDReader.read (fem, meshDir + meshName + ".inp", 1);
	    } 
	    catch (Exception e3) 
	    {
	       e.printStackTrace ();
	       e2.printStackTrace ();
	       e3.printStackTrace ();
	    }
	 }
      }
      return fem;
   }
   //*/

   public static ControlPanel createMusclePanel(RootModel root,
	 FemMuscleModel fem) {
      ControlPanel controlPanel = new ControlPanel("Palate Muscles", "LiveUpdate");
      controlPanel.setScrollable(true);
      FemControlPanel.addBundleControls(controlPanel, fem);
      root.addControlPanel(controlPanel);
      return controlPanel;
   }

   public static ControlPanel createControlPanel(RootModel root,
	 FemMuscleModel fem, ModelComponent topModel) {
      ControlPanel controlPanel = new ControlPanel("Palate options", "LiveUpdate");
      controlPanel.setScrollable(true);
      FemControlPanel.addMuscleControls(controlPanel, fem, topModel);
      controlPanel.addWidget("elements visible", fem,  "elements:renderProps.visible");
      controlPanel.addWidget("muscles visisble", fem, "bundles:renderProps.visible");

      root.addControlPanel(controlPanel);
      return controlPanel;
   }

   ControlPanel myControlPanel = null;
   public void attach (DriverInterface driver) 
   {
      if (useProbes == true)
      {
	 SoftPalateProbes.createAndAddProbes(softPalate, this);	// must probes be loaded here?
      }

      if (useControls == true)
      {
	 super.attach (driver);

	 myControlPanel = SoftPalateModel.createControlPanel (this, softPalate, myMechModel);
	 if (softPalate.getMuscleBundles().size()>0)
	    SoftPalateModel.createMusclePanel(this, softPalate);

	 driver.getViewer().setGridVisible(false);
      }
   }


}
