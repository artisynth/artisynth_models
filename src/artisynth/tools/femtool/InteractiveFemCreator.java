package artisynth.tools.femtool;

import java.awt.Color;
import java.awt.Panel;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JSeparator;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.AnsysWriter;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.tools.femtool.DynamicFemFixer.ElementAlignType;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.WavefrontReader;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

public class InteractiveFemCreator extends RootModel implements ActionListener   {

   private String outElemFile =  ArtisynthPath.getTempDir().getAbsolutePath() + "/mesh.elem";
   private String outNodeFile =  ArtisynthPath.getTempDir().getAbsolutePath() + "/mesh.node";
   
   public enum InitialFemGeneratorType {
      BASIC_TEMPLATE, HYBRID_PROJECTION, EXISTING_FEM
   }

   public enum FreezeType {
      NONE, SURFACE, INTERIOR, SELECTED, UNSELECTED, SNAPPED, SURFACE_CONSTRAIN, SURFACE_FORCE
   }

   MechModel myModel = new MechModel("DynamicFemFixer");

   public static InitialFemGeneratorType DEFAULT_INITIAL_GENERATOR_TYPE =
      InitialFemGeneratorType.BASIC_TEMPLATE;
   public static FreezeType DEFAULT_FREEZE_TYPE = FreezeType.SURFACE;

   InitialFemGeneratorType initGenType = DEFAULT_INITIAL_GENERATOR_TYPE;
   private FreezeType freezeType = DEFAULT_FREEZE_TYPE;
   private VolumetricMeshGenerator myInitGenerator = null;

   RenderProps frozenNodes;
   RenderProps activeNodes;
   
   // private enum FemFixerType {
   // DYNAMIC_FEM_FIXER
   // }
   //
   // private enum DynamicControllerType {
   // DYNAMIC_FEM_FIXER_CONTROLLER
   // }

   private Vector3d resolution = new Vector3d(16, 8, 3);
   private double expansionAmount = 0.3; // expand out 0.3 units

   private String surfaceFileName = ArtisynthPath.getHomeDir()
      + "/src/artisynth/models/alanMasseter/data/M16462/M16462MuscleSurface.obj";;

   private DynamicFemFixer myDynamicFixer = null;
   private DynamicFemFixerController myFixerController = null;
   private QHullSurfaceForceController mySurfaceForceController = null;
   private PolygonalMesh surfaceMesh = null;

   public static PropertyList myProps =
      new PropertyList(InteractiveFemCreator.class, RootModel.class);

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   static {
      myProps.add("resolution * *", "", new Vector3d());
      myProps.add("expansionAmount * *", "", 1.5, "%1.3f 1E [0,2]");
      myProps.add("surfaceFileName * *", "", null);
      myProps.add("initGenType * *", "", DEFAULT_INITIAL_GENERATOR_TYPE);
      myProps.add("freezeType * *", "", DEFAULT_FREEZE_TYPE);
   }

   public InteractiveFemCreator () {
      super();
   }

   public InteractiveFemCreator (String name) {
      super(name);

      myModel.setGravity(0, 0, 0);

      myDynamicFixer = new DynamicFemFixer(myModel);
      myDynamicFixer.setAlignType(ElementAlignType.LIMITED_ORTHOGONAL);
      myFixerController = new DynamicFemFixerController(myDynamicFixer);
      myInitGenerator = createGenerator(initGenType);

      // initial example... will remove eventually
      buildAndAddSurface();
      buildAndAddDynamicVolume();

      mySurfaceForceController = new QHullSurfaceForceController(
         myModel, surfaceMesh, myDynamicFixer.getModel()); 
      addSurfacePoints(mySurfaceForceController, myDynamicFixer.getModel());
      
      frozenNodes = new RenderProps();
      frozenNodes.setPointStyle(PointStyle.SPHERE);
      frozenNodes.setPointRadius(0.02);
      frozenNodes.setPointColor(Color.BLUE.brighter());
      activeNodes = new RenderProps(frozenNodes);
      activeNodes.setPointColor(Color.RED);
      
      addModel(myModel);
      addController(myFixerController);
      addController(mySurfaceForceController);

   }
   
   private void addSurfacePoints(QHullSurfaceForceController controller, FemModel3d model) {     
      controller.setEnabled(false);
      for (FemNode3d node : model.getNodes()) {
         if (model.isSurfaceNode(node)) {
            controller.addPoint(node);
         }
      }
   }

   private void buildAndAddSurface() {

      // remove surface it exists
      for (MeshComponent fmesh : myModel.meshBodies()) {
         if (fmesh.getMesh() == surfaceMesh) {
            myModel.removeMeshBody(fmesh);
            break;
         }
      }
      surfaceMesh = readSurface(surfaceFileName);

      // add surface mesh to model
      if (surfaceMesh == null) {
         return;
      }
      FixedMeshBody fmesh = new FixedMeshBody(surfaceMesh);
      RenderProps.setAlpha(fmesh, 0.2);
      RenderProps.setFaceColor(fmesh, Color.GREEN);
      myModel.addMeshBody(fmesh);

   }
   
   private void updateNodeRendering() {
      
      FemModel3d model = myDynamicFixer.getModel();
      if (model == null) {
         return;
      }
      
      for (FemNode3d node : model.getNodes()) {
         if (node.isDynamic()) {
            node.setRenderProps(activeNodes);
         } else {
            node.setRenderProps(frozenNodes);
         }
      }
      
   }

   public void buildAndAddDynamicVolume() {

      // remove FEM if it exists
      FemModel3d fixModel = myDynamicFixer.getModel();
      if (fixModel != null) {
         if (myModel.models().contains(fixModel)) {
            myModel.removeModel(fixModel);
         }
      }

      if (surfaceMesh == null) {
         return;
      }

      FemModel3d initialFem = buildInitialFem(surfaceMesh,
         new int[] { (int)resolution.x, (int)resolution.y, (int)resolution.z });

      myDynamicFixer.setModel(initialFem);
      fixModel = myDynamicFixer.getModel();
      myDynamicFixer.freezeSurface(true);

      RenderProps.setFaceColor(fixModel, Color.RED);
      RenderProps.setLineColor(fixModel.getElements(), Color.GRAY);
      RenderProps.setPointStyle(fixModel, PointStyle.SPHERE);
      RenderProps.setPointColor(fixModel, Color.RED);
      RenderProps.setPointRadius(fixModel, 0.02);
      
      myModel.addModel(fixModel);
      
   }

   private PolygonalMesh readSurface(String fileName) {

      // create initial model using HybridFemFactory
      PolygonalMesh myMesh = new PolygonalMesh();
      try {
         WavefrontReader wfr = new WavefrontReader(surfaceFileName);
         wfr.readMesh (myMesh);
         
         Vector3d c = new Vector3d();
         myMesh.computeCentroid(c);
         c.scale(-1);
         RigidTransform3d trans = new RigidTransform3d(c,new AxisAngle(1,0,0,Math.PI/2));
         myMesh.transform(trans);
         
         wfr.clear();
      } catch (IOException e) {
         e.printStackTrace();
         return null;
      }
      myMesh.triangulate();

      return myMesh;

   }

   private VolumetricMeshGenerator
      createGenerator(InitialFemGeneratorType type) {

      switch (type) {
         case HYBRID_PROJECTION: {
            return new HybridFemGenerator();
         }
         default:
            return new SimpleVolumeGenerator();
      }

   }

   private FemModel3d buildInitialFem(PolygonalMesh surface, int[] resolution) {

      FemModel3d myFem = myInitGenerator.generate(surface, resolution);
      return myFem;

   }

   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      addControlPanel();
      updateNodeRendering();
   }

   @Override
   public StepAdjustment advance(double t0, double t1, int flags) {
      StepAdjustment adj = super.advance(t0, t1, flags);
      return adj;
   }

   @Override
   public void initialize(double t) {
      super.initialize(t);
   }

   public Vector3d getResolution() {
      return resolution;
   }

   public void setResolution(Vector3d res) {
      resolution.set(res);
      reset();
   }

   public double getExpansionAmount() {
      return expansionAmount;
   }

   public void setExpansionAmount(double val) {
      expansionAmount = val;
   }

   public FreezeType getFreezeType() {
      return freezeType;
   }

   public void setFreezeType(FreezeType type) {

      mySurfaceForceController.setEnabled(false);
      switch (type) {
         case NONE:
            myDynamicFixer.freezeInterior(false);
            myDynamicFixer.freezeSurface(false);
            break;
         case INTERIOR:
            myDynamicFixer.freezeSurface(false);
            myDynamicFixer.freezeInterior(true);
            break;
         case SURFACE:
            myDynamicFixer.freezeInterior(false);
            myDynamicFixer.freezeSurface(true);
            break;
         case SELECTED:
            if (!freezeSelected(true)) {
               return;
            }
            break;
         case UNSELECTED:
            if (!freezeSelected(false)) {
               return;
            }
            break;
         case SNAPPED:
         {
            ArrayList<FemNode3d> snapped = myDynamicFixer.getNodesTouchingSurface(surfaceMesh, 0.1);
            if (snapped.size() < 3) {
               System.out.println("At least 3 nodes need to be snapped to surface");
               return;
            }
            
            for (FemNode3d node : myDynamicFixer.getModel().getNodes()) {
               if (snapped.contains(node)) {
                  node.setDynamic(false);
               } else {
                  node.setDynamic(true);
               }
            }
            break;
         }
         case SURFACE_CONSTRAIN:
            myDynamicFixer.freezeInterior(true);
            myDynamicFixer.freezeSurface(false);
            myDynamicFixer.constrainSurface(surfaceMesh);
            break;
         case SURFACE_FORCE:
            mySurfaceForceController.setEnabled(true);
            break;
         default:
            System.out.println("Type "+ type + " not yet implemented");
            return;
      }
      
      freezeType = type;
      updateNodeRendering();

   }
   
   private boolean freezeSelected(boolean select) {

      FemModel3d model = myDynamicFixer.getModel();
      if (model == null) {
         return false;
      }
      
      // first check how many nodes are selected
      Collection<FemNode3d> nodeList = model.getNodes();
      //      int numSelected = 0;
      //
      //      for (FemNode3d node : nodeList) {
      //         if (node.isSelected() == select) {
      //            numSelected++;
      //         }
      //      }
      //
      //      if (numSelected < 3) {
      //         System.err.println("Must have at least 3 nodes anchored");
      //         return false;
      //      }

      for (FemNode3d node : nodeList) {
         if (node.isSelected()==select) {
            node.setDynamic(false);
         } else {
            node.setDynamic(true);
         }
      }
      
      return true;
   }

   public InitialFemGeneratorType getInitGenType() {
      return initGenType;
   }

   public void setInitGenType(InitialFemGeneratorType type) {
      initGenType = type;
      myInitGenerator = createGenerator(type);
      reset();
   }

   public void setSurfaceFileName(String filename) {
      surfaceFileName = filename;
   }

   public String getSurfaceFileName() {
      return surfaceFileName;
   }

   private void addControlPanel() {
      ControlPanel panel =
         new ControlPanel("Fitting Parameters", "LiveUpdate");

      panel.addWidget("Enabled", myFixerController, "enabled");
      panel.addWidget("Initial volume generator", this, "initGenType");
      panel.addWidget("Resolution", this, "resolution");
      panel.addWidget("Projection/Expansion amount", this, "expansionAmount", 0, 2);
      panel.addWidget("Surface geometry", this, "surfaceFileName");
      panel.addWidget("Freeze  ", this, "freezeType");
      panel.addWidget(new JSeparator());
      panel.addWidget(new JLabel("DynamicFemFixer Props"));
      DynamicFemFixer.addControls(panel, myDynamicFixer);
      panel.addWidget(new JSeparator());
      panel.addWidget(new JLabel("DynamicFemFixerController Props"));
      DynamicFemFixerController.addControls(panel, myFixerController);
      panel.addWidget(new JSeparator());
      panel.addWidget(new JLabel("SurfaceForceController Props"));
      QHullSurfaceForceController.addControls(panel, mySurfaceForceController);
      panel.addWidget(new JSeparator());
      
      Panel topButtonPanel = new Panel();//new BoxLayout()); //new FlowLayout(FlowLayout.LEFT));
      Panel botButtonPanel = new Panel();//new BoxLayout()); //new FlowLayout(FlowLayout.LEFT));
      
      JButton b = new JButton("Expand");
      b.addActionListener(this);
      topButtonPanel.add(b);

      b = new JButton("Project");
      b.addActionListener(this);
      topButtonPanel.add(b);
      
      b = new JButton("Project Normally");
      b.setActionCommand("ProjectNormally");
      b.addActionListener(this);
      topButtonPanel.add(b);
      
      b = new JButton("Rest");
      b.setActionCommand("SetRest");
      b.addActionListener(this);
      topButtonPanel.add(b);

      b = new JButton("Reset");
      b.addActionListener(this);
      botButtonPanel.add(b);
      
      b = new JButton("Save");
      b.addActionListener(this);
      botButtonPanel.add(b);

      panel.addWidget(topButtonPanel);
      panel.addWidget(botButtonPanel);

      topButtonPanel.setBackground(Color.GREEN);
      botButtonPanel.setBackground(Color.GREEN);
      
      addControlPanel(panel);

   }

   public void reset() {
      Main.getMain().getScheduler().stopRequest();
      Main.getMain().getScheduler().waitForPlayingToStop();
      buildAndAddDynamicVolume();
      mySurfaceForceController.clearPoints();
      addSurfacePoints(mySurfaceForceController, myDynamicFixer.getModel());
      mySurfaceForceController.setFemReference(myDynamicFixer.getModel());
      
      setFreezeType(DEFAULT_FREEZE_TYPE);
      rerender();
      Main.getMain().reset();
   }

   public void actionPerformed(ActionEvent e) {

      String cmd = e.getActionCommand();

      Main main = Main.getMain();
      boolean simulating = main.isSimulating();

      if (simulating) {
         main.pause();
      }

      if ("Expand".equals(cmd)) {
         myDynamicFixer.expandSurfaceNormally(expansionAmount);
         myDynamicFixer.updateModel();
      } else if ("Project".equals(cmd)) {
         myDynamicFixer.projectSurfaceToMesh(surfaceMesh);
         myDynamicFixer.updateModel();
      } else if ("ProjectNormally".equals(cmd)) {
         myDynamicFixer.projectSurfaceToMeshNormally(surfaceMesh, expansionAmount, 0.9);
         myDynamicFixer.updateModel();
      } else if ("SetRest".equals(cmd)) {
         myDynamicFixer.setModelToRest();
      } else if ("Reset".equals(cmd)) {
         reset();
         return;
      } else if ("Save".equals(cmd)) {
         
         FemModel3d ref = myDynamicFixer.getModel();
         if (ref != null) {
            myDynamicFixer.updateModel();
            
            System.out.println("Writing files:");
            System.out.println("  " + outElemFile);
            System.out.println("  " + outNodeFile);
            
            AnsysWriter.writeElemFile(ref, outElemFile);
            AnsysWriter.writeNodeFile(ref, outNodeFile);
            
         }
         
      }

      if (simulating) {
         Main.getMain().play();
      }

      this.rerender();

   }

}
