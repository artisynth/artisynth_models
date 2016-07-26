package artisynth.tools.femtool;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import maspack.matrix.AffineTransform3d;
import maspack.properties.PropertyList;
import maspack.render.Renderer;
import maspack.render.RenderProps;
import maspack.render.RenderList;
import maspack.render.Renderer.PointStyle;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemElementRenderer;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.QuadtetElement;
import artisynth.core.femmodels.QuadhexElement;
import artisynth.core.femmodels.QuadwedgeElement;
import artisynth.core.femmodels.QuadpyramidElement;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.TimeBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.tools.exReader.ExElement;
import artisynth.tools.exReader.ExMeshGenerator;
import artisynth.tools.exReader.ExParser;
import artisynth.tools.exReader.ExRegion;
import artisynth.tools.exReader.TangentFlipper;
import artisynth.tools.femtool.DynamicFemFixer.ElementAlignType;

public class FemFixTest extends RootModel {

   static int test = 2;
   
   DynamicFemFixer myFixer = null;
   ElementAlignType myAlignmentType = ElementAlignType.ORTHOGONAL;
   double alignLimitRatio = 0.2;

   boolean update = true;
   double printInterval = 1;
   double updateInterval = 0.01;
   
   DynamicFemFixerController myFixController = null;
   
   boolean renderRest = false;
   boolean trim = false;
   double pointRadius = 1;

   RenderProps restRenderProps = new RenderProps();
   private static String exNodePath = ArtisynthPath.getSrcRelativePath(FemFixTest.class, "data/");
   ExParser myParser;
   String meshName = "brachioradialis";

   MechModel myModel;
   FemModel3d origModel;
   FemModel3d fixModel;

   FemElementRenderer[] myRenderers = new FemElementRenderer[8];

   public static PropertyList myProps =
      new PropertyList(FemFixTest.class, RootModel.class);
   static {
      myProps.add("renderRest * *", "Render rest positions", false);
      myProps.add("alignType * *", "Alignment type", ElementAlignType.LIMITED_ORTHOGONAL);
      myProps.add("minScaleRatio * *", "Minimum scale ratio", 0.2);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public FemFixTest(String name) {
      this(name,test);
   }

   public FemFixTest(String name, int testID) {

      myModel = new MechModel("test");
      myModel.setMaxStepSize(0.001);
      this.setMaxStepSize(0.005);
      
      switch(testID) {
         case -1:
            origModel = buildSingleWedge();
            break;
         case 0:
            origModel = buildSingleHex();
            break;
         case 1:
            origModel = buildTestHexes();
            break;
         default:
            origModel = buildMuscleModel();
            AffineTransform3d trans = new AffineTransform3d();
            trans.applyScaling(0.6, 0.6, 0.6);
            origModel.transformGeometry(trans);
            break;
      }      
      
      myFixer = new DynamicFemFixer(myModel);
      myFixer.setAlignType(myAlignmentType);
      myFixer.setMinScaleRatio(0.2);
      myFixController = new DynamicFemFixerController(myFixer);
      myFixController.setUpdateInterval(updateInterval);
      myFixController.setAutoPause(false);
      
      myFixer.setModel(origModel);
      fixModel = myFixer.getModel();
      myFixer.freezeSurface(true);
      
      if (testID <= 0) {
         // pick node to un-freeze
         fixModel.getNode(6+testID).setDynamic(true);
      }

      myModel.addModel(fixModel);
      
      myFixer.setYoungsModulus(100);
      myFixer.setPoissonRatio(0.3);
      myFixer.setParticleDamping(10);
      
      setRenderProps(origModel, false);
      setRenderProps(fixModel, true);

      //myModel.addModel(origModel);
      Main.getMain().getViewerManager().setBackgroundColor(Color.WHITE);

      addModel(myModel);
      addController(myFixController);

   }
   
   @Override
   public void initialize(double t) {
      super.initialize(t);
      
      resetModel();
   }

   @Override
   public StepAdjustment advance(double t0, double t1, int flags) {

      StepAdjustment res = super.advance(t0, t1, flags);

      if (TimeBase.modulo(t1, printInterval)==0 ) {

         // check if anything inverted
         for (FemElement3d elem : fixModel.getElements()) {
            if (elem.isInvertedAtRest()) {
               System.out.println("Element " + elem.getNumber() + " has an inverted rest position");
            }
            
            double detJ = DynamicFemFixer.getMinDetJ(elem);
            if (detJ < 0) {
               System.out.println("Element " + elem.getNumber() + " is inverted with detJ = " + detJ);
            }
         }
      }

      return res;

   }

   private void resetModel() {
      myFixer.updateIdealRestPositions();
   }
   
   private void setRenderProps(FemModel3d model, boolean visible) {

      model.setSurfaceRendering(SurfaceRender.None);

      RenderProps.setAlpha(model, 1.0);
      RenderProps.setDrawEdges(model, true);
      RenderProps.setVisible(model.getElements(), visible);
      RenderProps.setVisible(model.getNodes(), visible);
      RenderProps.setPointStyle(model.getNodes(), PointStyle.SPHERE);
      RenderProps.setPointRadius(model.getNodes(), pointRadius);
      RenderProps.setVisible(model, false);
      RenderProps.setLineWidth(model, 2);

      restRenderProps.setDrawEdges(true);
      restRenderProps.setPointStyle(PointStyle.SPHERE);
      restRenderProps.setPointRadius(pointRadius);
      restRenderProps.setLineColor(Color.ORANGE);
      restRenderProps.setPointColor(Color.ORANGE);
      restRenderProps.setAlpha(0.2);

      for (FemNode3d node : model.getNodes()) {
         if (node.isDynamic()) {
            RenderProps.setPointColor(node, Color.GREEN);
         } else {
            RenderProps.setPointColor(node, Color.RED);
         }
      }

   }

   private FemModel3d buildSingleWedge() {
      FemNode3d[] nodes = { new FemNode3d(0,0,0), new FemNode3d(10,0,0), new FemNode3d(10,10,0), 
                            new FemNode3d(0,0,10), new FemNode3d(10,0,10), new FemNode3d(15,15,10) };
      WedgeElement wedge = new WedgeElement(nodes);

      FemModel3d model = new FemModel3d();
      for (FemNode3d node : nodes) {
         model.addNode(node);
      }
      model.addElement(wedge);
      return model;
   }

   private FemModel3d buildSingleHex() {

      FemNode3d[] nodes = { new FemNode3d(0,0,0), new FemNode3d(0,10,0), new FemNode3d(10,10,0), new FemNode3d(10,0,0),
                            new FemNode3d(0,0,10), new FemNode3d(0,10,10), new FemNode3d(15,15,10), new FemNode3d(10,0,10) };
      HexElement hex = new HexElement(nodes);

      FemModel3d model = new FemModel3d();
      for (FemNode3d node : nodes) {
         model.addNode(node);
      }
      model.addElement(hex);
      return model;
   }

   private FemModel3d buildTestHexes() {

      FemNode3d[] nodes = {      new FemNode3d(0,0,0), new FemNode3d(10,0,0), new FemNode3d(10,10,0), new FemNode3d(0,10,0),
                                 new FemNode3d(0,0,10), new FemNode3d(10,0,10), new FemNode3d(17,17,17), new FemNode3d(0,10,10),
                                 new FemNode3d(0,0,20), new FemNode3d(10,0,20), new FemNode3d(10,10,20), new FemNode3d(0,10,20),
                                 new FemNode3d(20,0,0), new FemNode3d(20,10,0),
                                 new FemNode3d(20,0,10), new FemNode3d(20,10,10),
                                 new FemNode3d(20,0,20), new FemNode3d(20,10,20),
                                 new FemNode3d(10,20,0), new FemNode3d(0,20,0),
                                 new FemNode3d(10,20,10), new FemNode3d(0,20,10),
                                 new FemNode3d(10,20,20), new FemNode3d(0,20,20),
                                 new FemNode3d(20,20,0), new FemNode3d(20,20,10), new FemNode3d(20,20,20)   };

      int[][] HexNodeIdxs = { {4,5,6,7,0,1,2,3},
                              {8,9,10,11,4,5,6,7},
                              {5,14,15,6,1,12,13,2},
                              {9,16,17,10,5,14,15,6},
                              {7,6,20,21,3,2,18,19},
                              {11,10,22,23,7,6,20,21},
                              {6,15,25,20,2,13,24,18},
                              {10,17,26,22,6,15,25,20} };

      FemModel3d model = new FemModel3d();
      for (FemNode3d node : nodes) {
         model.addNode(node);
      }

      FemNode3d[] hexNodes = new FemNode3d[8]; 
      for (int i=0; i<HexNodeIdxs.length; i++) {
         for (int j=0; j<HexNodeIdxs[i].length; j++) {
            hexNodes[j] = nodes[HexNodeIdxs[i][j]];
         }
         HexElement hex = new HexElement(hexNodes);
         model.addElement(hex);
      }

      return model;

   }

   private FemModel3d buildMuscleModel() {

      myParser = new ExParser();

      try {
         myParser.parseExNode(exNodePath + meshName + ".exnode");
         myParser.parseExElem(exNodePath + meshName + ".exelem.fixed");
      } catch (IOException e) {
         e.printStackTrace();
         return null;
      }

      ExRegion lastRegion = myParser.getLastParsedRegion(); // last read region
      TangentFlipper flipper = new TangentFlipper();
      flipper.flipAllTangents(lastRegion.getElements(), 2);

      String modelName = lastRegion.getName().replace("/", "");

      // trim
      if (trim) {
         int [] idxs = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};//{16,17,18, 28, 29, 30}; //4,5,6,

         ArrayList<ExElement> newElems = new ArrayList<ExElement>();
         for (int i=0; i<idxs.length; i++) {
            newElems.add(lastRegion.getElements().get(idxs[i]));
         }
         lastRegion.setFromElements(newElems);
      }

      FemModel3d fem0 = null;
      try {
         fem0 = ExMeshGenerator.generateLinearFEM(
            lastRegion.getElements(), new int[]{2,2,2}, 0);
      } catch (Exception e) {
         e.printStackTrace();
      }

      for (FemElement3d femElem : fem0.getElements()) {
         if (femElem.isInvertedAtRest()) {
            System.out.println("Element " + femElem.myNumber
               + " is inverted at rest");
         }
      }

      fem0.setName(modelName);
      return fem0;

   }

   @Override
   public void render(Renderer renderer, int flags) {

      if (renderRest) {
         for (FemElement3d elem : fixModel.getElements()) {
            renderRestElement(renderer, elem);
         }
      }
      super.render(renderer, flags);  

   }

   static int getElementTypeIndex (FemElement3d elem) {

      if (elem instanceof TetElement) {
         return 0;
      }
      else if (elem instanceof HexElement) {
         return 1;
      }
      else if (elem instanceof WedgeElement) {
         return 2;
      }
      else if (elem instanceof PyramidElement) {
         return 3;
      }
      else if (elem instanceof QuadtetElement) {
         return 4;
      }
      else if (elem instanceof QuadhexElement) {
         return 5;
      }
      else if (elem instanceof QuadwedgeElement) {
         return 6;
      }
      else if (elem instanceof QuadpyramidElement) {
         return 7;
      }
      else {
         return -1;
      }
   }

   private void renderRestElement(Renderer renderer, FemElement3d elem) {

      int idx = getElementTypeIndex (elem);
      if (idx != -1) {
         if (myRenderers[idx] == null) {
            myRenderers[idx] = new FemElementRenderer (elem);
         }
         myRenderers[idx].renderRestWidget (renderer, elem, 1.0, myRenderProps);
      }
   }
   
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      
      addControlPanel();
      
   }

   public void setRenderRest(boolean render) {
      renderRest = render;
      rerender();
   }
   public boolean getRenderRest() {
      return renderRest;
   }
   
   public ElementAlignType getAlignType() {
      return myFixer.getAlignType();
   }
   public void setAlignType(ElementAlignType type) {
      myFixer.setAlignType(type);
   }
   public double getMinScaleRatio() {
      return myFixer.getMinScaleRatio();
   }
   public void setMinScaleRatio(double ratio) {
      myFixer.setMinScaleRatio(ratio);
   }
   
   public void addControlPanel() {
      ControlPanel FemFixPanel = new ControlPanel("Render Controls");
      FemFixPanel.addWidget("Render Rest Positions", this,"renderRest");
      FemFixPanel.addWidget("Alignment type", this, "alignType");
      FemFixPanel.addWidget("Min scale ratio", this,"minScaleRatio" );
      FemFixPanel.pack();
      addControlPanel(FemFixPanel);
   }
   
}
