package artisynth.models.toolbox.femtool;

import java.awt.Color;

import maspack.render.GLRenderer;
import maspack.render.RenderProps;
import maspack.render.RenderProps.PointStyle;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.TimeBase;
import artisynth.core.workspace.RootModel;
import artisynth.models.toolbox.femtool.DynamicFemFixer.ElementAlignType;

public class FemFixHexTest extends RootModel {

  
   DynamicFemFixer myFixer = null;
   DynamicFemFixerController myFixController = null;
   ElementAlignType myAlignmentType = ElementAlignType.LIMITED_ORTHOGONAL;

   double printInterval = 1;
   double updateInterval = 0.01;
   double myMinScale = 1.0/3.0;
     
   boolean renderRest = true;
   double pointRadius = 1;

   RenderProps restRenderProps = new RenderProps();

   MechModel myModel;
   FemModel3d fem;

   public FemFixHexTest(String name) {
      myModel = new MechModel("test");
      
      // build model
      fem = buildSingleHex();
      
      // set up fixer/controller
      myFixer = new DynamicFemFixer(myModel);
      myFixer.setAlignType(myAlignmentType);
      myFixer.setMinScaleRatio(myMinScale);
      myFixer.setYoungsModulus(1000);
      myFixer.setPoissonRatio(0.2);
      myFixer.setParticleDamping(0.1);
      
      myFixController = new DynamicFemFixerController(myFixer);
      myFixController.setUpdateInterval(updateInterval);
      myFixController.setAutoPause(false);
      
      // get the "fixing" FEM and add to model
      myFixer.setModel(fem);
      myModel.addModel(fem);
            
      // freeze all but one node
      // and set FEM parameters
      myFixer.freezeSurface(true);
      fem.getNode(6).setDynamic(true);
      
      // assign render properties
      setRenderProps(fem, true);
      
      // add model to controller
      addModel(myModel);
      addController(myFixController);
   }

   @Override
   public StepAdjustment advance(double t0, double t1, int flags) {

      // XXX here is where the model freezes at the fourth timestep
      if (Math.abs(t0-0.41)<1e-5) {
         System.out.println("Here is where the model freezes due to Pardiso iterative solver");
      }
      StepAdjustment res = super.advance(t0, t1, flags);

      // print information
      if (TimeBase.modulo(t1, printInterval)==0 ) {

         // check if anything inverted
         for (FemElement3d elem : fem.getElements()) {
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
   
   private void setRenderProps(FemModel3d model, boolean visible) {

      model.setSurfaceRendering(SurfaceRender.None);

      RenderProps.setAlpha(model, 1.0);
      RenderProps.setDrawEdges(model, true);
      RenderProps.setVisible(model.getElements(), visible);
      RenderProps.setVisible(model.getNodes(), visible);
      RenderProps.setPointStyle(model.getNodes(), PointStyle.SPHERE);
      RenderProps.setPointRadius(model.getNodes(), pointRadius);
      RenderProps.setVisible(model, false);

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

   private FemModel3d buildSingleHex() {

      FemNode3d[] nodes = { new FemNode3d(0,0,0), new FemNode3d(0,10,0), new FemNode3d(10,10,0), new FemNode3d(10,0,0),
                            new FemNode3d(0,0,10), new FemNode3d(0,10,10), new FemNode3d(15,15,10), new FemNode3d(10,0,10) };
      HexElement hex = new HexElement(nodes);

      FemModel3d model = new FemModel3d("hex");
      for (FemNode3d node : nodes) {
         model.addNode(node);
      }
      model.addElement(hex);
      return model;
   }

 
   @Override
   public void render(GLRenderer renderer, int flags) {

      if (renderRest) {
         for (FemElement3d elem : fem.getElements()) {
            renderRestElement(renderer, elem);
         }
      }
      super.render(renderer, flags);  

   }

   private void renderRestElement(GLRenderer renderer, FemElement3d elem) {

      FemNode3d[] nodes = elem.getNodes(); 
      float [][] v = new float[nodes.length][3];

      for (int i=0; i<nodes.length; i++) {
         v[i][0]=(float)(nodes[i].getRestPosition().x);
         v[i][1]=(float)(nodes[i].getRestPosition().y);
         v[i][2]=(float)(nodes[i].getRestPosition().z);        
      }

      if (elem instanceof HexElement) {
         renderer.drawHex(restRenderProps, 1, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
      } else if (elem instanceof WedgeElement) {
         renderer.drawWedge(restRenderProps, 1, v[0], v[1], v[2], v[3], v[4], v[5]);
      } else if (elem instanceof PyramidElement) {
         renderer.drawPyramid(restRenderProps, 1, v[0], v[1], v[2], v[3], v[4]);
      } else if (elem instanceof TetElement) {
         renderer.drawTet(restRenderProps, 1, v[0], v[1], v[2], v[3]);
      }

   }

}
