package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.palateV2.SoftPalateModel;
import maspack.render.RenderProps;
import maspack.render.RenderProps.LineStyle;

public class SoftPalateBracingDemo extends TongueBracingDemo {

   FemMuscleModel softPalate; 
   RigidBody pharyngealWall;
   
   public SoftPalateBracingDemo () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      myJawModel.setIntegrator (Integrator.ConstrainedBackwardEuler);
      double stepsize = 0.001;
      myJawModel.setMaxStepSize (stepsize);
      setMaxStepSize (stepsize);
      
//      myJawModel.getCollisionManager ().clear ();
      
      softPalate = SoftPalateModel.createSoftPalate ();
      myJawModel.addModel (softPalate);
      SoftPalateModel.anchorSoftPalate (softPalate, tongue, myJawModel);
      myJawModel.setCollisionBehavior(tongue, softPalate, true);
      pharyngealWall = SoftPalateModel.createAndAddPharyngealWall (myJawModel, tongue, softPalate);
//         SoftPalateModel.setupSoftPalateRenderProps (softPalate);
      setupRenderProps ();
      
      System.out.println("num handlers - "+myJawModel.getCollisionManager ().collisionHandlers ().size ());
      ContactMeasurer cm = new ContactMeasurer (myJawModel.getCollisionManager ());
//      for (CollisionHandler ch :  myJawModel.getCollisionManager ().collisionHandlers ()) {
//         cm.addCollisionHandler (ch);
//      }
      cm.setHandlersToMeasure (new int[]{4}); // tongue-pharynx
      addMonitor (cm);
   }
   
   public void setupRenderProps() {
      super.setupRenderProps ();
      
      Color tongueColor = tongue.getRenderProps ().getFaceColor ();
      Color pharynxColor = tongueColor.darker ().darker ();
      if (softPalate != null) {
         RenderProps.setLineWidth (softPalate, 0);
         RenderProps.setLineWidth (softPalate.getMuscleBundles (), 1);
         RenderProps.setLineStyle (softPalate.getMuscleBundles (), LineStyle.LINE);
         RenderProps.setVisible (softPalate.getMuscleBundles (), false);
         RenderProps.setFaceColor (softPalate, pharynxColor);
         softPalate.setElementWidgetSize (1);
         
         RenderProps.setVisible (softPalate.getNodes(), false);
         RenderProps.setVisible (softPalate.markers (), false);
//         RenderProps.setVisible (softPalate.getElements(), false);
         RenderProps.setVisible (softPalate.getMaterialBundles (), false);

      }
      
      
      if (pharyngealWall != null) {
         RenderProps.setFaceColor (pharyngealWall, pharynxColor);
      }
   }
   
   public void attach (DriverInterface driver) {
      super.attach (driver);
 
   }
 
}
