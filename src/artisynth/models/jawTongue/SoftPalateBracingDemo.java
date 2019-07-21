package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.palateV2.SoftPalateModel;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;

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
      
      //System.out.println("num handlers - "+myJawModel.getCollisionManagerOld ().collisionHandlers ().size ());
      CollisionResponse resp = myJawModel.setCollisionResponse (tongue, pharyngealWall);
      ContactMeasurer cm = new ContactMeasurer (resp);
//      for (CollisionHandler ch :  myJawModel.getCollisionManager ().collisionHandlers ()) {
//         cm.addCollisionHandler (ch);
//      }
      // need AJL_CONTOUR collider to be able to measure contact area
      myJawModel.getCollisionManager().setColliderType(
         CollisionManager.ColliderType.AJL_CONTOUR);
      cm.setHandlersToMeasure (new int[]{3}); // tongue-pharynx
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
         RenderProps.setVisible (softPalate.getAuxMaterialBundles (), false);

      }
      
      
      if (pharyngealWall != null) {
         RenderProps.setFaceColor (pharyngealWall, pharynxColor);
      }
   }
   
   public void attach (DriverInterface driver) {
      super.attach (driver);
 
   }
 
}
