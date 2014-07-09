package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.IOException;

import maspack.render.RenderProps;
import maspack.render.RenderProps.LineStyle;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.palateV2.SoftPalateModel;

public class SoftPalateBracingDemo extends TongueBracingDemo {

   FemMuscleModel softPalate; 
   RigidBody pharyngealWall;
   
   public SoftPalateBracingDemo () {
   }

   public SoftPalateBracingDemo (String name) throws IOException {
      super (name);
      
      softPalate = SoftPalateModel.createSoftPalate ();
      myJawModel.addModel (softPalate);
      SoftPalateModel.anchorSoftPalate (softPalate, tongue, myJawModel);
      myJawModel.setCollisionBehavior(tongue, softPalate, true);
      pharyngealWall = SoftPalateModel.createAndAddPharyngealWall (myJawModel, tongue, softPalate);
//         SoftPalateModel.setupSoftPalateRenderProps (softPalate);
      setupRenderProps ();
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
