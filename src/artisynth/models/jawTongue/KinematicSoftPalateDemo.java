package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.IOException;

import maspack.geometry.PolygonalMesh;
import maspack.interpolation.Interpolation.Order;
import maspack.render.RenderProps;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.workspace.RootModel;

public class KinematicSoftPalateDemo extends RootModel {
   MechModel myJawModel;
   
   public KinematicSoftPalateDemo () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      
      
      myJawModel = new MechModel("mech");
      addModel (myJawModel);
      
      addKinematicSoftPalate ();
      addInputProbe();
      
   }
   
   public void addKinematicSoftPalate() {
      try {
         PolygonalMesh spUp = new PolygonalMesh (AirwaySkinDemo.regGeomDir + "softpalate_up.obj");
         PolygonalMesh spDn = new PolygonalMesh (AirwaySkinDemo.regGeomDir + "softpalate_down.obj");
         MeshBlendController meshBlender = new MeshBlendController (spUp, spDn);
         addController (meshBlender);

         PolygonalMesh softPalate = meshBlender.getBlendedMesh ();
         MeshComponent comp = new MeshComponent ("softPalateSurf");
         comp.setMesh (softPalate);
         softPalate.setFixed (false);
         RenderProps.setFaceColor (comp, new Color(0.7f,0.6f,0.6f));
         myJawModel.addMeshBody (comp);
      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }
   
   public void addInputProbe() {
      NumericInputProbe ip = new NumericInputProbe (this.getControllers ().get (0), "blendFactor", 0, 5);
      ip.addData (new double[]{0, 1, 0}, 2.5);
      ip.setInterpolationOrder (Order.Cubic);
      addInputProbe (ip);
   }

}
