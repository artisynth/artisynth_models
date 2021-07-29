package artisynth.models.registration.demos;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.registration.DynamicRegistrationController;
import artisynth.models.registration.MeshCorrespondenceController;
import artisynth.models.registration.correspondences.GMMMeshCorrespondence;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;

/**
 * Registration demo with a sphere registered to an open box
 */
public class FemRegistrationDemo extends RootModel {

   MechModel mech;
   ControlPanel controls;
   
   static boolean removeFace = true;
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      mech = new MechModel("mech");
      mech.setGravity (0, 0, 1e-18);
      addModel(mech);
      
      double width = 0.01;
      int divisions = 4;
      PolygonalMesh sphere2 = MeshFactory.createOctahedralSphere (Math.cbrt (3/(4*Math.PI))*width, divisions);
      
      PolygonalMesh box = MeshFactory.createBox (width, width, width);
      if (removeFace) {
         box.removeFace (box.getFace (5));
         box.removeFace (box.getFace (4));
      }
      PolygonalMesh box2 = MeshFactory.subdivide (box, divisions);
    
      FemModel3d sphere2source = new FemModel3d("sphere2source");
      FemFactory.createFromMesh (sphere2source, sphere2, 1.1);
      sphere2source.setDensity (1000);
      sphere2source.setLinearMaterial (30000, 0.3, true);
      mech.addModel (sphere2source);
      sphere2source.setSurfaceRendering (SurfaceRender.Shaded);
      RenderProps.setFaceColor (sphere2source, Color.MAGENTA);
      
      FixedMeshBody box2target = new FixedMeshBody ("box2target", box2);
      RenderProps.setFaceColor (box2target, Color.CYAN);
      RenderProps.setAlpha (box2target, 0.2);
      RenderProps.setDrawEdges (box2target, true);
      RenderProps.setEdgeColor (box2target, Color.CYAN.darker ());
      mech.addRenderable (box2target);
      
      controls = new ControlPanel("controls");
      addControlPanel (controls);
      
      addRegistrationPair ("spherecube2", sphere2source.getSurfaceMeshComp (), box2target.getMesh ());
      
   }
   
   protected void addRegistrationPair(String name, FemMeshComp model, MeshBase target) {
      DynamicRegistrationController reg = new DynamicRegistrationController(mech);
      MeshCorrespondenceController mcc = reg.addRegistrationTarget (model, target, 1.0, new GMMMeshCorrespondence ());
      reg.setName (name);
      addController (reg);
      
      controls.addWidget (name + " scaling", reg, "forceScaling");
      controls.addWidget (name + " correspondence", mcc, "correspondenceComputer");
      controls.addWidget (name + " enabled", reg, "enabled");
      controls.pack ();
   }
   
   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);
      
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }
   }
}
