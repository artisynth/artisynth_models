package artisynth.models.face;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.Plane;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

public class RefFaceDemo extends RootModel {
   
   public static final double mm2m = 1d/1000d;

   public static final boolean linearMaterial = false;
   public static final double defaultMaxStepSizeSec = 0.005;
   public static final Integrator defaultIntegrator =
      Integrator.Trapezoidal;

   public static final String refFaceGeometryDir =
      ArtisynthPath.getSrcRelativePath (RefFaceDemo.class, "geometry/refmodel/");
   
   MechModel mech;
   FemMuscleModel face;
   RigidBody jaw;
   RigidBody maxilla;

   /*
    * assemble reference face model geometry
    */
   public RefFaceDemo () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      

      // create mech model
      mech = new MechModel ("mech");
      mech.setMaxStepSize (defaultMaxStepSizeSec);
      mech.setIntegrator (defaultIntegrator);
      addModel (mech);

      // create dynamic face model
      face = BadinFaceDemo.createFace(refFaceGeometryDir, "newface",
	    mm2m, linearMaterial);
      face.setName ("refface");
      mech.addModel (face);
      
      BadinFaceDemo.addMuscles(face, refFaceGeometryDir+"muscles.node", 
	    mm2m, new Plane(Vector3d.X_UNIT, 0));
      
      jaw = BadinFaceDemo.addBody(mech, "mandible", refFaceGeometryDir+"mandible.obj", mm2m);
      maxilla = BadinFaceDemo.addBody(mech, "maxilla", refFaceGeometryDir+"maxilla.obj", mm2m);
      enableLipSkullContact();
      enableLipLipContact();
      BadinFaceDemo.showCollisions(mech, true);
      
      fixNodes("fixed_skull_nose.node");
      fixNodes("fixed_mandible.node");
      
      setupRenderProps();
      
   }
   
   private void fixNodes(String nodeFilename) {
      for (FemNode3d n : BadinFaceDemo.getNodes(face, refFaceGeometryDir+nodeFilename)) {
	 n.setDynamic(false);
	 RenderProps.setPointColor(n, Color.GREEN);
      }
   }

   public void enableLipSkullContact() {
      mech.setCollisionBehavior(face, jaw, true);
      mech.setCollisionBehavior(face, maxilla, true);
   }

   public void enableLipLipContact() {

      //PolygonalMesh lowerlipMesh, upperlipMesh;
      try {
	 face.addMeshComp (
	    face.scanMesh (refFaceGeometryDir + "lowerlip_filled.smesh"));
	 face.addMeshComp (
	    face.scanMesh(refFaceGeometryDir + "upperlip_filled.smesh"));
      } catch (IOException e) {
	 e.printStackTrace();
      }
      mech.setCollisionBehavior(face, face, true);
   }
   
   public void setupRenderProps() {
      RenderProps.setFaceStyle(mech, FaceStyle.FRONT_AND_BACK);

      for (RigidBody body : mech.rigidBodies()) {
	 RenderProps.setFaceColor(body, new Color(0.4f, 0.6f, 0.8f));
	 RenderProps.setFaceStyle(body, FaceStyle.FRONT_AND_BACK);
	 RenderProps.setVisible(body, true);
      }

      RenderProps.setFaceColor(face, new Color(0.8f, 0.6f, 0.4f));
      RenderProps.setLineWidth(face.getMuscleBundles(), 2);
      RenderProps.setPointStyle(face, PointStyle.POINT);
      RenderProps.setPointSize(face, 4);
      RenderProps.setPointRadius(face, 0.0005);
      RenderProps.setPointColor(face.markers(), Color.LIGHT_GRAY);
      face.setElementWidgetSize(1);
      face.setSurfaceRendering(SurfaceRender.None);

      for (FemNode3d n : face.getNodes()) {
	 RenderProps.setVisibleMode(n, PropertyMode.Inherited);
      }
      
      RenderProps.setVisible(face.getNodes(), false);
      RenderProps.setPointColor(face.getNodes(), Color.DARK_GRAY);

   }
   
   public void attach(DriverInterface driver) {
      super.attach(driver);
      
      if (myControlPanels.size() == 0) {
	 BadinFaceDemo.createVisibilityPanel(this, mech);
	 FemControlPanel.createControlPanel(this, face, mech);
	 FemControlPanel.createMuscleBundlesPanel(this, face);
      }
      
      ArtisynthPath.setWorkingDir(new File(ArtisynthPath.getSrcRelativePath(BadinFaceDemo.class, "data/")));
      BadinFaceDemo.addWayPoints(this, 1d, 0.01);
      BadinFaceDemo.addMuscleProbes(this, face, 1d, "OOP");
   }

}
