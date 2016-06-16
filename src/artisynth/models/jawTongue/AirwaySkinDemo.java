package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import maspack.geometry.PolygonalMesh;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.ColorMapProps;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.ModelComponentBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.palateV2.SoftPalateModel;

public class AirwaySkinDemo extends StaticJawHyoidTongue {

   protected AirwaySkin airwaySkin = null;
   protected FemMuscleModel softPalate = null;
   
   // TODO: Use Unique names instead of using this hack.
   static {
      //ModelComponentBase.enforceUniqueNames = false;
   }
   
   public AirwaySkinDemo () throws IOException {
	   super();
   }

   @Override
   public void build (String[] args) throws IOException {
      // TODO: Use Unique names instead of using this hack.      
      ModelComponentBase.enforceUniqueNames = false;
      super.build (args);
   }

   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);

//      addDynamicSoftPalate();
      addKinematicSoftPalate ();
      addAirwaySkinToTongue ();
      addAirwayBoundaryBodies ();
      
      if (driver.getFrame() != null) {
         driver.getFrame ().setSize (907, 597);
      }
      
      //addTrackingController ();
   }

   public void addAirwayBoundaryBodies() {
      String[][] bodyMeshNames = new String[][]{
           {"maxilla",null},
           {"jaw",null},
           {"palate",(softPalate==null?"badinpalate_morphed.obj":"badinpharyx_morphed.obj")},
           {"face","badinface.obj"}
      };
      
      for (String[] bodyMeshName : bodyMeshNames) {
         RigidBody body = addBody (bodyMeshName[0], regGeomDir+bodyMeshName[1]);
         airwaySkin.addFrame (body);
         RenderProps.setVisible (body, false);
      }
      
      airwaySkin.computeWeights ();
   }
   
   public void addAirwaySkinToTongue () {
      airwaySkin = createAirwaySkin (myJawModel, tongue, softPalate);
   }

   public static AirwaySkin createAirwaySkin() {
      AirwaySkin airwaySkin = null;
      try {
         String meshFilename = ArtisynthPath.getSrcRelativePath (StaticJawHyoidTongue.class, "geometry/badinairwaylips_morphed_n.obj");
         String textureFilename = ArtisynthPath.getSrcRelativePath (StaticJawHyoidTongue.class, "geometry/lips.jpg");
         PolygonalMesh mesh = new PolygonalMesh (new File (meshFilename));
         mesh.triangulate ();
         mesh.scale (1000);
         airwaySkin = new AirwaySkin(mesh);
         
         RenderProps props = airwaySkin.getRenderProps ();
         props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);
         props.setFaceColor (Color.WHITE);
         props.setShading(Renderer.Shading.SMOOTH);
         ColorMapProps tprops = new ColorMapProps();
         tprops.setFileName(textureFilename);
         tprops.setEnabled(true);
         tprops.setColorMixing(Renderer.ColorMixing.MODULATE);
         props.setColorMap (tprops);
         
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      
      return airwaySkin;
      
   }
   

   public static AirwaySkin createAirwaySkin (MechModel mech, FemMuscleModel tongue, FemMuscleModel softPalate) {
      AirwaySkin airwaySkin = createAirwaySkin ();
      if (tongue != null) {
         airwaySkin.addFemModel (tongue);
      }
      if (softPalate != null) {
         airwaySkin.addFemModel (softPalate);
      }
      airwaySkin.computeWeights ();
      mech.addMeshBody (airwaySkin);
      return airwaySkin;
   }
   
   public void addKinematicSoftPalateSurface() {
      PolygonalMesh spUp = null;
      PolygonalMesh spDn = null;
      try {
         spUp = new PolygonalMesh (regGeomDir + "softpalate_up.obj");
         spDn = new PolygonalMesh (regGeomDir + "softpalate_superdown.obj");
      }
      catch (IOException e) {
         e.printStackTrace();
      }
         
      MeshBlendController meshBlender = new MeshBlendController (spUp, spDn);
      addController (meshBlender);

      PolygonalMesh softPalate = meshBlender.getBlendedMesh ();
      MeshComponent comp = new MeshComponent ("softPalateSurf");
      comp.setMesh (softPalate);
      softPalate.setFixed (false);
      RenderProps.setFaceColor (comp, new Color(0.7f,0.6f,0.6f));
      myJawModel.addMeshBody (comp);
      
      addBlendFactorProbe (meshBlender);

   }

   public void addKinematicSoftPalate() {
      softPalate = SoftPalateModel.createSoftPalate ();
      myJawModel.addModel (softPalate);
//      SoftPalateModel.anchorSoftPalate (softPalate, tongue, myJawModel);
      RenderProps.setVisible (softPalate.getNodes (), false);
      RenderProps.setVisible (softPalate.markers (), false);
      softPalate.setElementWidgetSize (1);
      
      FemBlendController femBlender = new FemBlendController (softPalate, 
         new File(regGeomDir+"sp_up_noattach.bin"), new File(regGeomDir+"sp_down_noattach.bin"));
      addController (femBlender);
      femBlender.apply (0, 0); // set in "up" posture
      softPalate.resetRestPosition ();
      for (FemNode n : softPalate.getNodes ()) {
         n.setDynamic (false);
      }

      addBlendFactorProbe (femBlender);
      
      // change out palate surface for pharyx since we have soft-palate now:
//      RigidBody pharyx = myJawModel.rigidBodies ().get ("palate");
//      String filename = regGeomDir+"badinpharyx_morphed_mm.obj";
//      try {
//         pharyx.setMesh (new PolygonalMesh (filename), filename);
//         airwaySkin.computeWeights ();
//      }
//      catch (IOException e) {
//         e.printStackTrace();
//      }

   }
   
   public static final RigidTransform3d XSoftPalate = 
      new RigidTransform3d (
         new Vector3d(-7.2117, 0.0, 12.08155),
         new AxisAngle (0, 1, 0, 0.106345));
      
   public FemModel3d addDynamicSoftPalate() {
      softPalate = SoftPalateModel.createSoftPalate ();
      myJawModel.addModel (softPalate);
      SoftPalateModel.anchorSoftPalate (softPalate, tongue, myJawModel);
      RenderProps.setVisible (softPalate.getNodes (), false);
      RenderProps.setVisible (softPalate.markers (), false);
      softPalate.setElementWidgetSize (1);
      
      // compute manual registration
//      try {
//         PolygonalMesh targetSurface = new PolygonalMesh (regGeomDir+"softpalate_init.obj");
//         PolygonalMesh currentSurface = softPalate.getSurfaceMesh ();
//         SurfaceRegistration reg = new SurfaceRegistration (currentSurface, targetSurface);
//         RigidTransform3d X = reg.calcReg ();
//         softPalate.transformGeometry (X);
//         System.out.println ("X="+X.toString ("%g"));
//      }
//      catch (IOException e) {
//         e.printStackTrace();
//      }
      
      softPalate.transformGeometry (XSoftPalate);
      
      return softPalate;
   }
   
   
   public void addBlendFactorProbe(Controller con) {
      NumericInputProbe ip = new NumericInputProbe (con, "blendFactor", 0, 1);
      ip.addData (new double[]{1, 0, 1}, 0.5);
      ip.setInterpolationOrder (Order.Cubic);
      addInputProbe (ip);
   }
   

}
