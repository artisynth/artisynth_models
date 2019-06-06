package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.tongue3d.FemMuscleTongueDemo;
import artisynth.models.tongue3d.HexTongueDemo;

/*
 * adapted from StaticJawHyoidTongue, uses line-muscle tongue, closed jaw posture
 */
public class TongueBracingDemo extends BadinJawHyoidTongue {
   
   /*
    * set in BadinJawHyoid:  public static final boolean useTeethSmoothedGeometry = true; 
    * 
    */
   
   // jawClose posture - zero gravity, bilateral closers = 0.001000
   public static final RigidTransform3d hyoid_jawClosePosture = new RigidTransform3d(
      new Vector3d(118.45785987483865, 0, 59.69376549922591), 
      new AxisAngle (0, -0.9999999328252734, 0, 0.0019850674801515024));
   
   public static final RigidTransform3d jaw_jawClosePosture = new RigidTransform3d (
      new Vector3d(95.47028674132166, 0, 90.44985976096676), 
      new AxisAngle(0, 1.0, 0, 0.10249463781435146)); 
   
   // jawOpen posture - zero gravity, bilateral closers = 0.001000
   public static final RigidTransform3d hyoid_jawOpenPosture = new RigidTransform3d(
      new Vector3d(118.3743462452258, 0.004368422557900965, 58.6713558501195), 
      new AxisAngle (-0.007643825923054012, -0.9999598311631389, 0.00468059669750253, 0.08611399919055665));
   
   public static final RigidTransform3d jaw_jawOpenPosture = new RigidTransform3d (
      new Vector3d(98.91376378011483, -0.04846767514701015, 83.91599266410334), 
      new AxisAngle(-0.0015256003159793605, -0.999995278749738, 0.002667399840595884, 0.02624577531272179)); 
   
   
   public TongueBracingDemo() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      // fix skeletal bodies
      myJawModel.rigidBodies ().get ("jaw").setDynamic (false);
      myJawModel.rigidBodies ().get ("hyoid").setDynamic (false);
      
      // remove jaw muscles and springs
      myJawModel.clearAxialSprings ();
      myJawModel.clearFrameSprings ();
      myJawModel.clearMultiPointSprings ();
      myJawModel.getMuscleExciters ().clear ();
      
      this.setAdaptiveStepping (false);

      FemMuscleTongueDemo.addExcitersFromFiles (tongue, ArtisynthPath.getSrcRelativePath (FemMuscleTongueDemo.class, "exciters/"));

      setupRenderProps ();
   }
  
   public void setupRenderProps() {
      
      
      RenderProps.setLineWidth (tongue, 0);
//      RenderProps.setLineWidth (tongue.getMuscleBundles (), 1);
//      RenderProps.setLineStyle (tongue.getMuscleBundles (), LineStyle.LINE);
//      RenderProps.setVisible (tongue.getMuscleBundles (), false);
      HexTongueDemo.setActivationColor(tongue);

      
      CollisionManager cm = myJawModel.getCollisionManager ();
      cm.setContactNormalLen (0);
      cm.setDrawContactNormals(false);
      RenderProps.setLineWidth (cm, 3);
      RenderProps.setLineColor (cm, new Color(0f,0f,1f));
      RenderProps.setEdgeWidth (cm, 4);
      RenderProps.setEdgeColor (cm, new Color(0f,1f,1f));
      RenderProps.setPointRadius (cm, 0);
      
      
   }
   

   public void attach (DriverInterface driver) {
      super.attach (driver);
      
      // set closed jaw posture for bodies
//      HexTongueDemo.setTonguePosture (tongue, regGeomDir+"tongue_posture_jawClose_nogravity.txt", true);
//      setBodyPosture (jaw_jawClosePosture, "jaw");
//      setBodyPosture (hyoid_jawClosePosture, "hyoid");
    
      HexTongueDemo.setTonguePosture (tongue, regGeomDir+"tongue_posture_jawOpen_rest.txt", false);
      setBodyPosture (jaw_jawOpenPosture, "jaw");
      setBodyPosture (hyoid_jawOpenPosture, "hyoid");      
      
      ArtisynthPath.setWorkingDir(new File(ArtisynthPath.getSrcRelativePath (this, "data/bracing/rest2a")));
      resetInitialState();

   }
}
