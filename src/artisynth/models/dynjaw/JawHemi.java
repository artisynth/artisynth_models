package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JFrame;

import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.ColorMapProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.ColorMixing;
import maspack.spatialmotion.SpatialInertia;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.materials.RotAxisFrameMaterial;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

public class JawHemi extends JawLarynxDemo
{
   protected boolean leftSideGraft = true;
   protected double defaultScarK = 800.0;

   Vector3d scarStiffness = new Vector3d();
   Vector3d scarForce = new Vector3d();
   ArrayList<PlanarSpring> scar = new ArrayList<PlanarSpring>(0);
   
   public JawHemi()
   {
      super();
   }
        
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      String jawreconMesh = "postcan.obj";
   // "jawRecon" + (leftSideGraft?"Left":"Right") + ".obj";

      setJawReconMesh(jawreconMesh);
      doReconstruction();
      fixHyoid();
      removeJoint(leftSideGraft);
      
      // set bolus inactive
      ((FoodBolus)myJawModel.forceEffectors ().get (
         "rightbolus")).setActive (false);
      ((FoodBolus)myJawModel.forceEffectors ().get (
         "leftbolus")).setActive (false);
      
      // hide all planar constraints as they obstruct graft
//      for (BodyConnector c : myJawModel.bodyConnectors())
//      {
//         if (c.getClass().isAssignableFrom(PlanarConnector.class))
//            RenderProps.setVisible(c, false);
//      }
   }
   
   protected void loadModel()
   {
      super.loadModel();
      
      // remove controls for tmj planes
      pokesToControl = new String[]{
           };
   }
   
   public void setJawReconMesh(String meshFilename)
   {
      RigidBody jaw = myJawModel.rigidBodies().get("jaw");
      
      myJawModel.setBodyMesh(jaw, meshFilename);

      // reset jaw inertia
      double density = jaw.getMass () / jaw.getMesh ().computeVolume ();
      jaw.setInertia (jaw.getMesh().createInertia (density));      
      
      // setup render and texture props
      RenderProps p = new RenderProps(jaw.getRenderProps());
      p.setFaceColor (new Color(1f, 1f, 0.75f));
      ColorMapProps tp = new ColorMapProps();
      tp.setEnabled (false);
      tp.setColorMixing (ColorMixing.MODULATE);
      //tp.setSphereMappingEnabled (false);
      //tp.setAutomatic (false);
      String name = ArtisynthPath.getSrcRelativePath (
         JawModel.class, "skull.jpg");
      System.out.println(name);
      tp.setFileName(name);
      p.setColorMap(tp);
      jaw.setRenderProps(p);
      
   }
   
   public void fixHyoid()
   {
      String[] tofix = new String[]{"hyoid", "thyroid", "cricoid", "sternum"};
      for (String name : tofix)
      {
         RigidBody body = myJawModel.rigidBodies ().get (name);
         body.setDynamic (false);
         RenderProps.setVisible (body, false);
         
         for (FrameMarker m : myJawModel.frameMarkers ())
            if (m.getFrame ()==body)
               RenderProps.setVisible (m, false);
      }
      
//      myJawModel.setShowMembrane (false);
      BodyConnector ctjoint = myJawModel.bodyConnectors ()
         .get("cricothyroid");
      ctjoint.setEnabled (false);
      RenderProps.setVisible (ctjoint, false);
      
      RenderProps.setVisible(myJawModel.rigidBodies ().get ("hyoid"), true);

      RigidBody jaw = myJawModel.rigidBodies ().get ("jaw");
      for (AxialSpring s : myJawModel.axialSprings ())
      {
         if (s.getFirstPoint () instanceof FrameMarker && 
             s.getSecondPoint () instanceof FrameMarker)
         {
            if (((FrameMarker)s.getFirstPoint ()).getFrame ()!=jaw &&
                ((FrameMarker)s.getSecondPoint ()).getFrame ()!=jaw)
               RenderProps.setVisible (s, false);
         }
      }
   }
   
   public void enableJoint(boolean isLeftSide, boolean isEnabled)
   {
      String[] todisable = new String[]{"TMJ", "MED", "POST"};
      for (String name : todisable)
      {
         BodyConnector con = myJawModel.bodyConnectors ()
            .get ((isLeftSide?"L":"R")+name);
         con.setEnabled (isEnabled);
         RenderProps.setVisible (con, isEnabled);
      }
   }
   
   public void addJoint(boolean isLeftSide)
   {
      enableJoint (isLeftSide, true);
   }   
   
   public void removeJoint(boolean isLeftSide)
   {
      enableJoint (isLeftSide, false);
   }
   
   public void doReconstruction()
   {
      
      for (FoodBolus fb : myFoodBoluses)
      {
         fb.setResistance(30.0);
      }
      
      String disabledMuscles[] = new String[]{
         "at",
         "mt",
         "pt",
         "ip",
         "sp",
         "dm",
         "sm",
         "am",
         "pm",
         "mp"
      };
      
      for (int i = 0; i < disabledMuscles.length; i++)
      {
         String muscleName = (leftSideGraft?"l":"r")+disabledMuscles[i];
         Muscle m = (Muscle)myJawModel.axialSprings().get(muscleName);
         m.setEnabled(false);
      }
      


//      addPassiveScarTissue();
//      add3DScarTissue ();
      addScarSprings ();
   }
   
   
   public enum scarDir {
      SCAR_YZ(new RotationMatrix3d(new AxisAngle(0, 1, 0, Math.PI/2))),
      SCAR_XZ(new RotationMatrix3d(new AxisAngle(1, 0, 0, Math.PI/2))),
      SCAR_XY(new RotationMatrix3d(new AxisAngle()));      
      
      public RotationMatrix3d R = new RotationMatrix3d();
     
      private scarDir(RotationMatrix3d R)
      {  this.R.set (R);
      }
   }
   
   public void addScarSprings()
   {
      addScarSpring ("ant_scar", new Point3d(33.2801, -11.0329, 25.8175));
      addScarSpring ("med_scar", new Point3d(39.704382, 2.5010928, 28.764355));
      addScarSpring ("post_scar", new Point3d(45.1543, 13.0315, 33.4041));
   }
   
   public void addScarSpring(String name, Point3d loc)
   {
      FrameMarker insertion_point = new FrameMarker();
      insertion_point.setName(name+"_insertion");

      RigidBody jaw = myJawModel.rigidBodies ().get ("jaw");
      RigidBody fixed = myJawModel.rigidBodies ().get ("maxilla");
      myJawModel.addFrameMarker(insertion_point, jaw, loc);
      
      RigidTransform3d X = new RigidTransform3d();
      X.p.set (loc);

      ScarSpring scar = new ScarSpring(name);
      scar.setFrameB (fixed);
      scar.setAttachFrameB (X);
      scar.setCollidingPoint (insertion_point);
      scar.setPlaneSize (10.0);
      scar.setMaterial (new RotAxisFrameMaterial (defaultScarK, 0, 0, 0));
      //scar.setStiffness (defaultScarK);
      myJawModel.addFrameSpring (scar);
      
      RenderProps.setFaceColor (scar, new Color(0.8f,0.8f,1f));
      RenderProps.setLineColor (scar, new Color(0.5f,0.5f,1f));
      RenderProps.setPointColor (insertion_point, new Color(0.2f,0.2f,1f));
   }
   
   
   public void add3DScarTissue()
   {
      FrameMarker m = new FrameMarker();
      m.setName("scar_origin");
      Point3d loc = new Point3d(40.336280, 0.79658276, 35.401429);
      if (!leftSideGraft) loc.x *= -1.0;
      myJawModel.addFrameMarker(m, myJawModel.rigidBodies().get("jaw"), loc);
      
      RigidTransform3d X = new RigidTransform3d();
      X.p.set (loc);
      
      for (scarDir d : scarDir.values ())
      {
         X.R.set (d.R);
         PlanarSpring p = new PlanarSpring(m);
         p.transformGeometry (X);
         p.setUnilateral (false);
         p.setPlaneSize (X.p.norm ()/10.0);
         scar.add (p);
         myJawModel.addForceEffector (p);
         RenderProps.setFaceColor (p, Color.PINK);
         RenderProps.setLineColor (p, Color.RED);
         RenderProps.setAlpha (p, 1.0);
      }
      setScarStiffness (new Vector3d(defaultScarK,defaultScarK,defaultScarK));
   }
   
   public void addPassiveScarTissue()
   {
      // add vertical spring to represent scar tissue
      double len = 48.0;
      double restlen = 38.0;
      double restlen_percent = len/restlen; // rest length percentage of len
      double maxforce = 5.0;
      double stiff = 100;
      Point3d loc;
      FrameMarker m;
      
//      AxialSpring scar = new AxialSpring("scartissue", stiff, 0.0, len*restlen_percent);
      Muscle scar = new Muscle();
      scar.setLinearMuscleMaterial(maxforce, restlen, len, 1.0);
      
      m = new FrameMarker();
      m.setName("scar_origin");
      loc = new Point3d(40.336280, 0.79658276, 35.401429);
      if (!leftSideGraft) loc.x *= -1.0;
      myJawModel.addFrameMarker(m, myJawModel.rigidBodies().get("jaw"), loc);
      scar.setFirstPoint(m);

      m = new FrameMarker();
      m.setName("scar_insersion");
      loc = new Point3d(51.276890, -9.3070053, 71.959810);
      if (!leftSideGraft) loc.x *= -1.0;
      myJawModel.addFrameMarker(m, myJawModel.rigidBodies().get("maxilla"), loc);
      scar.setSecondPoint(m);
      
      myJawModel.addAxialSpring(scar);
      
      RenderProps.setLineStyle(scar, LineStyle.CYLINDER);
      RenderProps.setLineRadius(scar, 
         myJawModel.getRenderProps().getLineRadius()*0.4);
      RenderProps.setLineColor(scar, Color.WHITE);
      
      
   }


   public Vector3d getScarStiffness()
   {
      return scarStiffness;
   }


   public void setScarStiffness(Vector3d scarStiffness)
   {
      this.scarStiffness = scarStiffness;
      for (int i = 0; i < scar.size (); i++)
      {
         AxialSpring.setStiffness (scar.get(i), scarStiffness.get (i));
      }
   }

   Vector3d f = new Vector3d();
   public Vector3d getScarForce()
   {
      for (int i = 0; i < scar.size (); i++)
      {
        scar.get (i).computeForce (f);
        f.negate ();
//        System.out.println("scar " + i + " = " + f.toString ());
        scarForce.set (i, f.get (i));
      }
      return scarForce;
   }
   
   
   public void loadControlPanel(RootModel root)
   {
      String panelNames[] = new String[]{
         "misc",
         "damping",
         "muscles",
         "joints",
         "scarStiffness"
      };
      loadControlPanel (root, panelNames);
   }
   
   public void attach(DriverInterface driver)
   {  
      this.setViewerEye (new Point3d(0.0, -268.0, -23.0));
      this.setViewerCenter (new Point3d(0.0, 44.0, 55.0));
      
      loadControlPanel (this);
      
//      super.attach (driver);
//      
//      endofchewBreakpoint.setTime(
//         TimeBase.milliSecondsToTicks(leftSideGraft?575:565));
      
//      if (scar.size () == 0)
//      {
//         for (ForceEffector f : myJawModel.forceEffectors ())
//         {
//            if (f instanceof PlanarSpring) // should name scar also
//            {
//               scar.add ((PlanarSpring)f);
//            }
//         }
//      }
   }
   
}
