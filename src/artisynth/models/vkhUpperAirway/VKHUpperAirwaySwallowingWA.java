package artisynth.models.vkhUpperAirway;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLGridResolution;
import maspack.render.GL.GLViewer;
import maspack.render.Dragger3d.DraggerType;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.widgets.DoubleFieldSlider;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.TetGenReader;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.AxialMuscleMaterial;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.LinearAxialMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.template.ModelTemplate;

public class VKHUpperAirwaySwallowingWA extends ModelTemplate {
   private class ClosestInfo {
      Object closest;
      double distance;
      Point3d newPos;
   }
   
   protected FemModel3d myAirway;

   public VKHUpperAirwaySwallowingWA () {
   }

   public VKHUpperAirwaySwallowingWA (String name) throws IOException {
      super (name);
      
      //#####################################################################
      // CONTROLS
      super.debug = false;
      super.SetAutoAttach = true;
      super.SetCollision = true;
      super.IncompressOption = false;
      super.useElementsInsteadOfFibres = false;
      super.fiberDefinedAcrossSets = true;
      super.groupExciters = true;
      
      super.rigidBodyPath = ArtisynthPath.getSrcRelativePath (
         VKHUpperAirwaySwallowingWA.class, "geometry/rigidBodies/");
      super.femPath = ArtisynthPath.getSrcRelativePath (
	         VKHUpperAirwaySwallowingWA.class, "geometry/fem/");
      super.otherPath = ArtisynthPath.getSrcRelativePath (
         VKHUpperAirwaySwallowingWA.class, "geometry/other/");
      
      super.bodyListAllFilename = "bodyList.txt";
      super.femListAllFilename = "femList_Airway.txt";
      super.femBundleSpringListFilename = "femBundleSpringList_individualBundleControl.txt";
      super.autoAttachListFilename = "autoAttachList.txt";
      super.collisionListFilename = "collision.txt";
      super.workingDirname = "src/artisynth/models/vkhUpperAirway/data";;
      //super.probesFilename = "swallowing_activations_individualBundleControl_Output.txt";
      super.probesFilename = "swallowing_activations_individualBundleControl_Input.txt";
      //#####################################################################
      
      // Step size
      super.MAX_STEP_SIZE_SEC = 10.0e-3; // 1 msec
      super.COLLISION_FRICTION_COEFF = 0.0;
      
      //super.GRAVITY = 9800;
      //[Ward2005]
      super.MUSCLE_DENSITY = 1.112E-6; 
      //[Ogneva2010:Transversal Stiffness and Young's Modulus of Single Fibers 
      //        from Rat Soleus Muscle Probed by Atomic Force Microscopy]
      super.FEM_MATERIAL = new LinearMaterial(24.7,0.47);
      //super.FEM_MATERIAL = new MooneyRivlinMaterial(1.037,0,0,0.486,0,10.370);
      super.MUSCLE_FORCE_SCALING = 1000;
      super.SPRING_MUSCLE_FORCE_SCALING = 1000;
      super.MUSCLE_MAX_FORCE = 5;
      super.MUSCLE_FIBRE_TYPE = "Peck";
      super.MUSCLE_MATERIAL = new BlemkerMuscle();
      ((BlemkerMuscle)super.MUSCLE_MATERIAL).setMaxStress (MUSCLE_MAXSTRESS);
      ((BlemkerMuscle)super.MUSCLE_MATERIAL).setMaxLambda(MUSCLE_MAXLAMBDA);
      ((BlemkerMuscle)super.MUSCLE_MATERIAL).setExpStressCoeff(0.00005);
      
      // Display and Rendering
      super.muscleControlPanel = true;
      super.drawBundle = true;
      super.drawBundleExcitation = true;
      super.drawNodes = false;
      super.drawAttachedNodes = false;
      super.pointRadius = 0.5;
      super.lineWidth = 1;
      super.lineRadius = 0.5;
      super.muscleShaded = true;
      super.elementWedgeSize = 1.0;
      super.includeWayPoints = true;
      super.wayPointStep = 0.02;
      super.stopPoint = 5.0d;
      
      createModel();
      myAirway = setupAirway();
      
      //tempSprings();
      tongueSprings();
      //addBolus();
   }
   public void tongueSprings() {
      double[][] tonguePoints = {
                                 {-129, 150.81827, -172.87491},
                                 {-129, 145.79293, -171.03006},
                                 {-129, 139.81829, -169.64685},
                                 {-129, 133.55621, -170.17975},
                                 {-129, 128.71073, -171.69885},
                                 {-129, 123.67795, -173.95324},
                                 {-129, 120.20648, -176.51514},
                                 {-129, 117.9292, -178.38451},
                                 {-129, 115.81805, -180.20967},
                                 {-129, 108.1578, -186.32036},
                                 {-129, 109.86305, -200.96668}
      };
      double[][] refPoints = {
                                 {-129, 156.32572, -164.71445},
                                 {-129, 150.79072, -157.02943},
                                 {-129, 139.71501, -149.74425},
                                 {-129, 125.06176, -151.28876},
                                 {-129, 119.85255, -153.36612},
                                 {-129, 113.9703, -156.37404},
                                 {-129, 109.2906, -159.46403},
                                 {-129, 105.42112, -163.76448},
                                 {-129, 101.84447, -166.98332},
                                 {-129, 117.5491, -186.35368},
                                 {-129, 117.5491, -206.35368}
      };
      FemModel3d tongue = (FemModel3d)myMechMod.models ().get ("Tongue");
      RigidBody ref = myMechMod.rigidBodies ().get ("ref_block");
      FemMarker marker1;
      FrameMarker marker2;
      Muscle spring;
      ControlPanel panel = new ControlPanel ("Tongue Muscle Controls", "LiveUpdate");
      for(int i=0;i<refPoints.length;i++) {
         marker1 = new FemMarker(tonguePoints[i][0],tonguePoints[i][1],tonguePoints[i][2]);
         tongue.addMarker (marker1);
         marker2 = new FrameMarker(ref, refPoints[i][0], refPoints[i][1], refPoints[i][2]);
         myMechMod.addFrameMarker (marker2);
         spring = new Muscle(marker1,marker2);
         spring.setName("Tongue_" + i);
         AxialMuscleMaterial mat = new LinearAxialMuscle();
         mat.setOptLength(spring.getLength());
         mat.setMaxForce (10);
         spring.setMaterial (mat);
         RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
         RenderProps.setLineRadius (spring, lineRadius*1);
         RenderProps.setLineColor (spring, Color.WHITE);
         spring.setExcitationColor(Color.RED);
         
         myMechMod.addAxialSpring(spring);
         
         DoubleFieldSlider slider =
            (DoubleFieldSlider)panel.addWidget(
               "Tongue_"+i, myMechMod, "axialSprings/" + "Tongue_"+i +
                     ":excitation", 0.0, 1.0);
      }
      
      marker1 = new FemMarker(-129, 140.14096, -188.17038);
      FemMarker marker3 = new FemMarker(-129, 120.14096, -188.17038);
      tongue.addMarker (marker1);
      tongue.addMarker (marker3);
      spring = new Muscle(marker1,marker3);
      spring.setName("Tongue_inner");
      AxialMuscleMaterial mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*1);
      myMechMod.addAxialSpring(spring);
      
      DoubleFieldSlider slider =
         (DoubleFieldSlider)panel.addWidget(
            "Tongue_inner", myMechMod, "axialSprings/" + "Tongue_inner" +
                  ":excitation", 0.0, 1.0);

      addControlPanel (panel);
   }
   
   public void tempSprings() {
      FemModel3d tongue = (FemModel3d)myMechMod.models ().get ("Tongue");
      RigidBody ref = myMechMod.rigidBodies ().get ("ref_block");
      FemMarker marker1 = new FemMarker(-129.513, 120.14096, -188.17038);
      tongue.addMarker (marker1);
      
      FrameMarker marker2 = new FrameMarker(ref, -129.513, 120.14096, -168.17038);
      myMechMod.addFrameMarker (marker2);
      Muscle spring;
      spring = new Muscle(marker1,marker2);
      spring.setName("Tongue_up");
      AxialMuscleMaterial mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*2);
      myMechMod.addAxialSpring(spring);
      
      marker2 = new FrameMarker(ref, -129.513, 120.14096, -208.17038);
      myMechMod.addFrameMarker (marker2);
      spring = new Muscle(marker1,marker2);
      spring.setName("Tongue_down");
      mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*2);
      myMechMod.addAxialSpring(spring);
      
      marker2 = new FrameMarker(ref, -129.513, 100.14096, -188.17038);
      myMechMod.addFrameMarker (marker2);
      spring = new Muscle(marker1,marker2);
      spring.setName("Tongue_back");
      mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*2);
      myMechMod.addAxialSpring(spring);

      marker2 = new FrameMarker(ref, -129.513, 140.14096, -188.17038);
      myMechMod.addFrameMarker (marker2);
      spring = new Muscle(marker1,marker2);
      spring.setName("Tongue_front");
      mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*2);
      myMechMod.addAxialSpring(spring);
      
      /*
      marker1 = new FemMarker(-129.513, 141.72322, -171.18339);
      tongue.addMarker (marker1);
      marker2 = new FrameMarker(ref, -129.513, 141.72322, -157.81629);
      myMechMod.addFrameMarker (marker2);
      spring = new Muscle(marker1,marker2);
      spring.setName("Tongue_tip_up");
      mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*3);
      myMechMod.addAxialSpring(spring);
      
      marker2 = new FrameMarker(ref, -129.513, 131.3335, -161.77106);
      myMechMod.addFrameMarker (marker2);
      spring = new Muscle(marker1,marker2);
      spring.setName("Tongue_tip_back");
      mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*3);
      myMechMod.addAxialSpring(spring);
      */
      /*
      marker1 = new FemMarker(-129.513, 119.86772, -177.91244);
      tongue.addMarker (marker1);
      marker2 = new FrameMarker(ref, -129.513, 128.38371, -206.09216);
      myMechMod.addFrameMarker (marker2);
      spring = new Muscle(marker1,marker2);
      spring.setName("Tongue_00");
      mat = new LinearAxialMuscle();
      mat.setOptLength(spring.getLength());
      mat.setMaxForce (10);
      spring.setMaterial (mat);
      RenderProps.setLineStyle(spring, LineStyle.SPINDLE);
      RenderProps.setLineRadius (spring, lineRadius*3);
      myMechMod.addAxialSpring(spring);
      */
   }
   public void addBolus() {
      try {
         String filename = "sphere2.1";
         FemMuscleModel fem = new FemMuscleModel ("bolus");
         TetGenReader.read (
            fem, 1, femPath + filename + ".node", femPath + filename + ".ele",
            new Vector3d(6,8,4));
         fem.transformGeometry (new RigidTransform3d(-129, 120, -165));
         fem.setDensity (0.00001);
         fem.setMaterial (new LinearMaterial(10,0.47));
         
         RenderProps.setFaceColor (fem, Color.GREEN);
         if(muscleShaded && elementWedgeSize==0) {
            fem.setSurfaceRendering (SurfaceRender.Shaded);
         } else {
            if(muscleShaded && elementWedgeSize>0) {
               fem.setElementWidgetSize(elementWedgeSize);
            }
            fem.setSurfaceRendering (SurfaceRender.None);
         }
         RenderProps.setFaceStyle (fem, Renderer.FaceStyle.FRONT);
         RenderProps.setShading (fem, Renderer.Shading.FLAT);
         
         myMechMod.addModel (fem);
         
         myMechMod.setCollisionBehavior (fem, 
            (Collidable)myMechMod.models ().get ("Tongue"), true);
         myMechMod.setCollisionBehavior (fem, 
            (Collidable)myMechMod.models ().get ("Pharynx"), true);
         myMechMod.setCollisionBehavior (fem, 
            (Collidable)myMechMod.models ().get ("Palatine_aponeurosis"), true);
      }
      catch (Exception e) {
         e.printStackTrace ();
      }
   }
   public void attach(DriverInterface driver)
   {
      super.attach(driver);
      setSagittalView(0.0);
   }
   public void setSagittalView(double gridOffset) {
      GLViewer v = Main.getMain().getViewer();
      
      //vc.autoFit();
      v.setAxialView(AxisAlignedRotation.Y_Z);
      
      if (v.getNumClipPlanes() < 1) {
         v.addClipPlane();
      }
      GLClipPlane clip  = v.getClipPlane (0);
      
      clip.setResolution(new GLGridResolution(100,10));
      clip.setPosition(getCenter());
      clip.setOrientation(new AxisAngle (0, 1, 0, Math.PI / 2));
      clip.setOffset (gridOffset);
      clip.setGridVisible (true);
      clip.setDragger (DraggerType.None);
   }
   
   public void attachToNearest(FemModel3d airway, ArrayList<FemModel3d> fems, ArrayList<RigidBody> rigids, 
      double maxDist, double reduceTol, boolean projectOut) {
      PointList<FemNode3d> nodes = airway.getNodes ();
      airway.getSurfaceMesh ();
      ClosestInfo closestFem = new ClosestInfo();
      ClosestInfo closestRigid = new ClosestInfo();
      
      for (int i=0; i<nodes.size (); i++) {
         FemNode3d n = nodes.get (i);
         Vertex3d v = airway.getSurfaceVertex (n);
         if (v== null) {
            continue;
         }
         
         findClosestFem (closestFem, v, fems, maxDist, true);
         findClosestRigid(closestRigid, v, rigids, maxDist, true);
         if (closestFem.closest != null && closestRigid.closest != null) {
            if (closestFem.distance < closestRigid.distance) {
               if (closestFem.distance < 0 && projectOut) { // node was inside, so get the projection
                  n.getPosition ().set(closestFem.newPos);
               }
               FemModel3d fem = (FemModel3d)closestFem.closest;
               myMechMod.addAttachment (fem.createPointAttachment(n,reduceTol));
            }
            else {
               myMechMod.attachPoint (n, (RigidBody) closestRigid.closest);
            }
         }
         else if (closestFem.closest != null) {
            if (closestFem.distance < 0 && projectOut) { // node was inside, so get the projection
               n.getPosition ().set(closestFem.newPos);
            }
            FemModel3d fem = (FemModel3d)closestFem.closest;
            myMechMod.addAttachment (fem.createPointAttachment(n,reduceTol));
         }
         else if (closestRigid.closest != null) {
               myMechMod.attachPoint (n, (RigidBody) closestRigid.closest);
         }
         
      }
   }

   FemModel3d setupAirway() {
      ArrayList<FemModel3d> fems = new ArrayList<FemModel3d>();
      ArrayList<RigidBody> rigids = new ArrayList<RigidBody>();
      FemModel3d airway=null;
      for( int i=0; i<myMechMod.models().size (); i++ ) {
         MechSystemModel msm = myMechMod.models().get (i);
         if( msm instanceof FemModel3d ) {
            if (!((FemModel3d) msm).getName ().equalsIgnoreCase ("Airway")) {
               fems.add ((FemModel3d) msm);
            }
            else {
               airway = ((FemModel3d) msm);
            }
         }
      }
      
      
      // Actually we only want to attach to maxilla
      RigidBody maxilla = myMechMod.rigidBodies ().get("Maxilla");
      if( maxilla != null ) {
         rigids.add ( maxilla );
      } 
      else {
         System.out.println("Could not find Maxilla for airway attachment!");
      }
      
//      double maxDistance = Double.POSITIVE_INFINITY;
      //double maxDistance = 10;
      double maxDistance = 3;
      boolean projectOut = false;
      double reduceTol = 1e-7;
      System.out.println (fems.size() + " FEMs to search for airway attachment");
      System.out.println (rigids.size() + " rigids to search for airway attachment");
      System.out.println (maxDistance + " mm set for max distance");
      
      //SkinMeshMulti smm = attachAirway(super.femPath + "Airway-final.obj", fems, rigids, maxDistance);
      attachToNearest (airway, fems, rigids, maxDistance, reduceTol, projectOut);
//      airway.setYoungsModulus (1e-15);
//      airway.setPoissonsRatio (0);
      airway.setLinearMaterial (1e-15, 0, true);
      
      return airway;
   }
   
   public ClosestInfo findClosestRigid (ClosestInfo ret, Vertex3d vtx, ArrayList<RigidBody> rigids, double maxDist, boolean checkNormal) {
      RigidBody closest = null;
      double minDist = maxDist;
      Vector2d coords = new Vector2d();
      Point3d newLoc = new Point3d();
      
      for (int i=0; i<rigids.size(); i++) {
         RigidBody rigid = rigids.get (i);
         BVFeatureQuery.getNearestFaceToPoint (
            newLoc, coords, rigid.getMesh(), vtx.getPosition ());
         //obbt.nearestFace (vtx.getPosition (), null, newLoc, coords, new TriangleIntersector());
         double d = vtx.getPosition().distance (newLoc);
         
         if( d < minDist ) {
            minDist = d;
            closest = rigid;
         }
      }
      
      ret.closest = (Object) closest;
      ret.distance = minDist;
      return ret;
   }
   
   public ClosestInfo findClosestFem (ClosestInfo ret, Vertex3d vtx, ArrayList<FemModel3d> fems, double maxDist, boolean checkNormal) {
      double minDist = maxDist;
      FemModel3d fem = null;
      Point3d newLoc = new Point3d();
      Point3d tmpPnt = new Point3d();
      
      Vector3d vNorm = new Vector3d();
      Vector3d tmp = new Vector3d();
      vtx.computeNormal (vNorm);
      /*
       * search fem list for nearest element.
       * attach to fem which is closest (or 
       * which this vtx lies inside).
       */
      for(FemModel3d cfem : fems) {
         FemElement3dBase celem = cfem.findContainingElement (vtx.pnt);
         
         if (celem == null) {
            celem = cfem.findNearestSurfaceElement (newLoc, vtx.pnt);
            tmp.sub (newLoc, vtx.pnt);
            double d = newLoc.distance (vtx.pnt);
            if( minDist > d ) {
               if (!checkNormal || tmp.dot (vNorm) > 0) {
                  minDist = d;
                  fem = cfem;
               }
            }
         }
         else {
            // The vertex is in a FEM, project it to surface and check the normal is correct
            fem = cfem;
            celem = cfem.findNearestSurfaceElement (newLoc, vtx.pnt);
            tmp.sub (newLoc, vtx.pnt);
            
            // Check that we're projecting the right way (against the normal)
            double multiply = 1;
            while (checkNormal && tmp.dot (vNorm) > 0) {
               tmpPnt.set(vtx.pnt);
               tmp.scale(multiply);
               tmpPnt.sub (tmp);
               celem = cfem.findNearestSurfaceElement (newLoc, tmpPnt);
               tmp.sub (newLoc, vtx.pnt);
               multiply++;
            }
            
            
            minDist = -newLoc.distance (vtx.pnt);
            
            break;
         }
         
      }
      
      ret.closest = (Object) fem;
      ret.distance = minDist;
      ret.newPos = newLoc;
      
      return ret;
   }

}
