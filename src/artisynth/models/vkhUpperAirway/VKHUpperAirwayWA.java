package artisynth.models.vkhUpperAirway;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLGridResolution;
import maspack.render.GL.GLViewer;
import maspack.render.Dragger3d.DraggerType;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.template.ModelTemplate;

public class VKHUpperAirwayWA extends ModelTemplate {
   private class ClosestInfo {
      Object closest;
      double distance;
      Point3d newPos;
   }

   FemModel3d myAirway;
   
   public VKHUpperAirwayWA () {
   }

   public VKHUpperAirwayWA (String name) throws IOException {
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
         VKHUpperAirwayWA.class, "geometry/rigidBodies/");
      super.femPath = ArtisynthPath.getSrcRelativePath (
	         VKHUpperAirwayWA.class, "geometry/fem/");
      super.otherPath = ArtisynthPath.getSrcRelativePath (
         VKHUpperAirwayWA.class, "geometry/other/");
      
      super.bodyListAllFilename = "bodyList.txt";
      super.femListAllFilename = "femList_Airway.txt";
      super.femBundleSpringListFilename = "femBundleSpringList.txt";
      super.autoAttachListFilename = "autoAttachList.txt";
      super.collisionListFilename = "collision.txt";
      super.workingDirname = "src/artisynth/models/vkhUpperAirway/data";;
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
      super.drawBundleExcitation = false;
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
      
//      debug = true;
      createModel();
      debug = false;
      
      myAirway = setupAirway();
      
      MechModel myMechModel=(MechModel) this.models ().get(0);
      System.out.println("PenetrationTol is: " + myMechModel.getPenetrationTol());
      myMechModel.setPenetrationTol(0.2);
      
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
            myMechMod.addAttachment (fem.createPointAttachment(n, reduceTol));
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
//   void setupAirway() {
//      ArrayList<FemModel3d> fems = new ArrayList<FemModel3d>();
//      ArrayList<RigidBody> rigids = new ArrayList<RigidBody>();
//      FemModel3d airway=null;
//      //for( MechSystemModel msm : myMechMod.models() ) {
//      for( int i=0; i<myMechMod.models().size (); i++ ) {
//         MechSystemModel msm = myMechMod.models().get (i);
//         if( msm instanceof FemModel3d ) {
//            if (!((FemModel3d) msm).getName ().equalsIgnoreCase ("Airway")) {
//               fems.add ((FemModel3d) msm);
//            }
//            else {
//               airway = ((FemModel3d) msm);
//            }
//         }
//      }
//      
//      // Actually we only want to attach to maxilla
//      RigidBody maxilla = myMechMod.rigidBodies ().get("Maxilla");
//      if( maxilla != null ) {
//         rigids.add ( maxilla );
//      } 
//      else {
//         System.out.println("Could not find Maxilla for airway attachment!");
//      }
//      
////      double maxDistance = Double.POSITIVE_INFINITY;
//      //double maxDistance = 10;
//      double maxDistance = 3;
//      double reduceTol = 1e-7;
////      System.out.println (fems.size() + " FEMs to search for airway attachment");
////      System.out.println (rigids.size() + " rigids to search for airway attachment");
//      System.out.println (maxDistance + " mm set for max distance");
//      
//      //SkinMeshMulti smm = attachAirway(super.femPath + "Airway-final.obj", fems, rigids, maxDistance);
//      attachToNearest (airway, fems, rigids, maxDistance, reduceTol);
//      
//   }
   
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
   
   public SkinMeshMulti attachAirway(String filename, ArrayList<FemModel3d> fems, ArrayList<RigidBody> rigids, double maxDist ){
      PolygonalMesh airway = null;
      try {
         airway = new PolygonalMesh (new File(filename) );
      }
      catch (Exception e) {
         e.printStackTrace();
         System.exit(1);
      }
      /*
       * TODO: fix this in the real mesh
       */
      airway.scale (0.5, 0.5, 0.5);
      //airway.isFixed = false;
      
      
      if (debug) 
         System.out.println("Num nodes in airway: " + airway.numVertices ());
      
      double reduceTol = 1e-7;
      
      SkinMeshMulti smm = new SkinMeshMulti( airway );
      smm.computeWeights (fems, rigids, maxDist, reduceTol);
      myMechMod.addMeshBody (smm);
      
      
      return smm;
   }
   
//   public void attachToNearest(FemModel3d airway, ArrayList<FemModel3d> fems, ArrayList<RigidBody> rigids, 
//      double maxDist, double reduceTol) {
//      PointList<FemNode3d> nodes = airway.getNodes ();
//      airway.getSurfaceMesh ();
//      ClosestInfo closestFem = new ClosestInfo();
//      ClosestInfo closestRigid = new ClosestInfo();
//      
//      for (int i=0; i<nodes.size (); i++) {
//         FemNode3d n = nodes.get (i);
//         FemMeshVertex v = airway.getSurfaceMeshVertex (n);
//         if (v== null) {
//            continue;
//         }
//         
//         findClosestFem (closestFem, v, fems, maxDist, true);
//         findClosestRigid(closestRigid, v, rigids, maxDist, true);
//         if (closestFem.closest != null && closestRigid.closest != null) {
//            if (closestFem.distance < closestRigid.distance) {
//               if (closestFem.distance < 0) { // node was inside, so get the projection
//                  n.getPosition ().set(closestFem.newPos);
//               }
//               myMechMod.attachPoint (n, (FemModel3d) closestFem.closest, reduceTol);
//            }
//            else {
//               myMechMod.attachPoint (n, (RigidBody) closestRigid.closest);
//            }
//         }
//         else if (closestFem.closest != null) {
//            if (closestFem.distance < 0) { // node was inside, so get the projection
//               n.getPosition ().set(closestFem.newPos);
//            }
//            myMechMod.attachPoint (n, (FemModel3d) closestFem.closest, reduceTol);
//         }
//         else if (closestRigid.closest != null) {
//               myMechMod.attachPoint (n, (RigidBody) closestRigid.closest);
//         }
//         
//      }
//   }
   
   public ClosestInfo findClosestRigid (ClosestInfo ret, Vertex3d vtx, ArrayList<RigidBody> rigids, double maxDist, boolean checkNormal) {
      RigidBody closest = null;
      double minDist = maxDist;
      Vector2d coords = new Vector2d();
      Point3d newLoc = new Point3d();
      
      for (int i=0; i<rigids.size(); i++) {
         RigidBody rigid = rigids.get (i);
         BVFeatureQuery.getNearestFaceToPoint (
            newLoc, coords, rigid.getMesh(), vtx.getPosition());
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
