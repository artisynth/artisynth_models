package artisynth.models.vkhUpperAirway;

import java.awt.event.ActionEvent;
import java.io.IOException;

import maspack.matrix.AxisAngle;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLGridResolution;
import maspack.render.GL.GLViewer;
import maspack.render.Dragger3d.DraggerType;
import artisynth.core.driver.Main;
import artisynth.core.driver.ViewerManager;
import maspack.matrix.AxisAlignedRotation;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.template.ModelTemplate;

public class VKHUpperAirwayDemo extends ModelTemplate {

   public VKHUpperAirwayDemo () {
   }

   public VKHUpperAirwayDemo (String name) throws IOException {
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
         VKHUpperAirwayDemo.class, "geometry/rigidBodies/");
      super.femPath = ArtisynthPath.getSrcRelativePath (
	         VKHUpperAirwayDemo.class, "geometry/fem/");
      super.otherPath = ArtisynthPath.getSrcRelativePath (
         VKHUpperAirwayDemo.class, "geometry/other/");
      
      super.bodyListAllFilename = "bodyList.txt";
      super.femListAllFilename = "femList.txt";
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
      
      createModel();
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

}
