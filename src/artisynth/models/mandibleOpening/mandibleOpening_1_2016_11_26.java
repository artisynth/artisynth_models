package artisynth.models.mandibleOpening;

import artisynth.core.workspace.RootModel;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SphericalJoint;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.template.ModelTemplate;

import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;

 

public class mandibleOpening_1_2016_11_26 extends ModelTemplate{

   public mandibleOpening_1_2016_11_26 () {
   }
   
   
   public void build(String[] args) throws IOException {

      super.build(args);      
      
      // turn on debug mode
      super.debug = true;
      
      super.MAX_STEP_SIZE_SEC = 0.001;
      super.COLLISION_FRICTION_COEFF = 0.0;
      super.SetAutoAttach = true;
      super.includeWayPoints = true;
      super.wayPointStep = 0.01;
      super.stopPoint = 2.4;

      super.drawNodes = true;
      super.drawAttachedNodes = true;
      super.drawBundleExcitation = true;
      super.drawBundle = true;
      super.lineRadius = 1.0;
      super.pointRadius = 0.2;
      super.BODY_FRAME_DAMPING = 0.07;

      super.BODY_ROTARY_DAMPING = 0.07;

      super.muscleControlPanel = true;
      super.groupExciters = false;
      super.SetCollision = true;
      super.IncompressOption = false;
  //    public boolean useProbes = true;
      // Gravity and other properties_from_Ling_VHLarynxDemo
      // super.BODY_DENSITY = 0.00000347;
      // super.BODY_DENSITY = 0.00000172;
      // previous obturator: super.BODY_DENSITY = 0.00000102;
      // 2013_12_11 super.BODY_DENSITY = 0.00000150;
      super.BODY_DENSITY = 0.00000172;
      // super.BODY_DENSITY = 1720;
      // super.MUSCLE_DENSITY = 0.00000106;
      super.MUSCLE_PARTICLE_DAMPING = 40;
      super.MUSCLE_STIFFNESS_DAMPING = 0.0005;
      // super.MUSCLE_YOUNGS_MODULUS = 10;
      // super.MUSCLE_MAXLAMBDA = 2.1;
      // super.MUSCLE_MAXSTRESS = 300;
      // super.MUSCLE_MAX_FORCE = 200;
      // super.MUSCLE_FORCE_SCALING = 1000;
     // super.MUSCLE_DAMPING = 0.01;
      // super.MUSCLE_PASSIVE_FRACTION = 0.015;
      // arbitrary super.MUSCLE_PASSIVE_FRACTION = 0.15;
      // super.COLLISION_FRICTION_COEFF = 0.0;
      super.SPRING_MUSCLE_MAX_FORCE = 50;
      super.SPRING_MUSCLE_FORCE_SCALING = 1000;
      // original super.SPRING_MUSCLE_PASSIVE_FRACTION = 0.015;
      super.SPRING_MUSCLE_PASSIVE_FRACTION = 0.015;
      // super.SPRING_MUSCLE_PASSIVE_FRACTION = 0.3;
      super.SPRING_MUSCLE_TENDON_RATIO = 0.5;
      // super.SPRING_MUSCLE_TENDON_RATIO = 0.0;
      // super.FEM_MATERIAL = new LinearMaterial(13700000, 0.3);
      // super.FEM_MATERIAL = new LinearMaterial(100000, 0.3);
      // super.FEM_MATERIAL = new LinearMaterial(9700, 0.3);
      // super.FEM_MATERIAL = new LinearMaterial(1370, 0.3);
   //   super.GRAVITY = 9800;
  //    myMechMod.setGravity(0, 0, -9800);
      // super.GRAVITY = 0;
   //   GRAVITY.scale(9800.0);
      // Passive Spring Properties
   //    super.SPRING_STIFFNESS = 9400;
      // super.SPRING_STIFFNESS = 20;
  //    super.SPRING_DAMPING = 0.04;

      // rigid body
      super.rigidBodyPath = ArtisynthPath.getSrcRelativePath(
            this, "mandibleOpening_Rigidbody/");
      System.out.println("Rigid body path: " + rigidBodyPath);

      super.bodyListAllFilename = "mandibleOpening_rigidbodylist.txt";

 //2016_11_26     super.femPath =
      //2016_11_26             ArtisynthPath.getSrcRelativePath(
      //2016_11_26                this, "obturatorGravity_1/obturatorGravity_fem/");
      //2016_11_26          super.femListAllFilename = "obturatorGravity_femBodylist.txt";

      // super.FEM_MATERIAL = new LinearMaterial(1960000, 0.30);

 super.otherPath = ArtisynthPath.getSrcRelativePath(this, "mandibleOpening_Others/");
          System.out.println("otherPath: " + otherPath);
      // super.femPropertyListFilename ="SY_Obturator_7_femPropertiesList.txt";
   super.frameMarkerListFilename = "mandibleOpening_frameMarkers.txt";
 //  RenderProps.setVisible(myMechMod.frameMarkers(), drawAttachedNodes);
     super.muscleSpringListFilename = "mandibleOpening_muscleSpringlist.txt";
    super.muscleSpringPropertyListFilename = "mandibleOpening_muscleSpringProperties.txt";



      /* Passive Springs */
    super.springListFilename = "mandibleOpening_springList.txt";
   super.springPropertyListFilename =
        "mandibleOpening_springProperties.txt";
   super.collisionListFilename = "mandibleOpening_collision.txt";
   //2016_11_26    super.femPropertyListFilename = "obturatorGravity_femPropertiesList.txt";
    //2016_11_26   super.autoAttachListFilename = "obturatorGravity_Autoattachlist.txt";
      // super.otherPath = rigidBodyPath;

      // super.frameMarkerListFilename = "JI_Rigid_spring_7_frameMarkers.txt";

      // super.muscleSpringListFilename =
      // "JI_Rigid_spring_7_muscleSpringlist.txt";


 

 
//    super.workingDirname = ArtisynthPath.getSrcRelativePath(
//       this, "mandibleOpening_Probes/");
 
 super.workingDirname = ArtisynthPath.getSrcRelativePath(this, "mandibleOpening_Probes/");
 super.probesPath = ArtisynthPath.getSrcRelativePath(this, "mandibleOpening_Probes/");
 super.probesFilename = "mandibleOpening.txt";
 
 
 
      createModel();

    

      }


}