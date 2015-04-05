package artisynth.models.dynjaw;

import java.io.IOException;
import java.util.ArrayList;

import maspack.matrix.Point3d;

import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentListView;

public class JawLongPharyngealDemo extends JawLarynxDemo{

   
   private String s1 = "lPalatopharyngeus maxilla thyroid  \n" +
   		"rPalatopharyngeus maxilla thyroid \n" + 
   		 "lSalpingopharyngeus cranium thyroid \n" + 
   		"rSalpingopharyngeus cranium thyroid \n" + 
   		 "lStylopharyngeus cranium thyroid \n" + 
   		"rStylopharyngeus cranium thyroid \n";
   
   /**
    * @param args
    */
   public JawLongPharyngealDemo () {
      // TODO Auto-generated constructor stub
      super();
   }

   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      //Get a list of rigid bodies in this myJawModel
      System.out.println("Hi from JawLongPharyngealDemo.build()");
      ComponentListView<RigidBody> rigidBodies = myJawModel.rigidBodies ();
      for (RigidBody rb : rigidBodies){
         System.out.println(rb.getName ());
      }
      
      //Coordinate data for the palatopharyngeus
      double [] lPalatopharyngeusOrigin = {19.44, 22.04, 55.81};
      double [] lPalatopharyngeusInsertion = {18.93, 30.01, 8.47};
      double [] rPalatopharyngeusOrigin = {-19.44, 22.04, 55.81};
      double [] rPalatopharyngeusInsertion = {-18.84, 30.07, 8.46};

      //Coordinate data for the salpingopharyngeus
      double [] lSalpingopharyngeusOrigin = {16.37, 23.66, 67.91};
      double [] lSalpingopharyngeusInsertion = {18.93, 30.01, 8.47};
      double [] rSalpingopharyngeusOrigin = {-16.37, 23.66, 67.91};
      double [] rSalpingopharyngeusInsertion = {-18.84, 30.07, 8.46};
      
      //Coordinate data for stylopharyngeus
      double [] lStylopharyngeusOrigin = {39.5132, 43.205, 65.41897};
      double [] lStylopharyngeusInsertion = {18.93, 30.01, 8.47};
      double [] rStylopharyngeusOrigin = {-39.5132, 43.205, 65.41897};
      double [] rStylopharyngeusInsertion = {-18.84, 30.07, 8.46};

      double maxForce = 15.6;
      double optLength = 73.024;
      double maxL = 93.828;
      double tendonRatio = 0.0;
      double muscleDamping = 0.0;
      
      //Add the palatopharyngeus muscles
      FrameMarker lPalatopharyngeusOriginFrameMarker = new FrameMarker ("lPalatopharyngeusOrigin"); 
      FrameMarker lPalatopharyngeusInsertionFrameMarker = new FrameMarker ("lPalatopharyngeusInsertion");
      FrameMarker rPalatopharyngeusOriginFrameMarker = new FrameMarker ("rPalatopharyngeusOrigin"); 
      FrameMarker rPalatopharyngeusInsertionFrameMarker = new FrameMarker ("rPalatopharyngeusInsertion");
      
      lPalatopharyngeusOriginFrameMarker.setFrame (rigidBodies.get ("maxilla"));
      lPalatopharyngeusInsertionFrameMarker.setFrame (rigidBodies.get ("thyroid"));
      rPalatopharyngeusOriginFrameMarker.setFrame (rigidBodies.get("maxilla"));
      rPalatopharyngeusInsertionFrameMarker.setFrame (rigidBodies.get("thyroid"));
      
      lPalatopharyngeusOriginFrameMarker.setLocation (new Point3d(lPalatopharyngeusOrigin));
      lPalatopharyngeusInsertionFrameMarker.setLocation (new Point3d(lPalatopharyngeusInsertion));
      rPalatopharyngeusOriginFrameMarker.setLocation (new Point3d(rPalatopharyngeusOrigin));
      rPalatopharyngeusInsertionFrameMarker.setLocation (new Point3d(rPalatopharyngeusInsertion));
      
      myJawModel.addFrameMarker(lPalatopharyngeusOriginFrameMarker);
      myJawModel.addFrameMarker(lPalatopharyngeusInsertionFrameMarker);
      myJawModel.addFrameMarker (rPalatopharyngeusOriginFrameMarker);
      myJawModel.addFrameMarker (rPalatopharyngeusInsertionFrameMarker);
      
      Muscle lPalatopharyngeus = new Muscle ("lPalatopharyngeus");
      lPalatopharyngeus.setFirstPoint (lPalatopharyngeusOriginFrameMarker);
      lPalatopharyngeus.setSecondPoint (lPalatopharyngeusInsertionFrameMarker);
      lPalatopharyngeus.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
      AxialSpring.setDamping (lPalatopharyngeus, muscleDamping);
      myJawModel.addAxialSpring (lPalatopharyngeus);
      Muscle rPalatopharyngeus = new Muscle ("rPalatopharyngeus");
      rPalatopharyngeus.setFirstPoint (rPalatopharyngeusOriginFrameMarker);
      rPalatopharyngeus.setSecondPoint (rPalatopharyngeusInsertionFrameMarker);
      rPalatopharyngeus.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
      AxialSpring.setDamping (rPalatopharyngeus, muscleDamping);
      myJawModel.addAxialSpring (rPalatopharyngeus);


      
      //Add the salpingopharyngeus muscles
      FrameMarker lSalpingopharyngeusOriginFrameMarker = new FrameMarker ("lSalpingopharyngeusOrigin");
      FrameMarker lSalpingopharyngeusInsertionFrameMarker = new FrameMarker ("lSalpingopharyngeusInsertion");
      FrameMarker rSalpingopharyngeusOriginFrameMarker = new FrameMarker ("rSalpingopharyngeusOrigin");
      FrameMarker rSalpingopharyngeusInsertionFrameMarker = new FrameMarker ("rSalpingopharyngeusInsertion");
      
      lSalpingopharyngeusOriginFrameMarker.setFrame (rigidBodies.get("cranium"));
      lSalpingopharyngeusInsertionFrameMarker.setFrame (rigidBodies.get("thyroid"));
      rSalpingopharyngeusOriginFrameMarker.setFrame (rigidBodies.get("cranium"));
      rSalpingopharyngeusInsertionFrameMarker.setFrame (rigidBodies.get("thyroid"));
      
      lSalpingopharyngeusOriginFrameMarker.setLocation (new Point3d(lSalpingopharyngeusOrigin));
      lSalpingopharyngeusInsertionFrameMarker.setLocation (new Point3d(lSalpingopharyngeusInsertion));
      rSalpingopharyngeusOriginFrameMarker.setLocation (new Point3d(rSalpingopharyngeusOrigin));
      rSalpingopharyngeusInsertionFrameMarker.setLocation (new Point3d(rSalpingopharyngeusInsertion));
      
      myJawModel.addFrameMarker (lSalpingopharyngeusOriginFrameMarker);
      myJawModel.addFrameMarker (lSalpingopharyngeusInsertionFrameMarker);
      myJawModel.addFrameMarker (rSalpingopharyngeusOriginFrameMarker);
      myJawModel.addFrameMarker (rSalpingopharyngeusInsertionFrameMarker); 

      Muscle lSalpingopharyngeus = new Muscle ("lSalpingopharyngeus");
      lSalpingopharyngeus.setFirstPoint (lSalpingopharyngeusOriginFrameMarker);
      lSalpingopharyngeus.setSecondPoint (lSalpingopharyngeusInsertionFrameMarker);
      lSalpingopharyngeus.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
      AxialSpring.setDamping (lSalpingopharyngeus, muscleDamping);
      myJawModel.addAxialSpring (lSalpingopharyngeus);  
      Muscle rSalpingopharyngeus = new Muscle ("rSalpingopharyngeus");
      rSalpingopharyngeus.setFirstPoint (rSalpingopharyngeusOriginFrameMarker);
      rSalpingopharyngeus.setSecondPoint (rSalpingopharyngeusInsertionFrameMarker);
      rSalpingopharyngeus.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
      AxialSpring.setDamping (rSalpingopharyngeus, muscleDamping);
      myJawModel.addAxialSpring (rSalpingopharyngeus);
      
      
      
      //Add the stylopharyngeus muscles
      FrameMarker lStylopharyngeusOriginFrameMarker = new FrameMarker ("lStylopharyngeusOrigin");
      FrameMarker lStylopharyngeusInsertionFrameMarker = new FrameMarker ("lStylopharyngeusInsertion");
      FrameMarker rStylopharyngeusOriginFrameMarker = new FrameMarker ("rStylopharyngeusOrigin");
      FrameMarker rStylopharyngeusInsertionFrameMarker = new FrameMarker ("rStylopharyngeusInsertion");
      
      lStylopharyngeusOriginFrameMarker.setFrame (rigidBodies.get("cranium"));
      lStylopharyngeusInsertionFrameMarker.setFrame (rigidBodies.get("thyroid"));
      rStylopharyngeusOriginFrameMarker.setFrame (rigidBodies.get("cranium"));
      rStylopharyngeusInsertionFrameMarker.setFrame (rigidBodies.get("thyroid"));
      
      lStylopharyngeusOriginFrameMarker.setLocation (new Point3d(lStylopharyngeusOrigin));
      lStylopharyngeusInsertionFrameMarker.setLocation (new Point3d(lStylopharyngeusInsertion));
      rStylopharyngeusOriginFrameMarker.setLocation (new Point3d(rStylopharyngeusOrigin));
      rStylopharyngeusInsertionFrameMarker.setLocation (new Point3d(rStylopharyngeusInsertion));
      
      myJawModel.addFrameMarker(lStylopharyngeusOriginFrameMarker);
      myJawModel.addFrameMarker(lStylopharyngeusInsertionFrameMarker);
      myJawModel.addFrameMarker (rStylopharyngeusOriginFrameMarker);
      myJawModel.addFrameMarker (rStylopharyngeusInsertionFrameMarker);
      
      Muscle lStylopharyngeus = new Muscle ("lStylopharyngeus");
      lStylopharyngeus.setFirstPoint (lStylopharyngeusOriginFrameMarker);
      lStylopharyngeus.setSecondPoint (lStylopharyngeusInsertionFrameMarker);
      lStylopharyngeus.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
      AxialSpring.setDamping (lStylopharyngeus, muscleDamping);
      myJawModel.addAxialSpring (lStylopharyngeus);  
      Muscle rStylopharyngeus= new Muscle ("rStylopharyngeus");
      rStylopharyngeus.setFirstPoint (rStylopharyngeusOriginFrameMarker);
      rStylopharyngeus.setSecondPoint (rStylopharyngeusInsertionFrameMarker);
      rStylopharyngeus.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
      AxialSpring.setDamping (rStylopharyngeus, muscleDamping);
      myJawModel.addAxialSpring (rStylopharyngeus);
   }
}
