package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import maspack.matrix.Point3d;

import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentListView;
import java.io.StringReader;
import maspack.util.ReaderTokenizer;

public class JawLongPharyngealDemo extends JawLarynxDemo{

   
   private static String s1 = "lPalatopharyngeus maxilla thyroid 19.44 22.04 55.81 18.93 30.01 8.47 \n" +
   		"rPalatopharyngeus maxilla thyroid -19.44 22.04 55.81 -18.84 30.07 8.46 \n" + 
   		 "lSalpingopharyngeus cranium thyroid 16.37 23.66 67.91 18.93 30.01 8.47\n" + 
   		"rSalpingopharyngeus cranium thyroid -16.37 23.66 67.91 -18.84 30.07 8.46\n" + 
   		 "lStylopharyngeus cranium thyroid 39.5132 43.205 65.41897 18.93 30.01 8.47\n" + 
   		"rStylopharyngeus cranium thyroid -39.5132 43.205 65.41897 -18.84 30.07 8.46\n";
   
   /**
    * @param args
    * @throws IOException 
    */
   public JawLongPharyngealDemo () throws IOException {
      // TODO Auto-generated constructor stub
      super();

   }

   public static void main(String [] args){
      ReaderTokenizer rt = new ReaderTokenizer (new StringReader (s1));
      try {
           while(rt.nextToken () != rt.TT_EOF){
              rt.pushBack ();
              String muscleName = rt.scanWord ();
              String originBody = rt.scanWord ();
              String insertionBody = rt.scanWord ();
              double [] originPoint = new double[3];
              rt.scanNumbers (originPoint, 3);
              double [] insertionPoint = new double[3];
              rt.scanNumbers (insertionPoint, 3);
              System.out.println(muscleName + " " + originBody + " " + insertionBody + " " + originPoint.toString () + " " + insertionPoint.toString ());
           }
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }      
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      
      double maxForce = 15.6;
      double optLength = 73.024;
      double maxL = 93.828;
      double tendonRatio = 0.0;
      double muscleDamping = 0.0;
      
      ReaderTokenizer rt = new ReaderTokenizer (new StringReader (s1));
      while (rt.nextToken () != ReaderTokenizer.TT_EOF){
         rt.pushBack ();
         String muscleName = rt.scanWord ();
         String originBody = rt.scanWord ();
         String insertionBody = rt.scanWord ();
         double [] originPoint = new double[3];
         rt.scanNumbers (originPoint, 3);
         double [] insertionPoint = new double[3];
         rt.scanNumbers (insertionPoint, 3);
         
         //Add frame markers for this muscle
         FrameMarker fmOrigin = new FrameMarker(muscleName + "_origin");
         FrameMarker fmInsertion = new FrameMarker(muscleName + "_insertion");
         fmOrigin.setFrame (myJawModel.rigidBodies ().get (originBody));
         fmInsertion.setFrame (myJawModel.rigidBodies ().get (insertionBody));
         fmOrigin.setLocation (new Point3d(originPoint));
         fmInsertion.setLocation (new Point3d(insertionPoint));
         myJawModel.addFrameMarker (fmOrigin);
         myJawModel.addFrameMarker (fmInsertion);
         
         //Set muscle properties and add it to the myJawModel mechmodel
         Muscle m = new Muscle(muscleName);
         m.setFirstPoint (fmOrigin);
         m.setSecondPoint (fmInsertion);
         m.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
         AxialSpring.setDamping (m, muscleDamping);
         myJawModel.addAxialSpring (m);
         
         //Set the renderprops for this muscle
         m.setExcitationColor (Color.RED);
      }
      
/*      //Coordinate data for the palatopharyngeus
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
*/
      
   
   }
}
