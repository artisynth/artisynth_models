package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import maspack.matrix.Point3d;

import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.probes.InputProbe;
import artisynth.core.probes.OutputProbe;

import java.io.StringReader;
import maspack.util.ReaderTokenizer;

public class JawLongPharyngealDemo extends JawLarynxDemo{

   
   private static String s1 = "lPalatopharyngeus maxilla thyroid 19.44 22.04 55.81 18.93 30.01 8.47 \n" +
   		"rPalatopharyngeus maxilla thyroid -19.44 22.04 55.81 -18.84 30.07 8.46 \n" + 
   		 "lSalpingopharyngeus cranium thyroid 16.37 23.66 67.91 18.93 30.01 8.47\n" + 
   		"rSalpingopharyngeus cranium thyroid -16.37 23.66 67.91 -18.84 30.07 8.46\n" + 
   		 "lStylopharyngeus cranium thyroid 39.5132 43.205 65.41897 18.93 30.01 8.47\n" + 
   		"rStylopharyngeus cranium thyroid -39.5132 43.205 65.41897 -18.84 30.07 8.46\n";
   
   
   private static double maxForce = 15.6;
   private static double optLength = 73.024;
   private static double maxL = 93.828;
   private static double tendonRatio = 0.0;
   private static double muscleDamping = 0.0;
   
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
      
      /*
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
         m.setExcitationColor (Color.RED);*/
         buildStylopharyngeus();
         buildPalatopharyngeus();
         buildSalpingopharyngeus();
      }
   
   
      private void buildStylopharyngeus(){
         /*
          * The stylopharyngeus originates from the styloglossus region of the styloid process.
          * It has a distinct elbow on its way to insertion at various parts of the thyroid cartilage.
          * In this simulation, it's approximated by two MultiPointMuscles that merge into those
          * two insertion points. Both muscles have the same elbow point, which is a reasonable 
          * approximation. The MultiPointMuscle that attaches to the horn of the thyroid cartilage is
          * internally referred to as Stph_high and the other one is Stph_low
         */

         //Features common to the left Stph
         FrameMarker lStphOrigin = new FrameMarker("lStph_origin");
         FrameMarker lStphElbow = new FrameMarker("lStph_elbow");
         
         //Assemble left Stph_high
         FrameMarker lStphHighInsertion = new FrameMarker("lStphHigh_insertion");
         lStphOrigin.setFrame (myJawModel.rigidBodies ().get ("maxilla"));
         lStphElbow.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         lStphHighInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
         lStphOrigin.setLocation (new Point3d(39.5, 42.2, 65.4));
         lStphElbow.setLocation (new Point3d(14.0, 30.6, 41.5));
         lStphHighInsertion.setLocation (new Point3d(18.9, 30.0, 8.4));
         myJawModel.addFrameMarker (lStphOrigin);
         myJawModel.addFrameMarker (lStphElbow);
         myJawModel.addFrameMarker (lStphHighInsertion);
         MultiPointMuscle lStphHighMuscle = MultiPointMuscle.createPeck ("lStph_high", maxForce, optLength, maxL, tendonRatio);
         lStphHighMuscle.addPoint (lStphOrigin);
         lStphHighMuscle.addPoint (lStphElbow);
         lStphHighMuscle.addPoint (lStphHighInsertion);
         AxialSpring.setDamping (lStphHighMuscle, muscleDamping);
         myJawModel.addMultiPointSpring (lStphHighMuscle);
         lStphHighMuscle.setExcitationColor (Color.RED);
         

         //ASsemble the left Stph_low
         FrameMarker lStphLowInsertion = new FrameMarker("lStphLow_insertion");
         lStphLowInsertion.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         lStphLowInsertion.setLocation (new Point3d(17.4, 26.4, -13.7));
         myJawModel.addFrameMarker (lStphLowInsertion);
         MultiPointMuscle lStphLowMuscle = MultiPointMuscle.createPeck ("lStph_low", maxForce, optLength, maxL, tendonRatio);
         lStphLowMuscle.addPoint (lStphOrigin);
         lStphLowMuscle.addPoint (lStphElbow);
         lStphLowMuscle.addPoint (lStphLowInsertion);
         AxialSpring.setDamping (lStphLowMuscle, muscleDamping);
         myJawModel.addMultiPointSpring (lStphLowMuscle);
         lStphLowMuscle.setExcitationColor (Color.RED);
         
         
         
         
         //Features common to the right Stph
         FrameMarker rStphOrigin = new FrameMarker("rStph_origin");
         FrameMarker rStphElbow = new FrameMarker("rStph_elbow");
         
         
         //Assemble right Stph_high
         FrameMarker rStphHighInsertion = new FrameMarker("rStphHigh_insertion");
         rStphOrigin.setFrame (myJawModel.rigidBodies ().get ("maxilla"));
         rStphElbow.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         rStphHighInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
         rStphOrigin.setLocation (new Point3d(-39.5, 42.2, 65.4));
         rStphElbow.setLocation (new Point3d(-14.0, 30.6, 41.5));
         rStphHighInsertion.setLocation (new Point3d(-18.9, 30.0, 8.4));
         myJawModel.addFrameMarker (rStphOrigin);
         myJawModel.addFrameMarker (rStphElbow);
         myJawModel.addFrameMarker (rStphHighInsertion);
         MultiPointMuscle rStphHighMuscle = MultiPointMuscle.createPeck ("rStph_high", maxForce, optLength, maxL, tendonRatio);
         rStphHighMuscle.addPoint (rStphOrigin);
         rStphHighMuscle.addPoint (rStphElbow);
         rStphHighMuscle.addPoint (rStphHighInsertion);
         AxialSpring.setDamping (rStphHighMuscle, muscleDamping);
         myJawModel.addMultiPointSpring (rStphHighMuscle);
         rStphHighMuscle.setExcitationColor (Color.RED);

         
         //ASsemble the left Stph_low
         FrameMarker rStphLowInsertion = new FrameMarker("rStphLow_insertion");
         rStphLowInsertion.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         rStphLowInsertion.setLocation (new Point3d(-17.4, 26.4, -13.7));
         myJawModel.addFrameMarker (rStphLowInsertion);
         MultiPointMuscle rStphLowMuscle = MultiPointMuscle.createPeck ("rStph_low", maxForce, optLength, maxL, tendonRatio);
         rStphLowMuscle.addPoint (rStphOrigin);
         rStphLowMuscle.addPoint (rStphElbow);
         rStphLowMuscle.addPoint (rStphLowInsertion);
         AxialSpring.setDamping (rStphLowMuscle, muscleDamping);
         myJawModel.addMultiPointSpring (rStphLowMuscle);
         rStphLowMuscle.setExcitationColor (Color.RED);
         
      }
      
      private void buildPalatopharyngeus(){

         /*
          * The palatopharyngeus muscles would be better simulated with low and high insertion points
          * around the thyroid cartilage, just like the palatopharyngeus. The muscles are almost
          * linear in shape so they are modelled as regular Axial Springs
         */         
         
         
         //Features common to the left Pph
         FrameMarker lPphOrigin = new FrameMarker("lPph_origin");
         

         //Add lPph high
         FrameMarker lPphHighInsertion = new FrameMarker("lPph_high_insertion");
         lPphOrigin.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         lPphOrigin.setLocation (new Point3d(19.44, 22.04, 55.81));
         lPphHighInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
         lPphHighInsertion.setLocation (new Point3d(6.8, 36.3, -11.7));
         myJawModel.addFrameMarker (lPphOrigin);
         myJawModel.addFrameMarker (lPphHighInsertion);
         Muscle lPphHigh = new Muscle("lPph_high");
         lPphHigh.setFirstPoint (lPphOrigin);
         lPphHigh.setSecondPoint (lPphHighInsertion);
         AxialSpring.setDamping (lPphHigh, muscleDamping);
         lPphHigh.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
         lPphHigh.setExcitationColor (Color.RED);
         myJawModel.addAxialSpring (lPphHigh);
         
         //Add lPph low
         FrameMarker lPphLowInsertion = new FrameMarker("lPph_low_insertion");
         lPphLowInsertion.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         lPphLowInsertion.setLocation (new Point3d(10.5, 34.2, -22.8));
         myJawModel.addFrameMarker (lPphLowInsertion);
         Muscle lPphLow = new Muscle("lPph_low");
         lPphLow.setFirstPoint (lPphOrigin);
         lPphLow.setSecondPoint (lPphLowInsertion);
         AxialSpring.setDamping (lPphLow, muscleDamping);
         lPphLow.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
         lPphLow.setExcitationColor (Color.RED);
         myJawModel.addAxialSpring (lPphLow);

         
         //Features common to the right Pph
         FrameMarker rPphOrigin = new FrameMarker("rPph_origin");

         //Add rPph high
         FrameMarker rPphHighInsertion = new FrameMarker("rPph_high_insertion");
         rPphOrigin.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         rPphOrigin.setLocation (new Point3d(-19.44, 22.04, 55.81));
         rPphHighInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
         rPphHighInsertion.setLocation (new Point3d(-6.8, 36.3, -11.7));
         myJawModel.addFrameMarker (rPphOrigin);
         myJawModel.addFrameMarker (rPphHighInsertion);
         Muscle rPphHigh = new Muscle("rPph_high");
         rPphHigh.setFirstPoint (rPphOrigin);
         rPphHigh.setSecondPoint (rPphHighInsertion);
         AxialSpring.setDamping (rPphHigh, muscleDamping);
         rPphHigh.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
         rPphHigh.setExcitationColor (Color.RED);
         myJawModel.addAxialSpring (rPphHigh);

         //Add rPph low
         FrameMarker rPphLowInsertion = new FrameMarker("rPph_low_insertion");
         rPphLowInsertion.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         rPphLowInsertion.setLocation (new Point3d(-10.5, 34.2, -22.8));
         myJawModel.addFrameMarker (rPphLowInsertion);
         Muscle rPphLow = new Muscle("rPph_low");
         rPphLow.setFirstPoint (rPphOrigin);
         rPphLow.setSecondPoint (rPphLowInsertion);
         AxialSpring.setDamping (rPphLow, muscleDamping);
         rPphLow.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
         rPphLow.setExcitationColor (Color.RED);
         myJawModel.addAxialSpring (rPphLow);
         
         
         
      }

      private void buildSalpingopharyngeus(){
         /*
          * The salpingopharyngeus is modeled with 2 Axial Springs.
          */
         
         //Assemble the left salpingopharyngeus
         FrameMarker lSalpOrigin = new FrameMarker("lSalp_origin");
         lSalpOrigin.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         lSalpOrigin.setLocation (new Point3d(16.37, 23.66, 67.91));
         FrameMarker lSalpInsertion = new FrameMarker("lSalp_insertion");
         lSalpInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
         lSalpInsertion.setLocation (new Point3d(18.3, 26.9, -11.1));
         myJawModel.addFrameMarker (lSalpOrigin);
         myJawModel.addFrameMarker (lSalpInsertion);
         Muscle lSalp = new Muscle("lSalp");
         lSalp.setFirstPoint (lSalpOrigin);
         lSalp.setSecondPoint (lSalpInsertion);
         AxialSpring.setDamping (lSalp, muscleDamping);
         lSalp.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
         lSalp.setExcitationColor (Color.RED);
         myJawModel.addAxialSpring (lSalp);
         
         //Assemble the right salpingopharyngeus
         FrameMarker rSalpOrigin = new FrameMarker("rSalp_origin");
         rSalpOrigin.setFrame (myJawModel.rigidBodies ().get ("cranium"));
         rSalpOrigin.setLocation (new Point3d(-16.37, 23.66, 67.91));
         FrameMarker rSalpInsertion = new FrameMarker("rSalp_insertion");
         rSalpInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
         rSalpInsertion.setLocation (new Point3d(-18.3, 26.9, -11.1));
         myJawModel.addFrameMarker (rSalpOrigin);
         myJawModel.addFrameMarker (rSalpInsertion);
         Muscle rSalp = new Muscle("rSalp");
         rSalp.setFirstPoint (rSalpOrigin);
         rSalp.setSecondPoint (rSalpInsertion);
         AxialSpring.setDamping (rSalp, muscleDamping);
         rSalp.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
         rSalp.setExcitationColor (Color.RED);
         myJawModel.addAxialSpring (rSalp);
         
         
         
      }
}

