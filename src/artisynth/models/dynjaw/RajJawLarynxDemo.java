package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.IOException;

import javax.swing.JFrame;

import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

public class RajJawLarynxDemo extends JawDemo {

   public RajJawLarynxDemo() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      //setIncisorVisible();
      String[] hiddenBodies = new String[] { "cranium", "pharynx", "vertebrae" };
      for (int i = 0; i < hiddenBodies.length; i++) {
	 RigidBody body = myJawModel.rigidBodies().get(hiddenBodies[i]);
	 if (body != null) RenderProps.setVisible(body, false);
      }

      RenderProps.setFaceColor(myJawModel.bodyConnectors().get("RTMJ"),
	    Color.PINK);
      RenderProps.setFaceColor(myJawModel.bodyConnectors().get("LTMJ"),
	    Color.PINK);
      
      
      String s1 = "lPalatopharyngeus maxilla thyroid 19.44 22.04 55.81 18.93 30.01 8.47 \n" +
      "rPalatopharyngeus maxilla thyroid -19.44 22.04 55.81 -18.84 30.07 8.46 \n" + 
       "lSalpingopharyngeus cranium thyroid 16.37 23.66 67.91 18.93 30.01 8.47\n" + 
      "rSalpingopharyngeus cranium thyroid -16.37 23.66 67.91 -18.84 30.07 8.46\n" + 
       "lStylopharyngeus cranium thyroid 39.5132 43.205 65.41897 18.93 30.01 8.47\n" + 
      "rStylopharyngeus cranium thyroid -39.5132 43.205 65.41897 -18.84 30.07 8.46\n";


      double maxForce = 15.6;
      double optLength = 73.024;
      double maxL = 93.828;
      double tendonRatio = 0.0;
      double muscleDamping = 0.0;
      
      
      
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
      lPphLowInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
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
      rPphLowInsertion.setFrame (myJawModel.rigidBodies ().get ("thyroid"));
      rPphLowInsertion.setLocation (new Point3d(-10.5, 34.2, -22.8));
      myJawModel.addFrameMarker (rPphLowInsertion);
      Muscle rPphLow = new Muscle("rPph_low");
      rPphLow.setFirstPoint (rPphOrigin);
      rPphLow.setSecondPoint (rPphLowInsertion);
      AxialSpring.setDamping (rPphLow, muscleDamping);
      rPphLow.setPeckMuscleMaterial (maxForce, optLength, maxL, tendonRatio);
      rPphLow.setExcitationColor (Color.RED);
      myJawModel.addAxialSpring (rPphLow);
      
      
      
      
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
      
      
      
      
      
      
      
      //Add the excitation components
      MuscleExciter exciter;
      ExcitationComponent exc;
      
      //Bilateral exciter for the salpingopharyngeus
      exciter = new MuscleExciter ("bi_salp");
      exc = (Muscle) myJawModel.axialSprings ().get ("lSalp");
      exciter.addTarget (exc, 1.0);
      exc = (Muscle) myJawModel.axialSprings ().get ("rSalp");
      exciter.addTarget (exc, 1.0);
      myJawModel.addMuscleExciter (exciter);
      

      //Bilateral exciter for the low and high Palatopharyngeal muscles
      exciter = new MuscleExciter ("bi_Pph_low");
      exc = (Muscle) myJawModel.axialSprings ().get ("lPph_low");
      exciter.addTarget (exc, 1.0);
      exc = (Muscle) myJawModel.axialSprings ().get ("rPph_low");
      exciter.addTarget (exc, 1.0);      
      myJawModel.addMuscleExciter (exciter);
      
      exciter = new MuscleExciter ("bi_Pph_high");
      exc = (Muscle) myJawModel.axialSprings ().get ("lPph_high");
      exciter.addTarget (exc, 1.0);
      exc = (Muscle) myJawModel.axialSprings ().get ("rPph_high");      
      exciter.addTarget (exc, 1.0);
      myJawModel.addMuscleExciter (exciter);
      
      
      //Bilateral exciter for the low and high stylopharyngeus muscles
      exciter = new MuscleExciter ("bi_Stph_low");
      exc = (MultiPointMuscle) myJawModel.multiPointSprings ().get ("lStph_low"); 
      exciter.addTarget (exc, 1.0);
      exc = (MultiPointMuscle) myJawModel.multiPointSprings ().get ("rStph_low");
      exciter.addTarget (exc, 1.0);      
      myJawModel.addMuscleExciter (exciter);
      
      exciter = new MuscleExciter ("bi_Stph_high");      
      exc = (MultiPointMuscle) myJawModel.multiPointSprings ().get ("lStph_high");
      exciter.addTarget (exc, 1.0);      
      exc = (MultiPointMuscle) myJawModel.multiPointSprings ().get ("rStph_high");
      exciter.addTarget (exc, 1.0);            
      myJawModel.addMuscleExciter (exciter);
      
      //setIncisorVisible();
   }

   public void setupJawModel()
   {
      loadBoluses();
      
      myJawModel.showMeshFaces(true);

      myJawModel.setThMemStiffness(385.6);
      myJawModel.setCtrMemStiffness(385.6);
      
      myJawModel.setIntegrator(Integrator.SymplecticEuler);
      myJawModel.setMaxStepSize(0.0001);
      
      setDampingParams(10, 100, 0.001);
      setJawDamping (20, 200);

   }	
   
   protected void loadModel() {
      // create jaw model
      try {
	 myJawModel = new JawModel("jawmodel", /* fixedLaryngeal */false, /* useComplexJoint */
	       true, /* curvJoint */true);
      } catch (Exception e) {
	 e.printStackTrace();
	 System.err.println("RootJaw() - error to instantiating AmiraJaw");
      }

      pokesToControl = new String[] {
	    // "lCondylarAngle",
	    // "rCondylarAngle",
	    // "lCondylarCant",
	    // "rCondylarCant",
	    "lMedWallAngle", "rMedWallAngle", "lPostWallAngle",
	    "rPostWallAngle", "lBiteAngle", "rBiteAngle", "lBiteCant",
	    "rBiteCant" };
   }

   public void loadControlPanel(RootModel root) {
      String panelNames[] = new String[] { "misc", "damping", "muscles",
	    "joints" };
      loadControlPanel(root, panelNames);
   }
   
   public void addJawOptions(ControlPanel panel)
   {
	  if (panel == null)
		 return;
	  JawPanel.createJawLarynxPanel(myJawModel, panel);
   }

   public void attach(DriverInterface driver) {
      workingDirname = "data/rightchew/";
      probesFilename = "rightchew.art";

      super.attach(driver);

      this.setViewerEye(new Point3d(0.0, -268.0, -23.0));
      this.setViewerCenter(new Point3d(0.0, 44.0, 55.0));
      setIncisorVisible();

      addBreakPoint(0.575);
   }

}
