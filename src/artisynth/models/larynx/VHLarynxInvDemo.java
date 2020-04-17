package artisynth.models.larynx;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import maspack.properties.Property;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;

public class VHLarynxInvDemo extends VHLarynxDemo {

   public VHLarynxInvDemo() {
   }

   double probeInterval=0.05;
   double probeStart = 0.0;
   double probeEnd = 3.1;
   TrackingController invcon;
   
   String[] targetMarkers = {
                             "Hyoid_anterior",
//                           "Hyoid_posterior",
//                           "SubglotticAirColumn_superior_posterior",
//                           "Cricoid_superior_anterior",
//                           "Hyoid_posterior_L",
//                           "Hyoid_posterior_R",
                       };
   
   public VHLarynxInvDemo(String name) throws IOException {
      super(name);
      
      //------------ ninds 5ml water swallow data -------------------
      // Data 108, trial 01,12
      // Data 110, trial 01,16
      // Data 111, trial 01,09
      // Data 112, trial 01,12
      
      super.probesPath = ArtisynthPath.getSrcRelativePath(this, "data/");
      super.probesFilename = "";
      //super.probesFilename = "../../probe2Markers.art";
      //super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_108/trial01/";
      super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_108/trial12/";
      
      //super.probesFilename = "../../probe2Markers_hyoid.art";
      //super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_110/trial01";
      //super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_110/trial16/";
      //super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_111/trial01/";
      //super.workingDirname = "src/artisynth/models/larynx/data/ninds/data_111/trial09/";
      
      super.wayPointStep = 0.05;
      super.stopPoint = 3.1d;
      super.MAX_STEP_SIZE_SEC = 1e-3;
      myMechMod.setMaxStepSize(MAX_STEP_SIZE_SEC);
      
      myMechMod.setPenetrationTol(0.005);
   }

   public void attach(DriverInterface driver)
   {
      super.attach(driver);
      setSagittalView(1.5);
      addInverseController();
      //addMonitor(new TrackingEnableMonitor());
      //Main.getMain().getInverseManager().setProbeDuration(stopPoint);
   }
   
   protected void addInverseController() {
      invcon = new TrackingController(myMechMod, "inverse"); 
      //invcon.addMotionTarget(myMechMod.rigidBodies().get("hyoid"));
      
      for(String markerName: targetMarkers) {
	 invcon.addMotionTarget(myMechMod.frameMarkers().get(markerName));
	 setParticleTracing(markerName+"_ref");
	 setFrameMarkerTracing(markerName);
      }
      
      for(MuscleExciter e: myMechMod.getMuscleExciters()) {
	 /*
	 if(e.getName().indexOf("_L")==-1 && e.getName().indexOf("_R")==-1) {
	    if(
		  e.getName().indexOf("Mylohyoid")>-1 || 
		  e.getName().indexOf("Geniohyoid")>-1 ||
		  e.getName().indexOf("Digastric_posterior")>-1 ||
		  e.getName().indexOf("Digastric_anterior")>-1 ||
		  e.getName().indexOf("Stylohyoid")>-1
		  ) {
	       invcon.addExciter(e);
	    }
	    
	    //invcon.addExciter(e);
	 }
	 */
	 
	 
	 if(e.getName().indexOf("_L")==-1 && e.getName().indexOf("_R")==-1
	       //&& e.getName().indexOf("Stylohyoid")==-1
	       //&& e.getName().indexOf("Mylohyoid")==-1
	       //&& e.getName().indexOf("Geniohyoid")==-1
	       //&& e.getName().indexOf("Digastric_posterior")==-1
	       && e.getName().indexOf("Pharyngeal_constrictor")==-1
	       && e.getName().indexOf("Thyrohyoid")==-1
	       ) {
	    invcon.addExciter(e);
	 }
	 
      }
      
      //invcon.addL2RegularizationTerm(500*1000);
      invcon.addL2RegularizationTerm(1);
      invcon.addDampingTerm(0.01);
      if(probeStart>0)
         invcon.setActive (false);
      addController(invcon);
      
      configureInputProbes(invcon);
      configureOutputProbes(invcon);
      invcon.createPanel (this);
   }
   private void configureInputProbes(TrackingController track) {
      NumericInputProbe targetProbe = new NumericInputProbe();
      targetProbe.setName("Target Input Positions");
      targetProbe.setAttachedFileName("ref_targetPos_input.txt");
      targetProbe.setStartStopTimes(probeStart, probeEnd);
      
      ArrayList<Property> props = new ArrayList<Property>();
      
      for (MotionTargetComponent target : track.getMotionTargetTerm().getTargets()) {
         if (target instanceof Point) {
            props.add( ((Point)target).getProperty("targetPosition"));
         }
      }
      targetProbe.setModel(track.getMech());
      targetProbe.setInputProperties(props.toArray(new Property[props.size()]));
      try {
         targetProbe.load();
      } catch (IOException e) {
         e.printStackTrace();
      }
      addInputProbe(targetProbe);
   }
   private void configureOutputProbes(TrackingController track) {

      NumericOutputProbe outProbe = new NumericOutputProbe();
      outProbe.setName("Target Output Positions");
      outProbe.setAttachedFileName("result_targetPos_output.txt");
      ArrayList<Property> props = new ArrayList<Property>();
      
      Point target;
      for(String markerName: targetMarkers) {
         target = myMechMod.frameMarkers().get(markerName);
         if(target!=null) {
            props.add( target.getProperty("targetPosition"));
         }
      }
      outProbe.setModel(track.getMech());
      outProbe.setOutputProperties(props.toArray(new Property[props.size()]));
      outProbe.setUpdateInterval(probeInterval);
      outProbe.setStartStopTimes(probeStart, probeEnd);
      addOutputProbe(outProbe);
      
      outProbe = new NumericOutputProbe();
      outProbe.setName("Activation Parameters Output");
      outProbe.setAttachedFileName("muscleActivation.txt");
      props = new ArrayList<Property>();
      
      for ( ExcitationComponent mex : track.getExciters()) {
         if (mex instanceof MuscleExciter) {
            props.add( ((MuscleExciter)mex).getProperty("excitation") );
         }
      }
      outProbe.setModel(track.getMech());
      outProbe.setOutputProperties(props.toArray(new Property[props.size()]));
      outProbe.setUpdateInterval(probeInterval);
      outProbe.setStartStopTimes(probeStart, probeEnd);
      addOutputProbe(outProbe);
   }
   public class TrackingEnableMonitor extends MonitorBase {

      PrintWriter myPw;
      FemModel3d myFem;

      TrackingEnableMonitor (){
      }

      public void apply (double t0, double t1) {
         if(t0>=probeStart && !invcon.isActive()) {
            invcon.setActive (true);
            for (MotionTargetComponent target : invcon.getMotionTargetTerm().getTargets()) {
               if (target instanceof Point) {
                  setParticleTracing(target.getName ());
               }
            }
         }
      }
   }
}
