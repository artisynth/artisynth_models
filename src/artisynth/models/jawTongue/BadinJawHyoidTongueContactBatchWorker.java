package artisynth.models.jawTongue;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.AbstractMap.SimpleEntry;

import maspack.interpolation.Interpolation.Order;
import maspack.properties.Property;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.tools.batchsim.SimpleTimedBatchWorker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker;
import artisynth.tools.batchsim.conditions.TimeChecker;
import artisynth.tools.batchsim.conditions.EquilibriumChecker.EquilibriumCondition;
import artisynth.tools.batchsim.conditions.TimeChecker.TimeCondition;

public class BadinJawHyoidTongueContactBatchWorker extends SimpleTimedBatchWorker {
   
   protected double mySettleTime;
   protected BadinJawHyoidTongueContact root;
   protected DistanceMonitor rootModelDistMonitor;
   protected FemMuscleModel face;
   protected ComponentList<MuscleExciter> exciters;
   protected PrintWriter myContactsFileWriter;
   protected PrintWriter myExcitationFileWriter;
   protected PrintWriter myFailedExcitationFileWriter;

   public BadinJawHyoidTongueContactBatchWorker(String[] args) throws IllegalStateException, IOException {
      
      super(args);
      myMaxTime = 1.00;
      mySettleTime = 0.40;
      
      root = (BadinJawHyoidTongueContact) Main.getMain().getRootModel();
      
      exciters = (ComponentList<MuscleExciter>) root.findComponent("models/jawmodel/models/tongue/exciters");
      myContactsFileWriter = initWriter(myOutputDirName, "contacts." + myName + ".txt");
      myExcitationFileWriter = initWriter(myOutputDirName, "excitations." + myName + ".txt");
      myFailedExcitationFileWriter = initWriter(myOutputDirName, "failedexcitations." + myName + ".txt");
      
      rootModelDistMonitor = root.getDistanceMonitor();
   
      root.removeAllInputProbes();
   }
   
   protected PrintWriter initWriter (String outputDir, String filename)
   throws IOException {
      return new PrintWriter (
         new BufferedWriter (
            new FileWriter (new File (outputDir, filename), true)));
   }
   
   @Override
   protected void preSim() {
      root = (BadinJawHyoidTongueContact) Main.getMain().getRootModel();
      exciters = (ComponentList<MuscleExciter>) root.findComponent("models/jawmodel/models/tongue/exciters");
      rootModelDistMonitor = root.getDistanceMonitor();
      root.removeAllInputProbes();
      addAllExciterProbes();
      super.preSim();
      for(MuscleExciter exc : exciters) {
          System.out.println(exc.getName() + " " + exc.getExcitation());
       }
      System.out.println("preSim finished");
   }

//   protected void addAllExciterProbes() {
//      for(MuscleExciter exc : exciters) {
//         root.addExciterProbe(exc.getName(), exc.getExcitation());
//      }
//   }
   
   protected void addAllExciterProbes() {
      for (String[] compPropVal : myCurrentTask) {
         String propPath = compPropVal[0];
         Property prop = myRootModel.getProperty (propPath);
         if (prop.getHost () instanceof MuscleExciter) {
            MuscleExciter exc = (MuscleExciter) prop.getHost();
            root.addExciterProbe(exc.getName(), exc.getExcitation());
         }
      }
   }

   protected void removeAllExciterProbes() {
      for(MuscleExciter exc : exciters) {
         Probe eProbe = root.getInputProbes().get(exc.getName() + " exciter probe");
         if(eProbe != null) {
            root.removeInputProbe(eProbe);
         }
      }
   }
   
   protected void recordContacts () {
      ArrayList<SimpleEntry<FrameMarker,Double>> distanceList =
         rootModelDistMonitor.getDistanceMappingList ();

      int frontContacts = 0;
      int midContacts = 0;
      int backContacts = 0;
      int latLeftContacts = 0;
      int latRightContacts = 0;
      int coronalContacts = 0;

      ArrayList<SimpleEntry<FrameMarker,Boolean>> contactList =
         DistanceMonitor.generateContactMappingList (
            distanceList, DistanceMonitor.CONTACT_THRESHOLD);
      for (SimpleEntry<FrameMarker,Boolean> entry : contactList) {
         if (entry.getValue ()) {
            if (root.getCoronalFrontAndMidMarkerNames ().contains (
               entry.getKey ().getName ())) {
               coronalContacts++;
            }

            // Roundabout way, but other methods like contains() and indexOf()
            // weren't working
            if (root.getFrontOralCavityMarkers ().get (
               entry.getKey ().getName ()) != null) {
               frontContacts++;
            }
            else if (root.getMidOralCavityMarkers ().get (
               entry.getKey ().getName ()) != null) {
               midContacts++;
            }
            else if (root.getBackOralCavityMarkers ().get (
               entry.getKey ().getName ()) != null) {
               backContacts++;
            }
            else if (root.getLateralLeftOralCavityMarkers ().get (
               entry.getKey ().getName ()) != null) {
               latLeftContacts++;
            }
            else if (root.getLateralRightOralCavityMarkers ().get (
               entry.getKey ().getName ()) != null) {
               latRightContacts++;
            }
         }
      }

      StringBuilder builder = new StringBuilder ();
      builder.append (myTaskCounter).append (",");
      builder.append (frontContacts).append (",");
      builder.append (midContacts).append (",");
      builder.append (backContacts).append (",");
      builder.append (latLeftContacts).append (",");
      builder.append (latRightContacts).append (",");
      builder.append (coronalContacts);
      myContactsFileWriter.println (builder);
      myContactsFileWriter.flush ();
   }


   @Override
   protected void recordSimResults() {
      if(myCurrentTaskSuccessful){
         recordContacts();
         // Record excitations
         StringBuilder builder = new StringBuilder ();
         builder.append (myTaskCounter).append (",");
         for (String[] propVal : myCurrentTask) {
            builder.append (propVal[1]).append (",");
         }
         builder.deleteCharAt (builder.length () - 1);
         myExcitationFileWriter.println (builder);
         myExcitationFileWriter.flush ();
      }
      else{
         StringBuilder failedexcitationbuilder = new StringBuilder();

         for(MuscleExciter exc : exciters) {
            failedexcitationbuilder.append(exc.getExcitation()).append(",");
         }
         failedexcitationbuilder.deleteCharAt(failedexcitationbuilder.length() - 1);
         myFailedExcitationFileWriter.println(failedexcitationbuilder);
         myFailedExcitationFileWriter.flush();
      }
   }

   @Override
   protected void postSim() {
      removeAllExciterProbes();
   }
   
   @Override
   protected void setUpStopConditionMonitor () {
      myMaxTime = 0.5;
      super.setUpStopConditionMonitor ();

      TimeChecker tchk =
         new TimeChecker (
            TimeCondition.IN_RANGE_INCLUSIVE,
            myMaxTime - mySettleTime + myRootModel.getMaxStepSize(),  myMaxTime);

      List<ModelComponent> comps = new LinkedList<>();
      for (ModelComponent comp: ((BadinJawHyoidTongueContact) myRootModel).getTongue().getNodes()) {
         comps.add (comp);
      }
      EquilibriumChecker echk =
         new EquilibriumChecker (EquilibriumCondition.STATIC, tchk, 1, comps);

      myStopConditionMonitor.addConditionChecker (echk);
   }
   
   @Override
   public void closeWriters () {
      super.closeWriters ();
      myContactsFileWriter.close ();
      myExcitationFileWriter.close ();
      myFailedExcitationFileWriter.close ();
   }

}

