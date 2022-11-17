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

import maspack.interpolation.Interpolation.Order;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
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
import artisynth.models.face.BadinFaceDemoLipOpening;

public class BadinJawHyoidTongueContactBatchWorker extends SimpleTimedBatchWorker {
   
   protected double mySettleTime;
   protected BadinJawHyoidTongueContact root;
   protected FemMuscleModel face;
   protected ComponentList<MuscleExciter> exciters;
   protected PrintWriter myExcitationFileWriter;
   protected PrintWriter myGeometryFileWriter;
   protected PrintWriter myFailedExcitationFileWriter;
   protected PrintWriter myFailedExcitationTargetFileWriter;
   ArrayList<Double> excitations = new ArrayList<Double>();
   

   public BadinJawHyoidTongueContactBatchWorker(String[] args) throws IllegalStateException, IOException {
      
      super(args);
      myMaxTime = 1.00;
      mySettleTime = 0.40;
      
      root = (BadinJawHyoidTongueContact) Main.getMain().getRootModel();
      
      exciters = (ComponentList<MuscleExciter>) root.findComponent("models/jawmodel/models/tongue/exciters");
      
      myExcitationFileWriter =
      new PrintWriter(
         new BufferedWriter(
            new FileWriter(
               new File(
                  myOutputDirName, "excitations." + myName + ".txt"),
               true)));
      
      myGeometryFileWriter =
      new PrintWriter(
         new BufferedWriter(
            new FileWriter(
               new File(
                  myOutputDirName, "geometry." + myName + ".txt"),
               true)));
      
      myFailedExcitationFileWriter =
      new PrintWriter(
         new BufferedWriter(
            new FileWriter(
               new File(
                  myOutputDirName, "failedexcitations." + myName + ".txt"),
               true)));
      
      myFailedExcitationTargetFileWriter =
      new PrintWriter(
         new BufferedWriter(
            new FileWriter(
               new File(
                  myOutputDirName, "failedexcitationtargets." + myName + ".txt"),
               true)));
      
      root.removeAllInputProbes();
   }
   
   @Override
   protected void preSim() {
      Main.getMain().clearWayPoints();
      removeAllExciterProbes();
      addAllExciterProbes();
      super.preSim();
      System.out.println("preSim finished");
   }

   protected void addAllExciterProbes() {
      for(MuscleExciter exc : exciters) {
         root.addExciterProbe(exc.getName(), exc.getExcitation());
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

   @Override
   protected void recordSimResults() {
      recordBinaryWayPoints(true);
      for(MuscleExciter exc : exciters){
         System.out.println(exc.getName() + " " + exc.getExcitation());
      }

      if(myCurrentTaskSuccessful){

      }
      else{
         StringBuilder failedexcitationbuilder = new StringBuilder();
         StringBuilder failedexcitationtargetbuilder = new StringBuilder();
         for(MuscleExciter exc : exciters) {
            failedexcitationbuilder.append(exc.getExcitation()).append(" ");
         }
         for(Double excitation : excitations){
            failedexcitationtargetbuilder.append(excitation).append(" ");
         }
         failedexcitationbuilder.deleteCharAt(failedexcitationbuilder.length() - 1);
         myFailedExcitationFileWriter.println(failedexcitationbuilder);
         myFailedExcitationFileWriter.flush();
         failedexcitationtargetbuilder.deleteCharAt(failedexcitationtargetbuilder.length() - 1);
         myFailedExcitationTargetFileWriter.println(failedexcitationtargetbuilder);
         myFailedExcitationTargetFileWriter.flush();
      }
   }

   @Override
   protected void postSim() {
      removeAllExciterProbes();
      Main.getMain().clearWayPoints();
   }

}

