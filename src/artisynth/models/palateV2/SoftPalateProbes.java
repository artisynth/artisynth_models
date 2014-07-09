package artisynth.models.palateV2;

import maspack.interpolation.Interpolation;
import maspack.interpolation.Interpolation.Order;
import maspack.properties.Property;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.workspace.RootModel;

public class SoftPalateProbes 
{
   static double displayTimeStart = 0.0;
   static double displayTimeEnd = 1.0;
   
   public static void createAndAddProbes(FemMuscleModel fem, RootModel rm)
   {
      rm.getInputProbes().add(palatoglossus_L_probe(fem));
      rm.getInputProbes().add(palatoglossus_R_probe(fem));
      rm.getInputProbes().add(palatoglossus_post_L_probe(fem));
      rm.getInputProbes().add(palatoglossus_post_R_probe(fem));
//      rm.getInputProbes().add(palatopharyngeus_L_probe(fem));
//      rm.getInputProbes().add(palatopharyngeus_R_probe(fem));
      rm.getInputProbes().add(levator_veli_palitini_L_probe(fem));
      rm.getInputProbes().add(levator_veli_palitini_R_probe(fem));
      rm.getInputProbes().add(tensor_veli_palitini_L_probe(fem));
      rm.getInputProbes().add(tensor_veli_palitini_R_probe(fem));
      rm.getInputProbes().add(musculus_uvulae_L_probe(fem));
      rm.getInputProbes().add(musculus_uvulae_R_probe(fem));
   }
   
   public static NumericInputProbe palatoglossus_L_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.palatoglossus_L_name;
      double[] time = {0.00, 0.20, 0.50, 0.60, 0.75};
      double[] act  = {0.01, 0.01, 0.04, 0.04, 0.00};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe palatoglossus_R_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.palatoglossus_R_name;
      double[] time = {0.00, 0.20, 0.50, 0.60, 0.75};
      double[] act  = {0.01, 0.01, 0.04, 0.04, 0.00};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe palatoglossus_post_L_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.palatoglossus_post_L_name;
      double[] time = {0.00, 0.20, 0.50, 0.60, 0.75};
      double[] act  = {0.00, 0.00, 0.10, 0.10, 0.00};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe palatoglossus_post_R_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.palatoglossus_post_R_name;
      double[] time = {0.00, 0.20, 0.50, 0.60, 0.75};
      double[] act  = {0.00, 0.00, 0.10, 0.10, 0.00};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe palatopharyngeus_L_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.palatopharyngeus_L_name;
      double[] time = {0.00, 0.05, 0.20, 0.50, 0.80, 0.95, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.10, 0.05, 0.00, 0.00};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe palatopharyngeus_R_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.palatopharyngeus_R_name;
      double[] time = {0.00, 0.05, 0.20, 0.50, 0.80, 0.95, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.10, 0.05, 0.00, 0.00};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe levator_veli_palitini_L_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.levator_veli_palitini_L_name;
      double[] time = {0.00, 0.05, 0.20, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.05};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe levator_veli_palitini_R_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.levator_veli_palitini_R_name;
//      double[] time = {0.00, 0.05, 0.30, 2.00};
//      double[] act  = {0.00, 0.00, 0.10, 0.10};
      double[] time = {0.00, 0.05, 0.20, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.05};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe tensor_veli_palitini_L_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.tensor_veli_palitini_L_name;
      double[] time = {0.00, 0.05, 0.20, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.05};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe tensor_veli_palitini_R_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.tensor_veli_palitini_R_name;
      double[] time = {0.00, 0.05, 0.20, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.05};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe musculus_uvulae_L_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.musculus_uvulae_L_name;
      double[] time = {0.00, 0.05, 0.20, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.05};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe musculus_uvulae_R_probe(FemMuscleModel fem)
   {
      String name = SoftPalateMuscles.musculus_uvulae_R_name;
      double[] time = {0.00, 0.05, 0.20, 1.00};
      double[] act  = {0.00, 0.00, 0.05, 0.05};
      
      return createMuscleProbe(fem, name, time, act);
   }
   
   public static NumericInputProbe createMuscleProbe(FemMuscleModel fem, String name, double[] time, double[] activation)
   {
      NumericInputProbe ip = new NumericInputProbe();
      ip.setName(name);
      
      Interpolation interp = new Interpolation();
      interp.setOrder(Order.Linear);
      ip.setInterpolation(interp);
      
      ip.setStartStopTimes(displayTimeStart, displayTimeEnd);	// time-axis
      ip.setDefaultDisplayRange(0.0, 0.2);				// y-axis
      ip.setModel(fem);
      
      Property[] props = {fem.getMuscleBundles().get(name).getProperty("excitation")};
      String[] driverExpressions = {"V0"};	// ??
      String[] variableNames = {"V0"};		// ??
      int[] variableDimensions = {1};
      ip.set(props, driverExpressions, variableNames, variableDimensions, null);
      
      for (int a=0; a<time.length; a++)
      {
	 ip.addData(new double[] {time[a], activation[a]}, NumericInputProbe.EXPLICIT_TIME);
      }
      
      return ip;
      
   }
   

}
