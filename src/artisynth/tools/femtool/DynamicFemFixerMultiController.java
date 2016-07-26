package artisynth.tools.femtool;

import java.util.ArrayList;
import java.util.HashMap;

import maspack.matrix.Point3d;
import maspack.properties.PropertyList;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.util.TimeBase;

public class DynamicFemFixerMultiController extends ControllerBase {

   public static double DEFAULT_UPDATE_INTERVAL = 0.1;
   public static double DEFAULT_CHECK_CONVERGENCE_INTERVAL = 0.1;
   public static double DEFAULT_CONVERGENCE_THRESHOLD = 1e-5;
   public static boolean DEFAULT_AUTO_PAUSE = false;
   public static boolean DEFAULT_AUTO_UPDATE_REF_MODELS = false;
   
   private double updateInterval = DEFAULT_UPDATE_INTERVAL;
   private double checkConvergenceInterval = DEFAULT_CHECK_CONVERGENCE_INTERVAL;
   private boolean autoPause = DEFAULT_AUTO_PAUSE;
   private double convergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;
   private boolean autoUpdateRefModels = DEFAULT_AUTO_UPDATE_REF_MODELS;
   
   ArrayList<DynamicFemFixer> myFixers = null;
   private HashMap<FemNode3d, Point3d> lastPositions;  // keeps track of previous positions for all nodes
   
   public static PropertyList myProps =
      new PropertyList(DynamicFemFixerMultiController.class);

   static {
      myProps.add("updateInterval * *", "", DEFAULT_UPDATE_INTERVAL);
      myProps.add("checkConvergenceInterval * *", "", DEFAULT_CHECK_CONVERGENCE_INTERVAL);
      myProps.add("convergenceThreshold * *", "", DEFAULT_CONVERGENCE_THRESHOLD);
      myProps.add("autoPause * *", "", DEFAULT_AUTO_PAUSE);
      myProps.add("autoUpdateRefModels * *", "", DEFAULT_AUTO_UPDATE_REF_MODELS);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public DynamicFemFixerMultiController() {
      super();
      lastPositions = new HashMap<FemNode3d,Point3d>();
      myFixers = new ArrayList<DynamicFemFixer>();
   }
   
   public void setUpdateInterval(double dt) {
      updateInterval = dt;
   }
   
   public double getUpdateInterval() {
      return updateInterval;
   }
   
   public void freezeAllSurfaces(boolean freeze) {
      
      for (DynamicFemFixer fixer : myFixers ) {
         fixer.freezeSurface(freeze);   
      }
      
   }
   
   public void freezeAllInteriors(boolean freeze) {
      
      for (DynamicFemFixer fixer : myFixers ) {
         fixer.freezeInterior(freeze);   
      }
      
   }
   
   public boolean getAutoPause() {
      return autoPause;
   }
   public void setAutoPause(boolean auto) {
      autoPause = auto;
   }
   
   public void setCheckConvergenceInterval(double interval) {
      checkConvergenceInterval = interval;
   }
   
   public double getCheckConvergenceInterval() {
      return checkConvergenceInterval;
   }
   
   public void setConvergenceThreshold(double epsilon) {
      convergenceThreshold = epsilon;
   }
   
   public double getConvergenceThreshold() {
      return convergenceThreshold;
   }
   
   public void setAutoUpdateRefModels(boolean auto) {
      autoUpdateRefModels = auto;
   }
   
   public boolean getAutoUpdateRefModels() {
      return autoUpdateRefModels;
   }
   
   public void setFemFixers(ArrayList<DynamicFemFixer> fixers) {
      myFixers = fixers;
      lastPositions.clear();
   }
     
   public void addFemFixer(DynamicFemFixer fixer) {
      if (!myFixers.contains(fixer)) {
         myFixers.add(fixer);
      }
   }
   
   public void removeFemFixer(DynamicFemFixer fixer) {
      myFixers.remove(fixer);
   }
   
   public void apply(double t0, double t1) {

      if (updateInterval > 0) {
         if (TimeBase.modulo(t0, updateInterval)==0 ) { 
            
            for (DynamicFemFixer fixer : myFixers) {
               fixer.updateIdealRestPositions();
            }
         }   
      }
      
      if (autoPause && checkConvergenceInterval > 0) {
         if (TimeBase.modulo(t1, checkConvergenceInterval)==0) {
            double diff = 0;
            
            for (DynamicFemFixer fixer : myFixers) {
               for (FemNode3d node : fixer.getModel().getNodes()) {
                  
                  Point3d lastPos = lastPositions.get(node);
                  if (lastPos == null) {
                     lastPos = new Point3d(node.getPosition());
                     lastPositions.put(node, lastPos);
                     diff += lastPos.norm();
                  } else {
                     diff += lastPos.distance(node.getPosition());
                     lastPos.set(node.getPosition());
                  }
               }
               diff = diff/fixer.getModel().numNodes();   // average change
            } // end looping through all fixers
            
            if (diff < convergenceThreshold) {
               Main.getMain().getScheduler().stopRequest();
            }
            
         } // end convergence check interval
      } // end autopause on
      
      // update reference models
      if (autoUpdateRefModels) {
         for (DynamicFemFixer fixer : myFixers) {
            fixer.updateModel();
         }
      }
      
   } // end function

   
   
}
