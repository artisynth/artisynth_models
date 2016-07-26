package artisynth.tools.femtool;

import java.util.HashMap;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.util.TimeBase;

public class DynamicFemFixerController extends ControllerBase {

   public static double DEFAULT_UPDATE_INTERVAL = 0.1;
   public static double DEFAULT_CHECK_CONVERGENCE_INTERVAL = 0.1;
   public static double DEFAULT_CONVERGENCE_THRESHOLD = 1e-5;
   public static boolean DEFAULT_AUTO_PAUSE = true;
   
   private double updateInterval = DEFAULT_UPDATE_INTERVAL;
   private double checkConvergenceInterval = DEFAULT_CHECK_CONVERGENCE_INTERVAL;
   private boolean autoPause = DEFAULT_AUTO_PAUSE;
   private double convergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;
   private DynamicFemFixer myFemFixer = null;
   
   boolean enabled = true;
   
   private HashMap<FemNode3d, Point3d> lastPositions;
   
   public static PropertyList myProps =
      new PropertyList(DynamicFemFixerController.class);

   static {
      myProps.add("enabled * *", "Controller enabled", true);
      myProps.add("updateInterval * *", "", DEFAULT_UPDATE_INTERVAL);
      myProps.add("checkConvergenceInterval * *", "", DEFAULT_CHECK_CONVERGENCE_INTERVAL);
      myProps.add("convergenceThreshold * *", "", DEFAULT_CONVERGENCE_THRESHOLD);
      myProps.add("autoPause * *", "", DEFAULT_AUTO_PAUSE);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public DynamicFemFixerController() {
      super();
      lastPositions = new HashMap<FemNode3d,Point3d>();
   }
   
   public DynamicFemFixerController(DynamicFemFixer fixer) {
      this();
      setFemFixer(fixer);
   }
   
   public DynamicFemFixerController(DynamicFemFixer fixer, double update) {
      this(fixer);
      updateInterval = update;
   }
       
   public void setUpdateInterval(double dt) {
      updateInterval = dt;
   }
   
   public double getUpdateInterval() {
      return updateInterval;
   }
   
   public void freezeSurface(boolean freeze) {
      myFemFixer.freezeSurface(freeze);
   }
   
   public void freezeInterior(boolean freeze) {
      myFemFixer.freezeInterior(freeze);
   }
   
   public void constrainSurface(boolean constrain) {
      if (constrain) {
         myFemFixer.constrainSurface(myFemFixer.getModel().getSurfaceMesh());
      } else {
         myFemFixer.removeMeshConstraints(myFemFixer.getModel().getSurfaceMesh());
      }
   }
   
   public void constrainSurface(PolygonalMesh mesh) {
      myFemFixer.constrainSurface(mesh);
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
   
   public void setFemFixer(DynamicFemFixer fixer) {
      myFemFixer = fixer;
      lastPositions.clear();
   }
     
   public void apply(double t0, double t1) {

      // exit if not enabled
      if (!enabled) {
         return;
      }
      
      if (updateInterval > 0) {
         if (TimeBase.modulo(t0, updateInterval)==0 ) { 
            myFemFixer.updateIdealRestPositions();
         }   
      }
      
      if (autoPause && checkConvergenceInterval > 0) {
         if (TimeBase.modulo(t1, checkConvergenceInterval)==0) {
            
            double diff = 0;
            for (FemNode3d node : myFemFixer.getModel().getNodes()) {
               
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
            diff = diff/myFemFixer.getModel().numNodes();   // average change
            
            if (diff < convergenceThreshold) {
               Main.getMain().getScheduler().stopRequest();
            }
            
         } // end convergence check interval
      } // end autopause on
      
      myFemFixer.updateModel();
      
   } // end function

   public boolean getEnabled() {
      return enabled;
   }
   public void setEnabled(boolean enable) {
      enabled = enable;
   }
   
   public static void addControls(
      ControlPanel controlPanel, DynamicFemFixerController controller) {
      
      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create (prop);
         controlPanel.addWidget(widget);
      }
   }
   
}
