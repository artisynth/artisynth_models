package artisynth.tools.femtool;

import maspack.matrix.Point3d;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.modelbase.ControllerBase;

/**
 * Scales model from centroid to have a particular volume
 * 
 * @author Antonio
 *
 */
public class VolumeScalingController extends ControllerBase {

   public static boolean DEFAULT_ENABLED = true;
   public static double DEFAULT_SMOOTHING = 0;

   private double volume = -1;
   private double smoothing = DEFAULT_SMOOTHING;
   private boolean enabled = DEFAULT_ENABLED;
   private double oldRestVol = -1;

   private FemModel3d myFem = null;

   public static PropertyList myProps =
      new PropertyList(VolumeScalingController.class);
   static {
      myProps.add("enabled * *", "Controller enabled", DEFAULT_ENABLED);
      myProps.add("smoothing * *", "Smoothing coefficient", DEFAULT_SMOOTHING, "NW [0,1]");
      myProps.add("volume * *", "Volume", -1);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public void setFem(FemModel3d fem) {
      myFem = fem;
   }

   public VolumeScalingController(FemModel3d fem, double volume) {
      this.myFem = fem;
      this.volume = volume;      
   }

   public void setVolume(double volume) {
      this.volume = volume;
   }
   public double getVolume() {
      return volume;
   }
   public void setEnabled(boolean set) {
      enabled = set;
      if (set == false) {
         oldRestVol = -1;
      }
   }
   public boolean getEnabled() {
      return enabled;
   }
   public double getSmoothing() {
      return smoothing;
   }
   public void setSmoothing(double s) {
      if (s < 0) {
         s = 0;
      } else if (s > 1) {
         s = 1;
      }
      smoothing = s;
   }

   @Override
   public void initialize(double t0) {
      super.initialize(t0);
      oldRestVol = -1;
   }

   public void apply(double t0, double t1) {
      if (enabled && volume > 0) {

         double vol = myFem.updateVolume();
         double restVol = myFem.getRestVolume();
         double newRestVol = volume; //(volume/vol)*restVol;

         if (oldRestVol > 0) {
            newRestVol = oldRestVol*smoothing + newRestVol*(1-smoothing);
         }
         scaleRestVolume(Math.cbrt(newRestVol/restVol));
         oldRestVol = restVol;

      }
   }

   public void scaleRestVolume(double s) {

      double oldVol = myFem.getRestVolume();
      for (FemNode3d node : myFem.getNodes()) {
         Point3d pos = node.getRestPosition();
         pos.scale(s);
         node.setRestPosition(pos);
      }
      myFem.invalidateRestData();
      myFem.updateRestVolume();
      myFem.invalidateStressAndStiffness();
      double vol = myFem.getRestVolume();
//      if (vol/oldVol != s) {
//         System.out.println("hm...");
//      }
   }
   
   public static void addControls(
      ControlPanel controlPanel, VolumeScalingController controller) {

      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create(prop);
         controlPanel.addWidget(widget);
      }
   }



}
