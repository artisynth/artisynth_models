package artisynth.models.registration.weights;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 *  The Weight function is computed using a scaled Gaussian function of distance
 *  outside a given radius.  Within the "snap" radius, force drops off linearly.
 *  with the option to also be scaled down by the dot-product of normals (attempting
 *  to ensure normal consistency)
 *  
 */
public class ConstantWeightFunction extends RegistrationWeightFunction {

   private static double DEFAULT_WEIGHT = 1;
   private static boolean DEFAULT_SCALE_BY_COSINE = false;
   
   double maxWeight;
   boolean scaleByCosine;
   
   static {
      // register with parent so can be used in property panels
      registerSubClass(ConstantWeightFunction.class);
   }
   
   public static PropertyList myProps = new PropertyList(ConstantWeightFunction.class, RegistrationWeightFunction.class);
   static {
      myProps.add("value * *", "constant registration Weight", DEFAULT_WEIGHT);
      myProps.add("scaleByCosine isScalingByCosine *", "scaling by cosine of angle between normals", DEFAULT_SCALE_BY_COSINE);
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public ConstantWeightFunction() {
      this(DEFAULT_WEIGHT, DEFAULT_SCALE_BY_COSINE);
   }
   
   public ConstantWeightFunction(double v) {
      this(v, DEFAULT_SCALE_BY_COSINE);
   }
   
   public ConstantWeightFunction(double value, boolean scaleByCosine) {
      this.maxWeight = value;
      this.scaleByCosine = scaleByCosine;
   }
   
   @Override
   public double computeWeight(
      Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget) {
      
      double w = maxWeight;
      
      // scale by dot product of normals if desired
      if (scaleByCosine) {
         double nd = nSource.dot(nTarget);
         w *= Math.max(nd, 0);
      }
      
      return w;
   }
   
   /**
    * Gets the constant weight value
    * @return weight
    */
   public double getValue() {
      return maxWeight;
   }
   
   /**
    * Gets constant weight value
    * @param v weight value
    */
   public void setValue(double v) {
      maxWeight = v;
   }
      
   /**
    * Scales by the dot-product of normals 
    * @return true if scales by normals, false otherwise
    */
   public boolean isScalingByCosine() {
      return scaleByCosine;
   }
   
   /**
    * Enables or disables scaling by dot-product of normals
    * @param set true to enable scaling, false otherwise
    */
   public void setScaleByCosine(boolean set) {
      scaleByCosine = set;
   }

}
