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
public class GaussianWeightFunction extends RegistrationWeightFunction {

   private static double DEFAULT_WEIGHT = 1;
   private static double DEFAULT_SIGMA = 0.1;  // 10 cm
   private static boolean DEFAULT_SCALE_BY_COSINE = false;
   
   double maxWeight;
   double sigma;
   boolean scaleByCosine;
   
   static {
      // register with parent so can be used in property panels
      registerSubClass(GaussianWeightFunction.class);
   }
   
   public static PropertyList myProps = new PropertyList(GaussianWeightFunction.class, RegistrationWeightFunction.class);
   static {
      myProps.add("maxWeight * *", "maximum registration Weight", DEFAULT_WEIGHT);
      myProps.add("sigma * *", "Gaussian sigma", DEFAULT_SIGMA);
      myProps.add("scaleByCosine isScalingByCosine *", "scaling by cosine of angle between normals", DEFAULT_SCALE_BY_COSINE);
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public GaussianWeightFunction() {
      this(DEFAULT_WEIGHT, DEFAULT_SIGMA, DEFAULT_SCALE_BY_COSINE);
   }
   
   public GaussianWeightFunction(double maxWeight, double sigma, boolean scaleByCosine) {
      this.maxWeight = maxWeight;
      this.sigma = sigma;
      this.scaleByCosine = scaleByCosine;
   }
   
   @Override
   public double computeWeight(
      Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget) {
      
      double dist = pTarget.distance (pSource);
      
      double w = Math.exp(-dist*dist/sigma*sigma); // Gaussian with unit peak         
      w *= maxWeight;   // scale up normalized log-normal to have specified peak Weight
      
      // scale by dot product of normals if desired
      if (scaleByCosine) {
         double nd = nSource.dot(nTarget);
         w *= Math.max(nd, 0);
      }
      
      return w;
   }
   
   public double getMaxWeight() {
      return maxWeight;
   }
   
   public void setMaxWeight(double mp) {
      maxWeight = mp;
   }
   
   public double getSigma() {
      return sigma;
   }
   
   public void setSigma(double s) {
      sigma = s;
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
