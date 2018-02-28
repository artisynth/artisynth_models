package artisynth.models.registration;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 *  The pressure function is computed using a scaled Gaussian function of distance
 *  outside a given radius.  Within the "snap" radius, force drops off linearly.
 *  with the option to also be scaled down by the dot-product of normals (attempting
 *  to ensure normal consistency)
 *  
 */
public class GaussianPressureFunction extends RegistrationPressureFunctionBase {

   private static double DEFAULT_PRESSURE = 1000;
   private static double DEFAULT_SIGMA = 0.1;  // 10 cm
   private static double DEFAULT_SNAP = 0.001; // 1 mm
   private static boolean DEFAULT_SCALE = false;
   
   double maxPressure;
   double sigma;
   double snap;
   boolean scaleNormalDotProduct;
   
   static {
      // register with parent so can be used in property panels
      registerSubClass(GaussianPressureFunction.class);
   }
   
   public static PropertyList myProps = new PropertyList(GaussianPressureFunction.class, RegistrationPressureFunctionBase.class);
   static {
      myProps.add("maxPressure * *", "maximum registration pressure", DEFAULT_PRESSURE);
      myProps.add("sigma * *", "Gaussian sigma", DEFAULT_SIGMA);
      myProps.add("snapDistance * *", "Registration snap distance", DEFAULT_SNAP);
      myProps.add("scaleByNormalDotProduct isScalingByNormalDotProduct *", "radius of target gravity sphere", DEFAULT_SCALE);
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public GaussianPressureFunction() {
      this(DEFAULT_PRESSURE, DEFAULT_SIGMA, DEFAULT_SNAP, DEFAULT_SCALE);
   }
   
   public GaussianPressureFunction(double maxPressure, double sigma, double snap, boolean scaleByNormalDotProduct) {
      this.maxPressure = maxPressure;
      this.sigma = sigma;
      this.snap = snap;
      this.scaleNormalDotProduct = scaleByNormalDotProduct;
   }
   
   @Override
   public void computeCorrespondencePressure(
      Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget,
      Vector3d pressure) {
      
      pressure.sub(pTarget, pSource);
      
      double dist = pressure.norm();
      double d = dist-snap;
      
      double pnorm;
      
      final double EPS = 1e-12;
      
      if (dist <= EPS*snap) {
         pressure.setZero();
         return;
      } else if (dist < snap) {
         pnorm = d/snap; // linear regime
      } else {
         pnorm = Math.exp(-d*d/sigma*sigma); // log-normal scaled to have unit peak         
      }
      pnorm *= maxPressure;   // scale up normalized log-normal to have specified peak pressure
      
      // scale by dot product of normals if desired
      if (scaleNormalDotProduct) {
         double nd = nSource.dot(nTarget);
         pnorm *= Math.max(nd, 0);
      }
      
      pressure.scale(pnorm/dist);
      
   }
   
   public double getMaxPressure() {
      return maxPressure;
   }
   
   public void setMaxPressure(double mp) {
      maxPressure = mp;
   }
   
   public double getSigma() {
      return sigma;
   }
   
   public void setSigma(double s) {
      sigma = s;
   }
   
   /**
    * Distance within which we are assumed to be snapped to the edge
    */
   public double getSnapDistance() {
      return snap;
   }
   
   public void setSnapDistance(double s) {
      snap = s;
   }

   public boolean isScalingByNormalDotProduct() {
      return scaleNormalDotProduct;
   }
   
   public void setScaleByNormalDotProduct(boolean set) {
      scaleNormalDotProduct = set;
   }

}
