package artisynth.models.registration;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 *  The pressure function is computed using a scaled Log-Gaussian function of distance,
 *  with the option to also be scaled down by the dot-product of normals (attempting
 *  to ensure normal consistency)
 *  
 */
public class LogNormalPressureFunction extends RegistrationPressureFunctionBase {

   private static double DEFAULT_PRESSURE = 1000;
   private static double DEFAULT_SIGMA = 0.1;  // 10 cm
   private static double DEFAULT_MU = 0.001;   // 1 mm
   private static boolean DEFAULT_SCALE = false;
   
   double maxPressure;
   double sigma;
   double mu;
   boolean scaleNormalDotProduct;
   
   static {
      // register with parent so can be used in property panels
      registerSubClass(LogNormalPressureFunction.class);
   }
   
   public static PropertyList myProps = new PropertyList(LogNormalPressureFunction.class, RegistrationPressureFunctionBase.class);
   static {
      myProps.add("maxPressure * *", "maximum registration pressure", DEFAULT_PRESSURE);
      myProps.add("sigma * *", "Gaussian sigma", DEFAULT_SIGMA);
      myProps.add("mu * *", "Gaussian mu", DEFAULT_MU);
      myProps.add("scaleByNormalDotProduct isScalingByNormalDotProduct *", "radius of target gravity sphere", DEFAULT_SCALE);
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public LogNormalPressureFunction() {
      this(DEFAULT_PRESSURE, DEFAULT_SIGMA, DEFAULT_MU, DEFAULT_SCALE);
   }
   
   public LogNormalPressureFunction(double maxPressure, double sigma, double mu, boolean scaleByNormalDotProduct) {
      this.maxPressure = maxPressure;
      this.sigma = sigma;
      this.mu = mu;
      this.scaleNormalDotProduct = scaleByNormalDotProduct;
   }
   
   @Override
   public void computeCorrespondencePressure(
      Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget,
      Vector3d pressure) {
      
      pressure.sub(pTarget, pSource);
      
      double dist = pressure.norm();
      
      double emu = Math.exp(mu);
      final double EPS = 1e-12;
      
      if (dist <= EPS*emu) {
         pressure.setZero();
         return;
      }
       
      double a = (Math.log(dist)-mu)/sigma;
      double pnorm = (emu/dist)*Math.exp(-a*a/2); // log-normal scaled to have unit peak         
      pnorm *= maxPressure;   // scale up normalized log-normal to have specified peak pressure
      
      // scale by dot product of normals if desired
      if (scaleNormalDotProduct) {
         double d = nSource.dot(nTarget);
         pnorm *= Math.max(d, 0);
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
   
   public double getMu() {
      return mu;
   }
   
   public void setMu(double m) {
      mu = m;
   }

   public boolean isScalingByNormalDotProduct() {
      return scaleNormalDotProduct;
   }
   
   public void setScaleByNormalDotProduct(boolean set) {
      scaleNormalDotProduct = set;
   }

}
