package artisynth.models.registration.old.pressures;

import org.python.modules.math;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 *  The pressure function is computed using a scaled Log-Gaussian function of distance,
 *  with the option to also be scaled down by the dot-product of normals (attempting
 *  to ensure normal consistency)
 *  
 */
public class LogNormalPressureFunction extends RegistrationPressureFunction {

   private static double DEFAULT_PRESSURE = 1000;
   private static double DEFAULT_SIGMA = 0.1;  // 10 cm
   private static double DEFAULT_MU = 0.001;   // 1 mm
   private static boolean DEFAULT_SCALE = false;
   
   double maxPressure;
   double sigma;
   double mu;
   boolean scaleCosine;
   
   static {
      // register with parent so can be used in property panels
      registerSubClass(LogNormalPressureFunction.class);
   }
   
   public static PropertyList myProps = new PropertyList(LogNormalPressureFunction.class, RegistrationPressureFunction.class);
   static {
      myProps.add("maxPressure * *", "maximum registration pressure", DEFAULT_PRESSURE);
      myProps.add("sigma * *", "Gaussian sigma", DEFAULT_SIGMA);
      myProps.add("mu * *", "Gaussian mu", DEFAULT_MU);
      myProps.add ("peak * *", "peak pressure distance", Math.exp (DEFAULT_MU - DEFAULT_SIGMA*DEFAULT_SIGMA));
      myProps.add ("halfPeak * *", "half-peak pressure distance", Math.exp (DEFAULT_MU - DEFAULT_SIGMA*DEFAULT_SIGMA + DEFAULT_SIGMA*Math.sqrt (2*Math.log (2))));
      myProps.add("scaleByCosine isScalingByCosine *", "scale by dot product of normals", DEFAULT_SCALE);
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public LogNormalPressureFunction() {
      this(DEFAULT_PRESSURE, DEFAULT_SIGMA, DEFAULT_MU, DEFAULT_SCALE);
   }
   
   public LogNormalPressureFunction(double maxPressure, double sigma, double mu, boolean scaleByCosine) {
      this.maxPressure = maxPressure;
      this.sigma = sigma;
      this.mu = mu;
      this.scaleCosine = scaleByCosine;
   }
   
   @Override
   public void computePressure(
      Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget,
      Vector3d pressure) {
      
      pressure.sub(pTarget, pSource);
      
      double dist = pressure.norm();
      
      double xpeak =Math.exp (mu - sigma*sigma);
      final double EPS = 1e-12;
      
      if (dist <= EPS*xpeak) {
         pressure.setZero();
         return;
      }
       
      double a = (Math.log(dist)-mu)/sigma;
      double b = Math.exp (mu - sigma*sigma/2);
      
      double pnorm = (b/dist)*Math.exp(-a*a/2); // log-normal scaled to have unit peak         
      pnorm *= maxPressure;   // scale up normalized log-normal to have specified peak pressure
      
      // scale by dot product of normals if desired
      if (scaleCosine) {
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
   
   /**
    * Determines values of mu and sigma that results in peak pressure and
    * half of peak pressure at the provided distances
    * 
    * @param p peak pressure distance
    * @param hp distance of half peak pressure
    */
   public void setPeakHalfPeak(double p, double hp) {
      
      // y1 = (mu - sigma^2) + sigma*sqrt(-2log(1)) = log(p)
      // y2 = (mu - sigma^2) + sigma*sqrt(-2log(0.5)) = log(hp)
      double y1 = Math.log (p);
      double y2 = Math.log (hp);
      
      setLogPeakHalfPeak(y1, y2);
      
   }
   
   /**
    * Sets the parameters mu, sigma such that the log-normal has 
    * peak pressure at distance exp(y1) and half of peak pressure
    * at distance exp(y2)
    * @param y1 log of peak distance
    * @param y2 log of half-peak distance
    */
   private void setLogPeakHalfPeak(double y1, double y2) {
      double b = 2*Math.log(2);
      double dy = Math.abs (y2 - y1);
      double sigma = dy/Math.sqrt(b);
      double mu = y1 + sigma*sigma;
      setSigma (sigma);
      setMu(mu);
   }
   
   /**
    * Determines value of mu and sigma that results in peak pressure at distance p.
    * For this computation, the half-peak remains fixed.
    * 
    * @param p peak pressure distance
    */
   public void setPeak(double p) {
      
      double y1 = Math.log (p);
      double y2 = mu - sigma*sigma + sigma*math.sqrt (2*Math.log (2));
      
      setLogPeakHalfPeak (y1, y2);
   }
   
   /**
    * Determines the values of mu and sigma that result in half-peak pressure at distance hp.
    * For this computation, the peak remains fixed
    * 
    * @param hp distance of half of peak pressure
    */
   public void setHalfPeak(double hp) {
      // y1 = (mu - sigma^2) + sigma*sqrt(-2log(1)) = log(p)
      // y2 = (mu - sigma^2) + sigma*sqrt(-2log(0.5)) = log(hp)
      double y1 = mu - sigma*sigma;
      double y2 = Math.log (hp);
      setLogPeakHalfPeak (y1, y2);
   }
   
   /**
    * Distance at which peak pressure is achieved
    * @return peak pressure distance
    */
   public double getPeak() {
      return Math.exp (mu - sigma*sigma);
   }
   
   /**
    * Distance at which half of peak pressure is achieved
    * @return half-peak pressure distance
    */
   public double getHalfPeak() {
      double y2 = mu - sigma*sigma + sigma*math.sqrt (2*Math.log (2));
      return Math.exp (y2);
   }
   
   /**
    * Scale by cosine between normals
    * @return true if scaling enabled
    */
   public boolean isScalingByCosine() {
      return scaleCosine;
   }
   
   /**
    * Enables/disables scaling by normals
    * @param set
    */
   public void setScaleByCosine(boolean set) {
      scaleCosine = set;
   }

}
