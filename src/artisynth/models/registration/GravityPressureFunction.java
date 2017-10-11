package artisynth.models.registration;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 *  The pressure function is computed using a "Gravity" force between a source point and a 
 *  uniform solid target sphere,
 *  with the option to also be scaled down by the dot-product of normals (attempting
 *  to ensure normal consistency).
 *  
 *  Gravity outside a sphere scales with 1/distance^2, whereas gravity inside a sphere
 *  scales with distance.
 *  
 */
public class GravityPressureFunction extends RegistrationPressureFunctionBase {

   private static double DEFAULT_PRESSURE = 1000;
   private static double DEFAULT_RADIUS = 0.001;  // 1mm
   private static boolean DEFAULT_SCALE = false;
   
   double targetRadius;          // sphere radius
   double maxPressure;  // maximum pressure, when dist = rad
   boolean scaleNormalDotProduct;
   
   static {
      // register with parent so can be used in property panels
      registerSubClass(GravityPressureFunction.class);
   }
   
   public static PropertyList myProps = new PropertyList(GravityPressureFunction.class, RegistrationPressureFunctionBase.class);
   static {
      myProps.add("maxPressure * *", "maximum registration pressure", DEFAULT_PRESSURE);
      myProps.add("targetRadius * *", "radius of target gravity sphere", DEFAULT_RADIUS);
      myProps.add("scaleByNormalDotProduct isScalingByNormalDotProduct *", "radius of target gravity sphere", DEFAULT_SCALE);
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public GravityPressureFunction() {
      this(DEFAULT_PRESSURE, DEFAULT_RADIUS, DEFAULT_SCALE);
   }
   
   /**
    * Creates a gravity-based pressure function
    * @param maxPressure peak pressure, when distance is equal to point radius
    * @param targetRadius radius of target point, distance at which force drops off linearly
    * @param scaleNormalDotProduct scale computed pressure by dot product of normals
    */
   public GravityPressureFunction(double maxPressure, double targetRadius, boolean scaleNormalDotProduct) {
      this.targetRadius = targetRadius;
      this.maxPressure = maxPressure;
      this.scaleNormalDotProduct = scaleNormalDotProduct;
   }
   
   @Override
   public void computeCorrespondencePressure(
      Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget,
      Vector3d pressure) {
      
      pressure.sub(pTarget, pSource);
      
      double dist = pressure.norm();
      double pnorm;
      
      final double EPS = 1e-12;
      
      if (dist < targetRadius*EPS) {
         pressure.setZero();
         return;
      } else if (dist >= targetRadius) {
         // drop off with dist^2, peak reached when spheres are adjacent
         pnorm = maxPressure*targetRadius*targetRadius/(dist*dist); 
      } else {
         // linearly go to zero as we approach full overlap
         pnorm = maxPressure*dist/targetRadius;
      }
      
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
   
   public double getTargetRadius() {
      return targetRadius;
   }
   
   public void setTargetRadius(double r) {
      targetRadius = r;
   }

   public boolean isScalingByNormalDotProduct() {
      return scaleNormalDotProduct;
   }
   
   public void setScaleByNormalDotProduct(boolean set) {
      scaleNormalDotProduct = set;
   }
   
}
