package artisynth.models.registration;

import artisynth.core.materials.AxialMaterial;
import artisynth.core.materials.LinearAxialMaterial;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 *  The pressure function is computed using a spring force,
 *  with the option to also be scaled down by the dot-product of normals (attempting
 *  to ensure normal consistency)
 *  
 */
public class SpringPressureFunction extends RegistrationPressureFunctionBase {

   private static boolean DEFAULT_SCALE = false;
   
   AxialMaterial mat;  // spring material
   boolean scaleNormalDotProduct;  // scale by normals
   
   static {
      // register with parent so can be used in property panels
      registerSubClass(SpringPressureFunction.class);
   }
   
   public static PropertyList myProps = new PropertyList(SpringPressureFunction.class, RegistrationPressureFunctionBase.class);
   static {
      myProps.add("material * *", "spring material", createDefaultMaterial());
      myProps.add("scaleByNormalDotProduct isScalingByNormalDotProduct *", "radius of target gravity sphere", DEFAULT_SCALE);
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   private static AxialMaterial createDefaultMaterial() {
      return new LinearAxialMaterial();
   }
   
   public SpringPressureFunction() {
      this(createDefaultMaterial(), DEFAULT_SCALE);
   }
   
   public SpringPressureFunction(AxialMaterial mat, boolean scaleByNormalDotProduct) {
      this.mat = mat;
      this.scaleNormalDotProduct = scaleByNormalDotProduct;
   }
   
   @Override
   public void computeCorrespondencePressure(
      Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget,
      Vector3d pressure) {
      
      pressure.sub(pTarget, pSource);
      
      double dist = pressure.norm();
      if (dist <= 0) {
         pressure.setZero();
         return;
      }
      
      double pnorm = mat.computeF(dist, 0, 0, 0);
      
      // scale by dot product of normals if desired
      if (scaleNormalDotProduct) {
         double d = nSource.dot(nTarget);
         pnorm *= Math.max(d, 0);
      }
      
      pressure.scale(pnorm/dist);
      
   }
   
   public boolean isScalingByNormalDotProduct() {
      return scaleNormalDotProduct;
   }
   
   public void setScaleByNormalDotProduct(boolean set) {
      scaleNormalDotProduct = set;
   }
   
   public AxialMaterial getMaterial() {
      return mat;
   }
   
   public void setMaterial(AxialMaterial mat) {
      this.mat = mat;
   }

}
