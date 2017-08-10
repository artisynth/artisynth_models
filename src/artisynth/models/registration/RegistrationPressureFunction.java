package artisynth.models.registration;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/**
 * Computes a desired pressure (N/d^2 - where d is the current distance unit) to be applied on the
 * source model surface to drive it towards the target.
 * 
 * @author Antonio
 *
 */
public interface RegistrationPressureFunction {
   
   /**
    * Computes the desired pressure between two points of correspondence
    * @param pSource point on the source surface
    * @param nSource normal on the source surface
    * @param pTarget point on the target surface
    * @param nTarget normal on the target surface
    * @param pressure output pressure
    */
   public void computeCorrespondencePressure(Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget, Vector3d pressure);

}
