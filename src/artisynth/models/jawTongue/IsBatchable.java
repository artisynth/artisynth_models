package artisynth.models.jawTongue;

/**
 * To perform batch muscle-driven simulations of a model, the root model should
 * implement this interface, which provides convenience methods to the Jython
 * script driving the simulations.
 */
public interface IsBatchable {

   /**
    * Adds an input probe which causes the excitation of the named tongue
    * exciter to increase from 0.0 to {@code maxExcitation} over 0.37
    * seconds, and then remain at {@code maxExcitation} until 1 second after the
    * start of the simulation.
    * 
    * @param exciterName
    * the name of the exciter for which an input probe should be created
    * @param maxExcitation
    * the maximum level of excitation of the named exciter
    */
   public void addExciterProbe (String exciterName, double maxExcitation); // {
//      if (getInputProbes ().get (exciterName + " exciter probe") == null) {
//         NumericInputProbe nip =
//            new NumericInputProbe (this, "<pathToExcitersList>/exciters/" + exciterName
//            + ":excitation", 0, 1);
//         nip.addData (
//            new double[] { 0.00, 0.0,
//                           0.03, 0.0,
//                           0.40, maxExcitation,
//                           1.00, maxExcitation
//                         }, NumericInputProbe.EXPLICIT_TIME);
//         nip.setName (exciterName + " exciter probe");
//         nip.setInterpolationOrder (Order.CubicStep);
//         addInputProbe (nip);
//      }
//   }
   /* ^^^ Suggested/recommended implementation. */

   /**
    * Removes the input probe of the named tongue exciter, if such a probe
    * exists.
    * 
    * @param exciterName
    * the name of the exciter for which the input probe should be removed
    */
   public void removeExciterProbe (String exciterName); // {
//      Probe p = getInputProbes().get (exciterName + " exciter probe");
//      if (p != null) {
//         removeInputProbe(p);
//      }
//   }
   /* ^^^ Suggested/recommended implementation. */
}
