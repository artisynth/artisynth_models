package artisynth.models.jawTongue;

import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.Probe;
import maspack.interpolation.Interpolation.Order;

public class BadinJawHyoidTongueContact extends BadinJawHyoidTongue
{
   public BadinJawHyoidTongueContact() {
      super();
   }

   public void addExciterProbe(String exciterName, double maxExcitation) {
    if (getInputProbes ().get (exciterName + " exciter probe") == null) {
    NumericInputProbe nip =
       new NumericInputProbe(this, "models/jawmodel/models/tongue/exciters/" + exciterName
       + ":excitation", 0, 1);
    nip.addData (
       new double[] { 0.00, 0.0,
                      0.03, 0.0,
                      0.40, maxExcitation,
                      1.00, maxExcitation
                    }, NumericInputProbe.EXPLICIT_TIME);
    nip.setName (exciterName + " exciter probe");
    nip.setInterpolationOrder (Order.CubicStep);
    addInputProbe (nip);
 }
}

   public void removeExciterProbe(String exciterName) {
      Probe p = getInputProbes().get(exciterName + " exciter probe");
      if (p != null) {
         removeInputProbe(p);
      }
   }
}
