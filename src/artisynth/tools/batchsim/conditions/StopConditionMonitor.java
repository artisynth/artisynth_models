package artisynth.tools.batchsim.conditions;

import artisynth.core.workspace.RootModel;

/**
 * A {@code StopConditionMonitor} is a {@link ConditionMonitor} that
 * additionally calls {@link RootModel#setStopRequest(boolean)
 * RootModel.setStopRequest(true)} in its {@link #apply(double, double)} method
 * if, and only if, any of its {@link ConditionChecker}s' {@link Condition} is
 * met.
 * 
 * @author Francois Roewer-Despres
 */
public class StopConditionMonitor extends ConditionMonitor {

   @Override
   public void apply (double t0, double t1) {
      super.apply (t0, t1);
      if (hasAnyConditionBeenMet ()) {
         ((RootModel) getGrandParent()).setStopRequest (true);
      }
   }

}
