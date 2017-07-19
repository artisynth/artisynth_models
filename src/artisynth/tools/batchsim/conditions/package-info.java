/**
 * Package containing {@link artisynth.tools.batchsim.conditions.Condition
 * Conditions}, {@link artisynth.tools.batchsim.conditions.ConditionChecker
 * ConditionCheckers}, the
 * {@link artisynth.tools.batchsim.conditions.ConditionMonitor
 * ConditionMonitor}, and the
 * {@link artisynth.tools.batchsim.conditions.StopConditionMonitor
 * StopConditionMonitor}.
 * <p>
 * Although the idea of {@code Conditions}, {@code ConditionCheckers}, and
 * {@code ConditionMonitors} could be widely applicable within {@code
 * ArtiSynth}, the motivation for their development stems from the desire to
 * use a {@code StopConditionMonitor} to provide a
 * {@link artisynth.core.workspace.RootModel RootModel}-independent way of
 * representing conditions under which a simulation should stop, and ensuring
 * that a simulation does stop when such a condition is met.
 * <p>
 * The best way to learn to use this package effectively is simply by reading
 * the JavaDocs of its contained classes and interfaces. Start with the
 * {@link artisynth.tools.batchsim.conditions.Condition Condition} and
 * {@link artisynth.tools.batchsim.conditions.ConditionChecker
 * ConditionChecker} interfaces (in that order). Then, move on to the
 * {@link artisynth.tools.batchsim.conditions.ConditionCheckerBase
 * ConditionCheckerBase},
 * {@link artisynth.tools.batchsim.conditions.ConditionMonitor
 * ConditionMonitor}, and
 * {@link artisynth.tools.batchsim.conditions.StopConditionMonitor
 * StopConditionMonitor} classes (in that order). Finally, have a look at the
 * various commonly-used {@code ConditionChecker} subclasses that have already
 * been implemented for convenience, and don't be afraid to implement new
 * {@code Conditions} and {@code ConditionCheckers} as needed.
 * 
 * @author Francois Roewer-Despres
 * @version 1.0
 */
package artisynth.tools.batchsim.conditions;