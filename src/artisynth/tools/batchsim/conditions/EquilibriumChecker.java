package artisynth.tools.batchsim.conditions;

import java.util.List;
import java.util.Objects;

import artisynth.core.modelbase.ModelComponent;
import maspack.matrix.Vector;
import maspack.properties.Property;

/**
 * An {@code EquilibriumChecker} checks whether the velocity of the model
 * components provided to it meet a particular {@link EquilibriumCondition}.
 * 
 * @author Francois Roewer-Despres
 */
public class EquilibriumChecker
extends ConditionCheckerBase<EquilibriumChecker.EquilibriumCondition> {

   /**
    * An equilibrium refers to the state of a given model component in which the
    * sum of the forces acting upon it is 0 (no net force).
    * <p>
    * An equilibrium is static, if the given model component is not moving
    * (velocity of 0). An equilibrium is dynamic, if the given model component
    * is not accelerating/decelerating (constant velocity).
    * 
    * @author Francois Roewer-Despres
    */
   public static enum EquilibriumCondition implements Condition {
      STATIC; // DYNAMIC not yet implemented.
   }

   protected double delta;
   protected List<ModelComponent> comps;

   /**
    * Creates a new {@link EquilibriumChecker} initialized with the given
    * parameter values.
    * 
    * @param cond
    * the initial {@link EquilibriumCondition} of this
    * {@code EquilibriumChecker}
    * @param velocityDelta
    * a threshold value: velocity differences that are less than this value will
    * be considered "not changing" (equal to some constant, perhaps 0)
    * @param components
    * a list of {@link ModelComponent}s for which the {@code "velocity"}
    * {@link Property} should be checked for an equilibrium state
    * @throws IllegalArgumentException
    * if any of the given {@code ModelComponents} do not have a {@code Property}
    * called {@code "velocity"}
    */
   public EquilibriumChecker (EquilibriumCondition cond, double velocityDelta,
   List<ModelComponent> components) throws IllegalArgumentException {
      this (null, cond, null, velocityDelta, components);
   }

   /**
    * Creates a new {@link EquilibriumChecker} initialized with the given
    * parameter values.
    * 
    * @param name
    * the name of this {@code EquilibriumChecker}
    * @param cond
    * the initial {@link EquilibriumCondition} of this
    * {@code EquilibriumChecker}
    * @param velocityDelta
    * a threshold value: velocity differences that are less than this value will
    * be considered "not changing" (equal to some constant, perhaps 0)
    * @param components
    * a list of {@link ModelComponent}s for which the {@code "velocity"}
    * {@link Property} should be checked for an equilibrium state
    * @throws IllegalArgumentException
    * if any of the given {@code ModelComponents} do not have a {@code Property}
    * called {@code "velocity"}
    */
   public EquilibriumChecker (String name, EquilibriumCondition cond,
   double velocityDelta, List<ModelComponent> components)
   throws IllegalArgumentException {
      this (name, cond, null, velocityDelta, components);
   }

   /**
    * Creates a new {@link EquilibriumChecker} initialized with the given
    * parameter values.
    * 
    * @param cond
    * the initial {@link EquilibriumCondition} of this
    * {@code EquilibriumChecker}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code EquilibriumChecker}
    * @param velocityDelta
    * a threshold value: velocity differences that are less than this value will
    * be considered "not changing" (equal to some constant, perhaps 0)
    * @param components
    * a list of {@link ModelComponent}s for which the {@code "velocity"}
    * {@link Property} should be checked for an equilibrium state
    * @throws IllegalArgumentException
    * if any of the given {@code ModelComponents} do not have a {@code Property}
    * called {@code "velocity"}
    */
   public EquilibriumChecker (EquilibriumCondition cond,
   ConditionChecker<?> nestedChecker, double velocityDelta,
   List<ModelComponent> components) throws IllegalArgumentException {
      this (null, cond, nestedChecker, velocityDelta, components);
   }

   /**
    * Creates a new {@link EquilibriumChecker} initialized with the given
    * parameter values.
    * 
    * @param name
    * the name of this {@code EquilibriumChecker}
    * @param cond
    * the initial {@link EquilibriumCondition} of this
    * {@code EquilibriumChecker}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code EquilibriumChecker}
    * @param velocityDelta
    * a threshold value: velocity differences that are less than this value will
    * be considered "not changing" (equal to some constant, perhaps 0)
    * @param components
    * a list of {@link ModelComponent}s for which the {@code "velocity"}
    * {@link Property} should be checked for an equilibrium state
    * @throws IllegalArgumentException
    * if any of the given {@code ModelComponents} do not have a {@code Property}
    * called {@code "velocity"}
    */
   public EquilibriumChecker (String name, EquilibriumCondition cond,
   ConditionChecker<?> nestedChecker, double velocityDelta,
   List<ModelComponent> components) throws IllegalArgumentException {
      super (name, cond, nestedChecker);
      delta = velocityDelta;
      for (ModelComponent comp : components) {
         if (comp.getProperty ("velocity") == null) {
            throw new IllegalArgumentException (
               "The provided model components must have a \"velocity\" property.");
         }
      }
      comps = components;
   }

   @Override
   protected boolean checkCondition (
      EquilibriumCondition cond, double t0, double t1) {
      switch (cond) {
         case STATIC:
            for (ModelComponent comp : comps) {
               Vector vel = (Vector)comp.getProperty ("velocity").get ();
               if (vel.norm () > delta) {
                  return false;
               }
            }
            return true;
         default:
            throw new UnsupportedOperationException (Objects.toString (cond));
      }
   }
}
