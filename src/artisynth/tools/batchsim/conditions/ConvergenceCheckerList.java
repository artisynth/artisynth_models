package artisynth.tools.batchsim.conditions;

import java.util.LinkedList;
import java.util.List;

/**
 * An {@code ConvergenceCheckerList} checks whether <b>every</b>
 * {@link ConvergenceChecker} it holds detects some convergence.
 * 
 * @author Francois Roewer-Despres
 */
public class ConvergenceCheckerList
extends ConditionCheckerBase<EmptyCondition> {

   /**
    * An {@code ObjectHolder} holds some {@link Object}, which can be directly
    * accessed via its {@link #value} field.
    *
    * @author Francois Roewer-Despres
    */
   public static class ObjectHolder {

      /** The value that this {@link ObjectHolder} is holding. */
      public Object value;

      /**
       * Creates a new {@link ObjectHolder} with {@link #value} set to
       * {@code null}.
       */
      public ObjectHolder () {
         this (null);
      }

      /**
       * Creates a new {@link ObjectHolder} with {@link #value} set to the given
       * value.
       *
       * @param val
       * the value
       */
      public ObjectHolder (Object val) {
         value = val;
      }
   }

   /**
    * A {@link ConvergenceChecker} checks that some object converges over time.
    *
    * @author Francois Roewer-Despres
    */
   public static interface ConvergenceChecker {
      /**
       * Returns {@code true} if, and only if, some {@link Object} has
       * converged.
       * <p>
       * During each time step, the value of the object during the previous time
       * step is given as {@code prev}. This value should be compared to the
       * current value to determine if the object has converged.
       * <p>
       * Whether or not the object has converged, the current value of the
       * object should be placed in the {@link ObjectHolder#value} field of
       * {@code current}.
       * <p>
       * If this is the very first time step, {@code first} will be {@code true}
       * and {@code prev} will be {@code null}. If {@code first} is
       * {@code false}, {@code prev} will be {@code null} if, and only if, the
       * value of {@code current} was {@code null} in the previous time step.
       *
       * @param first
       * {@code true} if, and only if, this is the very first time step.
       * @param prev
       * the value of the tracked object in the previous time step, or
       * {@code null} if this is the very first time step
       * @param current
       * the value of the object during this time step
       * @return whether or not the object has converged between the previous
       * time step and the current one
       */
      boolean checkConvergence (
         boolean first, Object prev, ObjectHolder current);
   }

   /**
    * A {@code PrevCurrentCheckerHolder} is simply a struct that holds a
    * {@link ConvergenceChecker} as well as its previous and current object.
    *
    * @author Francois Roewer-Despres
    */
   protected static class PrevCurrentCheckerHolder {

      /** The previous object of the {@link ConvergenceChecker}. */
      public Object prev = null;

      /** The current object of the {@link ConvergenceChecker}. */
      public ObjectHolder current;

      /** The {@link ConvergenceChecker}. */
      public ConvergenceChecker checker;

      /**
       * Creates a new {@link PrevCurrentCheckerHolder} with the given
       * {@link ConvergenceChecker}. The previous and current objects are set to
       * {@code null}.
       *
       * @param checker
       * the {@code ConvergenceChecker} to hold
       */
      public PrevCurrentCheckerHolder (ConvergenceChecker checker) {
         current = new ObjectHolder ();
         this.checker = checker;
      }
   }

   /** A list of {@link ConvergenceChecker}s to check. */
   protected List<PrevCurrentCheckerHolder> myConvCheckers;

   /**
    * Creates a new {@link ConvergenceCheckerList} with the given name and
    * {@link ConvergenceChecker}s.
    *
    * @param name
    * the name of this {@code ConvergenceCheckerList}
    * @param checkers
    * the {@code ConvergenceChecker}s to check
    */
   public ConvergenceCheckerList (String name, ConvergenceChecker... checkers) {
      this (name, null, checkers);
   }

   /**
    * Creates a new {@link ConvergenceCheckerList} with the given nestedChecker
    * and {@link ConvergenceChecker}s.
    *
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code ConvergenceCheckerList}
    * @param checkers
    * the {@code ConvergenceChecker}s to check
    */
   public ConvergenceCheckerList (
   ConditionChecker<? extends Condition> nestedChecker,
   ConvergenceChecker... checkers) {
      this (null, nestedChecker, checkers);
   }

   /**
    * Creates a new {@link ConvergenceCheckerList} with the given name,
    * nestedChecker, and {@link ConvergenceChecker}s.
    *
    * @param name
    * the name of this {@code ConvergenceCheckerList}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code ConvergenceCheckerList}
    * @param checkers
    * the {@code ConvergenceChecker}s to check
    */
   public ConvergenceCheckerList (String name,
   ConditionChecker<? extends Condition> nestedChecker,
   ConvergenceChecker... checkers) {
      super (name, null, nestedChecker);
      myConvCheckers = new LinkedList<> ();
      for (ConvergenceChecker checker : checkers) {
         myConvCheckers.add (new PrevCurrentCheckerHolder (checker));
      }
   }

   @Override
   protected boolean checkCondition (
      EmptyCondition cond, double t0, double t1) {
      boolean reset = t0 == 0;
      boolean result = true;
      for (PrevCurrentCheckerHolder pair : myConvCheckers) {
         if (reset) {
            pair.prev = null;
         }
         pair.current.value = null;
         result &=
            pair.checker.checkConvergence (reset, pair.prev, pair.current);
         pair.prev = pair.current.value;
      }
      return result;
   }

}
