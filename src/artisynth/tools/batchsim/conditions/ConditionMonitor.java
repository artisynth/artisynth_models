package artisynth.tools.batchsim.conditions;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import artisynth.core.modelbase.Monitor;
import artisynth.core.modelbase.MonitorBase;

/**
 * A {@code ConditionMonitor} is a {@link Monitor} that checks if any of its
 * {@link ConditionChecker}s' {@link Condition} is met during its
 * {@link #apply(double, double)} method, and reports this information.
 * <p>
 * When nesting {@code ConditionCheckers}, the resulting compound
 * {@code Condition} is true if, and only if, both the outer and inner (i.e.
 * nested) {@code ConditionChecker}'s {@code Condition} is true. That is,
 * <b>nesting {@code ConditionCheckers} is the equivalent of applying the
 * logical {@code AND} function to their respective {@code Condition}</b>. The
 * nesting can be done repeatedly (i.e. one {@code ConditionChecker} nested
 * inside another that is itself nested inside a third, and so on). This creates
 * a compound {@code Condition} that is composed of a chain of {@code AND}'ed
 * {@code Conditions}.
 * <p>
 * Sometimes, however, an application requires taking the logical {@code OR} of
 * one or more {@code Conditions}. In this case, the {@code Conditions}'
 * respective {@code ConditionCheckers} should not be nested. Rather, they
 * should be separately added to a {@code ConditionMonitor}, since in its
 * {@code apply(double, double)} method, a {@code ConditionMonitor} asks all its
 * {@code ConditionCheckers} to check whether their respective {@code Condition}
 * has been met, and each will verify and report its {@code Conditional} state
 * <i>independently</i> of the other {@code ConditionCheckers}. That is,
 * <b>adding separate (i.e. non-nested) {@code ConditionCheckers} to a
 * {@code ConditionMonitor} is the equivalent of applying the logical {@code OR}
 * function to the {@code ConditionCheckers}' respective {@code Condition}</b>.
 * <p>
 * By nesting some {@code ConditionCheckers}, but not others, and then adding
 * all the resulting "outermost" {@code ConditionCheckers} to a
 * {@code ConditionMonitor}, a complex Boolean {@code Conditional} expression
 * can be created. When combined with the {@link NotChecker}, every possible
 * Boolean {@code Conditional} expression can be created.
 * 
 * @author Francois Roewer-Despres
 */
public class ConditionMonitor extends MonitorBase {

   /**
    * A list of {@link Condition}s that are met as of the last call to
    * {@link #apply(double, double)}.
    */
   protected List<Condition> myMetConditions;

   /**
    * A list of {@link ConditionChecker}s whose {@link Condition} is checked
    * during each call to {@link #apply(double, double)}.
    */
   protected List<ConditionChecker<?>> myCheckers;

   /**
    * Creates a new {@link ConditionMonitor} initialized with no
    * {@link ConditionChecker}s.
    */
   public ConditionMonitor () {
      this (null);
   }

   /**
    * Creates a new {@link ConditionMonitor} initialized with the given
    * {@link ConditionChecker}.
    */
   public ConditionMonitor (ConditionChecker<?> checker) {
      myCheckers = new ArrayList<> ();
      myMetConditions = new ArrayList<> ();
      addConditionChecker (checker);
   }

   /**
    * Determines whether any {@link ConditionChecker}s' {@link Condition} has
    * been met during the last call to {@link #apply(double, double)}.
    * 
    * @return true if any {@code Condition} has been met; false otherwise
    */
   public synchronized boolean hasAnyConditionBeenMet () {
      return myMetConditions.size () > 0;
   }

   /**
    * Determines whether the given {@link Condition} has been met during the
    * last call to {@link #apply(double, double)}.
    * 
    * @param cond
    * the {@code Condition} to test
    * @return true if the given {@link Condition} has been met; false otherwise
    */
   public synchronized boolean hasConditionBeenMet (Condition cond) {
      return myMetConditions.contains (cond);
   }

   /**
    * Returns a list of all {@link Condition}s that have been met during the
    * last call to {@link #apply(double, double)}.
    * 
    * @return a list of all {@code Conditions} that have been met
    */
   public synchronized List<Condition> getMetConditions () {
      return new LinkedList<> (myMetConditions);
   }

   /**
    * Adds the given {@link ConditionChecker} to this {@link ConditionMonitor}'s
    * list of {@code ConditionCheckers}, but only if it is non-{@code null}.
    * 
    * @param checker
    * the {@code ConditionChecker} to add
    * @return true if, and only if, the given {@code ConditionChecker} was added
    */
   public synchronized boolean addConditionChecker (
      ConditionChecker<?> checker) {
      if (checker != null) {
         return myCheckers.add (checker);
      }
      return false;
   }

   /**
    * Removes the given {@link ConditionChecker} from this
    * {@link ConditionMonitor}'s list of {@code ConditionCheckers}. Does nothing
    * if the given {@code ConditionChecker} is not found in the list.
    * 
    * @param checker
    * the {@code ConditionChecker} to remove
    * @return true if, and only if, the given {@code ConditionChecker} was
    * removed
    */
   public synchronized boolean removeConditionChecker (
      ConditionChecker<?> checker) {
      if (checker != null) {
         return myCheckers.remove (checker);
      }
      return false;
   }

   /**
    * Returns all the {@link ConditionChecker}s.
    * 
    * @return all the {@link ConditionChecker}s
    */
   public synchronized List<ConditionChecker<?>> getConditionCheckers () {
      return myCheckers;
   }

   /**
    * Returns the first {@link ConditionChecker} whose name equals the given
    * name. If the given name is {@code null}, returns the first
    * {@code ConditionChecker} whose name is {@code null}.
    * 
    * @param name
    * the name of the {@code ConditionChecker} to get
    * @return the {@code ConditionChecker} whose name equals the given name, or
    * {@code null} if no {@code ConditionChecker}'s name equals the given name
    */
   public synchronized ConditionChecker<?> getConditionChecker (String name) {
      for (ConditionChecker<?> checker : myCheckers) {
         String curName = checker.getName ();
         if ((name == null) ? (curName == null) : (name.equals (curName))) {
            return checker;
         }
      }
      return null;
   }

   @Override
   public synchronized void apply (double t0, double t1) {
      myMetConditions.clear ();
      for (ConditionChecker<?> checker : myCheckers) {
         if (checker.conditionMet (t0, t1)) {
            Condition cond = checker.getCondition ();
            if (!myMetConditions.contains (cond)) {
               myMetConditions.add (cond);
            }
         }
      }
   }
}
