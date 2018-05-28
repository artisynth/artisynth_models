package artisynth.tools.batchsim.conditions;

/**
 * A {@code ConditionChecker} checks whether a particular {@link Condition}
 * value is met at the moment the checker's {@link #conditionMet()} or
 * {@link #conditionMet(Condition)} method is called.
 * <p>
 * When calling the {@code conditionMet()} or {@code conditionMet(Condition)}
 * methods, the return value will be {@code true} if, and only if, both this
 * {@code ConditionChecker}'s {@code Condition} value <b>and</b> its nested
 * {@code ConditionChecker}'s {@code Condition} value are true. Refer to
 * {@link ConditionCheckerBase} for details.
 * <p>
 * 
 * @param <C>
 * the type of {@code Condition} that this {@code ConditionChecker} can check
 * 
 * @author Francois Roewer-Despres
 * @see ConditionCheckerBase
 */
public interface ConditionChecker<C extends Condition> {

   /**
    * Returns the name of this {@link ConditionChecker} (may be {@code null}).
    * 
    * @return the name of this {@code ConditionChecker} (may be {@code null})
    */
   String getName ();

   /**
    * Sets the name of this {@link ConditionChecker} (may be {@code null}).
    * 
    * @param name
    * the new name of this {@code ConditionChecker}
    */
   void setName (String name);

   /**
    * Returns the current/active {@link Condition} of this
    * {@link ConditionChecker}.
    * 
    * @return the current/active {@code Condition} of this
    * {@code ConditionChecker}
    */
   C getCondition ();

   /**
    * Sets the current/active {@link Condition} of this
    * {@link ConditionChecker}.
    * 
    * @param cond
    * the new {@code Condition} of this {@code ConditionChecker}
    */
   void setCondition (C cond);

   /**
    * Returns the nested {@link ConditionChecker} of this
    * {@code ConditionChecker}.
    * 
    * @return the nested {@code ConditionChecker} of this
    * {@code ConditionChecker}
    */
   ConditionChecker<? extends Condition> getNestedChecker ();

   /**
    * Sets the nested {@link ConditionChecker} of this {@code ConditionChecker}.
    * 
    * @param nestedChecker
    * the new nested {@code ConditionChecker} of this {@code ConditionChecker}
    */
   void setNestedChecker (ConditionChecker<? extends Condition> nestedChecker);

   /**
    * Determines whether the current/active {@link Condition} and also the
    * nested {@link ConditionChecker}'s current/active {@code Condition} are
    * <b>both</b> met.
    * <p>
    * This method behaves as though by returning the value of the call
    * 
    * <pre>
    * conditionMet (0, 0)
    * </pre>
    * 
    * @return true if both {@code Conditions} are met; false otherwise
    * @see #conditionMet(Condition)
    */
   boolean conditionMet ();

   /**
    * Determines whether the current/active {@link Condition} and also the
    * nested {@link ConditionChecker}'s current/active {@code Condition} are
    * <b>both</b> met.
    * <p>
    * This method behaves as though by returning the value of the call
    * 
    * <pre>
    * conditionMet (getCondition (), t0, t1)
    * </pre>
    * 
    * @param t0
    * time at start of step
    * @param t1
    * time at end of step
    * @return true if both {@code Conditions} are met; false otherwise
    * @see #conditionMet(Condition)
    */
   boolean conditionMet (double t0, double t1);

   /**
    * Determines whether the given {@link Condition} and also the nested
    * {@link ConditionChecker}'s current/active {@code Condition} are
    * <b>both</b> met.
    * <p>
    * This method behaves as though by returning the value of the call
    * 
    * <pre>
    * conditionMet (getCondition (), 0, 0)
    * </pre>
    * 
    * @param cond
    * the {@code Condition} to check
    * @return true if both {@code Conditions} are met; false otherwise
    * @see #conditionMet()
    */
   boolean conditionMet (C cond);

   /**
    * Determines whether the given {@link Condition} and also the nested
    * {@link ConditionChecker}'s current/active {@code Condition} are
    * <b>both</b> met.
    * 
    * @param cond
    * the {@code Condition} to check
    * @param t0
    * time at start of step
    * @param t1
    * time at end of step
    * @return true if both {@code Conditions} are met; false otherwise
    * @see #conditionMet()
    */
   boolean conditionMet (C cond, double t0, double t1);

}
