package artisynth.tools.batchsim.conditions;

/**
 * A {@code NotChecker} is a special kind of {@link ConditionChecker} that does
 * not really have a {@link Condition} of its own to check. Rather, a
 * {@code NotChecker} checks whether its nested {@code ConditionChecker}'s
 * {@code Condition} is met. If so, it considers its own {@code Condition} to be
 * <i>not</i> met, and if not, it considers its own {@code Condition} to be met.
 * Thus, a {@code NotChecker} is the equivalent of applying the logical
 * {@code NOT} function to its nested {@code ConditionChecker}'s
 * {@code Condition}.
 *
 * @author Francois Roewer-Despres
 * @version 1.0
 */
public class NotChecker extends ConditionCheckerBase<NotChecker.NotCondition> {
   
   /**
    * Creates a new {@link NotChecker} to logically negate the given nested
    * {@link ConditionChecker}.
    * 
    * @param nestedChecker
    * the nested {@code ConditionChecker} of this {@code NotChecker}
    */
   public NotChecker (ConditionChecker<?> nestedChecker) {
      this (null, nestedChecker);
   }
   
   /**
    * Creates a new {@link NotChecker}, with the given name, to logically negate
    * the given nested {@link ConditionChecker}.
    * 
    * @param name
    * the name of this {@code NotChecker}
    * @param nestedChecker
    * the nested {@code ConditionChecker} of this {@code NotChecker}
    */
   public NotChecker (String name, ConditionChecker<?> nestedChecker) {
      super (name, null, nestedChecker);
   }
   
   /**
    * Acts simply as a placeholder. This enum is unused, and anywhere a
    * reference to a constant of this enum type is required, {@code null} should
    * be supplied instead.
    * 
    * @author Francois Roewer-Despres
    * @version 1.0
    */
   public static enum NotCondition implements Condition {
   }
   
   @Override
   public boolean conditionMet (NotCondition cond) {
      return checkCondition (cond);
   }

   @Override
   protected boolean checkCondition (NotCondition cond) {
      return !myNestedChecker.conditionMet ();
   }

}
