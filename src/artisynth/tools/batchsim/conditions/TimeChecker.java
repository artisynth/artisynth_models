package artisynth.tools.batchsim.conditions;

import java.util.Objects;

/**
 * A {@code TimeChecker} compares the current time in {@code ArtiSynth} (via
 * {@code Main.getMain().getTime()}) to an internal time range, and checks
 * whether its {@link TimeCondition} is met for the current time.
 * 
 * @author Francois Roewer-Despres
 */
public class TimeChecker
extends ConditionCheckerBase<TimeChecker.TimeCondition> {

   /** The default epsilon value for a {@link TimeChecker}. */
   public static final Double DEFAULT_EPSILON = 0.00001;

   /**
    * Given a time range specified by a minimum time and a maximum time, and
    * given a very small positive number, {@code epsilon}, a
    * {@code TimeCondition} represents a comparison of the current time relative
    * to that time range, where times are considered equal if, and only if, the
    * absolute difference between them is less than {@code epsilon}.
    * 
    * @author Francois Roewer-Despres
    */
   public static enum TimeCondition implements Condition {
      LESS_THAN_MIN,
      NOT_LESS_THAN_MIN,
      EQUAL_MIN,
      NOT_EQUAL_MIN,
      GREATER_THAN_MIN,
      NOT_GREATER_THAN_MIN,
      IN_RANGE_INCLUSIVE,
      NOT_IN_RANGE_INCLUSIVE,
      IN_RANGE_EXCLUSIVE,
      NOT_IN_RANGE_EXCLUSIVE,
      LESS_THAN_MAX,
      NOT_LESS_THAN_MAX,
      EQUAL_MAX,
      NOT_EQUAL_MAX,
      GREATER_THAN_MAX,
      NOT_GREATER_THAN_MAX;
   }

   protected double min;
   protected double max;
   protected double eps;

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values, and a default {@code epsilon}.
    * 
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    */
   public TimeChecker (TimeCondition cond, double minTime, double maxTime) {
      this (null, cond, null, minTime, maxTime, DEFAULT_EPSILON);
   }

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values, and a default {@code epsilon}.
    * 
    * @param name
    * the name of this {@code TimeChecker}
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    */
   public TimeChecker (String name, TimeCondition cond, double minTime,
   double maxTime) {
      this (name, cond, null, minTime, maxTime, DEFAULT_EPSILON);
   }

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values.
    * 
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    * @param epsilon
    * the threshold for equality
    */
   public TimeChecker (TimeCondition cond, double minTime, double maxTime,
   double epsilon) {
      this (null, cond, null, minTime, maxTime, epsilon);
   }

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values.
    * 
    * @param name
    * the name of this {@code TimeChecker}
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    * @param epsilon
    * the threshold for equality
    */
   public TimeChecker (String name, TimeCondition cond, double minTime,
   double maxTime, double epsilon) {
      this (name, cond, null, minTime, maxTime, epsilon);
   }

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values, and a default {@code epsilon}.
    * 
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    */
   public TimeChecker (TimeCondition cond, ConditionChecker<?> nestedChecker,
   double minTime, double maxTime) {
      this (null, cond, nestedChecker, minTime, maxTime, DEFAULT_EPSILON);
   }

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values, and a default {@code epsilon}.
    * 
    * @param name
    * the name of this {@code TimeChecker}
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    */
   public TimeChecker (String name, TimeCondition cond,
   ConditionChecker<?> nestedChecker, double minTime, double maxTime) {
      this (name, cond, nestedChecker, minTime, maxTime, DEFAULT_EPSILON);
   }

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values.
    * 
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    * @param epsilon
    * the threshold for equality
    */
   public TimeChecker (TimeCondition cond, ConditionChecker<?> nestedChecker,
   double minTime, double maxTime, double epsilon) {
      this (null, cond, nestedChecker, minTime, maxTime, epsilon);
   }

   /**
    * Creates a new {@link TimeChecker} initialized with the given parameter
    * values.
    * 
    * @param name
    * the name of this {@code TimeChecker}
    * @param cond
    * the initial {@link TimeCondition} of this {@code TimeChecker}
    * @param nestedChecker
    * the nested {@link ConditionChecker} of this {@code TimeChecker}
    * @param minTime
    * the minimum time in the time range
    * @param maxTime
    * the maximum time in the time range
    * @param epsilon
    * the threshold for equality
    */
   public TimeChecker (String name, TimeCondition cond,
   ConditionChecker<?> nestedChecker, double minTime, double maxTime,
   double epsilon) {
      super (name, cond, nestedChecker);
      min = minTime;
      max = maxTime;
      eps = epsilon;
   }

   @Override
   protected boolean checkCondition (TimeCondition cond, double t0, double t1) {
      switch (cond) {
         case LESS_THAN_MIN:
            return t0 < min;
         case NOT_LESS_THAN_MIN:
            return !(t0 < min);
         case EQUAL_MIN:
            return Math.abs (t0 - min) < eps;
         case NOT_EQUAL_MIN:
            return !checkCondition (TimeCondition.EQUAL_MIN, t0, t1);
         case GREATER_THAN_MIN:
            return t0 > min;
         case NOT_GREATER_THAN_MIN:
            return !(t0 > min);
         case IN_RANGE_INCLUSIVE:
            return min <= t0 && t0 <= max;
         case NOT_IN_RANGE_INCLUSIVE:
            return !checkCondition (TimeCondition.IN_RANGE_INCLUSIVE, t0, t1);
         case IN_RANGE_EXCLUSIVE:
            return min < t0 && t0 < max;
         case NOT_IN_RANGE_EXCLUSIVE:
            return !checkCondition (TimeCondition.IN_RANGE_EXCLUSIVE, t0, t1);
         case LESS_THAN_MAX:
            return t0 < max;
         case NOT_LESS_THAN_MAX:
            return !(t0 < max);
         case EQUAL_MAX:
            return Math.abs (t0 - max) < eps;
         case NOT_EQUAL_MAX:
            return !checkCondition (TimeCondition.EQUAL_MAX, t0, t1);
         case GREATER_THAN_MAX:
            return t0 > max;
         case NOT_GREATER_THAN_MAX:
            return !(t0 > max);
         default:
            throw new UnsupportedOperationException (Objects.toString (cond));
      }
   }
}
