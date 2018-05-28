package artisynth.tools.batchsim.conditions;

/**
 * A base class to implement the functionality common to all
 * {@link ConditionChecker}s.
 * <p>
 * In particular, all {@code ConditionCheckers} are <u>expected</u> to provide
 * <b>two</b> types of {@code public} constructors. The first takes a value of
 * type {@code C}, as may be expected. The second takes an additional parameter:
 * another {@code ConditionChecker}, known as a <i>nested condition checker</i>.
 * In so doing, {@code ConditionCheckers} use the
 * <a href="https://en.wikipedia.org/wiki/Decorator_pattern">decorator
 * pattern</a> (just as Java's <a href=
 * "https://docs.oracle.com/javase/7/docs/api/java/io/package-summary.html">I/O
 * Framework</a> does). A third type of {@code public} constructor exists that
 * additionally takes a {@link String} name for the {@code ConditionChecker}.
 * Although subclasses are strongly encouraged to implement such a constructor,
 * doing so is not required, as names can be set using {@link #setName(String)}.
 * <p>
 * When calling the {@link #conditionMet()} or {@link #conditionMet(Condition)}
 * methods, the return value will be {@code true} if, and only if, both this
 * {@code ConditionChecker}'s {@link Condition} <b>and</b> its nested
 * {@code ConditionChecker}'s {@code Condition} are met.
 * 
 * @param <C>
 * the type of {@code Condition} that this checker can check
 * 
 * @author Francois Roewer-Despres
 */
public abstract class ConditionCheckerBase<C extends Condition>
implements ConditionChecker<C> {

   /** My current name. */
   protected String myName;

   /** My current/active condition. */
   protected C myCondition;

   /** My nested {@link ConditionChecker}. */
   protected ConditionChecker<? extends Condition> myNestedChecker;

   /**
    * The default nested {@link ConditionChecker} whose {@link Condition} is
    * always met. This works, because {@code true} is the "unit" or "identity"
    * value of the logical {@code AND} function. That is, for all Boolean
    * expressions {@code x}, we have:
    * 
    * <pre>
    * x &amp;&amp; true == x
    * </pre>
    * 
    * @author Francois Roewer-Despres
    */
   public static final class TrueConditionChecker
   implements ConditionChecker<TrueConditionChecker.TrueCondition> {

      public static enum TrueCondition implements Condition {
         TRUE;
      }

      protected String myName;

      public TrueConditionChecker () {
         this (null);
      }

      public TrueConditionChecker (String name) {
         myName = name;
      }

      @Override
      public String getName () {
         return myName;
      }

      @Override
      public void setName (String name) {
         myName = name;
      }

      @Override
      public TrueCondition getCondition () {
         return TrueCondition.TRUE;
      }

      @Override
      public void setCondition (TrueCondition cond) {
      }

      @Override
      public ConditionChecker<? extends Condition> getNestedChecker () {
         return new TrueConditionChecker ();
      }

      /**
       * Calls to this method are ignored.
       * 
       * @param nestedChecker
       * ignored
       */
      @Override
      public void setNestedChecker (
         ConditionChecker<? extends Condition> nestedChecker) {
      }

      @Override
      public boolean conditionMet () {
         return true;
      }

      @Override
      public boolean conditionMet (double t0, double t1) {
         return true;
      }

      @Override
      public boolean conditionMet (TrueCondition cond) {
         return true;
      }

      @Override
      public boolean conditionMet (TrueCondition cond, double t0, double t1) {
         return true;
      }

   }

   /**
    * Creates a new {@link ConditionChecker}, initialized with the given
    * {@link Condition}.
    * 
    * @param cond
    * the initial {@code Condition} of this {@code ConditionChecker}
    */
   public ConditionCheckerBase (C cond) {
      this (null, cond, null);
   }

   /**
    * Creates a new {@link ConditionChecker}, initialized with the given name
    * and {@link Condition}.
    * 
    * @param name
    * the name of this {@code ConditionChecker}
    * @param cond
    * the initial {@code Condition} of this {@code ConditionChecker}
    */
   public ConditionCheckerBase (String name, C cond) {
      this (name, cond, null);
   }

   /**
    * Creates a new {@link ConditionChecker}, initialized with the given
    * {@link Condition}, and with the given nested {@code ConditionChecker}.
    * 
    * @param cond
    * the initial {@code Condition} of this {@code ConditionChecker}
    * @param nestedChecker
    * the nested {@code ConditionChecker} of this {@code ConditionChecker}
    */
   public ConditionCheckerBase (C cond,
   ConditionChecker<? extends Condition> nestedChecker) {
      this (null, cond, nestedChecker);
   }

   /**
    * Creates a new {@link ConditionChecker}, initialized with given name, with
    * the given {@link Condition}, and with the given nested
    * {@link ConditionChecker}.
    * 
    * @param name
    * the name of this {@code ConditionChecker}
    * @param cond
    * the initial {@link Condition} of this {@code ConditionChecker}
    * @param nestedChecker
    * the nested {@code ConditionChecker} of this {@code ConditionChecker}
    */
   public ConditionCheckerBase (String name, C cond,
   ConditionChecker<? extends Condition> nestedChecker) {
      setName (name);
      setCondition (cond);
      setNestedChecker (nestedChecker);
   }

   @Override
   public String getName () {
      return myName;
   }

   @Override
   public void setName (String name) {
      myName = name;
   }

   @Override
   public C getCondition () {
      return myCondition;
   }

   @Override
   public void setCondition (C cond) {
      myCondition = cond;
   }

   @Override
   public ConditionChecker<? extends Condition> getNestedChecker () {
      return myNestedChecker;
   }

   @Override
   public void setNestedChecker (
      ConditionChecker<? extends Condition> nestedChecker) {
      if (nestedChecker == null) {
         nestedChecker = new TrueConditionChecker ();
      }
      myNestedChecker = nestedChecker;
   }

   @Override
   public boolean conditionMet () {
      return conditionMet (0, 0);
   }

   @Override
   public boolean conditionMet (double t0, double t1) {
      return conditionMet (getCondition (), t0, t1);
   }

   @Override
   public boolean conditionMet (C cond) {
      return conditionMet (cond, 0, 0);
   }

   @Override
   public boolean conditionMet (C cond, double t0, double t1) {
      return checkCondition (cond, t0, t1)
      && myNestedChecker.conditionMet (t0, t1);
   }

   /**
    * Method called by {@link #conditionMet(Condition)}.
    * <p>
    * Subclasses should override <b>this</b> method to check whether the given
    * {@link Condition} is met. Subclasses should <b>not</b> override
    * {@code conditionMet(Condition)}. There is no need to check whether the
    * underlying nested {@link ConditionChecker}'s (i.e.
    * {@link #myNestedChecker}'s) {@code Condition} is also met, because that is
    * already checked in this base class.
    * 
    * @param cond
    * the {@code Condition} to check
    * @param t0
    * time at start of step
    * @param t1
    * time at end of step
    * @return true if the given {@code Condition} is met, and false otherwise
    */
   protected abstract boolean checkCondition (C cond, double t0, double t1);

}