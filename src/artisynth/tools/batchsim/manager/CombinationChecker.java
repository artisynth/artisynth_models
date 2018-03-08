package artisynth.tools.batchsim.manager;

import java.util.HashMap;

import artisynth.tools.batchsim.manager.PropertySpecification.PhonyPropValue;
import artisynth.tools.batchsim.manager.PropertySpecification.Redef;
import artisynth.tools.batchsim.manager.PropertySpecification.SpecificationType;

import java.util.LinkedList;
import java.util.List;

import maspack.util.IndentingPrintWriter;

/**
 * A {@code CombinationChecker} checks whether a particular task pushed to the
 * checker matches any combination of {@link SpecificationType#COMBINATORIAL}
 * {@link PropertySpecification} values from its list.
 * <p>
 * {@code CombinationCheckers} are used to implement "skip" statements and
 * "when" blocks of {@link Redef} statements.
 * <p>
 * If the block also contains nested Jython code blocks, a
 * {@code CombinationChecker} will execute it as an additional check. For
 * {@link #check()} to return {@code true}, there must be both a matching
 * combination of {@code PropertySpecification}s <b>and</b> all Jython code
 * blocks must evaluate to true.
 *
 * @author Francois Roewer-Despres
 */
public class CombinationChecker implements Printable {

   protected List<JythonCodeBlock> myJythonCodeBlocks;
   protected List<PropertySpecification> myOriginalPropSpecs;
   protected HashMap<String,List<Object>> myPropSpecs = new HashMap<> ();
   protected LinkedList<PropPathValueSetPair> myCurPropSpecs =
      new LinkedList<> ();

   /**
    * A {@link PropPathValueSetPair} is just a
    * {@link SpecificationType#COMBINATORIAL} {@link PropertySpecification}'s
    * property path as well as its value set.
    * <p>
    * It is sort of a light clone of a {@code PropertySpecification}.
    *
    * @author Francois Roewer-Despres
    */
   protected static class PropPathValueSetPair {
      public String myPropPath;
      public List<Object> myValueSet;

      public PropPathValueSetPair (String propPath, List<Object> valueSet) {
         myPropPath = propPath;
         myValueSet = valueSet;
      }
   }

   /**
    * Creates a new {@link CombinationChecker} with the given list of
    * {@link SpecificationType#COMBINATORIAL} {@link PropertySpecification}s.
    *
    * @param jythonCodeBlocks
    * the list of Jython code blocks
    * @param propSpecs
    * the list of {@code PropertySpecification}s
    */
   public CombinationChecker (List<JythonCodeBlock> jythonCodeBlocks,
   List<PropertySpecification> propSpecs) {
      myJythonCodeBlocks = jythonCodeBlocks;
      myOriginalPropSpecs = propSpecs;
      for (PropertySpecification propSpec : myOriginalPropSpecs) {
         myPropSpecs
            .put (propSpec.getPropertyPath (), propSpec.getCollection ());
      }
   }

   /**
    * Returns the list of {@link PropertySpecification}s of this
    * {@link CombinationChecker}.
    *
    * @return the list of {@code PropertySpecification}s
    */
   public List<PropertySpecification> getPropSpecs () {
      return myOriginalPropSpecs;
   }

   /**
    * If a {@link PropertySpecification} with the given property path was pushed
    * last, pop it.
    *
    * @param propPath
    * the property path
    */
   public void popIfNecessary (String propPath) {
      if (!myCurPropSpecs.isEmpty ()) {
         if (myCurPropSpecs.peek ().myPropPath.equals (propPath)) {
            pop ();
         }
      }
   }

   protected void pop () {
      PropPathValueSetPair pair = myCurPropSpecs.pop ();
      myPropSpecs.put (pair.myPropPath, pair.myValueSet);
   }

   /**
    * Pushes the {@link PropertySpecification} with the given property path and
    * value, then checks if this {@link CombinationChecker} has a matching
    * combination.
    *
    * @param propPath
    * the property path
    * @param value
    * the value
    * @param task
    * the current partial task
    * @return whether a combination is matching
    */
   public boolean pushAndCheck (
      String propPath, String value, List<PhonyPropValue> task) {
      push (propPath, value);
      return check (task);
   }

   /**
    * Pushes the {@link PropertySpecification} with the given property path and
    * value.
    *
    * @param propPath
    * the property path
    * @param value
    * the value
    */
   public void push (String propPath, String value) {
      if (myPropSpecs.containsKey (propPath)) {
         if (myPropSpecs.get (propPath).contains (value)) {
            List<Object> valueSet = myPropSpecs.remove (propPath);
            myCurPropSpecs.push (new PropPathValueSetPair (propPath, valueSet));
         }
      }
   }

   /**
    * Checks if this {@link CombinationChecker} has a matching combination.
    *
    * @param task
    * the current partial task
    * @return whether a combination is matching
    */
   public boolean check (List<PhonyPropValue> task) {
      for (JythonCodeBlock block : myJythonCodeBlocks) {
         if (!block.check (task)) { // If even one jython code block doesn't
            return false; // match, then the entire combination doesn't match.
         }
      }
      return myPropSpecs.isEmpty ();
   }

   /**
    * Pops everything from this {@link CombinationChecker}.
    */
   public void clear () {
      while (!myCurPropSpecs.isEmpty ()) {
         pop ();
      }
   }

   @Override
   public void print (IndentingPrintWriter writer) {
      for (JythonCodeBlock block : myJythonCodeBlocks) {
         block.print (writer);
      }
      for (PropertySpecification propSpec : myOriginalPropSpecs) {
         propSpec.print (writer, false);
      }
   }

   @Override
   public String toString () {
      return Utils.printableToString (this);
   }

}
