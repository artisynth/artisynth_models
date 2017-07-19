package artisynth.tools.batchsim.manager;

import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.LinkedList;
import java.util.List;

import maspack.util.IndentingPrintWriter;

public class CombinationChecker implements Printable {

   protected List<PropertySpecification> myOriginalPropSpecs;
   protected HashMap<String,List<Object>> myPropSpecs = new HashMap<> ();
   protected LinkedList<PropPathValueSetPair> myCurPropSpecs =
      new LinkedList<> ();

   protected static class PropPathValueSetPair {
      public String myPropPath;
      public List<Object> myValueSet;

      public PropPathValueSetPair (String propPath, List<Object> valueSet) {
         myPropPath = propPath;
         myValueSet = valueSet;
      }
   }

   public CombinationChecker (List<PropertySpecification> propSpecs) {
      myOriginalPropSpecs = propSpecs;
      for (PropertySpecification propSpec : myOriginalPropSpecs) {
         myPropSpecs
            .put (propSpec.getPropertyPath (), propSpec.getCollection ());
      }
   }

   public List<PropertySpecification> getPropSpecs () {
      return myOriginalPropSpecs;
   }

   public void popIfNecessary (String propPath) throws NoSuchElementException {
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

   public boolean pushAndCheck (String propPath, String value) {
      push (propPath, value);
      return check ();
   }

   public void push (String propPath, String value) {
      if (myPropSpecs.containsKey (propPath)) {
         if (myPropSpecs.get (propPath).contains (value)) {
            List<Object> valueSet = myPropSpecs.remove (propPath);
            myCurPropSpecs.push (new PropPathValueSetPair (propPath, valueSet));
         }
      }
   }

   public boolean check () {
      return myPropSpecs.isEmpty ();
   }

   public void clear () {
      while (!myCurPropSpecs.isEmpty ()) {
         pop ();
      }
   }

   @Override
   public void print (IndentingPrintWriter writer) {
      for (PropertySpecification propSpec : myOriginalPropSpecs) {
         propSpec.print (writer, false);
      }
   }

   @Override
   public String toString () {
      return Utils.printableToString (this);
   }

}
