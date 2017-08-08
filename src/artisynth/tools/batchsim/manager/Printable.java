package artisynth.tools.batchsim.manager;

import maspack.util.IndentingPrintWriter;

/**
 * A {@code Printable} can print itself using an {@link IndentingPrintWriter}.
 *
 * @author Francois Roewer-Despres
 */
public interface Printable {

   /**
    * Prints a string representation of this {@link Printable} using the given
    * {@link IndentingPrintWriter}.
    *
    * @param writer
    * the writer with which to print
    */
   void print (IndentingPrintWriter writer);

}
