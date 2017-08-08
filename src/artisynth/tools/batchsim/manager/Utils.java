package artisynth.tools.batchsim.manager;

import static artisynth.tools.batchsim.manager.PropertySpecification.SpecificationType.*;

import java.io.PrintStream;
import java.io.StringWriter;

import artisynth.core.gui.jythonconsole.ArtisynthJythonConsole;
import maspack.util.IndentingPrintWriter;

public class Utils {

   /**
    * Returns as a string a vector, where the i'th entry is the string
    * representation of a double value sampled from the i'th distribution ID of
    * the given probabilistic {@link PropertySpecification}.
    * 
    * @param propSpec
    * a probabilistic {@code PropertySpecification}
    * @return a vector of sampled distribution values as a string, or
    * {@code null} if the given property specification is not probabilistic
    */
   public static String createDistributionVectorAsString (
      DistributionSampler sampler, PropertySpecification propSpec) {
      if (propSpec.getSpecificationType () == PROBABILISTIC) {
         StringBuilder builder = new StringBuilder ();
         for (Object id : propSpec.getCollection ()) {
            builder.append (sampler.sample ((int)id)).append (" ");
         }
         return builder.toString ();
      }
      return null;
   }

   /**
    * If the given boolean condition is true, print the given message using
    * {@link PrintStream#println() ps.println()}, <b>UNLESS</b> the given
    * console is <b>non</b>-{@code null}, in which case the given message is
    * printed to the console's print stream.
    * 
    * @param console
    * the (possibly {@code null}) jython console to which to perhaps print
    * @param cond
    * the condition determining if the message gets printed or not
    * @param ps
    * the {@link PrintStream} to which to print
    * @param msg
    * the message to print
    */
   public static void printCond (
      ArtisynthJythonConsole console, boolean cond, PrintStream ps,
      String msg) {
      if (cond) {
         if (console == null) {
            ps.println (msg);
         }
         else {
            console.getConsole ().write (msg + "\n");
         }
      }
   }

   /**
    * Given a {@link Printable}, returns its string representation as would be
    * printed using {@link Printable#print(IndentingPrintWriter)}.
    *
    * @param printable
    * the {@code Printable}
    * @return a string representation of the {@code Printable}
    */
   public static String printableToString (Printable printable) {
      StringWriter writer = new StringWriter ();
      printable.print (new IndentingPrintWriter (writer));
      return writer.toString ();
   }
}
