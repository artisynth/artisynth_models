package artisynth.tools.batchsim.manager;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.python.core.PyObject;

import artisynth.core.gui.jythonconsole.ArtisynthJythonConsole;
import artisynth.core.util.ArtisynthPath;
import artisynth.tools.batchsim.manager.DistributionSampler.Distribution;
import static artisynth.tools.batchsim.manager.PropertySpecification.*;
import static artisynth.tools.batchsim.manager.PropertySpecification.SpecificationType.*;
import maspack.util.ReaderTokenizer;

/**
 * A {@code FileParser} parses the input file (property specification file) on
 * behalf of a {@link BatchManager}. This is done in accordance to the Property
 * Specification Language (PSL) of BatchSim (see the official documentation in
 * artisynth_models/doc/batchsim for details). The two classes are intimately
 * coupled, and are only separated to reduce clutter in the {@code BatchManager}
 * class (following proper OO theory, the code found here should reside within
 * the {@code BatchManager} itself).
 *
 * @author Francois Roewer-Despres
 */
public class FileParser {

   /**
    * The settings directing how a {@link FileParser} will parse an input file
    * (and what this file is).
    *
    * @author Francois Roewer-Despres
    */
   public static class Settings {
      public BatchManager manager;
      public ArtisynthJythonConsole console;
      public char comment;
      public char delim;
      public int interactionLevel;
      public boolean debug;
      public String propsFileName;
      public double epsilon;
      public int seed;
   }

   /**
    * The results of parsing the input file.
    *
    * @author Francois Roewer-Despres
    */
   public static class ParseResults {
      public List<PropertySpecification> propertySpecifications;
      public List<CombinationChecker> skipStatementCheckers;
      public int resultingInteractionLevel;
      public boolean containsCombinatorialSpecs;
      public boolean containsProbabilisticSpecs;
      public DistributionSampler sampler;
   }

   protected Settings mySettings;
   protected ReaderTokenizer rtok;
   protected ParseResults myResults;
   protected DistributionSampler mySampler;
   protected List<PropertySpecification> myPropSpecs;
   protected List<CombinationChecker> mySkipCheckers;

   /**
    * Creates a new {@link FileParser} with the given {@link Settings}.
    *
    * @param settings
    * the settings for this {@code FileParser}
    * @throws FileNotFoundException
    * if the {@link ReaderTokenizer} cannot be created because the input file
    * cannot be found
    */
   public FileParser (Settings settings) throws FileNotFoundException {
      mySettings = settings;
      createReaderTokenizer ();
      myResults = new ParseResults ();
      myResults.resultingInteractionLevel = mySettings.interactionLevel;
      myPropSpecs = new LinkedList<> ();
      mySkipCheckers = new LinkedList<> ();
   }

   /**
    * Creates a new {@link ReaderTokenizer} to read the input file.
    * 
    * @throws FileNotFoundException
    * if the file does not exist
    */
   protected void createReaderTokenizer () throws FileNotFoundException {
      Reader reader;
      if (mySettings.propsFileName.equals ("-")) {
         myResults.resultingInteractionLevel = 0;
         reader = new BufferedReader (new InputStreamReader (System.in));
      }
      else {
         reader = new FileReader (mySettings.propsFileName);
      }
      rtok = new ReaderTokenizer (reader);
      rtok.commentChar (mySettings.comment);
      rtok.quoteChar (mySettings.delim);
      rtok.quoteChar ('$'); // For Jython code blocks.
   }

   /**
    * Performs the parsing of the input file and returns the results.
    * 
    * @return the parsing results
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   public ParseResults parse () throws IOException {
      while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
         boolean phony = false;
         SpecificationType decorator = null;
         ArrayList<Number> decArgs = null;
         if (rtok.ttype == '@') {
            String phonyName = rtok.scanWord ();
            if ("PHONY".equals (phonyName)) {
               phony = true;
               rtok.nextToken ();
               if (rtok.ttype == '@') {
                  decArgs = new ArrayList<> ();
                  decorator = readDecorator (decArgs);
               }
               else {
                  rtok.pushBack ();
               }
            }
            else if ("COMB".equals (phonyName) || "PROB".equals (phonyName)) {
               rtok.pushBack ();
               decArgs = new ArrayList<> ();
               decorator = readDecorator (decArgs);
            }
            else {
               throw new IOException (
                  "unrecognized decorator on line " + rtok.lineno () + ": `"
                  + phonyName + "'");
            }
            myPropSpecs.addAll (readPropSpec (phony, decorator, decArgs));
         }
         else if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            String token = rtok.sval;
            if (token.equals ("skip")) {
               mySkipCheckers.add (readSkipOrWhen ("skip statement"));
            }
            else if (token.equals ("redef")) {
               readRedefStatement ();
            }
            else if (token.equals ("when") || token.equals ("end")) {
               throw new IOException (
                  "unexpected token on line " + rtok.lineno () + ": " + token);
            }
            else {
               throw new IOException (
                  "unrecognized token on line " + rtok.lineno () + ": "
                  + token);
            }
         }
         else {
            rtok.pushBack ();
            myPropSpecs.addAll (readPropSpec (phony, decorator, decArgs));
         }
      }

      rtok.close ();
      myResults.propertySpecifications = myPropSpecs;
      myResults.skipStatementCheckers = mySkipCheckers;
      myResults.sampler = mySampler;
      return myResults;
   }

   /**
    * Assumes the next bit of input (immediately after the '@') corresponds to a
    * decorator, then reads it, filling the given decArgs list with the
    * decorator arguments (if any), and return the type of the decorator.
    *
    * @param decArgs
    * an empty list to be filled with the decorator arguments (if any)
    * @return the type of the decorator
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected SpecificationType readDecorator (ArrayList<Number> decArgs)
      throws IOException {
      SpecificationType decorator = null;
      String decoratorName = rtok.scanWord ();
      if ("COMB".equals (decoratorName)) {
         decorator = COMBINATORIAL;
      }
      else if ("PROB".equals (decoratorName)) {
         decorator = PROBABILISTIC;
      }
      else if ("PHONY".equals (decoratorName)) {
         throw new IOException (
            "@PHONY decorator not allowed in this position on line "
            + rtok.lineno ());
      }
      else {
         throw new IOException (
            "unrecognized decorator on line " + rtok.lineno () + ": `"
            + decoratorName + "'");
      }
      switch (decorator) {
         case COMBINATORIAL:
            rtok.scanCharacter ('(');
            int n = rtok.scanInteger ();
            if (n <= 0) {
               throw new IOException (
                  "the argument to the combinatorial decorator on line "
                  + rtok.lineno () + " cannot be negative.");
            }
            decArgs.add (n);
            rtok.scanCharacter (')');
            break;
         case PROBABILISTIC:
            if (rtok.nextToken () == '(') {
               while (rtok.nextToken () != ')') {
                  rtok.pushBack ();
                  double p = rtok.scanNumber ();
                  if (p < 0 || p > 1) {
                     throw new IOException (
                        "the argument '" + p
                        + "' of the probabilistic decorator on line "
                        + rtok.lineno ()
                        + " does not lie in the range [0, 1].");
                  }
                  decArgs.add (p);
                  rtok.nextToken ();
                  if (rtok.ttype == ')') {
                     rtok.pushBack ();
                  }
                  else if (rtok.ttype == ',') {
                     continue;
                  }
                  else {
                     throwAppropriateException ();
                  }
               }
               double sum = 0;
               for (Number d : decArgs) {
                  sum += d.doubleValue ();
               }
               if (Math.abs (sum - 1) > mySettings.epsilon) {
                  throw new IOException (
                     "the probabilistic decorator on line " + rtok.lineno ()
                     + " has arguments which do not add to within "
                     + mySettings.epsilon + " of 1.");
               }
            }
            else {
               rtok.pushBack ();
            }
            break;
      }
      return decorator;
   }

   /**
    * Switches on rtok.ttype, throwing an {@link IOException} with an
    * appropriate message.
    *
    * @throws IOException
    * always, but with a message appropriate to the circumstance
    */
   protected void throwAppropriateException () throws IOException {
      switch (rtok.ttype) {
         case ReaderTokenizer.TT_EOF:
            throw new IOException (
               "unexpected end of file on line " + rtok.lineno ());
         case ReaderTokenizer.TT_EOL:
            throw new IOException (
               "unexpected end of line on line " + rtok.lineno ());
         case ReaderTokenizer.TT_NOTHING:
            throw new IOException ("no input on line " + rtok.lineno ());
         case ReaderTokenizer.TT_NUMBER:
            throw new IOException (
               "unexpected input on line " + rtok.lineno () + ": `" + rtok.nval
               + "'");
         case ReaderTokenizer.TT_WORD:
            throw new IOException (
               "unexpected input on line " + rtok.lineno () + ": `" + rtok.sval
               + "'");
         default:
            throw new IOException (
               "unexpected input on line " + rtok.lineno () + ": `"
               + (char)rtok.ttype + "'");
      }
   }

   /**
    * Reads the next amount of input, interpreting it as a skip statement or
    * when block of a redefinition statement, depending on the incoming flag.
    *
    * @param skipOrWhen
    * either "skip" or "when" depending of the value of the last consumed token
    * @return a {@link CombinationChecker} representing the read skip or when
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected CombinationChecker readSkipOrWhen (String skipOrWhen)
      throws IOException {
      List<JythonCodeBlock> codeBlocks = new LinkedList<> ();
      List<PropertySpecification> propSpecs = new LinkedList<> ();
      while (true) {
         rtok.nextToken ();
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            if (rtok.sval.equals ("end")) {
               break;
            }
            else if (rtok.sval.equals ("jython")) {
               codeBlocks.add (readJythonCodeBlock ());
               continue;
            }
            else {
               throwAppropriateException ();
            }
         }
         rtok.pushBack ();
         String propPath = rtok.scanQuotedString ('"');
         List<PropertySpecification> tmp;
         switch (rtok.nextToken ()) {
            case '=':
               tmp = readCombinatorialValueSet (false, propPath);
               break;
            case '~':
               throw new IOException (
                  "probabilistic specifications are not allowed in a "
                  + skipOrWhen + ", but found `~' on line " + rtok.lineno ());
            default:
               throw new IOException (
                  "expecting `=' in " + skipOrWhen + " on line "
                  + rtok.lineno ());
         }
         for (PropertySpecification propSpec : tmp) {
            boolean found = false;
            propPath = propSpec.getPropertyPath ();
            for (PropertySpecification propSpecDef : myPropSpecs) {
               if (propSpecDef.getPropertyPath ().equals (propPath)) {
                  found = true;
                  propSpec.setIndex (propSpecDef.getIndex ());
                  break;
               }
            }
            if (!found) {
               throw new IOException (
                  "component \"" + propPath + "\" on line " + rtok.lineno ()
                  + " in " + skipOrWhen + " has no prior definition");
            }
         }
         propSpecs.addAll (tmp);
      }
      if (propSpecs.isEmpty () && codeBlocks.isEmpty ()) {
         throw new IOException (
            skipOrWhen + " on line " + rtok.lineno () + " cannot be empty");
      }
      return new CombinationChecker (codeBlocks, propSpecs);
   }

   /**
    * Reads the next amount of input, interpreting it as a Jython code block
    * (where the "jython" keyword token has already been consumed).
    * 
    * @return the read block
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected JythonCodeBlock readJythonCodeBlock () throws IOException {
      StringBuilder builder = new StringBuilder ();
      rtok.nextToken ();
      while (!rtok.tokenIsWord ("end")) {
         rtok.pushBack ();
         builder.append (rtok.scanQuotedString ('$').trim ()).append ('\n');
         rtok.nextToken ();
      }
      if (builder.length () == 0) {
         throw new IOException (
            "Jython code block on line " + rtok.lineno () + " cannot be empty");
      }
      return new JythonCodeBlock (
         mySettings.manager, builder.toString (), mySettings.console);
   }

   /**
    * Reads the next amount of input, interpreting it as a redefinition
    * statement (where the "redef" keyword token has already been consumed).
    *
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected void readRedefStatement () throws IOException {
      List<PropertySpecification> propSpecs = new LinkedList<> ();
      int lineno = rtok.lineno ();
      while (true) {
         rtok.nextToken ();
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            if (rtok.sval.equals ("when")) {
               break;
            }
            else {
               throwAppropriateException ();
            }
         }
         boolean phony = false;
         SpecificationType decorator = null;
         ArrayList<Number> decArgs = null;
         if (rtok.ttype == '@') {
            String phonyName = rtok.scanWord ();
            if ("PHONY".equals (phonyName)) {
               phony = true;
               rtok.nextToken ();
               if (rtok.ttype == '@') {
                  decArgs = new ArrayList<> ();
                  decorator = readDecorator (decArgs);
               }
               else {
                  rtok.pushBack ();
               }
            }
            else if ("COMB".equals (phonyName) || "PROB".equals (phonyName)) {
               rtok.pushBack ();
               decArgs = new ArrayList<> ();
               decorator = readDecorator (decArgs);
            }
            else {
               throw new IOException (
                  "unrecognized decorator on line " + rtok.lineno () + ": `"
                  + phonyName + "'");
            }
         }
         else {
            rtok.pushBack ();
         }
         propSpecs.addAll (readPropSpec (phony, decorator, decArgs));
      }
      if (propSpecs.isEmpty ()) {
         throw new IOException (
            "redef block on line " + rtok.lineno () + " cannot be empty");
      }

      CombinationChecker checker = readSkipOrWhen ("when block");
      for (PropertySpecification propSpec : propSpecs) {
         boolean found = false;
         String propPath = propSpec.getPropertyPath ();
         for (PropertySpecification propSpecDef : myPropSpecs) {
            if (propSpecDef.getPropertyPath ().equals (propPath)) {
               found = true;
               propSpecDef.addRedef (propSpec, checker);
               for (PropertySpecification whenPropSpec : checker
                  .getPropSpecs ()) {
                  if (whenPropSpec.getIndex () > propSpecDef.getIndex ()) {
                     throw new IOException (
                        "component \"" + propPath
                        + "\" in redef block starting on line " + lineno
                        + " was defined before component \""
                        + whenPropSpec.getPropertyPath ()
                        + "\" in the corresponding when block");
                  }
                  if (whenPropSpec.getIndex () == propSpecDef.getIndex ()) {
                     throw new IOException (
                        "component \"" + propPath
                        + "\" in redef block starting on line " + lineno
                        + " cannot also be in the corresponding when block");
                  }
               }
               break;
            }
         }
         if (!found) {
            throw new IOException (
               "component \"" + propPath + "\" in redef block starting on line "
               + lineno + " has no prior definition");
         }
      }
   }

   /**
    * Reads the next amount of input, interpreting it as a
    * {@link PropertySpecification} (either
    * {@link SpecificationType#COMBINATORIAL} or
    * {@link SpecificationType#PROBABILISTIC}). Specifically, expect a
    * double-quoted string (the property path), then either `=' or `~', then a
    * combinatorial value set or a probabilistic distribution vector.
    *
    * @param phony
    * whether to mark this {@code PropertySpecification} as phony
    * @param decorator
    * the decorator for this {@code PropertySpecification}, or {@code null}
    * @param decArgs
    * the decorator arguments, or {@code null}
    * @return a list of property specifications, each expanded from the one read
    * in, if applicable
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected List<PropertySpecification> readPropSpec (
      boolean phony, SpecificationType decorator, ArrayList<Number> decArgs)
      throws IOException {
      List<PropertySpecification> propSpecs;
      String propPath = rtok.scanQuotedString ('"');
      switch (rtok.nextToken ()) {
         case '=':
            propSpecs = readCombinatorialValueSet (phony, propPath);
            break;
         case '~':
            createSampler ();
            propSpecs = readProbabilisticDistributionVector (phony, propPath);
            break;
         default:
            throw new IOException (
               "expecting one of `=' or `~' on line " + rtok.lineno ());
      }
      if (decorator != null) {
         for (int i = 0; i < propSpecs.size (); i++) {
            propSpecs.set (
               i, decoratePropSpec (propSpecs.get (i), decorator, decArgs));
         }
      }
      if (propSpecs.get (0).getSpecificationType () == COMBINATORIAL) {
         myResults.containsCombinatorialSpecs = true;
      }
      else {
         myResults.containsProbabilisticSpecs = true;
      }
      return propSpecs;
   }

   /**
    * Reads the next amount of input, interpreting it as a value set.
    * Specifically, expects the next token to be `{', and reads until the first
    * occurrence of `}' thereafter, or until I/O or parse error occurs.
    * 
    * @param phony
    * whether to mark this {@code PropertySpecification} as phony
    * @param propPath
    * the property path of the property associated with this value set
    * @return a list of combinatorial property specifications, each expanded
    * from the one read in and filled with the property and the value set
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected List<PropertySpecification> readCombinatorialValueSet (
      boolean phony, String propPath) throws IOException {
      PropertySpecification propSpec =
         new PropertySpecification (phony, propPath, COMBINATORIAL, -1);
      rtok.scanCharacter ('{');
      while (rtok.nextToken () != '}') {
         if (rtok.tokenIsQuotedString (mySettings.delim)) {
            propSpec.add (rtok.sval);
         }
         else {
            throwAppropriateException ();
         }
      }
      if (propSpec.size () == 0) {
         // If no values provided (i.e. had empty set "{}"), add the null value.
         propSpec.add (null);
      }
      return expandPropertyPath (propSpec);
   }

   /**
    * Reads the next amount of input, interpreting it as a distribution vector.
    * Specifically, expects the next token to be `[', and reads until the first
    * occurrence of `]' thereafter, or until I/O or parse error occurs.
    * 
    * @param phony
    * whether to mark this {@code PropertySpecification} as phony
    * @param propPath
    * the property path of the property associated with this value set
    * @return a list of probabilistic property specifications, each expanded
    * from the one read in and filled with the property and the distribution
    * vector
    * @throws IOException
    * if an I/O error occurs, or the file format is incorrect (causing a parse
    * error)
    */
   protected List<PropertySpecification> readProbabilisticDistributionVector (
      boolean phony, String propPath) throws IOException {
      PropertySpecification propSpec =
         new PropertySpecification (phony, propPath, PROBABILISTIC, -1);
      rtok.scanCharacter ('[');
      while (rtok.nextToken () != ']') {
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            // Since it's a word, we now expect tokens in the order:
            // <DistName> ( <param1value>, ... , <paramNvalue> )
            // where N = dist.numParams()
            Distribution dist = Distribution.valueOf (rtok.sval);
            int numParams = dist.numParams ();
            ArrayList<Double> params = new ArrayList<> ();
            rtok.scanCharacter ('(');
            for (int i = 0; i < numParams - 1; i++) {
               params.add (rtok.scanNumber ());
               rtok.scanCharacter (',');
            }
            params.add (rtok.scanNumber ());
            rtok.scanCharacter (')');

            int id = mySampler.addDistribution (dist, params);
            propSpec.add (id, new DistributionPrinter (dist, params));
         }
         else {
            throwAppropriateException ();
         }
      }
      if (propSpec.size () == 0) {
         throw new IOException (
            "the probabilistic value set on line " + rtok.lineno ()
            + " cannot be empty.");
      }
      return expandPropertyPath (propSpec);
   }

   /**
    * Decorates the given {@link PropertySpecification} with the given
    * decorator.
    * <p>
    * If the given {@code PropertySpecification} has the same
    * {@link SpecificationType} as the decorator, then it is simply returned
    * (unmodified). Otherwise, a new {@code PropertySpecification} of the same
    * type as the decorator is returned, properly decorated.
    *
    * @param propSpec
    * the {@code PropertySpecification} to decorate
    * @param decorator
    * the type of decoration
    * @param decArgs
    * the arguments to the decorator (or a list of size 0 if there were no
    * arguments to the decorator)
    * @return a possibly decorated {@code PropertySpecification}
    * @throws IOException
    * if the given {@code PropertySpecification} is combinatorial, the decorator
    * is probabilistic, and the decorator arguments list has size greater than
    * 0, but not equal to the size of the {@code PropertySpecification}'s value
    * set.
    */
   protected PropertySpecification decoratePropSpec (
      PropertySpecification propSpec, SpecificationType decorator,
      ArrayList<Number> decArgs) throws IOException {
      if (propSpec.getSpecificationType () == decorator) {
         return propSpec;
      }
      PropertySpecification decoratedPropSpec =
         new PropertySpecification (
            propSpec.isPhony (), propSpec.getPropertyPath (), decorator,
            propSpec.getIndex ());
      switch (decorator) {
         case COMBINATORIAL:
            for (int i = 0; i < decArgs.get (0).intValue (); i++) {
               decoratedPropSpec.add (
                  Utils.createDistributionVectorAsString (mySampler, propSpec));
            }
            break;
         case PROBABILISTIC:
            int size = decArgs.size ();
            if (size > 0 && propSpec.size () != size) {
               throw new IOException (
                  "the combinatorial property specification on line "
                  + rtok.lineno () + " has a decorator with a different "
                  + "number of arguments than the size of its value set");
            }
            double prob = -1;
            if (size == 0) {
               prob = 1.0 / propSpec.size ();
            }
            HashMap<String,Double> pmf = new HashMap<> ();
            Iterator<Number> it = decArgs.iterator ();
            for (Object value : propSpec.getCollection ()) {
               if (size > 0) {
                  prob = it.next ().doubleValue ();
               }
               pmf.put ((String)value, prob);
            }
            createSampler ();
            int id = mySampler.addCategoricalDistribution (pmf);
            decoratedPropSpec.add (id, new DistributionPrinter (pmf));
            break;
      }
      return decoratedPropSpec;
   }

   /**
    * Expands the component identifier set part (e.g. the
    * <code>{[0-10], a, b, c}</code> part) of the property path of the given
    * {@link PropertySpecification}, returning a list of
    * {@code PropertySpecification}s.
    * 
    * @param propSpec
    * the {@code PropertySpecification} whose path need expanding
    * @return a list of expanded {@code PropertySpecification}s
    * @throws IOException
    * if the set's format is incorrect
    */
   protected List<PropertySpecification> expandPropertyPath (
      PropertySpecification propSpec) throws IOException {
      Pattern setPat = Pattern.compile ("\\{([^\\}]*)\\}");
      String propPath = propSpec.getPropertyPath ();
      Matcher setMat = setPat.matcher (propPath);
      List<PropertySpecification> propSpecs = new LinkedList<> ();
      if (setMat.find ()) { // find {*} substrings in the property path
         String setInterior = setMat.group (1);
         if (setInterior.contains ("{")) {
            throw new IOException (
               "in \"" + propPath
               + "\": incorrectly balanced braces, or nested braces");
         }

         List<String> expandedNames = expandNameSet (setInterior, propPath);
         for (String name : expandedNames) {
            // Remove the quotes from the string
            String propPathNoQuotes =
               Pattern.compile ("\"|'").matcher (propPath).replaceAll ("");

            // Replace the component identifier set (e.g. "{[0-1], 3, 4}")
            // in place with one of the expanded names.
            propPathNoQuotes =
               setPat.matcher (propPathNoQuotes).replaceFirst (name);

            // Make a new PropertySpecification with the new property path.
            PropertySpecification clone = propSpec.clone ();
            clone.setPropertyPath (propPathNoQuotes);
            propSpecs.add (clone);
         }
         return propSpecs;
      }
      else if (propPath.contains ("{") || propPath.contains ("}")) {
         throw new IOException (
            "in \"" + propPath + "\": incorrectly balanced braces");
      }
      else { // No component identifier set in the property path
         // remove the quotes from the string
         propSpec.setPropertyPath (
            Pattern.compile ("\"|'").matcher (propPath).replaceAll (""));
         propSpecs.add (propSpec.clone ()); // Clone to increment the index.
         return propSpecs;
      }
   }

   /**
    * Given just a string of ranges and/or names that had appeared between curly
    * braces, tokenizes this string to create a list of names. Also expands the
    * ranges into integer component numbers.
    * 
    * @param setInterior
    * the string between the curly braces, without the curly braces anymore
    * @param propPath
    * the entire original string of the property path
    * @return a list of component identifiers tokenized from the setInterior
    * @throws IOException
    * if the format is incorrect
    */
   protected List<String> expandNameSet (String setInterior, String propPath)
      throws IOException {
      Pattern rangePat =
         Pattern.compile ("\\s*(\\[\\s*([0-9]+)\\s*-\\s*([0-9]+)\\s*\\])\\s*");
      Pattern otherPat = Pattern.compile ("\\s*([^\\[\\s]+)\\s*");
      LinkedList<String> expandedNames = new LinkedList<> ();
      boolean foundOne = false;
      while (true) {
         Matcher rangeMat = rangePat.matcher (setInterior);
         if (rangeMat.lookingAt ()) {
            setInterior = expandRange (rangeMat, expandedNames, propPath);
            foundOne = true;
            continue;
         }

         Matcher otherMat = otherPat.matcher (setInterior);
         if (otherMat.lookingAt ()) {
            addAtMostOnce (
               otherMat.group (1), expandedNames, propPath, otherMat.group (1));
            setInterior = otherMat.replaceFirst ("");
            foundOne = true;
            continue;
         }

         if (setInterior.isEmpty ()) {
            break; // Didn't find anything during this loop iteration.
         }
         else {
            throw new IOException (
               "in \"" + propPath + "\": parse error around `"
               + setInterior.charAt (0) + "'");
         }
      }
      if (!foundOne) {
         throw new IOException (
            "in \"" + propPath + "\": empty set \"{}\" is not allowed");
      }
      return expandedNames;
   }

   /**
    * Given a {@link String} range (e.g. {@code [0-9]}) through a
    * {@link Matcher}, expands the range into integers (as {@link String}s), and
    * adds them to the given names list.
    * 
    * @param rangeMat
    * the {@link Matcher}
    * @param expandedNames
    * the names list
    * @param propPath
    * the entire original string of the property path
    * @return the {@link Matcher}'s string with the range part removed
    * @throws IOException
    * if the format is incorrect
    */
   protected String expandRange (
      Matcher rangeMat, List<String> expandedNames, String propPath)
      throws IOException {
      int first = Integer.decode (rangeMat.group (2));
      int last = Integer.decode (rangeMat.group (3));
      if (first > last) {
         throw new IOException (
            "in \"" + rangeMat.group (1) + "\" in \"" + propPath + "\": \""
            + first + "\" cannot be greater than \"" + last + "\"");
      }
      else {
         for (int j = first; j <= last; j++) {
            addAtMostOnce (
               rangeMat.group (1), expandedNames, propPath,
               Integer.toString (j));
         }
      }
      return rangeMat.replaceFirst ("");
   }

   /**
    * Adds {@code newEl} to {@code expandedNames} if, and only if, it does not
    * already contain it.
    * 
    * @param group
    * a {@link Matcher} group containing {@code newEl}
    * @param expandedNames
    * the names list
    * @param propPath
    * the entire original string of the property path
    * @param newEl
    * the new {@link String} to add to {@code expandedNames}
    */
   protected void addAtMostOnce (
      String group, List<String> expandedNames, String propPath, String newEl) {
      if (!expandedNames.contains (newEl)) {
         expandedNames.add (newEl);
      }
      else {
         Utils.printCond (
            mySettings.console, mySettings.interactionLevel != 0, System.err,
            "BatchManager: warning: ignoring duplicate component \"" + newEl
            + "\" in \"" + group + "\" in \"" + propPath + "\"");
      }
   }

   /**
    * Accesses the Jython Console to create a new instance of DistSamplerImpl.
    * 
    * @throws FileNotFoundException
    * if the file "DistSamplerImpl.py" is not found
    * @throws RuntimeException
    * if the DistSamplerImpl cannot be instantiated
    */
   protected void createSampler ()
      throws FileNotFoundException, RuntimeException {
      if (mySampler == null) {
         if (mySettings.console == null) {
            throw new RuntimeException (
               "BatchManager: an active Jython Console is "
               + "required to load a Distribution Sampler.");
         }
         File file =
            ArtisynthPath
               .getSrcRelativeFile (BatchManager.class, "DistSamplerImpl.py");
         FileInputStream input = new FileInputStream (file);
         try {
            mySettings.console.execfile (input, file.toString ());
            PyObject samplerClass =
               mySettings.console.getConsole ().get ("DistSamplerImpl");
            mySampler =
               (DistributionSampler)samplerClass
                  .__call__ ().__tojava__ (DistributionSampler.class);
            if (mySettings.seed != -1) {
               mySampler.setSeed (mySettings.seed);
               Utils.printCond (
                  mySettings.console,
                  mySettings.interactionLevel == 2 || mySettings.debug,
                  System.out,
                  "BatchManager: random number generator seed set to: "
                  + mySettings.seed);
            }
         }
         catch (Exception e) {
            e.printStackTrace ();
            throw new RuntimeException (
               "BatchManager: could not load Distribution Sampler. Ensure "
               + "jdistlib jar is on the build path or class path.\n"
               + "Download jdistlib at http://jdistlib.sourceforge.net/");
         }
      }
   }

}
