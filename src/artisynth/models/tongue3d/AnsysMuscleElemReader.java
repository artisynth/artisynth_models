package artisynth.models.tongue3d;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.ArrayList;

import maspack.matrix.Vector3d;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.util.ArtisynthPath;

/**
 * A class to read muscle group and element definitions ICP's CreateMuscles.mac
 * 
 */
public class AnsysMuscleElemReader {

   /**
    * for testing
    */
   public static void main (String[] args) {

      FemMuscleModel model = new FemMuscleModel ();
      try {
      AnsysMuscleElemReader.read (model, new FileReader (
         ArtisynthPath.getSrcRelativePath (HexTongueDemo.class, "geometry/CreateMuscles.mac")));
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

      System.out.println("done MacMuscleDef test");
      
   }
   
   
   /** 
    * Adds muscle bundles and associated elements to the specified FEM model
    * 
    * @param model FEM model to be populated by muscle bundles
    * @param reader reader from which to read .mac file
    * @throws IOException if this is a problem reading the file
    */
   public static void read (
      FemMuscleModel model, Reader reader)
      throws IOException {

      ArrayList<String> bundleNames = new ArrayList<String> ();
      ArrayList<ArrayList<Integer>> bundleElemIdxs = new ArrayList<ArrayList<Integer>> ();
      
      readBundleNames (reader, bundleNames, bundleElemIdxs);
      
      for (int i = 0; i < bundleNames.size (); i++) {
         MuscleBundle mb = new MuscleBundle ();
         mb.setName (bundleNames.get (i));
         model.addMuscleBundle(mb);

         ArrayList<Integer> elemIdxs = bundleElemIdxs.get (i);
         for (Integer idx : elemIdxs) {
            // ansys elems are one indexed, convert to zero-indexed
            if (model.numElements () >= idx) {
               MuscleElementDesc mf = new MuscleElementDesc ();
               mf.setElement (model.getElement (idx - 1));
               mf.setDirection(Vector3d.X_UNIT);
               mb.addElement(mf);
            }
         }
      }
   }

   
   private static void readBundleNames (Reader reader, ArrayList<String> bundleNames,
      ArrayList<ArrayList<Integer>> bundleElemIdxs) throws IOException {

      
      ReaderTokenizer rtok = new ReaderTokenizer (new BufferedReader (reader));
      rtok.commentChar ('!');
      int numbundles = 0;
      if (findString("nb_muscles", rtok)) {
         rtok.scanCharacter ('=');
         numbundles = rtok.scanInteger ();
      }
      

      
      for (int i = 0; i < numbundles; i++) {
         findString ("muscleName", '(', rtok);
         rtok.scanInteger ();
         rtok.scanCharacter (')');
         rtok.scanCharacter ('=');
         rtok.nextToken ();
         bundleNames.add (rtok.sval);

      }
      
      for (int i = 0; i < numbundles; i++) {
         ArrayList<Integer> elemIdxs = new ArrayList<Integer> ();
         while (rtok.nextToken () != ReaderTokenizer.TT_WORD
         || rtok.sval.compareTo ("CM") != 0.0) {
            findString ("ELEM", ',', rtok);
            rtok.scanCharacter (',');
            int startidx = rtok.scanInteger ();
            rtok.nextToken ();
            if (rtok.ttype != ',') {
               // single idx only, continue to next ELEM token
               elemIdxs.add (startidx);
               rtok.pushBack ();
               continue;
            }
            // idx range specified, read all
            int endidx = rtok.scanInteger ();
            rtok.nextToken ();
            int inc;
            if (rtok.ttype != ',') {
               inc = 1;
               rtok.pushBack ();
            }
            else { // explicit increment specified, read inc 
               inc = rtok.scanInteger ();
            }

            for (int idx = startidx; idx <= endidx; idx+=inc) {
               elemIdxs.add (idx);
            }
           
         }
         rtok.scanCharacter (',');
         rtok.nextToken ();
         String groupName = rtok.sval;

         bundleElemIdxs.add (elemIdxs);
      }

   }

   private static boolean findString (String str, ReaderTokenizer rtok)
   throws IOException {
      
      return findString (str, null, rtok);
   }
   
   private static boolean findString (String str, Character expectedNextToken, ReaderTokenizer rtok)
      throws IOException {

      boolean foundString = false;
      while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype == ReaderTokenizer.TT_WORD && rtok.sval.compareTo (str) == 0.0) {
            if (expectedNextToken != null) {
               try {
                  // try to scan next character token
                  rtok.scanCharacter (expectedNextToken);
               }
               catch (Exception e) {
                  rtok.pushBack (); // wrong next character found, continue search
                  continue;
               }
            }
            foundString = true;
            break;
         }
          
      }
      return foundString;
   }

}
