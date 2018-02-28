package artisynth.models.tongue3d;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.LinkedList;

import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.materials.BlemkerAxialMuscle;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.util.ArtisynthPath;

/**
 * A class to read muscle fiber definitions from ICP's Fibres.mac
 * 
 */
public class AnsysMuscleFiberReader {

   public static final int rStyInsertionAnsys = 4037;
   public static final int lStyInsertionAnsys = 4042;
   public static final int rStyInsertionArtiSynth = 947;
   public static final int lStyInsertionArtiSynth = 946;
   
   
   /**
    * for testing
    */
   public static void main (String[] args) {

      FemMuscleModel model = new FemMuscleModel ();
      try {
      AnsysMuscleFiberReader.read (model, new FileReader (
         ArtisynthPath.getSrcRelativePath (HexTongueDemo.class, "geometry/Fibers.mac")));
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

      System.out.println("done MacMuscleDef test");
      
   }
   
   
   /** 
    * Adds muscle fibers and associated with bundles to the specified FEM model
    * 
    * @param model FEM model to be populated by muscle bundles
    * @param reader reader from which to read .mac file
    * @throws IOException if this is a problem reading the file
    */
   public static void read (
      FemMuscleModel model, Reader reader)
      throws IOException {

      
      FibreIndicesInfo fibreInfo = new FibreIndicesInfo ();
      readFibreIndices (fibreInfo, reader);
      addMuscles(model, fibreInfo);
      
   }

   
   public static void readFibreIndices (
      FibreIndicesInfo fibreInfo, Reader reader)
      throws IOException {

      
      ReaderTokenizer rtok = new ReaderTokenizer (new BufferedReader (reader));
      rtok.commentChar ('!');
      rtok.wordChar ('_');

      findSubString ("nb_max_fibres", '=', rtok);
      int max_fibers = rtok.scanInteger ();
      
      findSubString ("nb_max_noeuds", '=', rtok);
      int max_nodes = rtok.scanInteger ();
      
      findSubString ("nb_muscles", '=', rtok);
      int max_muscles = rtok.scanInteger ();
      
      
      fibreInfo.nodeIndices = new int[max_muscles][max_fibers][max_nodes];
      fibreInfo.numNodesPerFiber = new int[max_muscles][max_fibers];
      fibreInfo.numFibersPerMuscle = new int[max_muscles];
      fibreInfo.numMuscles = 0;
      
      String strtok = null;
      while ((strtok = findSubString ("FIBER", '(', rtok)) != null) {
         if (strtok.compareTo ("FIBER") == 0.0) {
            int ni = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (',');
            int fi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (',');
            int mi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (')');
            rtok.scanCharacter ('=');
            int nodeIndex = rtok.scanInteger ();
            fibreInfo.nodeIndices[mi][fi][ni] = AnsysToArtisynthNodeIdx(nodeIndex);
         }
         if (strtok.compareTo ("NB_NODES_FIBER") == 0.0) {
            int fi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (',');
            int mi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (')');
            rtok.scanCharacter ('=');
            int numNodes = rtok.scanInteger ();
            fibreInfo.numNodesPerFiber[mi][fi] = numNodes;
         }
         else if (strtok.compareTo ("NB_FIBERS") == 0.0) {
            int mi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (')');
            rtok.scanCharacter ('=');
            int numFibers = rtok.scanInteger ();
            fibreInfo.numFibersPerMuscle[mi] = numFibers;
            fibreInfo.numMuscles++;
         }
      }

      
   }
   
   private static int AnsysToArtisynthNodeIdx(int ansysNodeIdx) {
      switch(ansysNodeIdx) {
         case rStyInsertionAnsys:
            return rStyInsertionArtiSynth;
         case lStyInsertionAnsys:
            return lStyInsertionArtiSynth;
         default:
            return ansysNodeIdx-1;
      }
   }

   
   
   private static String findSubString (String str, Character expectedNextToken, ReaderTokenizer rtok)
      throws IOException {

      while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
         if (rtok.ttype == ReaderTokenizer.TT_WORD && rtok.sval.contains (str)) {
            String strtok = rtok.sval;
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
            return strtok;
         }
          
      }
      return null;
   }
   
   public static void addMuscles (
      FemMuscleModel fem, FibreIndicesInfo fibreInfo) {
      
      if (fibreInfo.numMuscles > fem.getMuscleBundles ().size ()) {
         return;
      }
      
      for (int mi = 0; mi < fibreInfo.numMuscles; mi++) {
         MuscleBundle b = fem.getMuscleBundles ().get (mi);
         for (int fi = 0; fi < fibreInfo.numFibersPerMuscle[mi]; fi++) {
            LinkedList<Muscle> fascicle = new LinkedList<Muscle>();
            for (int ni = 0; ni < fibreInfo.numNodesPerFiber[mi][fi]-1; ni++) {
               Muscle m = new Muscle();
               m.setFirstPoint (fem.getNode (fibreInfo.nodeIndices[mi][fi][ni]));
               m.setSecondPoint (fem.getNode (fibreInfo.nodeIndices[mi][fi][ni+1]));
               b.addFibre (m);
               fascicle.add (m);
               
               // generic default muscle parameters
               ConstantAxialMuscle mat = new ConstantAxialMuscle();
//               PeckAxialMuscle mat = new PeckAxialMuscle();
//               LinearAxialMuscle mat = new LinearAxialMuscle();
               mat.setForceScaling (1.0);

//               BlemkerAxialMuscle mat = new BlemkerAxialMuscle();
//               mat.setExpStressCoeff(0);
               
               mat.setMaxForce (1.0);
               double len = m.getFirstPoint ().distance (m.getSecondPoint ());
               mat.setOptLength (len);
               mat.setMaxLength (2*len);
               m.setMaterial(mat);
            }
            b.addFascicle (fascicle);
         }
      }
      
   }


   public static class FibreIndicesInfo {
      public int[][][] nodeIndices;
      public int[][] numNodesPerFiber;
      public int[] numFibersPerMuscle;
      public int numMuscles;
      
      
   }
   
}
