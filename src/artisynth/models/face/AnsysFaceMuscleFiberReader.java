package artisynth.models.face;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.util.HashMap;
import java.util.LinkedList;

import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.gui.editorManager.SetDefaultCollisionsCommand;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.mechmodels.Muscle;

/**
 * A class to read muscle fiber definitions from ICP's Fibres.mac
 * 
 */
public class AnsysFaceMuscleFiberReader {
   
  
   /**
    * Adds muscle fibers to the face FEM model. Based on AnsysMuscleFiberReader
    * for tongue model, with following modifications: 
    * -- "nb_muscles" is first in file 
    * -- order of FIBER, and NB_NODES_FIBER indices are reversed 
    * -- nodeIds are non-sequential, therefore require hashmap argument, 
    *    which is generated in AnsysReader.readNodeFile()
    * 
    * @param model
    * FEM face model to be populated by muscle bundles
    * @param nodeIdToIndex
    * mapping from node ids to fem marker indices
    * @param fibreFileReader
    * reader for the face muscle topology .mac file
    * @throws IOException
    * if this is a problem reading the file
    */
   public static void read (
      FemMuscleModel model, HashMap<Integer,Integer> nodeIdToIndex, Reader fibreFileReader)
      throws IOException {

      
      ReaderTokenizer rtok = new ReaderTokenizer (new BufferedReader (fibreFileReader));
      rtok.commentChar ('!');
      rtok.wordChar ('_');
      
      
      findSubString ("nb_muscles", '=', rtok);
      int max_muscles = rtok.scanInteger ();

      findSubString ("nb_max_fibres", '=', rtok);
      int max_fibers = rtok.scanInteger ();
      
      findSubString ("nb_max_noeuds", '=', rtok);
      int max_nodes = rtok.scanInteger ();

      
      
      int[][][] nodeIds = new int[max_muscles][max_fibers][max_nodes];
      int[][] numNodesPerFiber = new int[max_muscles][max_fibers];
      int[] numFibersPerMuscle = new int[max_muscles];
      int numMuscles = 0;
      
      String strtok = null;
      while ((strtok = findSubString ("FIBER", '(', rtok)) != null) {
         if (strtok.compareTo ("FIBER") == 0.0) {
            int mi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (',');
            int fi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (',');
            int ni = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (')');
            rtok.scanCharacter ('=');
            int nodeId = rtok.scanInteger ();
            nodeIds[mi][fi][ni] = nodeId;
         }
         if (strtok.compareTo ("NB_NODES_FIBER") == 0.0) {
            int mi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (',');
            int fi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (')');
            rtok.scanCharacter ('=');
            int numNodes = rtok.scanInteger ();
            numNodesPerFiber[mi][fi] = numNodes;
         }
         else if (strtok.compareTo ("NB_FIBERS") == 0.0) {
            int mi = rtok.scanInteger ()-1; // zero-based indexing
            rtok.scanCharacter (')');
            rtok.scanCharacter ('=');
            int numFibers = rtok.scanInteger ();
            numFibersPerMuscle[mi] = numFibers;
            numMuscles++;
         }
      }
      

   /* debugging */
//      for (int mi = 0; mi < numMuscles; mi++) {
//         for (int fi = 0; fi < numFibersPerMuscle[mi]; fi++) {
//            for (int ni = 0; ni < numNodesPerFiber[mi][fi] - 1; ni++) {
//               if (nodeIdToIndex.get (nodeIds[mi][fi][ni]) != null)
//                  System.out.printf("%d %d %d : %d = %d\n",mi,fi,ni,nodeIds[mi][fi][ni],nodeIdToIndex.get (nodeIds[mi][fi][ni]));
//            }
//         }
//      }
      
      addMuscleFibers(model, nodeIdToIndex, nodeIds, numNodesPerFiber, numFibersPerMuscle, numMuscles);
      
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

   public static void addMuscleFibers (
      FemMuscleModel fem, HashMap<Integer,Integer> nodeIdToIndex,
      int[][][] nodeIds, int[][] numNodes, int[] numFibers, int numMuscles) {

      for (int mi = 0; mi < numMuscles; mi++) {
         MuscleBundle b = new MuscleBundle ();
         fem.addMuscleBundle (b);
         for (int fi = 0; fi < numFibers[mi]; fi++) {
            LinkedList<Muscle> fascicle = new LinkedList<Muscle> ();
            for (int ni = 0; ni < numNodes[mi][fi] - 1; ni++) {
               Muscle m = new Muscle ();
               Integer mkr0= nodeIdToIndex.get (nodeIds[mi][fi][ni]);
               Integer mkr1 = nodeIdToIndex.get (nodeIds[mi][fi][ni + 1]);
               if (mkr0 == null || mkr1 == null) {
                  continue;
               }
               m.setFirstPoint (fem.markers ().getByNumber (mkr0));
               m.setSecondPoint (fem.markers ().getByNumber (mkr1));
               b.addFibre (m);
               fascicle.add (m);

               setDefaultMuscleFibreProperties(m);

            }
            b.setFibresActive(true);
            b.addFascicle (fascicle);
         }
      }

   }

   public static void setDefaultMuscleFibreProperties(Muscle m) {
      // generic default muscle parameters
      double optLen = m.getFirstPoint ().distance (m.getSecondPoint ());
      ConstantAxialMuscle mat = new ConstantAxialMuscle();
      mat.setAxialMuscleMaterialProps(/*maxForce=*/1.0, optLen, /*maxLen=*/2*optLen, 
	    0, 0, 0, /*forceScaling=*/1.0);
      m.setMaterial(mat);
   }
   
}
