package artisynth.models.larynx_QL2.tools;

import java.util.ArrayList;
import java.util.HashMap;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.models.larynx_QL2.tools.Neighbour.NeighbourType;

/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class Neighbour {
       //Hex element node neighbourhood index arrays
       public static int[][] orthogonalNeighbourIndices = new int[][]{{1, 3, 4}, {0, 2, 5}, {1, 3, 6}, {0, 2, 7}, {0, 5, 7}, {1, 4, 6}, {5, 7, 2}, {4, 6, 3}};
       public static int[][] faceDiagonalNeighbourIndices = new int[][]{{2, 7, 5}, {3, 4, 6}, {0, 5, 7}, {1, 4, 6}, {1, 3, 6}, {0, 2, 7}, {1, 3, 4}, {0, 2, 5}};
       public static int[] volumeDiagonalNeighbourIndices = new int[]{6, 7, 4, 5, 2, 3, 0, 1};

       public FemNode3d node;
       public NeighbourType type;
       public double optimalDistance;

       public static enum NeighbourType {
              ORTHOGONAL,
              FACE_DIAGONAL,
              VOLUME_DIAGONAL;
       }

       public Neighbour(FemNode3d neighbour, NeighbourType neighbourType, double optimalDistance) {
              this.node = neighbour;
              this.type = neighbourType;
              this.optimalDistance = optimalDistance;
       }

       public static boolean testForContainment(ArrayList<Neighbour> neighbours, FemNode3d testNeighbour) {
              for (Neighbour neighbour : neighbours) {
                     if (neighbour.node.equals(testNeighbour)) {
                            return true;
                     }
              }
              return false;
       }


       public static boolean testForContainment(ArrayList<Neighbour> neighbours, FemNode3d testNeighbour, NeighbourType neighbourType) {
    	   for (Neighbour neighbour : neighbours) {
    		   if (neighbour.node.equals(testNeighbour) && neighbour.type == neighbourType) {
    			   return true;
    		   }
    	   }
    	   return false;
       }


       public static HashMap<Integer, ArrayList<Neighbour>> getNeighbourMap(FemModel3d fem, double optimalOrthogonalLength) {
              HashMap<Integer, ArrayList<Neighbour>> neighbourMap = new HashMap<Integer, ArrayList<Neighbour>>();
              for (FemNode3d node : fem.getNodes()) {
                     neighbourMap.put(node.getNumber(), findNeighbours(node, optimalOrthogonalLength));
              }
              return neighbourMap;
       }


       /** Produces an array of FEM nodes neighbour to the input node. **/
       public static ArrayList<Neighbour> findNeighbours(FemNode3d node, double optimalOrthogonalDistance) {
              ArrayList<Neighbour> neighbourNodes = new ArrayList<Neighbour>();
              int numElements = node.getElementDependencies().size();

              if (numElements > 0) {
                     for (FemElement3d elem : node.getElementDependencies()) {
                            int count = 0;
                            FemNode3d[] elemNodes =  elem.getNodes();
                            for (FemNode3d elemNode : elemNodes) {
                                   if (elemNode.equals(node)) {
                                          break;
                                   }
                                   count++;
                            }

                            if (count < 8) {
                                   for (int i : Neighbour.orthogonalNeighbourIndices[count]) {
                                          FemNode3d testNeighbour = elemNodes[i];
                                          if (!Neighbour.testForContainment(neighbourNodes, testNeighbour)) {
                                                 neighbourNodes.add(new Neighbour(testNeighbour, Neighbour.NeighbourType.ORTHOGONAL, optimalOrthogonalDistance));         
                                          }
                                   }

                                   double faceDiagonalOrthogonalDistance = Math.sqrt(2*optimalOrthogonalDistance*optimalOrthogonalDistance);
                                   for (int i : Neighbour.faceDiagonalNeighbourIndices[count]) {
                                          FemNode3d testNeighbour = elemNodes[i];
                                          if (!Neighbour.testForContainment(neighbourNodes, testNeighbour)) {
                                                 neighbourNodes.add(new Neighbour(testNeighbour, Neighbour.NeighbourType.FACE_DIAGONAL, faceDiagonalOrthogonalDistance));        
                                          }
                                   }

                                   double optimalVolumeDiagonalDistance = Math.sqrt(3*optimalOrthogonalDistance*optimalOrthogonalDistance);
                                   FemNode3d testNeighbour = elemNodes[Neighbour.volumeDiagonalNeighbourIndices[count]];
                                   if (!Neighbour.testForContainment(neighbourNodes, testNeighbour)) {
                                          neighbourNodes.add(new Neighbour(testNeighbour, Neighbour.NeighbourType.VOLUME_DIAGONAL, optimalVolumeDiagonalDistance));
                                   }
                            } else {
                                   System.out.println("Node " + node.getNumber() + " not registered with element " + elem.getNumber() + " dependency");
                            }

                     }
              }


              return neighbourNodes;
       }

}

