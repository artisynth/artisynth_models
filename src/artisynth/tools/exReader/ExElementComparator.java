package artisynth.tools.exReader;

import java.util.ArrayList;
import java.util.Comparator;

public class ExElementComparator implements Comparator<ExElement> {

   /**
    * [-1,1] means we don't need to re-write header info
    */
   public int compare(ExElement o1, ExElement o2) {

      // number of components
      int nComp1=o1.getNodes()[0].getField(ExNode.COORDINATE_FIELD).getNumComponents();
      int nComp2=o2.getNodes()[0].getField(ExNode.COORDINATE_FIELD).getNumComponents();
      if ( nComp1 > nComp2 ) {
         return 2;
      } else if (nComp1 < nComp2) {
         return -2;
      }
      
      // interpolation type
      NodeInterpolator.InterpType[] it1 = o1.getInterpType();
      NodeInterpolator.InterpType[] it2 = o2.getInterpType();
      for (int i=0; i<it1.length; i++) {
         if (it1[i].compareTo(it2[i])>0) {
            return 3;
         } else if (it1[i].compareTo(it2[i])<0) {
            return -3;
         }
      }
      
      // shape
      int cmp = o1.getShape().compareTo(o2.getShape());
      if (cmp < 0) {
         return -4;
      } else if (cmp > 0) {
         return 4;
      }

      // locations of duplicated nodes
      ArrayList<ExNode> uniqueNodes1 = new ArrayList<ExNode>();
      ArrayList<Integer> dupNodeIdxs1 = new ArrayList<Integer>();
      ArrayList<ExNode> uniqueNodes2 = new ArrayList<ExNode>();
      ArrayList<Integer> dupNodeIdxs2 = new ArrayList<Integer>();
      o1.getNodes(uniqueNodes1, dupNodeIdxs1);
      o2.getNodes(uniqueNodes2, dupNodeIdxs2);
      for (int i=0; i<dupNodeIdxs1.size(); i++) {
         if (dupNodeIdxs1.get(i) > dupNodeIdxs2.get(i)) {
            return 6;
         } else if (dupNodeIdxs1.get(i) < dupNodeIdxs2.get(i)) {
            return -6;
         }
      }
      
      // version used
      int[] nodesVers1 = o1.getNodeVersions();
      int[] nodesVers2 = o2.getNodeVersions();
      for (int i=0; i<nodesVers1.length; i++) {
         if (nodesVers1[i] > nodesVers2[i]) {
            return 5;
         } else if (nodesVers1[i] < nodesVers2[i]) {
            return -5;
         }
      }

      // nodes themselves
      ExNodeComparator nodeComparer = new ExNodeComparator();
      for (int i=0; i<uniqueNodes1.size(); i++) {
         int val = nodeComparer.compare(uniqueNodes1.get(i), uniqueNodes2.get(i));
         if ( val < 0 ) {
            return -1;
         } else if (val > 0) {
            return 1;
         }
      }
      
      return 0;
   }

}
