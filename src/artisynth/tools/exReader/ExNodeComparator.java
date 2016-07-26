package artisynth.tools.exReader;

import java.util.Comparator;

public class ExNodeComparator implements Comparator<ExNode> {

   /**
    * [-1,1] means we don't need to re-write header info
    */
   public int compare(ExNode o1, ExNode o2) {

      
      // number of fields
      int fields1 = o1.getFields().size();
      int fields2 = o2.getFields().size();
      if (fields1 > fields2) {
         return 2;
      } else if (fields1 < fields2){
         return -2;
      }
      
      // number of components
      int nComp1=o1.getField(ExNode.COORDINATE_FIELD).getNumComponents();
      int nComp2=o2.getField(ExNode.COORDINATE_FIELD).getNumComponents();
      if ( nComp1 > nComp2 ) {
         return 3;
      } else if (nComp1 < nComp2) {
         return -3;
      }
      
      // number of derivatives
      ExNodeComponent comp1 = o1.getComponent(ExNode.COORDINATE_FIELD, ExNode.COORDINATE_COMPONENTS[0]);
      ExNodeComponent comp2 = o2.getComponent(ExNode.COORDINATE_FIELD, ExNode.COORDINATE_COMPONENTS[0]);
      
      int nDx1 = comp1.getNumDerivatives();
      int nDx2 = comp2.getNumDerivatives();
      if (nDx1 > nDx2) {
         return 4;
      } else if (nDx1 < nDx2) {
         return -4;
      }
      
      // number of versions
      int nVer1 = comp1.getNumVersions();
      int nVer2 = comp2.getNumVersions();
      if (nVer1 > nVer2) {
         return 5;
      } else if (nVer1 < nVer2) {
         return -5;
      }

      // values of each field
      for (int i=0; i<fields1; i++) {
         ExNodeField f1 = o1.getFields().get(i);
         ExNodeField f2 = o2.getFields().get(i);
         for (int j=0; j<f1.getNumComponents(); j++) {
            ExNodeComponent c1 = f1.getComponents().get(j);
            ExNodeComponent c2 = f2.getComponents().get(j);
            
            for (int k=0; k<c1.getNumVersions(); k++) {
               for (int m=0; m<c1.getNumDerivatives(); m++) {
                  double d1 = c1.getDerivative(k, m);
                  double d2 = c2.getDerivative(k, m);
                  
                  if (d1 < d2) {
                     return -1;
                  } else if (d1 > d2) {
                     return 1;
                  }
               }
            }
            
         }
      }
      
      return 0;
   }

}
