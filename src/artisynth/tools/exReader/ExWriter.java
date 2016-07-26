package artisynth.tools.exReader;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;

import maspack.util.NumberFormat;

public class ExWriter {
   
   static final String[] DERIV_STR = { "", 
                                       "(d/ds1)", 
                                       "", 
                                       "(d/ds1,d/ds2,d2/ds1ds2)",
                                       "",
                                       "",
                                       "",
                                       "(d/ds1,d/ds2,d2/ds1ds2,d/ds3,d2/ds1ds3,d2/ds2ds3,d3/ds1ds2ds3)"
                                      };
   
   public static void writeNodes(ArrayList<ExNode> nodes, String region, PrintWriter out, NumberFormat fmt) {
      
      ExNodeComparator cmp = new ExNodeComparator();
      if (fmt == null) {
         fmt = new NumberFormat("% g");
      }
      
      if (!region.startsWith("/")) {
         region = "/" + region;
      }
      
      out.println("Region: " + region);
      
      ExNode headerNode = nodes.get(0);
      String head = getNodeHeader(headerNode);
      out.print(head);
      
      for (ExNode node : nodes) {
         
         // write new header if we must
         if ( Math.abs(cmp.compare(node, headerNode)) > 1 ) {
            headerNode = node;
            head = getNodeHeader(headerNode);
            out.print(head);
         }
         
         out.print(" Node: " + node.getNodeIdx() + "\n");
         for (ExNodeField field : node.getFields()) {
            for (ExNodeComponent comp : field.getComponents()) {
               for (int i=0; i<comp.getNumVersions(); i++) {
                  for (int j=0; j<comp.getNumDerivatives()+1; j++) {
                     out.print(" " + fmt.format(comp.getDerivative(i, j)));
                  }
                  out.print("\n");
               }
            }
            
         }
         
      }
   }
   
   public static void writeElements(ArrayList<ExElement> elems, String region, PrintWriter out, NumberFormat fmt) {
   
      ExElementComparator cmp = new ExElementComparator();
      if (fmt == null) {
         fmt = new NumberFormat("% g");
      }
      
      if (!region.startsWith("/")) {
         region = "/" + region;
      }
      
      out.println("Region: " + region);
      ExRegion newRegion = new ExRegion(elems);
      
      // lines
      out.println(" Shape. Dimension=1, line");
      out.println(" #Scale factor sets=0");
      out.println(" #Nodes=0");
      out.println(" #Fields=0");
      for (ExLine line : newRegion.getLines()) {
         out.println(" Element: 0 0 " + line.getIdx());
      }
      
      // faces
      out.println("Shape. Dimension=2, line*line");
      for (ExFace face : newRegion.getFaces()) {
         out.println(" Element: 0 " + face.getIndex() + " 0");
         out.println(" Faces:");
         
         for (ExLine line : face.getLines()) {
            if (line != null) {
               out.println(" 0 0 " + line.getIdx());
            } else {
               out.println(" 0 0 0");
            }
         }
      }
      
      // elements
      out.println(" Shape. Dimension=3, line*line*line");
      
      ExElement headerElem = elems.get(0);
      String head = getElementHeader(headerElem);
      out.print(head);
      
     for (ExElement elem : elems) {
         
         // write new header if we must
         if ( Math.abs(cmp.compare(elem, headerElem)) > 1 ) {
            headerElem = elem;
            head = getElementHeader(headerElem);
            out.print(head);
         }
         
         // element IDs
         out.println(" Element: " + elem.getIdx() + " 0 0");
         out.println(" Faces:");
         for (ExFace face : elem.getFaces()) {
            if (face != null) {
               out.println(" 0 " + face.getIndex() + " 0");
            } else {
               out.println(" 0 0 0");
            }
         }
         
         // list of nodes
         out.println(" Nodes:");
         ArrayList<ExNode> nodeList = new ArrayList<ExNode>();
         ArrayList<Integer> dupIdxs= new ArrayList<Integer>();
         elem.getNodes(nodeList, dupIdxs);
         
         for (ExNode node : nodeList) {
            out.print(" " + node.getNodeIdx());
         }
         out.print("\n");
         
         // scale factors
         out.println(" Scale factors:");
         double [][] sf = elem.getScaleFactors();
         for (int i=0; i<sf.length; i++) {
            for (int j=0; j<sf[i].length; j++) {
               out.print("  " + fmt.format(sf[i][j]));
            }
            out.print("\n");
         }
         
     }
     
     out.flush();
      
   }
   
   public static String getElementHeader(ExElement elem) {
      
      String head = " #Scale factor sets=1\n";
      
      // interpolation string
      NodeInterpolator.InterpType[] interp = elem.getInterpType();
      
      String interpStr = interpString(interp[0]); 
      int nVals = interpValFactor(interp[0]);
      for (int i=1; i<interp.length; i++) {
         interpStr += "*" + interpString(interp[i]);
         nVals *= interpValFactor(interp[i]);
      }
      head += " " + interpStr + ", ";
      
      int nScaleFactors = 8*nVals;
      head += "#Scale factors=" + nScaleFactors + "\n";
      
      // get nodes
      ArrayList<ExNode> nodeList = new ArrayList<ExNode>();
      ArrayList<Integer> dupIdxs= new ArrayList<Integer>();
      elem.getNodes(nodeList, dupIdxs);
      ExNode[] allNodes = elem.getNodes();
      int [] allVersions = elem.getNodeVersions();
      
      // XXX: assuming element has same fields as first node
      int nFields = nodeList.get(0).getFields().size();  
      head += " #Nodes="+nodeList.size() + "\n";
      head += " #Fields=" + nFields + "\n";
      
      
      for (int i=0; i<nFields; i++) {
         ExNodeField field = nodeList.get(0).getFields().get(i); 
         int nComp = field.getNumComponents();
         
         ArrayList<String> compNames = new ArrayList<String>();
         String fieldName = field.getFieldName();
         
         head += " " + (i+1) + ") " + fieldName + ", ";
         head += field.getFieldType() + ", " + field.getFieldStructure() + ", ";
         head += "#Components=" + nComp + "\n";
      
         // build component string
         for (int j=0; j<nComp; j++) {
            ExNodeComponent comp = field.getComponents().get(j);
            int nDeriv = comp.getNumDerivatives();
            compNames.add(comp.getName());
            
            head += " " + comp.getName() + ". " + interpStr;
            head += ", no modify, standard node based.\n";
            head += "   #Nodes=8\n";
            
            int scaleIdx = 1;
            for (int k=0; k<8; k++) {
               int idx = nodeList.indexOf(allNodes[k])+1;
               head += "   " + idx + ". #Values=" + nVals + "\n";
               head += "     " + "Value indices:";
               
               int baseIdx = allVersions[k]*(nDeriv + 1) + 1;
               
               for (int m=0; m<nVals; m++) {
                  head += " " + (baseIdx + m);
               }
               head += "\n";
               head += "     Scale factor indices:";
               
               for (int m=0; m<nVals; m++) {
                  head += " " + scaleIdx++;
               }
               head += "\n";
            } // nodes
         } // components
         
      } // fields

      return head;
   }
   
   private static int interpValFactor(NodeInterpolator.InterpType interpType) {
      switch(interpType) {
         case CUBIC:
            return 2;
         case LINEAR:
            return 1;
      }
      return 1;
   }
   
   private static String interpString(NodeInterpolator.InterpType interpType) {
      
      switch(interpType) {
         case CUBIC:
            return "c.Hermite";
         case LINEAR:
            return "l.Lagrange";
      }
      
      return "l.Lagrange";
      
   }
   
   public static String getNodeHeader(ExNode node) {
      
      String head = "";
      int nFields = node.getFields().size();
      head += " #Fields=" + nFields + "\n";
      
      
      for (int i=0; i<nFields; i++) {
         ExNodeField field = node.getFields().get(i);
         int nComp = field.getNumComponents();
         
         head += " " + (i+1) + ") ";
         head += field.getFieldName() + ", " + field.getFieldType() + ", ";
         head += field.getFieldStructure() + ", #Components=" + nComp + "\n";
      
         int valIdx = 1;
         for (int j=0; j<nComp; j++) {
            ExNodeComponent comp = field.getComponents().get(j); 
            int nDeriv = comp.getNumDerivatives();
            int nVer = comp.getNumVersions();
            
            head += "  " + comp.getName() + ". Value index="+valIdx + ", ";
            head += "#Derivatives=" + nDeriv + " " + DERIV_STR[nDeriv] + ", ";
            head += "#Versions=" + nVer + "\n";
            
            valIdx += (nDeriv+1)*nVer;
            
         }
         
      }
      
      return head;
   }

}
