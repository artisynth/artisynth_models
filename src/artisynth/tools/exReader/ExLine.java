package artisynth.tools.exReader;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

public class ExLine {

   ExNode[] myNodes;
   int[] nodeVersions;
   ArrayList<ExFace> dependentFaces;
   int idx;

   public ExLine () {
      myNodes = new ExNode[2];
      nodeVersions = new int[2];
      dependentFaces = new ArrayList<ExFace>();
   }

   public static ExLine findOrCreateLine(ExNode node1, int version1, ExNode node2, int version2) {
      for (ExLine line : node1.getDependentLines()) {
         if (line.connects(node1, version1, node2, version2)) {
            return line;
         }
      }
      // if we never find a connecting line, create it

      ExLine newLine = new ExLine();
      newLine.setNodes(node1, node2);
      return newLine;

   }

   public boolean connects(ExNode node1, int version1, ExNode node2,
      int version2) {
         
      if ( myNodes[0] == node1 && myNodes[1] == node2 ) {
//         if (nodeVersions[0] != version1 || nodeVersions[1] != version2) {
//            System.out.println("Line connects points but uses different node version");
//            return false;
//         }
         return true;
      } else if (myNodes[0] == node2 && myNodes[1] == node1 ) {
//         if (nodeVersions[0] != version2 || nodeVersions[1] != version1) {
//            System.out.println("Line connects points but uses different node version");
//            return false;
//         }
         return true;
      }
      return false;
   }

   public void setIdx(int index) {
      idx = index;
   }

   public int getIdx() {
      return idx;
   }

   public void setNodes(ExNode start, ExNode end) {
      setNodes(start, 0, end, 0);
   }

   public void setNodes(ExNode start, int startVersion, ExNode end,
      int endVersion) {
      setNode(0, start, startVersion);
      setNode(1, end, endVersion);
   }

   public void setNode(int idx, ExNode node, int version) {

      // remove existing dependencies
      if (myNodes[idx] != null) {
         myNodes[idx].removeDependentLine(this);
      }

      // set node
      myNodes[idx] = node;
      nodeVersions[idx] = version;

      // add dependencies
      node.addDependentLine(this);
   }

   public ArrayList<ExFace> getDependenFaces() {
      return dependentFaces;
   }

   public boolean addDependentFace(ExFace face) {
      if (!dependentFaces.contains(face)) {
         dependentFaces.add(face);
         return true;
      }
      return false;
   }

   public boolean removeDependentFace(ExFace face) {
      if (dependentFaces.contains(face)) {
         dependentFaces.remove(face);
         return true;
      }
      return false;
   }

   public ExNode[] getNodes() {
      ExNode[] ret = new ExNode[myNodes.length];

      for (int i = 0; i < myNodes.length; i++) {
         ret[i] = myNodes[i];
      }
      return ret;
   }

   public int[] getNodeVersions() {
      int[] ret = new int[nodeVersions.length];
      for (int i = 0; i < nodeVersions.length; i++) {
         ret[i] = nodeVersions[i];
      }
      return ret;
   }

   public boolean isCollapsed() {
      return (myNodes[0] == myNodes[1]);
   }

   public ArrayList<ExElement> getDependentElements() {
      ArrayList<ExElement> ret = new ArrayList<ExElement>();

      for (ExFace face : dependentFaces) {
         ArrayList<ExElement> fe = face.getDependentElements();

         for (ExElement elem : fe) {
            if (!ret.contains(elem)) {
               ret.add(elem);
            }
         }
      }
      return ret;
   }

   public boolean hasNode(ExNode node) {

      for (ExNode myNode : getNodes()) {
         if (node == myNode) {
            return true;
         }
      }
      return false;
   }

   public boolean hasNodes(ArrayList<ExNode> nodes) {

      for (ExNode node : nodes) {
         boolean found = false;
         for (int i = 0; i < myNodes.length; i++) {
            if (node == myNodes[i]) {
               found = true;
               break;
            }
         }

         if (found == false) {
            return false;
         }
      }

      // all nodes are found
      return true;
   }

   public int getNodeIdx(ExNode node) {
      for (int i = 0; i < myNodes.length; i++) {
         if (node == myNodes[i]) {
            return i;
         }
      }
      return -1;
   }

   public boolean equals(ArrayList<ExNode> nodes) throws NodeOrderingException {
      // assume zero version
      ArrayList<Integer> versions = new ArrayList<Integer>();
      versions.add(0);
      versions.add(0);  
           
      return equals(nodes,versions);
   }
   
   public boolean equals(ArrayList<ExNode> nodes, ArrayList<Integer> versions)
      throws NodeOrderingException {
      if (nodes.size() != myNodes.length) {
         return false;
      }

      boolean exact = true;
      for (int i = 0; i < myNodes.length; i++) {
         if (nodes.get(i) != myNodes[i]) { //|| versions.get(i) != nodeVersions[i]) {
            exact = false;
         }
      }

      if (exact) {
         if (versions.get(0) != nodeVersions[0] || versions.get(1) != nodeVersions[1]) {
            
            boolean err = false;
            // try to fix versions            
            if (versions.get(0) != nodeVersions[0]) {
               if (nodes.get(0).equalCoordinateVersions(nodeVersions[0], versions.get(0))) {
                  versions.set(0, nodeVersions[0]);
               } else {
                  err = true;
               }
            }
         
         
            if (versions.get(1) != nodeVersions[1]) {
               if (nodes.get(1).equalCoordinateVersions(nodeVersions[1], versions.get(1))) {
                  versions.set(1, nodeVersions[1]);
               } else {
                  err=true;
               }
            }
            
            err=false;  //XXX: disable error
            if (err) {
               String errMsg = "Line " + getIdx() + " has two different versions (" +
                  + nodes.get(0).getNodeIdx()+"["+versions.get(0)+"], " + nodes.get(1).getNodeIdx()+"["+versions.get(1)+"]) && (" 
                  + nodes.get(0).getNodeIdx()+"["+nodeVersions[0]+"], " + nodes.get(1).getNodeIdx()+"["+nodeVersions[1]+"])";
               throw new NodeOrderingException(this, errMsg);
            }
            
         }
            
         return true;
      }

      if (connects(nodes.get(0),versions.get(0), nodes.get(1), versions.get(1))) {
         throw new NodeOrderingException(
            this, "( " 
               + nodes.get(0).getNodeIdx() + " "
               + nodes.get(1).getNodeIdx() + " should be " +
               + nodes.get(1).getNodeIdx() + " "
               + nodes.get(0).getNodeIdx() + " )");
      }

      return false;
   }

   public boolean isOnSurface() {

      // check if any dependent faces are on surface
      for (ExFace face : dependentFaces) {
         if (face.isOnSurface()) {
            return true;
         }
      }
      return false;

   }

}
