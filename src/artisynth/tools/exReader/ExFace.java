package artisynth.tools.exReader;

import java.util.ArrayList;
import java.util.Arrays;

import quickhull3d.Point3d;

import maspack.matrix.Vector3d;

public class ExFace {

   public enum Shape {
      QUADRILATERAL,
      TRIANGLE,
      LINE, // collapsed to line
      POINT // collapsed to point
   };

   int idx;
   ExLine myLines[];
   ArrayList<ExElement> dependentElements;

   // line 0: nodes 0 2
   // line 1: nodes 1 3
   // line 2: nodes 0 1
   // line 3: nodes 2 3
   static int[][] lineNodeIdxs = { { 0, 2 },
                                  { 1, 3 },
                                  { 0, 1 },
                                  { 2, 3 } };

   public ExFace () {
      myLines = new ExLine[4];
      dependentElements = new ArrayList<ExElement>();
   }

   public static ExFace findOrCreateFace(ExLine[] lines) {

      for (ExFace face : lines[0].getDependenFaces()) {
         if (face.equals(lines)) {
            return face;
         }
      }

      // if we get here, we have to create the face
      ExFace newFace = new ExFace();
      for (int i = 0; i < 4; i++) {
         newFace.setLine(i, lines[i]);
      }

      return newFace;

   }

   public void setIdx(int index) {
      idx = index;
   }

   public int getIndex() {
      return idx;
   }

   public void setLine(int pos, ExLine line) {

      // remove previous dependencies
      ExLine oldLine = myLines[pos];
      if (oldLine != null) {
         oldLine.removeDependentFace(this);
      }

      // insert new line
      myLines[pos] = line;
      if (line != null) {
         line.addDependentFace(this);
      }

   }

   public boolean addDependentElement(ExElement elem) {
      if (!dependentElements.contains(elem)) {
         dependentElements.add(elem);
         return true;
      }
      return false;
   }

   public boolean removeDependentElement(ExElement elem) {
      if (dependentElements.contains(elem)) {
         dependentElements.remove(elem);
         return true;
      }
      return false;
   }

   public ArrayList<ExElement> getDependentElements() {
      return dependentElements;
   }

   public ExNode[] getNodes() {

      ExNode[] ret = new ExNode[4];

      for (int i = 0; i < myLines.length; i++) {
         if (myLines[i] != null) {
            ExNode[] lineNodes = myLines[i].getNodes();
            for (int j = 0; j < lineNodes.length; j++) {
               ret[lineNodeIdxs[i][j]] = lineNodes[j];
            }
         }
      }
      return ret;
   }

   public int[] getNodeVersions() {
      int[] ret = new int[4];

      for (int i = 0; i < myLines.length; i++) {
         if (myLines[i] != null) {
            int[] lineNodeVersions = myLines[i].getNodeVersions();
            for (int j = 0; j < lineNodeVersions.length; j++) {
               ret[lineNodeIdxs[i][j]] = lineNodeVersions[j];
            }
         }
      }
      return ret;
   }

   public ExLine[] getLines() {
      return Arrays.copyOf(myLines, myLines.length);
   }

   // checks if has nodes, without replacement
   public boolean hasNodes(ArrayList<ExNode> nodeList) {

      ArrayList<ExNode> cmpNodes =
         new ArrayList<ExNode>(Arrays.asList(getNodes()));
      for (int i = 0; i < nodeList.size(); i++) {
         boolean found = false;
         for (int j = 0; j < cmpNodes.size(); j++) {
            if (nodeList.get(i) == cmpNodes.get(j)) {
               found = true;
               cmpNodes.remove(j);
               break;
            }
         }
         // if a node exists in mynodes but not supplied list, we end
         if (found == false) {
            return false;
         }
      }
      return true;

   }

   public Shape getShape() {

      int nValidLines = 0;
      for (int i = 0; i < myLines.length; i++) {
         if (myLines[i] != null) {
            if (!myLines[i].isCollapsed()) {
               nValidLines++;
            }
         }
      }

      switch (nValidLines) {
         case 0:
            return Shape.POINT;
         case 2:
            return Shape.LINE;
         case 3:
            return Shape.TRIANGLE;
         case 4:
            return Shape.QUADRILATERAL;
         default:
            System.out.println("Unknown face shape: " + nValidLines + " lines");
            return Shape.POINT;
      }

   }

   // nodes around the face
   public ExNode[] getNodes(boolean loop) {

      ExNode[] myNodes = getNodes();

      // change order to nodes go around the loop
      if (loop) {
         ExNode tmp = myNodes[3];
         myNodes[3] = myNodes[2];
         myNodes[2] = tmp;
      }
      return myNodes;
   }

   // nodes around the face
   public Shape getNodes(ArrayList<ExNode> nodeList) {
      Shape myShape = getShape();
      nodeList.clear();

      // get nodes going around face in a loop
      ExNode[] myNodes = getNodes(true);

      // add only unique nodes (remove duplicate)
      for (int i = 0; i < myNodes.length; i++) {
         if (myNodes[i] != null) {
            if (!nodeList.contains(myNodes[i])) {
               nodeList.add(myNodes[i]);
            }
         }
      }

      return myShape;
   }

   // nodes looping around the face, CW w.r.t. outward normal from provided
   // element
   // true if order is reversed
   public boolean getNodes(ArrayList<ExNode> nodeList, ExElement elem) {
            
      Shape myShape = getNodes(nodeList);
      int faceIdx = elem.getFaceIdx(this);
      
      boolean reversed = false;
      if (faceIdx % 2 == 1) {
         reversed = true;
      }
      
      if (elem.isFlipped()) {
         reversed = !reversed;
      }
         
      if (reversed) {
         mirrorNodeOrder(nodeList, myShape);
      }

      return reversed;
   }

   public static void mirrorNodeOrder(ArrayList<ExNode> nodeList, Shape shape) {

      ExNode tmp = null; // used for shuffling nodes
      switch (shape) {
         case TRIANGLE:
            // flip second and third node
            tmp = nodeList.get(1);
            nodeList.set(1, nodeList.get(2));
            nodeList.set(2, tmp);
            break;
         case QUADRILATERAL:
            // 1 2 3 4 -> 1 4 3 2
            tmp = nodeList.get(1);
            nodeList.set(1, nodeList.get(3));
            nodeList.set(3, tmp);
            break;
      }
   }

   public boolean hasNode(ExNode node) {

      for (ExNode myNode : getNodes()) {
         if (node == myNode) {
            return true;
         }
      }
      return false;
   }

   public boolean isCollapsed() {
      Shape myShape = getShape();
      if (myShape == Shape.POINT || myShape == Shape.LINE) {
         return true;
      }
      return false;
   }

   public int getLineIdx(ExLine line) {
      for (int i = 0; i < myLines.length; i++) {
         if (line == myLines[i]) {
            return i;
         }
      }
      return -1;
   }

   public int getNodeIdx(ExNode node) {
      ExNode[] myNodes = getNodes();
      for (int i = 0; i < myNodes.length; i++) {
         if (node == myNodes[i]) {
            return i;
         }
      }
      return -1;
   }

   public boolean equals(ArrayList<ExNode> nodeList) throws NodeOrderingException {

      ExNode[] myNodes = getNodes();
      boolean exact = true;

      if (nodeList.size() != myNodes.length) {
         return false;
      }

      for (int i = 0; i < myNodes.length; i++) {
         if (myNodes[i] != nodeList.get(i)) {
            exact = false;
         }
      }

      if (exact) {
         return true;
      } else if (hasNodes(nodeList)) {
         // System.out.println("Warning: face has all nodes, but wrong ordering");
         String msg =  "Face has all nodes, but wrong order (";
         for (ExNode node : nodeList) {
            msg += " " + node.getNodeIdx();
         }
         msg += " should be ";
         for (ExNode node : getNodes()) {
            msg += " " + node.getNodeIdx();
         }
         msg += " )";
         
         throw new NodeOrderingException(this, msg);
         
      }

      return false;
   }

   public boolean hasLines(ArrayList<ExLine> lineList) {

      ArrayList<ExLine> cmpLines =
         new ArrayList<ExLine>(Arrays.asList(getLines()));

      for (int i = 0; i < lineList.size(); i++) {
         boolean found = false;
         for (int j = 0; j < cmpLines.size(); j++) {
            if (lineList.get(i) == cmpLines.get(j)) {
               found = true;
               cmpLines.remove(j);
               break;
            }
         }

         if (found == false) {
            return false;
         }
      }
      return true;

   }

   public boolean equals(ExLine[] lineList) {

      if (lineList.length != myLines.length) {
         return false;
      }

      boolean exact = true;

      ArrayList<ExLine> lines = new ArrayList<ExLine>(); // used for checking
                                                         // hasLines
      for (int i = 0; i < myLines.length; i++) {
         if (myLines[i] != lineList[i]) {
            exact = false;
         }
         lines.add(lineList[i]);
      }

      if (exact) {
         return true;
      } else if (hasLines(lines)) {
         System.out.println("Warning: face has all lines, but wrong ordering");
         return true;
      }

      return false;
   }

   public boolean isOnSurface() {

      // check if only has one dependent element
      if (dependentElements.size() < 2) {
         return true;
      }
      return false;

   }

}
