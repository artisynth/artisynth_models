package artisynth.tools.exReader;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;

import artisynth.tools.exReader.NodeInterpolator.InterpType;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

public class ExElement {

   public enum Shape {
      HEX, WEDGE, PYRAMID, TET, COLLAPSED
   }

   double[][] scaleFactors; // nodes*values
   ExFace myFaces[];
   int idx;
   boolean reverseComputed = false;
   boolean reversed = false;
   
   NodeInterpolator.InterpType myInterpType[]; // specifies how to interpolate
                                               // each dimension

   // face 0: nodes 0 2 4 6
   // face 1: nodes 1 3 5 7
   // face 2: nodes 0 1 4 5
   // face 3: nodes 2 3 6 7
   // face 4: nodes 0 1 2 3
   // face 5: nodes 4 5 6 7
   public static int[][] faceNodeIdxs = {
                                         { 0, 2, 4, 6 },
                                         { 1, 3, 5, 7 },
                                         { 0, 4, 1, 5 },
                                         { 2, 6, 3, 7 },
                                         { 0, 1, 2, 3 },
                                         { 4, 5, 6, 7 } };
   public static int[][] lineNodeIdxs = {
                                         { 0, 2 },
                                         { 1, 3 },
                                         { 0, 1 },
                                         { 2, 3 },
                                         { 0, 4 },
                                         { 2, 6 },
                                         { 1, 5 },
                                         { 3, 7 },
                                         { 4, 6 },
                                         { 5, 7 },
                                         { 4, 5 },
                                         { 6, 7 }
   };

   // line ordering for the element
   static int[][] faceLineIdxs = {
                                  { 4, 5, 0, 8 },
                                  { 6, 7, 1, 9 },
                                  { 2, 10, 4, 6 },
                                  { 3, 11, 5, 7 },
                                  { 0, 1, 2, 3 },
                                  { 8, 9, 10, 11 }
   };

   public ExElement () {
      myFaces = new ExFace[6];
      reverseComputed = false;
   }

   public ExElement (ArrayList<ExNode> nodes) {

      switch (nodes.size()) {
         case 4:
      }

   }

   public static ExElement
      createTet(ExNode n1, ExNode n2, ExNode n3, ExNode n4) {

      ExNode[] nodes = new ExNode[8];
      nodes[0] = n1;
      nodes[1] = n1;
      nodes[2] = n2;
      nodes[3] = n3;
      nodes[4] = n4;
      nodes[5] = n4;
      nodes[6] = n4;
      nodes[7] = n4;

      return createElement(nodes);
   }

   public static ExElement createPyramid(ExNode n1, ExNode n2, ExNode n3,
      ExNode n4, ExNode n5) {

      ExNode[] nodes = new ExNode[8];
      nodes[0] = n1;
      nodes[1] = n2;
      nodes[2] = n3;
      nodes[3] = n4;
      nodes[4] = n5;
      nodes[5] = n5;
      nodes[6] = n5;
      nodes[7] = n5;

      return createElement(nodes);
   }

   public static ExElement createWedge(ExNode n1, ExNode n2, ExNode n3,
      ExNode n4, ExNode n5, ExNode n6) {

      ExNode[] nodes = new ExNode[8];
      nodes[0] = n1;
      nodes[1] = n1;
      nodes[2] = n3;
      nodes[3] = n2;
      nodes[4] = n4;
      nodes[5] = n4;
      nodes[6] = n6;
      nodes[7] = n5;

      return createElement(nodes);
   }

   public static ExElement createHex(ExNode n1, ExNode n2, ExNode n3,
      ExNode n4, ExNode n5, ExNode n6, ExNode n7, ExNode n8) {

      ExNode[] nodes = new ExNode[8];
      nodes[0] = n1;
      nodes[1] = n2;
      nodes[2] = n4;
      nodes[3] = n3;
      nodes[4] = n5;
      nodes[5] = n6;
      nodes[6] = n8;
      nodes[7] = n7;

      return createElement(nodes);
   }

   //   must have 8 nodes specified in the correct ExElement format ordering
   public static ExElement createElement(ExNode[] nodes) {
      int versions[] = new int[nodes.length];
      for (int i=0; i<versions.length; i++) {
         versions[i] = 0;
      }
      return createElement(nodes,versions);
   }
   
   // must have 8 nodes specified in the correct ExElement format ordering
   public static ExElement createElement(ExNode[] nodes, int[] versions) {
      ExElement newElem = new ExElement();
      NodeInterpolator.InterpType[] interp =
      { InterpType.LINEAR, InterpType.LINEAR, InterpType.LINEAR };
      newElem.setInterpType(interp);

      double sf[][] =
      { { 1 }, { 1 }, { 1 }, { 1 }, { 1 }, { 1 }, { 1 }, { 1 } };

      newElem.setScaleFactors(sf);

      ExLine[] lines = new ExLine[12];
      ExFace[] faces = new ExFace[6];

      // generate all lines
      ExNode[] lineNodes = new ExNode[2];
      int[] lineVers = new int[2];
      for (int i = 0; i < lineNodeIdxs.length; i++) {
         for (int j = 0; j < 2; j++) {
            lineNodes[j] = nodes[lineNodeIdxs[i][j]];
            lineVers[j] = versions[lineNodeIdxs[i][j]];
         }
         if (lineNodes[0] != lineNodes[1]) {
            lines[i] = ExLine.findOrCreateLine(lineNodes[0], lineVers[0], lineNodes[1], lineVers[1]);
         } else {
            lines[i] = null;
         }
      }

      // generate all faces
      ExLine[] faceLines = new ExLine[4];
      for (int i = 0; i < faceLineIdxs.length; i++) {
         int nullCount = 0;
         for (int j = 0; j < 4; j++) {
            faceLines[j] = lines[faceLineIdxs[i][j]];
            if (faceLines[j] == null) {
               nullCount++;
            }
         }
         if (nullCount < 2) {
            faces[i] = ExFace.findOrCreateFace(faceLines);
         } else {
            faces[i] = null;
         }
      }

      // construct element from faces
      for (int i = 0; i < 6; i++) {
         newElem.setFace(i, faces[i]);
      }

      return newElem;
   }

   public int getIdx() {
      return idx;
   }

   public void setIdx(int index) {
      idx = index;
   }

   public void setScaleFactors(double[][] sf) {

      scaleFactors = new double[sf.length][];

      // copy scale factors array
      for (int i = 0; i < sf.length; i++) {
         scaleFactors[i] = new double[sf[i].length];
         for (int j = 0; j < sf[i].length; j++) {
            scaleFactors[i][j] = sf[i][j];
         }
      }

   }

   public void setInterpType(NodeInterpolator.InterpType[] type) {
      myInterpType = Arrays.copyOf(type, type.length);
   }

   public NodeInterpolator.InterpType[] getInterpType() {
      return myInterpType;
   }

   public void setFace(int pos, ExFace face) {
      // remove previous dependencies
      ExFace oldFace = myFaces[pos];
      if (oldFace != null) {
         oldFace.removeDependentElement(this);
      }

      // insert new line
      myFaces[pos] = face;
      if (face != null) {
         face.addDependentElement(this);
      }
      
      reverseComputed = false;
      
   }

   public ExFace[] getFaces() {
      return Arrays.copyOf(myFaces, myFaces.length);
   }

   // warning: configured for hex-like elements (although should be sufficient)
   public ExNode[] getNodes() {
      ExNode[] ret = new ExNode[8];

      for (int i = 0; i < myFaces.length; i++) {
         if (myFaces[i] != null) {
            ExNode[] faceNodes = myFaces[i].getNodes();
            for (int j = 0; j < faceNodes.length; j++) {
               ret[faceNodeIdxs[i][j]] = faceNodes[j];
            }
         }
      }
      return ret;
   }

   // warning: configured for hex-like elements (although should be sufficient)
   public int[] getNodeVersions() {
      int[] ret = new int[8];

      for (int i = 0; i < myFaces.length; i++) {
         if (myFaces[i] != null) {
            int[] faceNodeVersions = myFaces[i].getNodeVersions();
            for (int j = 0; j < faceNodeVersions.length; j++) {
               ret[faceNodeIdxs[i][j]] = faceNodeVersions[j];
            }
         }
      }
      return ret;
   }

   public ExLine[] getLines() {
      ExLine[] ret = new ExLine[12];

      for (int i = 0; i < myFaces.length; i++) {
         if (myFaces[i] != null) {
            ExLine[] faceLines = myFaces[i].getLines();
            for (int j = 0; j < faceLines.length; j++) {
               ret[faceLineIdxs[i][j]] = faceLines[j];
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

   public boolean equals(ArrayList<ExNode> nodeList) {

      ExNode[] myNodes = getNodes();

      if (nodeList.size() != myNodes.length) {
         return false;
      }

      boolean exact = false;

      for (int i = 0; i < myNodes.length; i++) {
         if (myNodes[i] != nodeList.get(i)) {
            exact = false;
         }
      }

      if (exact) {
         return true;
      } else if (hasNodes(nodeList)) {
         System.out
            .println("Warning: elements match, but nodes in different order");
         return true;
      }

      return false;
   }

   public Shape getShape() {

      int nTris = 0;
      int nQuads = 0;

      // count how many triangle and quadrilateral faces
      for (ExFace face : myFaces) {
         if (face != null) {
            ExFace.Shape faceShape = face.getShape();
            if (faceShape == ExFace.Shape.QUADRILATERAL) {
               nQuads++;
            } else if (faceShape == ExFace.Shape.TRIANGLE) {
               nTris++;
            }
         }
      }

      if (nQuads == 6 && nTris == 0) {
         return Shape.HEX;
      } else if (nQuads == 3 && nTris == 2) {
         return Shape.WEDGE;
      } else if (nQuads == 1 && nTris == 4) {
         return Shape.PYRAMID;
      } else if (nQuads == 0 && nTris == 4) {
         return Shape.TET;
      }

      // if we haven't determined its shape, it's collapsed
      return Shape.COLLAPSED;
   }

   public boolean isCollapsed() {
      Shape myShape = getShape();
      if (myShape == Shape.COLLAPSED) {
         return true;
      }
      return false;
   }

   // fills nodeList with the nodes in specified order:
   // Hex: Quad face, CW wrt outword normal
   // Wedge: Triangle face, CW
   // Pyramid: Quad face, CW
   // Tet: Triangle, CW
   public Shape getNodes(ArrayList<ExNode> nodeList) {
      Shape myShape = getShape();

      ExNode[] myNodes = getNodes();
      nodeList.clear();
      int faceIdx = 0;

      // first set of nodes
      switch (myShape) {
         case HEX:
         case PYRAMID:
            // find quadrilateral face
            for (int i = 0; i < myFaces.length; i++) {
               if (myFaces[i] != null) {
                  if (myFaces[i].getShape() == ExFace.Shape.QUADRILATERAL) {
                     faceIdx = i;
                     break;
                  }
               }
            }
            // add quad nodes
            myFaces[faceIdx].getNodes(nodeList);
            break;
         case WEDGE:
         case TET:
            // find triangular face
            for (int i = 0; i < myFaces.length; i++) {
               if (myFaces[i] != null) {
                  if (myFaces[i].getShape() == ExFace.Shape.TRIANGLE) {
                     faceIdx = i;
                     break;
                  }
               }
            }
            myFaces[faceIdx].getNodes(nodeList);
            break;
      }

      // second set of nodes
      ArrayList<ExNode> secondFaceNodes = new ArrayList<ExNode>();
      switch (myShape) {
         case HEX:
         case WEDGE:
            // opposite face
            myFaces[faceIdx + 1].getNodes(secondFaceNodes);
            nodeList.addAll(secondFaceNodes);
            break;
         case PYRAMID:
         case TET:
            // add missing node
            for (int i = 0; i < myNodes.length; i++) {
               if (myNodes[i] != null) {
                  if (!nodeList.contains(myNodes[i])) {
                     nodeList.add(myNodes[i]);
                     break;
                  }
               }
            }
            break;
      }

      // check direction and flip if necessary
      if (isFlipped()) {
         mirrorNodeOrder(nodeList, myShape);
      }

      return myShape;
   }
   
   public int getNodes(ArrayList<ExNode> nodeList, ArrayList<Integer> dupIdxs) {
      
      ArrayList<Integer> dupCount = new ArrayList<Integer>();
      int nDup = 0;
      HashMap<Integer,Integer> dupMap = new HashMap<Integer,Integer>();
      
      nodeList.clear();
      ExNode[] nodes = getNodes();
      for (int i=0; i<8; i++) {
         int idx = nodeList.indexOf(nodes[i]);
         if (idx >= 0) {
            // check if already in duplicate list
            if (dupMap.containsKey(idx)) {
               int k = dupMap.get(idx);
               dupCount.set(k, dupCount.get(k)+1);
            } else {
               dupIdxs.add(idx);
               dupCount.add(1);
               dupMap.put(idx,nDup);
               nDup++;
            }
         } else {
            nodeList.add(nodes[i]);
         }
      }
      
      // sort dupIdx by dupCount
      bubbleSort(dupCount, dupIdxs);
      
      return nDup;
   }
   
   
   // should be fine since we're dealing with 2-3 elements
   private void bubbleSort(ArrayList<Integer> byThis, ArrayList<Integer> follow) {
      
      int n=byThis.size();
      
      for (int i=0; i<n; i++) {
         for (int j=1; j<n-i; j++) {
            
            if (byThis.get(j-1) > byThis.get(j)) {
               Collections.swap(byThis, j, j-1);
               Collections.swap(follow, j, j-1);
            }
            
         }
      }
      
   }

   // changes order of the nodes in "nodeList" to flip the polarization
   // (direction of outward normals)
   public static void mirrorNodeOrder(ArrayList<ExNode> nodeList, Shape shape) {

      ExNode tmp = null; // used for shuffling nodes

      switch (shape) {
         case WEDGE:
            // top side (5<=>6)
            tmp = nodeList.get(4);
            nodeList.set(4, nodeList.get(5));
            nodeList.set(5, tmp);
         case TET:
            // bottom side (2<=>3)
            tmp = nodeList.get(1);
            nodeList.set(1, nodeList.get(2));
            nodeList.set(2, tmp);
            break;
         case HEX:
            // top side (6<=>8)
            tmp = nodeList.get(5);
            nodeList.set(5, nodeList.get(7));
            nodeList.set(7, tmp);
         case PYRAMID:
            // bottom side (2<=>4)
            tmp = nodeList.get(1);
            nodeList.set(1, nodeList.get(3));
            nodeList.set(3, tmp);
            break;
      }
   }

   public int getFaceIdx(ExFace face) {
      for (int i = 0; i < myFaces.length; i++) {
         if (face == myFaces[i]) {
            return i;
         }
      }
      return -1;
   }

   public int getLineIdx(ExLine line) {

      ExLine[] myLines = getLines();
      for (int i = 0; i < myLines.length; i++) {
         if (line == myLines[i]) {
            return i;
         }
      }
      return -1;
   }

   // return 0, 1 or 2 if line follows along xi_0, xi_1 or xi_2
   public int getLineDimension(ExLine line) {

      int lineIdx = getLineIdx(line);

      int lineDim;
      if ((lineIdx == 2) || (lineIdx == 3) || (lineIdx == 10)
         || (lineIdx == 11)) {
         lineDim = 0;
      } else if ((lineIdx == 0) || (lineIdx == 1) || (lineIdx == 8)
         || (lineIdx == 9)) {
         lineDim = 1;
      } else {
         lineDim = 2;
      }
      return lineDim;
   }

   public int[] getFaceDimensions(ExFace face) {

      int faceDims[] = new int[2];

      int faceIdx = getFaceIdx(face);

      faceDims[0] = (faceIdx / 2 + 1) % 3;
      faceDims[1] = (faceIdx / 2 + 2) % 3;

      return faceDims;
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

   // use the basis vectors to interpolate an interior point
   public Point3d interp(double[] xi) {

      return NodeInterpolator.interp(
         xi, myInterpType, getNodes(), getNodeVersions(), scaleFactors,
         NodeInterpolator.ALL8NODES, NodeInterpolator.ORIGINAL_DIMENSIONS);

   }

   // use the basis vectors to interpolate given arclength
   public Point3d interp(double[] arc, double xi[]) {
      return NodeInterpolator.interpArc(
         arc, myInterpType, getNodes(), getNodeVersions(), scaleFactors,
         NodeInterpolator.ALL8NODES, NodeInterpolator.ORIGINAL_DIMENSIONS, xi);

   }

   public Matrix3d
      computeAdjustedJ0(double[] psi, double eps, Point3d outputPnt) {
      Matrix3d J0 = new Matrix3d();

      Vector3d[] HighPnt = new Vector3d[3];
      Vector3d[] LowPnt = new Vector3d[3];
      Vector3d[] dPsi = new Vector3d[3];
      double[] xi = new double[3];
      double[] psiOrig = Arrays.copyOf(psi, 3);
      int[] xiOrder = new int[3];
      boolean[] xiFlip = new boolean[3];

      double oldAcc = NodeInterpolator.defaultThreshold;
      NodeInterpolator.setDefaultThreshold(eps / 10);

      Shape myShape = getShape();
      switch (myShape) {
         case HEX:
            // adjust scaling of input to match rest cube (biunit cube centered
            // at origin)
            for (int i = 0; i < 3; i++) {
               xi[i] = (psi[i] + 1) / 2.0;
            }
            J0 = computeJ0(xi, eps);
            J0.scale(0.5);
            break;
         case WEDGE:
            // we have to find the triangular face, orient dimensions
            int triIDx = -1;
            for (int i = 0; i < myFaces.length; i = i + 2) {
               if (myFaces[i] != null) {
                  if (myFaces[i].getShape() == ExFace.Shape.TRIANGLE) {
                     triIDx = i;
                     break;
                  }
               }
            }

            int collapsedLine = 0;
            ExLine[] faceLines = myFaces[triIDx].getLines();
            for (int i = 0; i < faceLines.length; i++) {
               if (faceLines[i] == null) {
                  collapsedLine = i;
                  break;
               } else if (faceLines[i].isCollapsed()) {
                  collapsedLine = i;
                  break;
               }
            }

            xiOrder[2] = triIDx / 2; // indicates which is cartesian [-1,1]
                                     // (psi[3])
            xiOrder[1] = (xiOrder[2] + (1 - collapsedLine / 2) + 1) % 3; // direction
                                                                         // that
                                                                         // is
                                                                         // adjusted
                                                                         // for
                                                                         // triangle
                                                                         // (psi[2])
            xiOrder[0] = (xiOrder[2] + (collapsedLine / 2) + 1) % 3; // direction
                                                                     // for
                                                                     // cartesian
                                                                     // [0 1]

            xiFlip[0] = (collapsedLine % 2 == 0); // flip if initial dimension
                                                  // is collapsed
            xiFlip[1] = false;
            xiFlip[2] = false;

            boolean s1Corner = false; // corner where s=1 must adjust for d/dr
            boolean r1Corner = false; // corder where r=1 must adjust for d/ds

            for (int i = 0; i < 3; i++) {
               double sep = eps;
               psi[0] = psiOrig[0];
               psi[1] = psiOrig[1];
               psi[2] = psiOrig[2]; // reset

               psi[i] = psiOrig[i] + eps / 2;
               if (psi[0] + psi[1] > 1) {
                  psi[i] = psiOrig[i];
                  sep = sep - eps / 2;
               } else if (psi[2] > 1) {
                  psi[2] = 1;
                  sep = sep - eps / 2;
               }
               xi = psiToXi(psi, xiOrder, xiFlip, myShape);
               HighPnt[i] = interp(xi);

               psi[i] = psiOrig[i] - eps / 2;
               if (psi[0] < 0 || psi[1] < 0) {
                  if ((i == 0) && (psiOrig[1] == 1)) {
                     s1Corner = true;
                     psi[0] = eps / 2;
                     psi[1] = 1 - eps / 2;
                     sep = eps / Math.sqrt(2);
                  } else if ((i == 1) && (psiOrig[0] == 1)) {
                     r1Corner = true;
                     psi[0] = 1 - eps / 2;
                     psi[1] = eps / 2;
                     sep = eps / Math.sqrt(2);
                  } else {
                     psi[i] = 0;
                     sep = sep - eps / 2;
                  }
               } else if (psi[2] < -1) {
                  psi[2] = -1;
                  sep = sep - eps / 2;
               }
               xi = psiToXi(psi, xiOrder, xiFlip, myShape);
               LowPnt[i] = interp(xi);
               psi[i] = psiOrig[i];

               dPsi[i] = new Vector3d(HighPnt[i]);
               dPsi[i].sub(LowPnt[i]);
               dPsi[i].scale(1.0 / sep);
            }

            // adjust for angled derivative
            if (r1Corner) {
               dPsi[1].scale(-Math.sqrt(2));
               dPsi[1].add(dPsi[0]);
            } else if (s1Corner) {
               dPsi[0].scale(-Math.sqrt(2));
               dPsi[0].add(dPsi[1]);
            }

            for (int i = 0; i < 3; i++) {
               J0.setColumn(i, dPsi[i]);
            }

            break;
         case PYRAMID:
            System.out.println("Warning, adjusted J0 not implemented.");
            return null;
         case TET:
            System.out.println("Warning, adjusted J0 not implemented.");
            return null;
      }

      // adjust for orientation
      ArrayList<ExNode> elemNodes = new ArrayList<ExNode>();
      getNodes(elemNodes);
      int offset = 0;
      if (elemNodes.size() == 5 || elemNodes.size() == 8) {
         offset = 1;
      }
      Vector3d origin = new Vector3d(elemNodes.get(0).getCoordinate()); // "origin"
                                                                        // for
                                                                        // element
      Vector3d v1 = new Vector3d(elemNodes.get(1).getCoordinate());
      v1.sub(origin);
      Vector3d v2 = new Vector3d(elemNodes.get(2 + offset).getCoordinate());
      v2.sub(origin);
      v1.cross(v2); // v1 = v1 x v2
      Vector3d v3 = new Vector3d(elemNodes.get(3 + offset).getCoordinate());
      v3.sub(origin);
      if (v1.dot(v3) < 0) {
         J0.scale(-1.0);
      }

      xi = psiToXi(psiOrig, xiOrder, xiFlip, getShape());
      outputPnt.set(interp(xi));

      NodeInterpolator.setDefaultThreshold(oldAcc);

      return J0;
   }

   // shape dependent
   public Matrix3d computeAdjustedJ0(double[] psi, double eps) {
      Point3d outpnt = new Point3d();
      return computeAdjustedJ0(psi, eps, outpnt);
   }

   private static double[] psiToXi(double[] psi, int[] xiOrder,
      boolean[] xiFlip, Shape shape) {
      double[] xi = new double[3];

      switch (shape) {
         case HEX:
            for (int i = 0; i < 3; i++) {
               xi[xiOrder[i]] = (psi[i] + 1) / 2.0;
            }
            break;
         case WEDGE:
            xi[xiOrder[0]] = psi[0];
            if (psi[0] >= 1.0 - 1e-5) {
               xi[xiOrder[1]] = 0;
            } else {
               xi[xiOrder[1]] = psi[0] * psi[1] / (1 - psi[0]) + psi[1];
            }
            xi[xiOrder[2]] = (psi[2] + 1) / 2.0;
            break;
         case PYRAMID:
            break;
         case TET:
      }

      for (int i = 0; i < 3; i++) {
         if (xiFlip[i]) {
            xi[xiOrder[i]] = 1 - xi[xiOrder[i]];
         }
      }

      return xi;
   }

   // uses basis function coordinates, independent of shape of actual element
   public Matrix3d computeJ0(double[] xi, double eps) {
      Matrix3d J0 = new Matrix3d();
      J0.setZero();

      double oldAcc = NodeInterpolator.defaultThreshold;
      NodeInterpolator.setDefaultThreshold(eps / 10);

      Vector3d pnt = interp(xi); // center point

      double[] xiLow = new double[3];
      double[] xiHigh = new double[3];
      double[] xiOrig = Arrays.copyOf(xi, 3);
      Vector3d[] dPsi = new Vector3d[3];
      Vector3d tmp = new Vector3d();

      for (int i = 0; i < 3; i++) {

         xiLow[i] = Math.max(xi[i] - eps, 0);
         xiHigh[i] = Math.min(xi[i] + eps, 1);

         xi[i] = xiHigh[i];
         dPsi[i] = interp(xi);
         xi[i] = xiLow[i];
         tmp = interp(xi);
         xi[i] = xiOrig[i];

         dPsi[i].sub(tmp);
         dPsi[i].scale(1.0 / (xiHigh[i] - xiLow[i]));

         J0.setColumn(i, dPsi[i]);
      }

      NodeInterpolator.setDefaultThreshold(oldAcc);

      return J0;
   }

   public double[][] getScaleFactors() {
      return scaleFactors;
   }

   public boolean isOnSurface() {

      for (ExFace face : myFaces) {
         if (face != null) {
            if (face.isOnSurface()) {
               return true;
            }
         }
      }

      return false;

   }
   
   public boolean isFlipped() {
      if (reverseComputed) {
         return reversed;
      }
      computeVolume(); // if volume is negative, reversed = true
      return reversed;
   }
   
   public double computeVolume() {
      
      double vol = computeVolume(getNodes());
      if (vol < 0) {
         reversed = true;
         reverseComputed = true;
      }
      return vol;
      
   }
   
   public static double computeVolume(ExNode[] nodes) {
      Point3d[] pt = new Point3d[8];
      for (int i=0; i<pt.length; i++) {
         pt[i] = new Point3d(nodes[i].getCoordinate());
      }
      return computeVolume(pt);
      
   }

   // Efficient Computation of Volume of Hexahedral Cells, J.Grandy 1997
   // requires 8 points
   public static double computeVolume(Point3d[] pt) {

      double x0 = pt[0].x, y0 = pt[0].y, z0 = pt[0].z;
      double x1 = pt[1].x, y1 = pt[1].y, z1 = pt[1].z;
      double x2 = pt[2].x, y2 = pt[2].y, z2 = pt[2].z;
      double x3 = pt[3].x, y3 = pt[3].y, z3 = pt[3].z;
      double x4 = pt[4].x, y4 = pt[4].y, z4 = pt[4].z;
      double x5 = pt[5].x, y5 = pt[5].y, z5 = pt[5].z;
      double x6 = pt[6].x, y6 = pt[6].y, z6 = pt[6].z;
      double x7 = pt[7].x, y7 = pt[7].y, z7 = pt[7].z;

      double v0 = ((x2 + x3 - x4 - x5) * y1 + (x4 + x6 - x3 - x1) * y2
         + (-x1 + x2) * y3 + (-x2 - x6 + x1 + x5) * y4 + (x1 - x4) * y5
         + (-x2 + x4) * y6) * z0;
      double v1 = ((-x2 + x5 - x3 + x4) * y0 + (x0 - x3) * y2 
         + (x0 - x7 + x2 - x5) * y3 + (x5 - x0) * y4 
         + (x7 - x4 + x3 - x0) * y5 + (-x5 + x3) * y7) * z1;
      double v2 = ((x3 - x4 - x6 + x1) * y0 + (x3 - x0) * y1
         + (x7 - x1 + x6 - x0) * y3 + (-x6 + x0) * y4 
         + (-x7 + x0 - x3 + x4) * y6 + (x6 - x3) * y7) * z2;
      double v3 = ((x1 - x2) * y0 + (x7 - x2 + x5 - x0) * y1 
         + (-x7 + x0 - x6 + x1) * y2 + (x7 - x1) * y5 
         + (-x7 + x2) * y6 + (-x5 + x2 - x1 + x6) * y7) * z3;
      double v4 = ((-x5 + x2 - x1 + x6) * y0 + (x0 - x5) * y1 
         + (x6 - x0) * y2 + (-x7 + x0 - x6 + x1) * y5 
         + (x7 - x2 + x5 - x0) * y6 + (-x6 + x5) * y7) * z4;
      double v5 = ((x4 - x1) * y0 + (-x7 + x0 - x3 + x4) * y1 
         + (-x7 + x1) * y3 + (x7 - x1 + x6 - x0) * y4 + (x7 - x4) * y6 
         + (x3 - x4 - x6 + x1) * y7) * z5;
      double v6 = ((-x4 + x2) * y0 + (x7 - x4 + x3 - x0) * y2 + (x7 - x2) * y3
            + (x0 - x7 + x2 - x5) * y4 + (-x7 + x4) * y5 + (-x2 + x5 - x3 + x4)
            * y7) * z6;
      double v7 = ((x5 - x3) * y1 + (x3 - x6) * y2 + (-x2 - x6 + x1 + x5) * y3
            + (x6 - x5) * y4 + (x4 + x6 - x3 - x1) * y5 + (x2 + x3 - x5 - x4)
            * y6) * z7;

      double v = (v0 + v1 + v2 + v3 + v4 + v5 + v6 + v7) / 12;

      return v;
   }

}
