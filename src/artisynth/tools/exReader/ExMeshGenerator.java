package artisynth.tools.exReader;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;

public class ExMeshGenerator {

   public static boolean useArclength = false;
   public static boolean useLinearInternally = false;
   public static boolean flipZTangents = false;

   // internal classes
   private static class int3 {
      int x;
      int y;
      int z;

      public int3 (int xval, int yval, int zval) {
         x = xval;
         y = yval;
         z = zval;
      }

      public void set(int i, int val) {
         if (i == 0) {
            x = val;
         }
         else if (i == 1) {
            y = val;
         }
         else if (i == 2) {
            z = val;
         }
      }

      public int get(int i) {
         if (i == 0) {
            return x;
         }
         else if (i == 1) {
            return y;
         }
         else if (i == 2) {
            return z;
         }
         return 0;
      }

      public int size() {
         return 3;
      }

      public String toString() {
         return x + " " + y + " " + z;
      }
   }

   private static class FaceNodes {
      int nNodes1;
      int nNodes2;
      FemNode3d[][] myNodes;
   }

   private static class LineNodes {
      int nNodes;
      FemNode3d[] myNodes;
   }

   public static FemModel3d generateLinearFEM(ArrayList<ExElement> elements,
      int[] split, double density) throws Exception {
      return generateLinearFEM(null, elements, split, density);
   }
   
   public static FemModel3d generateLinearFEM( FemModel3d model,
      ArrayList<ExElement> elements,
      int[] split, double density) throws Exception {

      if (model == null) {
         model = new FemModel3d();
      }

      HashMap<ExElement,int3> splitNumberMap = new HashMap<ExElement,int3>();
      HashMap<ExFace,FaceNodes> faceNodeMap = new HashMap<ExFace,FaceNodes>();
      HashMap<ExLine,LineNodes> lineNodeMap = new HashMap<ExLine,LineNodes>();
      HashMap<ExNode,FemNode3d> nodeNodeMap = new HashMap<ExNode,FemNode3d>();

      model.setDensity(density);

      // determine how to split each element
      for (ExElement elem : elements) {
         if (!splitNumberMap.containsKey(elem)) {
            getSplitInfo(elem, split, splitNumberMap, true, false);
         }
      }

      // generate all elements
      for (ExElement elem : elements) {
         createSplitElement(
            elem, model, splitNumberMap, faceNodeMap, lineNodeMap, nodeNodeMap);
      }

      return model;
   }

   public static FemModel3d generateLinearFEM(FemModel3d model, File exNodeFile, File exElemFile,
      int[] resolution, double density) throws IOException {
      
      ExParser parser = new ExParser();
      parser.parseExNode(exNodeFile);
      parser.parseExElem(exElemFile);

      // get last region
      ExRegion myRegion = parser.getLastParsedRegion();
      if (flipZTangents) {
         TangentFlipper flipper = new TangentFlipper();
         flipper.flipAllTangents(myRegion.getElements(), 2);
      }
      
      String name = myRegion.getName().replace("/", " ").trim();
      
      try {
         model =
            ExMeshGenerator.generateLinearFEM(model, myRegion.getElements(), resolution, density);
         model.setName(name);
      } catch (Exception e) {
         throw new IllegalStateException("Error generating mesh");
      }

      return model;
      
   }
   
   // populates FemModel3d
   public static FemModel3d generateLinearFEM(FemModel3d model, ArrayList<ExElement> elements,
      double density) {

      if (model == null) {
         model = new FemModel3d();
      }
      model.clear();
      model.setDensity(density);

      HashMap<ExNode,Integer> nodeIndexMap = new HashMap<ExNode,Integer>();
      HashMap<ExElement,Integer> elemIndexMap =
         new HashMap<ExElement,Integer>();

      int elemIdx = 0;
      int nodeIdx = 0;

      // creates a 0-based index map for all elements and nodes
      // and adds all nodes to the model
      for (ExElement elem : elements) {
         elemIndexMap.put(elem, elemIdx++);
         for (ExNode node : elem.getNodes()) {
            if (!nodeIndexMap.containsKey(node)) {
               nodeIndexMap.put(node, nodeIdx);

               Point3d nodePnt = new Point3d(node.getCoordinate());
               // fill coordinate
               FemNode3d femNode = new FemNode3d(nodePnt);
               model.addNumberedNode(femNode, nodeIdx);

               // increment counter
               nodeIdx++;
            }
         }
      }

      // add actual elements to the model
      ArrayList<ExNode> elemNodes = new ArrayList<ExNode>();
      ArrayList<Integer> nodeIdxs = new ArrayList<Integer>();

      for (ExElement elem : elements) {
         elemNodes.clear();
         nodeIdxs.clear();

         ExElement.Shape elemShape = elem.getNodes(elemNodes);
         for (ExNode node : elemNodes) {
            nodeIdxs.add(nodeIndexMap.get(node)); // build node index list
         }

         switch (elemShape) {
            case HEX:
               createHex(model, nodeIdxs, elemIndexMap.get(elem));
               break;
            case WEDGE:
               createWedge(model, nodeIdxs, elemIndexMap.get(elem));
               break;
            case PYRAMID:
               createPyramid(model, nodeIdxs, elemIndexMap.get(elem));
               break;
            case TET:
               createTet(model, nodeIdxs, elemIndexMap.get(elem));
               break;
            default:
               System.err.println("Error: unknown shape '" + elemShape + "'");
         }

      } // end looping through elements

      return model;

   }

   private static TetElement createTet(ArrayList<FemNode3d> nodes,
      FemModel3d model) {
      return createTet(nodes, -1, model);
   }

   private static TetElement createTet(FemModel3d model,
      ArrayList<Integer> nodeIds, int elemId) {

      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
      for (int i = 0; i < nodeIds.size(); i++) {
         nodes.add(model.getByNumber(nodeIds.get(i)));
      }
      return createTet(nodes, elemId, model);
   }

   private static TetElement createTet(ArrayList<FemNode3d> nodes, int elemId,
      FemModel3d model) {

      TetElement e =
         new TetElement(
            nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3));

      if (e.computeVolumes() < 0) {
         System.out
            .println("Warning: tet as defined has negative volume, mirroring nodes... ("
               + elemId + ")");
         mirrorNodeOrder(nodes, ExElement.Shape.TET);
         e =
            new TetElement(
               nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3));
         System.out.println("found ccw tet");
      }

      if (elemId < 0) {
         model.addElement(e);
      }
      else {
         model.addNumberedElement(e, elemId);
      }

      return e;
   }

   private static HexElement createHex(ArrayList<FemNode3d> nodes,
      FemModel3d model) {
      return createHex(nodes, -1, model);
   }

   private static HexElement createHex(FemModel3d model,
      ArrayList<Integer> nodeIds, int elemId) {

      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
      for (int i = 0; i < nodeIds.size(); i++) {
         nodes.add(model.getByNumber(nodeIds.get(i)));
      }
      return createHex(nodes, elemId, model);
   }

   private static HexElement createHex(ArrayList<FemNode3d> nodes, int elemId,
      FemModel3d model) {

      // Artisynth wants hexes in CCW order for some reason
      mirrorNodeOrder(nodes, ExElement.Shape.HEX);

      HexElement e =
         new HexElement(
            nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3),
            nodes.get(4), nodes.get(5), nodes.get(6), nodes.get(7));
      double detJ = e.computeVolumes();

      // if negative volume, switch order
      if (detJ < 0) {
         System.out
            .println("Warning: hex as defined has negative volume, mirroring nodes... ("
               + elemId + ")");
         mirrorNodeOrder(nodes, ExElement.Shape.HEX);
         e =
            new HexElement(
               nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3),
               nodes.get(4), nodes.get(5), nodes.get(6), nodes.get(7));
         if (e.computeVolumes() < 0) {
            System.out.println("found neg volume hex, # " + e.getNumber());
         }
      }
      if (elemId < 0) {
         model.addElement(e);
      }
      else {
         model.addNumberedElement(e, elemId);
      }

      return e;
   }

   private static PyramidElement createPyramid(ArrayList<FemNode3d> nodes,
      FemModel3d model) {
      return createPyramid(nodes, -1, model);
   }

   private static PyramidElement createPyramid(FemModel3d model,
      ArrayList<Integer> nodeIds, int elemId) {
      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
      for (int i = 0; i < nodeIds.size(); i++) {
         nodes.add(model.getByNumber(nodeIds.get(i)));
      }
      return createPyramid(nodes, elemId, model);
   }

   private static PyramidElement createPyramid(ArrayList<FemNode3d> nodes,
      int elemId, FemModel3d model) {

      PyramidElement e =
         new PyramidElement(
            nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3),
            nodes.get(4));

      if (e.computeVolumes() < 0) {
         System.out
            .println("Warning: pyramid as defined has negative volume, mirroring nodes...");
         mirrorNodeOrder(nodes, ExElement.Shape.PYRAMID);
         e =
            new PyramidElement(
               nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3),
               nodes.get(4));
         if (e.computeVolumes() < 0) {
            System.out.println("found inverted pyramid, # " + e.getNumber());
         }
      }

      if (elemId < 0) {
         model.addElement(e);
      }
      else {
         model.addNumberedElement(e, elemId);
      }
      return e;
   }

   private static WedgeElement createWedge(ArrayList<FemNode3d> nodes,
      FemModel3d model) {
      return createWedge(nodes, -1, model);
   }

   private static WedgeElement createWedge(FemModel3d model,
      ArrayList<Integer> nodeIds, int elemId) {
      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
      for (int i = 0; i < nodeIds.size(); i++) {
         nodes.add(model.getByNumber(nodeIds.get(i)));
      }
      return createWedge(nodes, elemId, model);
   }

   private static WedgeElement createWedge(ArrayList<FemNode3d> nodes,
      int elemId, FemModel3d model) {

      WedgeElement e =
         new WedgeElement(
            nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3),
            nodes.get(4), nodes.get(5));
      if (e.computeVolumes() < 0) {
         mirrorNodeOrder(nodes, ExElement.Shape.WEDGE);
         e =
            new WedgeElement(
               nodes.get(0), nodes.get(1), nodes.get(2), nodes.get(3),
               nodes.get(4), nodes.get(5));
         if (e.computeVolumes() < 0) {
            System.out.println("found inverted wedge, # " + e.getNumber());
         }
         System.out
            .println("Warning: wedge as defined has negative volume, mirroring nodes... ("
               + elemId + ")");
         System.out.println("Computed wedge volume: " + e.getVolume());

         String nodeStr = "nodeIDs = [";
         String nodeCoords = "nodes = [\n";
         for (int i = 0; i < nodes.size(); i++) {
            nodeStr += nodes.get(i).myNumber + " ";
            nodeCoords += nodes.get(i).getPosition().toString() + "\n";
         }
         nodeStr += "];\n";
         nodeCoords += "];\n";
         // System.out.print (nodeStr);
         // System.out.print (nodeCoords);

      }

      if (elemId < 0) {
         model.addElement(e);
      }
      else {
         model.addNumberedElement(e, elemId);
      }
      return e;
   }

   // create splitNum nodes along an ExLine
   public static void splitLine(ExLine line, FemModel3d model, ExElement elem,
      int splitNum, HashMap<ExLine,LineNodes> lineNodeMap,
      HashMap<ExNode,FemNode3d> nodeNodeMap) {

      if (lineNodeMap.containsKey(line)) {
         return;
      }

      // determine interpolation type for this line
      int lineDim = elem.getLineDimension(line);

      NodeInterpolator.InterpType[] interpType =
         new NodeInterpolator.InterpType[1];
      interpType[0] = elem.getInterpType()[lineDim];

      // if line is interior, use linear
      if (useLinearInternally && !line.isOnSurface()) {
         interpType[0] = NodeInterpolator.InterpType.LINEAR;
      }

      FemNode3d[] nodes = new FemNode3d[splitNum + 1];
      int lineIdx = elem.getLineIdx(line);
      double xi[] = { 0 };
      int dim[] = { 0 };
      dim[0] = elem.getLineDimension(line);

      // interior nodes
      for (int i = 1; i < splitNum; i++) {
         xi[0] = ((double)i) / splitNum;

         FemNode3d newNode;
         if (useArclength) {
            newNode =
               new FemNode3d(
                  NodeInterpolator.interpArc(
                     xi, interpType, elem.getNodes(), elem.getNodeVersions(),
                     elem.getScaleFactors(), ExElement.lineNodeIdxs[lineIdx],
                     dim));
         }
         else {
            newNode =
               new FemNode3d(
                  NodeInterpolator.interp(
                     xi, interpType, elem.getNodes(), elem.getNodeVersions(),
                     elem.getScaleFactors(), ExElement.lineNodeIdxs[lineIdx],
                     dim));
         }
         nodes[i] = newNode;
         model.addNode(newNode);
      }

      // fill in front/back
      ExNode[] lineNodes = line.getNodes();
      for (int i = 0; i < 2; i++) {

         // check if node already created, if not create it
         FemNode3d femNode = nodeNodeMap.get(lineNodes[i]);
         if (femNode == null) {
            femNode =
               new FemNode3d(new Point3d(lineNodes[i].getCoordinate()));
            model.addNode(femNode);
            nodeNodeMap.put(lineNodes[i], femNode);
         }
         nodes[i * splitNum] = femNode; // set node in array
      }

      // store array of nodes in hashmap
      LineNodes nodeArray = new LineNodes();
      nodeArray.myNodes = nodes;
      nodeArray.nNodes = splitNum + 1;
      lineNodeMap.put(line, nodeArray);

   }

   public static void splitFace(ExFace face, FemModel3d model, ExElement elem,
      int3 splitNums, HashMap<ExFace,FaceNodes> faceNodeMap,
      HashMap<ExLine,LineNodes> lineNodeMap,
      HashMap<ExNode,FemNode3d> nodeNodeMap) {

      if (faceNodeMap.containsKey(face)) {
         return;
      }

      NodeInterpolator.InterpType[] interpType =
         new NodeInterpolator.InterpType[2];
      int[] faceDims = elem.getFaceDimensions(face);
      for (int i = 0; i < 2; i++) {
         interpType[i] = elem.getInterpType()[faceDims[i]];
      }

      // if face is interior, use linear
      if (useLinearInternally && !face.isOnSurface()) {
         for (int i = 0; i < 2; i++) {
            interpType[i] = NodeInterpolator.InterpType.LINEAR;
         }
      }

      int faceIdx = elem.getFaceIdx(face);
      int nSplitX = (int)splitNums.get(faceDims[0]);
      int nSplitY = (int)splitNums.get(faceDims[1]);

      FemNode3d[][] nodes = new FemNode3d[nSplitX + 1][nSplitY + 1];
      double xi[] = { 0, 0 };

      // interior nodes
      for (int i = 1; i < nSplitX; i++) {
         xi[0] = ((double)i) / nSplitX;
         for (int j = 1; j < nSplitY; j++) {
            xi[1] = ((double)j) / nSplitY;
            FemNode3d newNode;
            if (useArclength) {
               newNode =
                  new FemNode3d(NodeInterpolator.interpArc(
                     xi, interpType, elem.getNodes(), elem.getNodeVersions(),
                     elem.getScaleFactors(), ExElement.faceNodeIdxs[faceIdx],
                     faceDims));
            }
            else {
               newNode =
                  new FemNode3d(NodeInterpolator.interp(
                     xi, interpType, elem.getNodes(), elem.getNodeVersions(),
                     elem.getScaleFactors(), ExElement.faceNodeIdxs[faceIdx],
                     faceDims));
            }
            nodes[i][j] = newNode;
            model.addNode(newNode);
         }
      }

      // generate nodes on lines
      ExLine[] faceLines = face.getLines();
      for (int i = 0; i < faceLines.length; i++) {

         // if line isn't null, generate nodes
         if (faceLines[i] != null) {
            int nSplit = nSplitY;
            if (i > 1) {
               nSplit = nSplitX;
            }

            // check if nodes need to be created
            if (!lineNodeMap.containsKey(faceLines[i])) {
               // create nodes on line
               splitLine(
                  faceLines[i], model, elem, nSplit, lineNodeMap, nodeNodeMap);
            }

            // copy nodes into this face
            FemNode3d[] colNodes = lineNodeMap.get(faceLines[i]).myNodes;

            // along dim1
            if (i > 1) {
               int col = (i % 2) * nSplitY;
               for (int j = 0; j < nSplitX + 1; j++) {
                  nodes[j][col] = colNodes[j];
               }
            }
            else {
               // along dim2
               int row = (i % 2) * nSplitX;
               for (int j = 0; j < nSplitY + 1; j++) {
                  nodes[row][j] = colNodes[j];
               }
            } // end checking which dimension to copy line
         } // end checking if line is not null
         else {

            // duplicate node along entire line

            // if i=0, then nodes are

            // find node to be repeated
            ExNode dupNode = face.getNodes()[ExFace.lineNodeIdxs[i][0]];
            FemNode3d edgeNode = nodeNodeMap.get(dupNode);

            // if not already created, create and add to fem model
            if (edgeNode == null) {
               edgeNode =
                  new FemNode3d(new Point3d(dupNode.getCoordinate()));
               model.addNode(edgeNode);
               nodeNodeMap.put(dupNode, edgeNode);
            }

            // along dim1
            if (i > 1) {
               int col = (i % 2) * nSplitY;
               for (int j = 0; j < nSplitX + 1; j++) {
                  nodes[j][col] = edgeNode;
               }
            }
            else {
               // along dim2
               int row = (i % 2) * nSplitX;
               for (int j = 0; j < nSplitY + 1; j++) {
                  nodes[row][j] = edgeNode;
               }
            } // end checking which dimension to copy line
         } // end filling in line
      } // end looping through lines

      FaceNodes nodeArray = new FaceNodes();
      nodeArray.myNodes = nodes;
      nodeArray.nNodes1 = nSplitX + 1;
      nodeArray.nNodes2 = nSplitY + 1;
      faceNodeMap.put(face, nodeArray);

   }

   public static void createSplitElement(ExElement elem, FemModel3d model,
      HashMap<ExElement,int3> splitDimMap,
      HashMap<ExFace,FaceNodes> faceNodeMap,
      HashMap<ExLine,LineNodes> lineNodeMap,
      HashMap<ExNode,FemNode3d> nodeNodeMap) {

      int3 split = splitDimMap.get(elem);
      if (split == null) {
         System.out.println("Don't know how to split this element yet");
         return;
      }

      NodeInterpolator.InterpType[] interpType = elem.getInterpType();

      // if element is interior, use linear
      if (useLinearInternally) {
         for (int i = 0; i < 3; i++) {
            interpType[i] = NodeInterpolator.InterpType.LINEAR;
         }

      }

      FemNode3d[][][] nodes =
         new FemNode3d[(int)split.x + 1][(int)split.y + 1][(int)split.z + 1];

      double[] xi = new double[3];
      int[] dir = NodeInterpolator.ORIGINAL_DIMENSIONS; // assume no reordering
                                                        // of dimensions

      // generate all interior nodes
      for (int i = 1; i < split.x; i++) {
         xi[0] = (double)i / split.x;
         for (int j = 1; j < split.y; j++) {
            xi[1] = (double)j / split.y;
            for (int k = 1; k < split.z; k++) {
               xi[2] = (double)k / split.z;
               FemNode3d newNode;
               if (useArclength) {
                  newNode =
                     new FemNode3d(NodeInterpolator.interpArc(
                        xi, interpType, elem.getNodes(),
                        elem.getNodeVersions(),
                        elem.getScaleFactors(), NodeInterpolator.ALL8NODES,
                        dir));
               }
               else {
                  newNode =
                     new FemNode3d(NodeInterpolator.interp(
                        xi, interpType, elem.getNodes(),
                        elem.getNodeVersions(),
                        elem.getScaleFactors(), NodeInterpolator.ALL8NODES,
                        dir));
               }
               nodes[i][j][k] = newNode;
               model.addNode(newNode);
            }
         }
      }

      // generate nodes on faces, (TODO: should depend on orientations)
      ExFace[] elemFaces = elem.getFaces();
      for (int i = 0; i < elemFaces.length; i++) {
         // if face is null, go to next one
         if (elemFaces[i] != null) {
            if (!faceNodeMap.containsKey(elemFaces[i])) {
               splitFace(
                  elemFaces[i], model, elem, splitDimMap.get(elem),
                  faceNodeMap, lineNodeMap, nodeNodeMap);
            }

            FemNode3d[][] faceNodes = faceNodeMap.get(elemFaces[i]).myNodes;
            int fixedIdx = (int)((i % 2) * split.get(i / 2));
            // fill in slot
            if (i < 2) {
               // front/back
               for (int j = 0; j < split.y + 1; j++) {
                  for (int k = 0; k < split.z + 1; k++) {
                     nodes[fixedIdx][j][k] = faceNodes[j][k];
                  }
               }

            }
            else if (i < 4) {
               // side/side
               for (int j = 0; j < split.x + 1; j++) {
                  for (int k = 0; k < split.z + 1; k++) {
                     nodes[j][fixedIdx][k] = faceNodes[k][j];
                  }
               }
            }
            else {
               // top/bottom
               for (int j = 0; j < split.x + 1; j++) {
                  for (int k = 0; k < split.y + 1; k++) {
                     nodes[j][k][fixedIdx] = faceNodes[j][k];
                  }
               }
            } // end filling in face
         } // end checking full face
         else {
            // fill in collapsed face
            // XXX
            // determine if we have a point (pyramid/tet) or a line (wedge)
            ArrayList<ExNode> faceNodes = new ArrayList<ExNode>();
            ArrayList<Integer> faceNodeVersions = new ArrayList<Integer>();
            ExNode[] elemNodes = elem.getNodes();
            int[] elemNodeVersions = elem.getNodeVersions();

            int[] faceNodeIdxs = ExElement.faceNodeIdxs[i];

            // gather all nodes corresponding to face
            for (int j = 0; j < faceNodeIdxs.length; j++) {
               if (!faceNodes.contains(elemNodes[faceNodeIdxs[j]])) {
                  faceNodes.add(elemNodes[faceNodeIdxs[j]]);
                  faceNodeVersions.add(elemNodeVersions[faceNodeIdxs[j]]);
               }
            }

            // check if collapsed to node
            if (faceNodes.size() == 1) {
               // fill entire face with single node

               // get node
               FemNode3d collapsedNode = nodeNodeMap.get(faceNodes.get(0));
               if (collapsedNode == null) {
                  collapsedNode =
                     new FemNode3d(new Point3d(faceNodes
                        .get(0).getCoordinate()));
                  model.addNode(collapsedNode);
                  nodeNodeMap.put(faceNodes.get(0), collapsedNode);
               }

               int fixedIdx = (int)((i % 2) * split.get(i / 2));
               // fill in slot
               if (i < 2) {
                  // front/back
                  for (int j = 0; j < split.y + 1; j++) {
                     for (int k = 0; k < split.z + 1; k++) {
                        nodes[fixedIdx][j][k] = collapsedNode;
                     }
                  }

               }
               else if (i < 4) {
                  // side/side
                  for (int j = 0; j < split.x + 1; j++) {
                     for (int k = 0; k < split.z + 1; k++) {
                        nodes[j][fixedIdx][k] = collapsedNode;
                     }
                  }
               }
               else {
                  // top/bottom
                  for (int j = 0; j < split.x + 1; j++) {
                     for (int k = 0; k < split.y + 1; k++) {
                        nodes[j][k][fixedIdx] = collapsedNode;
                     }
                  }
               } // end filling in face

            }
            else {

               // find line, fill face with line
               ArrayList<ExLine> lineList =
                  faceNodes.get(0).getDependentLines();
               ExLine collapsedLine = null;
               for (ExLine line : lineList) {
                  if (line.connects(
                     faceNodes.get(0), faceNodeVersions.get(0),
                     faceNodes.get(1), faceNodeVersions.get(1))) {
                     collapsedLine = line;
                     break;
                  }
               }

               // check what dimension line goes along
               int lineIdx = elem.getLineIdx(collapsedLine);
               int dim = 0;
               if (lineIdx > 3 && lineIdx < 8) {
                  dim = 2;
               }
               else if ((lineIdx < 2) || (lineIdx == 8) || (lineIdx == 9)) {
                  dim = 1;
               }

               // check if line has nodes generated
               LineNodes lineNodes = lineNodeMap.get(collapsedLine);
               if (lineNodes == null) {
                  // generate nodes on line
                  splitLine(
                     collapsedLine, model, elem, (int)split.get(dim),
                     lineNodeMap, nodeNodeMap);
                  lineNodes = lineNodeMap.get(collapsedLine);
               }
               FemNode3d[] lineNodeArray = lineNodes.myNodes;

               // fill in face
               int fixedIdx = (int)((i % 2) * split.get(i / 2));
               // fill in slot
               if (i < 2) {
                  // front/back
                  for (int j = 0; j < split.y + 1; j++) {
                     for (int k = 0; k < split.z + 1; k++) {
                        if (dim == 2) {
                           nodes[fixedIdx][j][k] = lineNodeArray[j];
                        }
                        else {
                           nodes[fixedIdx][j][k] = lineNodeArray[k];
                        }
                     }
                  }

               }
               else if (i < 4) {
                  // side/side
                  for (int j = 0; j < split.x + 1; j++) {
                     for (int k = 0; k < split.z + 1; k++) {
                        if (dim == 1) {
                           nodes[j][fixedIdx][k] = lineNodeArray[j];
                        }
                        else {
                           nodes[j][fixedIdx][k] = lineNodeArray[k];
                        }
                     }
                  }
               }
               else {
                  // top/bottom
                  for (int j = 0; j < split.x + 1; j++) {
                     for (int k = 0; k < split.y + 1; k++) {
                        if (dim == 1) {
                           nodes[j][k][fixedIdx] = lineNodeArray[k];
                        }
                        else {
                           nodes[j][k][fixedIdx] = lineNodeArray[j];
                        }
                     }
                  }
               } // end filling in face
            }

         } // done filling in nodes from face
      } // done looping through faces

      FemNode3d[] elemNodes = new FemNode3d[8];
      // assemble elements
      for (int i = 0; i < split.x; i++) {
         for (int j = 0; j < split.y; j++) {
            for (int k = 0; k < split.z; k++) {
               elemNodes[0] = nodes[i][j][k];
               elemNodes[1] = nodes[i + 1][j][k];
               elemNodes[2] = nodes[i][j + 1][k];
               elemNodes[3] = nodes[i + 1][j + 1][k];
               elemNodes[4] = nodes[i][j][k + 1];
               elemNodes[5] = nodes[i + 1][j][k + 1];
               elemNodes[6] = nodes[i][j + 1][k + 1];
               elemNodes[7] = nodes[i + 1][j + 1][k + 1];
               createElement(model, elemNodes);
            }
         }
      }

      // // for now, simply print stuff
      // for (int i=0; i<nodes.length; i++) {
      // for (int j=0; j<nodes[i].length; j++) {
      // for (int k=0; k<nodes[i][j].length; k++) {
      // int idx = nodes[i][j][k].getNumber();
      // System.out.print(" "+idx);
      // }
      // System.out.print("\n");
      // }
      // System.out.print("\n");
      // }
      // System.out.println("node = zeros([" + nodes.length +
      // ", "+nodes[0].length+", " + nodes[0][0].length + ", 3]);");
      // for (int i=0; i<nodes.length; i++) {
      // for (int j=0; j<nodes[i].length; j++) {
      // for (int k=0; k<nodes[i][j].length; k++) {
      // System.out.print("node("+(i+1)+","+(j+1)+","+(k+1)+",:) = [");
      // System.out.print(nodes[i][j][k].getPosition().x + " " +
      // nodes[i][j][k].getPosition().y + " " + nodes[i][j][k].getPosition().z);
      // System.out.print("]';\n");
      // }
      // System.out.print("\n");
      // }
      // System.out.print("\n");
      // }

   }

   public static void createElement(FemModel3d model, FemNode3d[] nodes) {

      // count number of unique nodes to determine shape
      ArrayList<FemNode3d> unique = new ArrayList<FemNode3d>();
      for (int i = 0; i < nodes.length; i++) {
         if (!unique.contains(nodes[i])) {
            unique.add(nodes[i]);
         }
      }

      ExElement.Shape elemShape;
      switch (unique.size()) {
         case 8:
            elemShape = ExElement.Shape.HEX;
            break;
         case 6:
            elemShape = ExElement.Shape.WEDGE;
            break;
         case 5:
            elemShape = ExElement.Shape.PYRAMID;
            break;
         case 4:
            elemShape = ExElement.Shape.TET;
            break;
         default:
            System.err.println("Error: cannot create an element out of "
               + unique.size() + " nodes.");
            return;
      }
      try {
         unique = orderNodes(nodes, elemShape);
      } catch (Exception e) {
         System.err.println(e.getMessage());
      }
      createElement(model, unique);

   }

   public static void createElement(FemModel3d model,
      ArrayList<FemNode3d> nodes) {

      switch (nodes.size()) {
         case 8:
            createHex(nodes, model);
            break;
         case 6:
            createWedge(nodes, model);
            break;
         case 5:
            createPyramid(nodes, model);
            break;
         case 4:
            createTet(nodes, model);
            break;
         default:
            System.err.println("Error: cannot create an element out of "
               + nodes.size() + " nodes.");
      }
   }

   public static ArrayList<FemNode3d> orderNodes(FemNode3d[] nodes,
      ExElement.Shape shape) {

      ArrayList<FemNode3d> out = new ArrayList<FemNode3d>();

      // add first set of nodes
      int firstFaceIdx = 0;
      int nFaceNodes = 4; // quad
      if (shape == ExElement.Shape.WEDGE || shape == ExElement.Shape.TET) {
         nFaceNodes = 3; // triangular
      }

      // Flip last two nodes so they go around face in a loop
      final int[] LOOP = { 0, 1, 3, 2 };

      // Add first set of nodes, which correspond to quad face in hex/pyramid
      // or tri face in wedge/tet
      boolean foundFace = false;
      for (int i = 0; i < 6; i++) {
         out.clear();
         for (int j = 0; j < 4; j++) {
            FemNode3d nextNode = nodes[ExElement.faceNodeIdxs[i][LOOP[j]]];
            if (!out.contains(nextNode)) {
               out.add(nextNode);
            }
         }

         // check if we found the correct face
         if (out.size() == nFaceNodes) {
            firstFaceIdx = i;
            foundFace = true;
            break;
         }
      }

      if (foundFace == false) {
         System.out.println("huh?  The element seems to be mal-formed");
      }

      // add remaining nodes
      switch (shape) {
         case HEX:
         case WEDGE:
            // add nodes from opposite face
            for (int i = 0; i < 4; i++) {
               // nodes from opposite face
               FemNode3d nextNode =
                  nodes[ExElement.faceNodeIdxs[firstFaceIdx + 1][LOOP[i]]];
               if (!out.contains(nextNode)) {
                  out.add(nextNode);
               }
            }
            break;
         case PYRAMID:
         case TET:
            // add single missing node
            for (int i = 0; i < nodes.length; i++) {
               if (!out.contains(nodes[i])) {
                  out.add(nodes[i]);
                  break;
               }
            }
            break;
      }

      double vol = computeVolume(nodes);
      boolean mirrored = false;

      // check if CCW, if so reverse order
      if (firstFaceIdx % 2 == 1) {
         mirrored = true;
      }
      if (vol < 0) {
         mirrored = !mirrored;
      }

      if (mirrored) {
         mirrorNodeOrder(out, shape);
      }
      return out;
   }

   public static void mirrorNodeOrder(ArrayList<FemNode3d> nodeList,
      ExElement.Shape shape) {

      FemNode3d tmp = null; // used for shuffling nodes
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
         default:
            break;
      }
   }

   public void getSplitInfo(int[] split, ArrayList<ExElement> elemList,
      HashMap<ExFace,Integer> directedFaceMap,
      HashMap<ExElement,int3> splitDimMap) throws Exception {

      // if the following arrays can be filled without bloodshed,
      // it is possible to split the elements
      directedFaceMap.clear();
      splitDimMap.clear();

      // goes through all elements, performing the splitting
      for (ExElement elem : elemList) {
         if (!splitDimMap.containsKey(elem)) {
            getSplitInfo(elem, split, splitDimMap, true, false);
         }
      }
   }

   public static void orientTriFaces(ExElement elem,
      HashMap<ExFace,Integer> orientedFaceMap) throws Exception {

      ExElement.Shape elemShape = elem.getShape();

      // exit on hex (since has no triangular faces)
      if (elemShape == ExElement.Shape.HEX) {
         return;
      }

      // collect triangular faces
      ArrayList<ExFace> triFaces = new ArrayList<ExFace>();
      ExFace[] elemFaces = elem.getFaces();
      for (int i = 0; i < elemFaces.length; i++) {
         if (elemFaces[i] != null) {
            if (elemFaces[i].getShape() == ExFace.Shape.TRIANGLE) {
               triFaces.add(elemFaces[i]);
            }
         }
      }

      int orientationIdx = -1; // default to "not set"
      switch (elemShape) {

         case WEDGE:
         case PYRAMID:

            // opposite faces (0-1, 2-3) must match

            // check consistency
            for (int i = 0; i < triFaces.size(); i++) {
               if (orientedFaceMap.containsKey(triFaces.get(i))) {
                  int tmpIdx = orientedFaceMap.get(triFaces.get(i));
                  tmpIdx = (tmpIdx + 2 * (int)(i / 2)) % 4;
                  if ((orientationIdx > 0) && (tmpIdx != orientationIdx)) {
                     throw new Exception(
                        "Unable to create consistent orientation for wedge element");
                  }
                  orientationIdx = tmpIdx;
               }
            }

            // choose orientation
            if (orientationIdx < 0) {
               System.out.println("Choosing an orientation for "
                  + elemShape.toString() + " element " + elem.getIdx());
               ExLine[] faceLines = triFaces.get(0).getLines();

               // default to collapsed line
               for (int i = 0; i < faceLines.length; i++) {
                  if (faceLines[i] == null) {
                     orientationIdx = i;
                     break;
                  }
                  else if (faceLines[i].isCollapsed()) {
                     orientationIdx = i;
                     break;
                  }
               }
            }

            // set orientation if need to
            for (int i = 0; i < triFaces.size(); i++) {
               ExFace face = triFaces.get(i);
               if (!orientedFaceMap.containsKey(face)) {
                  int collapsed = (orientationIdx + 2 * (int)(i / 2)) % 4;
                  orientedFaceMap.put(face, collapsed);
               }
            }

            break;
         case TET:
            throw new Exception("Orienting TET faces not yet implemented");
      }

   }

   public static void getSplitNumbers(ExElement elem, int[] split,
      HashMap<ExElement,int3> splitNumberMap) throws Exception {

      int3 splitNumbers = new int3(0, 0, 0); // how many divisions to make
                                             // along each dimension

      // find all adjacent elements and check if faces split consistently
      for (ExFace face : elem.getFaces()) {
         if (face != null) {

            // find adjacent element
            ExElement adjacentElem = getAdjacentElement(elem, face);
            if (adjacentElem != null) {

               // check if element has split numbers
               if (splitNumberMap.containsKey(adjacentElem)) {
                  // if it has split numbers, determine those corresponding to
                  // the face
                  int3 adjascentNums = splitNumberMap.get(adjacentElem);

                  // transform to fit this element
                  if (!fillThisSplitDims(
                     elem, splitNumbers, adjacentElem, adjascentNums)) {
                     // there was an error
                     throw new Exception(
                        "Error: cannot split element consistently");
                  }
               }
            }
         }
      }

      // collect split numbers
      ArrayList<Integer> sn = new ArrayList<Integer>();
      for (int i = 0; i < split.length; i++) {
         sn.add(split[i]);
      }

      // remove ones already used
      for (int i = 0; i < splitNumbers.size(); i++) {
         if (splitNumbers.get(i) != 0) {
            int idx = sn.indexOf(splitNumbers.get(i));
            if (idx >= 0) {
               sn.remove(idx); // remove used number from list
            }
         }
      }

      // set remaining
      for (int i = 0; i < splitNumbers.size(); i++) {
         if (splitNumbers.get(i) == 0) {
            splitNumbers.set(i, sn.get(0));
            sn.remove(0);
         }
      }
      splitNumberMap.put(elem, splitNumbers); // set split numbers

   }

   public static ExElement getAdjacentElement(ExElement elem, ExFace face) {

      if (face == null) {
         return null;
      }

      for (ExElement faceElem : face.getDependentElements()) {
         if (faceElem != elem) {
            return faceElem;
         }
      }
      return null;

   }

   public static void getSplitInfo(ExElement elem, int[] split,
      HashMap<ExElement,int3> splitNumberMap, boolean fullSplit,
      boolean surfaceOnly) throws Exception {

      // choose orientations for triangular faces [XXX: REMOVED, relying on
      // exnode/exelem format for consistency]
      // orientTriFaces(elem, orientedFaceMap);

      // determine how to split the element
      getSplitNumbers(elem, split, splitNumberMap);
      // System.out.println("Splitting element " + elem.getIdx() + " as: " +
      // splitNumberMap.get(elem).toString() );

      if (!fullSplit) {
         // set splitting values all to ones
         for (int i = 0; i < split.length; i++) {
            split[i] = 1;
         }
      }

      // recursively repeat on all bordering elements
      ExFace[] elemFaces = elem.getFaces();
      for (int i = 0; i < elemFaces.length; i++) {
         if (elemFaces[i] != null) {

            // check if there's an adjacent element
            ExElement adjacentElem = getAdjacentElement(elem, elemFaces[i]);
            if ((adjacentElem != null)
               && (!splitNumberMap.containsKey(adjacentElem))) {

               // if "surface only", only call if element is on surface
               if (!surfaceOnly || adjacentElem.isOnSurface()) {
                  getSplitInfo(
                     adjacentElem, split, splitNumberMap, fullSplit,
                     surfaceOnly); // split adjacent element
               }

            } // done checking if there is an adjacent element
         }
      } // end looping through faces

   }

   // copies appropriate split numbers from thatElem to thisElem, if possible
   // if copying is not possible or results in a conflict, returns false
   private static boolean fillThisSplitDims(ExElement thisElem,
      int3 thisSplitDims,
      ExElement thatElem, int3 thatSplitDims) {
      // get the common face between the two elements

      int thisPolar = -1; // index of common face for "this" element
      int thatPolar = -1; // index of common face for "that" element
      boolean commonFaceFound = false;

      ExFace[] thisFaces = thisElem.getFaces();
      ExFace[] thatFaces = thatElem.getFaces();

      // find common face
      for (int i = 0; i < thisFaces.length; i++) {
         for (int j = 0; j < thatFaces.length; j++) {
            if (thisFaces[i] == thatFaces[j]) {
               thisPolar = (int)(i / 2); // 0 = xi2/xi3, 1=xi3/xi1, 2=xi1/xi2
               thatPolar = (int)(j / 2);
               commonFaceFound = true;
               break;
            }
         }

         if (commonFaceFound) {
            break;
         }
      }

      if (!commonFaceFound) {
         System.out.println("No common face found");
         return false;
      }

      // 0 = xi2/xi3, 1=xi3/xi1, 2=xi1/xi2
      // copy over correct split numbers as long as consistent
      // (either thisSplitDims not yet set (0), or if values
      // match
      for (int i = 0; i < 2; i++) {
         int thisOffset = (thisPolar + i + 1) % 3;
         int thatOffset = (thatPolar + i + 1) % 3;

         if ((thisSplitDims.get(thisOffset) != 0) &&
            (thisSplitDims.get(thisOffset) != thatSplitDims.get(thatOffset))) {
            System.err.println("Error: inconsistent split numbers");
            return false;
         }
         thisSplitDims.set(thisOffset, thatSplitDims.get(thatOffset));
      }

      return true;
   }

   // ===================================================
   // surface mesh generation
   // ===================================================

   public static PolygonalMesh generateSurfaceMesh(File exNodeFile, File exElemFile,
      int[] resolution) throws IOException {
      
      ExParser parser = new ExParser();
      parser.parseExNode(exNodeFile);
      parser.parseExElem(exElemFile);

      // get last region
      ExRegion myRegion = parser.getLastParsedRegion();

      PolygonalMesh mesh = null;
      String name = myRegion.getName().replace("/", " ").trim();
      
      if (flipZTangents) {
         TangentFlipper flipper = new TangentFlipper();
         flipper.flipAllTangents(myRegion.getElements(), 2);
      }
      
      try {
         mesh =
            ExMeshGenerator.generateSurfaceMesh(
               myRegion.getElements(), resolution);
      } catch (Exception e) {
         e.printStackTrace();
         return null;
      }
      mesh.setName(name);

      return mesh;
      
   }
   public static PolygonalMesh generateSurfaceMesh(String exNodeFile, String exElemFile,
      int[] resolution) throws FileNotFoundException, IOException {
      return generateSurfaceMesh(new File(exNodeFile), new File(exElemFile), resolution);
   }

   public static PolygonalMesh generateSurfaceMesh(
      ArrayList<ExElement> elements, int[] split) throws Exception {

      HashMap<ExElement,int3> splitNumberMap = new HashMap<ExElement,int3>();
      HashMap<ExFace,FaceNodes> faceNodeMap = new HashMap<ExFace,FaceNodes>();
      HashMap<ExLine,LineNodes> lineNodeMap = new HashMap<ExLine,LineNodes>();
      HashMap<ExNode,FemNode3d> nodeNodeMap = new HashMap<ExNode,FemNode3d>();

      FemModel3d tmpModel = new FemModel3d();

      // determine how to split each element
      for (ExElement elem : elements) {
         if (!splitNumberMap.containsKey(elem)) {
            getSplitInfo(elem, split, splitNumberMap, true, true);
         }
      }

      for (ExElement elem : elements) {
         // find elements on surface
         if (elem.isOnSurface()) {

            // go through and generate surface faces
            for (ExFace face : elem.getFaces()) {
               if (face != null) {
                  if (face.isOnSurface()) {
                     // split surface
                     if (!faceNodeMap.containsKey(face)) {
                        splitFace(
                           face, tmpModel, elem, splitNumberMap.get(elem),
                           faceNodeMap, lineNodeMap, nodeNodeMap);
                     } // end checking if map already contains face
                  }// end checking if face is on surface
               } // end checking if face is non-null
            } // end looping through faces

         } // end checking if the element is on the surface
      } // end looping through elements

      // now we should have all the faces in faceNodeMap, we should be able to
      // generate a polygonal mesh
      PolygonalMesh mesh = new PolygonalMesh();
      HashMap<FemNode3d,Vertex3d> nodeToVertexMap =
         new HashMap<FemNode3d,Vertex3d>();

      for (ExFace face : faceNodeMap.keySet()) {
         FaceNodes fn = faceNodeMap.get(face);
         // System.out.println("Split face "+face.getIndex());

         // determine direction
         ArrayList<ExNode> cwNodeList = new ArrayList<ExNode>();
         boolean reversed =
            face.getNodes(cwNodeList, face.getDependentElements().get(0)); // CW
                                                                           // ordering

         // create all vertices
         for (int i = 0; i < fn.nNodes1; i++) {
            for (int j = 0; j < fn.nNodes2; j++) {
               if (!nodeToVertexMap.containsKey(fn.myNodes[i][j])) {
                  Vertex3d vtx = new Vertex3d(fn.myNodes[i][j].getPosition());
                  nodeToVertexMap.put(fn.myNodes[i][j], vtx);
                  mesh.addVertex(vtx);
               }
            }
         }

         int[] rowOffset = { 0, 1, 1, 0 };
         int[] colOffset = { 0, 0, 1, 1 };
         ArrayList<Vertex3d> faceVertices = new ArrayList<Vertex3d>();

         // build faces out of vertices
         for (int i = 0; i < fn.nNodes1 - 1; i++) {
            for (int j = 0; j < fn.nNodes2 - 1; j++) {

               // add faces
               faceVertices.clear();
               for (int k = 0; k < 4; k++) {
                  Vertex3d vtx =
                     nodeToVertexMap.get(fn.myNodes[i + rowOffset[k]][j
                        + colOffset[k]]);
                  if (!faceVertices.contains(vtx)) {
                     faceVertices.add(vtx);
                  } // end checking if vertex needs to be added to face
               } // end looping through vertices that make up face

               Vertex3d[] vertices = faceVertices.toArray(new Vertex3d[0]);
               // reverse order of nodes if required (to be CCW)
               if (!reversed) {
                  Vertex3d tmp = vertices[1];
                  vertices[1] = vertices[vertices.length - 1];
                  vertices[vertices.length - 1] = tmp;
               }
               mesh.addFace(vertices);

            } // end looping through columns
         } // end looping through rows

      } // end looping through faces

      return mesh;
   }

   public static void setUseArclength(boolean useArc) {
      useArclength = useArc;
   }

   // requires 8 nodes
   public static double computeVolume(FemNode3d[] nodes) {

      Point3d[] pts = new Point3d[8];
      for (int i = 0; i < 8; i++) {
         pts[i] = nodes[i].getPosition();
      }
      return ExElement.computeVolume(pts);

   }

}
