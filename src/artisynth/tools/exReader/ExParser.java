package artisynth.tools.exReader;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.ArrayList;

import maspack.util.ReaderTokenizer;

public class ExParser {

   private class LineInfo {
      int idx;
   }

   private class FaceInfo {
      int idx;
      int nLines;
      int[] lineIdxs;
   }

   private class ElemInfo {
      int idx;
      int nFaces;
      int nNodes;
      int nScaleFactors;
      int[] faceIdxs;
      int[] nodeIdxs;
      double[] scaleFactors;
   }

   private class ShapeInfo {
      int dimension;
      String type;
      String interpType;
      int nNodes;
      int nScaleFactorSets;
      String[] scaleFactorType;
      int[] nScaleFactors;
   }

   private class ElemComponentInfo {
      String name;
      String interpType;
      int nNodes;
      ElemNodeInfo[] myNodes;
   }

   private class ElemNodeInfo {
      int nodeIdx;
      int nVals;
      int[] valIdxs;
      int[] scaleFactorIdxs;
   }

   private class ElemFieldInfo {
      public String name;
      public String type;
      public String structure;
      public String interpType;
      public int nComponents;
      ElemComponentInfo[] myComponents;
   }

   private class FieldInfo {
      public String name;
      public String type;
      public String structure;
      public int nComponents;
      public int[] valIndx;
      public int[] nDeriv;
      public int[] nVers;
      public String[] compName;
   }

   private ArrayList<ExRegion> myRegions;

   private FieldInfo myFields[];
   private ShapeInfo currShapeInfo;
   private ElemFieldInfo myElemFields[];

   // temporary info used for constructing elements
   private ArrayList<LineInfo> lineInfoList;
   private ArrayList<FaceInfo> faceInfoList;
   private ArrayList<ElemInfo> elemInfoList;

   private ExRegion currRegion;

   public ExParser () {
      myRegions = new ArrayList<ExRegion>();
      lineInfoList = new ArrayList<LineInfo>();
      faceInfoList = new ArrayList<FaceInfo>();
      elemInfoList = new ArrayList<ElemInfo>();

      currRegion = new ExRegion("/"); // default region
      myRegions.add(currRegion);
      currShapeInfo = new ShapeInfo();
   }

   public void parseExNode(File file) throws IOException,
   FileNotFoundException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(file));
      parseExNode(rtok);
   }

   public void parseExElem(File file) throws IOException,
   FileNotFoundException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(file));
      parseExElem(rtok);
   }

   public void parseExNode(String filename) throws IOException,
   FileNotFoundException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));
      parseExNode(rtok);
   }

   public void parseExElem(String filename) throws IOException,
   FileNotFoundException {
      ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(filename));
      parseExElem(rtok);
   }

   public void parseExElem(ReaderTokenizer rtok) throws IOException {

      // turn off comment char
      int saveHash = rtok.getCharSetting('#');
      int saveExclamation = rtok.getCharSetting('!');
      int saveColon = rtok.getCharSetting(':');
      int saveBS = rtok.getCharSetting('/');
      int saveEqual = rtok.getCharSetting('=');
      int saveAsterix = rtok.getCharSetting('*');
      boolean saveEOL = rtok.getEolIsSignificant();

      rtok.wordChars("#:/=*");
      rtok.commentChar('!');
      rtok.eolIsSignificant(true);

      while (rtok.ttype != ReaderTokenizer.TT_EOF) {
         // advance to next word or EOF
         rtok.nextToken();
         while (rtok.ttype != ReaderTokenizer.TT_EOF
            && rtok.ttype != ReaderTokenizer.TT_WORD) {
            rtok.nextToken();
         }
         if (rtok.ttype == ReaderTokenizer.TT_EOF) {
            break;
         }

         // can be "Region:, Group, #Fields, Node:, Shape, #Scale, #Nodes
         String word = rtok.sval;
         if (word.startsWith("Region")) {
            parseRegionLine(rtok);
         }
         else if (word.startsWith("Group")) {
            // ignore group and move on
            toEOL(rtok);
         }
         else if (word.startsWith("Shape")) {
            parseShapeLine(rtok, currShapeInfo);
         }
         else if (word.startsWith("Element")) {
            parseElemInfo(rtok, currShapeInfo);
         }
         else if (word.startsWith("#Scale")) {
            parseScaleFactorSets(rtok, currShapeInfo);
         }
         else if (word.startsWith("#Nodes")) {
            String nodeLine = rtok.sval;

            // check if line not finished
            if (nodeLine.charAt(nodeLine.length() - 1) == '=') {
               nodeLine += readLine(rtok);
            }
            // check if it has a number, otherwise we need to read the next
            // token
            currShapeInfo.nNodes = parseIntValue(nodeLine, "#Nodes");

            // nodeLine += readLine(rtok); // read the rest of the line if
            // required to
         }
         else if (word.startsWith("#Fields")) {
            parseElemFields(rtok, currShapeInfo);
         }
         else {
            System.out.println("Warning: unknown token '" + rtok.sval
               + "' on line " + rtok.lineno());
            toEOL(rtok);
         }
      }

      // revert rtok settings
      rtok.setCharSetting('#', saveHash);
      rtok.setCharSetting('!', saveExclamation);
      rtok.setCharSetting(':', saveColon);
      rtok.setCharSetting('/', saveBS);
      rtok.setCharSetting('=', saveEqual);
      rtok.setCharSetting('*', saveAsterix);

      rtok.eolIsSignificant(saveEOL);
   }

   public void parseExNode(ReaderTokenizer rtok) throws IOException {

      // turn off comment char
      int saveHash = rtok.getCharSetting('#');
      int saveExclamation = rtok.getCharSetting('!');
      int saveColon = rtok.getCharSetting(':');
      int saveBS = rtok.getCharSetting('/');
      int saveEqual = rtok.getCharSetting('=');
      int saveAsterix = rtok.getCharSetting('*');
      boolean saveEOL = rtok.getEolIsSignificant();

      rtok.wordChars("#:/=*");
      rtok.commentChar('!');
      rtok.eolIsSignificant(true);

      while (rtok.ttype != ReaderTokenizer.TT_EOF) {
         // advance to next word or EOF
         while (rtok.ttype != ReaderTokenizer.TT_EOF
            && rtok.ttype != ReaderTokenizer.TT_WORD) {
            rtok.nextToken();
         }
         if (rtok.ttype == ReaderTokenizer.TT_EOF) {
            break;
         }

         // can be "Region:, Group, #Fields, Node:, Shape, #Scale, #Nodes
         String word = rtok.sval;
         if (word.startsWith("Region")) {
            parseRegionLine(rtok);
         }
         else if (word.startsWith("Group")) {
            // ignore group and move on
            toEOL(rtok);
         }
         else if (word.startsWith("#Fields")) {
            parseFields(rtok);
         }
         else if (word.startsWith("Node")) {
            parseNode(rtok);
         }
         else {
            toEOL(rtok);
         }
      }

      // revert rtok settings
      rtok.setCharSetting('#', saveHash);
      rtok.setCharSetting('!', saveExclamation);
      rtok.setCharSetting(':', saveColon);
      rtok.setCharSetting('/', saveBS);
      rtok.setCharSetting('=', saveEqual);
      rtok.setCharSetting('*', saveAsterix);

      rtok.eolIsSignificant(saveEOL);
   }

   public void parseElemInfo(ReaderTokenizer rtok, ShapeInfo myShape)
      throws IOException {

      if (myShape.dimension == 1) {
         // get number
         LineInfo newElemInfo = new LineInfo();
         newElemInfo.idx = parseElemIndex(rtok);
         if (newElemInfo.idx > 0) {
            lineInfoList.add(newElemInfo);
         }
      }
      else if (myShape.dimension == 2) {
         FaceInfo newElemInfo = new FaceInfo();
         newElemInfo.idx = parseElemIndex(rtok);

         // read faces (lines)
         scanWord(rtok);
         if (!rtok.sval.equals("Faces:")) {
            throw new IOException("Expected \"Faces:\" on line "
               + rtok.lineno());
         }

         newElemInfo.nLines = 4; // always 4 as far as I know
         newElemInfo.lineIdxs = new int[newElemInfo.nLines];
         for (int i = 0; i < newElemInfo.nLines; i++) {
            newElemInfo.lineIdxs[i] = parseElemIndex(rtok);
         }

         if (newElemInfo.idx > 0) {
            faceInfoList.add(newElemInfo);
         }

      }
      else if (myShape.dimension == 3) {

         ElemInfo newElemInfo = new ElemInfo();
         newElemInfo.idx = parseElemIndex(rtok);

         // read faces (lines)
         scanWord(rtok);
         if (!rtok.sval.equals("Faces:")) {
            throw new IOException("Expected \"Faces:\" on line "
               + rtok.lineno());
         }

         newElemInfo.nFaces = 6; // always 6 as far as I know
         newElemInfo.faceIdxs = new int[newElemInfo.nFaces];
         for (int i = 0; i < newElemInfo.nFaces; i++) {
            newElemInfo.faceIdxs[i] = parseElemIndex(rtok);
         }

         // read nodes
         newElemInfo.nNodes = myShape.nNodes;
         newElemInfo.nodeIdxs = new int[newElemInfo.nNodes];
         scanWord(rtok);
         if (!rtok.sval.equals("Nodes:")) {
            throw new IOException("Expected \"Nodes:\" on line "
               + rtok.lineno());
         }
         toEOL(rtok);
         scanNumbers(rtok, newElemInfo.nodeIdxs, newElemInfo.nNodes); // read in
         // node
         // indices

         // Scale factors
         int totalSF = 0;
         for (int i = 0; i < myShape.nScaleFactorSets; i++) {
            totalSF += myShape.nScaleFactors[i];
         }
         newElemInfo.nScaleFactors = totalSF;
         newElemInfo.scaleFactors = new double[totalSF];

         if (newElemInfo.nScaleFactors > 0) {
            scanWord(rtok);
            if (!rtok.sval.startsWith("Scale")) {
               throw new IOException("Expected \"Scale factors:\" on line "
                  + rtok.lineno());
            }
            toEOL(rtok);
            scanNumbers(rtok, newElemInfo.scaleFactors, totalSF);
         }

         if (newElemInfo.idx > 0) {
            elemInfoList.add(newElemInfo);
         }

         // find coordinates field
         int idx = 0;
         for (int i = 0; i < myElemFields.length; i++) {
            if (myElemFields[i].name.equals("coordinates")) {
               idx = i;
               break;
            }
         }

         findOrBuildElement(
            newElemInfo, myElemFields[idx], faceInfoList, lineInfoList,
            currRegion);

         // // print info
         // ExNode[] elemNodes = newElem.getNodes();
         // int[] elemNodeVersions = newElem.getNodeVersions();
         // System.out.print("New Element " + newElem.getIdx() + "  Nodes: \t");
         // for (int i=0; i<elemNodes.length; i++) {
         // System.out.print(elemNodes[i].getNodeIdx() +
         // "("+elemNodeVersions[i]+") ");
         // }
         // System.out.print("\n");

      }

   }

   public ExNode getNode(int index, ArrayList<ExNode> nodeList) {
      for (ExNode node : nodeList) {
         if (node.getNodeIdx() == index) {
            return node;
         }
      }
      return null;
   }

   // CAUTION: assumes hex-like element
   public ExElement findOrBuildElement(ElemInfo elemInfo,
      ElemFieldInfo elemFieldInfo, ArrayList<FaceInfo> faceInfoList,
      ArrayList<LineInfo> lineInfoList, ExRegion region) {

      // element is collapsed
      if (elemInfo == null) {
         return null;
      }

      ArrayList<ExNode> orderedNodes = new ArrayList<ExNode>();
      ArrayList<Integer> orderedVersions = new ArrayList<Integer>();

      // creates ordered list of nodes and versions to use in the element
      // System.out.print("Element " + elemInfo.idx+ "  Nodes: \t");
      for (int i = 0; i < elemFieldInfo.myComponents[0].nNodes; i++) {
         ExNode nextNode =
            getNode(
               elemInfo.nodeIdxs[elemFieldInfo.myComponents[0].myNodes[i].nodeIdx - 1],
               region.getNodes());
         int nodeSize =
            nextNode.getComponent("coordinates", "x").getNumDerivatives() + 1;
         orderedNodes.add(nextNode);
         int ver =
            (int)((elemFieldInfo.myComponents[0].myNodes[i].valIdxs[0] - 1) / nodeSize);
         orderedVersions.add(ver);
         if (ver >= nextNode.getComponent("coordinates", "x").getNumVersions()) {
            System.out.println("Invalid version number: " + ver
               + ", Val Index: "
               + elemFieldInfo.myComponents[0].myNodes[i].valIdxs[0]
                  + ", Num Versions: "
                  + nextNode.getComponent("coordinates", "x").getNumVersions());
         }
         // System.out.print(nextNode.getNodeIdx() + "("+ver+") ");
      }
      // System.out.print("\n");

      // check if element exists, return if found
      ArrayList<ExElement> shortElemList =
         orderedNodes.get(0).getDependentElements();
      for (int i = 0; i < shortElemList.size(); i++) {
         if (shortElemList.get(i).equals(orderedNodes)) {
            return shortElemList.get(i);
         }
      }

      // element not found, so build it
      ExElement newElem = new ExElement();
      newElem.setIdx(elemInfo.idx);

      // face 0: nodes 0 2 4 6
      // face 1: nodes 1 3 5 7
      // face 2: nodes 0 1 4 5
      // face 3: nodes 2 3 6 7
      // face 4: nodes 0 2 1 3
      // face 5: nodes 4 6 5 7
      int[][] faceNodeIdxs = ExElement.faceNodeIdxs;

      // for each face, check if exists, if not build it
      for (int i = 0; i < faceNodeIdxs.length; i++) {

         FaceInfo faceInfo = getFaceInfo(elemInfo.faceIdxs[i], faceInfoList); // get
         // face
         // information

         // build list of nodes for the face
         ArrayList<ExNode> faceNodes = new ArrayList<ExNode>();
         ArrayList<Integer> faceVersions = new ArrayList<Integer>();
         for (int j = 0; j < faceNodeIdxs[i].length; j++) {
            faceNodes.add(orderedNodes.get(faceNodeIdxs[i][j]));
            faceVersions.add(orderedVersions.get(faceNodeIdxs[i][j]));
         }

         // find or build the face and add it to the element
         ExFace face;
         try {
            face = findOrBuildFace(
               faceInfo, faceNodes, faceVersions, lineInfoList, region);
         } catch (NodeOrderingException e) {
            face = (ExFace)e.getObj();
            System.out
            .println("Element " + newElem.getIdx() + " has a funny face "
               + face.getIndex() + ": " + e.getMessage());
         }
         newElem.setFace(i, face);
      }

      // fill in scale factors
      ElemComponentInfo compInfo = elemFieldInfo.myComponents[0]; // read
      // component
      // information
      // from "x",
      // assume same
      // for "y" and
      // "z"
      double[][] scaleFactors = new double[compInfo.nNodes][];
      for (int i = 0; i < compInfo.nNodes; i++) {
         scaleFactors[i] = new double[compInfo.myNodes[i].nVals];
         for (int j = 0; j < compInfo.myNodes[i].nVals; j++) {
            scaleFactors[i][j] =
               elemInfo.scaleFactors[compInfo.myNodes[i].scaleFactorIdxs[j] - 1];
         }
      }
      newElem.setScaleFactors(scaleFactors);

      // check interpolation type
      String[] strType = elemFieldInfo.interpType.split("\\*");
      if (strType.length < 3) {
         System.out.println("Invalid element interpolation type: "
            + elemFieldInfo.interpType);
         strType = new String[] { "l.Lagrange", "l.Lagrange", "l.Lagrange" };
      }

      NodeInterpolator.InterpType[] interpType =
         new NodeInterpolator.InterpType[3];
      for (int i = 0; i < 3; i++) {
         if (strType[i].contains("c.Hermite")) {
            interpType[i] = NodeInterpolator.InterpType.CUBIC;
         }
         else if (strType[i].contains("l.Lagrange")) {
            interpType[i] = NodeInterpolator.InterpType.LINEAR;
         }
         else {
            System.out.println("Unknown interplation type: " + strType[i]);
            interpType[i] = NodeInterpolator.InterpType.LINEAR;
         }
      }
      newElem.setInterpType(interpType);

      if (newElem.isCollapsed()) {
         System.out.println("Hmm... element seems to be collapsed.");
      }

      // add element to region
      region.addElement(newElem);

      return newElem;

   }

   public ExFace findOrBuildFace(FaceInfo faceInfo, ArrayList<ExNode> nodes,
      ArrayList<Integer> versions, ArrayList<LineInfo> lineInfoList,
      ExRegion region) throws NodeOrderingException {

      // if faceInfo == null, it is collapsed
      if (faceInfo == null) {
         return null;
      }

      // check if face exists, return if found
      ArrayList<ExFace> shortFaceList = nodes.get(0).getDependentFaces();
      for (int i = 0; i < shortFaceList.size(); i++) {

         boolean equal = shortFaceList.get(i).equals(nodes);
         if (equal) {
            return shortFaceList.get(i);
         }
      }

      // build new face
      ExFace newFace = new ExFace();
      newFace.setIdx(faceInfo.idx);

      // line 0: nodes 0 2
      // line 1: nodes 1 3
      // line 2: nodes 0 1
      // line 3: nodes 2 3
      int[][] lineNodeIdxs = ExFace.lineNodeIdxs;

      boolean lineErr = false;

      // for each face, check if exists, if not build it
      for (int i = 0; i < lineNodeIdxs.length; i++) {

         LineInfo lineInfo = getLineInfo(faceInfo.lineIdxs[i], lineInfoList); 

         // build list of nodes for the line
         ArrayList<ExNode> lineNodes = new ArrayList<ExNode>();
         ArrayList<Integer> lineVersions = new ArrayList<Integer>();
         for (int j = 0; j < lineNodeIdxs[i].length; j++) {
            lineNodes.add(nodes.get(lineNodeIdxs[i][j]));
            lineVersions.add(versions.get(lineNodeIdxs[i][j]));
         }

         // find or build the face and add it to the element
         ExLine line;
         try {
            //            if (lineInfo != null) {
            //               System.out.println("Line index: "+lineInfo.idx);
            //            } else {
            //               System.out.println("Line info is null");
            //            }
            line = findOrBuildLine(lineInfo, lineNodes, lineVersions, region);

         } catch (NodeOrderingException e) {
            lineErr = true;
            line = (ExLine)e.getObj();
            System.out.println("Trouble building face " + newFace.getIndex()
               + " because of line " + line.getIdx() + ": "+ e.getMessage());
         }
         newFace.setLine(i, line);
      }

      region.addFace(newFace);
      if (lineErr) {
         throw new NodeOrderingException(
            newFace, "Face messed up because of funny lines");
      }

      return newFace;

   }

   public ExLine findOrBuildLine(LineInfo lineInfo, ArrayList<ExNode> nodes,
      ArrayList<Integer> versions, ExRegion region)
         throws NodeOrderingException {

      // line is collapsed
      if (lineInfo == null) {
         return null;
      }

      // check if line exists, return if found
      ArrayList<ExLine> shortLineList = nodes.get(0).getDependentLines();
      for (int i = 0; i < shortLineList.size(); i++) {
         boolean equals = shortLineList.get(i).equals(nodes, versions);
         if (equals) {
            return shortLineList.get(i);
         }
      }

      // build new line
      ExLine newLine = new ExLine();
      newLine.setIdx(lineInfo.idx);

      for (int i = 0; i < 2; i++) {
         newLine.setNode(i, nodes.get(i), versions.get(i));
      }

      region.addLine(newLine);
      return newLine;

   }

   public LineInfo getLineInfo(int index, ArrayList<LineInfo> lineList) {

      if (index == 0) {
         return null;
      }

      for (LineInfo line : lineList) {
         if (line.idx == index) {
            return line;
         }
      }

      System.out.println("Warning: cannot find line " + index);
      return null;
   }

   public FaceInfo getFaceInfo(int index, ArrayList<FaceInfo> faceList) {

      if (index == 0) {
         return null;
      }
      for (FaceInfo face : faceList) {
         if (face.idx == index) {
            return face;
         }
      }

      System.out.println("Warning: cannot find face " + index);
      return null;
   }

   public int parseElemIndex(ReaderTokenizer rtok) throws IOException {

      double triple[] = new double[3];

      scanNumbers(rtok, triple, triple.length);
      for (int i = 0; i < triple.length; i++) {
         if (triple[i] != 0) {
            return (int)triple[i];
         }
      }

      return 0;

   }

   public void parseScaleFactorSets(ReaderTokenizer rtok, ShapeInfo myShape)
      throws IOException {
      // get number of sets
      String line = readLine(rtok);
      myShape.nScaleFactorSets = parseIntValue(line, "sets");
      myShape.scaleFactorType = new String[myShape.nScaleFactorSets];
      myShape.nScaleFactors = new int[myShape.nScaleFactorSets];

      for (int i = 0; i < myShape.nScaleFactorSets; i++) {
         line = readLine(rtok);
         int idx = line.indexOf(',');
         myShape.interpType = line.substring(0, idx).trim();
         myShape.scaleFactorType[i] = line.substring(0, idx).trim();
         myShape.nScaleFactors[i] = parseIntValue(line, "#Scale factors");
      }
   }

   public void parseElemFields(ReaderTokenizer rtok, ShapeInfo myShape)
      throws IOException {

      // Parse number of fields
      int nFields = parseIntValue(rtok.sval, "#Fields");
      toEOL(rtok);

      myElemFields = new ElemFieldInfo[nFields];

      // parse each field information
      int fieldIdx = 0;
      for (int i = 0; i < nFields; i++) {

         // parse field information
         rtok.nextToken(); // field index
         fieldIdx = ((int)rtok.nval) - 1;
         if (myElemFields[fieldIdx] == null) {
            myElemFields[fieldIdx] = new ElemFieldInfo();
            myElemFields[fieldIdx].interpType = myShape.interpType;
         }

         while (rtok.nextToken() != ReaderTokenizer.TT_WORD)
            ; // read until next word
         myElemFields[fieldIdx].name = rtok.sval;

         String line = readLine(rtok);
         String fieldLine[] = line.split(","); // separate by commas
         myElemFields[fieldIdx].type = fieldLine[0].trim();
         myElemFields[fieldIdx].structure = fieldLine[1].trim();
         String tmp = fieldLine[2].trim();
         int ncmpnts = parseIntValue(tmp, "#Components");

         myElemFields[fieldIdx].nComponents = ncmpnts;
         myElemFields[fieldIdx].myComponents = new ElemComponentInfo[ncmpnts];

         // parse field components
         for (int j = 0; j < ncmpnts; j++) {
            line = readLine(rtok);
            ElemComponentInfo newComp = new ElemComponentInfo();
            myElemFields[fieldIdx].myComponents[j] = newComp;

            // get field component name
            int idx = line.indexOf('.');
            while (idx < 0) {
               line = readLine(rtok);
               idx = line.indexOf('.');
            }
            newComp.name = line.substring(0, idx).trim();
            line = line.substring(idx + 1);
            idx = line.indexOf(',');
            newComp.interpType = line.substring(0, idx).trim();

            // read all nodes
            while (rtok.nextToken() != ReaderTokenizer.TT_WORD)
               ;
            line = rtok.sval;
            if (line.endsWith("=")) {
               line += readLine(rtok);
            }
            else {
               toEOL(rtok);
            }
            newComp.nNodes = parseIntValue(line, "#Nodes");
            newComp.myNodes = new ElemNodeInfo[newComp.nNodes];

            // parse each node
            for (int k = 0; k < newComp.nNodes; k++) {
               line = readLine(rtok);
               idx = line.indexOf('.');
               if (idx < 0) {
                  while (idx < 0) {
                     line = readLine(rtok);
                     idx = line.indexOf('.');
                  }
               }
               ElemNodeInfo newNode = new ElemNodeInfo();
               newComp.myNodes[k] = newNode;
               newNode.nodeIdx = parseInteger(line.substring(0, idx));
               newNode.nVals = parseIntValue(line, "#Values");
               newNode.valIdxs = new int[newNode.nVals];
               newNode.scaleFactorIdxs = new int[newNode.nVals];

               // value indices
               while (true) {
                  rtok.nextToken();
                  if (rtok.ttype == ReaderTokenizer.TT_WORD) {
                     if (rtok.sval.startsWith("indices:")) {
                        break;
                     }
                     else if (!rtok.sval.startsWith("Value")) {
                        throw new IOException(
                           "Expected \"Value indices\" on line "
                              + rtok.lineno());
                     }
                  }
               }
               scanNumbers(rtok, newNode.valIdxs, newNode.nVals);

               // scale factor indices
               while (true) {
                  rtok.nextToken();
                  if (rtok.ttype == ReaderTokenizer.TT_WORD) {
                     if (rtok.sval.startsWith("indices:")) {
                        break;
                     }
                     else if (!rtok.sval.startsWith("Scale")
                        && !rtok.sval.startsWith("factor")) {
                        throw new IOException(
                           "Expected \"Scale factor indices:\" on line "
                              + rtok.lineno());
                     }
                  }
               }
               scanNumbers(rtok, newNode.scaleFactorIdxs, newNode.nVals);

            }

         }
         rtok.nextToken (); // prepare for next field
      }

   }

   public void parseShapeLine(ReaderTokenizer rtok, ShapeInfo myShape)
      throws IOException {

      // skip to next word to find dimension
      rtok.nextToken();
      while (rtok.ttype != ReaderTokenizer.TT_WORD) {
         if (rtok.ttype == ReaderTokenizer.TT_EOL) {
            throw new IOException("Unexpected end of line on line: "
               + rtok.lineno());
         }
         rtok.nextToken();
      }

      myShape.nNodes = 0;
      myShape.nScaleFactorSets = 0;
      myShape.dimension = parseIntValue(rtok.sval, "Dimension");

      // get type
      rtok.nextToken();
      boolean useDefault = false;
      while (rtok.ttype != ReaderTokenizer.TT_WORD) {
         if (rtok.ttype == ReaderTokenizer.TT_EOL) {
            useDefault = true;
            break;
         }
         rtok.nextToken();
      }

      if (useDefault) {
         myShape.type = "line";
      }
      else {
         myShape.type = rtok.sval;
      }

      toEOL(rtok);

   }

   public void parseRegionLine(ReaderTokenizer rtok) throws IOException {

      // read region name
      if (!rtok.sval.endsWith(":")) {
         rtok.nextToken(); // advance until we get the colon
      }
      rtok.nextToken();

      // clear temporary arrays storing info for constructing elements
      lineInfoList.clear();
      faceInfoList.clear();
      elemInfoList.clear();

      String regionName = "/";
      if (rtok.ttype == ReaderTokenizer.TT_WORD) {
         regionName = rtok.sval;
      }

      // find region if exists, otherwise create
      boolean found = false;
      for (ExRegion reg : myRegions) {
         if (reg.getName().equals(regionName)) {
            currRegion = reg;
            found = true;
            break;
         }
      }

      if (found == false) {
         ExRegion newRegion = new ExRegion(regionName);
         myRegions.add(newRegion);
         currRegion = newRegion;
      }

      // go to end of line
      toEOL(rtok);

   }

   public void parseFields(ReaderTokenizer rtok) throws IOException {

      // Parse number of fields
      int nFields = parseIntValue(rtok.sval, "#Fields");

      toEOL(rtok);

      myFields = new FieldInfo[nFields];

      // parse each field information
      int fieldIdx = 0;
      for (int i = 0; i < nFields; i++) {

         // parse field information
         rtok.nextToken(); // field index
         fieldIdx = ((int)rtok.nval) - 1;
         if (myFields[fieldIdx] == null) {
            myFields[fieldIdx] = new FieldInfo();
         }

         while (rtok.nextToken() != ReaderTokenizer.TT_WORD)
            ; // read until next word
         myFields[fieldIdx].name = rtok.sval;

         String line = readLine(rtok);
         String fieldLine[] = line.split(","); // separate by commas
         myFields[fieldIdx].type = fieldLine[0].trim();
         myFields[fieldIdx].structure = fieldLine[1].trim();
         String tmp = fieldLine[2].trim();
         int ncmpnts = parseIntValue(tmp, "#Components");

         myFields[fieldIdx].nComponents = ncmpnts;

         // parse field components
         myFields[fieldIdx].valIndx = new int[ncmpnts];
         myFields[fieldIdx].nDeriv = new int[ncmpnts];
         myFields[fieldIdx].nVers = new int[ncmpnts];
         myFields[fieldIdx].compName = new String[ncmpnts];

         for (int j = 0; j < ncmpnts; j++) {
            line = readLine(rtok);

            // get field component name
            int idx = line.indexOf('.');
            myFields[fieldIdx].compName[j] = line.substring(0, idx).trim();
            line = line.substring(idx);

            // get value index
            myFields[fieldIdx].valIndx[j] =
               parseIntValue(line, "Value index", 1) - 1;

            // get number of derivatives
            myFields[fieldIdx].nDeriv[j] =
               parseIntValue(line, "#Derivatives", 0);

            // get number of versions (set to 1 if not found)
            myFields[fieldIdx].nVers[j] = parseIntValue(line, "#Versions", 1);

         }
         rtok.nextToken ();  // prepare for next field
      }
      rtok.nextToken(); // prep for next line

   }

   public void parseNode(ReaderTokenizer rtok) throws IOException {

      // node index
      ExNode node = new ExNode();
      node.setIdx(rtok.scanInteger());

      int maxIndex = 0;

      // determine size of numeric array
      for (int i = 0; i < myFields.length; i++) {
         for (int j = 0; j < myFields[i].nComponents; j++) {
            int lastIdx =
               myFields[i].valIndx[j] + (myFields[i].nDeriv[j] + 1)
               * myFields[i].nVers[j];
            if (lastIdx > maxIndex) {
               maxIndex = lastIdx;
            }
         }
      }

      double vals[] = new double[maxIndex];
      // go to EOL
      toEOL(rtok);

      // read numbers
      int read = scanNumbers(rtok, vals, maxIndex);

      if (read != maxIndex) {
         System.out.println("Error: expecting " + maxIndex + " numbers, read "
            + read + ".");
      }

      // now create the actual node
      for (int i = 0; i < myFields.length; i++) {
         ExNodeField nf =
            new ExNodeField(
               myFields[i].name, myFields[i].type, myFields[i].structure);
         for (int j = 0; j < myFields[i].nComponents; j++) {
            ExNodeComponent nc =
               new ExNodeComponent(
                  myFields[i].nVers[j], myFields[i].nDeriv[j], vals,
                  myFields[i].valIndx[j]);
            nc.setName(myFields[i].compName[j]);
            nf.addComponent(nc);
         }
         node.addField(nf);
      }

      currRegion.addNode(node);

   }

   // gets integer from an expression of the form "label=value"
   private int parseIntValue(String expression, String label)
      throws IOException {
      int iStart = expression.indexOf(label);
      if (iStart < 0) {
         throw new IOException("Label '" + label
            + "' not found in expression '" + expression + "'.");
      }
      iStart = expression.indexOf('=', iStart) + 1;
      expression = expression.substring(iStart);
      return parseInteger(expression);
   }

   private int parseIntValue(String expression, String label, int defaultValue) {
      int iStart = expression.indexOf(label);
      if (iStart < 0) {
         return defaultValue;
      }
      iStart = expression.indexOf('=', iStart) + 1;
      expression = expression.substring(iStart);
      return parseInteger(expression);
   }

   private int parseInteger(String str) throws NumberFormatException {

      int startIdx = 0;
      int endIdx = str.length();

      if (str.length() == 0) {
         throw new NumberFormatException("No number found in string '" + str
            + "'");
      }

      // loop to first numeric character
      int idx = 0;
      while (!(str.charAt(idx) >= '0' && str.charAt(idx) <= '9')) {
         idx++;
         if (idx >= str.length()) {
            break;
         }
      }
      startIdx = idx;

      // accumulate numeric characters
      while (str.charAt(idx) >= '0' && str.charAt(idx) <= '9') {
         idx++;
         if (idx >= str.length()) {
            break;
         }
      }
      endIdx = idx;

      str = str.substring(startIdx, endIdx);

      if (startIdx > endIdx) {
         throw new NumberFormatException("No number found in string '" + str
            + "'");
      }
      return Integer.parseInt(str);
   }

   // read next word, skipping over newlines
   public String scanWord(ReaderTokenizer rtok) throws IOException {

      while (true) {
         rtok.nextToken();
         if (rtok.ttype == ReaderTokenizer.TT_WORD) {
            return rtok.sval;
         }
         else if (rtok.ttype != ReaderTokenizer.TT_EOL) {
            break;
         }
      }
      return null;

   }

   // read numbers, skipping over newlines
   public int scanNumbers(ReaderTokenizer rtok, double val[], int maxCount)
      throws IOException {

      int readCount = 0;
      while (true) {
         rtok.nextToken();
         if (rtok.ttype == ReaderTokenizer.TT_NUMBER) {
            val[readCount] = rtok.nval;
            readCount++;

            // if anything else other than number or EOL, then break
         }
         else if (rtok.ttype != ReaderTokenizer.TT_EOL) {
            break;
         }
         if (readCount == maxCount) {
            break;
         }
      }
      return readCount;

   }

   public int scanNumbers(ReaderTokenizer rtok, int val[], int maxCount)
      throws IOException {

      int readCount = 0;
      while (true) {
         rtok.nextToken();
         if (rtok.ttype == ReaderTokenizer.TT_NUMBER) {
            val[readCount] = (int)rtok.nval;
            readCount++;

            // if anything else other than number or EOL, then break
         }
         else if (rtok.ttype != ReaderTokenizer.TT_EOL) {
            break;
         }
         if (readCount == maxCount) {
            break;
         }
      }
      return readCount;
   }

   protected String readLine(ReaderTokenizer rtok) throws IOException {

      Reader rtokReader = rtok.getReader();
      String line = "";
      int c;
      while (true) {
         c = rtokReader.read();

         if (c == '\n' || c < 0) {
            rtok.setLineno(rtok.lineno() + 1); // increase line number
            break;
         }
         line += (char)c;
      }

      return line;
   }

   protected int nextToken(ReaderTokenizer rtok) throws IOException {
      rtok.nextToken();
      return rtok.ttype;
   }

   private void toEOL(ReaderTokenizer rtok) throws IOException {
      while ((rtok.ttype != ReaderTokenizer.TT_EOL)
         && (rtok.ttype != ReaderTokenizer.TT_EOF)) {
         nextToken(rtok);
      }
   }

//   private void toNextLine(ReaderTokenizer rtok) throws IOException {
//      toEOL(rtok);
//      if (rtok.ttype == ReaderTokenizer.TT_EOL) {
//         nextToken(rtok); // advance one more if not end-of-file
//      }
//   }

   public ExRegion getRegion(String name) {
      for (ExRegion reg : myRegions) {
         if (reg.getName().equals(name)) {
            return reg;
         }
      }
      return null;
   }

   public ExRegion getLastParsedRegion() {
      return currRegion;
   }

   public ArrayList<ExRegion> getRegions() {
      return myRegions;
   }

   public void clear() {
      myRegions.clear();
      lineInfoList.clear();
      faceInfoList.clear();
      elemInfoList.clear();

      currRegion = new ExRegion("/"); // default region
      myRegions.add(currRegion);
      currShapeInfo = new ShapeInfo();
   }
}
