package artisynth.tools.exReader;

import java.util.ArrayList;

import artisynth.tools.exReader.NodeInterpolator.InterpType;
import maspack.matrix.Vector3d;

public class TangentFlipper {

   public final int[][] FLIP_IDX = { { 1, 3, 5, 7 },
                                    { 2, 3, 6, 7 },
                                    { 4, 5, 6, 7 } };

//   public HashMap<ExNode,NodeFlippedFlags> flipFlagsMap;
//   private class NodeFlippedFlags {
//      public int nVersions;
//      public int nDimensions;
//      public ArrayList<boolean[]> checked;
//      public ArrayList<boolean[]> flipped;
//
//      public NodeFlippedFlags (int nVers, int nDim) {
//         nVersions = nVers;
//         nDimensions = nDim;
//
//         checked = new ArrayList<boolean[]>(nVers);
//         flipped = new ArrayList<boolean[]>(nVers);
//
//         for (int i = 0; i < nVers; i++) {
//            checked.add(new boolean[nDim]);
//            flipped.add(new boolean[nDim]);
//         }
//
//      }
//
//      // flips a derivative if required
//      // true if polarization set consistently
//      // false if causes a clash
//      public boolean flipDeriv(int ver, int dim, boolean flip) {
//         if (checked.get(ver)[dim]) {
//            if (flipped.get(ver)[dim] != flip) {
//               return false;
//            }
//         } else {
//            checked.get(ver)[dim] = true;
//            flipped.get(ver)[dim] = flip;
//         }
//         return true;
//      }
//
//      public void addVersion() {
//         nVersions++;
//         checked.add(new boolean[nDimensions]);
//         flipped.add(new boolean[nDimensions]);
//
//      }
//
//   }

   public TangentFlipper () {
//      flipFlagsMap = new HashMap<ExNode,NodeFlippedFlags>();
   }

   public void fixTangents(ArrayList<ExElement> elems, double cosineThreshold) {

      ArrayList<ExLine> uniqueLines = new ArrayList<ExLine>();
      
      for (ExElement elem : elems) {
         uniqueLines.clear();
         for (ExLine line : elem.getLines()) {
            if (line != null && !uniqueLines.contains(line)) {
               fixLine(line, elem, cosineThreshold);
               uniqueLines.add(line);
            }
         }
      }

   }

   private void fixLine(ExLine line, ExElement elem, double cosineThreshold) {

      int[] vers = line.getNodeVersions();
      ExNode[] nodes = line.getNodes();
      Vector3d[] nodeInterpDirs = new Vector3d[2];

      Vector3d lineDisp = new Vector3d(nodes[1].getCoordinate());
      lineDisp.sub(new Vector3d(nodes[0].getCoordinate()));
      lineDisp.normalize();
      
      // check if interpolation is in other halfspace
      int lineIdx = elem.getLineIdx(line);
      double[][] scaleFactors = elem.getScaleFactors();
      int[] nodeIdxs = ExElement.lineNodeIdxs[lineIdx];
      
      int dim = elem.getLineDimension(line);
      int dims[] = { 0 };
      dims[0] = dim;

      NodeInterpolator.InterpType interpType = elem.getInterpType()[dim];
      
      // ignore if linear, since no derivatives
      if (interpType == InterpType.LINEAR) {
         return;
      }
      
      for (int i = 0; i < 2; i++) {
         nodeInterpDirs[i] = new Vector3d(nodes[i].getDerivativeCoordinate(vers[i], FLIP_IDX[dim][0]));
         nodeInterpDirs[i].scale(scaleFactors[nodeIdxs[i]][FLIP_IDX[dim][0]]);
         double tangentNorm = nodeInterpDirs[i].norm();
         nodeInterpDirs[i].normalize();
         double cosine = nodeInterpDirs[i].dot(lineDisp);
         
         boolean flip = (cosine < cosineThreshold);

         if (flip) {
            System.out.println("Flipping tangent for node "
               + nodes[i].getNodeIdx() + "("+vers[i]+") in element " + elem.getIdx()
               + " along dimension " + dim +", cosine="+cosine+", norm="+tangentNorm);
            
            // flip scale factors
            flipTangentSF(scaleFactors, nodeIdxs[i], dim);
            
         }

      }

   }
   
   private void flipTangentSF(double[][] scaleFactors, int nodeIdx, int dim) {
      // flip scale factors
      for (int k = 0; k < FLIP_IDX[dim].length; k++) {
         if (FLIP_IDX[dim][k] < scaleFactors[nodeIdx].length) {
            scaleFactors[nodeIdx][FLIP_IDX[dim][k]] = -scaleFactors[nodeIdx][FLIP_IDX[dim][k]];
         }
      }
   }
   
   public void flipAllTangents(ArrayList<ExElement> elems, int dim) {
      
      for (ExElement elem : elems) {
         double[][] scaleFactors = elem.getScaleFactors();
         
         for (int i=0; i<8; i++) {
            flipTangentSF(scaleFactors, i, dim);
         }
      }
      
   }

}
