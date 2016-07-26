package artisynth.tools.exReader;

import java.util.Arrays;

import maspack.matrix.Point3d;

// Might be used to make faster
//  public class ArclengthInterpData {
//   ArrayList<Double> arcArray;
//   ArrayList<Double> xiArray;
//   double totalArclength;
//
//   public ArclengthInterpData() {
//	 arcArray = new ArrayList<Double>();
//	 xiArray = new ArrayList<Double>();
//	 totalArclength = 0;
//   }
//   
//   public void setArclength(double len) {
//	 totalArclength = len;
//   }
//   
//   public double getArclength() {
//	 return totalArclength;
//   }
//   
//   void addEntry(double arc, double xi) {
//	int idx = Collections.binarySearch(arcArray, arc);
//	// if entry not found, add it
//	if (idx<0) {
//	   int insertion = (idx+1)*(-1);
//	   xiArray.add(insertion, xi);
//	   arcArray.add(insertion, arc);
//	}
//   }
//}

public class NodeInterpolator {

   public enum InterpType {
      LINEAR, CUBIC // TRICUBIC, TRILINEAR, BICUBIC, BILINEAR,
   }

   static double defaultStepSize = 0.01;
   static double defaultThreshold = 0.001;

   public static final int[] ALL8NODES = { 0, 1, 2, 3, 4, 5, 6, 7 };
   public static final int[] ALL4NODES = { 0, 1, 2, 3 };
   public static final int[] ALL2NODES = { 0, 1 };
   public static final int[] ORIGINAL_DIMENSIONS = { 0, 1, 2 };

   // kronecker-like operation to build indices for derivatives/nodes to use in
   // interpolation
   private static int index_kron(int[] v1, int v1Size, int[] v2, int v2Size,
      int power, int[] out) {

      for (int i = v1Size - 1; i >= 0; i--) {
         for (int j = v2Size - 1; j >= 0; j--) {
            out[i * v2Size + j] = v1[i] + v2[j] * ((int)Math.pow(2, power));
         }
      }

      return v1Size * v2Size;
   }

   // kronecker product of two vectors
   public static int kron(double[] v1, int v1Size, double[] v2, int v2Size,
      double[] out) {

      for (int i = v1Size - 1; i >= 0; i--) {
         for (int j = v2Size - 1; j >= 0; j--) {
            out[i * v2Size + j] = v1[i] * v2[j];
         }
      }
      return v1Size * v2Size;

   }

   // linear basis function
   private static double linearBasis(int derivIdx, int nodeIdx, double xi) {

      if (derivIdx == 0) {
         if (nodeIdx == 0) {
            return 1 - xi;
         } else if (nodeIdx == 1) {
            return xi;
         }
      }
      return 0;
   }

   private static double cubicBasis(int derivIdx, int nodeIdx, double xi) {

      // if (adjust) {
      // // first-order correction for linear spacing
      // double theta = Math.atan(2.0*Math.sqrt(xi-xi*xi)/(1-2*xi));
      // if (xi > 0.5) {
      // theta += Math.PI; // correct to be a positive angle
      // }
      // theta = theta/3;
      // xi = -0.5*Math.cos(theta)+Math.sqrt(3)*0.5*Math.sin(theta)+0.5;
      // }

      if ((derivIdx == 0) && (nodeIdx == 0)) {
         return 1 - 3 * xi * xi + 2 * xi * xi * xi;
      } else if ((derivIdx == 0) && (nodeIdx == 1)) {
         return 3 * xi * xi - 2 * xi * xi * xi;
      } else if ((derivIdx == 1) && (nodeIdx == 0)) {
         return xi * (xi - 1) * (xi - 1);
      } else if ((derivIdx == 1) && (nodeIdx == 1)) {
         return xi * xi * (xi - 1);
      }

      return 0;

   }

   // interpolation function
   static int xxx=0;
   public static Point3d interp(double[] xi, InterpType[] interpTypes,
      ExNode[] nodes, int[] versions,
      double[][] scaleFactors, int[] nodeIdxs, int[] dimensions) {

      Point3d out = new Point3d(0, 0, 0);

      int nOrder = xi.length;
      int nTerms = 1;
      for (int i = 0; i < nOrder; i++) {
         if (interpTypes[i] == InterpType.CUBIC) {
            nTerms *= 4;
         } else if (interpTypes[i] == InterpType.LINEAR) {
            nTerms *= 2;
         }
      }
      double[] coeffs = new double[nTerms];
      int[] pntIdxs = new int[nTerms];
      int[] derivIdxs = new int[nTerms];
      double[] basis = new double[4];

      final int[] LINEAR_DERIV_IDX = { 0, 0 };
      final int[] LINEAR_PNT_IDX = { 0, 1 };
      final int[] CUBIC_DERIV_IDX = { 0, 0, 1, 1 };
      final int[] CUBIC_PNT_IDX = { 0, 1, 0, 1 };

      int nTermsComputed = 1;
      coeffs[0] = 1;
      for (int i = 0; i < nOrder; i++) {

         if (interpTypes[i] == InterpType.CUBIC) {

            for (int j = 0; j < 4; j++) {
               basis[j] =
                  cubicBasis(CUBIC_DERIV_IDX[j], CUBIC_PNT_IDX[j], xi[i]);
            }

            index_kron(
               derivIdxs, nTermsComputed, CUBIC_DERIV_IDX, 4, dimensions[i],
               derivIdxs);
            index_kron(pntIdxs, nTermsComputed, CUBIC_PNT_IDX, 4, i, pntIdxs);
            nTermsComputed = kron(coeffs, nTermsComputed, basis, 4, coeffs);

         } else if (interpTypes[i] == InterpType.LINEAR) {

            for (int j = 0; j < 2; j++) {
               basis[j] =
                  linearBasis(LINEAR_DERIV_IDX[j], LINEAR_PNT_IDX[j], xi[i]);
            }

            index_kron(
               derivIdxs, nTermsComputed, LINEAR_DERIV_IDX, 2, i, derivIdxs);
            index_kron(pntIdxs, nTermsComputed, LINEAR_PNT_IDX, 2, i, pntIdxs);
            nTermsComputed = kron(coeffs, nTermsComputed, basis, 2, coeffs);
         }
      }

      // now that we have all indices and coefficients, we can actually build
      // the point
      for (int i = 0; i < nTerms; i++) {
//         xxx++;
//         System.out.println(xxx);
//         if (xxx==3) {
//            System.out.println("breaking...");
//         }
         out.scaledAdd(coeffs[i]
            * scaleFactors[nodeIdxs[pntIdxs[i]]][derivIdxs[i]],
            new Point3d(
               nodes[nodeIdxs[pntIdxs[i]]].getDerivativeCoordinate(
                  versions[nodeIdxs[pntIdxs[i]]], derivIdxs[i]))
            );
      }
      return out;
   }

   public static double arclength(int arcDimIdx, double min, double max,
      double[] xi, double step,
      InterpType[] interpTypes, ExNode[] nodes, int[] versions,
      double[][] scaleFactors, int[] nodeIdxs, int[] dimensions) {

      Point3d p1;
      Point3d p2;

      double arc = 0;
      double xiTest[] = Arrays.copyOf(xi, xi.length);
      int nSteps = (int)((max - min) / step);

      xiTest[arcDimIdx] = min;
      p1 =
         interp(
            xiTest, interpTypes, nodes, versions, scaleFactors, nodeIdxs,
            dimensions);

      for (int i = 0; i < nSteps; i++) {
         xiTest[arcDimIdx] = min + (i + 1) * step;
         p2 =
            interp(
               xiTest, interpTypes, nodes, versions, scaleFactors, nodeIdxs,
               dimensions);
         p1.sub(p2); // displacement vector
         arc += p1.norm(); // add distance
         p1.set(p2); // copy p2 to p1 for next iteration
      }

      // last piece
      xiTest[arcDimIdx] = max;
      p2 =
         interp(
            xiTest, interpTypes, nodes, versions, scaleFactors, nodeIdxs,
            dimensions);
      p1.sub(p2);
      arc += p1.norm();

      return arc;

   }

   // interpolate by arclength
   public static Point3d interpArc(double[] xi, InterpType[] interpTypes,
      ExNode[] nodes, int[] versions,
      double[][] scaleFactors, int[] nodeIdxs, int[] dimensions) {

      double[] xiOut = new double[xi.length];
      return interpArc(
         xi, defaultThreshold, defaultStepSize, interpTypes, nodes, versions,
         scaleFactors, nodeIdxs, dimensions, xiOut);

   }

   public static Point3d interpArc(double[] xi, InterpType[] interpTypes,
      ExNode[] nodes, int[] versions,
      double[][] scaleFactors, int[] nodeIdxs, int[] dimensions,
      double[] xiOut) {
      return interpArc(
         xi, defaultThreshold, defaultStepSize, interpTypes, nodes, versions,
         scaleFactors, nodeIdxs, dimensions, xiOut);
   }

   // interpolate by arclength
   public static Point3d interpArc(double[] xi, double threshold,
      double maxStep,
      InterpType[] interpTypes, ExNode[] nodes, int[] versions,
      double[][] scaleFactors, int[] nodeIdxs, int[] dimensions,
      double[] xiOut) {

      Point3d p1;
      Point3d p2 = new Point3d();
      Point3d disp = new Point3d();
      double step;
      boolean switchDir = false;

      // copy xi
      for (int i = 0; i < xi.length; i++) {
         xiOut[i] = xi[i];
      }

      for (int i = 0; i < xi.length; i++) {

         // determine total arclength
         double totalArc = arclength(i, 0, 1, xiOut, maxStep, interpTypes,
            nodes, versions, scaleFactors, nodeIdxs, dimensions);

         // if on collapsed dimension, don't adjust
         if (totalArc < 1e-8) {
            continue;
         }

         step = maxStep;
         double arc = 0;
         double nextArc = 0;

         // compute arclength fraction

         if (xi[i] > 0.5) {
            switchDir = true;
            xiOut[i] = 1;
            arc = totalArc;
         } else {
            switchDir = false;
            xiOut[i] = 0;
         }

         p1 =
            interp(
               xiOut, interpTypes, nodes, versions, scaleFactors, nodeIdxs,
               dimensions);

         while (Math.abs(arc / totalArc - xi[i]) > threshold) {

            if (switchDir) {
               xiOut[i] -= step;
            } else {
               xiOut[i] += step;
            }
            if (xiOut[i] > 1) {
               xiOut[i] = 1;
            }
            p2 =
               interp(
                  xiOut, interpTypes, nodes, versions, scaleFactors, nodeIdxs,
                  dimensions);
            disp.set(p1);
            disp.sub(p2);

            if (switchDir) {
               nextArc = arc - disp.norm();
            } else {
               nextArc = arc + disp.norm();
            }

            if ((nextArc / totalArc > xi[i]) && !switchDir) {
               xiOut[i] -= step; // return xi
               step = step / 2;
            } else if (switchDir && (nextArc / totalArc < xi[i])) {
               xiOut[i] += step; // return xi
               step = step / 2;
            } else {
               p1.set(p2);
               arc = nextArc;
            } // end checking if we've overshot

         } // done finding good xiOut[i]

      } // done looping through xi

      return interp(
         xiOut, interpTypes, nodes, versions, scaleFactors, nodeIdxs,
         dimensions); // output Point

   }

   public static void setDefaultThreshold(double threshold) {
      defaultThreshold = threshold;
   }

   public static void setDefaultStepSize(double stepSize) {
      defaultStepSize = stepSize;
   }

}
