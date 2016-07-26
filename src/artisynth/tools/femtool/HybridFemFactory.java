package artisynth.tools.femtool;

import java.util.Hashtable;

import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;

public class HybridFemFactory  {
   
   public static FemModel3d createHexRoundedBeam(
      double l, double w_in, double w_margin, int nl, int n_in, int n_margin) {
      int[] nodelist0 = new int[4 * n_margin * (nl + 1)];
      int[] nodelist1 = new int[4 * n_margin * (nl + 1)];

      int numCenterNodes = (nl + 1) * (n_in + 1) * (n_in + 1);
      int numMarginNodes = (nl + 1) * (n_in + 1) * n_margin;

      int count = 0;
      // Top Right
      for (int i = 0; i <= nl; i++) {
         for (int j = 0; j < n_margin; j++) {
            // Top
            nodelist0[count] = numCenterNodes + j * (nl + 1) * (n_in + 1) + i;
            // Right
            nodelist1[count] =
               numCenterNodes + 2 * numMarginNodes + (nl + 1) * n_margin * n_in
                  + (n_margin - j - 1) * (nl + 1) + i;
            count++;
         }
      }
      // Top Left
      for (int i = 0; i <= nl; i++) {
         for (int j = 0; j < n_margin; j++) {
            // Top
            nodelist0[count] =
               numCenterNodes + j * (nl + 1) * (n_in + 1) + (nl + 1) * n_in + i;
            // Left
            nodelist1[count] =
               numCenterNodes + 3 * numMarginNodes + (nl + 1) * n_margin * n_in
                  + j * (nl + 1) + i;
            count++;
         }
      }

      // Bottom Right
      for (int i = 0; i <= nl; i++) {
         for (int j = 0; j < n_margin; j++) {
            // Bottom
            nodelist0[count] =
               numCenterNodes + numMarginNodes + j * (nl + 1) * (n_in + 1) + i;
            // Right
            nodelist1[count] =
               numCenterNodes + 2 * numMarginNodes + j * (nl + 1) + i;
            count++;
         }
      }
      // Bottom Left
      for (int i = 0; i <= nl; i++) {
         for (int j = 0; j < n_margin; j++) {
            // Bottom
            nodelist0[count] =
               numCenterNodes + numMarginNodes + (nl + 1) * n_in + j * (nl + 1)
                  * (n_in + 1) + i;
            // Left
            nodelist1[count] =
               numCenterNodes + 3 * numMarginNodes + (n_margin - j - 1)
                  * (nl + 1) + i;
            count++;
         }
      }

      return mergeNodes(
         createHexCross(l, w_in, w_margin, nl, n_in, n_margin), nodelist0,
         nodelist1);
   }

   public static FemModel3d createHexCross(
      double l, double w_in, double w_margin, int nl, int n_in, int n_margin) {
      FemModel3d[] fem_list = new FemModel3d[5];

      double dis = w_in / 2 + w_margin / 2;

      // Create the center square
      FemModel3d centerFem = new FemModel3d();
      FemFactory.createHexGrid(centerFem, l, w_in, w_in, nl, n_in, n_in);
      fem_list[0] = centerFem;

      if (n_margin > 0) {
         // Create the top beam
         FemModel3d topFem = new FemModel3d();
         FemFactory
            .createHexGrid(topFem, l, w_in, w_margin, nl, n_in, n_margin);
         topFem.transformGeometry(new RigidTransform3d(0, 0, dis));
         fem_list[1] = topFem;

         // Create the bottom beam
         FemModel3d bottomFem = new FemModel3d();
         FemFactory.createHexGrid(
            bottomFem, l, w_in, w_margin, nl, n_in, n_margin);
         bottomFem.transformGeometry(new RigidTransform3d(0, 0, -dis));
         fem_list[2] = bottomFem;

         // Create the right beam
         FemModel3d rightFem = new FemModel3d();
         FemFactory.createHexGrid(
            rightFem, l, w_margin, w_in, nl, n_margin, n_in);
         rightFem.transformGeometry(new RigidTransform3d(0, -dis, 0));
         fem_list[3] = rightFem;
         // Create the left beam
         FemModel3d leftFem = new FemModel3d();
         FemFactory.createHexGrid(
            leftFem, l, w_margin, w_in, nl, n_margin, n_in);
         leftFem.transformGeometry(new RigidTransform3d(0, dis, 0));
         fem_list[4] = leftFem;

         for (int i = 1; i < 5; i++) {
            FemFactory.addFem(fem_list[0], fem_list[i], 0.0001);
         }
      }

      return fem_list[0];
   }

   public static FemModel3d createHexCylinder(
      double l, double r_in, double r_out, int nl, int n_in, int n_margin) {

      if (r_out < r_in) {
         System.out
            .println("Error: in createHexCylinder(), r_out cannot be less than r_in");
         return null;
      }

      double w_in = Math.sqrt(2) * r_in;
      double w_out = Math.sqrt(2) * r_out;
      double w_margin = w_out - w_in;

      FemModel3d fem =
         createHexRoundedBeam(l, w_in, w_margin, nl, n_in, n_margin);
      projToCylinder(fem, nl, n_in, n_margin);

      return fem;
   }

   public static FemModel3d createHybridMuscleEllipsoid(
      double l, double r_in, double r_out, int nl, int n_in, int n_margin) {
      int n_end = n_margin + ((int)n_in / 2);
      return createHybridMuscleEllipsoid(
         l, r_in, r_out, nl, n_in, n_margin, n_end);
   }

   public static FemModel3d createHybridMuscleEllipsoid(
      double l, double r_in, double r_out, int nl, int n_in, int n_margin,
      int n_end) {
      FemModel3d fem =
         createHybridMuscleEllipsoid_KeepNodes(
            l, r_in, r_out, nl, n_in, n_margin, n_end);
      return removeUnwantedNodes(fem);
   }

   public static FemModel3d createHybridMuscleEllipsoid_KeepNodes(
      double l, double r_in, double r_out, int nl, int n_in, int n_margin,
      int n_end) {

      FemModel3d fem = createHexCylinder(l, r_in, r_out, nl, n_in, n_margin);

      // Reduce end elements
      int[] nodes;
      int[] nodesTemp = new int[0];
      int[] nodesAll = new int[0];
      for (int j = 0; j < n_end; j++) {
         for (int i = 0; i < n_end - j; i++) {
            nodes = roundedBeamCircleNodes(j, i, nl, n_in, n_margin);
            nodesAll = new int[nodes.length + nodesTemp.length];
            System.arraycopy(nodes, 0, nodesAll, 0, nodes.length);
            System.arraycopy(
               nodesTemp, 0, nodesAll, nodes.length, nodesTemp.length);
            nodesTemp = nodesAll;

            nodes = roundedBeamCircleNodes(j, nl - i, nl, n_in, n_margin);
            nodesAll = new int[nodes.length + nodesTemp.length];
            System.arraycopy(nodes, 0, nodesAll, 0, nodes.length);
            System.arraycopy(
               nodesTemp, 0, nodesAll, nodes.length, nodesTemp.length);
            nodesTemp = nodesAll;
         }
      }
      fem = reudceHexToWedgeAndTet(fem, nodesAll);

      return fem;
   }

   public static void projToCylinder(FemModel3d fem, int nl, int n_in,
      int n_margin) {
      int[] nodes;

      double y, z;
      double r;
      double radStep;
      double radStart;
      FemNode3d node;
      for (int layer = 0; layer < (n_margin + ((int)n_in / 2) + n_in % 2); layer++) {
         for (int i = 0; i <= nl; i++) {
            nodes = roundedBeamCircleNodes(layer, i, nl, n_in, n_margin);
            int count = 0;
            r =
               Math.sqrt(Math.pow(fem.getNode(nodes[0]).getPosition().z, 2)
                  + Math.pow(fem.getNode(nodes[0]).getPosition().z, 2));
            if (layer < n_margin) {
               radStep = Math.toRadians(90 / n_in);
               // Top
               radStart = Math.toRadians(-45);
               for (int j = 0; j < n_in + 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart + radStep * j);
                  z = r * Math.cos(radStart + radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
               // Bottom
               radStart = Math.toRadians(225);
               for (int j = 0; j < n_in + 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart - radStep * j);
                  z = r * Math.cos(radStart - radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
               // Right
               radStart = Math.toRadians(225) + radStep;
               for (int j = 0; j < n_in - 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart + radStep * j);
                  z = r * Math.cos(radStart + radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
               // Left
               radStart = Math.toRadians(135) - radStep;
               for (int j = 0; j < n_in - 1; j++) {
                  node = fem.getNode(nodes[count]);
                  y = r * Math.sin(radStart - radStep * j);
                  z = r * Math.cos(radStart - radStep * j);
                  node.setPosition(node.getPosition().x, y, z);
                  count++;
               }
            } else {
               int layer_in = layer - n_margin;
               if ((n_in - 2 * layer_in) > 0) {
                  radStep = Math.toRadians(90 / (n_in - 2 * layer_in));
                  // Top
                  radStart = Math.toRadians(-45);
                  for (int j = 0; j <= n_in - 2 * layer_in; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart + radStep * j);
                     z = r * Math.cos(radStart + radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
                  // Bottom
                  radStart = Math.toRadians(225);
                  for (int j = 0; j <= n_in - 2 * layer_in; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart - radStep * j);
                     z = r * Math.cos(radStart - radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
                  // Right
                  radStart = Math.toRadians(225) + radStep;
                  for (int j = 0; j < n_in - 2 * layer_in - 1; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart + radStep * j);
                     z = r * Math.cos(radStart + radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
                  // Left
                  radStart = Math.toRadians(135) - radStep;
                  for (int j = 0; j < n_in - 2 * layer_in - 1; j++) {
                     node = fem.getNode(nodes[count]);
                     y = r * Math.sin(radStart - radStep * j);
                     z = r * Math.cos(radStart - radStep * j);
                     node.setPosition(node.getPosition().x, y, z);
                     count++;
                  }
               }
            }
         }
      }
      fem.resetRestPosition();
   }

   public static int[] roundedBeamCircleNodes(int layer, int depth, int nl,
      int n_in, int n_margin) {
      int numCenterNodes = (nl + 1) * (n_in + 1) * (n_in + 1);
      int numMarginNodes_TopBottom = (nl + 1) * (n_in + 1) * n_margin;
      int numMarginNodes_LeftRight = (nl + 1) * (n_in - 1) * n_margin;
      int[] nodesToReduceList = null;

      if (layer < n_margin) {
         nodesToReduceList = new int[4 * n_in];
         int count = 0;
         // Top
         for (int j = 0; j < n_in + 1; j++) {
            nodesToReduceList[count] =
               numCenterNodes + (nl + 1) * (n_in + 1) * (n_margin - 1 - layer)
                  + j * (nl + 1) + depth;
            count++;
         }
         // Bottom
         for (int j = n_in + 1; j < 2 * (n_in + 1); j++) {
            nodesToReduceList[count] =
               numCenterNodes + (nl + 1) * (n_in + 1) * (n_margin - 1 + layer)
                  + j * (nl + 1) + depth;
            count++;
         }
         // Right
         for (int j = 0; j < n_in - 1; j++) {
            nodesToReduceList[count] =
               numCenterNodes + 2 * numMarginNodes_TopBottom + (nl + 1) * layer
                  + j * (nl + 1) * n_margin + depth;
            count++;
         }
         // Left
         for (int j = 0; j < n_in - 1; j++) {
            nodesToReduceList[count] =
               numCenterNodes + 2 * numMarginNodes_TopBottom
                  + numMarginNodes_LeftRight + (nl + 1)
                  * (n_margin - 1 - layer) + j * (nl + 1) * n_margin + depth;
            count++;
         }
      } else if (layer < (n_margin + ((int)n_in / 2) + n_in % 2)) {
         int layer_in = layer - n_margin;
         nodesToReduceList = new int[4 * (n_in - 2 * layer_in)];
         int count = 0;
         // Top
         for (int j = layer_in; j <= n_in - layer_in; j++) {
            nodesToReduceList[count] =
               (n_in - layer_in) * (nl + 1) * (n_in + 1) + j * (nl + 1) + depth;
            count++;
         }
         // Bottom
         for (int j = layer_in; j <= n_in - layer_in; j++) {
            nodesToReduceList[count] =
               layer_in * (nl + 1) * (n_in + 1) + j * (nl + 1) + depth;
            count++;
         }
         // Right
         for (int j = layer_in + 1; j < n_in - layer_in; j++) {
            nodesToReduceList[count] =
               layer_in * (nl + 1) + j * (nl + 1) * (n_in + 1) + depth;
            count++;
         }
         // Left
         for (int j = layer_in + 1; j < n_in - layer_in; j++) {
            nodesToReduceList[count] =
               (n_in - layer_in) * (nl + 1) + j * (nl + 1) * (n_in + 1) + depth;
            count++;
         }
      } else if (n_in % 2 == 0) {
         nodesToReduceList = new int[1];
         nodesToReduceList[0] =
            (nl + 1) * (n_in + 1) * (n_in / 2) + (nl + 1) * (n_in / 2) + depth;
      }

      return nodesToReduceList;
   }

   public static FemModel3d removeUnwantedNodes(FemModel3d fem) {
      FemModel3d femResult = new FemModel3d();

      Hashtable<String,FemNode3d> hashedNodes =
         new Hashtable<String,FemNode3d>();
      Hashtable<String,FemNode3d> hashedNodesKeep =
         new Hashtable<String,FemNode3d>();

      String key;
      for (FemNode3d n : fem.getNodes()) {
         key =
            String.format(
               "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
               n.getPosition().z);
         FemNode3d n_duplicated = new FemNode3d(n.getPosition());
         hashedNodes.put(key, n_duplicated);
      }

      for (FemElement3d el : fem.getElements()) {
         if (el.numNodes() == 8) {
            FemNode3d[] el_nodes = new FemNode3d[8];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               key =
                  String.format(
                     "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
                     n.getPosition().z);
               el_nodes[i] = hashedNodes.get(key);
               i++;
               if (!hashedNodesKeep.containsKey(key)) {
                  hashedNodesKeep.put(key, hashedNodes.get(key));
               }
            }
            femResult.addElement(new HexElement(el_nodes));
         } else if (el.numNodes() == 6) {
            FemNode3d[] el_nodes = new FemNode3d[6];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               key =
                  String.format(
                     "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
                     n.getPosition().z);
               el_nodes[i] = hashedNodes.get(key);
               i++;
               if (!hashedNodesKeep.containsKey(key)) {
                  hashedNodesKeep.put(key, hashedNodes.get(key));
               }
            }
            femResult.addElement(new WedgeElement(el_nodes));
         } else if (el.numNodes() == 4) {
            FemNode3d[] el_nodes = new FemNode3d[4];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               key =
                  String.format(
                     "%.5f_%.5f_%.5f", n.getPosition().x, n.getPosition().y,
                     n.getPosition().z);
               el_nodes[i] = hashedNodes.get(key);
               i++;
               if (!hashedNodesKeep.containsKey(key)) {
                  hashedNodesKeep.put(key, hashedNodes.get(key));
               }
            }
            femResult.addElement(new TetElement(
               el_nodes[0], el_nodes[1], el_nodes[2], el_nodes[3]));
         }
      }

      for (FemNode3d n : hashedNodesKeep.values()) {
         femResult.addNode(n);
      }

      return femResult;
   }

   public static FemModel3d reudceHexToWedgeAndTet(FemModel3d fem,
      int[] nodesToReduceList) {
      FemModel3d femReduced = new FemModel3d();
      Hashtable<Integer,FemNode3d> hashedUndesiredNodes =
         new Hashtable<Integer,FemNode3d>();
      Hashtable<Integer,FemNode3d> hashedMapping =
         new Hashtable<Integer,FemNode3d>();

      for (int i : nodesToReduceList) {
         hashedUndesiredNodes.put(i, fem.getNode(i));
      }

      for (FemNode3d n : fem.getNodes()) {
         // if(!hashedUndesiredNodes.containsKey(n.getNumber())) {
         FemNode3d n_duplicated = new FemNode3d(n.getPosition());
         hashedMapping.put(n.getNumber(), n_duplicated);
         femReduced.addNode(n_duplicated);
         // }
      }
      int count, i;
      int[] nIdx = new int[8];
      int[] rIdx = new int[8];
      for (FemElement3d el : fem.getElements()) {
         count = 0;
         i = 0;
         for (FemNode3d n : el.getNodes()) {
            nIdx[i] = n.getNumber();
            if (hashedUndesiredNodes.containsKey(n.getNumber())) {
               rIdx[count] = i;
               count++;
            }
            i++;
         }

         if (count == 2 && el.numNodes() == 8) {
            FemNode3d[] el_nodes = new FemNode3d[6];
            String caseCount = rIdx[0] + "_" + rIdx[1];

            if (caseCount.matches("0_1")) {
               int[] order = { 2, 6, 5, 3, 7, 4 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("1_2")) {
               int[] order = { 3, 7, 6, 0, 4, 5 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("2_3")) {
               int[] order = { 4, 7, 0, 5, 6, 1 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("4_5")) {
               int[] order = { 6, 1, 2, 7, 0, 3 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("5_6")) {
               int[] order = { 7, 2, 3, 4, 1, 0 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("6_7")) {
               int[] order = { 0, 4, 3, 1, 5, 2 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("0_3")) {
               int[] order = { 2, 7, 6, 1, 4, 5 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("4_7")) {
               int[] order = { 5, 0, 1, 6, 3, 2 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("0_4")) {
               int[] order = { 5, 6, 7, 1, 2, 3 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("1_5")) {
               int[] order = { 4, 6, 7, 0, 2, 3 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("2_6")) {
               int[] order = { 7, 4, 5, 3, 0, 1 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else if (caseCount.matches("3_7")) {
               int[] order = { 0, 2, 1, 4, 6, 5 };
               for (int x = 0; x < 6; x++) {
                  el_nodes[x] = hashedMapping.get(nIdx[order[x]]);
               }
               femReduced.addElement(new WedgeElement(el_nodes));
            } else {
               FemNode3d[] el_HexNodes = new FemNode3d[8];
               int k = 0;
               for (int t : nIdx) {
                  el_HexNodes[k] = hashedMapping.get(t);
                  if (hashedMapping.get(t) == null)
                     System.out.println("Can't find node: " + t);
                  k++;
               }
               femReduced.addElement(new HexElement(el_HexNodes));
            }
         } else if (count == 3 && el.numNodes() == 8) {
            FemNode3d[] el_nodes1 = new FemNode3d[4];
            FemNode3d[] el_nodes2 = new FemNode3d[4];
            String caseCount = rIdx[0] + "_" + rIdx[1] + "_" + rIdx[2];

            if (caseCount.matches("0_1_2")) {
               int[] order = { 3, 5, 7, 6, 7, 5, 3, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_2_3")) {
               int[] order = { 5, 7, 1, 6, 1, 7, 5, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_4_5")) {
               int[] order = { 1, 7, 2, 6, 2, 7, 1, 3 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_1_5")) {
               int[] order = { 4, 7, 2, 6, 2, 7, 4, 3 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_3_7")) {
               int[] order = { 4, 2, 5, 6, 5, 2, 4, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_4_7")) {
               int[] order = { 3, 2, 5, 6, 5, 2, 3, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("0_3_4")) {
               int[] order = { 7, 1, 6, 2, 6, 1, 7, 5 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_2_5")) {
               int[] order = { 7, 0, 6, 3, 6, 0, 7, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));

            } else if (caseCount.matches("1_2_3")) {
               int[] order = { 0, 6, 4, 7, 4, 6, 0, 5 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_2_6")) {
               int[] order = { 3, 5, 4, 7, 4, 5, 3, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_4_5")) {
               int[] order = { 0, 6, 3, 2, 3, 6, 0, 7 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("1_5_6")) {
               int[] order = { 4, 3, 2, 7, 2, 3, 4, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_3_7")) {
               int[] order = { 6, 0, 5, 1, 5, 0, 6, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_6_7")) {
               int[] order = { 3, 0, 5, 1, 5, 0, 3, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_5_6")) {
               int[] order = { 1, 7, 0, 3, 0, 7, 1, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("2_6_7")) {
               int[] order = { 3, 0, 5, 1, 5, 0, 3, 4 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("3_4_7")) {
               int[] order = { 6, 0, 1, 2, 1, 0, 6, 5 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("3_6_7")) {
               int[] order = { 2, 1, 4, 5, 4, 1, 2, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("4_5_7")) {
               int[] order = { 6, 0, 2, 3, 2, 0, 6, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("5_6_7")) {
               int[] order = { 4, 0, 2, 3, 2, 0, 4, 1 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("4_5_6")) {
               int[] order = { 7, 3, 1, 2, 1, 3, 7, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else if (caseCount.matches("4_6_7")) {
               int[] order = { 5, 3, 1, 2, 1, 3, 5, 0 };
               for (int x = 0; x < 4; x++) {
                  el_nodes1[x] = hashedMapping.get(nIdx[order[x]]);
                  el_nodes2[x] = hashedMapping.get(nIdx[order[x + 4]]);
               }
               femReduced.addElement(new TetElement(
                  el_nodes1[0], el_nodes1[1], el_nodes1[2], el_nodes1[3]));
               femReduced.addElement(new TetElement(
                  el_nodes2[0], el_nodes2[1], el_nodes2[2], el_nodes2[3]));
            } else {
               FemNode3d[] el_HexNodes = new FemNode3d[8];
               int k = 0;
               for (int t : nIdx) {
                  el_HexNodes[k] = hashedMapping.get(t);
                  if (hashedMapping.get(t) == null)
                     System.out.println("Can't find node: " + t);
                  k++;
               }
               femReduced.addElement(new HexElement(el_HexNodes));
            }

         } else {
            if (el.numNodes() == 6 && count < 5) {
               FemNode3d[] el_WedgeNodes = new FemNode3d[6];
               for (int k = 0; k < 6; k++) {
                  el_WedgeNodes[k] = hashedMapping.get(nIdx[k]);
               }
               femReduced.addElement(new WedgeElement(el_WedgeNodes));
            } else if (el.numNodes() == 4 && count < 3) {
               FemNode3d[] el_TetNodes = new FemNode3d[4];
               for (int k = 0; k < 4; k++) {
                  el_TetNodes[k] = hashedMapping.get(nIdx[k]);
               }
               femReduced.addElement(new TetElement(
                  el_TetNodes[0],
                  el_TetNodes[1],
                  el_TetNodes[2],
                  el_TetNodes[3]
                  ));
            } else if (count < 4) {
               FemNode3d[] el_HexNodes = new FemNode3d[8];
               int k = 0;
               for (int t : nIdx) {
                  el_HexNodes[k] = hashedMapping.get(t);
                  k++;
               }
               femReduced.addElement(new HexElement(el_HexNodes));
            }
         }
      }

      return femReduced;
   }

   public static FemModel3d mergeNodes(FemModel3d fem, int[] nodelist0,
      int[] nodelist1) {
      if (nodelist0.length != nodelist1.length) {
         System.out
            .println("Warning: Cannot merge nodes, array size mismatch.");
         return null;
      }
      FemNode3d[] femNodelist0 = new FemNode3d[nodelist0.length];
      FemNode3d[] femNodelist1 = new FemNode3d[nodelist1.length];

      for (int i = 0; i < nodelist0.length; i++) {
         femNodelist0[i] = fem.getNode(nodelist0[i]);
         femNodelist1[i] = fem.getNode(nodelist1[i]);
      }
      return mergeNodes(fem, femNodelist0, femNodelist1);
   }

   public static FemModel3d mergeNodes(FemModel3d fem, FemNode3d[] nodelist0,
      FemNode3d[] nodelist1) {
      if (nodelist0.length != nodelist1.length) {
         System.out
            .println("Warning: Cannot merge nodes, array size mismatch.");
         return null;
      }

      FemModel3d femMerged = new FemModel3d();
      Hashtable<Integer,Integer> hashedPairs = new Hashtable<Integer,Integer>();
      Hashtable<Integer,Integer> hashedPairs_reverse =
         new Hashtable<Integer,Integer>();
      Hashtable<Integer,Point3d> hashedPairPosition =
         new Hashtable<Integer,Point3d>();
      Hashtable<Integer,FemNode3d> hashedMapping =
         new Hashtable<Integer,FemNode3d>();

      FemNode3d n0, n1;

      for (int i = 0; i < nodelist0.length; i++) {
         n0 = nodelist0[i];
         n1 = nodelist1[i];
         hashedPairs.put(n0.getNumber(), n1.getNumber());
         hashedPairs_reverse.put(n1.getNumber(), n0.getNumber());
         Point3d p = new Point3d();
         p.add(n0.getPosition(), n1.getPosition());
         p.scale(0.5);
         hashedPairPosition.put(n0.getNumber(), p);
      }

      for (FemNode3d n : fem.getNodes()) {
         if (!hashedPairs_reverse.containsKey(n.getNumber())) {
            FemNode3d n_duplicated;
            if (hashedPairs.containsKey(n.getNumber())) {
               n_duplicated =
                  new FemNode3d(hashedPairPosition.get(n.getNumber()));
               hashedMapping.put(hashedPairs.get(n.getNumber()), n_duplicated);
            } else {
               n_duplicated = new FemNode3d(n.getPosition());
            }
            femMerged.addNode(n_duplicated);
            hashedMapping.put(n.getNumber(), n_duplicated);

         }
      }

      for (FemElement3d el : fem.getElements()) {
         if (el.numNodes() == 8) {
            FemNode3d[] el_nodes = new FemNode3d[8];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               el_nodes[i] = hashedMapping.get(n.getNumber());
               i++;
            }
            femMerged.addElement(new HexElement(el_nodes));
         } else if (el.numNodes() == 6) {
            FemNode3d[] el_nodes = new FemNode3d[6];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               el_nodes[i] = hashedMapping.get(n.getNumber());
               i++;
            }
            femMerged.addElement(new WedgeElement(el_nodes));
         } else if (el.numNodes() == 4) {
            FemNode3d[] el_nodes = new FemNode3d[4];
            int i = 0;
            for (FemNode3d n : el.getNodes()) {
               el_nodes[i] = hashedMapping.get(n.getNumber());
               i++;
            }
            femMerged.addElement(new TetElement(
               el_nodes[0], el_nodes[1], el_nodes[2], el_nodes[3]));
         }
      }

      return femMerged;
   }
   
}
