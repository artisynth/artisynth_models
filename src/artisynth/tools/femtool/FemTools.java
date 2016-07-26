package artisynth.tools.femtool;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.modelbase.RenderableComponentList;

public class FemTools {

   /**
    * Computes the centre of mass of a 3D FEM element
    * 
    * @param elem
    * The FEM element of which to compute the CoM
    * @return The centre of mass of the element
    */
   public static Point3d getCentreOfMass (FemElement3d elem) {

      Point3d CoM = new Point3d (0, 0, 0);

      FemNode3d[] myNodes = elem.getNodes ();

      // HEX
      if (elem instanceof HexElement) {
         for (FemNode3d node : elem.getNodes ()) {
            CoM.add (node.getPosition ());
         }
         CoM.scale (1.0 / 8.0);
      }

      // PYRAMID
      else if (elem instanceof PyramidElement) {
         for (int i = 0; i < 4; i++) {
            CoM.add (myNodes[i].getPosition ());
         }
         CoM.scaledAdd (4.0 / 3.0, myNodes[4].getPosition ());
         CoM.scale (3.0 / 16.0);
      }

      // WEDGE
      else if (elem instanceof WedgeElement) {
         for (int i = 0; i < 6; i++) {
            CoM.add (myNodes[i].getPosition ());
         }
         CoM.scale (1.0 / 6.0);
      }

      // TET
      else if (elem instanceof TetElement) {
         for (int i = 0; i < 4; i++) {
            CoM.add (myNodes[i].getPosition ());
         }
         CoM.scale (1.0 / 4.0);
      }

      // OTHER
      else {
         System.out
            .println ("Warning: CoM function not implemented for class '"
            + elem.getClass ().getName ()
            + "', computing simple average of nodes");
         for (int i = 0; i < myNodes.length; i++) {
            CoM.add (myNodes[i].getPosition ());
         }
         CoM.scale (1.0 / myNodes.length);
      }

      return CoM;

   }

   /**
    * Computes the center of mass for a FEM model
    * 
    * @param model
    * The FEM model of which to compute the centre of mass
    * @return the centre of mass
    */
   public static Point3d getCentreOfMass (FemModel3d model) {

      Point3d CoM = new Point3d (0, 0, 0);

      RenderableComponentList<FemElement3d> elemList = model.getElements ();
      for (FemElement3d elem : elemList) {
         CoM.add (getCentreOfMass (elem));
      }
      CoM.scale (1.0 / elemList.size ());

      return CoM;
   }

   // Replaced with IntegrationPoint3d.create (John Lloyd, August 2012).
//   /**
//    * Creates an "Integration Point" object for a particular element, allowing
//    * for a computation of deformation gradient
//    * 
//    * @param elem  FEM element to use
//    * @param s0    coordinates within element
//    * @param s1
//    * @param s2
//    * @param w     weight to use
//    * @return
//    */
//   public static IntegrationPoint3d createIntegrationPoint (FemElement3d elem,
//	 double s0, double s1, double s2, double w) {
//
//      int nnodes = elem.numNodes();
//      int npvals = elem.numPressureVals();
//      
//      Vector3d coords = new Vector3d();
//      Vector3d dNds = new Vector3d();
//      VectorNd shapeVals = new VectorNd(nnodes);
//      VectorNd pressureShapeVals = new VectorNd(npvals);
//
//      IntegrationPoint3d pnt =
//	    new IntegrationPoint3d (nnodes, npvals, s0, s1, s2, w);
//      coords.set (s0, s1, s2);
//      for (int i=0; i<nnodes; i++) {
//	 shapeVals.set (i, elem.getN (i, coords));
//	 elem.getdNds (dNds, i, coords);
//	 pnt.setShapeGrad (i, dNds);
//      }
//      for (int i=0; i<npvals; i++) {
//         pressureShapeVals.set (i, elem.getH (i, coords));
//      }
//      pnt.setShapeVals (shapeVals);
//      pnt.setPressureShapeVals (pressureShapeVals);
//      return pnt;
//   }
}
