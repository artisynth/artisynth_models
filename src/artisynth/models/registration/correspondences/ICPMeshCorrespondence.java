package artisynth.models.registration.correspondences;

import artisynth.models.registration.weights.ConstantWeightFunction;
import artisynth.models.registration.weights.RegistrationWeightFunction;
import maspack.geometry.AABBTree;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVFeatureQuery.ObjectDistanceCalculator;
import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Boundable;
import maspack.geometry.Face;
import maspack.geometry.Feature;
import maspack.geometry.MeshBase;
import maspack.geometry.OBBTree;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 * Uses ICP-based weights, with option of selecting a weighting function
 */
public class ICPMeshCorrespondence extends MeshCorrespondenceComputer {

   static boolean DEFAULT_PERP = false;

   // properties
   static PropertyList myProps = new PropertyList (ICPMeshCorrespondence.class, MeshCorrespondenceComputer.class);
   static {
      myProps.add ("weightFunction", "ICP weight function", createDefaultWeightFunction());
      myProps.add ("perpendicular", "Ensure normal consistency by scaling w by cosine angle", DEFAULT_PERP);
   }

   // model/target
   MeshBase target;
   BVTree targetTree;
   MeshBase model;

   RegistrationWeightFunction weightFunc;
   boolean perp;

   private static class VertexEntry {
      Feature nearfeature; // nearest feature
      Point3d nearPnt;     // nearest point
      double w;            // weight 

      VertexEntry() {
         nearfeature = null;
         nearPnt = new Point3d();
         w = 1.0;
      }
   }

   VertexEntry[] vinfo;    // force information

   private static RegistrationWeightFunction createDefaultWeightFunction() {
      return new ConstantWeightFunction ();
   }

   public ICPMeshCorrespondence() {
      this.perp = DEFAULT_PERP;
      this.weightFunc = createDefaultWeightFunction ();
   }

   @Override
   public void initialize(MeshBase model, MeshBase target) {

      this.target = target;
      this.model = model;

      this.vinfo = new VertexEntry[model.numVertices ()];
      for (int i=0; i<vinfo.length; ++i) {
         vinfo[i] = new VertexEntry();
      }

      if (target.isFixed ()) {
         targetTree = new OBBTree(target);
      } else {
         targetTree = new AABBTree(target);
      }
      //      targetTree.setBvhToWorld (target.getMeshToWorld ());
      //      targetTree.setMaxLeafElements (4);
      //      targetTree.setMargin (target.getRadius ()*1e-8);
      //      targetTree.build (target.getVertices ());

      // initialize distance/weights
      updateDistancesAndWeights ();      
   }

   /**
    * Sets whether to scale correspondence by dot product of normals
    * @param set new state
    */
   public void setPerpendicular(boolean set) {
      this.perp = set;
   }

   /**
    * Gets whether the correspondence weight is scaled by dot product of normals
    * @return perpendicular state
    */
   public boolean getPerpendicular() {
      return perp;
   }


   @Override
   public double getCorrespondence (Point3d x, Vertex3d mvtx) {
      int m = mvtx.getIndex ();
      if (x != null) {
         x.set (vinfo[m].nearPnt);
      }
      return vinfo[m].w / model.numVertices ();
   }

   @Override
   public void apply(double t0, double t1) {
      // update distances
      updateDistancesAndWeights();
   }

   /**
    * Find nearest face with consistent normal (i.e. divides distance by dot product of normals)
    */
   private static class PointFeatureNormalDistanceCalculator implements ObjectDistanceCalculator {

      Point3d myPnt;
      Vector3d myNrml;
      Point3d myNearest;
      Feature myFeature;
      double myDist;

      public PointFeatureNormalDistanceCalculator () {
         myPnt = new Point3d();
         myNrml = new Vector3d();
         myNearest = new Point3d();
         reset();
      }

      @Override
      public void reset() {
         myFeature = null;
         myDist = Double.POSITIVE_INFINITY;
      }

      public void setPoint (Point3d pnt, Vector3d nrm, RigidTransform3d XBvhToWorld) {
         if (XBvhToWorld == RigidTransform3d.IDENTITY) {
            myPnt.set (pnt);
            myNrml.set (nrm);
         }
         else {
            myPnt.inverseTransform (XBvhToWorld, pnt);
            myNrml.inverseTransform (XBvhToWorld, nrm);
         }
      }          

      @SuppressWarnings("unused")
      public void setPoint (Point3d pnt, Vector3d nrm) {
         myPnt.set (pnt);
         myNrml.set (nrm);
      }          

      public double nearestDistance (BVNode node) {
         return node.distanceToPoint (myPnt);
      }

      public double nearestDistance (Boundable e) {
         myFeature = null;
         Point3d nearest = new Point3d();

         Feature ff = (Feature)e;
         ff.nearestPoint(nearest, myPnt);

         Vector3d nrm = new Vector3d();
         if (ff instanceof Face) {
            Face face = (Face)ff;
            face.getWorldNormal (nrm);
         } else if (ff instanceof Vertex3d) {
            Vertex3d v = (Vertex3d)ff;
            v.computeWorldNormal (nrm);
         } else {
            nrm.set (myNrml);
         }

         double w = nrm.dot (myNrml);
         double d = nearest.distance(myPnt);
         if (w <= 0) {
            d = Double.POSITIVE_INFINITY;
         } else {
            d = d/w;
         }

         if (d < myDist) {
            myDist = d;
            myFeature = ff;
            myNearest.set(nearest);
         }
         return d;
      }

      public Feature nearestObject () {
         return myFeature;
      }

      @Override
      public double nearestDistance() {
         return myDist;
      }

      public Point3d nearestPoint() {
         return myNearest;
      }
   }

   /**
    * Finds the nearest feature in the bounding-volume hierarchy to a point
    * and normal in world coordinates
    * 
    * @param bvh bounding volume hierarchy
    * @param pnt point (world-coordinates)
    * @return set of nearest features
    */
   public Feature nearestFeatureNormalToPoint(BVFeatureQuery bvq, BVTree bvh, Point3d pnt, Vector3d nrm) {

      PointFeatureNormalDistanceCalculator calc = new PointFeatureNormalDistanceCalculator();
      calc.setPoint(pnt, nrm, bvh.getBvhToWorld ());
      bvq.nearestObject (bvh, calc);
      
      return calc.nearestObject ();      
   }

   /**
    * Computes distances to target and updates vertex areas on source
    */
   public void updateDistancesAndWeights() {
      // maybe update tree
      targetTree.setBvhToWorld (target.getMeshToWorld ());
      if (!target.isFixed ()) {
         targetTree.update ();
      }

      BVFeatureQuery bvq = new BVFeatureQuery ();
      Vector3d snrm = new Vector3d();
      Vector3d tnrm = new Vector3d();



      // loop through each model vertex
      for (Vertex3d vtx : model.getVertices ()) {
         int vidx = vtx.getIndex ();

         // find nearest feature
         VertexEntry dentry = vinfo[vidx];

         if (perp) {
            Vector3d nrm = new Vector3d();
            vtx.computeWorldNormal (nrm);
            dentry.nearfeature = nearestFeatureNormalToPoint (bvq, targetTree, vtx.getWorldPoint (), nrm);
         } else {
            dentry.nearfeature = bvq.nearestFeatureToPoint (dentry.nearPnt, targetTree, vtx.getWorldPoint ());
         }

         if (dentry.nearfeature != null) {
            vtx.computeWorldNormal (snrm);
            if (dentry.nearfeature instanceof Face) {
               Face face = (Face)dentry.nearfeature;
               face.getWorldNormal (tnrm);
            } else if (dentry.nearfeature instanceof Vertex3d) {
               Vertex3d v = (Vertex3d)dentry.nearfeature;
               v.computeWorldNormal (tnrm);
            } else {
               tnrm.setZero();
            }

            dentry.w = weightFunc.computeWeight (vtx.getWorldPoint (), snrm, dentry.nearPnt, tnrm);
         } else {
            dentry.w = 0;
         }
      }
   }

   /**
    * The weight function computes a weight to apply to the surface of the source during registration
    * @return the weight function
    */
   public RegistrationWeightFunction getWeightFunction() {
      if (weightFunc == null) {
         weightFunc = createDefaultWeightFunction();
      }
      return weightFunc;
   }

   /**
    * The weight function computes a weight to apply to the surface of the source during registration
    * @param pfunc
    */
   public void setWeightFunction(RegistrationWeightFunction pfunc) {
      this.weightFunc = pfunc;
   }


   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

}
