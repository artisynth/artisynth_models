package artisynth.models.registration.correspondences;

import java.util.TreeSet;

import maspack.geometry.AABBTree;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVFeatureQuery.NearestKCollector;
import maspack.geometry.BVFeatureQuery.ObjectDistanceCalculator;
import maspack.geometry.BVFeatureQuery.ObjectDistanceEntry;
import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Boundable;
import maspack.geometry.Feature;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.util.DynamicArray;

/**
 * Uses GMM-based weights, with option of selecting K nearest model points to each target point
 */
public class TargetGMMMeshCorrespondence extends MeshCorrespondenceComputer {

   public static double DEFAULT_W = 0.1;     // 10% of points are noise
   public static int DEFAULT_K =  30;        // use 30 target points
   public static boolean DEFAULT_AUTO_VARIANCE = true;  // estimate variance 
   public static boolean DEFAULT_PERP = false;  // ensure normal consistency

   // properties
   static PropertyList myProps = new PropertyList (TargetGMMMeshCorrespondence.class, MeshCorrespondenceComputer.class);
   static {
      myProps.add ("noiseFraction", "probability of noise", DEFAULT_W, "[0,1]");
      myProps.add ("nearestK", "Number of nearest target points to consider", DEFAULT_K);
      myProps.add ("perpendicular", "Ensure normal consistency by scaling w by cosine angle", DEFAULT_PERP);
      myProps.add ("variance", "GMM variance", 0);
      myProps.add ("autoVariance isAutoVariance setAutoVariance", "automatic estimation of variance (otherwise explicit)", DEFAULT_AUTO_VARIANCE);
      
      MeshCorrespondenceComputer.registerSubClass (TargetGMMMeshCorrespondence.class);
   }
   
   // model/target
   MeshBase target;
   BVTree modelTree;
   MeshBase model;
   
   double w;         // noise weight
   int klimit;       // nearest target points
   boolean perp;
   
   private static class ProbEntry {
      int n;         // target vertex index
      double d2;     // distance squared
      double p;      // probability
   }
   
   private static class VertexEntry {
      DynamicArray<ProbEntry> P = new DynamicArray<ProbEntry>(ProbEntry.class); // limit
      Point3d x = new Point3d();        // target
      Vector3d n = new Vector3d();      // target normal
      double psum = 0;                  // weight
   }
   
   VertexEntry[] vinfo;
   
   double sigma2;          // variance
   boolean computeSigma2;  // whether to compute or assume explicit
   
   public TargetGMMMeshCorrespondence() {
      this.klimit = DEFAULT_K;
      this.w = DEFAULT_W;
      this.computeSigma2 = DEFAULT_AUTO_VARIANCE;
      this.perp = DEFAULT_PERP;
   }
   
   @Override
   public void initialize (MeshBase model, MeshBase target) {
      
      this.target = target;
      this.model = model;
      
      this.vinfo = new VertexEntry[model.numVertices ()];
      for (int i=0; i<vinfo.length; ++i) {
         vinfo[i] = new VertexEntry();
      }
            
      modelTree = new AABBTree(model);
      modelTree.setBvhToWorld (model.getMeshToWorld ());
      modelTree.setMaxLeafElements (4);
      modelTree.setMargin (model.getRadius ()*1e-8);
      modelTree.build (model.getVertices ());
      
      // initialize distance/variance/probabilities
      updateDistances ();
      if (computeSigma2) {
         initializeVariance ();
      }
      updateProbabilities ();
      updateCorrespondences ();
      
   }
   
   @Override
   public double getCorrespondence (Point3d x, Vertex3d mvtx) {
      int m = mvtx.getIndex ();
      if (x != null) {
         x.set (vinfo[m].x);
      }
      double w = vinfo[m].psum / model.numVertices ();
      if (perp) {
         Vector3d n = new Vector3d();
         mvtx.computeWorldNormal (n);
         w = w*vinfo[m].n.dot (n);
         if (w < 0) {
            w = 0;
         }
      }
      
      return w;
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
   public void apply(double t0, double t1) {
      // update estimate of sigma given last correspondence, then update new distances and probabilities
      if (computeSigma2) {
         updateVariance ();
      }
      updateDistances ();
      updateProbabilities ();
      updateCorrespondences ();
   }
   
   /**
    * Finds distances from source to target (either nearest K, or all vertex-to-vertex)
    */
   public void updateDistances() {
      if (klimit <= 0 || klimit >= target.numVertices ()) {
         updateAllDistances ();
      } else {
         updateKDistances (klimit);
      }
   }
   
   /**
    * Consider all target points
    */
   private void updateAllDistances() {
      
      // loop through each model vertex
      for (Vertex3d vtx : model.getVertices ()) {
         int vidx = vtx.getIndex ();
         VertexEntry ventry = vinfo[vidx];
      
         // maybe create P row
         if (ventry.P.size () != target.numVertices ()) {
            ventry.P = new DynamicArray<ProbEntry>(ProbEntry.class, target.numVertices ());
            for (int i=0; i<ventry.P.size(); ++i) {
               ventry.P.set (i,  new ProbEntry());
            }
         }
         
         // compute distances
         int nidx = 0;         
         for (Vertex3d tvtx : target.getVertices ()) {
            ProbEntry pentry = ventry.P.get (nidx);
            pentry.d2 = vtx.getWorldPoint ().distanceSquared (tvtx.getWorldPoint ());
            pentry.n = tvtx.getIndex ();
            pentry.p = 0;
            ++nidx;
         }
      }
   }
   
   /**
    * Number of nearest model points to consider.  A zero indicates to use all model points for each target point.
    * 
    * @return size of k-nearest neighbor search
    */
   public int getNearestK() {
      return klimit;
   }
   
   /**
    * Sets the number of nearest model points to consider.  This can help reduce the complexity
    * from M*N (where M is number of model points, N is number of target points) to N*K*log(M).
    * Set to 0 to consider all points.
    * 
    * @param k size of k-nearest neighbor search
    */
   public void setNearestK(int k) {
      if (k < 0) {
         k = 0;
      }
      this.klimit = k;
   }
   
   /**
    * Probability of sample being considered noise
    * @return noise fraction
    */
   public double getNoiseFraction() {
      return w;
   }
   
   /**
    * Probability of sample being considered noise
    * @param w noise fraction
    */
   public void setNoiseFraction(double w) {
      if (w < 0) {
         w = 0;
      } else if (w > 1) {
         w = 1;
      }
      this.w = w;
      updateDistances ();
      updateProbabilities ();
   }

   /**
    * GMM variance, equal for all model points
    * @return variance
    */
   public double getVariance() {
      return sigma2;
   }
   
   /**
    * GMM variance, equal for all model points
    * @param v new variance
    */
   public void setVariance(double v) {
      if (v <= 0) {
         initializeVariance ();
      } else {
         sigma2 = v;
      }
      updateDistances ();
      updateProbabilities ();
   }
   
   /**
    * Sets whether the variance should be automatically estimated from the data.
    * Otherwise, we rely on explicitly setting the variance
    * @param set 
    */
   public void setAutoVariance(boolean set) {
      computeSigma2 = set;
   }
   
   /**
    * Determines whether variance is automatically computed or if it is explicit
    * @return true if computed, false is explicit
    */
   public boolean isAutoVariance() {
      return computeSigma2;
   }
   
   /**
    * Find nearest face with consistent normal (i.e. divides distance by dot product of normals)
    */
   private static class PointVertexNormalDistanceCalculator implements ObjectDistanceCalculator {

      Point3d myPnt;
      Vector3d myNrml;
      Point3d myNearest;
      Vertex3d myFeature;
      double myDist;

      public PointVertexNormalDistanceCalculator () {
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
         if (e instanceof Vertex3d) {
            Vector3d nrm = new Vector3d();
            Vertex3d ff = (Vertex3d)e;
            ff.nearestPoint(nearest, myPnt);
            ff.computeWorldNormal (nrm);
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
         else {
            return -1;
         }
      }

      public Vertex3d nearestObject () {
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
    * Finds the K nearest features in the bounding-volume hierarchy to a point
    * and normal in world coordinates
    * 
    * The resulting tree is ordered by distance (nearest to furthest).  There may be fewer than K
    * entries if the bounding tree contains fewer than K elements, and there may be more than K
    * entries if the last size()-k+1 elements all have exactly the same distance
    *  
    * @param bvh bounding volume hierarchy
    * @param k number of elements to return
    * @param pnt point (world-coordinates)
    * @return set of nearest features
    */
   public TreeSet<ObjectDistanceEntry<Feature>> nearestKVertexNormalToPoint(BVFeatureQuery bvq, BVTree bvh, int k, Point3d pnt, Vector3d nrm) {
      
      PointVertexNormalDistanceCalculator calc = new PointVertexNormalDistanceCalculator();
      calc.setPoint(pnt, nrm, bvh.getBvhToWorld ());
      
      NearestKCollector<Feature> collector = new NearestKCollector<> (k);
      bvq.nearestObjects (bvh, calc, collector);

     return collector.nearest();      
   }
   
   /**
    * Only consider nearest K target points
    */
   private void updateKDistances(int k) {
      // maybe update tree
      modelTree.setBvhToWorld (model.getMeshToWorld ());
      modelTree.update ();
      
      BVFeatureQuery bvq = new BVFeatureQuery ();
      
      // clear all distances
      for (VertexEntry ve : vinfo) {
         ve.P.clear();
         ve.n.setZero ();
         ve.x.setZero ();
         ve.psum = 0;
      }
      
      // loop through each target vertex
      for (Vertex3d vtx : target.getVertices ()) {
         int nidx = vtx.getIndex ();
         
         // find k nearest model vertices
         TreeSet<ObjectDistanceEntry<Feature>> nearest;
         if (perp) {
            Vector3d nrm = new Vector3d();
            vtx.computeWorldNormal (nrm);
            nearest = nearestKVertexNormalToPoint (bvq, modelTree, k, vtx.getWorldPoint (), nrm);
         } else {
            nearest = bvq.nearestKFeaturesToPoint (modelTree, k, vtx.getWorldPoint ());
         }
         
         // compute distances
         for (ObjectDistanceEntry<Feature> ode : nearest) {
            int m = ((Vertex3d)ode.getObject ()).getIndex ();
            ProbEntry pentry = new ProbEntry ();
            pentry.d2 = ode.getDistance ();
            pentry.d2 = pentry.d2*pentry.d2;  // square
            pentry.n = nidx;
            pentry.p = 0;  // assume equal probability at first
            vinfo[m].P.add (pentry); // add as a probability entry
         }
      }
   }

   /**
    * Estimates an initial variance based on equal probabilities
    */
   public void initializeVariance() {
      // use approximately 1/3 of sum of squared distances
      // \sigma^2 = \frac{1}{3MN}\sum_{m,n=1}^{M,N}\|x_n- y_m\|^2
      sigma2 = 0;
      int count = 0;
      for (VertexEntry ventry : vinfo) {
         for (ProbEntry Pmn : ventry.P) {
            sigma2 += Pmn.d2;
            ++count;
         }
      }
      sigma2 = sigma2/count/3;
   }
   
   /**
    * Computes an updated variance based on current correspondence probabilities
    */
   public void updateVariance() {
      // \sigma^2 = \frac{1}{3N_P}\sum_{m,n=1}^{M,N}P_{mn}\|x_n- y_m\|^2
      sigma2 = 0;
      double Np = 0;
      for (VertexEntry ventry : vinfo) {
         for (ProbEntry Pmn : ventry.P) {
            Np += Pmn.p;
            sigma2 += Pmn.p*Pmn.d2;
         }
      }
      sigma2 = sigma2/Np/3;
      if (Double.isNaN (sigma2)) {
         sigma2 = 0;
      }
   }
   
   /**
    * Updates GMM probabilities based on current variance and distances
    */
   public void updateProbabilities() {

      // constant for correspondence to "noise", no longer 1/N if only K nearest?
      int N = target.numVertices ();
      double c = Math.pow(2*Math.PI*sigma2, 3.0/2.0)*w/(1-w+1e-12)*model.numVertices ();
      
      // first pass compute exponential and column sums
      double[] csums = new double[target.numVertices ()];
      for (VertexEntry ventry : vinfo) {
         for (ProbEntry Pmn : ventry.P) {
            Pmn.p = Math.exp (-0.5*Pmn.d2/sigma2);
            csums[Pmn.n] += Pmn.p;  // column sum
         }
      }
      
      // second pass to compute probability
      for (VertexEntry ventry : vinfo) {
         ventry.psum = 0;
         for (ProbEntry Pmn : ventry.P) {
            Pmn.p = Pmn.p / (csums[Pmn.n] + c / N); //Prow.length);
            if (Double.isNaN (Pmn.p)) {
               Pmn.p = 0;
            }
            ventry.psum += Pmn.p;   // sum across row
         }
      }
   }
   
   public MatrixNd getProbabilityMatrix() {
      MatrixNd Pmat = new MatrixNd(model.numVertices (), target.numVertices ());
      
      for (int m=0; m<vinfo.length; ++m) {
         for (int i=0; i<vinfo[m].P.size(); ++i) {
            Pmat.add (m, vinfo[m].P.get (i).n, vinfo[m].P.get(i).p);
         }
      }
      
      return Pmat;
   }
   
   public void updateCorrespondences() {
      
      Vector3d n = new Vector3d();
      for (VertexEntry ventry : vinfo) {
         ventry.x.setZero ();
         ventry.n.setZero ();
         
         if (ventry.psum > 0) {
            for (ProbEntry pmn : ventry.P) {
               ventry.x.scaledAdd (pmn.p, target.getVertex (pmn.n).getWorldPoint ());
               target.getVertex (pmn.n).computeWorldNormal (n);
               ventry.n.scaledAdd (pmn.p, n);
            }
            ventry.x.scale (1.0/ventry.psum);
            ventry.n.normalize ();
         }
      }
      
   }

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

}
