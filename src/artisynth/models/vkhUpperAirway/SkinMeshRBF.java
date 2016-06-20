package artisynth.models.vkhUpperAirway;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.MeshBase;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.PolarDecomposition3d;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix.WriteFormat;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SparseMatrixNd;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.Renderer;
import maspack.render.RenderList;
import maspack.solvers.PardisoSolver;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.MeshInfo;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.util.*;

public class SkinMeshRBF extends MeshComponent {
   public enum ControlTypes {RIGID, FEM, NEIGHBOURS};
   
   ArrayList<Point3d> myBaseBndPnts;
   Point3d[] myBasePnts;
   
   RadialBasisFunction myRBFx;
   RadialBasisFunction myRBFy;
   RadialBasisFunction myRBFz;
   ArrayList<FemNode3d> myBndNodes;
   
   double rbfMaxR;

   public SkinMeshRBF () {
      super();
      myBasePnts = new Point3d[0];
      myBaseBndPnts = new ArrayList<Point3d>();

      myRBFx = new RadialBasisFunction();
      myRBFy = new RadialBasisFunction();
      myRBFz = new RadialBasisFunction();
      
      myBndNodes = new ArrayList<FemNode3d>();
   }

   public SkinMeshRBF (MeshBase mesh) {
      this();
      setMesh (mesh);
      mesh.setFixed (false);
      rbfMaxR = Double.POSITIVE_INFINITY;
      
   }

   // public void updateBounds (Point3d pmin, Point3d pmax) {
   //    myMeshInfo.getMesh().updateBounds (pmin, pmax);
   // }
   
   public void setRBFMaxRadius (double r){
      rbfMaxR = r;
   }
   
   private void setBasePnts() {
      MeshBase mesh = getMesh();
      int numVtxs = mesh.numVertices();
      myBasePnts = new Point3d[numVtxs];
      for (int i=0; i<numVtxs; i++) {
         Vertex3d vtx = mesh.getVertices().get(i);
         myBasePnts[i] = new Point3d(vtx.getPosition());
      }
   }
   
   /*
    * creates a new copy of points defining the boundary
    */
   private void setBaseBoundaryPoints(ArrayList<FemNode3d> nodes) {
      myBaseBndPnts.clear ();
      for (int i=0; i<nodes.size (); i++) {
         myBaseBndPnts.add (new Point3d(nodes.get (i).getPosition ()));
      }
      
      myRBFx = new RadialBasisFunction ();
      myRBFx.setMaxRadius (rbfMaxR);
      myRBFy = new RadialBasisFunction ();
      myRBFy.setMaxRadius (rbfMaxR);
      myRBFz = new RadialBasisFunction ();
      myRBFz.setMaxRadius (rbfMaxR);
      
      myRBFx.setBoundaryPoints (myBaseBndPnts);
      myRBFy.setBoundaryPoints (myBaseBndPnts);
      myRBFz.setBoundaryPoints (myBaseBndPnts);
      
   }

   // public MeshBase getMesh() {
   //    // need to check for myMeshInfo != null because this method gets
   //    // by setDefaultInputs in the super constructor.
   //    if (myMeshInfo != null) {
   //       return myMeshInfo.getMesh();
   //    }
   //    else {
   //       return null;
   //    }
   // }
   
   public void computeWeights (ArrayList<FemModel3d> fems, double maxDist) {
      setBasePnts();
      findNearestNodes (fems, null, maxDist);
      //recomputeRBFWeights();
   }
   
   //public void computeRBFWeights (ArrayList<FemNode3d> nodes, ArrayList<RigidBody> rigids, double maxDist) {
   public void findNearestNodes (ArrayList<FemModel3d> fems, ArrayList<RigidBody> rigids, double maxDist) {
      //boolean airwayDebug = true;
      //ArrayList<Vertex3d> verts = myMeshInfo.getMesh().getVertices ();
      //System.out.println("Mesh has " + verts.size() + " verts");
      myBndNodes.clear();
      
      PolygonalMesh mesh = (PolygonalMesh)getMesh();
      Point3d newLoc = new Point3d();
      Vector2d coords = new Vector2d();
      BVFeatureQuery query = new BVFeatureQuery();

      /*
       * add all nodes which are closer to airway than maxDist
       */
      int inCount=0;
      int outCount=0;
      for (FemModel3d fem : fems) {
         // this could works very similarly to the code that adds
         // marker points into a mesh
         for( int i=0; i<fem.getNodes ().size (); i++ ) {
            FemNode3d n = fem.getNode (i);
            if( fem.getSurfaceVertex (n) == null ) {
               inCount++;
               continue;
            }
            outCount++;
            query.nearestFaceToPoint (newLoc, coords, mesh, n.getPosition ());
            double d = n.getPosition().distance (newLoc);
            if (d < maxDist) {
               myBndNodes.add (n);
            }
         }
      }
      
      System.out.println("in: "+inCount+" out: "+outCount);
      setBaseBoundaryPoints (myBndNodes);
      
   }
   
   public void recomputeRBFWeights () {
      double[] dxs, dys, dzs;
      dxs = new double[myBndNodes.size ()];
      dys = new double[myBndNodes.size ()];
      dzs = new double[myBndNodes.size ()];
      for (int i=0; i<myBaseBndPnts.size (); i++) {
         dxs[i] = myBndNodes.get(i).getPosition ().x - myBaseBndPnts.get (i).x;
         dys[i] = myBndNodes.get(i).getPosition ().y - myBaseBndPnts.get (i).y;
         dzs[i] = myBndNodes.get(i).getPosition ().z - myBaseBndPnts.get (i).z;
      }
      //System.out.println("u[0] = "+dxs[4]+" v[0] = "+dys[4]+" w[0] = "+dzs[4]);
      
      myRBFx.computeWeights (dxs);
      myRBFy.computeWeights (dys);
      myRBFz.computeWeights (dzs);
   }
   
   public void updateSlavePos () {
      MeshBase mesh = myMeshInfo.getMesh();
      int numVerts = mesh.numVertices ();
      
      recomputeRBFWeights();
      
      /*
       * Update the positions of all vertices depending
       * on their controller
       */
      double dx,dy,dz;
      Vector3d dX = new Vector3d();
      for( int i=0; i<numVerts; i++ ) {
         Vertex3d vtx = mesh.getVertices().get(i);
         
         Point3d basePnt = myBasePnts[i];
         
         dx = myRBFx.interpolate (basePnt);
         dy = myRBFy.interpolate (basePnt);
         dz = myRBFz.interpolate (basePnt);
//         if( i == 0 )
//            System.out.println("dx: "+dx+" dy: "+dy+" dz: "+dz);
         dX.set (dx,dy,dz);
         dX.add (basePnt);
//         dX.set(basePnt);
         
         vtx.getPosition ().set (dX);
         
      }
      
      mesh.notifyVertexPositionsModified();
      
   }
   
   @Override
   public void prerender (RenderList list) {
      MeshBase mesh = getMesh();
      if (mesh != null) {
         mesh.prerender (myRenderProps);
      }
   }
   
   @Override
   public void render(Renderer renderer, int flags) {
      MeshBase mesh = getMesh();
      if (mesh != null) {
         // System.out.println ("drawMesh " + getName());
         flags |= isSelected() ? Renderer.HIGHLIGHT : 0;
         mesh.render (renderer, myRenderProps, flags);
      }
   }

   // public void prerender (RenderList list) {
   //    myMeshInfo.prerender();
   // }

   public void scaleDistance (double s) {
      super.scaleDistance (s);
      for (int i=0; i<myBasePnts.length; i++) {
         myBasePnts[i].scale (s);
      }
   }  

   public void transformGeometry (
      AffineTransform3dBase X, PolarDecomposition3d pd,
      Map<TransformableGeometry,Boolean> transformSet, int flags) {
      
      if ((flags & TransformableGeometry.TG_SIMULATING) != 0) {
         return;
      }
      for (int i=0; i<myBasePnts.length; i++) {
         if (X instanceof RigidTransform3d) {
            myBasePnts[i].transform ((RigidTransform3d)X);
         }
         else {
            myBasePnts[i].transform (X);
         }
      }
      updateSlavePos();
   }
  
   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens) 
      throws IOException {
      return super.scanItem (rtok, tokens);
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

   }

   public void write (PrintWriter pw, NumberFormat fmt, Object ref)
      throws IOException {
      dowrite (pw, fmt, ref);
   }

}

/*
 * Implements a radial basis function with linear polynomial term
 * see De Boer 2007 (two papers) for details
 */
class RadialBasisFunction {
   double[] alpha; // Radial basis coefficients
   double[] beta; // polynomial coefficeints
   
   PardisoSolver pardiso;
   SparseMatrixNd myA;
   
   ArrayList<Point3d> myBoundaryNodes;
   
   double myMaxRad;
   double myMaxRadInv;
   
   RadialBasisFunction(){
      alpha = new double[0];
      beta = new double[4]; // polynomial must be linear
      myBoundaryNodes = new ArrayList<Point3d>();
      pardiso = null;
      myA = null;
      myMaxRad = Double.POSITIVE_INFINITY;
   }
   
   void setMaxRadius (double r) {
      myMaxRad = r;
      myMaxRadInv = 1/r;
   }
   
   void setBoundaryPoints (ArrayList<Point3d> boundaryPoints) {
      alpha = new double[boundaryPoints.size ()];
      myBoundaryNodes.clear();
      
      for (int i=0; i<boundaryPoints.size (); i++) {
         myBoundaryNodes.add (boundaryPoints.get (i));
      }
      
   }
   
   /*
    * Implement some different basis functions and select here
    */
   public double getPhi( double r ) {
      
//      return getGTPSPhi(r);
      return getCPC2 (r);
   }
   
   /*
    * Global Thin-plate spline
    */
   double getGTPSPhi (double r) {
      if( r == 0 ) {
         return 0;
      }
      double phi = r*r*Math.log10 (r);
      return phi;
   }
   
   double getCPC2 (double r) {
      if (r >= myMaxRad) {
         return 0;
      }
      double xi = r/myMaxRad;
      //double val = Math.pow(1-xi,4)*(4*xi + 1); <-- this is quite slow!
      double val=(1-xi)*(1-xi)*(1-xi)*(1-xi)*(4*xi+1);
      
      
      return val;
   }
   
   void setupPardiso() {
      pardiso = new PardisoSolver();
//      pardiso.analyze (myA, myBoundaryNodes.size ()+4, Matrix.INDEFINITE );
      pardiso.analyze (myA, myBoundaryNodes.size ()+4, Matrix.SYMMETRIC ); 
   }
   
   /*
    * Get an interpolated value
    */
   public double interpolate (Point3d pos) {
      double val = 0;
      for (int i=0; i<myBoundaryNodes.size (); i++) {
         double r = pos.distance (myBoundaryNodes.get (i));
         val += alpha[i]*getPhi (r);
      }
      val += beta[0] + beta[1]*pos.x + beta[2]*pos.y + beta[3]*pos.z;
      
      return val;
   }
   
   /*
    * Solve the block matrix 
    * [ M    P ] [alpha]   [u]
    * [        ] [     ] = [ ]
    * [ P^T  0 ] [beta ]   [0]
    */
   public void computeWeights (double[] u) {
      int nNodes = myBoundaryNodes.size ();
      myA = new SparseMatrixNd( nNodes+4, nNodes+4);
      
      for( int i=0; i<myBoundaryNodes.size(); i++ ) {
         Point3d p1 = myBoundaryNodes.get (i);
         for( int j=i; j<myBoundaryNodes.size(); j++ ) {
            Point3d p2 = myBoundaryNodes.get (j);
            
            double phi = getPhi(p1.distance (p2));
            if (i == j) {
               myA.set(i,j,phi);
            }
            else if (phi != 0) {
               myA.set (i,j, phi);
               myA.set (j,i, phi);
            } 
         }
      }
      // set P and P^T
      for (int i=0; i<nNodes; i++) {
         myA.set(i, nNodes, 1);
         myA.set(i, nNodes+1, myBoundaryNodes.get (i).x);
         myA.set(i, nNodes+2, myBoundaryNodes.get (i).y);
         myA.set(i, nNodes+3, myBoundaryNodes.get (i).z);
         
         myA.set(nNodes, i, 1);
         myA.set(nNodes+1, i, myBoundaryNodes.get (i).x);
         myA.set(nNodes+2, i, myBoundaryNodes.get (i).y);
         myA.set(nNodes+3, i, myBoundaryNodes.get (i).z);
      }
      
      myA.set(nNodes,nNodes,0);
      myA.set(nNodes+1,nNodes+1,0);
      myA.set(nNodes+2,nNodes+2,0);
      myA.set(nNodes+3,nNodes+3,0);
      
      double[] b = new double[nNodes+4];
      double[] x = new double[nNodes+4];
      for( int i=0; i<nNodes; i++ ) {
         b[i] = u[i];
      }
      for( int i=0; i<4; i++ ) {
         b[nNodes+i] = 0;
      }
      try{
         if( myA.isSymmetric (1e-11) ){
            System.out.println("is symmetric!!");
         }
         PrintWriter pw = new PrintWriter("RBFMatrixAndSource.txt");
         myA.write (pw, new NumberFormat("%g"), WriteFormat.CRS);
         String source = new String();
         for (int i=0; i<nNodes+4; i++) {
            source = source + b[i] + " ";
         }
         pw.write("\n\n");
         pw.write (source);
         pw.flush ();
         pw.close ();
         
      } catch (IOException e){
         e.printStackTrace ();
      }
      if (pardiso == null) {
         setupPardiso();
      }
      pardiso.factor();
      pardiso.solve (x, b);
      for( int i=0; i<nNodes; i++) {
         alpha[i] = x[i];
      }
      for( int i=0; i<4; i++) {
         beta[i] = x[nNodes+i];
      }
   }
}
