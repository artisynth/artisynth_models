package artisynth.models.vkhUpperAirway;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Deque;
import java.util.Map;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.HalfEdge;
import maspack.geometry.MeshBase;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.matrix.PolarDecomposition3d;
import maspack.render.Renderer;
import maspack.render.RenderList;
import maspack.util.IndentingPrintWriter;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.MeshInfo;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.CompositeComponentBase;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.util.*;

public class SkinMeshMulti extends MeshComponent {
   public enum ControlTypes {RIGID, FEM, NEIGHBOURS};

   RigidTransform3d[] myBasePoses;
   Point3d[] myBasePnts;
   
   ArrayList<RigidBody> myRigidBodies;
   //ArrayList<FemModel3d> myFems;
   ArrayList<FemNode3d> myFemNodes;
   ArrayList<Vertex3d> myNeighbours;
   
   double[] myRigidWeights;
   double[] myFemWeights;
   double[] myNeighbourWeights;
   
   int[] myNumFemWeights;
   int[] myNumNeighbourWeights;
   
   ControlTypes[] myControllerType;
   
   public SkinMeshMulti () {
      super();
      myBasePnts = new Point3d[0];
      myBasePoses = new RigidTransform3d[0];
      
      myRigidBodies = new ArrayList<RigidBody>();
      //myFems = new ArrayList<FemModel3d>();
      myFemNodes = new ArrayList<FemNode3d>();
      myNeighbours = new ArrayList<Vertex3d>();
      
      myRigidWeights = new double[0];
      myFemWeights = new double[0];
      myNeighbourWeights = new double[0];
      
   }

   public SkinMeshMulti (MeshBase mesh) {
      this();
      setMesh (mesh);
      mesh.setFixed (false);
      int size = mesh.numVertices ();
      
      myControllerType = new ControlTypes[size];
      myNumFemWeights = new int[size];
      myNumNeighbourWeights = new int[size];
   }

   // public void updateBounds (Point3d pmin, Point3d pmax) {
   //    myMeshInfo.getMesh().updateBounds (pmin, pmax);
   // }
   
   public void setMeshControllerTypes( ControlTypes [] types ) {
      myControllerType = Arrays.copyOf(types, types.length);
   }
   
   /*
    * Rigid operations
    */
   public ArrayList<RigidBody> getAttachedBodies() {
      return myRigidBodies;
   }
   
   private void setBasePoses() {
      int numBods = myRigidBodies.size();      
      myBasePoses = new RigidTransform3d[numBods];
      for (int j=0; j<numBods; j++) {
         myBasePoses[j] = new RigidTransform3d(myRigidBodies.get(j).getPose());
      }
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

   public void setRigidBodies (ArrayList<RigidBody> bodies) {
      myRigidBodies.clear();
      for (int i=0; i<bodies.size (); i++) {
         myRigidBodies.add (bodies.get(i));
      }
      setBasePoses();
   }
   
   public void setRigidWeights (ArrayList<Double> weights) {
      myRigidWeights = new double[weights.size()];
      for( int i=0; i<weights.size(); i++ ) {
         myRigidWeights[i] = weights.get(i);
      }
      setBasePoses(); // reset the base poses, just in case
      setBasePnts();      
   }
   
   /*
    * Fem operations
    */
   public void setFemNodes (ArrayList<FemNode3d> fems ) {
      myFemNodes.clear();
      for (int i=0; i<fems.size (); i++) {
         myFemNodes.add(fems.get(i));
      }
   }
   
   public void setFemWeights (ArrayList<Double> weights) {
      myFemWeights = new double[weights.size()];
      for( int i=0; i<weights.size(); i++ ) {
         myFemWeights[i] = weights.get(i);
      }
      
   }
   
   public void setNumFemWeights(int [] numFemWeights) {
      myNumFemWeights = Arrays.copyOf(numFemWeights, numFemWeights.length);
   }
   
   /*
    * Neighbour operations
    */
   public void setNeighbours (ArrayList<Vertex3d> verts ) {
      myNeighbours.clear ();
      for( int i=0; i<verts.size(); i++ ) {
         myNeighbours.add (verts.get(i));
      }
   }
   
   public void setNeighbourWeights (ArrayList<Double> weights) {
      myNeighbourWeights = new double[weights.size()];
      for( int i=0; i<weights.size(); i++ ) {
         myNeighbourWeights[i] = weights.get(i);
      }
   }
   
   public void setNumNeighbourWeights(int [] numNeiWeights) {
      myNumNeighbourWeights = Arrays.copyOf(numNeiWeights, numNeiWeights.length);
   }
   
   public void computeWeights (ArrayList<FemModel3d> fems, ArrayList<RigidBody> rigids, double maxDist, double reduceTol) {
      boolean airwayDebug = true;
      ArrayList<Vertex3d> verts = getMesh().getVertices ();
      System.out.println("Mesh has " + verts.size() + " verts");
      
      
      //ArrayList<RigidBody> rigids = new ArrayList<RigidBody>();
      ArrayList<Double> rigidWeights = new ArrayList<Double>();
      ArrayList<FemNode3d> femNodes = new ArrayList<FemNode3d>();
      ArrayList<Double> femWeights = new ArrayList<Double>();
      ArrayList<Vertex3d> neiNodes = new ArrayList<Vertex3d>();
      ArrayList<Double> neiWeights = new ArrayList<Double>();
      
      int[] numFemWeightsArray = new int[verts.size ()];
      int[] numNeiWeightsArray = new int[verts.size ()];
      SkinMeshMulti.ControlTypes[] controlTypes = new SkinMeshMulti.ControlTypes[verts.size()];
      
      for (int i=0; i<verts.size(); i++) {
         // this could works very similarly to the code that adds
         // marker points into a mesh
         Vertex3d vtx = verts.get(i);
         
         int numFemWeights = getFemWeights (fems, vtx, maxDist, reduceTol, femWeights, femNodes);
         numFemWeightsArray[i] = numFemWeights;
         
         if (numFemWeights > 0) {  // ie we found one closer than maxDist
            controlTypes[i] = ControlTypes.FEM;
            numNeiWeightsArray[i] = 0;
         }
         else { 
            // We now search for a close rigid fem to attach to
            boolean foundRigid;
            vtx = verts.get(i);
            foundRigid = getRigidWeights (rigids, vtx, maxDist, rigidWeights);
            if (foundRigid) {
               controlTypes[i] = ControlTypes.RIGID;
               numNeiWeightsArray[i] = 0;
            }
            else {
               // else this node is floating, and should be a weighted sum of its neighbours
               controlTypes[i] = SkinMeshMulti.ControlTypes.NEIGHBOURS;
               int numNeighbours = getNeighbourWeights(vtx, neiWeights, neiNodes);
               numNeiWeightsArray[i] = numNeighbours;
               if( numNeighbours == 0 ) {
                  System.out.println("Warning! no nearby neighbours for this node!");
               }
            }
         }
      }
      
      setMeshControllerTypes (controlTypes);
      
      setRigidBodies (rigids);
      setRigidWeights (rigidWeights);
      if (airwayDebug) 
         System.out.println(rigids.size() + " rigids, " + rigidWeights.size() + " rigidWeights.");
//      for (int i=0; i<rigidWeights.size(); i+=2) {
//         System.out.println( rigidWeights.get(i) + " " + rigidWeights.get(i+1));
//      }
      
      setFemNodes (femNodes);
      setFemWeights (femWeights);
      setNumFemWeights (numFemWeightsArray);
      if (airwayDebug) {
         int tmpsum = 0;
         for (int i=0; i<verts.size(); i++ ){
            tmpsum+= numFemWeightsArray[i];
         }
         System.out.println(femNodes.size() + " femNodes, " + femWeights.size() + " femWeights, " + tmpsum + " totalFemWeights.");
      }
      
      setNeighbours (neiNodes);
      setNeighbourWeights (neiWeights);
      setNumNeighbourWeights (numNeiWeightsArray);
      if (airwayDebug) {
         int tmpsum = 0;
         for (int i=0; i<verts.size(); i++ ){
            tmpsum+= numNeiWeightsArray[i];
         }
         System.out.println (neiNodes.size() + " neiNodes, " + neiWeights.size() + " neiWeights." + tmpsum + " totalNeiWeights.");
      }
      
      return;
   }
   
   int getNeighbourWeights (Vertex3d vtx, ArrayList<Double> weights, ArrayList<Vertex3d> nodes) {
      Iterator<HalfEdge> it = vtx.getIncidentHalfEdges ();
      int numNeighbours = 0;
      ArrayList<Vertex3d> neitmp = new ArrayList<Vertex3d>();
      
      while( it.hasNext() ) {
         HalfEdge he = it.next();
         if (he.getHead ().equals (vtx)) {
            if( neitmp.contains (he.getTail())) {
               System.out.println("We already have this vertex!");
            }
            neitmp.add( he.getTail());
            numNeighbours++;
         }
         else if (he.getTail ().equals(vtx)) {
            if( neitmp.contains (he.getHead())) {
               System.out.println("We already have this vertex!");
            }
            neitmp.add( he.getHead());
            numNeighbours++;
         }
         else {
            throw new IllegalArgumentException("Neither head nor tail connect to vertex!!");
         }
         
      }
      
      double tmpSum = 0;
      double [] ws = new double[numNeighbours];
      for( int i=0; i<numNeighbours; i++ ) {
         // try all equal weight
         ws[i] = 1;
         tmpSum += ws[i]; 
      }
      
      if( numNeighbours != neitmp.size() ) {
         System.out.println("Warning, size mismatch!");
      }
      
      for( int i=0; i<numNeighbours; i++ ) {
         nodes.add (neitmp.get (i));
         weights.add (ws[i] / tmpSum);
      }
      
      
      return numNeighbours;
      
   }
   
   int getFemWeights (ArrayList<FemModel3d> fems, Vertex3d vtx, double maxDist, double reduceTol, ArrayList<Double> weights, ArrayList<FemNode3d> nodes) {
      double minDist = maxDist;
      FemElement3dBase elem = null;
      //FemModel3d fem = null;
      Point3d newLoc = new Point3d();
      
      Vector3d vNorm = new Vector3d();
      Vector3d tmp = new Vector3d();
      vtx.computeNormal (vNorm);
      /*
       * search fem list for nearest element.
       * attach to fem which is closest (or 
       * which this vtx lies inside).
       */
      for(FemModel3d cfem : fems) {
         FemElement3dBase celem = cfem.findContainingElement (vtx.pnt);
         
         if (celem == null) {
            // won't use newLoc since we're not projecting vertex onto FEM
            celem = cfem.findNearestSurfaceElement (newLoc, vtx.pnt);
            tmp.sub (newLoc, vtx.pnt);
            double d = newLoc.distance (vtx.pnt);
//            if( minDist > d ) {//&& tmp.dot (vNorm)>=0) {
            if( minDist > d ) { //&& tmp.dot (vNorm)>=0) {
               minDist = d;
               elem = celem;
            }
         }
         else {
            // If the vtx is inside this fem, take it as found and leave
            elem = celem;
            break;
         }
         
      }
      // Basically use barycentric coordinates for weighting
      int numWeights = 0;
      if (elem != null) {
         VectorNd coords = new VectorNd (elem.numNodes());
         elem.getMarkerCoordinates (coords, null, vtx.pnt, false); 
         for (int k=0; k<coords.size(); k++) {
            if (Math.abs(coords.get(k)) >= reduceTol) {
               nodes.add (elem.getNodes()[k]);
               weights.add (coords.get(k)); 
               numWeights++;
            }
         }
      }
      
      return numWeights;
   
   }
   
   boolean getRigidWeights (ArrayList<RigidBody> rigids, Vertex3d vtx, double maxDist, ArrayList<Double> weights){
      Vector2d coords = new Vector2d();
      Point3d newLoc = new Point3d();
      RigidBody closest = null;
      double minDist = maxDist;
      double []dtmp = new double[rigids.size()];
      
      for (int i=0; i<rigids.size(); i++) {
         RigidBody rigid = rigids.get (i);
         BVFeatureQuery.getNearestFaceToPoint (
            newLoc, coords, rigid.getMesh(), vtx.getPosition());
         double d = vtx.getPosition().distance (newLoc);
         
         if( d < minDist ) {
            minDist = d;
            closest = rigid;
         }
         dtmp[i] = d;
         
      }
      double sumw=0;
      for (int i=0; i<rigids.size(); i++) {
         double d,w;
         d = dtmp[i];
         if(  d == minDist ) {
            w = 1;
         }
         else if( d < maxDist ) {
            w = (minDist*minDist)/(d*d);
         }
         else {
            w = 0;
         }
         dtmp[i] = w;
         sumw += w;
         
      }
      /*
       * Only add weights if we have at least one rigid within maxDist
       */
      if( closest != null ) {
         for (int i=0; i<rigids.size(); i++) {
            weights.add (dtmp[i]/sumw);
         }
      }
      
      /*
       * return true if we found a rigid closer than maxDist
       */
      return closest!=null;
   }

   public void updateSlavePos () {

      MeshBase mesh = getMesh();
      int numVerts = mesh.numVertices ();
      
      int rigidCount = 0;
      int femCount = 0;
      int neighbourCount = 0;
      
      Point3d pos = new Point3d();
      Point3d tmp = new Point3d();
      RigidTransform3d[] XBW = null;
      
      /*
       * Update the positions of all vertices depending
       * on their controller
       */
      for( int i=0; i<numVerts; i++ ) {
         Vertex3d vtx = mesh.getVertices().get(i);
         
         switch( myControllerType[i] ) {
            case RIGID: 
               pos.setZero();
               if( XBW == null ) {
                  /*
                   * Precompute all rigid transforms
                   */
                  XBW = new RigidTransform3d[myRigidBodies.size()];
                  for( int j=0; j<myRigidBodies.size(); j++ ) {
                     RigidTransform3d X = new RigidTransform3d (myRigidBodies.get(j).getPose());
                     X.mulInverseRight (X, myBasePoses[j]);
                     XBW[j] = X;
                  }
               }
               
               for (int j=0; j<myRigidBodies.size(); j++) {
                  double w = myRigidWeights[rigidCount+j];
                  tmp.transform (XBW[j], myBasePnts[i]); 
                  
                  pos.scaledAdd (w, tmp, pos);
                  
               }
               rigidCount += myRigidBodies.size();
               vtx.getPosition().set (pos);
               break;
            case FEM:
               pos.setZero();
               for( int j=0; j<myNumFemWeights[i]; j++ ) {
                  double w = myFemWeights[femCount+j];
                  FemNode3d n = myFemNodes.get(femCount+j);
                  Point3d np = n.getPosition ();
                  
                  pos.scaledAdd (w, np, pos);
                  
               }
               femCount += myNumFemWeights[i];
               vtx.getPosition().set (pos);
               break;
            case NEIGHBOURS:
               if( true ) 
                  break; // for now, do nothing
               // pos.setZero(); // holds displacement
               tmp.setZero();
               for( int j=0; j<myNumNeighbourWeights[i]; j++ ) {
                  double w = myNeighbourWeights[neighbourCount+j];
                  Vertex3d v = myNeighbours.get (neighbourCount+j);
                  
                  // Calculate weight based on an RBF?
                  double rbfVal = 0;//getRBFWeight(vtx.pnt.distance (v.pnt));
                  
                  tmp.set (v.pnt);
                  int vidx = mesh.getVertices().indexOf (v);
                  tmp.sub(myBasePnts[vidx]);
                  
                  // remember pos is holding displacements
                  pos.scaledAdd (w*rbfVal, tmp, pos);
                  
               }
               neighbourCount += myNumNeighbourWeights[i];
               vtx.pnt.set (myBasePnts[i]);
               vtx.pnt.add (pos);
               break;
            default:
               break;
         }
      }
      
      mesh.notifyVertexPositionsModified();
      
   }

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

//   @Override
//   public void connectToHierarchy () {
//      super.connectToHierarchy ();
//      for (RigidBody bod : myRigidBodies) {
//         bod.addBackReference (this);
//      }
//   }
//
//   @Override
//   public void disconnectFromHierarchy() {
//      super.disconnectFromHierarchy();
//      for (RigidBody bod : myRigidBodies) {
//         bod.removeBackReference (this);
//      }
//   }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (scanAttributeName (rtok, "mesh")) {
         myMeshInfo.scan (rtok);
         myMeshInfo.getMesh().setFixed (false);
         return true;
      }
      else if (scanAndStoreReferences (
         rtok, "rigidBodies", tokens) >= 0) {
         return true;
      }
      else if (scanAttributeName (rtok, "basePnts")) {
         rtok.scanToken ('[');
         Point3d pnt = new Point3d();
         ArrayList<Point3d> pntList =
         new ArrayList<Point3d>(myMeshInfo.numVertices());
         while (rtok.nextToken() != ']') {
            rtok.pushBack();
            pnt.scan (rtok);
            pntList.add (new Point3d(pnt));
         }
         myBasePnts = pntList.toArray(new Point3d[0]);
         return true;
      }
      else if (scanAttributeName (rtok, "basePoses")) {
         rtok.scanToken ('[');
         RigidTransform3d X = new RigidTransform3d();
         ArrayList<RigidTransform3d> poseList =
         new ArrayList<RigidTransform3d>(myRigidBodies.size());
         while (rtok.nextToken() != ']') {
            rtok.pushBack();
            X.scan (rtok);
            poseList.add (new RigidTransform3d(X));
         }
         myBasePoses = poseList.toArray(new RigidTransform3d[0]);
         return true;
      }
      else if (scanAttributeName (rtok, "weights")) {
         rtok.scanToken ('[');
         ArrayList<Double> weightList =
         new ArrayList<Double>(
         myRigidBodies.size()*myMeshInfo.numVertices());
         while (rtok.nextToken() != ']') {
            rtok.pushBack();
            weightList.add (rtok.scanNumber());
         }
         myRigidWeights = new double[weightList.size()];
         for (int i=0; i<weightList.size(); i++) {
            myRigidWeights[i] = weightList.get(i);
         }
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }
   
   protected boolean postscanItem (
   Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {

      if (postscanAttributeName (tokens, "rigidBodies")) {
         myRigidBodies.clear();
         postscanReferences (
            tokens, myRigidBodies, RigidBody.class, ancestor);
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      int numBods = myRigidBodies.size();
      int numVtxs = myMeshInfo.numVertices();

      super.writeItems (pw, fmt, ancestor);
      pw.println ("rigidBodies=");
      ScanWriteUtils.writeBracketedReferences (pw, myRigidBodies, ancestor);
       pw.println ("weights=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      for (int i=0; i<numVtxs; i++) {
         for (int j=0; j<numBods; j++) {
            pw.print (fmt.format (myRigidWeights[i*numBods+j]) + " ");
         }
      }
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");
      pw.println ("basePnts=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      for (int i=0; i<numVtxs; i++) {
         pw.println (myBasePnts[i].toString (fmt));
       }
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");
      pw.println ("basePoses=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      for (int i=0; i<numBods; i++) {
         pw.println ("[");
         IndentingPrintWriter.addIndentation (pw, 2);         
         pw.println (myBasePoses[i].toString (fmt));
         IndentingPrintWriter.addIndentation (pw, -2);         
         pw.println ("]");
       }
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");
   }

   
}
