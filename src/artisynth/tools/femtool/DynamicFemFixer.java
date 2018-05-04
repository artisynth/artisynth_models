package artisynth.tools.femtool;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.TriangleIntersector;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.ParticleMeshConstraint;
import artisynth.core.modelbase.ModelComponentBase;

public class DynamicFemFixer extends ModelComponentBase implements HasProperties {

   public enum ElementAlignType {
      RIGID, ORTHOGONAL, LIMITED_ORTHOGONAL
   }

   public static ElementAlignType DEFAULT_ELEMENT_ALIGN_TYPE =
      ElementAlignType.ORTHOGONAL;
   public static double DEFAULT_MIN_SCALE_RATIO = 0.3;
   public static double DEFAULT_YOUNGS_MODULUS = 4000;
   public static double DEFAULT_POISSON_RATIO = 0.2;
   public static double DEFAULT_PARTICLE_DAMPING = 10;
   public static double DEFAULT_DENSITY = 0.01;

   private ElementAlignType alignType = DEFAULT_ELEMENT_ALIGN_TYPE;
   private double minScaleRatio = DEFAULT_MIN_SCALE_RATIO;
   private double youngsModulus = DEFAULT_YOUNGS_MODULUS;
   private double poissonRatio = DEFAULT_POISSON_RATIO;
   private double particleDamping = DEFAULT_PARTICLE_DAMPING;
   private double density = DEFAULT_DENSITY;

   private FemModel3d model = null;

   private ArrayList<PolygonalMesh> myMeshList = null;
   private ArrayList<ParticleMeshConstraint> myMeshConstraints = null;
   private ArrayList<FemNode3d> myStaticNodes = null;
   private MechModel myMechModel = null;

   public static PropertyList myProps =
      new PropertyList(DynamicFemFixer.class);

   static {
      myProps.add("alignType * *", "", DEFAULT_ELEMENT_ALIGN_TYPE);
      myProps.add("minScaleRatio * *", "", DEFAULT_MIN_SCALE_RATIO);
      myProps.add("youngsModulus * *", "", DEFAULT_YOUNGS_MODULUS);
      myProps.add("poissonRatio * *", "", DEFAULT_POISSON_RATIO);
      myProps.add("particleDamping * *", "", DEFAULT_PARTICLE_DAMPING);
      myProps.add("density * *", "", DEFAULT_DENSITY);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public Property getProperty(String pathName) {
      return PropertyList.getProperty(pathName, this);
   }

   // private boolean equalizeVolumes = false; // XXX seems to cause a crash
   // somewhere
   // private double naturalVolume = 1;
   // private boolean verbose = false;
   // private int printPos = 0;

   private static final double ltri = Math.sqrt(3) / 2;
   public static final Point3d[] NATURAL_HEX =
   { new Point3d(0, 0, 1),
    new Point3d(1, 0, 1),
    new Point3d(1, 1, 1),
    new Point3d(0, 1, 1),
    new Point3d(0, 0, 0),
    new Point3d(1, 0, 0),
    new Point3d(1, 1, 0),
    new Point3d(0, 1, 0)
   };

   public static final Point3d[] NATURAL_WEDGE =
   { new Point3d(0, 0, 0),
    new Point3d(1, 0, 0),
    new Point3d(0.5, ltri, 0),
    new Point3d(0, 0, 1),
    new Point3d(1, 0, 1),
    new Point3d(0.5, ltri, 1) };

   public static final Point3d[] NATURAL_PYRAMID =
   { new Point3d(0, 0, 0),
    new Point3d(1, 0, 0),
    new Point3d(1, 1, 0),
    new Point3d(0, 1, 0),
    new Point3d(0.5, 0.5, 1 / Math.sqrt(2)) };

   public static final Point3d[] NATURAL_TET =
   { new Point3d(0, 0, 0),
    new Point3d(1, 0, 0),
    new Point3d(0.5, ltri, 0),
    new Point3d(0.5, 0.5 / Math.sqrt(3), Math.sqrt(2) / Math.sqrt(3)) };

   public DynamicFemFixer (MechModel mech) {

      myMechModel = mech;
      myMeshList = new ArrayList<PolygonalMesh>();
      myMeshConstraints = new ArrayList<ParticleMeshConstraint>();
      myStaticNodes = new ArrayList<FemNode3d>();

      
      
   }

   public DynamicFemFixer (FemModel3d ref, MechModel mech) {
      this(mech);
      setModel(ref);
   }

   public void setModel(FemModel3d ref) {
      model = ref;
      //assignProperties(ref);
   }
   
   public void assignDefaultProperties(FemModel3d ref) {
//      ref.setYoungsModulus(DEFAULT_YOUNGS_MODULUS);
//      ref.setPoissonsRatio(DEFAULT_POISSON_RATIO);
      ref.setLinearMaterial (DEFAULT_YOUNGS_MODULUS, DEFAULT_POISSON_RATIO, true);
      ref.setParticleDamping(DEFAULT_PARTICLE_DAMPING);
      ref.setDensity(DEFAULT_DENSITY);
   }
   
   public void assignProperties(FemModel3d ref) {
//      ref.setYoungsModulus(youngsModulus);
//      ref.setPoissonsRatio(poissonRatio);
      ref.setLinearMaterial (youngsModulus, poissonRatio, true);
      ref.setParticleDamping(particleDamping);
      ref.setDensity(density);
   }

   public double getMinScaleRatio() {
      return minScaleRatio;
   }

   public void setMinScaleRatio(double minScaleRatio) {
      minScaleRatio = Math.abs(minScaleRatio);
      if (minScaleRatio > 1) {
         minScaleRatio = 1 / minScaleRatio;
      }
      this.minScaleRatio = minScaleRatio;
   }

   public double getYoungsModulus() {
      return youngsModulus;
   }

   public void setYoungsModulus(double youngsModulus) {
      this.youngsModulus = youngsModulus;
      if (model != null) {
         model.setLinearMaterial (youngsModulus, poissonRatio, true);
      }
   }

   public double getPoissonRatio() {
      return poissonRatio;
   }

   public void setPoissonRatio(double poissonRatio) {
      this.poissonRatio = poissonRatio;
      if (model != null) {
         model.setLinearMaterial (youngsModulus, poissonRatio, true);
      }
   }

   public double getParticleDamping() {
      return particleDamping;
   }

   public void setParticleDamping(double particleDamping) {
      this.particleDamping = particleDamping;
      if (model != null) {
         model.setParticleDamping(particleDamping);
      }
   }
   
   public double getDensity() {
      return density;
   }

   public void setDensity(double density) {
      this.density = density;
      if (model != null) {
         model.setDensity(density);
      }
   }

   public FemModel3d getModel() {
      return model;
   }

   public void updateModel() {
      model.invalidateRestData();
   }

   public void setModelToRest() {
      setToRestPositions(model.getNodes());
      model.invalidateRestData();
   }

   public void updateIdealRestPositions() {
      updateIdealRestPositions(model, alignType, minScaleRatio);
   }

   public void freezeSurface(boolean freeze) {
      freezeSurfaceNodes(model, freeze);
   }

   public void constrainSurface(PolygonalMesh mesh) {
      addMeshConstraints(getSurfaceNodes(), mesh);
   }

   // on reference model
   public ArrayList<FemNode3d> getSurfaceNodes() {

      ArrayList<FemNode3d> surfaceNodes = new ArrayList<FemNode3d>();
      for (FemNode3d node : model.getNodes()) {
         if (model.isSurfaceNode(node)) {
            surfaceNodes.add(node);
         }
      }
      return surfaceNodes;
   }

   public ArrayList<FemNode3d> getNodesTouchingSurface(PolygonalMesh mesh) {
      return getNodesTouchingSurface(model, mesh, 1e-10);
   }

   public ArrayList<FemNode3d> getNodesTouchingSurface(PolygonalMesh mesh,
      double epsilon) {
      return getNodesTouchingSurface(model, mesh, epsilon);
   }

   public static ArrayList<FemNode3d> getNodesTouchingSurface(FemModel3d model,
      PolygonalMesh mesh, double epsilon) {

      ArrayList<FemNode3d> tNodes = new ArrayList<FemNode3d>();

      Point3d nearest = new Point3d();
      Vector2d coords = new Vector2d();

      for (FemNode3d node : model.getNodes()) {
         BVFeatureQuery.getNearestFaceToPoint (
            nearest, coords, mesh, node.getPosition());
         if (nearest.distance(node.getPosition()) < epsilon) {
            tNodes.add(node);
         }
      }

      return tNodes;
   }

   public void freezeInterior(boolean freeze) {
      freezeInteriorNodes(model, freeze);
   }

   public void projectSurfaceToMesh(PolygonalMesh mesh) {
      projectSurfaceToMesh(model, mesh);
   }

   public void projectSurfaceToMeshNormally(PolygonalMesh mesh, double maxDist,
      double maxConsistency) {
      projectSurfaceToMeshNormally(
         model, mesh, maxDist, maxConsistency);
   }

   public void expandSurfaceRadial(Point3d center, double scale) {
      DynamicFemFixer.expandSurfaceRadially(
         model, center, scale);
   }

   public void expandSurfaceNormally(double distance) {
      DynamicFemFixer.expandSurfaceNormally(
         model, distance);
   }

   public static void updateIdealRestPositions(FemModel3d model,
      ElementAlignType alignType) {
      updateIdealRestPositions(model, alignType, DEFAULT_MIN_SCALE_RATIO);
   }

   public static void updateIdealRestPositions(FemModel3d model,
      ElementAlignType alignType, double minScaleRatio) {
      for (FemElement3d elem : model.getElements()) {
         updateIdealRestPositions(elem, alignType, minScaleRatio);
      }
   }

   public static void updateIdealRestPositions(FemElement3d elem,
      ElementAlignType alignType) {
      updateIdealRestPositions(elem, alignType, DEFAULT_MIN_SCALE_RATIO);
   }

   public static void updateIdealRestPositions(FemElement3d elem,
      ElementAlignType alignType, double minScaleRatio) {

      ArrayList<Point3d> nodePositions = new ArrayList<Point3d>();
      ArrayList<Point3d> restPositions = new ArrayList<Point3d>();
      FemNode3d nodes[] = elem.getNodes();

      Point3d[] naturalShape = null;

      for (FemNode3d node : nodes) {
         nodePositions.add(node.getPosition());
      }

      // determine rest positions
      switch (nodes.length) {
         case 4:
            naturalShape = NATURAL_TET;
            break;
         case 5:
            naturalShape = NATURAL_PYRAMID;
            break;
         case 6:
            naturalShape = NATURAL_WEDGE;
            break;
         case 8:
            naturalShape = NATURAL_HEX;
            break;
         default:
            System.err.println("Error: unknown shape");
            return;
      }

      for (Point3d pnt : naturalShape) {
         restPositions.add(new Point3d(pnt));
      }

      AffineTransform3d trans = new AffineTransform3d();

      switch (alignType) {
         case ORTHOGONAL:
            trans.fitOrthogonal(nodePositions, restPositions);
            break;
         case LIMITED_ORTHOGONAL:
            Vector3d S = trans.fitOrthogonal(nodePositions, restPositions);

            // enforce limits
            boolean rescale = false;
            int iMax = 0;
            for (int i = 1; i < 3; i++) {
               if (Math.abs(S.get(i)) > Math.abs(S.get(iMax))) {
                  iMax = i;
               }
            }
            for (int i = 0; i < 3; i++) {
               if (Math.abs(S.get(i) / S.get(iMax)) < minScaleRatio) {
                  rescale = true;
                  int a = 1;
                  if (S.get(i) < 0) {
                     a = -1;
                  }
                  S.set(i, a * Math.abs(S.get(iMax) * minScaleRatio));
               }
            }

            if (rescale) {
               // rescale each column
               Vector3d col = new Vector3d();
               for (int i = 0; i < 3; i++) {
                  trans.A.getColumn(i, col);
                  col.normalize();
                  col.scale(S.get(i));
                  trans.A.setColumn(i, col);
               }
            }

            break;
         default:
            trans.fitRigid(nodePositions, restPositions, true);
      }

      if (trans.A.determinant() < 0) {
         // System.out
         // .println("Warning: affine transformation has flipped element");
         trans.applyScaling(1, 1, -1); // i
      }

      // // re-adjust scaling
      // // XXX fix this for different element types
      // if (equalizeVolumes) {
      // Vector3d S = new Vector3d();
      // Vector3d col = new Vector3d();
      // for (int i=0; i<3; i++) {
      // trans.A.getColumn(i, col);
      // S.set(i, col.norm());
      // }
      //
      // double approxVol = S.x*S.y*S.z;
      // double s = Math.pow(naturalVolume/approxVol,1.0/3.0);
      //
      // for (int i=0; i<3; i++) {
      // trans.A.getColumn(i, col);
      // col.scale(s);
      // trans.A.setColumn(i, col);
      // }
      //
      // }

      // apply new rest positions to model
      for (int i = 0; i < nodes.length; i++) {
         Point3d pos = restPositions.get(i);
         pos.transform(trans);
         nodes[i].setRestPosition(pos);
      }

      // recompute initial InvJ0 at each integration node
      elem.invalidateRestData();
      elem.getIntegrationData();

   }

   // public void setVerbose(boolean verbose) {
   // this.verbose = verbose;
   // }
   // public void setEqualizeVolumes(boolean set) {
   // equalizeVolumes = set;
   // }

   public void setAlignType(ElementAlignType type) {
      alignType = type;
   }

   public ElementAlignType getAlignType() {
      return alignType;
   }

   public void setDynamic(List<FemNode3d> nodes, boolean dynamic) {
      for (FemNode3d node : nodes) {
         setDynamic(node, dynamic);
      }
   }

   public void setDynamic(FemNode3d node, boolean dynamic) {

      node.setDynamic(dynamic);

      // update dynamic node list
      if (dynamic) {
         if (myStaticNodes.contains(node)) {
            myStaticNodes.remove(node);
         }
      } else {
         if (!myStaticNodes.contains(node)) {
            myStaticNodes.add(node);
         }
      }

   }

   public void updateStaticNodeList() {
      myStaticNodes.clear();

      for (FemNode3d node : model.getNodes()) {
         if (!node.isDynamic()) {
            myStaticNodes.add(node);
         }
      }

   }

   public ArrayList<FemNode3d> getStaticNodeList() {
      return myStaticNodes;
   }

   public static double getMinDetJ(FemElement3d e) {

      double myMinDetJ = Double.POSITIVE_INFINITY;

      IntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      IntegrationData3d[] idata = e.getIntegrationData();

      e.setInverted(false); // will check this below
      // e.myAvgStress.setZero();
      for (int k = 0; k < ipnts.length; k++) {
         IntegrationPoint3d pt = ipnts[k];
         //pt.computeJacobianAndGradient(e.getNodes(), idata[k].getInvJ0());
         double detJ = pt.computeJacobianDeterminant(e.getNodes());
         if (detJ < myMinDetJ) {
            myMinDetJ = detJ;
         }
      }
      return myMinDetJ;

   }

   public static double getMinDetJ(FemModel3d model) {

      double myMinDetJ = Double.POSITIVE_INFINITY;

      for (FemElement3d elem : model.getElements()) {
         double detJ = getMinDetJ(elem);
         if (detJ < myMinDetJ) {
            myMinDetJ = detJ;
         }
      }
      return myMinDetJ;

   }

   public static ArrayList<FemElement3d> getInvertedElements(FemModel3d model) {

      ArrayList<FemElement3d> inverted = new ArrayList<FemElement3d>();
      for (FemElement3d elem : model.getElements()) {

         double detJ = DynamicFemFixer.getMinDetJ(elem);
         if (detJ < 0) {
            inverted.add(elem);
         }
      }

      if (inverted.size() > 0) {
         return inverted;
      }

      return null;
   }

   public ArrayList<FemElement3d> getInvertedElements() {
      return getInvertedElements(model);
   }

   public static void freezeSurfaceNodes(FemModel3d model, boolean freeze) {

      for (FemNode3d node : model.getNodes()) {
         if (model.isSurfaceNode(node)) {
            node.setDynamic(!freeze);
         }
      }

   }

   public static void freezeInteriorNodes(FemModel3d model, boolean freeze) {
      for (FemNode3d node : model.getNodes()) {
         if (!model.isSurfaceNode(node)) {
            node.setDynamic(!freeze);
         }
      }
   }

   private static double computeNormalConsistency(Vertex3d vtx,
      Vector3d nrm) {

      nrm.set(0, 0, 0);
      Iterator<HalfEdge> it = vtx.getIncidentHalfEdges();
      ArrayList<Vector3d> normals = new ArrayList<Vector3d>();

      while (it.hasNext()) {
         HalfEdge he = it.next();
         Face face = he.getFace();
         nrm.add(face.getNormal());
         normals.add(face.getNormal());
      }

      if (nrm.norm() < 1e-5) {
         nrm.set(0, 0, 0);
         return 0;
      }
      nrm.normalize();

      // evaluate confidence of normal
      double d = 0;
      for (Vector3d n : normals) {
         d += n.dot(nrm);
      }
      d = d / normals.size();
      return d;
   }

   public static void projectSurfaceToMeshNormally(FemModel3d fem,
      PolygonalMesh mesh, double maxDist, double minConsistency) {

      if (fem == null || mesh == null) {
         return;
      }

      Point3d nearest = new Point3d();
      BVFeatureQuery query = new BVFeatureQuery();

      // normal direction
      Vector3d nrm = new Vector3d();
      Vector3d duv = new Vector3d();
      Vector3d duv2 = new Vector3d();
      
      
      for (FemNode3d node : fem.getNodes()) {

         if (fem.isSurfaceNode(node)) {

            double d =
               computeNormalConsistency(fem.getSurfaceVertex(node), nrm);

            // search in both directions
            //Face face = obb.intersect(node.getPosition(), nrm, duv, ti);
            //nrm.negate();
            //Face face2 = obb.intersect(node.getPosition(), nrm, duv2, ti);
            
            Face face = query.nearestFaceAlongRay (
               null, duv, mesh, node.getPosition(), nrm);
            nrm.negate();
            Face face2 = query.nearestFaceAlongRay (
               null, duv2, mesh, node.getPosition(), nrm);
            
            // find closest face
            if (face == null && face2 != null) {
               face = face2;
            } else if (face != null && face2 != null && Math.abs(duv2.x) < Math.abs(duv.x)) {
               face = face2;
            } else {
               nrm.negate();  // return to first normal, use first face
            }
            
            
            // only project if there is an intersection
            if (face != null && d > minConsistency) {
               // nearest.set(0,0,0);
               // nearest.scaledAdd(duv.y, face.getVertex(0).getPosition());
               // nearest.scaledAdd(duv.z, face.getVertex(1).getPosition());
               // nearest.scaledAdd(1-duv.y-duv.z,
               // face.getVertex(2).getPosition());

               nearest =
                  VolumetricMeshGenerator.projToSurface(
                     node.getPosition(), nrm, face);
               if (node.getPosition().distance(nearest) < maxDist) {
                  node.setPosition(nearest);
               }
            }
         }
      }

   }

   public static void projectSurfaceToMesh(FemModel3d fem,
      PolygonalMesh mesh) {

      if (fem == null || mesh == null) {
         return;
      }

      Point3d nearest = new Point3d();
      Vector2d coords = new Vector2d();

      for (FemNode3d node : fem.getNodes()) {

         if (fem.isSurfaceNode(node)) {
            // project to closest point on mesh
            BVFeatureQuery.getNearestFaceToPoint (
               nearest, coords, mesh, node.getPosition());
            node.setPosition(nearest);
         }
      }

   }

   public static void
      updateReferenceModel(FemModel3d model) {

      for (FemNode3d node : model.getNodes()) {
         node.setRestPosition(node.getPosition());
      }

   }

   public static void setToRestPositions(Collection<FemNode3d> nodes) {
      for (FemNode3d node : nodes) {
         node.setRestPosition(node.getPosition());
      }
   }

   public static void expandSurfaceRadially( FemModel3d fem, Point3d center, double scale) {

      Point3d p = new Point3d();

      for (FemNode3d node : fem.getNodes()) {

         if (fem.isSurfaceNode(node)) {

            p.set(node.getPosition());
            p.sub(center);
            p.scale(scale);
            p.add(center);
            node.setPosition(p);

         }
      }

   }

   public static void expandSurfaceNormally(FemModel3d fem, double dist) {

      Vector3d nrm = new Vector3d();
      Point3d p = new Point3d();

      for (FemNode3d node : fem.getNodes()) {
         if (fem.isSurfaceNode(node)) {

            Vertex3d vtx = fem.getSurfaceVertex(node);
            vtx.computeNormal(nrm);
            nrm.normalize();
            nrm.scale(dist);

            p.set(node.getPosition());
            p.add(nrm);
            node.setPosition(p);

         }
      }

   }

   public void addDependentMesh(PolygonalMesh mesh) {
      if (!myMeshList.contains(mesh)) {
         myMeshList.add(mesh);
      }
   }

   public void removeDependentMesh(PolygonalMesh mesh) {

      // remove constraints involving mesh
      removeMeshConstraints(mesh);

      if (myMeshList.contains(mesh)) {
         myMeshList.remove(mesh);
      }

   }

   public void addMeshConstraint(FemNode3d node, PolygonalMesh mesh) {
      addDependentMesh(mesh);

      ParticleMeshConstraint c = new ParticleMeshConstraint(node, mesh);
      // RenderProps.setVisible(c, false);
      myMeshConstraints.add(c);
      myMechModel.addConstrainer(c);
   }

   public void removeMeshConstraints(FemNode3d node) {

      for (ParticleMeshConstraint c : myMeshConstraints) {
         if (c.getParticle(0) == node) {
            removeMeshConstraint(c);
         }
      }

   }

   public void removeMeshConstraints(PolygonalMesh mesh) {

      for (ParticleMeshConstraint c : myMeshConstraints) {
         if (c.getMesh() == mesh) {
            removeMeshConstraint(c);
         }
      }

   }

   public void removeMeshConstraint(FemNode3d node, PolygonalMesh mesh) {

      for (ParticleMeshConstraint c : myMeshConstraints) {
         if (c.getMesh() == mesh && c.getParticle(0) == node) {
            removeMeshConstraint(c);
         }
      }

   }

   public void removeMeshConstraint(ParticleMeshConstraint c) {
      myMechModel.removeConstrainer(c);
      myMeshConstraints.remove(c);
   }

   public void addMeshConstraints(List<FemNode3d> nodes, PolygonalMesh mesh) {
      for (FemNode3d node : nodes) {
         addMeshConstraint(node, mesh);
      }
   }

   public ArrayList<ParticleMeshConstraint> getMeshConstraints() {
      return myMeshConstraints;
   }
   
   public static void addControls(
      ControlPanel controlPanel, DynamicFemFixer fixer) {
      
      for (PropertyInfo propInfo : myProps) {
         Property prop = fixer.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create (prop);
         controlPanel.addWidget(widget);
      }
   }


}
