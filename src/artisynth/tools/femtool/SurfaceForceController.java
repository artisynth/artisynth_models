package artisynth.tools.femtool;

import java.util.ArrayList;
import java.util.Iterator;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointForce;
import artisynth.core.modelbase.ControllerBase;

public class SurfaceForceController extends ControllerBase {

   public static boolean DEFAULT_ENABLED = true;
   public static double DEFAULT_ELASTICITY = 1;
   public static double DEFAULT_SMOOTHING = 0;
   public static boolean DEFAULT_PERPENDICULAR = true;
   
   private PolygonalMesh surface = null;
   private boolean enabled = DEFAULT_ENABLED;
   private double k = DEFAULT_ELASTICITY;
   private double d = DEFAULT_SMOOTHING;
   private boolean perpendicular = DEFAULT_PERPENDICULAR;
   private double maxDist = Double.POSITIVE_INFINITY;
   
   
   private MechModel myModel = null;
   private FemModel3d myFem = null;
   
   private boolean zero = false;
   
   ArrayList<PointForce> forceList = new ArrayList<PointForce>();
   
   public static PropertyList myProps =
      new PropertyList(SurfaceForceController.class);
   static {
      myProps.add("enabled * *", "Controller enabled", DEFAULT_ENABLED);
      myProps.add("perpendicular * *", "Controller enabled", DEFAULT_PERPENDICULAR);
      myProps.add("maxDistance * *", "Maximum distance to use in computing force", Double.POSITIVE_INFINITY);
      myProps.add("elasticity * *", "Force elasticity", DEFAULT_ELASTICITY);
      myProps.add("smoothing * *", "Force smoothing", DEFAULT_SMOOTHING, "NW [0,1]");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public void setFemReference(FemModel3d fem) {
      myFem = fem;
   }
   
   public SurfaceForceController(MechModel model, PolygonalMesh surface) {
      this.myModel = model;
      this.surface = surface;      
   }
   
   public void apply(double t0, double t1) {
      if (!enabled) {
         zeroForces();
         return;
      }
      updateForces();
   }
   
   private void zeroForces() {
      
      if (zero) {
         return;
      }
      
      for (PointForce force : forceList) {
         force.setMagnitude(0);
      }
      zero = true;
   }
   
   private void updateForces() {
      
      zero = false;
      
      BVFeatureQuery query = new BVFeatureQuery();
      Vector2d coords = new Vector2d();
      Point3d nearest = new Point3d();
      Vector3d dir = new Vector3d();
      
      
      for (PointForce force : forceList) {
         
         Point p = force.getPoint();
         // project to surface
         query.nearestFaceToPoint (nearest, coords, surface, p.getPosition());
         dir.sub(nearest, p.getPosition());
         
         if (dir.norm() > maxDist) {
            dir.scale(maxDist/dir.norm());
         }
         dir.scale(k);
         dir.scale(1-d);
         
         Vector3d prevForce = new Vector3d(force.getForce());
         prevForce.scale(d);
         dir.add(prevForce);
         
         if (perpendicular && (myFem != null)) {
            if (p instanceof FemNode3d) {
               FemNode3d node = (FemNode3d)p;
               if (myFem.isSurfaceNode(node)) {
                  double mag = dir.norm();
                  Vertex3d vtx = myFem.getSurfaceVertex(node);
                  computeNormalConsistency(vtx, dir);
                  dir.scale(-mag);
               }
            }
         }
         
         // System.out.println("Force: " + dir.toString("% .2g"));
         force.setForce(dir);   
      }
   }   
   
   public void setElasticity(double k) {
      this.k = k;
      updateForces();
   }
   
   public double getElasticity() {
      return this.k;
   }
   
   public void setSmoothing(double d) {
      this.d = d;
      updateForces();
   }
   
   public double getSmoothing() {
      return d;
   }
   
   public void setEnabled(boolean enabled) {
      this.enabled = enabled;
   }
   
   public boolean getEnabled() {
      return enabled;
   }
      
   public void addPoint(Point p) {
      PointForce f = new PointForce(p);
      RenderProps.setVisible(f, false);
      f.setName("sfc_" + forceList.size());
      myModel.addForceEffector(f);
      forceList.add(f);
      
   }
   
   public void clearPoints() {
      for (PointForce f : forceList) {
         myModel.removeForceEffector(f);
      }
      forceList.clear();
   }

   public static void addControls(
      ControlPanel controlPanel, SurfaceForceController controller) {
      
      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create (prop);
         controlPanel.addWidget(widget);
      }
   }


   public boolean getPerpendicular() {
      return perpendicular;
   }


   public void setPerpendicular(boolean perpendicular) {
      this.perpendicular = perpendicular;
      updateForces();
   }
   
   @Override
   public void initialize(double t0) {
      super.initialize(t0);
      
      for (PointForce force : forceList) {
         force.setForce(Vector3d.ZERO);
         Vector3d prev = force.getForce();
         // System.out.println("force:" + prev);
      }
      
   }
   
   private static double computeNormalConsistency(Vertex3d vtx, Vector3d nrm) {

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
   
   public void setMaxDistance(double dist) {
      if (dist < 0) {
         maxDist = Double.POSITIVE_INFINITY;
      }
      maxDist = dist;
   }
   
   
   public double getMaxDistance() {
      return maxDist;
   }
   
}
