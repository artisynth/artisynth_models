package artisynth.tools.femtool;

import java.awt.Color;
import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;
import quickhull3d.QuickHull3D;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointForce;
import artisynth.core.modelbase.ControllerBase;

public class QHullSurfaceForceController extends ControllerBase {

   public static boolean DEFAULT_ENABLED = true;
   public static double DEFAULT_ELASTICITY = 1;
   public static boolean DEFAULT_PERPENDICULAR = true;
   
   private PolygonalMesh surface = null;
   private boolean enabled = DEFAULT_ENABLED;
   private double k = DEFAULT_ELASTICITY;
   private double hullK = DEFAULT_ELASTICITY;
   
   private MechModel myModel = null;
   private FemModel3d myFem = null;
   
   private boolean zero = false;
   
   ArrayList<PointForce> forceList = new ArrayList<PointForce>();
   ArrayList<PointForce> hullForceList = new ArrayList<PointForce>();
   
   ArrayList<Point3d> hullPoints = new ArrayList<Point3d>();
   ArrayList<FemMarker> hullFemMarkers = new ArrayList<FemMarker>();
   
   public static PropertyList myProps =
      new PropertyList(QHullSurfaceForceController.class);
   static {
      myProps.add("enabled * *", "Controller enabled", DEFAULT_ENABLED);
      myProps.add("elasticity * *", "Force elasticity", DEFAULT_ELASTICITY);
      myProps.add("hullElasticity * *", "hull elasticity", DEFAULT_ELASTICITY);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public void setFemReference(FemModel3d fem) {
      myFem = fem;
      hullFemMarkers.clear();
      clearHullForces();
      addFemMarkers();
      addHullForces();
   }
   
   public QHullSurfaceForceController(MechModel model, PolygonalMesh surface, FemModel3d fem) {
      this.myModel = model;
      this.surface = surface;      
      this.myFem = fem;
      
      computeHull(surface);
      addFemMarkers();
      addHullForces();
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
      for (PointForce force : hullForceList) {
         force.setMagnitude(0);
      }
      
      zero = true;
   }
   
   private void computeHull(PolygonalMesh mesh) {
      QuickHull3D hull = new QuickHull3D();
      double coords[] = new double[3*mesh.numVertices()];
      
      for (int i=0; i<mesh.numVertices(); i++) {
         Point3d pos = mesh.getVertex(i).getPosition();
         coords[i*3] = pos.x;
         coords[i*3+1] = pos.y;
         coords[i*3+2] = pos.z;
      }
      hull.build(coords);
           
      hullPoints = new ArrayList<Point3d>();
      for (quickhull3d.Point3d p : hull.getVertices()) {
         Point3d pnt = new Point3d(p.x, p.y, p.z);
         hullPoints.add(pnt);
      }
   }
   
   private void addFemMarkers() {
      for (Point3d pnt : hullPoints) {
         FemMarker fm = new FemMarker(pnt);
         RenderProps.setVisible(fm, true);
         RenderProps.setPointColor(fm, Color.CYAN);
         myFem.addMarker(fm);         
         hullFemMarkers.add(fm);
      }
   }
   
   private void addHullForces() {
      for (FemMarker mkr : hullFemMarkers) {
         PointForce pf = new PointForce(mkr);
         RenderProps.setVisible(pf, false);
         pf.setName("hull_" + hullForceList.size());
         myModel.addForceEffector(pf);
         hullForceList.add(pf);
      }
   }
   
   private void updateFemMarkers() {
      for (int i=0; i<hullPoints.size(); i++) {
         FemMarker mkr = hullFemMarkers.get(i);
         Point3d pnt = hullPoints.get(i);
         mkr.setPosition(pnt);
         mkr.resetElement (myFem);
         mkr.updateAttachment ();
      }
   }
   
   void updateForces() {
      
      zero = false;
      
      if (!enabled) {
         for (PointForce force : forceList) {
            force.setForce(Vector3d.ZERO);         
         }
         for (PointForce force : hullForceList) {
            force.setForce(Vector3d.ZERO);         
         }  
      }
      
      //OBBTree obbt = surface.getObbtree();
      BVFeatureQuery query = new BVFeatureQuery();
      Vector2d coords = new Vector2d();
      Point3d nearest = new Point3d();
      Vector3d dir = new Vector3d();
      
      
      for (PointForce force : forceList) {
         
         Point p = force.getPoint();
         // project to surface
         query.nearestFaceToPoint (nearest, coords, surface, p.getPosition());
         dir.sub(nearest,p.getPosition());
         dir.scale(k);
       //  if (dir.norm() > 1) {
            // System.out.println("large force detected");
         //}         
         force.setForce(dir);         
      }
      
      updateFemMarkers();
      
      for (int i=0; i<hullForceList.size(); i++) {
         PointForce force = hullForceList.get(i);
         Point3d pnt = hullPoints.get(i);
         FemMarker mkr = hullFemMarkers.get(i);
         
         dir.sub(pnt, mkr.getPosition());
         dir.scale(hullK);
         
         if (dir.norm() > 0) {
            // System.out.println("non-zero force detected");
         }
         
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
   
   public void setHullElasticity(double k) {
      this.hullK = k;
      updateForces();
   }
   
   public double getHullElasticity() {
      return this.hullK;
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
      f.setForce(Vector3d.ZERO);
      
   }
   
   public void clearPoints() {
      for (PointForce f : forceList) {
         myModel.removeForceEffector(f);
      }
      forceList.clear();
   }

   public void clearHullForces() {
      for (PointForce f : hullForceList) {
         myModel.removeForceEffector(f);
      }
      hullForceList.clear();
   }
   
   public static void addControls(
      ControlPanel controlPanel, QHullSurfaceForceController controller) {
      
      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create (prop);
         controlPanel.addWidget(widget);
      }
   }
}
