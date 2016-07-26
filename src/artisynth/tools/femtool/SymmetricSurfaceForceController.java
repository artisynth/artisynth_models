package artisynth.tools.femtool;

import java.awt.Color;
import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
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
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointForce;
import artisynth.core.modelbase.ControllerBase;

public class SymmetricSurfaceForceController extends ControllerBase {

   public static boolean DEFAULT_ENABLED = true;
   public static double DEFAULT_ELASTICITY = 1;

   private PolygonalMesh surface = null;
   private boolean enabled = DEFAULT_ENABLED;
   private double k = DEFAULT_ELASTICITY;
   private double symK = DEFAULT_ELASTICITY;
   private double maxDist = Double.POSITIVE_INFINITY;

   private MechModel myModel = null;
   private FemModel3d myFem = null;

   private boolean zero = false;

   ArrayList<PointForce> forceList = new ArrayList<PointForce>();
   ArrayList<PointForce> symForceList = new ArrayList<PointForce>();
   ArrayList<Point3d> symPoints = new ArrayList<Point3d>();
   ArrayList<FemMarker> symFemMarkers = new ArrayList<FemMarker>();

   public static PropertyList myProps =
      new PropertyList(SymmetricSurfaceForceController.class);

   static {
      myProps.add("enabled * *", "Controller enabled", DEFAULT_ENABLED);
      myProps.add("elasticity * *", "Force elasticity", DEFAULT_ELASTICITY);
      myProps.add(
         "symElasticity * *", "Symmetric elasticity", DEFAULT_ELASTICITY);
      myProps.add(
         "maxDistance * *", "Maximum distance to use in computing force",
         Double.POSITIVE_INFINITY);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public void setFem(FemModel3d fem) {
      myFem = fem;
      symFemMarkers.clear();
   }

   public SymmetricSurfaceForceController (MechModel model,
      PolygonalMesh surface, FemModel3d fem) {
      this.myModel = model;
      this.surface = surface;
      this.myFem = fem;
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
      for (PointForce force : symForceList) {
         force.setMagnitude(0);
      }

      zero = true;
   }

   private void updateFemMarkers() {
      
      Point3d loc = new Point3d();
      for (int i = 0; i < symPoints.size(); i++) {
         FemMarker mkr = symFemMarkers.get(i);
         Point3d pnt = symPoints.get(i);
         
         mkr.setPosition(pnt);
         mkr.resetElement(myFem);
         mkr.updateAttachment();
      }
      
   }

   void updateForces() {

      zero = false;

      if (!enabled) {
         for (PointForce force : forceList) {
            force.setForce(Vector3d.ZERO);
         }
         for (PointForce force : symForceList) {
            force.setForce(Vector3d.ZERO);
         }
      }

      BVFeatureQuery query = new BVFeatureQuery();
      Vector2d coords = new Vector2d();
      Point3d nearest = new Point3d();
      Vector3d dir = new Vector3d();

      for (PointForce force : forceList) {

         Point p = force.getPoint();
         // project to surface
         query.nearestFaceToPoint(nearest, coords, surface, p.getPosition());
         dir.sub(nearest, p.getPosition());
         
         if (dir.norm() > maxDist) {
            dir.scale(maxDist/dir.norm());
         }
         dir.scale(k);

         force.setForce(dir);
      }

      updateFemMarkers();

      for (int i = 0; i < symForceList.size(); i++) {
         PointForce force = symForceList.get(i);
         Point3d pnt = symPoints.get(i);
         FemMarker mkr = symFemMarkers.get(i);

         dir.sub(pnt, mkr.getPosition());
         if (dir.norm() > maxDist) {
            dir.scale(maxDist/dir.norm());
         }
         
         dir.scale(symK);

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

   public void setSymElasticity(double k) {
      this.symK = k;
      updateForces();
   }

   public double getSymElasticity() {
      return this.symK;
   }

   public void setEnabled(boolean enabled) {
      this.enabled = enabled;
   }

   public boolean getEnabled() {
      return enabled;
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

   public void addPoint(Point p) {
      PointForce f = new PointForce(p);
      RenderProps.setVisible(f, false);
      f.setName("sfc_" + forceList.size());
      myModel.addForceEffector(f);
      forceList.add(f);
      f.setForce(Vector3d.ZERO);

   }

   public void addSymPoint(Point3d pnt) {
      symPoints.add(pnt);

      FemMarker fm = new FemMarker(pnt); // projects to FEM?
      PointForce f = new PointForce(fm);
      RenderProps.setVisible(fm, true);
      RenderProps.setVisible(f, false);
      RenderProps.setPointColor(fm, Color.CYAN);
      
      myFem.addMarker(fm);
      symFemMarkers.add(fm);

      symForceList.add(f);
      f.setForce(Vector3d.ZERO);
      f.setName("rsfc_" + symForceList.size());

      myModel.addForceEffector(f);

   }

   public void clearPoints() {
      for (PointForce f : forceList) {
         myModel.removeForceEffector(f);
      }
      forceList.clear();
   }

   public void clearSymPoints() {
      for (PointForce f : symForceList) {
         myModel.removeForceEffector(f);
      }
      symForceList.clear();
      symPoints.clear();
   }

   public static void addControls(
      ControlPanel controlPanel, SymmetricSurfaceForceController controller) {

      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create(prop);
         controlPanel.addWidget(widget);
      }
   }

   @Override
   public void initialize(double t0) {
      super.initialize(t0);

      for (PointForce force : forceList) {
         force.setForce(Vector3d.ZERO);
      }
      for (PointForce force : symForceList) {
         force.setForce(Vector3d.ZERO);
      }

   }
}
