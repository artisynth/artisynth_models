package artisynth.models.registration.old;

import java.util.Iterator;

import artisynth.core.mechmodels.DynamicMeshComponent;
import artisynth.models.registration.old.pressures.GravityPressureFunction;
import artisynth.models.registration.old.pressures.RegistrationPressureFunction;
import maspack.geometry.AABBTree;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVTree;
import maspack.geometry.Face;
import maspack.geometry.Feature;
import maspack.geometry.HalfEdge;
import maspack.geometry.MeshBase;
import maspack.geometry.OBBTree;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyInfoList;
import maspack.properties.PropertyList;

/**
 * Computes and applies pressures (computed using a pressure function)
 * involved in a physics-based ICP (iterative-closest-point) registration.
 * 
 * The point force applied to the model is given by:
 *      f = sum_m area(v_m) p(y_m, x_m)
 * which is then interpolated to the underlying master components.
 * 
 * We approximate the pressure as a constant weight in time.
 * 
 * Note: this is much less stable than ICPRegistrationForce with a weight, or GMMRegistrationForce
 *
 */
@Deprecated
public class PressureMeshRegistrationForce extends MeshRegistrationForce {

   // properties
   static PropertyList myProps = new PropertyList (PressureMeshRegistrationForce.class);
   static {
      myProps.add ("pressureFunction", "pressure function", createDefaultPressureFunction());
   }
   
   // model/target
   MeshBase target;
   BVTree targetTree;
   DynamicMeshComponent model;

   RegistrationPressureFunction pressureFunc;
   
   private static class VertexEntry {
      Feature nearfeature; // nearest feature
      Point3d nearPnt;     // nearest point
      double area;
      Vector3d pressure;   // pressure
      
      VertexEntry() {
         nearfeature = null;
         nearPnt = new Point3d();
         area = 0;
         pressure = new Vector3d();
      }
   }
   
   VertexEntry[] vinfo;    // force information
   
   private static RegistrationPressureFunction createDefaultPressureFunction() {
      return new GravityPressureFunction ();
   }
   
   public PressureMeshRegistrationForce() {
      this.pressureFunc = createDefaultPressureFunction ();
   }
     
   @Override
   public void initialize(DynamicMeshComponent model, MeshBase target) {
      
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
      
      // initialize distance/pressure
      updateDistancesAreasAndPressures ();      
   }
   
   @Override
   public DynamicMeshComponent getSource () {
      return model;
   }
   
   @Override
   public double getCorrespondence (Point3d x, Vertex3d mvtx) {

      int m = mvtx.getIndex ();
      VertexEntry ventry = vinfo[m];
      
      Point3d ym = mvtx.getWorldPoint ();
      
      double d = ym.distance (ventry.nearPnt);
      double pn = ventry.pressure.norm ();      
      double s = d/pn;

      if (x != null) {
         x.set (mvtx.getWorldPoint ());
         // if pressure is essentially zero, then leave xm = ym
         if (pn > d*1e-12) {
            x.scaledAdd (s, ventry.pressure);
         }
      }
      
      double wm = 0;
      // if distance is essentially zero, then leave wm = 0
      if (d > 1e-12*pn) {
         wm = 1.0 / model.numVertices () * ventry.area / s;
      }
   
      return wm;
   }
   
   @Override
   public void update() {
      // update distances
      updateDistancesAreasAndPressures();
   }
   
   /**
    * Computes distances to target and updates vertex areas on source
    */
   public void updateDistancesAreasAndPressures() {
      
      // maybe update tree
      targetTree.setBvhToWorld (target.getMeshToWorld ());
      if (!target.isFixed ()) {
         targetTree.update ();
      }
      
      BVFeatureQuery bvq = new BVFeatureQuery ();
      Vector3d snrm = new Vector3d();
      Vector3d tnrm = new Vector3d();
      
      // loop through each model vertex
      for (Vertex3d vm : model.getMesh ().getVertices ()) {
         int vidx = vm.getIndex ();
         
         // find nearest feature
         VertexEntry dentry = vinfo[vidx];
         dentry.nearfeature = bvq.nearestFeatureToPoint (dentry.nearPnt, targetTree, vm.getWorldPoint ());
         
         // update area
         dentry.area = 0;
         // divide attached face areas
         Iterator<HalfEdge> hedges = vm.getIncidentHalfEdges();
         while (hedges.hasNext()) {
            HalfEdge he = hedges.next();
            double fa = he.getFace().computeArea();
            fa = fa/he.getFace().numVertices();
            dentry.area += fa;
         }
         
         // update pressure
         if (dentry.nearfeature != null) {
            vm.computeWorldNormal (snrm);
            if (dentry.nearfeature instanceof Face) {
               Face face = (Face)dentry.nearfeature;
               face.getWorldNormal (tnrm);
            } else if (dentry.nearfeature instanceof Vertex3d) {
               Vertex3d v = (Vertex3d)dentry.nearfeature;
               v.computeWorldNormal (tnrm);
            } else {
               tnrm.setZero();
            }
            pressureFunc.computePressure (vm.getWorldPoint (), snrm, dentry.nearPnt, tnrm, dentry.pressure);
         } else {
            dentry.pressure.setZero ();
         }
      }
   }
   
   /**
    * The pressure function computes a pressure to apply to the surface of the source during registration
    * @return the pressure function
    */
   public RegistrationPressureFunction getPressureFunction() {
      if (pressureFunc == null) {
         pressureFunc = createDefaultPressureFunction();
      }
      return pressureFunc;
   }


   /**
    * The pressure function computes a pressure to apply to the surface of the source during registration
    * @param pfunc
    */
   public void setPressureFunction(RegistrationPressureFunction pfunc) {
      this.pressureFunc = pfunc;
   }

   @Override
   public PropertyInfoList getAllPropertyInfo () {
      return myProps;
   }

}
