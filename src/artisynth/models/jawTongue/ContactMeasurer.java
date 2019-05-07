package artisynth.models.jawTongue;

import java.util.HashMap;
import java.util.Map;

import artisynth.core.mechmodels.CollidableBody;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.modelbase.MonitorBase;
import maspack.collision.IntersectionContour;
import maspack.geometry.Vertex3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

public class ContactMeasurer extends MonitorBase {

   public ContactMeasurer (CollisionResponse resp) {
      response = resp;
   }

//   public ArrayList<CollisionHandler> collisionHandlers = new ArrayList<CollisionHandler> ();
   CollisionResponse response;
   int[] handlerIndices = null;
   double contactForce;
   double contactPressure;
   double contactArea;

   public static PropertyList myProps =
      new PropertyList (ContactMeasurer.class, MonitorBase.class);

   static {
      myProps.add ("contactForce", "tongue-pharyx contact force", 0d);
      myProps.add ("contactPressure", "tongue-pharyx  contact pressure", 0d);
      myProps.add ("contactArea", "tongue-pharyx  contact area", 0d);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   public void setHandlersToMeasure(int[] idxs) {
      handlerIndices = idxs;
   }
    
   public void computeContactForces (double dt) {
      contactForce = 0;
      contactArea = 0;
      contactPressure = 0;
      //for (int idx : handlerIndices) {
         double collForce = 0.0;
         double f_localMax = 0.0;
         HashMap<Vertex3d,Vector3d> collForces =
            new HashMap<Vertex3d,Vector3d> ();

         Map<Vertex3d,Vector3d> collMap = response.getContactForces(0);

         if (collMap != null) {
            for (Vertex3d v : collMap.keySet ()) {
               Vector3d force = new Vector3d (collMap.get (v));
               collForces.put (v, force);
               collForce = collForce + force.norm ();
               if (force.norm () > f_localMax)
                  f_localMax = force.norm ();
            }
         }
         // System.out.println("total force = "+collForce);
         // System.out.println("forces = "+collForces.values ().toString ());

         double collArea = response.getContactArea(0);
         
//         if (h.getRenderContactInfo () == null) {
//            System.out.println("renderContactInfo null");
//         }
//         
//         double collArea = 0;
//         if (h.getRenderContactInfo () != null) {
//            if (h.getRenderContactInfo ().getContours() != null
//            && h.getRenderContactInfo ().getContours().size () > 0) {
//               for (IntersectionContour c : h
//                  .getRenderContactInfo ().getContours()) {
//                  collArea += c.computePlanarArea ();
//               }
//            }
//         }

         contactForce += collForce;
         contactArea += collArea;
         contactPressure += (collArea > 0)? contactForce / collArea : contactForce;
      //}
   }

   public double getContactForce () {
      return contactForce;
   }

   public void setContactForce (double contactForce) {
      this.contactForce = contactForce;
   }

   public double getContactPressure () {
      return contactPressure;
   }

   public void setContactPressure (double contactPressure) {
      this.contactPressure = contactPressure;
   }

   public double getContactArea () {
      return contactArea;
   }

   public void setContactArea (double contactArea) {
      this.contactArea = contactArea;
   }

   @Override
   public void apply (double t0, double t1) {
      computeContactForces (t1 - t0);    
   }

}
