package artisynth.models.registration;

import artisynth.core.mechmodels.DynamicMeshComponent;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.modelbase.ControllerBase;
import artisynth.models.registration.correspondences.ICPMeshCorrespondence;
import artisynth.models.registration.correspondences.MeshCorrespondenceComputer;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;

/**
 * Computes correspondences for a mesh and updates them in a registration controller
 */
public class MeshCorrespondenceController extends ControllerBase {

   public static double DEFAULT_MESH_WEIGHT = 1;
   
   DynamicRegistrationController reg;
   MeshCorrespondenceComputer corr;
   DynamicMeshComponent model;
   MeshBase target;
   double meshWeight;
   
   static PropertyList myProps = new PropertyList(MeshCorrespondenceController.class, ControllerBase.class);
   static {
      myProps.add ("correspondenceComputer", "correspondence computer", createDefaultComputer (), "CE");
      myProps.add ("meshWeight", "weight applied to entire mesh", DEFAULT_MESH_WEIGHT);
   }
   
   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   
   public MeshCorrespondenceController(DynamicMeshComponent model, MeshBase target) {
      this(model, target, createDefaultComputer());
   }
   
   public MeshCorrespondenceController(DynamicMeshComponent model, MeshComponent target) {
      this(model, target.getMesh (), createDefaultComputer());
   }
   
   public MeshCorrespondenceController(DynamicMeshComponent model, MeshBase target, MeshCorrespondenceComputer computer) {
      this(model, target, computer, null);
   }
   
   /**
    * Constructs a controller that computes mesh registration correspondences based on
    * a helper object.
    * 
    * @param source source surface
    * @param target target surface
    * @param computer helper for computing correspondences
    * @param reg registration controller for setting target locations and weights
    */
   public MeshCorrespondenceController(DynamicMeshComponent source, MeshBase target, 
      MeshCorrespondenceComputer computer, DynamicRegistrationController reg) {
      this.model = source;
      this.target = target;
      this.meshWeight = DEFAULT_MESH_WEIGHT;
      setCorrespondenceComputer (computer);
      setRegistrationController (reg);
   }
   
   public static MeshCorrespondenceComputer createDefaultComputer() {
      return new ICPMeshCorrespondence ();
   }
   
   /**
    * Sets helper class for computer correspondences
    * @param corr correspondence computer
    */
   public void setCorrespondenceComputer(MeshCorrespondenceComputer corr) {
      this.corr = corr;
      if (corr != null) {
         this.corr.initialize (model.getMesh (), target);
      }
   }
   
   /**
    * Retrieves helper class for computer correspondences
    * @return the internal correspondence computer
    */
   public MeshCorrespondenceComputer getCorrespondenceComputer() {
      return this.corr;
   }
   
   /**
    * Applies correspondences for given registration controller
    * @param reg registration controller
    */
   public void setRegistrationController(DynamicRegistrationController reg) {
      this.reg = reg;
   }
   
   /**
    * Weight factor applied to entire set of mesh correspondences
    * @return mesh weight
    */
   public double getMeshWeight() {
      return meshWeight;
   }
   
   /**
    * Sets weight factor for entire set of mesh correspondences
    * @param w weight
    */
   public void setMeshWeight(double w) {
      meshWeight = w;
   }
   
   @Override
   public void initialize (double t) {
      this.apply (t, t);
   }
   
   @Override
   public void apply (double t0, double t1) {
      
      // update weights / targets for registration
      if (corr != null) {
         corr.apply (t0, t1);
      }
      
      // pass on weights to registration controller
      if (reg != null) {
         Point3d c = new Point3d();
         VectorNd wn = reg.getRegistrationWeight (model);
         MeshComponent target = reg.getRegistrationTarget (model);
         
         for (int i=0; i<model.numVertices (); ++i) {
            Vertex3d mvtx = model.getVertex (i);
            double w = meshWeight*corr.getCorrespondence (c, mvtx);
            RigidTransform3d rtrans = target.getMeshToWorld ();
            c.inverseTransform (rtrans);
            target.getVertex (i).setPosition (c);
            wn.set (i, w);
         }
         target.getMesh ().notifyVertexPositionsModified ();
      }
      
   }

}
