package artisynth.models.registration.old;

import java.util.HashSet;

import artisynth.core.mechmodels.DynamicMeshComponent;
import artisynth.core.mechmodels.ForceEffector;
import artisynth.core.mechmodels.PointAttachment;
import artisynth.core.modelbase.PropertyChangeEvent;
import artisynth.core.modelbase.PropertyChangeListener;
import artisynth.models.registration.AttachmentForceUtilities;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix;
import maspack.matrix.Point3d;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector3d;
import maspack.properties.CompositeProperty;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyInfoList;
import maspack.properties.PropertyList;
import maspack.util.DynamicArray;
import maspack.util.InternalErrorException;

@Deprecated
public abstract class MeshRegistrationForce implements ForceEffector, CompositeProperty {

   public static double DEFAULT_BETA = 1000; // force scaling
   double beta; // force scaling
   
   static HashSet<Class<?>> mySubClassSet = new HashSet<>();
   static DynamicArray<Class<?>> mySubClasses =
   new DynamicArray<>(new Class<?>[0]);

   static {
      registerSubClass(PressureMeshRegistrationForce.class);
   }

   /**
    * Finds any registered subclasses of this class, for use in property panels
    * 
    * @return list of registered subclasses
    */
   public static Class<?>[] getSubClasses() {
      return mySubClasses.getArray();
   }

   /**
    * Register a subclass of this class, for use in property panels
    * 
    * @param clazz
    */
   protected static void registerSubClass(Class<?> clazz) {
      if (!mySubClassSet.contains(clazz)) {
         mySubClassSet.add(clazz);
         mySubClasses.add(clazz);
      }
   }

   public MeshRegistrationForce() {
      this.beta = DEFAULT_BETA;
   }
   
   /**
    * Initializes the force generator with a given source (model) and target
    * @param source attached to FEM model
    * @param target target mesh
    */
   public abstract void initialize(DynamicMeshComponent source, MeshBase target);

   /**
    * Update any internals required for computing forces
    */
   public abstract void update();

   public abstract DynamicMeshComponent getSource(); 

   /**
    * Retrieves the corresponding point and weight for registration
    * 
    * If the target output x is null, should still return the weight
    * 
    * @param x target correspondence point (output)
    * @param mvtx model vertex
    * @return correspondence weight
    */
   public abstract double getCorrespondence(Point3d x, Vertex3d mvtx);
   
   /**
    * Force scaling parameter
    * @return scale
    */
   public double getForceScaling() {
      return beta;
   }
   
   /**
    * Force scaling parameter
    * @param b scale
    */
   public void setForceScaling(double b) {
      beta = b;
   }

   @Override
   public void applyForces (double t) {
      // f_i = \sum_m w_m Q_i^T dy_m/dq_i^T (x_m - y_m)

      Vector3d f = new Vector3d();
      Point3d xm = new Point3d();

      DynamicMeshComponent model = getSource ();
      
      // over model vertices m
      for (Vertex3d vm : model.getMesh ().getVertices ()) {

         double wm = beta*getCorrespondence (xm, vm);

         if (wm != 0) {
            // world location of model point
            Point3d ym = vm.getWorldPoint ();
            f.sub (xm, ym);
            f.scale (wm);

            // add force to attachment
            PointAttachment pa = model.getVertexAttachment (vm);
            AttachmentForceUtilities.addPointForce(pa, f);
            
            // System.out.println ("v" + vm.getIndex () + ", pos: " + ym.toString ("%.5f") + ", target: " + xm.toString ("%.5f") + ", force: " + f.toString ("%.5f"));
         }
      }
   }

   @Override
   public void addSolveBlocks (SparseNumberedBlockMatrix K) {

      // K_{i,j} = - \sum_m w_m Q_i^T dy_m/dq_i^T dy_m/dq_j Q_j

      DynamicMeshComponent model = getSource ();

      // over model vertices m
      for (Vertex3d vm : model.getMesh ().getVertices ()) {

         double wm = beta*getCorrespondence (null, vm);

         // if the weight is not zero
         if (wm != 0) {
            PointAttachment pa = model.getVertexAttachment (vm);
            AttachmentForceUtilities.addSolveBlocks (K, pa, pa);
         } // non-zero weight
         
      } // vertex
   }

   @Override
   public void addPosJacobian (SparseNumberedBlockMatrix K, double s) {

      // K_{i,j} = - \sum_m w_m Q_i^T dy_m/dq_i^T dy_m/dq_j Q_j

      DynamicMeshComponent model = getSource ();

      // over model vertices m
      for (Vertex3d vm : model.getMesh ().getVertices ()) {

         // compute weight
         double wm = -s*beta*getCorrespondence (null, vm);

         if (wm != 0) {
            // add attachment Jacobian attachment
            PointAttachment pa = model.getVertexAttachment (vm);
            AttachmentForceUtilities.addScaledMulTranspose (K, wm, pa, pa);
            
            // System.out.println ("v" + vm.getIndex () + ", wm: " + wm + ", K: \n" + K.toString ("%.5f"));
         }
      }
   }
   
   @Override
   public void addVelJacobian (SparseNumberedBlockMatrix M, double s) {
      // nothing
   }

   @Override
   public int getJacobianType () {
      return Matrix.SYMMETRIC;
   }

   public static PropertyList myProps = new PropertyList (MeshRegistrationForce.class);
   protected PropertyInfo myPropInfo;
   protected HasProperties myPropHost;
   
   // properties
   static {
      myProps.add ("forceScaling", "force scale factor", DEFAULT_BETA, "[0,inf]");
   }

   @Override
   public PropertyInfoList getAllPropertyInfo () {
      return myProps;
   }

   public PropertyInfo getPropertyInfo() {
      return myPropInfo;
   }

   public void setPropertyInfo(PropertyInfo info) {
      myPropInfo = info;
   }

   public HasProperties getPropertyHost() {
      return myPropHost;
   }

   public void setPropertyHost(HasProperties newParent) {
      myPropHost = newParent;
   }

   public Property getProperty(String name) {
      return PropertyList.getProperty(name, this);
   }

   public boolean hasProperty(String name) {
      return getAllPropertyInfo().get(name) != null;
   }

   public MeshRegistrationForce clone() {
      MeshRegistrationForce func = null;
      try {
         func = (MeshRegistrationForce)super.clone();
      }
      catch (CloneNotSupportedException e) {
         throw new InternalErrorException(
         "cannot clone super in RegistrationPressureFunctionBase");
      }
      return func;
   }

   public boolean isWritable() {
      return false;
   }

   protected void notifyHostOfPropertyChange(String name) {
      if (myPropHost instanceof PropertyChangeListener) {
         ((PropertyChangeListener)myPropHost).propertyChanged(new PropertyChangeEvent(this, name));
      }
   }
}
