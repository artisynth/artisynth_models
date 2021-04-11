package artisynth.models.registration.correspondences;

import java.util.HashSet;

import artisynth.core.modelbase.ModelComponentBase;
import artisynth.core.modelbase.PropertyChangeEvent;
import artisynth.core.modelbase.PropertyChangeListener;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.properties.CompositeProperty;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.util.DynamicArray;
import maspack.util.InternalErrorException;

/**
 * Base-class that allows class registration for use in property panels
 */
public abstract class MeshCorrespondenceComputer 
   extends ModelComponentBase implements CompositeProperty {

   static HashSet<Class<?>> mySubClassSet = new HashSet<>();
   static DynamicArray<Class<?>> mySubClasses = new DynamicArray<>(new Class<?>[0]);

   static {
      registerSubClass(ICPMeshCorrespondence.class);
      registerSubClass(GMMMeshCorrespondence.class);
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
   
   public static PropertyList myProps = new PropertyList (MeshCorrespondenceComputer.class);
   protected PropertyInfo myPropInfo;
   protected HasProperties myPropHost;
   
   @Override
   public PropertyList getAllPropertyInfo () {
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

   public MeshCorrespondenceComputer clone() {
      MeshCorrespondenceComputer func = null;
      try {
         func = (MeshCorrespondenceComputer)super.clone();
      }
      catch (CloneNotSupportedException e) {
         throw new InternalErrorException(
         "cannot clone super in " + this.getClass ().getName ());
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
   
   /**
    * Initialize correspondence data for a given source and target mesh
    * @param source source mesh
    * @param target target mesh
    */
   public abstract void initialize(MeshBase source, MeshBase target);
   
   /**
    * Update correspondences internally
    * @param t0 time at start of time-step
    * @param t1 time at end of time-step
    */
   public abstract void apply(double t0, double t1);
   
   /**
    * Retrieves the computer correspondence and weight for provided mesh vertex
    * @param c    output correspondence position
    * @param vtx  model vertex
    * @return     correspondence weight
    */
   public abstract double getCorrespondence(Point3d c, Vertex3d vtx);
   
}
