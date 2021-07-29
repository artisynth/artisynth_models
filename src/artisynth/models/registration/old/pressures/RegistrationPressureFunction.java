package artisynth.models.registration.old.pressures;

import java.util.HashSet;

import artisynth.core.modelbase.PropertyChangeEvent;
import artisynth.core.modelbase.PropertyChangeListener;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.CompositeProperty;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.util.DynamicArray;
import maspack.util.InternalErrorException;

/**
 * Computes a desired pressure (N/d^2 - where d is the current distance unit) to be applied on the
 * source model surface to drive it towards the target.
 */
public abstract class RegistrationPressureFunction implements CompositeProperty {

   static HashSet<Class<?>> mySubClassSet = new HashSet<>();
   static DynamicArray<Class<?>> mySubClasses =
   new DynamicArray<>(new Class<?>[0]);

   static {
      registerSubClass(GaussianPressureFunction.class);
      registerSubClass(GravityPressureFunction.class);
      registerSubClass(LogNormalPressureFunction.class);
      registerSubClass(SpringPressureFunction.class);
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

   /**
    * Computes the desired pressure between two points of correspondence
    * @param pSource point on the source surface
    * @param nSource normal on the source surface
    * @param pTarget point on the target surface
    * @param nTarget normal on the target surface
    * @param pressure output pressure
    */
   public abstract void computePressure(Point3d pSource, Vector3d nSource, Point3d pTarget, Vector3d nTarget, Vector3d pressure);

   protected PropertyInfo myPropInfo;
   protected HasProperties myPropHost;

   public static PropertyList myProps = new PropertyList(RegistrationPressureFunction.class);

   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   @Override
   public Property getProperty(String name) {
      return PropertyList.getProperty(name, this);
   }

   @Override
   public RegistrationPressureFunction clone() {
      RegistrationPressureFunction func = null;
      try {
         func = (RegistrationPressureFunction)super.clone();
      }
      catch (CloneNotSupportedException e) {
         throw new InternalErrorException(
         "cannot clone super in RegistrationPressureFunction");
      }
      return func;
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

   public boolean hasProperty(String name) {
      return getAllPropertyInfo().get(name) != null;
   }

   public boolean isWritable() {
      return false;
   }

   protected void notifyHostOfPropertyChange(String name) {
      if (myPropHost instanceof PropertyChangeListener) {
         ((PropertyChangeListener)myPropHost)
         .propertyChanged(new PropertyChangeEvent(this, name));
      }
   }

}
