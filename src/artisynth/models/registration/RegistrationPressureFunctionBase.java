package artisynth.models.registration;

import java.util.HashSet;

import artisynth.core.modelbase.ModelComponentBase;
import artisynth.core.modelbase.PropertyChangeEvent;
import artisynth.core.modelbase.PropertyChangeListener;
import maspack.properties.CompositeProperty;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.util.DynamicArray;
import maspack.util.InternalErrorException;

public abstract class RegistrationPressureFunctionBase
   extends ModelComponentBase
   implements RegistrationPressureFunction, CompositeProperty {

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

   public static PropertyList myProps =
      new PropertyList(RegistrationPressureFunctionBase.class);

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   protected PropertyInfo myPropInfo;
   protected HasProperties myPropHost;

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

   public RegistrationPressureFunctionBase clone() {
      RegistrationPressureFunctionBase func = null;
      try {
         func = (RegistrationPressureFunctionBase)super.clone();
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
         ((PropertyChangeListener)myPropHost)
            .propertyChanged(new PropertyChangeEvent(this, name));
      }
   }

}
