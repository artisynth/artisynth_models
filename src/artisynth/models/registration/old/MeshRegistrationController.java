package artisynth.models.registration.old;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.DynamicMeshComponent;
import artisynth.core.mechmodels.ForceComponent;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.ControllerBase;
import maspack.geometry.MeshBase;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;

/**
 * Dynamic FEM-based registration controller
 */
@Deprecated
public class MeshRegistrationController extends ControllerBase implements ForceComponent, HasProperties {

   public static boolean DEFAULT_ENABLED = true;
   
   DynamicMeshComponent source;
   MeshBase target;
   MeshRegistrationForce force;
   boolean enabled;
   boolean forceAttaching;
   boolean forceAttached;
   
   static PropertyList myProps = new PropertyList (MeshRegistrationController.class, ControllerBase.class);
   static {
      myProps.add ("registrationForce", "registration force computer", createDefaultRegistrationForce(), "CE");
      myProps.add ("enabled isEnabled setEnabled", "enable/disable controller", DEFAULT_ENABLED);
   }
   
   public MeshRegistrationController(DynamicMeshComponent source, MeshBase target) {
      this(source, target, createDefaultRegistrationForce ());
   }
   
   protected MechModel findMechModel() {
      CompositeComponent parent = source.getParent ();
      while (parent != null && !(parent instanceof MechModel)) {
         parent = parent.getParent ();
      }
      return (MechModel)parent;
   }
   
   @Override
   public void connectToHierarchy (CompositeComponent connector) {
      if (connector == getParent()) {
         attachForceEffector ();
      }
      super.connectToHierarchy (connector);
   }
   
   protected void attachForceEffector() {
      if (!forceAttaching) {
         MechModel mech = findMechModel ();
         forceAttaching = true;
         if (mech != null) {
            mech.addForceEffector (this);
            forceAttached = true;
         }
         forceAttaching = false;
      }
   }
   
   protected void detachForceEffector() {
      if (!forceAttaching) {
         MechModel mech = findMechModel ();
         forceAttaching = true;
         if (mech != null) {
            mech.removeForceEffector (this);
            forceAttached = false;
         }
         forceAttaching = false;
      }
   }
   
   @Override
   public void disconnectFromHierarchy (CompositeComponent connector) {
      if (connector == getParent()) {
         detachForceEffector();
      }
      super.disconnectFromHierarchy (connector);
   }
   
   public MeshRegistrationController(DynamicMeshComponent source, MeshBase target, MeshRegistrationForce force) {
      this.forceAttaching = false;
      this.forceAttached = false;
      this.source = source;
      this.target = target;
      this.force = force;
      this.force.initialize (source, target);
      this.enabled = DEFAULT_ENABLED;
   }
   
   public DynamicMeshComponent getSource() {
      return source;
   }
   
   /**
    * Force scaling parameter
    * @return scale
    */
   public double getForceScaling() {
      return getRegistrationForce ().getForceScaling ();
   }
   
   /**
    * Force scaling parameter
    * @param b scale
    */
   public void setForceScaling(double b) {
      getRegistrationForce ().setForceScaling (b);
   }
   
   public static void addControls(
      ControlPanel controlPanel, MeshRegistrationController controller) {
      
      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create (prop);
         controlPanel.addWidget(widget);
      }
   }

   private static MeshRegistrationForce createDefaultRegistrationForce() {
      return new PressureMeshRegistrationForce ();
   }
   
   /**
    * Set object for computing the registration forces
    * @param force generates registration forces
    */
   public void setRegistrationForce(MeshRegistrationForce force) {
      if (this.force != force) {
         this.force = force.clone ();
         this.force.initialize (source, target);
      }
   }
   
   /**
    * Retrieves the object that computes registration forces
    * @return force generator
    */
   public MeshRegistrationForce getRegistrationForce() {
      return force;
   }
   
   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   
   @Override
   public void apply (double t0, double t1) {
      if (isEnabled ()) {
         // try attaching now if not already attached
         if (!forceAttached) {
            attachForceEffector ();
         }
         
         //         FunctionTimer ft = new FunctionTimer ();
         //         ft.start ();
         force.update ();
         //         ft.stop ();
         //         double usec = ft.getTimeUsec ();
         //         System.out.println (getName () + ".apply(),\n    " + usec + " us");
      }
   }
   
   /**
    * Controller is enabled
    * @return true if enabled
    */
   public boolean isEnabled() {
      return enabled;
   }
   
   /**
    * Set controller enabled or not
    * @param set enables controller if set
    */
   public void setEnabled(boolean set) {
      enabled = set;
   }

   @Override
   public void applyForces (double t) {
      if (isEnabled()) {
         force.applyForces (t);
      }
   }

   @Override
   public void addSolveBlocks (SparseNumberedBlockMatrix M) {
      if (isEnabled ()) {
         force.addSolveBlocks (M);
      }
   }

   @Override
   public void addPosJacobian (SparseNumberedBlockMatrix M, double s) {
      if (isEnabled ()) {
         force.addPosJacobian (M, s);
      }
   }

   @Override
   public void addVelJacobian (SparseNumberedBlockMatrix M, double s) {
      if (isEnabled ()) {
         force.addVelJacobian (M, s);
      }
   }

   @Override
   public int getJacobianType () {
      return force.getJacobianType ();
   }

}
