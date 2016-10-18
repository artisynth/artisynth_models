package artisynth.models.tongue3d;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.Renderer.Shading;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SolidJoint;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.workspace.DriverInterface;

public class TongueLollipop extends FemMuscleTongueDemo {

   RigidBody lolliStick = null;
   RigidBody lolliHead = null;
   
   public static PropertyList myProps =
      new PropertyList(TongueLollipop.class, FemMuscleTongueDemo.class);

   private double dz = 0;
   private double zOrig = 0;
   private boolean drawCollisions = true;
   
   static {
      myProps.add("lolliDeltaZ * *", "z translation of lollipop", 0, "NW [-0.05,0.05]");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      
      createLollipop();
      addCollisions();
      adjustTongueParameters();
      setMaxStepSize(0.005);
      
      fixLollipop();
      
      setupAdditionalRenderProps();
   }
   
   protected void setLollipopMass() {
      RigidBody lolliHead = mech.rigidBodies().get("lollipop_head");
      RigidBody lolliStick = mech.rigidBodies().get("lollipop_stick");
      lolliHead.setDensity(10);
      lolliStick.setDensity(10);
      
   }
   
   protected void adjustTongueParameters() {
      tongue.setMaterial(new LinearMaterial(10000,0.45));
      ArrayList<MuscleBundle> bundles = new ArrayList<MuscleBundle>();
      bundles.addAll(tongue.getMuscleBundles());
      for (MuscleBundle bundle : bundles) {
         tongue.removeMuscleBundle(bundle);   
      }
      
   }
   
   protected void fixLollipop() {
      lolliHead.setDynamic(false);
      lolliStick.setDynamic(false);
      
      RigidTransform3d pose = lolliHead.getPose();
      RigidTransform3d trans = new RigidTransform3d(0,0,-0.01);
      pose.mul(trans, pose);
      lolliHead.setPose(pose);
      lolliStick.setPose(pose);
   }
   
   protected void addCollisions() {
      mech.setCollisionBehavior(tongue, lolliHead, true);
      mech.setCollisionBehavior(lolliStick, tongue, true);
   }
   
   public void setupAdditionalRenderProps() {
      if (lolliHead != null) {
         RenderProps.setFaceColor(lolliHead, Color.GREEN);
         RenderProps.setShading(lolliHead, Shading.SMOOTH);
         RenderProps.setAlpha(lolliHead, 0.9);
      }
      if (lolliStick != null) {
         RenderProps.setFaceColor(lolliStick, Color.WHITE);
      }
      
      tongue.setSurfaceRendering(SurfaceRender.Shaded);
      setCollisionRenderProps();
      
   }
   
   protected void setCollisionRenderProps() {

      CollisionManager collisions = mech.getCollisionManager();
      RenderProps.setVisible(collisions, drawCollisions);
      RenderProps.setEdgeWidth(collisions, 4);
      RenderProps.setEdgeColor(collisions, Color.BLUE);
      RenderProps.setLineWidth(collisions, 3);
      RenderProps.setLineColor(collisions, Color.YELLOW);
      collisions.setContactNormalLen(0.001);
      collisions.setDrawContactNormals(true);
   }
   
   protected void createLollipop() {
      RigidTransform3d lolliTrans = new RigidTransform3d();
      lolliTrans.setRotation(new AxisAngle(0, -1, 0, Math.toRadians(25)));
      lolliTrans.setTranslation(new Vector3d(0.07, 0, 0.13));
      
      PolygonalMesh stick = MeshFactory.createCylinder(0.001, 0.05, 20);
      RigidTransform3d stickTrans = new RigidTransform3d();
      stickTrans.setRotation(new AxisAngle(0,1,0,Math.PI/2));
      stickTrans.setTranslation(new Vector3d(-0.05/2,0,0));
      stick.transform(stickTrans);
      
      
      PolygonalMesh head = MeshFactory.createCylinder(0.0125, 0.004, 20);
      Face topFace = head.getFaces().get(head.numFaces()-1);
      Face bottomFace = head.getFaces().get(head.numFaces()-2);
      MeshFactory.triangulateFaceCentroid(topFace);
      MeshFactory.triangulateFaceCentroid(bottomFace);
      
      head.triangulate();
      stick.triangulate();
      
      RigidBody rbStick = new RigidBody("lollipop_stick");
      rbStick.setMesh(stick, null);
      RigidBody rbHead = new RigidBody("lollipop_head");
      rbHead.setMesh(head, null);
      
      SolidJoint weld = new SolidJoint(rbStick, rbHead);
      
      rbStick.setPose(lolliTrans);
      rbHead.setPose(lolliTrans);
      
      mech.addRigidBody(rbHead);
      mech.addRigidBody(rbStick);
      mech.addBodyConnector(weld);
      lolliHead = rbHead;
      lolliStick = rbStick;
      
   }
   
   protected void addLollipopProbe() {
      NumericInputProbe p = new NumericInputProbe(this, "lolliDeltaZ", 0, 5);
      p.addData(0, new double[] {0.01});
      p.addData(1.25, new double[] {-0.02});
      p.addData(2.5, new double[] {0.0});
      p.addData(3.75, new double[] {-0.02});
      p.addData(5, new double[] {0.01});
      addInputProbe(p);
      setLolliDeltaZ(0.01);
   }
   
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      zOrig = lolliHead.getPosition().z;
      removeAllInputProbes();
      removeAllWayPoints();
      addLollipopProbe();
   }
    
   public void setLolliDeltaZ(double z) {
      dz = z;
      Point3d pos = lolliHead.getPosition();
      pos.z = zOrig + dz;
      lolliHead.setPosition(pos);
      lolliStick.setPosition(pos);
      
   }
   
   public double getLolliDeltaZ() {
      return dz;
   }
   
}
