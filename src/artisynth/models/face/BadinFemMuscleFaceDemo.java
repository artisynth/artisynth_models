package artisynth.models.face;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.tongue3d.FemMuscleTongueDemo;

public class BadinFemMuscleFaceDemo extends BadinFaceDemo{

   public static double muscleThickness = 0.006;
   
   public static PropertyList myProps = new PropertyList(
	 BadinFemMuscleFaceDemo.class, BadinFaceDemo.class);

   static {
      myProps.add("muscleThickness * *",
	    "distance used to compute muscle elements from fibers", 0.0,
	    "[0,0.01]");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public BadinFemMuscleFaceDemo() {
      super();

   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      setFibresActive(false);
      setMuscleElements(muscleThickness);
      setDefaultOOP();
      
      FemMuscleTongueDemo.addExcitersFromFiles(face, 
	    ArtisynthPath.getSrcRelativePath(this, "/exciters"));

      
      ((GenericMuscle)face.getMuscleMaterial()).setMaxStress(100000);
      face.setDirectionRenderLen(0.5);
      face.setElementWidgetSize(1);
      RenderProps.setLineWidth(face.getElements(), 0);
      RenderProps.setLineColor(face, new Color(.2f, .2f, .2f));
      // face.setSubSurfaceRendering(false);
      RenderProps.setVisible(face.markers(), false);
      RenderProps.setVisible(face.getMuscleBundles(), true);

      
//    double len = 0.25; // from manual scaling of ct image to fit badinskull mesh
//    addImagePlane(mech, faceGeometryDir+"badinct_sagittal256.png", len,
//       new RigidTransform3d(new Vector3d(0.138072, 0, 0.165425), 
//          new AxisAngle (0.99911, 0.02975, -0.02975,  Math.toRadians(90.051))));//(1,0,0,Math.PI/2.0)));

      
   }
   
   public void setDefaultOOP() {
      // use 7M fiber directions
      // add elements for 7th upper, 6th lower ring
      
      MuscleBundle oop = face.getMuscleBundles().get("OOP");
      loadoop("7M");
//      oop.clearElements();
//      
//      MuscleExciter ex = new MuscleExciter("OOPu");
//      face.addMuscleExciter(ex);
//      addToExciter(ex, addoop("7Du"));
//      addToExciter(ex, addoop("7Mu"));
//      addToExciter(ex, addoop("7Su"));
//      
//      ex = new MuscleExciter("OOPl");
//      face.addMuscleExciter(ex);
//      addToExciter(ex, addoop("6Dl"));
//      addToExciter(ex, addoop("6Ml"));
//      addToExciter(ex, addoop("6Sl"));


      oop.setElementWidgetSize(1);
      RenderProps.setVisible(oop, true);
      
      RenderProps.setVisible (face.getElements(), false);
      RenderProps.setVisible (face.getMuscleBundles(), false);
      RenderProps.setVisible (face.getNodes(), false);
      
      face.setIncompressible (IncompMethod.OFF);

   }
   
   public void loadoop(String name) {
      RefFemMuscleFaceDemo.loadMuscleElements(face, face.getMuscleBundles().get("OOP"),
	    faceGeometryDir+"oop/"+name+"_elements.txt");
      
      RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, face.getMuscleBundles().get("OOP"));
   }
   
   public static RigidBody addImagePlane(MechModel mech, String imageFileName,
	 double len, RigidTransform3d pose) {
      RigidBody image = new RigidBody("ctimage");
      image.setDynamic(false);
      image.setPose(pose);
      image.setMesh(
         MeshFactory.createRectangle(len, len, /*textureCoords=*/true), null);
      RenderProps.setColorMapFileName(image, imageFileName);
      RenderProps.setColorMapEnabled(image, true);
      RenderProps.setFaceColor(image, Color.WHITE);
      mech.addRigidBody(image);
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(len/2,-len/2,0));
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(len/2,len/2,0));
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(-len/2,-len/2,0));
      mech.addFrameMarker (new FrameMarker(), image, new Point3d(-len/2,len/2,0));
      return image;
   }
   
   public void addToExciter(MuscleExciter ex, ArrayList<MuscleElementDesc> elemsToAdd) {
      for (MuscleElementDesc desc : elemsToAdd) {
	 ex.addTarget(desc, 1);
      }
   }
   
   public ArrayList<MuscleElementDesc> addoop(String name) {
      MuscleBundle oop = face.getMuscleBundles().get("OOP");
      ArrayList<MuscleElementDesc> muscleElems = new ArrayList<MuscleElementDesc>();
      String filename = faceGeometryDir+"oop/"+name+"_elements.txt";
      for (Integer id : BadinFaceDemo.readIntList(filename)) {
	 MuscleElementDesc desc = new MuscleElementDesc();
	 FemElement3d elem = face.getElements().getByNumber(id);
	 desc.setElement(elem);
	 oop.addElement(desc);
	 muscleElems.add(desc);
      }
      oop.computeElementDirections();
      return muscleElems;
   }
   
   public void diroop() {
      RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, face.getMuscleBundles().get("OOP"));
   }
   
   public void setFibresActive(boolean active) {
      for (MuscleBundle b : face.getMuscleBundles()) {
	 b.setFibresActive(active);
      }
   }

   public void setMuscleElements(double muscleThickness) {
      setMuscleElements(face, muscleThickness);
   }

   public static void setMuscleElements(FemMuscleModel face,
	 double muscleThickness) {
      for (MuscleBundle b : face.getMuscleBundles()) {
	 b.clearElements();
	 b.addElementsNearFibres(muscleThickness);
	 b.computeElementDirections();
      }
   }
   
   
   public double getMuscleThickness() {
      return muscleThickness;
   }

   public void setMuscleThickness(double size) {
      muscleThickness = size;
      setMuscleElements(muscleThickness);
   }

   
   @Override
   public void attach(DriverInterface driver) {
      // TODO Auto-generated method stub
      super.attach(driver);
      if (options != null) {
	 options.addWidget("oop visisble", face, "bundles/OOP:renderProps.visible");
	 options.addWidget("oop elems visisble", face, "bundles/OOP/elementDescs:renderProps.visible");
      }
   }

   /*
    *  transform  to CT data space for Yohan's visualization software
    */
   public void makeTransformedMeshes() {
      

      RigidTransform3d X = new RigidTransform3d();
      X.set(new double[][]{
      new double[]{0.9981347984218668, 6.765421556309548E-17, 0.06104853953485623, 0.0057170448252484435},
      new double[]{-9.540979117872439E-17, 0.9999999999999998, 1.3877787807814457E-16, -0.12648000000000004},
      new double[]{-0.06104853953485628, -2.42861286636753E-16, 0.998134798421867, 0.03279229226171611},
      new double[]{0.0, 0.0, 0.0, 1.0}});
      X.invert();
      
      RigidTransform3d zup9mm = new RigidTransform3d(new Vector3d(0, 0, 0.009), new AxisAngle());

      try {
      PolygonalMesh mesh;

      mesh = new PolygonalMesh(new File("/Users/stavness/Desktop/badin_maxilla_teeth_extended_lateral_fixed.obj"));
      mesh.transform(X);
      mesh.transform(zup9mm);
      mesh.write(new PrintWriter("/Users/stavness/Desktop/badin_maxilla_teeth_extended_lateral_fixed_transformed.obj"), "%g");

      mesh = new PolygonalMesh(new File("/Users/stavness/Desktop/badin_jaw_teeth_condyles_extended.obj"));
      mesh.transform(X);
      mesh.transform(zup9mm);
      mesh.write(new PrintWriter("/Users/stavness/Desktop/badin_jaw_teeth_condyles_extended_transformed.obj"), "%g");
      
      }
      catch (IOException e) {
	 e.printStackTrace();
      }
      
      face.transformGeometry(X);
      face.transformGeometry(zup9mm);
      
      
      for (RigidBody rb : mech.rigidBodies()) {
	 if (rb.getMesh() != null) {
	    rb.getMesh().transform(X);
	 }
      }
      
      for (RigidBody rb : mech.rigidBodies()) {
	 if (rb.getMesh() != null) {
	    rb.getMesh().transform(zup9mm);
	 }
      }
      
   }
   
}
