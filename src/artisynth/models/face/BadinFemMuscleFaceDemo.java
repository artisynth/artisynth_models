package artisynth.models.face;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import maspack.collision.IntersectionContour;
import maspack.collision.SurfaceMeshContourIxer;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
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
//      setUpperFaceStatic();
//      buildCutPlanes();

      
//    double len = 0.25; // from manual scaling of ct image to fit badinskull mesh
//    addImagePlane(mech, faceGeometryDir+"badinct_sagittal256.png", len,
//       new RigidTransform3d(new Vector3d(0.138072, 0, 0.165425), 
//          new AxisAngle (0.99911, 0.02975, -0.02975,  Math.toRadians(90.051))));//(1,0,0,Math.PI/2.0)));

      
   }
   
   public void setUpperFaceStatic(){
      double tol = 0.13;
      for(FemNode3d n : face.getNodes()){
         if(n.getPosition ().z>tol){
            n.setDynamic (false);
         }
      }
   }
   
   private void buildCutPlanes(){
      Point3d centroid = new Point3d();
      /*Vector3d tangent = new Vector3d();
      Vector3d v1 = face.getNode (2003).getPosition().clone ();
      v1.sub(face.getNode(6391).getPosition().clone());
      Vector3d v2 = face.getNodes ().get ("1054").getPosition().clone ();
      v2.sub (face.getNodes ().get ("1053").getPosition().clone ());
      tangent.cross (v1, v2);*/
      for(int i = 0; i < 5; i++){
         centroid.setZero ();
         centroid.scaledAdd (i/(double)8, face.getNodes ().get ("1054").getPosition ());
         centroid.scaledAdd (i/(double)8, face.getNodes ().get ("1053").getPosition ());
         centroid.scaledAdd ((4-i)/(double)8, face.getNode (6391).getPosition ());
         centroid.scaledAdd ((4-i)/(double)8, face.getNode (2003).getPosition ());
         RotationMatrix3d R = new RotationMatrix3d();
         R.setZDirection (new Vector3d(1,0,0));
         RigidTransform3d T = new RigidTransform3d(centroid,R.getAxisAngle ());
         PolygonalMesh mesh = MeshFactory.createRectangle (0.1, 0.1, true);
         mesh.setMeshToWorld (T);
         MeshComponent comp = new MeshComponent();
         comp.setMesh (mesh);
         mech.add (comp);
      }
   }
   
   public List<Double> getXSectionAreas (boolean recompute) {
      ArrayList<Double> toRet = new ArrayList<Double> ();
      List<Double> areas = new ArrayList<Double>();
      if (recompute) {
         computeXsectionalAreasAndGenerateContours (areas);
      }
      toRet.addAll (areas);
      return toRet;
   }
   
   protected List<Double> computeXsectionalAreasAndGenerateContours (List<Double> areas) {
      SurfaceMeshContourIxer intersector = new SurfaceMeshContourIxer();
      for(int i = 17 ; i < 22 ; i++){
         String comppath = "models/mech/"+Integer.toString(i);
         MeshComponent comp = (MeshComponent)findComponent(comppath);
         boolean collided =
            intersector.findContours (
               (PolygonalMesh)comp.getMesh (),
               face.getSurfaceMesh ());
         if (collided) {
            // areas[i] = useClosestToCenterlineApproach (i);
            areas.add (useSubtractSmallFromBigApproach (intersector));
         }
      }
      return areas;
   }
   
   protected double useSubtractSmallFromBigApproach (SurfaceMeshContourIxer intersector) {
      double finalArea = 0;
      double largestArea = 0;
      int largestAreaContourIdx = -1;
      for (int j = 0; j < intersector.getContours ().size (); j++) {
         IntersectionContour contour = intersector.getContours ().get (j);
         double area = contour.computePlanarArea ();
         if (area > largestArea && contour.isClosed()) {
            largestArea = area;
            largestAreaContourIdx = j;
         }
      }

      if (largestAreaContourIdx == -1) {
         finalArea = 0;
      }
      else {
         finalArea = largestArea; // Add largest area.
         for (int j = 0; j < intersector.getContours ().size (); j++) {
            IntersectionContour contour = intersector.getContours ().get (j);
            if (j != largestAreaContourIdx) {
               if (contour.isClosed ()) {
                  double area = contour.computePlanarArea ();
                  finalArea -= area; // Subtract "island" areas.
               }
            }
         }
      }
      return finalArea;
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
      
      //RenderProps.setVisible (face.getElements(), false);
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
   
//   public void addToExciter(MuscleExciter ex, ArrayList<MuscleElementDesc> elemsToAdd) {
//      for (MuscleElementDesc desc : elemsToAdd) {
//	 ex.addTarget(desc, 1);
//      }
//   }
   
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
