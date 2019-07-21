package artisynth.models.face;

import java.awt.Color;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;


public class RefFemMuscleFaceDemo extends RefFaceDemo{

   public double muscleThickness = 0.003;
   
   Plane midSagittalPlane = new Plane(Vector3d.X_UNIT, 0);
   
   
   public static PropertyList myProps = new PropertyList(
	 RefFemMuscleFaceDemo.class, RefFaceDemo.class);

   static {
      myProps.add("muscleThickness * *",
	    "distance used to compute muscle elements from fibers", 0.0,
	    "[0,0.01]");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public RefFemMuscleFaceDemo() {
      super();

   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      

      setFibresActive(false);
      setMuscleElements(muscleThickness);
//      loadAnsysOOP();
      loadMuscleElements(face, face.getMuscleBundles().get("OOP"), 
	    refFaceGeometryDir+"oop/oldoop_elements.txt");
      
      face.getMuscleBundles().get("OOP").computeElementDirections();
      
      ((GenericMuscle)face.getMuscleMaterial()).setMaxStress(100000);
      face.setDirectionRenderLen(0.5);
      //face.setElementWidgetSize(0);
      RenderProps.setLineWidth(face.getElements(), 0);
      // face.setSubSurfaceRendering(false);
   }
   
   public void setFibresActive(boolean active) {
      for (MuscleBundle b : face.getMuscleBundles()) {
	 b.setFibresActive(active);
      }
   }
   
   public void setMuscleElements(double muscleThickness) {
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
   
   private static Muscle addMuscleFiber(MuscleBundle bundle, Point p0, Point p1) {
      Muscle m = new Muscle();
      m.setFirstPoint(p0);
      m.setSecondPoint(p1);
      bundle.addFibre(m);
      AnsysFaceMuscleFiberReader.setDefaultMuscleFibreProperties(m);
      return m;
   }

   
   public static void loadMuscleElements(FemMuscleModel face, MuscleBundle bundle, String filename) {
      bundle.getElements().clear();
      addMuscleElements (face, bundle, filename);
   }
   
   public static void addMuscleElements(FemMuscleModel face, MuscleBundle bundle, String filename) {
      for (Integer id : BadinFaceDemo.readIntList(filename)) {
         MuscleElementDesc desc = new MuscleElementDesc();
         FemElement3d elem = face.getElements().getByNumber(id);
         desc.setElement(elem);
         bundle.addElement(desc);
      }
   }
   
   public void loadoop(String name) {
      RefFemMuscleFaceDemo.loadMuscleElements(face, face.getMuscleBundles().get("OOP"),
	    refFaceGeometryDir+"oop/"+name+"_elements.txt");
      
      RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, face.getMuscleBundles().get("OOP"));
   }
   
   public void addoop(String name) {
      MuscleBundle oop = face.getMuscleBundles().get("OOP");
      String filename = refFaceGeometryDir+"oop/"+name+"_elements.txt";
      for (Integer id : BadinFaceDemo.readIntList(filename)) {
	 MuscleElementDesc desc = new MuscleElementDesc();
	 FemElement3d elem = face.getElements().getByNumber(id);
	 desc.setElement(elem);
	 oop.addElement(desc);
      }
      
      oop.computeElementDirections();
   }
   
   public void diroop() {
      RefFemMuscleFaceDemo.setFibresFromElementCentroids(face, face.getMuscleBundles().get("OOP"));
   }
   
   public void loadOOPElements() {
      loadMuscleElements(face, face.getMuscleBundles().get("OOP"), 
	    refFaceGeometryDir+"OOP_7thRing_elements.txt");
   }
   
   public void loadAnsysOOP() {
      MuscleBundle oop = face.getMuscleBundles().get("OOP");
      loadMuscleElements(face, oop, 
	    refFaceGeometryDir+"OOP_7thRing_elements.txt");
      try {
	 setDirectionsFromFile(oop, refFaceGeometryDir+"oop_elem_type.txt", 
	       refFaceGeometryDir+"oop_type_dir.txt");
      } catch (IOException e) {
	 e.printStackTrace();
      }
   }
   
   public static void loadDirections(MuscleBundle bundle, String filename) {
      ArrayList<Vector3d> dirs = new ArrayList<Vector3d>();
      ReaderTokenizer rtok;
      try {
	 rtok = new ReaderTokenizer(new FileReader(filename));
	 while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	    rtok.pushBack();
	    Vector3d dir = new Vector3d();
	    dir.scan(rtok);
	    dirs.add(dir);
	 }
      } catch (IOException e) {
	 e.printStackTrace();
      }
      
      if (bundle.getElements().size() == dirs.size()) {
	 for (int i = 0; i < dirs.size(); i++) {
	    bundle.getElements().get(i).setDirection(dirs.get(i));
	 }
      }
   }
   
   public static void setFibresFromElementCentroids(FemMuscleModel fem, MuscleBundle bundle) {
      ArrayList<FemElement3dBase> elems = new ArrayList<>();
      for (MuscleElementDesc desc : bundle.getElements()) {
	 elems.add(desc.getElement());
      }
      setFibresFromElementCentroids(fem, bundle, elems);
   }
   
   
   public static void setFibresFromElementCentroids(FemMuscleModel face, MuscleBundle bundle, String filename) {
      ArrayList<FemElement3d> elems = new ArrayList<FemElement3d>();
      for (Integer id : BadinFaceDemo.readIntList(filename)) {
	 MuscleElementDesc desc = new MuscleElementDesc();
	 FemElement3d elem = face.getElements().getByNumber(id);
	 desc.setElement(elem);
	 bundle.addElement(desc);
      }
//      setFibresFromElementCentroids(face, bundle);
   }
   
   public static void setFibresFromElementCentroids(FemMuscleModel fem, MuscleBundle bundle, ArrayList<FemElement3dBase> elems) {
      bundle.getFibres().clear();
      Point3d centroid = new Point3d();
      FemMarker first = null, prev = null;
      for (FemElement3dBase elem : elems) {
	 if (elem instanceof WedgeElement) 
	    continue; // ignore wedges because centroids are offset
	 IntegrationPoint3d warpingPnt = elem.getWarpingPoint();
	 warpingPnt.computeRestPosition (centroid, elem.getNodes());
	 FemMarker mkr = new FemMarker(elem, centroid);
	 fem.addMarker(mkr);
	 if (first == null) {
	    first = mkr;
	 }
	 else {
	    addMuscleFiber(bundle, prev, mkr);
	 }
	 prev = mkr;
      }
      addMuscleFiber(bundle, prev, first);    
      bundle.computeElementDirections();
   }
   
   public static void setDirectionsFromFile (MuscleBundle bundle, String elemFilename, String dirFilename) throws IOException {
      ReaderTokenizer rtok;
      rtok = new ReaderTokenizer(new FileReader(elemFilename));
      HashMap<Integer, Integer> elem2Type = new HashMap<Integer, Integer>();
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 rtok.pushBack();
	 int elem = rtok.scanInteger();
	 int type = rtok.scanInteger();
	 elem2Type.put(elem, type);
      }
      rtok.close();
      System.out.println("num elems = "+elem2Type.size());
      
      HashMap<Integer, Vector3d> type2Dir = new HashMap<Integer, Vector3d>();

      rtok = new ReaderTokenizer(new FileReader(dirFilename));
      while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
	 rtok.pushBack();
	 int type = rtok.scanInteger();
	 Vector3d dir = new Vector3d();
	 dir.scan(rtok);
	 type2Dir.put(type, dir);
      }
      System.out.println("num types = "+type2Dir.size());
      
      for (MuscleElementDesc desc : bundle.getElements()) {
	 int elemId = desc.getElement().getNumber();
	 Vector3d dir = type2Dir.get(elem2Type.get(elemId));
	 desc.setDirection(dir); 
      }
      
   }
   

   /*
    * helper methods to show OOP muscle elements from original ANSYS data
    */
   
   public void loadAnsysOOPElements() {
      
      String elemfile = "OOP_active_elements.elem";
      String nodefile = "OOP_active_node.node";
      
      showNodes(nodefile);
      showElems(elemfile);
      
   }
   
   public void showElems (String elemfile) {
      double ews = 0.8f;
      try {
	 LinkedHashMap<Integer, ArrayList<Integer>> elemMap = AnsysReader.readElemFile(new FileReader(refFaceGeometryDir+elemfile), true);
	 for (Integer id : elemMap.keySet()) {
	    FemElement3d leftElem = face.getElements().getByNumber(id);
	    if (leftElem != null) {
	       leftElem.setElementWidgetSize(ews);
	       FemElement3d rightElem = findSymmetricElement(leftElem, midSagittalPlane);
	       if (rightElem != null) {
		  rightElem.setElementWidgetSize(ews);
	       }
	       else {
		       System.out.println("null symmetric OOP elem");
	       }
	    }
	    else {
	       System.out.println("null OOP elem");
	    }
	       
	 }
      } catch (IOException e) {
	 e.printStackTrace();
      }
   }
   
   public void showNodes(String nodefile) {
      
      try {
	 LinkedHashMap<Integer, Point3d> nodeMap = AnsysReader.readNodeFile(new FileReader(refFaceGeometryDir+nodefile), true);
	 for (Integer id : nodeMap.keySet()) {
	    Point3d pos = new Point3d(nodeMap.get(id));
	    pos.scale(mm2m);
	    FemNode3d n = face.getNodes().getByNumber(id);
	    if (n != null) {
	       RenderProps.setPointStyle(n, PointStyle.SPHERE);
	       RenderProps.setPointColor(n, Color.CYAN);
	    }
	 }
      } catch (IOException e) {
	 e.printStackTrace();
      }
   }
   
   public FemElement3d findSymmetricElement(FemElement3d elem,
	 Plane midSagittalPlane) {
      FemElement3d rightElem = null;
      FemNode3d[] elemNodes = new FemNode3d[elem.numNodes()];
      for (int i = 0; i < elem.numNodes(); i++) {
	 FemNode3d n = elem.getNodes()[i];
	 FemNode3d rn = findNode(face, getReflectedPosition(n.getPosition(), midSagittalPlane, BadinFaceDemo.midsagittalTol), BadinFaceDemo.midsagittalTol);
	 if (rn == null) {
	    System.err.println("null rightside node OOP elements");
	    elemNodes = null;
	    break;
	 }
	 elemNodes[i] = rn;
      }
      if (elemNodes != null) {
	 rightElem = findElement(face, elemNodes);
      }
      return rightElem;
   }
   
   public static FemElement3d findElement(FemModel3d fem, FemNode3d[] nodes) {
      for (FemElement3d e : fem.getElements()) {
	 FemNode3d[] enodes = e.getNodes();
	 for (int i = 0; i < e.numNodes(); i++) {
	    if (match(enodes, nodes)) {
	       return e;
	    }
	 }
      }
      return null;
   }
   
   public static boolean match (FemNode3d[] n0, FemNode3d[] n1){
      boolean match = true;
      for (int i = 0; i < n0.length; i++) {
	 boolean found = false;
	 for (int j = 0; j < n1.length; j++) {
	    if (n0[i] == n1[j]) {
	       found = true;
	       break;
	    }
	 }
	 if (!found) {
	    match = false;
	    break;
	 }
      }
      return match;
   }
   
   public static Point3d getReflectedPosition(Point3d pos, Plane reflectionPlance, double tol) {
      Point3d reflect = new Point3d();
      double dist = reflectionPlance.distance(pos);
      if (dist < -tol) {
	 reflectionPlance.reflect(reflect, pos);
      }
      else {
	 reflect.set(pos);
      }
      return reflect;
   }
   
   public static FemNode3d findNode(FemMuscleModel fem, Point3d pos, double tol) {
      FemNode3d n = null;
      Point3d dist = new Point3d();
      for (int i = 0; i < fem.numNodes(); i++) {
	 dist.sub(pos, fem.getNode(i).getPosition());
	 if (dist.norm() < tol) {
	    n = fem.getNode(i);
	    break;
	 }
      }
      return n;
   }
}


