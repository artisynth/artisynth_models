package artisynth.models.jawTongue;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.net.InetAddress;
import java.util.ArrayList;

import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.collision.SurfaceMeshCollider;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.Renderer;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;
import maspack.render.Renderer.DrawMode;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.workspace.DriverInterface;

import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;

public class AirwayXsectionDemo extends AirwaySkinDemo {
   public static final double[][] PlaneInfo = new double[][]{
                          {50.6, 97.7, 50.097},
                          {59.5, 104.6, 45.25},
                          {70.7, 111.6, 63.53},
                          {84.55, 117.9, 85.36},
                          {105.1, 117.0, 102.65},
                          {120.8, 109.3, 136.26},
                          {129.6, 100.9, 143.3},
                          {136.9, 85.22, 179.78},
                          {138.16, 74.64, 194.76},
                          {139.01, 63.64, 186.18}, // periform
                          {136.04, 44.996, 182.74}};
// double[][] PlaneInfo = new double[][]{{120.8, 109.3, 46.26}};
   
   public static final double MINIMUM_TUBE_AREA = 1e-4;
   
   public static final double anteriorAngle = 45.0;
   public static final double posteriorAngle = 180.0;
   public static final int numXsections = 20;
   public static final boolean smoothXsectionAngles = false;

   RenderableComponentList<MeshComponent> planes = new RenderableComponentList<MeshComponent> (MeshComponent.class, "xsectionPlanes");
   Polyline centerline;
   PolylineMesh centerlineMesh;
   MeshComponent centerlineMeshComponent;
   //SurfaceMeshCollider collider = new SurfaceMeshCollider ();
   
   OSCPortOut oscSender1; //for sending areas to jass synth
   OSCPortOut oscSender2; //for sending twomass params
   
   protected static final boolean defaultUseExplicitLipOpening = false;
   protected boolean useExplicitLipOpening = defaultUseExplicitLipOpening;
   
   
   protected static final double defaultLipOpening = 0.1;
   protected double lipOpening = defaultLipOpening;
   
   protected static final double defaultTwoMassLung = 700.0;
   protected double twoMassLung = defaultTwoMassLung;
   
   /// PROPERTIES ///

   public static PropertyList myProps = new PropertyList (
      AirwayXsectionDemo.class, AirwaySkinDemo.class);

   static {
      myProps.add (
         "useExplicitLipOpening * *", "override computed cross-sectional area and use the value provided by the lipOpening property",
         defaultUseExplicitLipOpening);
      myProps.add (
         "lipOpening * *", "cross sectional area for lip opening",
         defaultLipOpening);
      myProps.add (
         "twoMassLung * *", "two mass lung parameter", defaultTwoMassLung);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   public AirwayXsectionDemo () throws IOException {
	   super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      RenderProps.setEdgeColor (this, Color.CYAN);
      RenderProps.setEdgeWidth (this, 6);
      
      myJawModel.add(planes);
      
      //OSC setup;
      int OSC_PORT = 12000;
      String OSC_HOST = "127.0.0.1";
      oscSender1 = new OSCPortOut(InetAddress.getByName(OSC_HOST), OSC_PORT);
      OSC_PORT = 12002;//for twomass
      oscSender2 = new OSCPortOut(InetAddress.getByName(OSC_HOST), OSC_PORT);
      System.out.println("sending OSC tube areas to " + OSC_HOST + ":" + OSC_PORT);
   }

   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);

//      addXsectionPlanesFromList ();
      createCenterlineFromList ();
      
//      loadCenterlineFromFile(regGeomDir+"badinairwaylips_morphed_centerline.obj");
      addXsectionPlanesFromCenterline (numXsections);
      
//      addMonitor (new CenterlineOptimizer ());
//      myJawModel.setDynamicsEnabled (false);
      
      RenderProps.setVisible (airwaySkin, false);
      RenderProps.setVisible (tongue, false);
//      RenderProps.setVisible (tongue.getElements(), false);
      RenderProps.setVisible (tongue.getMuscleBundles(), false);
      
      
      RenderProps.setFaceStyle (planes, FaceStyle.NONE);
      RenderProps.setDrawEdges (planes, true);
      RenderProps.setFaceColor (planes, Color.LIGHT_GRAY);
      RenderProps.setEdgeColor (planes, Color.WHITE);
      RenderProps.setEdgeWidth (planes, 2);
      RenderProps.setColorMapEnabled (planes, false);

   }
   
   public void createCenterlineFromList() {
      centerlineMesh = new PolylineMesh ();
      int i = 0;
      for (double[] params : PlaneInfo) {
         Point3d pos = new Point3d(params[0], 0, params[1]);
         centerlineMesh.addVertex (pos);
         System.out.println ((i++)+" = "+pos.toString ());
      }
      centerline = centerlineMesh.addLine (
         centerlineMesh.getVertices ().toArray (new Vertex3d[0]));
      centerlineMeshComponent = new MeshComponent ("centerline");
      centerlineMeshComponent.setMesh (centerlineMesh);
      myJawModel.addMeshBody (centerlineMeshComponent);
      
      RenderProps.setLineColor (centerlineMeshComponent, 
         Color.getHSBColor (centerlineMesh.numLines ()/8f, 0.5f, 0.5f));
   }

   public void loadCenterlineFromFile(String filename) {
      try {
         centerlineMesh = new PolylineMesh (new File(filename));
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      if (centerlineMesh.numLines () > 1) {
         System.out.println("Warning loaded centerline mesh has multiple lines, using the last one");
      }
      centerline = centerlineMesh.getLines ().get (centerlineMesh.numLines ()-1);
      centerlineMeshComponent = new MeshComponent ("centerline");
      centerlineMeshComponent.setMesh (centerlineMesh);
      myJawModel.addMeshBody (centerlineMeshComponent);
      
      RenderProps.setLineColor (centerlineMeshComponent, 
         Color.getHSBColor (centerlineMesh.numLines ()/8f, 0.5f, 0.5f));
   }
   
   public void addXsectionPlanesFromCenterline(int n) {
      for (int i = 0; i < n; i++) {
         double s = i/(double)(n-1);
         Point3d center = centerline.interpolatePosition (s);         
//         System.out.println("position("+s+")="+center.toString ());

         if (smoothXsectionAngles) {
            addPlane (center, new AxisAngle (0, 1, 0, 
               Math.toRadians (anteriorAngle*(1-s)+posteriorAngle*s)));
         } else {
            Vector3d normal = centerline.interpolateTangent (s); // tangent to centerline is normal to xsection plane
//            System.out.println("tangent("+s+")="+normal.toString ());
            RotationMatrix3d rot = new RotationMatrix3d ();
            rot.setZDirection (normal);
            addPlane (center, rot.getAxisAngle ());
         }
      }
   }

   private Point3d computeCentroid(IntersectionContour contour) {
      Point3d center = new Point3d();
      double s = 1d/contour.size();
      for (int i = 0; i < contour.size (); i++) {
         center.scaledAdd (s, contour.get (i));
      }
      return center;
   }
   
   public void updateCenterline() {
      computeXsectionalAreas (); // updates contours
      if (planes.size () != contours.size ()) {
         System.err.println("missing contours!");
      }
//      centerlineMesh.clear ();
      Vertex3d[] vtxs = new Vertex3d[contours.size ()];
      for (int i = 0; i < contours.size (); i++) {
         Point3d pnt = computeCentroid(contours.get (i));
         pnt.y = 0; // project to midsagittal center;
         vtxs[i] = centerlineMesh.addVertex (pnt);
      }
      centerline = centerlineMesh.addLine (vtxs);
      RenderProps.setLineColor (centerlineMeshComponent, 
         Color.getHSBColor (centerlineMesh.numLines ()/8f, 0.5f, 0.5f));

   }
   
   public void updateXsectionPlanesFromCenterline() {
      int n = centerline.numVertices ();
      if (planes.size () != n) {
         planes.clear ();
         addXsectionPlanesFromCenterline (n);
      } 
      
      for (int i = 0; i < n; i++) {
         MeshComponent plane = planes.get (i);
         double s = i/(double)(n-1);
         Point3d center = centerline.interpolatePosition (s);         
//         System.out.println("position("+s+")="+center.toString ());

         if (smoothXsectionAngles) {
            plane.getMesh ().setMeshToWorld (new RigidTransform3d (center, new AxisAngle (0, 1, 0, 
               Math.toRadians (anteriorAngle*(1-s)+posteriorAngle*s))));
         } else {
            Vector3d normal = centerline.interpolateTangent (s); // tangent to centerline is normal to xsection plane
//            System.out.println("tangent("+s+")="+normal.toString ());
            RotationMatrix3d rot = new RotationMatrix3d ();
            rot.setZDirection (normal);
            plane.getMesh ().setMeshToWorld (new RigidTransform3d (center, rot.getAxisAngle ()));
         }
      }
   }
   
   public void addXsectionPlanesFromList() {     
      for (double[] params : PlaneInfo) {
         Point3d pos = new Point3d(params[0], 0, params[1]);
         AxisAngle rot = new AxisAngle (0, 1, 0, Math.toRadians (params[2]));
         addPlane (pos, rot);
      }
   }
   
   public MeshComponent addPlane (Point3d pos, AxisAngle rot) {
      
      MeshComponent plane = new MeshComponent ();
      String meshFilename = regGeomDir+"plane_xy.obj";
      PolygonalMesh mesh = null;
      try {
         mesh = new PolygonalMesh (meshFilename);
         mesh.scale (40, 50, 1); // mediolateral is y-dir
         plane.setMesh (mesh, meshFilename, null);
      }
      catch (IOException e) {
         e.printStackTrace();
      }

      plane.getMesh ().setMeshToWorld (new RigidTransform3d (pos, rot));
      planes.add (plane);

      return plane;
   }
   
   //ArrayList<ContactInfo> contacts = new ArrayList<ContactInfo> ();
   ArrayList<IntersectionContour> myRenderContours = new ArrayList<IntersectionContour> ();
   ArrayList<IntersectionContour> contours = new ArrayList<IntersectionContour> ();
   SurfaceMeshCollider myIxer = new SurfaceMeshCollider ();
   
   public double[] computeXsectionalAreas() {
      double[] areas = new double[planes.size ()];

      contours.clear();
      for (int i = 0; i < planes.size (); i++) {
         if (useExplicitLipOpening && i == 0) { // lips are at 0th plane
            areas[0] = Math.max (lipOpening, MINIMUM_TUBE_AREA);
            continue;
         }
         double crossSectionalArea = 0;
         ArrayList<IntersectionContour> contourList = myIxer.getContours (
              (PolygonalMesh)planes.get(i).getMesh (), 
              (PolygonalMesh)airwaySkin.getMesh ());
         if (contourList != null) {
            for (int ci = 0; ci < contourList.size (); ci++) {
               IntersectionContour contour = contourList.get (ci);
               if (contour.isClosed()) {
                  crossSectionalArea += contour.computePlanarArea ();
                  contours.add (contour);
               }
            }
         }
         areas[i] = Math.max (crossSectionalArea, MINIMUM_TUBE_AREA);
      }
      
//      System.out.println ("areas = " + (new VectorNd(areas)).toString ("%4.1f"));
      synchronized(myRenderContours) {
         myRenderContours.clear ();
         myRenderContours.addAll (contours);
      }
      return areas;
   }
   
   @Override
   public void prerender (RenderList list) {
      super.prerender (list);
      
      double[] areas = computeXsectionalAreas ();

      //send OSC
      OSCMessage msg = new OSCMessage("/tubes");
      OSCMessage msg2 = new OSCMessage("/artisynth/twomass/lung");
      //msg2.addArgument (1.0);
      msg2.addArgument ((float)twoMassLung);
      for (int i = 0; i < areas.length; i++) {
    	  msg.addArgument((float)areas[areas.length - i - 1]); //send the list "backwards"
    	  //msg.addArgument(areas[i]) //send the list "forwards"
      }
      try {
    	  oscSender1.send(msg);
    	  oscSender2.send (msg2);
    	  }
      catch (IOException e) {
    	  System.out.println(e.toString());
      }
      
      
   }

   public void render (Renderer renderer, int flags) {
      render (renderer, flags, myRenderProps);
   }

   public void render (Renderer renderer, int flags, RenderProps props) {
      super.render (renderer, flags);

      //GL2 gl = renderer.getGL2 ().getGL2 ();
      
      if (props.getEdgeWidth () > 0) {
         renderer.setLineWidth (props.getEdgeWidth ());
         renderer.setEdgeColoring (props, false);
         renderer.setShading (Shading.NONE);
         
         synchronized(myRenderContours) {
            for (IntersectionContour contour : myRenderContours) {
               if (contour != null) {
                  renderer.beginDraw (DrawMode.LINE_STRIP);
                  for (IntersectionPoint p : contour) {
                     renderer.addVertex (p.x, p.y, p.z);
                  }
                  renderer.endDraw();
               }
            }
         }
      }
   }
   
   public class CenterlineOptimizer extends MonitorBase {

      @Override
      public void apply (double t0, double t1) {
         updateCenterline ();
         planes.clear ();
         addXsectionPlanesFromCenterline (numXsections);
//         updateXsectionPlanesFromCenterline ();
      }
      
   }
   
   @Override
   public void addKinematicSoftPalate() {
      // don't add soft palate for this demo
   }
   
   public double getLipOpening () {
      return lipOpening;
   }
   
   public void setLipOpening (double lipOpening) {
      this.lipOpening = lipOpening;
   }
   
   public boolean getUseExplicitLipOpening () {
      return useExplicitLipOpening;
   }
   
   public void setUseExplicitLipOpening (boolean useExplicitLipOpening) {
      this.useExplicitLipOpening = useExplicitLipOpening;
   }
   
   public double getTwoMassLung() {
      return twoMassLung;
   }
   
   public void setTwoMassLung(double tmLung) {
      this.twoMassLung = tmLung;
   }
   
   
}
