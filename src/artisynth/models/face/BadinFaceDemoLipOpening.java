package artisynth.models.face;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.Robot;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

import javax.imageio.ImageIO;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.Renderer.Shading;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.GenericMuscle;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.InputProbe;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.MDLMeshIO;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.jawTongue.BadinJawHyoid;
import artisynth.models.jawTongue.JawHyoidFemMuscleTongue;

public class BadinFaceDemoLipOpening extends BadinFemMuscleFaceDemo {

   boolean collideTongueFace = true;
   Color openingColor = new Color (0f,1f,0f);

   
   double myIntraoralPressure = 0;

   public static PropertyList myProps =
      new PropertyList (BadinFaceDemoLipOpening.class, RootModel.class);

   static {
      myProps.addReadOnly ("lipOpeningArea *", "lip opening aera");
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   public BadinFaceDemoLipOpening () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      face = setupFaceModel();

      BadinFaceDemo.attachFaceToSkull (face);
      fixFaceToJaw (face, "face_jaw_attachments.txt");

      
      setFaceRenderProps (face);

      /*
       * set OOP muscle
       * 
       */
      loadoop("7M"); // for fibers
      face.getMuscleBundles().get("OOP").clearElements();
//      addoop("7Mu");
//      addoop("6Ml");
      
      /*
       * thick OOP
       */
      
//      addoop("4Mu");
//      addoop("5Su");
////      addoop("5Ml");
//      addoop("5Mu");
      addoop("5Du");
      addoop("6Sl");
      addoop("6M");
      addoop("6Du");
      addoop("7Sl");
      addoop("7M");
      addoop("7Du");
      addoop("8Su");
      addoop("8Mu");
      addoop("8Du");
      
      /*
       * line based opposing muscles
       */
//      for (MuscleBundle b : face.getMuscleBundles ()) {
//         if (!b.getName ().equals ("OOP")) {
//            b.clearElements ();
//            b.setFibresActive (true);
//         }
//      }
      
      addLipOpeningPlane ();

//      RigidBody upperteeth = mech.rigidBodies ().get ("upperteeth");
//      RenderProps.setVisible (upperteeth, true);      
//      MeshComponent lowerlip = face.getMeshComp ("lowerlip");


      
   }
   
   public void addLipOpeningPlane() {
      MeshComponent lipOpeningPlane = null;

      if (lipOpeningPlane == null) {
         lipOpeningPlane = new MeshComponent ();
         lipOpeningPlane.setMesh (MeshFactory.createPlane (.08,.08,10,10));
         RenderProps.setFaceColor (lipOpeningPlane, openingColor);
         mech.addMeshBody (lipOpeningPlane);
         
      }
      
      lipOpeningPlane.setMeshToWorld (new RigidTransform3d (0.1, 0.0, 0.1, 0, Math.PI/2, 0));
//
//      Point3d pos = face.getByNumber (329).getPosition ();
//      Vector3d v = new Vector3d();
//      v.sub (pos, face.getByNumber (226).getPosition ());
//      v.cross (Vector3d.Y_UNIT);
//      RotationMatrix3d rot = new RotationMatrix3d ();
//      rot.setZDirection (v);
//      System.out.println("pos="+pos);
//      lipOpeningPlane.setMeshToWorld (new RigidTransform3d (pos, rot));
      
   }
   

   public static void fixFaceToJaw (FemModel3d face, String fileName) {
      for (Integer idx : readIntList (faceGeometryDir + fileName)) {
         if (idx < face.numNodes()) {
            FemNode3d n = face.getNode(idx);
            n.setDynamic (false);
         }
      }
   }


   
   public FemMuscleModel setupFaceModel () {
//      FemMuscleModel face;
//      face = BadinFaceDemo.createFace(BadinFaceDemo.faceGeometryDir,
//	    "face_skullconformed_quality", /*linearMaterial=*/false);
      
//      face = BadinFaceDemo.createFace(BadinFaceDemo.faceGeometryDir,
//	    "badinface_oop_csa_midsagittal", /*linearMaterial=*/false);
      face.setName ("badinface");
      
//      BadinFaceDemo.addMuscles(face, BadinFaceDemo.muscleNodesFile, BadinFaceDemo.midSagittalPlane);

      //      BadinFaceDemo.addMuscles(face, BadinFaceDemo.faceGeometryDir + "face_muscles_yohan.node", BadinFaceDemo.midSagittalPlane);
//    BadinFemMuscleFaceDemo.setMuscleElements(face, BadinFemMuscleFaceDemo.muscleThickness);
//    RefFemMuscleFaceDemo.loadMuscleElements(face, face.getMuscleBundles().get("OOP"), 
//	    BadinFaceDemo.faceGeometryDir+"OOP_7thRing_elements.txt");
      
      BlemkerMuscle muscleMat = new BlemkerMuscle ();
      muscleMat.setMaxStress (100000);
      face.getMuscleBundles ().get ("OOP").setMuscleMaterial (muscleMat);
      
      ((GenericMuscle)face.getMuscleMaterial()).setMaxStress(100000);
      
      face.setDirectionRenderLen(0.5);
      face.setElementWidgetSize(1);
      

      
  
//      face.scaleDistance (BadinJawHyoid.m2mm);
      
      return face;
   }


   public void setFaceRenderProps (FemModel3d face) {
      RenderProps.setVisible (face.getNodes (), false);
      RenderProps.setVisible (face.getElements (), false);
      RenderProps.setVisible (face.markers (), false);
      RenderProps.setLineWidth (face, 1);
      face.setSurfaceRendering (SurfaceRender.Shaded); 
      RenderProps.setFaceColor(face, Color.RED);
      
      RenderProps.setLineWidth(face.getElements(), 0);
      RenderProps.setLineColor(face, new Color(.2f, .2f, .2f));
      // face.setSubSurfaceRendering(false);
      
      
      for (FemNode3d n : face.getNodes()) {
	 RenderProps.setVisibleMode(n, PropertyMode.Inherited);
      }
      
      for (RigidBody r : mech.rigidBodies ()) {
         RenderProps.setFaceColor (r, Color.RED);
         RenderProps.setVisible (r, false);
      }
      

   }

   public void addExcitationProbe(MuscleExciter ex) {
      double duration = 1.0, rampDuration = 0.4;
      NumericInputProbe ip = new NumericInputProbe(ex.getProperty("excitation"), ex);
      ip.setStartStopTimes(0, duration);
      ip.setInterpolationOrder(Order.Linear);
      ip.setName(ex.getName()+" excitation");
      ip.addData(0, new VectorNd(1));
      ip.addData(rampDuration, new VectorNd(new double[]{ex.getExcitation()}));
      ip.addData(duration, new VectorNd(new double[]{ex.getExcitation()}));
      ip.setAttachedFileName(ex.getName()+"txt");
      addInputProbe(ip);
   }

   
   @Override
   public void attach(DriverInterface driver) {
      // TODO Auto-generated method stub
      super.attach(driver);
      
//      ControlPanel panel;
//      panel = FemControlPanel.createControlPanel(this, face, face);
//      panel.setName("Face Controls");
//      
//      panel = FemControlPanel.createMuscleBundlesPanel(this, face);
//      panel.setName("Face Muscles");
      
      File workingDirFile = new File (
         ArtisynthPath.getSrcRelativePath(BadinFaceDemo.class, "data/approximant/"));
      if (workingDirFile.exists ()) {
	 ArtisynthPath.setWorkingDir (workingDirFile);
	 try {
	    Main.getMain().loadProbesFile(new File(workingDirFile+"/0probes.art"));
	    for (Probe probe : getInputProbes ()) {
	       probe.load ();
	    }
	 } catch (IOException e) {
	    e.printStackTrace();
	 }
      }
      
      getMainViewer().setOrthographicView(true);
      getMainFrame ().setSize (1053, 1113); // 1000 x 1000 viewer 
      getMainViewer().setAxialView (AxisAlignedRotation.NY_Z);
      
      // Chenhao view
//      setViewerEye (new Point3d(-0.56632, 0.000422501, 0.107807));
//      setViewerCenter (new Point3d(0.134409, 0.000422502, 0.107807));
      Main.getMain ().screenShot ("foo.png");
      // Ian view
      setViewerEye (new Point3d(-0.521787, 0.0009809, -0.126136));
      setViewerCenter (new Point3d(0.134664, 0.0009809, 0.117214));
      //connor view
//      setViewerEye (new Point3d(-0.56632, 0.000422501, 0.107807));
//      setViewerCenter (new Point3d(0.134409, 0.000422502, 0.107807));
      getMainViewer().zoom(0.5);
   }
   
   private static Rectangle getViewerBounds(GLViewer viewer) {
      Rectangle area = new Rectangle();
      area.width = viewer.getCanvas().getWidth();
      area.height = viewer.getCanvas().getHeight();
      area.x = viewer.getCanvas().getLocationOnScreen().x;
      area.y = viewer.getCanvas().getLocationOnScreen().y;
      return area;
   }
   

   /**
    * Method to get the distance between this pixel's color and the passed color
    * @param testColor the color to compare to
    * @return the distance between this pixel's color and the passed color
    */
   public double colorDistance(Color testColor, Color color)
   {
      float[] c = new float[3];
      float[] t = new float[3];
      color.getColorComponents (c);
      testColor.getColorComponents (t);
      
      return Math.sqrt (
         (c[0]-t[0])*(c[0]-t[0])+
         (c[1]-t[1])*(c[1]-t[1])+
         (c[2]-t[2])*(c[2]-t[02]));
   }
   
   /**
    * measure lip opening area by counting green pixels. Note this only works
    * if camera is at the correct frontal viewing angle.
    * 
    * @return lip opening area in in mm^2
    */
   public double getLipOpeningArea() {
      double openingArea = -1;
      BufferedImage screenshot = null;
      ArrayList<Color> pixelColors = new ArrayList<> ();
      ArrayList<Integer> pixelCnts = new ArrayList<> ();
      try {
         screenshot = (new Robot()).
            createScreenCapture(getViewerBounds(getMainViewer ()));
         System.out.println("total pixels = "+screenshot.getWidth ()*screenshot.getHeight ());
        
         int cnt = 0;
         for (int i = 0; i < screenshot.getWidth (); i++) {
            for (int j = 0; j < screenshot.getHeight (); j++) {
               Color pixelColor = new Color(screenshot.getRGB (i, j));
               if (pixelColor.getGreen ()>200) {
                  if (!pixelColors.contains (pixelColor)) {
               
                  pixelColors.add (pixelColor);
                  pixelCnts.add (new Integer (0));
                  }
               
               int idx = pixelColors.indexOf (pixelColor);
               int c = pixelCnts.get (idx) + 1;
               pixelCnts.set (idx, c); 
//               cnt++;
               }
               
               if (colorDistance (openingColor, pixelColor) < 0.5) {
                  cnt++;
               }
            }
         }
         System.out.println("opening pixels = "+cnt);
         for (int i = 0; i < pixelColors.size (); i++) {
            System.out.println("pixel color = "+pixelColors.get (i)+" "+pixelCnts.get (i));
         }
         System.out.println("opening color = "+openingColor+" matching colors = "+cnt);
         double distPerPixel = getMainViewer().distancePerPixel (Vector3d.ZERO);
         openingArea = distPerPixel*distPerPixel*cnt*1000000; // meter to mm conversion
         System.out.println("dist-per-pixel = "+distPerPixel+"; opening area = "+openingArea);
         
         System.out.println("----");

         
      } catch (Exception e) {
         e.printStackTrace();
      }
            
//      short[] threshold = new short[256];
//      for (int i = 0; i < 256; i++)
//         threshold[i] = (i < 128) ? (short)0 : (short)255;
//         BufferedImageOp thresholdOp =
//            new LookupOp(new ShortLookupTable(0, threshold), null);
//         BufferedImage destination = thresholdOp.filter(screenshot, null);
      
         try {
            ImageIO.write(screenshot, "png", new File("test.png"));
         }
         catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
         return openingArea;
   }
   
   


}
