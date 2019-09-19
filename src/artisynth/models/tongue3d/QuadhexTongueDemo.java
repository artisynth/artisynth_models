package artisynth.models.tongue3d;

import java.awt.Component;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyMode;
import maspack.render.GL.GLClipPlane;
import maspack.render.GL.GLViewer;
import maspack.render.RenderProps;
import maspack.render.Dragger3d.DraggerType;
import maspack.render.Renderer.LineStyle;
import maspack.util.ReaderTokenizer;
import maspack.widgets.DoubleFieldSlider;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;
import artisynth.core.driver.Main;
import artisynth.core.driver.ViewerManager;
import artisynth.core.femmodels.*;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.gui.selectionManager.SelectionManager;
import artisynth.core.materials.*;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import maspack.util.ClassAliases;
import artisynth.core.workspace.DriverInterface;

public class QuadhexTongueDemo extends HexTongueDemo {
   
   public static final double muscleExcitationSliderMax = 0.4;
   public static final boolean addSkullMeshes = false;
   public static final boolean useBlemkerMuscle = true;

   public QuadhexTongueDemo () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      
      
      mech = new MechModel("mech");
      mech.setIntegrator (Integrator.ConstrainedBackwardEuler);
      mech.setMaxStepSize (0.01);
      setAdaptiveStepping(false);
      addModel(mech);
      
      tongue = createFemMuscleTongue (linearMaterial);
      mech.addModel (tongue);

      addExciters (tongue);
      setupRenderProps();
      
      for (FemElement elem : tongue.getElements ()) {
         if (elem.getRestVolume () < 0) {
            System.out.println("elem "+elem.myNumber+" degenerate");
         }
      }
      
      if (addSkullMeshes)
	 setupStaticBadinJawTongue();

      addMuscleExciterProbes(this, tongue);
      // set first and last probe active
      getInputProbes().get(0).setActive (true);
      getInputProbes().get(getInputProbes().size()-1).setActive(true);

      // delete all elements except those associated with GGP
      ArrayList<FemElement3dBase> ggpElems = new ArrayList<FemElement3dBase>();
      for (MuscleElementDesc d :
              tongue.getMuscleBundles().get("GGP_L").getElements()) {
         ggpElems.add (d.getElement());
      }
      for (MuscleElementDesc d :
              tongue.getMuscleBundles().get("GGP_R").getElements()) {
         ggpElems.add (d.getElement());
      }
      tongue.getMuscleBundles().get("STY_R").setFibresActive(false);
      tongue.getMuscleBundles().get("STY_L").setFibresActive(false);

      for (MuscleBundle m : tongue.getMuscleBundles()) {
         if (!m.getName().equals("GGP_L") &&
             !m.getName().equals("GGP_R")) {
            m.setMuscleMaterial (new InactiveMuscle());
         }
      }
      ggpElems.add (tongue.getElements().getByNumber (127));
      ggpElems.add (tongue.getElements().getByNumber (346));
      ggpElems.add (tongue.getElements().getByNumber (497));
      ggpElems.add (tongue.getElements().getByNumber (716));
      
      SelectionManager selman = Main.getMain().getSelectionManager();
      for (FemElement3d e : tongue.getElements()) {
         if (!ggpElems.contains (e)) {
            selman.addSelected (e);
         }
         // boolean liesOnY = false;
         // for (FemNode3d n : e.getNodes()) {
         //    Point3d pos = n.getPosition();
         //    if (Math.abs(pos.y) < 1e-10) {
         //       liesOnY = true;
         //       break;
         //    }
         // }              
         // if (!liesOnY) {
         //    selman.addSelected (e);
         // }
      }
      
   }
   
   public void setupRenderProps() {
      
      tongue.setElementWidgetSize (0);
      tongue.setDirectionRenderLen(0.5);
//      tongue.setSurfaceRendering (SurfaceRender.Strain);
      tongue.setSurfaceRendering (SurfaceRender.None);
//      tongue.setSurfaceRendering(SurfaceRender.Shaded);
      RenderProps.setVisible(tongue.getElements(), true);
      RenderProps.setVisible(tongue.getNodes(), false);
      RenderProps.setLineStyle(tongue.getMuscleBundles(), LineStyle.LINE);
      RenderProps.setLineWidth(tongue.getMuscleBundles(), 2);
      RenderProps.setVisible(tongue.getMuscleBundles(), true);
   }
   
   
   
   public static FemMuscleModel createFemMuscleTongue(boolean linearMaterial) {
      
      FemMuscleModel tongue = new FemMuscleModel ("tongue");
      FemMuscleModel linTongue = new FemMuscleModel("tmp");

      readFromAnsysReader(linTongue, "tongue");
      addStyNodes (linTongue);
      FemFactory.createQuadraticModel (tongue, linTongue);

      setAttachmentNodes (tongue);
      setExtraAttachmentNodes (tongue);
      if (linearMaterial) {
         tongue.setMaterial (new LinearMaterial ());
      }
      else {
         tongue.setMaterial (new MooneyRivlinMaterial ());
         tongue.setSoftIncompMethod (IncompMethod.FULL);
      }

      HexTongueDemo.setTongueProperties (tongue);
      addMuscleElements(tongue);
//      setMuscleProps(tongue);
      if (useBlemkerMuscle) {
	 tongue.setMuscleMaterial(new BlemkerMuscle());
      }
      else {
	 tongue.setMuscleMaterial(new GenericMuscle());
      }
      tongue.setIncompressible(FemModel3d.IncompMethod.OFF); //XXX incomp current unstable with Blemker muscle model
      tongue.setDirectionRenderLen(0.5);
      enableExtrinsicStyloglossusFibres(tongue);

      setupRenderProps(tongue);
      setNodeNames (tongue);
      return tongue;
      
   }

   private static FemNode3d[] getNodesByIndex (FemModel3d fem, int[] idxs) {
      FemNode3d[] nodes = new FemNode3d[idxs.length];
      for (int i=0; i<idxs.length; i++) {
         nodes[i] = fem.getNodes().get(idxs[i]);
      }
      return nodes;
   }

   private static int[] myExtraJawAttachments = new int[] {
      2902, 2898, 1586, 2880, 1557, 1528, 1499, 1470, 1432, 2781, 
      2778, 2784, 2810, 2814, 2832, 2836, 2854, 2858, 2876, 2786,
      2792, 2789, 2811, 2806, 2914, 2920, 2917, 2937, 2935, 2940,
      2938, 2941, 2933, 2945, 3524, 3534, 2943, 2946, 2930, 2947,
      2639, 2927, 2637, 2640, 2601, 2594, 2391, 2600, 1423, 1431,
      1427, 1467, 1496, 1525, 1554, 1583, 1580, 1551, 1522, 1493,
      1464, 1459, 1438, 1435, 1441, 1598, 1604, 1621, 1601, 1463,
      1619, 1624, 1618, 1623, 1625, 2340, 2326, 1629, 1628, 1630,
      1631, 1267, 1611, 1614, 1263, 1219, 1217, 1211, 1266, 976
   };

   private static int[] myExtraHyoidAttachments = new int[] {
      1134, 1298, 1179, 1294, 1181, 1180, 1296, 1071, 1079, 1078,
      1076, 2483, 2484, 2479, 1102, 1088, 1094, 1091, 1097, 1107,
      2498, 2496, 2493, 2503, 2489, 2485, 2377, 2381, 2382, 2369,
      2373, 2401, 2408, 2482, 2406, 2403, 2398, 2452, 2454, 2455,
      3126, 2670, 2528, 2563, 2561,
      2562, 2668, 2666, 2658, 2570, 2569, 3177, 3170, 3175, 3143,
      2481, 3144, 2458, 2457, 3160, 3161, 3165, 982, 1187, 1286,
      985, 959, 988, 962, 1080, 1084, 966, 954, 967, 1073,
      990, 993, 1869, 1868, 1041, 1074, 1842, 1885, 1043, 1886,
      1038, 1036, 1040, 1902, 1891, 1900, 1188, 1896
   };

   public static void setExtraAttachmentNodes (FemModel3d fem) {
      for (FemNode3d n : getNodesByIndex(fem, myExtraJawAttachments)) {
         n.setDynamic (false);
	 RenderProps.setPointColor(n, Color.RED);
      }
      for (FemNode3d n : getNodesByIndex(fem, myExtraHyoidAttachments)) {
         n.setDynamic (false);
	 RenderProps.setPointColor(n, Color.BLUE);
      }
   }

   public static void setMuscleProps(FemMuscleModel tongue) {
      VectorNd maxForces = getMaxForces(tongue);
      for (int i = 0; i < tongue.getMuscleBundles().size(); i++) {
	 MuscleBundle b = tongue.getMuscleBundles().get(i);
//	 GenericMuscle mm = new GenericMuscle();
	 BlemkerMuscle mm = new BlemkerMuscle();
//	 mm.setMaxStress(MuscleMaxStressPerNewton*maxForces.get(i));
	 b.setMuscleMaterial(mm);
      }
   }
   
   public static void addMuscleElements(FemMuscleModel tongue) {
      /*
       * load muscle fibers
       */
      try {
//         GenericMuscle mm = new GenericMuscle();
//         mm.setMaxStress (5000);
//         tongue.setMuscleMaterial (mm);
         
         AnsysMuscleElemReader.read (tongue, new FileReader (geometrypath
         + "CreateMuscles.mac"));
         AnsysMuscleFiberReader.read (tongue, new FileReader (geometrypath
            + "Fibers.mac"));
         
         for (MuscleBundle b : tongue.getMuscleBundles()) {
            b.setFibresActive(false);
            b.computeElementDirections();
            RenderProps.setVisible(b.getFibres(), false);
//            b.setDirectionRenderLen(0.005);
         }
         
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
   
   public static void enableExtrinsicStyloglossusFibres(FemMuscleModel tongue) {
      String[] styBundleNames = new String[]{"STY_L", "STY_R"};
      int[] styAttachNodeIdx = new int[]{947,946};
      
      for (int i = 0; i < styBundleNames.length; i++) {
	 MuscleBundle sty = tongue.getMuscleBundles().get(styBundleNames[i]);
	 sty.setFibresActive(true);
	 RenderProps.setVisibleMode(sty.getFibres(), PropertyMode.Inherited);
//	 pruneFibers(sty, tongue.getNode(styAttachNodeIdx[i]));
	 for (Muscle m : sty.getFibres()) {
	    if (m.getMaterial() instanceof AxialMuscleMaterial) {
	       AxialMuscleMaterial mat = 
	          (AxialMuscleMaterial)m.getMaterial().clone();
	       mat.setForceScaling(0.5);
	       m.setMaterial (mat);	       
	    }
	 }
      }
      
   }

   /*
    * removes muscles NOT connected to point p
    */
   public static void pruneFibers(MuscleBundle bundle, Point p) {
      ArrayList<Muscle> fibers = new ArrayList<Muscle>();
      for (Muscle m : bundle.getFibres()) {
	 if (m.getFirstPoint()!=p && m.getSecondPoint()!=p) 
	    fibers.add(m);
      }
      
      System.out.println(bundle.getName()+" removing "+fibers.size());
      
      for (Muscle m : fibers) {
	 bundle.removeFibre(m);
      }
   }

//   public static String excitersFile = "bilateralExciters.art";
   public static String excitersFile = "newExciters.art";
   public static String excitersDir = ArtisynthPath.getSrcRelativePath(QuadhexTongueDemo.class, "exciters");
   
   public static void addExciters (FemMuscleModel fem) {
//      loadExciters(fem, excitersFile);
      addBilateralExciters(fem);
//      addExcitersFromFiles(fem, excitersDir);
   }
   
   public static void addExcitersFromFiles(FemMuscleModel fem, String dirname) {
      File directory = new File(dirname);
      if (directory != null && directory.isDirectory()) {
	 for (String fileName : directory.list()) {
	    if (fileName.endsWith(".art")) {
	       addExciterFromFile(fem, new File(directory.getAbsolutePath()
		     + "/" + fileName));
	    }
	 }
      }
   }

   public static void addExciterFromFile(FemMuscleModel fem, File file) {
      try {
	 ReaderTokenizer rtok = ArtisynthIO.newReaderTokenizer(file);
	 if (rtok.nextToken() != ReaderTokenizer.TT_WORD)
	    throw new IOException("cannot create instance of " + rtok.sval);
	 MuscleExciter mex = (MuscleExciter) ClassAliases.newInstance(
	       rtok.sval, MuscleExciter.class);
         ScanWriteUtils.scanfull (rtok, mex, fem);
	 fem.addMuscleExciter(mex);
      } catch (Exception e) {
	 e.printStackTrace();
      }
   }
   
   public static void loadExciters (FemMuscleModel fem, String excitersFilename) {
      try {
	 ReaderTokenizer rtok = ArtisynthIO.newReaderTokenizer(new File(
	       ArtisynthPath.getSrcRelativePath(QuadhexTongueDemo.class,
		     "exciters/lists/" + excitersFilename)));

	 // ignore first line
	 if (rtok.nextToken() != ReaderTokenizer.TT_WORD)
	    throw new IOException("cannot create instance of " + rtok.sval);
         ScanWriteUtils.scanfull (rtok, fem.getMuscleExciters(), fem);	 
      } catch (IOException e) {
	 e.printStackTrace();
      }
   }
   
   public static void addBilateralExciters (FemMuscleModel fem) {
      
      // add exciters if 21 muscle groups: 20 paired, 1 unpaired
      if (fem.getMuscleBundles ().size() == 21) {
         for (int i = 0; i < 20; i+=2) {
            MuscleBundle left = fem.getMuscleBundles ().get (i);
            MuscleBundle right = fem.getMuscleBundles ().get (i+1);
            String[] name = left.getName ().split ("_");
            MuscleExciter ex = new MuscleExciter (name[0]);
            ex.addTarget (left, 1.0);
            ex.addTarget (right, 1.0);
            fem.addMuscleExciter (ex);
         }
         // add exciter for unpaired muscle group (SL)
         MuscleBundle unpaired = fem.getMuscleBundles ().get (20);
         MuscleExciter ex = new MuscleExciter (unpaired.getName ());
         ex.addTarget (unpaired, 1.0);
         fem.addMuscleExciter (ex);
      }
      
   }


   ControlPanel myControlPanel;
   ControlPanel myMusclePanel;
   

   
   public void attach (DriverInterface driver) {
//      setSagittalView(tongue);
      if (myControlPanels.size() == 0 && tongue != null) {
	 FemControlPanel.createControlPanel(this, tongue, myModels.get(0));
	 FemControlPanel.createMuscleExcitersPanel(this, tongue);
      }
      // createProbes(1.0);
      // e

      // moved to main constructor
      //addExcitationProbes();
   }
   
   public static final int frameWidthOffset = 600-547;
   public static final int frameHeightOffset = 600-517;
   
   public static void setSagittalView(FemMuscleModel tongue) {
      setSagittalView(tongue, true, 1d);
   }
   
   public static void setSagittalView(FemMuscleModel tongue, boolean clipped, double scale) {
      setSagittalView(tongue, clipped, scale, 600, 600);
   }
   
   public static void setSagittalView(FemMuscleModel tongue, boolean clipped, double scale, int width, int height) {
      
      GLViewer v = Main.getMain().getViewer();
      if (v == null) {
         return;
      }
      Main.getMain().getMainFrame().setSize(width+frameWidthOffset, height+frameHeightOffset);
      
      v.setOrthographicView (true);
      v.setGridVisible (true);
      
      if (v.getNumClipPlanes() < 1) {
	 v.addClipPlane();
      }
      GLClipPlane clip  = v.getClipPlane (0);
      
      clip.setPosition(new Point3d());
      clip.setOrientation(new AxisAngle (1, 0, 0, Math.PI / 2));
      double gridOffset = 0.003*scale; // 0.005 // to see muscle dirs
      clip.setOffset (gridOffset);
      clip.setGridVisible (false);
      clip.setDragger (DraggerType.None);
      
      double nearz = 0.001 * scale;
      double farz = 1d * scale;
      double fieldHeight = 0.1 * scale;
      Point3d eye = new Point3d(0.1, -0.4, 0.096);
      eye.scale(scale);
      Point3d centre = new Point3d(0.1, 0, 0.095);
      centre.scale(scale);
      
      v.setOrthogonal (fieldHeight, nearz, farz);
      v.setEye (eye);
      v.setCenter (centre);
      
      if (clipped) {
	 try {

	    tongue.scanSurfaceMesh(geometrypath + "/tongue_righthalf.smesh");
	 } catch (IOException e) {
	    e.printStackTrace();
	 }
      }
      else {
//	 v.myUnClippedRenderList.clear();
//	 v.myUnClippedRenderList.addIfVisible(tongue);
//	 v.removeRenderable(tongue);
      }
      tongue.setElementWidgetSize(0);
      tongue.setSurfaceRendering(SurfaceRender.Stress);
   }
   
   public static void setSliderRange(ControlPanel panel, String propName, double min, double max) {
      for (Component comp : panel.getPropertyPanel().getWidgets()) {
	 if (comp instanceof LabeledComponentBase) {
	    String widgetPropName = PropertyWidget.getProperty((LabeledComponentBase)comp).getName();
	    if (widgetPropName.compareTo(propName)==0) {
	       if (comp instanceof DoubleFieldSlider) {
		  ((DoubleFieldSlider) comp).setRange(min, max);
	       }
	    }
	 }
      }
   }

   public void addMexProbe(MuscleExciter mex, double duration, double ex) {
      NumericInputProbe ip = new NumericInputProbe(mex, "excitation", 0, duration);
      ip.addData(0, new VectorNd(1));
      VectorNd exData = new VectorNd(1);
      exData.set(0, ex);
      ip.addData(duration, exData);
      if (mex.getName() != null) 
	 ip.setName(mex.getName());
      addInputProbe(ip);
   }
   
}
