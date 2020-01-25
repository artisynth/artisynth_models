package artisynth.models.tongue3d;

import java.awt.Component;
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
import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.*;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import maspack.util.ClassAliases;
import artisynth.core.workspace.DriverInterface;

public class FemMuscleTongueDemo extends HexTongueDemo {

   public static final double muscleExcitationSliderMax = 0.4;
   public static final boolean addSkullMeshes = false;
   public static final boolean useBlemkerMuscle = true;

   public FemMuscleTongueDemo () {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {

      mech = new MechModel("mech");
      mech.setIntegrator(Integrator.ConstrainedBackwardEuler);
      mech.setMaxStepSize(0.01);
      addModel(mech);

      tongue = createFemMuscleTongue(linearMaterial);
      mech.addModel(tongue);

      addExciters(tongue);
      setupRenderProps();

      for (FemElement elem : tongue.getElements()) {
         if (elem.getRestVolume() < 0) {
            System.out.println("elem " + elem.myNumber + " degenerate");
         }
      }

      if (addSkullMeshes)
         setupStaticBadinJawTongue();

      addMuscleExciterProbes(this, tongue);
      // set first and last probe active
      getInputProbes().get(0).setActive (true);
      getInputProbes().get(getInputProbes().size()-1).setActive(true);
   }

   public void setupRenderProps() {
      FemMuscleTongueDemo.setMuscleProps (tongue);
   }
   
   public static void setupRenderProps(FemMuscleModel tongue) {
      tongue.setElementWidgetSize(0);
      tongue.setDirectionRenderLen(0.5);
      // tongue.setSurfaceRendering (SurfaceRender.Strain);
      tongue.setSurfaceRendering(SurfaceRender.None);
      // tongue.setSurfaceRendering(SurfaceRender.Shaded);
      RenderProps.setVisible(tongue.getElements(), true);
      RenderProps.setVisible(tongue.getNodes(), false);
      RenderProps.setLineStyle(tongue.getMuscleBundles(), LineStyle.LINE);
      RenderProps.setLineWidth(tongue.getMuscleBundles(), 3);
      RenderProps.setVisible(tongue.getMuscleBundles(), true);

      // make muscle bundle visibility inherited
      for (MuscleBundle b : tongue.getMuscleBundles()) {
         RenderProps.setVisibleMode(b, PropertyMode.Inherited);
         RenderProps.setLineColor(b, FemControlPanel.getMuscleColor(b.getNumber()));
         b.setElementWidgetSize(0);
      }
   }

   public static FemMuscleModel createFemMuscleTongue(boolean linearMaterial) {
      System.out.println ("createFemMuscleTongue");
      FemMuscleModel tongue = new FemMuscleModel("tongue");
      createFemMuscleTongue (tongue, linearMaterial);
      return tongue;
   }
   
   public static void createFemMuscleTongue(FemMuscleModel tongue, boolean linearMaterial) {

      readFromAnsysReader(tongue, "tongue");
      addStyNodes(tongue);
      setAttachmentNodes(tongue);
      if (linearMaterial)
         tongue.setMaterial(new LinearMaterial());
      else
         tongue.setMaterial(new MooneyRivlinMaterial());

      HexTongueDemo.setTongueProperties(tongue);
      addMuscleElements(tongue);
      // setMuscleProps(tongue);
      if (useBlemkerMuscle) {
         tongue.setMuscleMaterial(new BlemkerMuscle());
      }
      else {
         tongue.setMuscleMaterial(new GenericMuscle());
      }
      tongue.setIncompressible(IncompMethod.OFF); // XXX incomp current unstable
                                                  // with Blemker muscle model
      tongue.setDirectionRenderLen(0.5);
      enableExtrinsicStyloglossusFibres(tongue);

      setupRenderProps(tongue);
      setNodeNames(tongue);

   }

   public static void setMuscleProps(FemMuscleModel tongue) {
      VectorNd maxForces = getMaxForces(tongue);
      for (int i = 0; i < tongue.getMuscleBundles().size(); i++) {
         MuscleBundle b = tongue.getMuscleBundles().get(i);
         // GenericMuscle mm = new GenericMuscle();
         BlemkerMuscle mm = new BlemkerMuscle();
         // mm.setMaxStress(MuscleMaxStressPerNewton*maxForces.get(i));
         b.setMuscleMaterial(mm);
      }
   }

   public static void addMuscleElements(FemMuscleModel tongue) {
      /*
       * load muscle fibers
       */
      try {
         // GenericMuscle mm = new GenericMuscle();
         // mm.setMaxStress (5000);
         // tongue.setMuscleMaterial (mm);

         AnsysMuscleElemReader.read(tongue, new FileReader(geometrypath
            + "CreateMuscles.mac"));
         AnsysMuscleFiberReader.read(tongue, new FileReader(geometrypath
            + "Fibers.mac"));

         for (MuscleBundle b : tongue.getMuscleBundles()) {
            b.setFibresActive(false);
            b.computeElementDirections();
            RenderProps.setVisible(b.getFibres(), false);
            // b.setDirectionRenderLen(0.005);
         }

      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public static void enableExtrinsicStyloglossusFibres(FemMuscleModel tongue) {
      String[] styBundleNames = new String[] { "STY_L", "STY_R" };
      int[] styAttachNodeIdx = new int[] { 947, 946 };

      for (int i = 0; i < styBundleNames.length; i++) {
         MuscleBundle sty = tongue.getMuscleBundles().get(styBundleNames[i]);
         sty.setFibresActive(true);
         RenderProps.setVisibleMode(sty.getFibres(), PropertyMode.Inherited);
         // pruneFibers(sty, tongue.getNode(styAttachNodeIdx[i]));
         for (Muscle m : sty.getFibres()) {
            if (m.getMaterial() instanceof AxialMuscleMaterial) {
               AxialMuscleMaterial mat =
                  (AxialMuscleMaterial)m.getMaterial().clone();
               mat.setForceScaling(0.5);
               m.setMaterial(mat);
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
         if (m.getFirstPoint() != p && m.getSecondPoint() != p)
            fibers.add(m);
      }

      System.out.println(bundle.getName() + " removing " + fibers.size());

      for (Muscle m : fibers) {
         bundle.removeFibre(m);
      }
   }

   // public static String excitersFile = "bilateralExciters.art";
   public static String excitersFile = "newExciters.art";
   public static String excitersDir = ArtisynthPath.getSrcRelativePath(
      FemMuscleTongueDemo.class, "exciters");

   public static void addExciters(FemMuscleModel fem) {
      // loadExciters(fem, excitersFile);
      addBilateralExciters(fem);
      // addExcitersFromFiles(fem, excitersDir);
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
         MuscleExciter mex = (MuscleExciter)ClassAliases.newInstance(
            rtok.sval, MuscleExciter.class);
         ScanWriteUtils.scanfull (rtok, mex, fem);
         // XXX need to add the exciters to their targets. Do this with
         // a gain of 1 for now.
         for (int i=0; i<mex.numTargets(); i++){
            ExcitationComponent targ = mex.getTarget(i);
            targ.addExcitationSource (mex, 1);
         }
         fem.addMuscleExciter(mex);
      }
      catch (Exception e) {
         System.out.println (
            "WARNING: error reading exciter file "+file+
            ":\n" + e.getMessage());
      }
   }

   public static void loadExciters(FemMuscleModel fem, String excitersFilename) {
      try {
         ReaderTokenizer rtok = ArtisynthIO.newReaderTokenizer(new File(
            ArtisynthPath.getSrcRelativePath(FemMuscleTongueDemo.class,
               "exciters/lists/" + excitersFilename)));

         // ignore first line
         if (rtok.nextToken() != ReaderTokenizer.TT_WORD)
            throw new IOException("cannot create instance of " + rtok.sval);
         ScanWriteUtils.scanfull (rtok, fem.getMuscleExciters(), fem);
      } catch (IOException e) {
         e.printStackTrace();
      }
   }

   public static void addBilateralExciters(FemMuscleModel fem) {

      // add exciters if 21 muscle groups: 20 paired, 1 unpaired
      if (fem.getMuscleBundles().size() == 21) {
         for (int i = 0; i < 20; i += 2) {
            MuscleBundle left = fem.getMuscleBundles().get(i);
            MuscleBundle right = fem.getMuscleBundles().get(i + 1);
            String[] name = left.getName().split("_");
            MuscleExciter ex = new MuscleExciter(name[0]);
            ex.addTarget(left, 1.0);
            ex.addTarget(right, 1.0);
            fem.addMuscleExciter(ex);
         }
         // add exciter for unpaired muscle group (SL)
         MuscleBundle unpaired = fem.getMuscleBundles().get(20);
         MuscleExciter ex = new MuscleExciter(unpaired.getName());
         ex.addTarget(unpaired, 1.0);
         fem.addMuscleExciter(ex);
      }

   }

   ControlPanel myControlPanel;
   ControlPanel myMusclePanel;

   public void attach(DriverInterface driver) {
      // setSagittalView(tongue);
      if (myControlPanels.size() == 0 && tongue != null) {
         FemControlPanel.createControlPanel(this, tongue, myModels.get(0));
         FemControlPanel.createMuscleExcitersPanel(this, tongue);
      }
      // createProbes(1.0);
      // e

      // moved to main constructor
      // addExcitationProbes();
   }

   public static final int frameWidthOffset = 600 - 547;
   public static final int frameHeightOffset = 600 - 517;

   public static void setSagittalView(FemMuscleModel tongue) {
      setSagittalView(tongue, true, 1d);
   }

   public static void setSagittalView(FemMuscleModel tongue, boolean clipped,
      double scale) {
      setSagittalView(tongue, clipped, scale, 600, 600);
   }

   public static void setSagittalView(FemMuscleModel tongue, boolean clipped,
      double scale, int width, int height) {
      
      GLViewer v = Main.getMain().getViewer();
      if (v == null) {
         return;
      }
      Main.getMain().getMainFrame().setSize(
         width + frameWidthOffset, height + frameHeightOffset);

      v.setOrthographicView(true);
      v.setGridVisible(true);

      if (v.getNumClipPlanes() < 1) {
         v.addClipPlane();
      }
      GLClipPlane clip = v.getClipPlane (0);

      clip.setPosition(new Point3d());
      clip.setOrientation(new AxisAngle(1, 0, 0, Math.PI / 2));
      double gridOffset = 0.003 * scale; // 0.005 // to see muscle dirs
      clip.setOffset(gridOffset);
      clip.setGridVisible(false);
      clip.setDragger(DraggerType.None);

      double nearz = 0.001 * scale;
      double farz = 1d * scale;
      double fieldHeight = 0.1 * scale;
      Point3d eye = new Point3d(0.1, -0.4, 0.096);
      eye.scale(scale);
      Point3d centre = new Point3d(0.1, 0, 0.095);
      centre.scale(scale);

      v.setOrthogonal(fieldHeight, nearz, farz);
      v.setEye(eye);
      v.setCenter(centre);

      if (clipped) {
         try {

            tongue.scanSurfaceMesh(geometrypath + "/tongue_righthalf.smesh");
         } catch (IOException e) {
            e.printStackTrace();
         }
      }
      else {
         // v.myUnClippedRenderList.clear();
         // v.myUnClippedRenderList.addIfVisible(tongue);
         // v.removeRenderable(tongue);
      }
      tongue.setElementWidgetSize(0);
      tongue.setSurfaceRendering(SurfaceRender.Stress);
   }

   public static void setSliderRange(ControlPanel panel, String propName,
      double min, double max) {
      for (Component comp : panel.getPropertyPanel().getWidgets()) {
         if (comp instanceof LabeledComponentBase) {
            String widgetPropName =
               PropertyWidget.getProperty((LabeledComponentBase)comp).getName();
            if (widgetPropName.compareTo(propName) == 0) {
               if (comp instanceof DoubleFieldSlider) {
                  ((DoubleFieldSlider)comp).setRange(min, max);
               }
            }
         }
      }
   }

   public void addMexProbe(MuscleExciter mex, double duration, double ex) {
      NumericInputProbe ip =
         new NumericInputProbe(mex, "excitation", 0, duration);
      ip.addData(0, new VectorNd(1));
      VectorNd exData = new VectorNd(1);
      exData.set(0, ex);
      ip.addData(duration, exData);
      if (mex.getName() != null)
         ip.setName(mex.getName());
      addInputProbe(ip);
   }

}
