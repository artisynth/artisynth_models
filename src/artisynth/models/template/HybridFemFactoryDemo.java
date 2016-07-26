package artisynth.models.template;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import javax.swing.JSeparator;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.PointStyle;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.NumericProbePanel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.Model;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.tools.femtool.HybridFemFactory;
import artisynth.tools.femtool.HybridFemGenerator;
import artisynth.tools.femtool.VolumetricMeshGenerator;
import artisynth.tools.femtool.HybridFemGenerator.TemplateType;

public class HybridFemFactoryDemo extends RootModel {

   // ******************************
   int templateCase = 0; // 0-beam, 1-cylinder, 2-ellipsoid, others-ellipsoid 2
   boolean showExample = true;
   boolean showTemplate = false;

   boolean fixNode = true;
   double xFix = -230;
   boolean shaded = true;
   boolean showBox = true;
   // ******************************

   MechModel myMechMod;

   public HybridFemFactoryDemo () {
      super();
   }

   public HybridFemFactoryDemo (String name) throws IOException {
      super(name);

      // create mech model
      myMechMod = new MechModel("mech");
      addModel(myMechMod);

      FemModel3d myFemMod;

      if (showExample) {
         // Rigid Body
         String bodyPath, fileName;

         
         bodyPath =  ArtisynthPath.getHomeDir() + "/src/artisynth/models/alanMasseter/data/";
         fileName = "MasseterVolMaya.obj";
         RigidBody rb = addBody(bodyPath, fileName);
         
         
         bodyPath =
            ArtisynthPath.getHomeDir() + "/src/artisynth/models/phuman/data/";
         fileName = "flexor_digitorum_superficialis_wrap.obj";
         addBody(bodyPath, fileName);

         fileName = "extensor_carpi_radialis_longus_wrap.obj";
         addBody(bodyPath, fileName);

         fileName = "flexor_carpi_radialis_wrap.obj";
         addBody(bodyPath, fileName);

         /*
          * bodyPath = ArtisynthPath.getHomeDir()+
          * "/src/artisynth/models/palate/geometry/"; fileName =
          * "tongueIan.obj"; addBody(bodyPath,fileName);
          * 
          * bodyPath = ArtisynthPath.getSrcRelativePath ( ModelTemplate.class,
          * "geometry/rigidBodies/"); fileName = "fish.obj";
          * addBody(bodyPath,fileName); myMechMod.rigidBodies ().get
          * (parseName(fileName)).getMesh ().transform (new
          * RigidTransform3d(0,0,-100));
          */

         HybridFemGenerator femFactory = new HybridFemGenerator();
         
         if (templateCase == 0) {
            System.out.println("Case 0: Using hex beam template.");
            femFactory.setTemplateType(TemplateType.BEAM);
            femFactory.setBorderPercentage(0.02);
         } else if (templateCase == 1) {
            System.out.println("Case 1: Using hex cylinder template.");
            femFactory.setTemplateType(TemplateType.CYLINDER);
            femFactory.setBorderPercentage(0.03);
         } else if (templateCase == 2) {
            System.out.println("Case 2: Using hybrid ellipsoid template.");
            femFactory.setTemplateType(TemplateType.SPINDLE);
            femFactory.setBorderPercentage(0.01);
         } else {
            System.out.println("Case 3: Using hybrid ellipsoid template 2.");
            femFactory.setTemplateType(TemplateType.SPINDLE);
            femFactory.setBorderPercentage(0.03);
         }
         
         for (RigidBody body : myMechMod.rigidBodies()) {
            if (showBox) {
               drawBox(body);
            }
            
            if (templateCase <3 ) {
               myFemMod =
                  femFactory.generate(body.getMesh(), new int[] {20,8,4});
            } else {
               myFemMod = femFactory.generate(body.getMesh(), new int[]{20,8,4,6});
            }

            addFEM(myFemMod, body.getName());
            checkInvertedElements(myFemMod);
         }
      }

      if (showTemplate) {
         double l = 60, r_in = 5, r_out = 8;
         double w_in = Math.sqrt(2) * r_in;
         double w_out = Math.sqrt(2) * r_out;
         double w_margin = w_out - w_in;

         int nl = 10, n_in = 4, n_margin = 2;

         // Basic Hex Cross
         myFemMod =
            HybridFemFactory.createHexCross(
               l, w_in, w_margin, nl, n_in, n_margin);
         myFemMod.transformGeometry(new RigidTransform3d(-200, 0, 60));
         addFEM(myFemMod, "HexCross");

         // Basic Rounded Beam
         myFemMod =
            HybridFemFactory.createHexRoundedBeam(
               l, w_in, w_margin, nl, n_in, n_margin);
         myFemMod.transformGeometry(new RigidTransform3d(-200, 0, 30));
         addFEM(myFemMod, "HexRoundedCylinder");

         // Basic Cylinder
         myFemMod =
            HybridFemFactory.createHexCylinder(
               l, r_in, r_out, nl, n_in, n_margin);
         myFemMod.transformGeometry(new RigidTransform3d(-200, 0, 0));
         addFEM(myFemMod, "HexCylinder");

         // Basic Ellipsoid
         myFemMod =
            HybridFemFactory.createHybridMuscleEllipsoid(
               l, r_in, r_out, nl, n_in, n_margin);
         myFemMod.transformGeometry(new RigidTransform3d(-200, 0, -30));
         addFEM(myFemMod, "HybridEllipsoid");

         // Basic Ellipsoid, n_end
         myFemMod =
            HybridFemFactory.createHybridMuscleEllipsoid(
               l, r_in, r_out, nl, n_in, n_margin, 2);
         myFemMod.transformGeometry(new RigidTransform3d(-200, 0, -60));
         addFEM(myFemMod, "HybridEllipsoid_2");
      }

      createVisibilityPanel();
   }

   private void drawBox(RigidBody body) {
      RigidTransform3d trans =
         VolumetricMeshGenerator.getPrincipalAxes(body.getMesh());
      Point3d[] edgePoints =
         VolumetricMeshGenerator.getTightBox(body.getMesh(), trans);

      // trans.R.rotateZDirection(new
      // Vector3d(trans.R.m00,trans.R.m10,trans.R.m20));
      Vector3d principalAxis_x = new Vector3d();
      Vector3d principalAxis_y = new Vector3d();
      Vector3d principalAxis_z = new Vector3d();
      trans.R.getColumn(0, principalAxis_x);
      trans.R.getColumn(1, principalAxis_y);
      trans.R.getColumn(2, principalAxis_z);

      drawLine(edgePoints[0], edgePoints[1]);
      drawLine(edgePoints[1], edgePoints[2]);
      drawLine(edgePoints[2], edgePoints[3]);
      drawLine(edgePoints[3], edgePoints[0]);
      drawLine(edgePoints[0], edgePoints[4]);
      drawLine(edgePoints[1], edgePoints[5]);
      drawLine(edgePoints[2], edgePoints[6]);
      drawLine(edgePoints[3], edgePoints[7]);
      drawLine(edgePoints[5], edgePoints[4]);
      drawLine(edgePoints[6], edgePoints[5]);
      drawLine(edgePoints[7], edgePoints[6]);
      drawLine(edgePoints[4], edgePoints[7]);

      double scaleLength = edgePoints[0].distance(edgePoints[1]);
      principalAxis_x.scale(scaleLength);
      principalAxis_y.scale(scaleLength);
      principalAxis_z.scale(scaleLength);

      drawVector(new Point3d(trans.p), principalAxis_x, Color.RED);
      drawVector(new Point3d(trans.p), principalAxis_y, Color.GREEN);
      drawVector(new Point3d(trans.p), principalAxis_z, Color.BLUE);
   }

   private void addFEM(FemModel3d fem, String name) {
      fem.setName(name);
      if (shaded) {
         fem.setElementWidgetSize(1.0);
         // fem.setSurfaceRendering (SurfaceRender.Shaded);
      }
      if (fixNode) {
         for (FemNode3d n : fem.getNodes()) {
            if (n.getPosition().x <= xFix) {
               n.setDynamic(false);
               RenderProps.setPointStyle(n, PointStyle.SPHERE);
               RenderProps.setPointColor(n, Color.CYAN);
            }
         }
      }

      RenderProps.setFaceStyle(fem, Renderer.FaceStyle.FRONT);
      RenderProps.setFaceColor(fem, Color.PINK);
      RenderProps.setLineWidth(fem, 2);
      RenderProps.setLineColor(fem, Color.GRAY);
      RenderProps.setPointRadius(fem, 0.5);
      myMechMod.addModel(fem);
   }

   private RigidBody addBody(String filepath, String filename) throws IOException {
      RigidBody body = new RigidBody(parseName(filename));
      body.setMesh(new PolygonalMesh(new File(filepath + filename)), null);

      // body.getMesh ().scale (0.5);

      myMechMod.addRigidBody(body);
      RenderProps.setFaceColor(body, NumericProbePanel.colorList[
         myMechMod.rigidBodies().size() % NumericProbePanel.colorList.length]);
      RenderProps.setAlpha(body, 0.3);
      RenderProps.setVisible(body, true);
      
      return body;
   }

   private String parseName(String filename) {
      return filename.substring(0, filename.lastIndexOf('.'));
   }

   private void drawLine(Point3d c, Point3d d) {
      AxialSpring spring0 = new AxialSpring(50, 20, 10);
      RenderProps.setLineStyle(spring0, Renderer.LineStyle.CYLINDER);
      RenderProps.setLineRadius(spring0, 0.1);
      RenderProps.setLineColor(spring0, Color.GREEN);

      Particle p0 = new Particle(5, c);
      RenderProps.setPointRadius(p0, 0.1);
      RenderProps.setPointStyle(p0, Renderer.PointStyle.POINT);
      RenderProps.setPointColor(p0, Color.BLUE);
      p0.setDynamic(true);
      myMechMod.addParticle(p0);

      Particle p1 = new Particle(5, d);
      RenderProps.setPointRadius(p1, 0.2);
      RenderProps.setPointStyle(p1, Renderer.PointStyle.SPHERE);
      RenderProps.setPointColor(p1, Color.RED);
      p0.setDynamic(true);
      myMechMod.addParticle(p1);

      myMechMod.attachAxialSpring(p0, p1, spring0);
   }

   private void drawVector(Point3d c, Vector3d u, Color co) {
      AxialSpring spring0 = new AxialSpring(50, 20, 10);
      RenderProps.setLineStyle(spring0, Renderer.LineStyle.CYLINDER);
      RenderProps.setLineRadius(spring0, 0.1);
      RenderProps.setLineColor(spring0, co);

      Particle p0 = new Particle(5, c);
      myMechMod.addParticle(p0);

      Point3d p = new Point3d(c);
      p.scaledAdd(1, u);
      Particle p1 = new Particle(5, p);
      RenderProps.setPointRadius(p1, 0.1);
      RenderProps.setPointStyle(p1, Renderer.PointStyle.SPHERE);
      RenderProps.setPointColor(p1, Color.RED);
      myMechMod.addParticle(p1);
      myMechMod.attachAxialSpring(p0, p1, spring0);
   }

   private void checkInvertedElements(FemModel3d fem) {
      int count = 0;
      for (FemElement3d e : fem.getElements()) {
         if (e.isInvertedAtRest()) {
            count++;
            RenderProps.setLineColor(e, Color.RED);
            RenderProps.setLineWidth(e, 5);
         }
      }
      if (count > 0) {
         System.out.println("Warning: " + count + " inverted elements in \""
            + fem.getName() + "\"!");
      }
   }

   public void createVisibilityPanel() {

      if (myMechMod == null)
         return;
      ControlPanel panel = new ControlPanel("Show", "LiveUpdate");

      for (RigidBody body : myMechMod.rigidBodies()) {
         if (!body.getName().matches("ref_block"))
            panel.addWidget(body.getName(), body, "renderProps.visible");
      }
      panel.addWidget(new JSeparator());
      for (Model mod : myMechMod.models()) {
         panel.addWidget(mod.getName(), mod, "renderProps.visible");
      }
      addControlPanel(panel);
   }
}
