package artisynth.models.palate;

import java.awt.Color;
import java.util.ArrayList;
import java.io.File;
import java.io.IOException;

import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.PathFinder;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.PointParticleAttachment;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.jawTongue.BadinJawHyoidTongue;
import artisynth.models.jawTongue.JawHyoidFemMuscleTongue;
import artisynth.models.template.ModelTemplate;
import artisynth.models.tongue3d.HexTongueDemo;

public class JawTongueSoftPalate extends JawHyoidFemMuscleTongue {
   FemMuscleModel softPalate;
   String probesFilename = "probes.art";
   public JawTongueSoftPalate () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      String palateDir = PathFinder.findSourceDir (this);
      System.out.println ("palateDir=" + palateDir);
      ArtisynthPath.setWorkingDir (new File(palateDir + "/data"));
      
      // Add Soft Palate
      softPalate = RegisteredSoftPalate.createSoftPalate(
         palateDir + "/geometry/",
         "softPalate_v2_smoothed",
         "landmarks_v2_smoothed",
         /*linearMaterial=*/true);

      softPalate.setName ("softPalate");
      myJawModel.addModel (softPalate);
      RegisteredSoftPalate.setSoftPalateRendering (softPalate);
      RegisteredSoftPalate.anchorPalate(softPalate, myJawModel);
      
      // Attach Points
      ArrayList<AxialSpring> springConnectors = new ArrayList<AxialSpring>();
      // PG R
      AxialSpring PG_R1 = new AxialSpring("PG_R1",500,0.001,1); 
      PG_R1.setFirstPoint(tongue.getNode(578));
      PG_R1.setSecondPoint(softPalate.getNode(48));
      PG_R1.setRestLength(2.0);
      AxialSpring PG_R2 = new AxialSpring("PG_R2",500,0.001,1);
      PG_R2.setName("PG_R2");
      PG_R2.setFirstPoint(tongue.getNode(710));
      PG_R2.setSecondPoint(softPalate.getNode(50));
      PG_R2.setRestLength(1.5);
      myJawModel.addAxialSpring (PG_R1);
      myJawModel.addAxialSpring (PG_R2);
      springConnectors.add(PG_R1);
      springConnectors.add(PG_R2);
      // PG L
      AxialSpring PG_L1 = new AxialSpring("PG_L1",500,0.001,1);
      PG_L1.setName("PG_L1");
      PG_L1.setFirstPoint(tongue.getNode(165));
      PG_L1.setSecondPoint(softPalate.getNode(1237));
      PG_L1.setRestLength(2.0);
      AxialSpring PG_L2 = new AxialSpring("PG_L2",500,0.001,1);
      PG_L2.setName("PG_L2");
      PG_L2.setFirstPoint(tongue.getNode(297));
      PG_L2.setSecondPoint(softPalate.getNode(1206));
      PG_L2.setRestLength(1.5);
      myJawModel.addAxialSpring (PG_L1);
      myJawModel.addAxialSpring (PG_L2);
      springConnectors.add(PG_L1);
      springConnectors.add(PG_L2);
      
      // Set collision
      myJawModel.setCollisionBehavior(tongue, softPalate, true);
      //softPalate.setGravity(0, 0, 0);
      //tongue.setGravity(0, 0, 0);
      
      for(AxialSpring s:springConnectors) {
	 RenderProps.setPointStyle(s.getFirstPoint(), PointStyle.SPHERE);
	 RenderProps.setPointStyle(s.getSecondPoint(), PointStyle.SPHERE);
	 RenderProps.setPointRadius(s.getFirstPoint(), 0.5);
	 RenderProps.setPointRadius(s.getSecondPoint(), 0.5);
	 RenderProps.setPointColor(s.getFirstPoint(), Color.RED);
	 RenderProps.setPointColor(s.getSecondPoint(), Color.RED);
	 RenderProps.setVisible(s.getFirstPoint(), true);
	 RenderProps.setVisible(s.getSecondPoint(), true);
	 RenderProps.setLineStyle (s, LineStyle.CYLINDER);
	 RenderProps.setLineColor (s, Color.PINK);
	 RenderProps.setLineRadius (s, 0.3);
	 RenderProps.setVisible(s, true);
      }
      for(BodyConnector c: myJawModel.bodyConnectors()) {
	 RenderProps.setVisible(c, false);
      }
      RenderProps.setAlpha(myJawModel.rigidBodies().get("maxilla"), 0.2);
      RenderProps.setAlpha(myJawModel.rigidBodies().get("jaw"), 0.2);
      
      RenderProps.setVisible(myJawModel.rigidBodies().get("maxilla"), false);
      //RenderProps.setVisible(myJawModel.rigidBodies().get("jaw"), false);
      RenderProps.setVisible(myJawModel.axialSprings(), false);
      RenderProps.setVisible(myJawModel.multiPointSprings(), false);
      RenderProps.setVisible(myJawModel.particles(), false);
      
   }
   ControlPanel myControlPanel = null;
   public void attach (DriverInterface driver) {
      super.attach (driver);
      loadProbes();
      ModelTemplate.addWayPoints(this, 1.2d, 0.01);
      
      myControlPanel = RegisteredSoftPalate.createControlPanel (this,softPalate, myJawModel);
      if (softPalate.getMuscleBundles().size()>0)
	 RegisteredSoftPalate.createMusclePanel(this, softPalate);
   }
   public void loadProbes()
   {
      if (probesFilename == null || !myInputProbes.isEmpty () || !myOutputProbes.isEmpty ())
         return;

      String probeFileFullPath = ArtisynthPath.getWorkingDir().getPath() + "/"
		+ probesFilename;
      try
      {
	 scanProbes(
		  ArtisynthIO.newReaderTokenizer(probeFileFullPath));
	 System.out.println("Loaded Probes from File: " + probeFileFullPath);
      }
      catch (Exception e)
      {
	 if (debug)
	    System.out.println("Error reading probe file");
	 //e.printStackTrace();
      }
   }
   
}
