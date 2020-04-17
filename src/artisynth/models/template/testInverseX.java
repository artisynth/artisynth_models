package artisynth.models.template;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.widgets.BooleanSelector;
import maspack.widgets.DoubleFieldSlider;
import maspack.widgets.PropertyWidget;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;
import maspack.util.PathFinder;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.NumericProbePanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.core.inverse.TrackingController;

public class testInverseX extends RootModel {

   public static String rbpath =
      ArtisynthPath.getHomeRelativePath (
         "src/maspack/geometry/sampleData/", ".");
   
   MechModel myMechMod;
   FemMuscleModel fem1, fem2;
   
   public testInverseX() {
      
   }
   public testInverseX(String name) {
      super(name);
      
      File workingDir = new File (
         PathFinder.getSourceRelativePath(this, "data"));
      ArtisynthPath.setWorkingDir(workingDir);
      
      myMechMod = new MechModel ("mech");
      
      RigidBody body = new RigidBody("box");
      setBodyMesh(body,rbpath+"box.obj",1);
      myMechMod.addRigidBody(body);
      
      fem1 = new FemMuscleModel ("fem1");
      FemFactory.createHexGrid(fem1, 2, 0.5, 0.5, 5, 2, 2);
      fem1.transformGeometry(new RigidTransform3d(2,0,0.5));
      RenderProps.setPointStyle (fem1, Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius(fem1, 0.03);
      RenderProps.setLineWidth(fem1, 2);
      fem1.setDensity(1);
      fem1.setLinearMaterial (50000, 0.33, true);
      
      fem2 = new FemMuscleModel ("fem2");
      FemFactory.createHexGrid(fem2, 2, 0.5, 0.5, 5, 2, 2);
      fem2.transformGeometry(new RigidTransform3d(2,0,-0.5));
      RenderProps.setPointStyle (fem2, Renderer.PointStyle.SPHERE);
      RenderProps.setPointRadius(fem2, 0.03);
      RenderProps.setLineWidth(fem2, 2);
      fem2.setDensity(1);
      fem2.setLinearMaterial (50000, 0.33, true);
      
      myMechMod.addModel(fem1);
      myMechMod.addModel(fem2);
      
      
      // Attach nodes
      for(FemNode3d n: fem1.getNodes()) {
	 if(n.getPosition().x==1) {
	    myMechMod.attachPoint (n, body);
	    RenderProps.setPointColor (n, Color.BLUE);
	 } else if(n.getPosition().x==3) {
	    n.setDynamic(false);
	    RenderProps.setPointColor (n, Color.GREEN);
	 }
      }
      for(FemNode3d n: fem2.getNodes()) {
	 if(n.getPosition().x==1) {
	    myMechMod.attachPoint (n, body);
	    RenderProps.setPointColor (n, Color.BLUE);
	 } else if(n.getPosition().x==3) {
	    n.setDynamic(false);
	    RenderProps.setPointColor (n, Color.GREEN);
	 }
      }
      
      // Define muscle fibres
      MuscleBundle bundle1 = new MuscleBundle("bundle1");
      fem1.addMuscleBundle(bundle1);
      fem1.setDirectionRenderLen(0.5);
      for (FemElement3d elem : fem1.getElements()) {
	 MuscleElementDesc desc = new MuscleElementDesc();
	 desc.setElement(elem);
	 desc.setDirection(Vector3d.X_UNIT);
	 bundle1.addElement(desc);
      }
      MuscleBundle bundle2 = new MuscleBundle("bundle2");
      fem2.addMuscleBundle(bundle2);
      fem2.setDirectionRenderLen(0.5);
      for (FemElement3d elem : fem2.getElements()) {
	 MuscleElementDesc desc = new MuscleElementDesc();
	 desc.setElement(elem);
	 desc.setDirection(Vector3d.X_UNIT);
	 bundle2.addElement(desc);
      } 
      myMechMod.setGravity(0.0, 0.0, 0.0);
      addModel (myMechMod);
      
      createAllMusclesPanel();
   }
   private void setBodyMesh (RigidBody body, String meshname, double scale) {
      try {
         PolygonalMesh mesh = new PolygonalMesh (new File (meshname));
         mesh.scale (scale);
         body.setMesh (mesh, meshname);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }   
   public void createAllMusclesPanel() {
      if (myMechMod == null)
         return;
      ControlPanel panel = new ControlPanel ("Muscle Controls", "LiveUpdate");
      FemModel3d muscle;
      String name;
      Color color;
      for (int k = 0; k < myMechMod.models().size(); k++)
      {
		 name = myMechMod.models().get(k).getName();
		 //color = getComplementaryColor(muscleList.get(k).color);
		 muscle = (FemModel3d) myMechMod.models().get(name);
		 if (((FemMuscleModel)muscle).getMuscleBundles().size() > 0) {
		         ComponentList<MuscleBundle> muscles =
		            ((FemMuscleModel)muscle).getMuscleBundles();
		         for (int i = 0; i < muscles.size(); ++i) {
		            if(k>=(NumericProbePanel.colorList.length))
		               color = NumericProbePanel.colorList[k%NumericProbePanel.colorList.length];
		            else
		               color = NumericProbePanel.colorList[k];
		            RenderProps.setLineColor (muscles.get(i), color);
		            DoubleFieldSlider slider =
		               (DoubleFieldSlider)panel.addWidget (
		                  name + " [N per Muscle]", myMechMod, 
		                  "models/" + muscle.getName() + "/bundles/" + i + ":excitation", 
		                  0, 1);
		            slider.setRoundingTolerance (0.001);
		            slider.getLabel().setForeground (color);
		            BooleanSelector selector =
		               (BooleanSelector)PropertyWidget.create (
		                  "active", muscles.get (i),
		                  "fibresActive");
		            slider.getLabel().setForeground (color);
		            slider.add (selector);
		            BooleanSelector checkBox =
		               (BooleanSelector)PropertyWidget.create (
		                  "visible", muscles.get (i), "renderProps.visible");
		            slider.add (checkBox);
		            checkBox.addValueChangeListener (new ValueChangeListener() {
		               public void valueChange (ValueChangeEvent e) {
		                  rerender();
		               }
		            });
		            slider.add (checkBox);
		         }
		 }
      }
	 addControlPanel (panel);
   }
   public void attach(DriverInterface driver)
   {
      super.attach(driver);
      addTrackingController();
   }
   
   protected void addTrackingController() {
      TrackingController invcon = new TrackingController(myMechMod, "inverse");
      invcon.addMotionTarget(myMechMod.rigidBodies().get(0));
      
      invcon.addExciter(((FemMuscleModel)fem1).getMuscleBundles().get(0));
      invcon.addExciter(((FemMuscleModel)fem2).getMuscleBundles().get(0));
      invcon.createProbesAndPanel (this);
      addController(invcon);
   }
}
