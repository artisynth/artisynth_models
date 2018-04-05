package artisynth.models.larynx_QL2.tools;

import java.awt.Color;
import java.awt.GraphicsEnvironment;

import javax.swing.JFrame;
import javax.swing.JSeparator;

import maspack.widgets.DoubleFieldSlider;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.RootModel;
import artisynth.models.larynx_QL2.components.HardBody;
import artisynth.models.larynx_QL2.components.SoftBody;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods;
import artisynth.models.larynx_QL2.components.Structures;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class ControlTools {


	public static void createFemControlPanel(RootModel root, MechModel mech, FemMuscleModel fem) {
		ControlPanel masterControlPanel = new ControlPanel(fem.getName() + " controls", "LiveUpdate");
		masterControlPanel.setScrollable(true);
		String name = fem.getName();
		FemControlPanel.createControlPanel(root, fem, mech);
		masterControlPanel.addWidget(name + " nodes visible", fem, "nodes:renderProps.visible");
		masterControlPanel.addWidget(name + " elements visible", fem, "elements:renderProps.visible");
		masterControlPanel.addWidget(name + " elements alpha", fem,  "elements:renderProps.alpha", 0.0, 1.0);
		masterControlPanel.addWidget(name + " nodes radius", fem,  "nodes:renderProps.pointRadius", 0.0, 1.0);
		masterControlPanel.addWidget(name + " muscles visible", fem, "bundles:renderProps.visible");
		masterControlPanel.addWidget(new JSeparator());
		root.addControlPanel(masterControlPanel);
	}

	public static ControlPanel createControlPanels(RootModel root, JFrame frame, final MechModel mech, final Structures structs) {
		int x = (GraphicsEnvironment.getLocalGraphicsEnvironment().getScreenDevices().length > 1 ? 1920 : 0);
		int y = frame.getLocation().y;
		int muscleLocY = 0;

		//Master control panel
		ControlPanel masterControlPanel = new ControlPanel("Master Controls", "LiveUpdate");
		masterControlPanel.setScrollable(true);

		for (SoftBody sb : Structures.getSoftBodies()) {
			if (sb.isUsed()) {
				String name = sb.getName();

				FemMuscleModel fem = sb.getFem();
				ControlPanel cp = FemControlPanel.createControlPanel(root, fem, mech);
				cp.addWidget(name + " elements alpha", fem,  "elements:renderProps.alpha", 0.0, 1.0);
				cp.addWidget(name + " nodes radius", fem,  "nodes:renderProps.pointRadius", 0.0, 1.0);
				
				masterControlPanel.addWidget(name + " nodes visible", fem, "nodes:renderProps.visible");
				masterControlPanel.addWidget(name + " elements visible", fem, "elements:renderProps.visible");
				masterControlPanel.addWidget(name + " elements alpha", fem,  "elements:renderProps.alpha", 0.0, 1.0);
				masterControlPanel.addWidget(name + " nodes radius", fem,  "nodes:renderProps.pointRadius", 0.0, 1.0);
				masterControlPanel.addWidget(name + " muscles visible", fem, "fibers.:renderProps.visible");
				masterControlPanel.addWidget(new JSeparator());
				
				
			}
		}

		for (HardBody hb : Structures.getHardBodies()) {
			if (hb.isUsed()) {
				RigidBody rb = hb.getBody();
				masterControlPanel.addWidget(rb.getName() + " visible", rb, "renderProps.visible"); 
			}
		}

		masterControlPanel.pack();
		masterControlPanel.setLocation(x, y);
		root.addControlPanel(masterControlPanel);

		for (SoftBody sb : Structures.getSoftBodies()) {
			//Create muscle control panels
			if (sb.isUsed() && sb.musclesEnabled()) {
				createFemMusclePanel(root, sb.getFem(), x += masterControlPanel.getWidth(), y);
			}
		}
		
		createRigidBodyControlPanel(root, mech, "Rigid Body Muscles", x, muscleLocY);
		root.mergeAllControlPanels(true);
		return masterControlPanel;
	}
	
	public static void createRigidBodyControlPanel(RootModel root, MechModel mech, String panelName, int x, int y) {
		//Rigid body muscles control panel
		ControlPanel rigidBodyMusclesControlPanel = new ControlPanel(panelName, "LiveUpdate");
		rigidBodyMusclesControlPanel.setScrollable(true);
		addExcitersToPanel(rigidBodyMusclesControlPanel, mech);
		rigidBodyMusclesControlPanel.pack();
		rigidBodyMusclesControlPanel.setLocation(x, y);
		root.addControlPanel(rigidBodyMusclesControlPanel);        
	}

	public static ControlPanel createFemMusclePanel(RootModel root, FemMuscleModel fem, int x, int y) {
		ControlPanel muscleControlPanel = new ControlPanel(fem.getName() + " muscles", "LiveUpdate");
		muscleControlPanel.setScrollable(true);
		addExcitersToPanel(muscleControlPanel, fem, FemMethods.defaultFemMuscleColor);
		muscleControlPanel.setLocation(x, y);
		muscleControlPanel.pack();
		root.addControlPanel(muscleControlPanel);
		return muscleControlPanel;
	}

	private static void addExcitersToPanel(ControlPanel panel, FemMuscleModel fem, Color color) {
		for (MuscleExciter mex : fem.getMuscleExciters()) {
			DoubleFieldSlider slider = (DoubleFieldSlider) panel.addWidget(mex.getName(), fem, "exciters/" + mex.getNumber() + ":excitation", 0, 1);
			slider.setRoundingTolerance(0.00001);
		}
	}

	private static void addExcitersToPanel(ControlPanel panel, MechModel mech) {
		for (MuscleExciter mex : Structures.getMuscleTissueExciters()) {
			DoubleFieldSlider slider = (DoubleFieldSlider) panel.addWidget(mex.getName(), mech, "exciters/" + mex.getNumber() + ":excitation", 0, 1);
			slider.setRoundingTolerance(0.00001);
		}
	}
}
