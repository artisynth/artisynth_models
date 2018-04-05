package artisynth.models.larynx_QL2;

import java.awt.event.KeyAdapter;
import java.awt.event.KeyListener;
import java.io.File;
import java.util.ArrayList;

import javax.swing.event.MouseInputListener;

import maspack.matrix.AxisAngle;
import maspack.render.Renderable;
import maspack.render.GL.GLViewer;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.driver.Main;
import artisynth.core.driver.SchedulerListener;
import artisynth.core.driver.Scheduler.Action;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.selectionManager.SelectionListener;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.ComponentChangeListener;
import artisynth.core.workspace.RootModel;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.larynx_QL2.components.Names.VocalTractType;
import artisynth.models.larynx_QL2.tools.ControlTools;
import artisynth.models.larynx_QL2.tools.RenderTools;
//import artisynth.models.larynx_QL2.tools.ViewerTools;
import artisynth.models.larynx_QL2.tools.AuxTools.Timer;
import artisynth.models.larynx_QL2.components.SoftBody;
import artisynth.models.larynx_QL2.components.Structures;
import artisynth.models.larynx_QL2.DriverListener;

/** Builds a vocal tract foundation for constructing simulations. Specific simulations should be created in their own unique class (stored in the {@link simulations} package) and <b>must extend</b> the {@link artisynth.models.moisik.simulations.AbstractSimulation Simulation} class, which itself extends
 *  the VocalTractBase class, provides additional convenience features for building simulations, and enforces the implementation of driver-related methods provided by the {@link DriverListener} class.
 *  It is important <b>not</b> to modify the base classes (i.e. {@link VocalTractBase}, {@link Structures}, {@link artisynth.models.moisik.simulations.AbstractSimulation Simulation}, and so forth) so as to ensure proper model functioning.  
 *  <p>
 *  Specific simulations specify which vocal tract model type to use by means of the {@link artisynth.models.moisik.components.Names.VocalTractType VocalTractType} enum. This enum also contains a mehtod {@link artisynth.models.moisik.components.Names.VocalTractType#getFullFileName getFullFileName}, which is used to build the absolute path name for the particular model.
 *  All vocal tract models have a manditory "skeleton" (jaw, hyoid bone, larynx cartilage framework, etc.) plus optional FEM models for deformable bodies. 
 *  Manditory components can be set to non-dynamic and invisible if they are not desired in a give simulation using  {@link artisynth.models.moisik.components.AbstractBody#setDynamic setDynamic} and {@link artisynth.models.moisik.components.AbstractBody#setVisible setVisible}.
 *  Several wrappers are used to extend the basic ArtiSynth model components with convenience methods and for organization.  
 *  The skeletonal model components (RigidBodies; see {@link HardBody}) and all simple connective tissues (i.e. single and multiple axial springs; see {@link Tissue} and its inhereiting classes {@link Ligamentous} and {@link Contractile})
 *  are setup inside the {@link Structures} class. Deformable bodies (FEMs) are setup by means of the {@link SoftBody} class.
 *  <p>
 *  Several convenience methods can be found in the following classes located in the 'moisik' package (note that many of these classes have useful inner classes):<br>
 *  <UL>
 *  <LI>{@link Render} contains methods for manipulating ArtiSynth rendering, viewer configuration, and model render properties.
 *  <LI>{@link Control} contains methdods for generating control panels.
 *  <LI>{@link ParameterTools} contains methdods for manipulating parameters of model components and for creating numeric probes.
 *  <LI>{@link Utility} contains an assortment of convenience methods, including a nested class {@link EditTools} which allows for the specification of custom event handling. 
 *  <LI>{@link FemMethods} contains methdods for creating and manipulating FEMs.
 *  <LI>{@link Garbage} contains deprecated code that the user wishes to be temporarily retained.
 *  </UL>
 *  <p>
 *  
 *  Three subpackages contain important classes for creating specific simulations as follows:
 *  <UL>
 *  <LI>The {@link simulations simulations package} stores all high-level classes which specify particular model components. All simulation classes must extend the {@link AbstractSimulation} class.
 *  <LI>The {@link components components package} contains classes that provide methods to construct any model component of interest. {@link Structures} is a singleton class which, once called by the {@link VocalTractBase} constructor, assembles the basic skeletal base of a vocal tract model and any requested FEMs. This class also contains a list of all different tissue and body types with methods to access these in specific simulation classes. The {@link Names} interface provides a redundantly structured list of all model component names (including abbreviations) for the purposes of locating specific structures in specific simulation classes.
 *  <LI>The {@link numerics numerics package} contains methods that allow for simulation of 2nd-order dynamical systems (specified as Ku = f). This is used as an alternative to ArtiSynth's numerical methods for such systems. It's main application is to allow for element-inversion-immune "relaxation" of hex-meshes generated using the methods in {@link FemMethods.ForgedFem}. 
 *  </UL>
 *  <p>
 *  Finally, there are two packages for file organization:
 *  <UL> 
 *  <LI> The {@link data data package} stores all model components with separate subpackages for different vocal tracts. Observe the following conventions:
 *  <UL> 
 *  <LI> All mesh files (for rigid bodies) are stored in <b>OBJ</b> (Wavefront) format.
 *  <LI> All vocal tract skeleton tissues (i.e. axial spring ligaments and muscles) are stored in <b>CSV</b> format as outlined in the {@link artisynth.models.moisik.components.Tissue#storeTissueData storeTissueData} and {@link artisynth.models.moisik.components.Tissue#createTissuesFromCSV createTissuesFromCSV} methods in the {@link components} package.
 *  <LI> All FEMs are saved in <b>CSV</b> format as outlined in the {@link artisynth.models.moisik.components.FemMethods.FemIOMethods} class. Note that Ansys based (.elem, .node) FEMs can be loaded with the {@link artisynth.core.femmodels.AnsysReader AnsysReader} class, but these should subsequently be saved as <b>CSV</b> files using the {@link artisynth.models.moisik.components.FemMethods.FemIOMethods#saveFemToCSV saveFemToCSV} method for better consistency and readability.  
 *  <LI> All FEM muscle data is stored in <b>CSV</b> format as outlined in the {@link artisynth.models.moisik.components.FemMethods.FemIOMethods#saveMusclesToCSV readMusclesFromCSV} and {@link artisynth.models.moisik.components.FemMethods.FemIOMethods#saveMusclesToCSV saveMusclesToCSV} methods.
 *  </UL>
 *  <LI> The {@link output output package} is a repository for all data output including probe data, screen shots, and movies.
 *  </UL> 
 *
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public abstract class VocalTractBase extends RootModel {
	public static VocalTractBase vtBase;

	public static String modelDirectory;
	public static String dataDirectory = enforceSourcePath(ArtisynthPath.getSrcRelativePath(VocalTractBase.class, "data/"));
	public static String simulationsDirectory = enforceSourcePath(ArtisynthPath.getSrcRelativePath(VocalTractBase.class, "data/simulations/"));
	public static String outputDirectory = enforceSourcePath(ArtisynthPath.getSrcRelativePath(VocalTractBase.class, "output/"));

	public static String enforceSourcePath(String candidatePath) {
		String invalidSourceFolder = File.separator + "bin" + File.separator;
		if (candidatePath.contains(invalidSourceFolder)) {
			candidatePath = candidatePath.replace(invalidSourceFolder, File.separatorChar + "src" + File.separatorChar);
		}
		return candidatePath;
	}

	
	public VocalTractType vtType = VocalTractType.BADIN_FACE;

	//Model components
	public static MechModel mech;
	public Structures structs;
	public static ArrayList<MuscleExciter> inverseExciters = new ArrayList<MuscleExciter>();
	public static ControlPanel masterControlPanel;
	public static ArrayList<Renderable> renderables = new ArrayList<Renderable>();
	
	//Scaling factor for model points (particles, frame markers, etc.) based on model extent
	public double characteristicPointSize = 1.0;

	//Driver listener (guaranteed to be set by the Simulation class)
	private ArrayList<DriverListener> driverListeners = new ArrayList<DriverListener>();

	//Custom listeners
	private static ArrayList<Object> customListeners = new ArrayList<Object>();

	//Model timers
	private static Timer loadTimer;
	private static Timer simulationTimer;

	//Flags
	private boolean showOnSecondaryScreenFlag = true;
	private boolean showTimeLineFlag = false;
	
	public void setShowViewerOnSecondaryScreen(boolean secondaryScreenFlag) { showOnSecondaryScreenFlag = secondaryScreenFlag; }
	public void setTimeLineVisible(boolean visibleFlag) { showTimeLineFlag = visibleFlag; }
	
	public VocalTractBase() {}
	public VocalTractBase(String name) { 
		super(name);
		buildVocalTract(name, vtType); 
	}
	public VocalTractBase(String name, VocalTractType vtType) { 		
		super(name);
		this.vtType = vtType;
		modelDirectory = vtType.getModelDirectory();
		loadTimer = new Timer(name);
		buildVocalTract(name, vtType);
		Main.getMain().clearWayPoints();
	}

	public void addDriverListener(DriverListener driverListener) { this.driverListeners.add(driverListener); }

	public void buildVocalTract(String name, VocalTractType vtModel) {
		vtBase = this;
		loadTimer.start();

		//Set default working directory to repository location
		ArtisynthPath.setWorkingDir(new File(dataDirectory));

		setDefaultViewOrientation(AxisAngle.IDENTITY);

		//Define the base mechanical model & set the world transform
		mech = new MechModel(name);
		mech.setGravity(0.0, -9.81, 0.0);

		//Create all model components and add them
		if (!vtModel.isEmpty()) {
			structs = new Structures(mech, vtModel);
		}
		
		//Add the activated mech model to the root model
		addModel(mech);

		//Setup general render properties
		characteristicPointSize = RenderTools.getModelExtent(mech)*5e-3;
		RenderTools.setupGlobalRenderProperties(mech, characteristicPointSize);
		loadTimer.stop("Time required to load '" + loadTimer.timeeName + "' was ");
		//Create a listener that keeps track of the time
		simulationTimer = new Timer();
		Main.getMain().getScheduler().addListener(new SchedulerListener() {
			@Override
			public void schedulerActionPerformed (Action action) {
				if (action == Action.Play) {
					simulationTimer.start();
				}
				else if (action == Action.Stopped || action == Action.Pause) {
					simulationTimer.stop("Simulation time was ");
				}

			}
		});
	}


	@Override
	public StepAdjustment advance(double t0, double t1, int flags) {
		try {
			super.advance(t0, t1, flags);

			if (!vtType.isEmpty()) {
				if (structs.isStiffeningUsed()) {
					for (SoftBody sb : Structures.getSoftBodies()) {
						if (sb.isUsed() && sb.isStiffening()) {
							sb.stiffen();
						}		
					}		
				}
			}

			for (DriverListener dl : driverListeners) {
				dl.driverAdvance(t0, t1, flags);
			}
		}
		catch (maspack.matrix.NumericalException ne) {

			for (SoftBody sb : Structures.getSoftBodies()) {
				if (sb.isUsed()) {
					sb.printCrashReport();
				}
			}

			Main.getMain().getViewer().rerender();
			Main.getMain().getScheduler().stopRequest();

			//Force repaint
			GLViewer viewer = Main.getMain().getViewer();
			viewer.getCanvas().paint(viewer.getCanvas().getGraphics());

		}
		return null;
	}


	
	@Override
	public void attach(DriverInterface driver) {
		super.attach(driver);
		
		//Add renderable components (mainly text feeders)
		for (Renderable renderable : renderables) {
			addRenderable(renderable);
		}
		

		try {
			for (DriverListener dl : driverListeners) {
				dl.addInverseController();
			}
			/*
			for (Controller controller : getControllers()) {
				if (controller instanceof GeneralInverseController) {
					inverseExciters.add(((GeneralInverseController) controller).getMuscleExciter());
				}
			}
			*/
			Main.getMain().getScheduler().addListener(new SchedulerListener() {
				public void schedulerActionPerformed(Action action) { 
					if (action == Action.Rewind || action == Action.Reset) {
						Structures.resetSoftBodyRendering();
					}
				}
			});

			RenderTools.setTargetPointSize(this);

			if (!vtType.isEmpty()) {
				structs.printStats();
			}
			masterControlPanel = ControlTools.createControlPanels(this, driver.getFrame(), mech, structs);
			
			if (showTimeLineFlag) {
				setTimelineVisible(false);
			}
			
			/*
			if (showOnSecondaryScreenFlag) {
				ViewerTools.showOnScreen(1, driver.getFrame());
			}
			*/
			
			for (DriverListener dl : driverListeners) {
				dl.driverAttach(driver);
			}
			//loadTimer.stop("Time required to load '" + loadTimer.timeeName + "' was ");

		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}

	@Override
	public void detach(DriverInterface driver) {
		super.detach(driver);

		for (Object customListener : customListeners) {
			if (customListener instanceof KeyAdapter) {
				driver.getViewerManager().removeKeyListener((KeyListener) customListener);
			}
			else if (customListener instanceof SelectionListener) {
				Main.getMain().getSelectionManager().removeSelectionListener((SelectionListener) customListener);
			}
			else if (customListener instanceof MouseInputListener) {
				Main.getMain().getViewerManager().removeMouseListener((MouseInputListener) customListener);
			}
		}

		for (DriverListener dl : driverListeners) {
			dl.driverDetach(driver);	
		}
		
	}

	public static VocalTractBase getInstance() { return vtBase; }
	public static MechModel getMech() { return mech; }
	public static String getDataDirectory() { return dataDirectory; }
	public static String getOutputDirectory() { return outputDirectory; }
	public static ArrayList<MuscleExciter> getInverseExciters() { return inverseExciters; }
	public static Timer getSimulationTimer() { return simulationTimer; }
	public static String getModelPath() { return VocalTractBase.vtBase.vtType.getModelDirectory(); }
	public static void setTimelineVisible(boolean visibleFlag) { Main.getMain().getTimeline().setVisible(visibleFlag); }
	
	public Structures getStructures() { return structs; }
	public static void addCustomListener(Object customListener) { 
		customListeners.add(customListener);

		if (customListener instanceof KeyAdapter) {
			Main.getMain().getViewerManager().addKeyListener((KeyAdapter) customListener);
		}
		else if (customListener instanceof SelectionListener) {
			Main.getMain().getSelectionManager().addSelectionListener((SelectionListener) customListener);
		}
		else if (customListener instanceof ComponentChangeListener) {
			vtBase.addComponentChangeListener((ComponentChangeListener) customListener);
		}
		else if (customListener instanceof MouseInputListener) {
			Main.getMain().getViewerManager().addMouseListener((MouseInputListener) customListener);
		}

	}
	public void scheduleRenderableAdd(Renderable renderable) {
		renderables.add(renderable);
	}
}
