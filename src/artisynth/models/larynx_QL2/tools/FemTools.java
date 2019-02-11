package artisynth.models.larynx_QL2.tools;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import javax.swing.JSeparator;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.BVFeatureQuery.InsideQuery;
import maspack.matrix.Line;
import maspack.matrix.Matrix3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.PointForce;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.larynx_QL2.VocalTractBase;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemMiscMethods;
import artisynth.models.larynx_QL2.tools.Neighbour;
import artisynth.models.larynx_QL2.tools.AuxTools.FlipAxis;
import artisynth.models.larynx_QL2.tools.ColorTools.ColorMixer;
import artisynth.models.larynx_QL2.tools.FemTools.FemSymmetryMap.SymmetryAxis;
import artisynth.models.larynx_QL2.tools.FemTools.ForgedFem.NeighbourData;
import artisynth.models.larynx_QL2.tools.FemTools.ForgedFem.ProjectionData.NormalRule;
import artisynth.models.larynx_QL2.tools.Neighbour.NeighbourType;
import artisynth.models.larynx_QL2.DriverListener;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class FemTools {

	/** Clears all muscle bundles from the FEM. **/
	public static void removeAllMuscles(FemMuscleModel fem) {
		ArrayList<MuscleBundle> bundles = new ArrayList<MuscleBundle>();
		for (MuscleBundle mb : fem.getMuscleBundles()) {
			bundles.add(mb);
		}

		for (MuscleBundle mb : bundles) {
			fem.removeMuscleBundle(mb);
		}
	}

	public static class HexFemBuilder implements DriverListener {
		public PolygonalMesh meshTarget;
		public FemSymmetryMap fsm;
		public VocalTractBase vtBase;
		double maxElementWidth;

		public HexFemBuilder(VocalTractBase vtBase, PolygonalMesh meshTarget, double maxElementWidth, String fileName) {
			this.vtBase = vtBase;
			this.meshTarget = meshTarget;
			this.maxElementWidth = maxElementWidth;
			vtBase.addDriverListener(this);
			fsm = createSymmetricHexFem(VocalTractBase.mech, meshTarget, maxElementWidth);
			RenderProps.setVisible(fsm.fem.getElements(), true);
			RenderProps.setPointStyleMode(fsm.fem, PropertyMode.Explicit);
			for (FemNode3d node : fsm.fem.getNodes()) {
				RenderProps.setVisible(node, true);
				RenderProps.setPointStyle(node, PointStyle.SPHERE);
				RenderProps.setPointRadius(node, 5e-4);
			}

			VocalTractBase.mech.addModel(fsm.fem);

			//EditTools.Saver.addFemSymmetrySaver('m', fsm, fsm.fem, fileName);
		}

		public FemMuscleModel getFem() { return fsm.fem; }

		public void addEmbeddedMorpher(FemMuscleModel fem, double maxElementWidth) {
			for (FemMeshComp fmc : fem.getMeshComps()) {
				try {
					fem.removeMeshComp(fmc);
				} catch (Exception ex) {

				}
			}
			fem.invalidateSurfaceMesh();
			FemMeshComp sourceMeshComp = fem.getSurfaceMeshComp();
			fem.add(sourceMeshComp);
			FemEmbeddedRelaxer comp = new FemEmbeddedRelaxer(fem, meshTarget, maxElementWidth);
			comp.setName("registration");
			vtBase.addController(comp);
		}

		@Override
		public StepAdjustment driverAdvance(double t0, double t1, int flags) {	
			fsm.symmetrizeFem();

			for (FemElement3d elem : fsm.fem.getElements()) {
				if (elem.isInverted()) {
					System.out.println("Warning: Element " + elem.getNumber() + " is inverted at rest.");
					RenderProps.setFaceColor(elem, Color.pink);
				}
				else {
					RenderProps.setFaceColor(elem, Color.gray);
				}
			}
			return null;
		}

		@Override
		public void driverAttach(DriverInterface driver) {
			//Remove unnecessary nodes
			HashSet<FemNode3d> deleteThese = new HashSet<FemNode3d>();
			deleteThese.clear();
			for (FemNode3d node : fsm.fem.getNodes()) {
				if (node.getElementDependencies().size() < 1) {
					deleteThese.add(node);
				}
			}
			for (FemNode3d node : deleteThese) {
				fsm.fem.removeNode(node);
			}

			addEmbeddedMorpher(fsm.fem, maxElementWidth);
			ControlPanel panel = new ControlPanel("controls");
			FemEmbeddedRelaxer controller = (FemEmbeddedRelaxer) vtBase.getControllers().get("registration");
			if (controller != null) {
				FemEmbeddedRelaxer.addControls(VocalTractBase.masterControlPanel, controller);
			}

			FemControlPanel.addFem3dControls(VocalTractBase.masterControlPanel, fsm.fem, vtBase);

		}

		@Override
		public void driverDetach(DriverInterface driver) {}

		@Override
		public void addInverseController() {}		
	}


	/**
	 * Controller to apply forces from an embedded FemMesh to a target mesh
	 * @author Antonio
	 *
	 */
	public static class FemEmbeddedRelaxer extends ControllerBase {
		public static boolean DEFAULT_ENABLED = true;
		public static double DEFAULT_MAX_PRESSURE = 1.0;
		public static double DEFAULT_SIGMA = 2.5;
		public static boolean DEFAULT_PERPENDICULAR = true;
		public static double DEFAULT_SNAP_THRESHOLD = 0.001;
		public static double DEFAULT_BETA = 0.0;//1e-8;
		public static double DEFAULT_INTERNODAL_DISTANCE = 1.0;
		public static double DEFAULT_PUFF = 0.0;

		private PolygonalMesh targetSurface = null;

		private boolean enabled = DEFAULT_ENABLED;
		private boolean perpendicular = DEFAULT_PERPENDICULAR;
		private double maxPressure = DEFAULT_MAX_PRESSURE;
		private double sigma = DEFAULT_SIGMA;
		private double snap = DEFAULT_SNAP_THRESHOLD;
		private double beta = DEFAULT_BETA;
		private double internodalDistance = DEFAULT_INTERNODAL_DISTANCE;
		private double puff = DEFAULT_PUFF;

		HashMap<FemNode3d,PointForce> nodeToForceMap = null;
		HashMap<FemNode3d,FemNode3d> externalInternalNodeMap = null;
		RenderableComponentList<PointForce> pointForces = null;

		public static PropertyList myProps = new PropertyList(FemEmbeddedRelaxer.class);
		static {
			myProps.add("enabled * *", "Controller enabled", DEFAULT_ENABLED);
			myProps.add("perpendicular * *", "Controller enabled", DEFAULT_PERPENDICULAR);
			myProps.add("maxPressure * *", "Maximum pressure applied over faces", 1.5);
			myProps.add("sigma * *", "Gaussian drop-off of forces", 1);
			myProps.add("snap * *", "Within this distance, forces scale down to zero for stability", 1);
			myProps.add("beta * *", "Internodal restoring force strength", 1);
			myProps.add("internodalDistance * *", "Rest internodal distance", 1);
			myProps.add("puff * *", "Element puffing", DEFAULT_PUFF);
		}


		private FemModel3d fem;
		private HashMap<Integer, ArrayList<Neighbour>> neighbourMap;
		private HashMap<FemNode3d,PointForce> nodeToForceMapFem;

		public PropertyList getAllPropertyInfo() {
			return myProps;
		}

		public FemEmbeddedRelaxer(FemMuscleModel fem, PolygonalMesh target, double internodalDistance) {
			this.internodalDistance = internodalDistance*0.75;
			this.nodeToForceMap = new HashMap<>();
			this.fem = fem;
			this.neighbourMap = Neighbour.getNeighbourMap(fem, internodalDistance);
			this.nodeToForceMapFem = new HashMap<>();
			this.externalInternalNodeMap = new HashMap<FemNode3d, FemNode3d>();
			this.targetSurface = target;
			this.nodeToForceMap = new HashMap<>();

			pointForces = new RenderableComponentList<>(PointForce.class, fem.getName()+ "_forces");
			PolygonalMesh mesh = fem.getSurfaceMesh();
			ArrayList<FemNode3d> surfaceNodes = new ArrayList<FemNode3d>();
			for (int i = 0; i < mesh.numVertices(); i++) {
				FemNode3d node = fem.getSurfaceNode(mesh.getVertex(i));
				if (node != null) {
					surfaceNodes.add(node);
				}
			}

			for (int i = 0; i < mesh.numVertices(); i++) {
				FemNode3d node = fem.getSurfaceNode(mesh.getVertex(i));
				if (node != null) {
					for (Neighbour neighbour : neighbourMap.get(node.getNumber())) {
						if ((neighbour.type == NeighbourType.ORTHOGONAL) && (!surfaceNodes.contains(neighbour.node))) {
							externalInternalNodeMap.put(node, neighbour.node);
							RenderProps.setPointColor(neighbour.node, Color.magenta);
						}
					}
				}
			}			

			//Create point forces for all nodes in the FEM
			for (FemNode3d node : fem.getNodes()) {
				PointForce pf = new PointForce("point_force_" + node.getNumber());
				pf.setPoint(node);
				RenderProps.setVisible(pf, false);
				pointForces.add(pf);
				nodeToForceMapFem.put(node, pf);
			}

			for (FemNode3d node : externalInternalNodeMap.keySet()) {
				PointForce pf = new PointForce("pf_ext_int" + externalInternalNodeMap.get(node).getNumber());
				pf.setPoint(externalInternalNodeMap.get(node));
				RenderProps.setVisible(pf, false);
				RenderProps.setLineColor(pf, Color.green);
				RenderProps.setLineRadius(pf, 1e-4);
				pointForces.add(pf);
				nodeToForceMapFem.put(externalInternalNodeMap.get(node), pf);
			}

			RenderProps.setVisible(pointForces, false);

			for (PointForce f : pointForces) {
				f.setForceScaling (1.0);
			}     
			VocalTractBase.mech.add(pointForces);
		}

		public void apply(double t0, double t1) {
			if (!enabled) {
				//zeroForces();
				return;
			}
			updateForces();
		}

		private void zeroForces() {
			/*
			if (zero) {
				return;
			}

			for (PointForce force : forceList) {
				force.setMagnitude(0);
			}
			zero = true;
			 */
		}


		private void updateForces() {
			PolygonalMesh mesh = fem.getSurfaceMesh();
			//Compute mesh attraction force
			for (int i = 0; i < mesh.numVertices(); i++) {
				FemNode3d node = fem.getSurfaceNode(mesh.getVertex(i));
				if (node != null) {
					//Vector3d force = new Vector3d();					
					Point3d projection = new Point3d();
					BVFeatureQuery.getNearestFaceToPoint(projection, null, targetSurface, node.getPosition());

					Vector3d force = new Vector3d(projection);
					force.sub(node.getPosition());
					double mag = force.norm();

					force.normalize();
					force.scale(Math.min(sigma*(Math.exp(mag) - 1), maxPressure));

					Vector3d neighbourForce = new Vector3d();
					for (Neighbour neighbour : neighbourMap.get(node.getNumber())) {
						Vector3d dist = new Vector3d(neighbour.node.getPosition());
						dist.sub(node.getPosition());
						double ad = dist.norm();	//actual distance
						double id = 1.0;			//desired internodal distance
						double nullify = 1.0;
						switch (neighbour.type) {
						case FACE_DIAGONAL:
							id = Math.sqrt(2*internodalDistance*internodalDistance);
							break;
						case ORTHOGONAL:
							id = this.internodalDistance;
							nullify = 1.0;
							break;
						case VOLUME_DIAGONAL:
							id = Math.sqrt(3*internodalDistance*internodalDistance);
							nullify = 1.0;
							break;
						default:
							id = this.internodalDistance;
							break;

						}						

						double diff = ad - id;
						dist.normalize();
						dist.scale(Math.signum(diff)*beta*Math.exp(-(diff*diff)));
						neighbourForce.add(dist);
					}

					//Set force on this node
					PointForce pf = nodeToForceMapFem.get(node);
					pf.setForce(Vector3d.ZERO);
					if (pf != null) {
						force.add(neighbourForce);
						pf.setForce(force);
					}

					//Set force on this node's internal orthogonal neighbour
					FemNode3d internalNode = externalInternalNodeMap.get(node);
					PointForce pfei = nodeToForceMapFem.get(internalNode);
					Point3d extIntDist = new Point3d(internalNode.getPosition());
					extIntDist.sub(node.getPosition());
					double eiMag = Math.max(internodalDistance - extIntDist.norm(), 0.0);

					extIntDist.normalize();
					extIntDist.scale(puff*eiMag);
					pfei.setForce(extIntDist);
				}
			}
		}

		public void setEnabled(boolean enabled) {
			this.enabled = enabled;
		}

		public boolean getEnabled() {
			return enabled;
		}

		public static void addControls(ControlPanel controlPanel, FemEmbeddedRelaxer controller) {
			controlPanel.addWidget(new JSeparator());
			for (PropertyInfo propInfo : myProps) {
				Property prop = controller.getProperty(propInfo.getName());
				LabeledComponentBase widget = PropertyWidget.create (prop);
				controlPanel.addWidget(widget);
			}
			controlPanel.pack();
		}


		public boolean getPerpendicular() {
			return perpendicular;
		}


		public void setPerpendicular(boolean perpendicular) {
			this.perpendicular = perpendicular;
			updateForces();
		}

		public double getMaxPressure() {
			return maxPressure;
		}

		public void setMaxPressure(double p) {
			maxPressure = p;
			updateForces();
		}

		public double getSigma() {
			return sigma;
		}

		public void setSigma(double s) {
			sigma = s;
			updateForces();
		}

		public void setSnap(double s) {
			snap = s;
			updateForces();
		}

		public double getSnap() {
			return snap;
		}

		public void setBeta(double b) {
			beta = b;
			updateForces();
		}

		public double getBeta() {
			return beta;
		}

		public void setPuff(double p) {
			puff = p;
			updateForces();
		}

		public double getPuff() {
			return puff;
		}
		public void setInternodalDistance(double d) {
			internodalDistance = d;
			updateForces();
		}

		public double getInternodalDistance() {
			return internodalDistance;
		}
	}

	public static FemSymmetryMap createSymmetricHexFem(MechModel mech, PolygonalMesh surfaceTargetMesh, double maxElementWidth) {
		//==========================================================================================================
		//Initialize
		//==========================================================================================================
		MeshComponent targetMeshComp = new MeshComponent(surfaceTargetMesh, "targetMesh", null);
		RenderProps.setFaceColor(targetMeshComp, Color.CYAN.darker());
		RenderProps.setAlpha(targetMeshComp, 0.25);
		mech.addMeshBody(targetMeshComp);

		//Create a new FEM from a voxelization of the surfaceTarget
		FemMuscleModel fem = FemTools.createVoxelFem(null, surfaceTargetMesh, 2, maxElementWidth, false);
		fem.setName("new_fem");
		fem.setDensity(1000.0);
		fem.setMaterial(new LinearMaterial(1.0, 0.2));
		fem.setParticleDamping(500.0); // large damping
		fem.setStiffnessDamping(1.0);

		//Create the symmetry map
		FemSymmetryMap fsm = new FemSymmetryMap(fem, SymmetryAxis.X, maxElementWidth*2.5e-1);
		fsm.cleanFemWithSymmetryMap();

		//==========================================================================================================
		//Trim the FEM
		//==========================================================================================================
		HashSet<FemElement3d> deleteThese = new HashSet<FemElement3d>();
		ArrayList<FemElement3d> checkElements = new ArrayList<FemElement3d>();
		checkElements.addAll(fsm.negativeElements);
		checkElements.addAll(fsm.centerElements);

		boolean meshOutside = false;
		BVFeatureQuery query = new BVFeatureQuery();
		for (FemElement3d element : checkElements) {
			int outsideCount = 0;
			for (FemNode3d node : element.getNodes()) {
				if (meshOutside) {
					InsideQuery iq = query.isInsideMesh(surfaceTargetMesh, node.getPosition(), maxElementWidth*0.05);
					if (iq == InsideQuery.OUTSIDE) {
						outsideCount += 1;
					} 
				} else {

					InsideQuery iq = BVFeatureQuery.isInsideMesh(surfaceTargetMesh, node.getPosition());
					if (iq == InsideQuery.OUTSIDE) {
						outsideCount += 1;
					} else {
						//Check to make sure the node-to-mesh distance is not less than 75% of the max element edge length
						Point3d proj = new Point3d();
						BVFeatureQuery.getNearestFaceToPoint(proj, null, surfaceTargetMesh, node.getPosition());
						if (proj.distance(node.getPosition()) <= maxElementWidth*0.01) {
							outsideCount += 1;
						}
					}
				}
			}
			if (outsideCount >= 1) {
				deleteThese.add(element);
			}
		}

		//Remove elements
		for (FemElement3d elem : deleteThese) {
			fem.removeElement(elem);
			fem.removeElement(fsm.elementSymmetryForward.get(elem));
			fsm.negativeElements.remove(elem);
			fsm.positiveElements.remove(fsm.elementSymmetryForward.get(elem));
		}

		//Remove unnecessary nodes
		HashSet<FemNode3d> deleteNodes = new HashSet<FemNode3d>();
		for (FemNode3d node : fsm.negativeNodes) {
			if (node.getElementDependencies().size() < 1) {
				deleteNodes.add(node);
			}
		}
		for (FemNode3d node : deleteNodes) {
			fem.removeNode(node);
			fem.removeNode(fsm.nodeSymmetryForward.get(node));
			fsm.negativeNodes.remove(node);
			fsm.positiveNodes.remove(fsm.nodeSymmetryForward.get(node));
		}

		//==========================================================================================================
		//Create a shell for the voxelization:
		//==========================================================================================================
		fem.invalidateSurfaceMesh();
		PolygonalMesh femSurface = fem.getSurfaceMesh();
		FemMeshComp sourceMeshComp = fem.addMesh(femSurface);
		RenderProps.setVisible(sourceMeshComp, false);

		HashMap<Integer, ArrayList<Neighbour>> neighbourMap = Neighbour.getNeighbourMap(fem, maxElementWidth);
		HashMap<FemNode3d, ArrayList<Neighbour>> externalNodeNeighbours = new HashMap<FemNode3d, ArrayList<Neighbour>>();
		HashMap<FemNode3d, FemNode3d> externalProjNodes = new HashMap<FemNode3d, FemNode3d>();
		HashMap<FemNode3d, FemNode3d> externalNodesOfProjs = new HashMap<FemNode3d, FemNode3d>();
		HashMap<FemNode3d, Vector3d> externalNormals = new HashMap<FemNode3d, Vector3d>();
		HashMap<FemNode3d, Vector3d> externalNormalsSmoothed = new HashMap<FemNode3d, Vector3d>();
		ArrayList<FemNode3d> externalNodes = new ArrayList<FemNode3d>();

		//Compute normals
		for (int i = 0; i < sourceMeshComp.numVertices(); i++) {
			FemNode3d node = sourceMeshComp.getNodeForVertex(sourceMeshComp.getVertex(i));
			if (fsm.negativeNodes.contains(node)) {
				externalNodes.add(node);
				Vector3d normal = new Vector3d();
				sourceMeshComp.getVertex(i).computeAngleWeightedNormal(normal);
				externalNormals.put(node, normal);
				FemNode3d nodeSymmetryMate = fsm.nodeSymmetryForward.get(node);
				externalNodes.add(nodeSymmetryMate);
				externalNormals.put(nodeSymmetryMate, new Vector3d(AuxTools.flipValue(new Point3d(normal), FlipAxis.X)));
			}
			else if (fsm.centerNodes.contains(node)){
				externalNodes.add(node);
				Vector3d normal = new Vector3d();
				sourceMeshComp.getVertex(i).computeAngleWeightedNormal(normal);
				externalNormals.put(node, normal);
			}
		}

		//Smooth normals
		for (FemNode3d node : externalNodes) {
			Point3d newNormal = new Point3d(externalNormals.get(node));
			double numPoints = 1;
			for (Neighbour neighbour : neighbourMap.get(node.getNumber())) {
				if ((neighbour.type != NeighbourType.VOLUME_DIAGONAL) && (externalProjNodes.containsValue(neighbour.node))) {
					newNormal.add(externalNormals.get(neighbour.node));
					numPoints += 1;
				}
			}
			newNormal.scale(1/numPoints);
			externalNormalsSmoothed.put(node, newNormal);
		}
		externalNormals.clear();
		externalNormals.putAll(externalNormalsSmoothed);

		//Create shell projection nodes and their symmetry mates
		for (int i = 0; i < sourceMeshComp.numVertices(); i++) {
			FemNode3d node = sourceMeshComp.getNodeForVertex(sourceMeshComp.getVertex(i));
			if (fsm.negativeNodes.contains(node)) {
				Vector3d normal = externalNormals.get(node);
				Point3d newNodePosition = new Point3d(node.getPosition());
				Point3d scaledNormal = new Point3d(normal);
				scaledNormal.scale(maxElementWidth);
				newNodePosition.add(scaledNormal);
				FemNode3d newNode = new FemNode3d(newNodePosition);			
				fem.addNode(newNode);

				FemNode3d newNodeSymmetryMate = new FemNode3d(AuxTools.flipValue(newNodePosition, FlipAxis.X));	
				fem.addNode(newNodeSymmetryMate);

				//Store properties of this node
				externalNodeNeighbours.put(node, neighbourMap.get(node.getNumber()));

				externalProjNodes.put(node, newNode);
				externalNodesOfProjs.put(newNode, node);

				//GeometryTools.createSpring(mech, node.getPosition(), newNodePosition, 1e-3, 5e-4, 1e-4, Color.white);
				RenderProps.setVisible(newNode, true);
				RenderProps.setPointRadius(newNode, 1e-3);

				//Store properties of this node's symmetry mate
				FemNode3d nodeSymmetryMate = fsm.nodeSymmetryForward.get(node);
				externalNodeNeighbours.put(nodeSymmetryMate, neighbourMap.get(nodeSymmetryMate.getNumber()));
				externalProjNodes.put(nodeSymmetryMate, newNodeSymmetryMate);
				externalNodesOfProjs.put(newNodeSymmetryMate, nodeSymmetryMate);

				//externalNormals.put(nodeSymmetryMate, new Vector3d(AuxTools.flipValue(new Point3d(normal), FlipAxis.X)));
				//GeometryTools.createSpring(mech, nodeSymmetryMate.getPosition(), externalProjNodes.get(nodeSymmetryMate).getPosition(), 1e-3, 5e-4, 1e-4, Color.white);

				fsm.negativeNodes.add(newNode);
				fsm.positiveNodes.add(newNodeSymmetryMate);

				fsm.nodeSymmetryForward.put(newNode, newNodeSymmetryMate);
			} 
			else if (fsm.centerNodes.contains(node)){
				Vector3d normal = externalNormals.get(node);
				Point3d newNodePosition = new Point3d(node.getPosition());
				Point3d scaledNormal = new Point3d(normal);
				scaledNormal.scale(maxElementWidth);
				newNodePosition.add(scaledNormal);
				FemNode3d newNode = new FemNode3d(newNodePosition);			
				fem.addNode(newNode);

				//Store properties of this node
				externalNodeNeighbours.put(node, neighbourMap.get(node.getNumber()));

				externalProjNodes.put(node, newNode);
				externalNodesOfProjs.put(newNode, node);

				//GeometryTools.createSpring(mech, node.getPosition(), newNodePosition, 1e-3, 5e-4, 1e-4, Color.white);
				RenderProps.setVisible(newNode, true);
				RenderProps.setPointRadius(newNode, 1e-3);

				fsm.centerNodes.add(newNode);
			}
		}

		fsm.colorFemBySymmetryMapping();

		//Using projection nodes, build face array
		ArrayList<ArrayList<FemNode3d>> externalFaceNodes = new ArrayList<ArrayList<FemNode3d>>();
		for (FemElement3d elem : fem.getElements()) {
			int[] fis = elem.getFaceIndices();
			FemNode3d[] nodes = elem.getNodes();

			//Iterate through each face, check if nodes are external
			for (int f = 0; f < 6; f++) {
				int i = f*5;
				if (externalNodes.contains(nodes[fis[i+1]]) && externalNodes.contains(nodes[fis[i+2]]) && externalNodes.contains(nodes[fis[i+3]]) && externalNodes.contains(nodes[fis[i+4]])) {
					ArrayList<FemNode3d> faceNodes = new ArrayList<FemNode3d>();
					faceNodes.add(nodes[fis[i+1]]);
					faceNodes.add(nodes[fis[i+2]]);
					faceNodes.add(nodes[fis[i+3]]);
					faceNodes.add(nodes[fis[i+4]]);
					externalFaceNodes.add(faceNodes);
				}		
			}
		}

		//Check if any face node sets are the same; if so, remove them since the faces are adjoined
		ArrayList<ArrayList<FemNode3d>> removalFaceNodes = new ArrayList<ArrayList<FemNode3d>>();
		for (ArrayList<FemNode3d> nodes : externalFaceNodes) {
			for (ArrayList<FemNode3d> checkNodes : externalFaceNodes) {
				if (nodes.containsAll(checkNodes) && nodes != checkNodes) {
					removalFaceNodes.add(nodes);
					removalFaceNodes.add(checkNodes);
				}	
			}
		}

		//Remove the adjoined faces
		for (ArrayList<FemNode3d> nodes : removalFaceNodes) {
			if (externalFaceNodes.contains(nodes)) {
				externalFaceNodes.remove(nodes);
			}
		}

		int count = 0;
		for (ArrayList<FemNode3d> fNs : externalFaceNodes) {
			fem.addElement(new HexElement(fNs.get(3), fNs.get(2), fNs.get(1), fNs.get(0), externalProjNodes.get(fNs.get(3)), externalProjNodes.get(fNs.get(2)), externalProjNodes.get(fNs.get(1)), externalProjNodes.get(fNs.get(0))));
			count += 1;
			if (count > 6) {
				//break;
			}
		}

		//Find new node locations on the surface mesh
		for (FemNode3d node : externalProjNodes.keySet()) {
			if (fsm.negativeNodes.contains(externalProjNodes.get(node))) {
				Vector3d normal = externalNormals.get(node);
				Point3d intersection = BVFeatureQuery.nearestPointAlongRay(surfaceTargetMesh, node.getPosition(), normal);
				externalProjNodes.get(node).setPosition(intersection);

				//Pass value to symmetry mate
				fsm.nodeSymmetryForward.get(externalProjNodes.get(node)).setPosition(AuxTools.flipValue(externalProjNodes.get(node).getPosition(), FlipAxis.X));
			} else if (fsm.centerNodes.contains(externalProjNodes.get(node))) {
				Vector3d normal = externalNormals.get(node);
				Point3d intersection = BVFeatureQuery.nearestPointAlongRay(surfaceTargetMesh, node.getPosition(), normal);
				externalProjNodes.get(node).setPosition(intersection);
				//fsm.restrictPositionToSymmetryPlane(externalProjNodes.get(node));
				externalProjNodes.get(node).setDynamic(false);
			}
		}

		//Locate internal nodes
		ArrayList<FemNode3d> internalNodes = new ArrayList<FemNode3d>();
		for (FemNode3d node : fem.getNodes()) {
			if (!externalNodesOfProjs.containsKey(node)) {
				internalNodes.add(node);
			}
		}


		//==========================================================================================================
		//Iteratively smooth the newly created FEM:
		//==========================================================================================================
		neighbourMap = Neighbour.getNeighbourMap(fem, maxElementWidth);
		boolean invertedElements = true;
		int iterations = 0;
		while ((invertedElements) && (iterations <= 4)) {

			//Smooth out the positions of the surface nodes
			neighbourMap = Neighbour.getNeighbourMap(fem, maxElementWidth);
			HashMap<FemNode3d, Point3d> externalProjPos = new HashMap<FemNode3d, Point3d>();
			for (FemNode3d node : externalNodes) {
				FemNode3d extNode = externalProjNodes.get(node);
				Point3d newPos = new Point3d(extNode.getPosition());
				double numPoints = 1;
				for (Neighbour neighbour : neighbourMap.get(extNode.getNumber())) {
					if ((neighbour.type != NeighbourType.VOLUME_DIAGONAL) && (externalProjNodes.containsValue(neighbour.node))) {
						newPos.add(neighbour.node.getPosition());
						numPoints += 1;
					}
				}
				newPos.scale(1/numPoints);
				externalProjPos.put(extNode, newPos);
			}

			externalNormals.clear();
			fem.invalidateSurfaceMesh();
			PolygonalMesh mesh = fem.getSurfaceMesh();
			for (int i = 0; i < mesh.numVertices(); i++) {
				Vector3d normal = new Vector3d();
				FemNode3d node = fem.getSurfaceNode(mesh.getVertex(i));
				mesh.getVertex(i).computeAngleWeightedNormal(normal);
				externalNormals.put(node, normal);	
			}

			//Smooth normals
			for (FemNode3d node : externalNormals.keySet()) {
				Point3d newNormal = new Point3d(externalNormals.get(node));
				double numPoints = 1;
				for (Neighbour neighbour : neighbourMap.get(node.getNumber())) {
					if ((neighbour.type != NeighbourType.VOLUME_DIAGONAL) && (externalNormals.containsKey(neighbour.node))) {
						newNormal.add(externalNormals.get(neighbour.node));
						numPoints += 1;
					}
				}
				newNormal.scale(1/numPoints);
				externalNormalsSmoothed.put(node, newNormal);
			}
			externalNormals.clear();
			externalNormals.putAll(externalNormalsSmoothed);

			for (FemNode3d node : externalProjPos.keySet()) {	
				Vector3d normal = externalNormals.get(node);
				Point3d pos = externalProjPos.get(node);
				Point3d newPos = BVFeatureQuery.nearestPointAlongRay(surfaceTargetMesh, pos, normal);

				if (newPos != null) {
					node.setPosition(newPos);
				}
				else {
					newPos = node.getPosition();
				}

				normal.negate();
				normal.scale(maxElementWidth*0.8);
				Point3d newExternalPos = new Point3d(newPos);
				newExternalPos.add(normal);
				FemNode3d externalNode = externalNodesOfProjs.get(node);
				Point3d extPos = externalNode.getPosition();

				newExternalPos = AuxTools.getAveragePosition(extPos, newExternalPos);
				externalNode.setPosition(newExternalPos);
			}


			//Check for inverted elements
			fem.resetRestPosition();
			boolean testInversionFlag = false;
			System.out.println("Checking for inverted elements.");
			for (FemElement3d elem : fem.getElements()) {
				if (elem.isInvertedAtRest()) {
					System.out.println("Warning: Element " + elem.getNumber() + " is inverted at rest.");
					testInversionFlag = true;
					RenderProps.setFaceColor(elem, Color.pink);
				}
				else {
					RenderProps.setFaceColor(elem, Color.gray);
				}
			}

			if (!testInversionFlag) {
				invertedElements = false;
			}
			iterations += 1;
		}

		//==========================================================================================================
		//Puff out and smooth the external projection layer of nodes:
		//==========================================================================================================
		//Smooth out the positions of the surface nodes
		HashMap<FemNode3d, Point3d> externalProjPos = new HashMap<FemNode3d, Point3d>();
		neighbourMap = Neighbour.getNeighbourMap(fem, maxElementWidth);
		for (FemNode3d node : externalNodes) {
			FemNode3d extNode = externalProjNodes.get(node);
			Point3d newPos = new Point3d(extNode.getPosition());
			double numPoints = 1;
			for (Neighbour neighbour : neighbourMap.get(extNode.getNumber())) {
				if ((neighbour.type != NeighbourType.VOLUME_DIAGONAL) && (externalProjNodes.containsValue(neighbour.node))) {
					newPos.add(neighbour.node.getPosition());
					numPoints += 1;
				}
			}
			newPos.scale(1/numPoints);
			externalProjPos.put(extNode, newPos);
		}

		externalNormals.clear();
		fem.invalidateSurfaceMesh();
		PolygonalMesh mesh = fem.getSurfaceMesh();
		for (int i = 0; i < mesh.numVertices(); i++) {
			FemNode3d node = fem.getSurfaceNode(mesh.getVertex(i));

			if (!fsm.centerNodes.contains(node)) {
				Vector3d normal = new Vector3d();
				mesh.getVertex(i).computeAngleWeightedNormal(normal);
				normal.scale(maxElementWidth*0.25);
				externalProjPos.get(node).add(normal);

				node.setPosition(externalProjPos.get(node));
			} else {
				Point3d proj = new Point3d();
				BVFeatureQuery.getNearestFaceToPoint(proj, null, surfaceTargetMesh, node.getPosition());
				externalProjPos.get(node).set(proj);
			}
		}

		for (FemNode3d node : externalProjPos.keySet()) {	
			Point3d pos = externalProjPos.get(node);
			node.setPosition(pos);
		}


		//Smooth out internal nodes
		double externalWeight = 1.1;
		double internalWeight = 1.0;
		for (int j = 0; j <= 100; j++) {

			HashMap<FemNode3d, Point3d> internalNodeNewPos = new HashMap<FemNode3d, Point3d>();
			for (FemNode3d node : internalNodes) {
				Point3d newPos = node.getPosition();
				newPos.set(node.getPosition());
				double weights = 1.0;
				for (Neighbour neighbour : neighbourMap.get(node.getNumber())) {
					if (neighbour.type == NeighbourType.ORTHOGONAL) {
						if (externalNodes.contains(neighbour.node)) {
							newPos.scaledAdd(externalWeight, neighbour.node.getPosition());
							weights += externalWeight;
						} else {
							newPos.scaledAdd(internalWeight, neighbour.node.getPosition());
							weights += internalWeight;
						}
					}

				}
				newPos.scale(1/weights);
				internalNodeNewPos.put(node, newPos);
			}

			for (FemNode3d node : internalNodeNewPos.keySet()) {
				node.setPosition(internalNodeNewPos.get(node));
			}

			internalNodeNewPos.clear();
		}			

		//Smooth external nodes again
		//Smooth out the positions of the surface nodes
		externalProjPos.clear();
		neighbourMap = Neighbour.getNeighbourMap(fem, maxElementWidth);
		for (FemNode3d node : externalNodes) {
			FemNode3d extNode = externalProjNodes.get(node);
			Point3d newPos = new Point3d(extNode.getPosition());
			double numPoints = 1;
			for (Neighbour neighbour : neighbourMap.get(extNode.getNumber())) {
				if ((neighbour.type != NeighbourType.VOLUME_DIAGONAL) && (externalProjNodes.containsValue(neighbour.node))) {
					newPos.add(neighbour.node.getPosition());
					numPoints += 1;
				}
			}
			newPos.scale(1/numPoints);
			externalProjPos.put(extNode, newPos);
		}

		externalNormals.clear();
		fem.invalidateSurfaceMesh();
		mesh = fem.getSurfaceMesh();
		for (int i = 0; i < mesh.numVertices(); i++) {
			FemNode3d node = fem.getSurfaceNode(mesh.getVertex(i));

			Vector3d normal = new Vector3d();
			mesh.getVertex(i).computeAngleWeightedNormal(normal);
			normal.scale(maxElementWidth*0.25);
			externalProjPos.get(node).add(normal);

			node.setPosition(externalProjPos.get(node));

		}

		for (FemNode3d node : externalProjPos.keySet()) {	
			Point3d pos = externalProjPos.get(node);
			node.setPosition(pos);
		}



		fem.resetRestPosition();
		fem.setElementWidgetSize(1.0);
		fem.invalidateStressAndStiffness();
		System.out.println("Checking for inverted elements.");
		int invertedCount = 0;
		for (FemElement3d elem : fem.getElements()) {
			if (elem.isInvertedAtRest()) {
				System.out.println("Warning: Element " + elem.getNumber() + " is inverted at rest.");
				RenderProps.setFaceColor(elem, Color.pink);
				invertedCount++;
			}
			else {
				RenderProps.setFaceColor(elem, Color.gray);
			}
		}
		System.out.println("Number of inverted elements: " + invertedCount);

		System.out.println("Finished building FEM shell.");

		return fsm;
	}



	public static FemMuscleModel createVoxelFem(FemMuscleModel fem, PolygonalMesh mesh, int minRes, double maxElemWidth, boolean elementCenteredFlag) {
		if (fem == null) {
			fem = new FemMuscleModel();
		}

		OBB obb = new OBB(mesh);
		Vector3d center = new Vector3d();
		obb.getCenter(center);
		Point3d min = new Point3d();
		Point3d max = new Point3d();
		Point3d mid = new Point3d();

		mesh.getWorldBounds(min, max);
		mid.add(max, min);
		mid.scale(0.5);

		Vector3d hw = new Vector3d(obb.getHalfWidths());
		double dz = 2 * hw.z / minRes;
		if (dz > maxElemWidth) {
			dz = maxElemWidth;
		}

		int[] res = new int[3];
		res[0] = (int)(Math.round(2*hw.x/dz));
		res[1] = (int)(Math.round(2*hw.y/dz));
		res[2] = (int)(Math.round(2*hw.z/dz));

		if (elementCenteredFlag) {
			res[0] = ((res[0] % 2) == 1 ? res[0] : res[0] + 1);
			res[1] = ((res[1] % 2) == 1 ? res[1] : res[1] + 1);
			res[2] = ((res[2] % 2) == 1 ? res[2] : res[2] + 1);
		} else {
			res[0] = ((res[0] % 2) == 1 ? res[0] + 1 : res[0]);
			res[1] = ((res[1] % 2) == 1 ? res[1] + 1 : res[1]);
			res[2] = ((res[2] % 2) == 1 ? res[2] + 1 : res[2]);
		}

		FemFactory.createHexGrid(fem, 2*hw.x, 2*hw.y, 2*hw.z, res[0], res[1], res[2]);
		fem.transformGeometry(obb.getTransform());

		double dx, dy;
		dx = 2 * hw.x / res[0];
		dy = 2 * hw.y / res[1];
		dz = 2 * hw.z / res[2];
		double r = 1.05*Math.sqrt(dx * dx + dy * dy + dz * dz);

		//Check for nodes that are outside and farther than r
		BVFeatureQuery query = new BVFeatureQuery();

		HashSet<FemNode3d> deleteThese = new HashSet<FemNode3d>();
		for (FemNode3d node : fem.getNodes()) {
			//boolean inside = query.isInsideOrientedMesh(mesh, node.getPosition(), r);
			InsideQuery inside = query.isInsideMesh(mesh, node.getPosition(), r);
			if (inside != InsideQuery.INSIDE) {
				deleteThese.add(node);
			}
		}

		// remove elements/nodes
		for (FemNode3d node : deleteThese) {
			// remove element dependencies
			ArrayList<FemElement3d> elems = new ArrayList<>(node.getElementDependencies());
			for (FemElement3d elem : elems) {
				fem.removeElement(elem);
			}
			//Remove node
			fem.removeNode(node);
		}


		//Remove un-necessary nodes
		deleteThese.clear();
		for (FemNode3d node : fem.getNodes()) {
			if (node.getElementDependencies().size() < 1) {
				deleteThese.add(node);
			}
		}
		for (FemNode3d node : deleteThese) {
			fem.removeNode(node);
		}

		return fem;
	}


	/** Adds elements based on "pill shaped" proximity region defined by <i>nodeProximity</i> to muscle fibers.
	 * <b>Note:</b> If the muscle bundle already has elements, these are removed prior to locating new elements.
	 * 
	 * @param fem fem model
	 * @param nodeProximity proximity near fibers
	 */
	public static void addElementsToMuscles(FemMuscleModel fem, double nodeProximity) {
		for (MuscleBundle mb : fem.getMuscleBundles()) {
			mb.getElements().removeAll();

			for (Muscle m : mb.getFibres()) {
				for (FemNode3d node : findNodesNearFiber(fem, m, nodeProximity)) {
					System.out.println("Add dependencies of node " + node.getNumber() + " to muscle " + mb.getName());
					for (FemElement3d element : node.getElementDependencies()) {
						boolean addElementFlag = true;
						for (MuscleElementDesc med : mb.getElements()) {
							if (med.getElement().equals(element)) {
								addElementFlag = false;
								break;
							}
						}

						if (addElementFlag) {
							MuscleElementDesc med = new MuscleElementDesc(element, new Vector3d());
							RenderProps.setVisible(med, false);
							mb.addElement(med);
						}
					}
				}

				try {
					mb.computeElementDirections();
				}
				catch (Exception ex) {
					System.err.println("An error occurred calculating element directions for muscle '" + mb.getName() + "'.");
				}
			}
		}

		//Check that all fibers have some element descriptions
		System.out.println("Check for completeness of element assignment:");
		boolean badMusclesFlag = false;
		for (MuscleBundle mb : fem.getMuscleBundles()) {
			for (Muscle m : mb.getFibres()) {
				ArrayList<FemElement3d> elems0 = FemMiscMethods.getElements(m.getFirstPoint());
				ArrayList<FemElement3d> elems1 = FemMiscMethods.getElements(m.getSecondPoint());

				if (elems0.size() == 0 || elems1.size() == 0) {
					badMusclesFlag = true;
					System.err.println("\tFiber " + m.getNumber() + " of muscle '" + mb.getName() + "' has no element description.");
				}
			}
		}

		if (badMusclesFlag) {
			System.err.println("Element assignment did not produce well-formed muscles, consider changing node proximity factor (currently it is " + nodeProximity + ")");
		}
		else {
			System.out.println("Muscles of FEM '" + fem.getName() + "' were successfully associated with elements");
		}
	}




	public static ArrayList<FemNode3d> findNodesNearFiber(FemMuscleModel fem, Muscle fiber, double inclusionRadius) {
		ArrayList<FemNode3d> proximalNodes = new ArrayList<FemNode3d>();
		Line line = new Line();
		Point3d p1 = fiber.getFirstPoint().getPosition();
		Point3d p2 = fiber.getSecondPoint().getPosition();

		line.setPoints(p1, p2);

		if (!line.getDirection().containsNaN()) {
			for (FemNode3d node : fem.getNodes()) {
				Point3d proj = new Point3d();
				line.nearestPoint(proj, node.getPosition());

				double distance = proj.distance(node.getPosition());

				//First check if the distance to the line is within the inclusion "capsule"
				if (distance <= inclusionRadius) {
					//Check if the projection point falls within the radius of either point, or somewhere between the two
					double p1Dist = proj.distance(p1);
					double p2Dist = proj.distance(p2);
					double p1p2Dist = p1.distance(p2);

					if (p1Dist <= inclusionRadius || p2Dist <= inclusionRadius || p1Dist + p2Dist <= p1p2Dist) {
						proximalNodes.add(node); 
					}
				}
			}
		}

		return proximalNodes;
	}

	public static double getMaxElementQuality(FemMuscleModel fem) {
		double maxQuality = 0.0;
		for (FemElement3d elem : fem.getElements()) {
			maxQuality = Math.max(FemTools.calculateElementQuality(elem), maxQuality);
		}

		return maxQuality;
	}

	public static void colorizeElementByQuality(FemElement3d element, double quality, ColorMixer lowQuality, ColorMixer highQuality) {
		RenderProps.setFaceColor(element, ColorTools.createBicolor(quality, lowQuality, highQuality));
	}

	public static void colorizeByQualityThreshold(FemMuscleModel fem, double maxQuality, double thresholdQuality) {
		for (FemElement3d elem : fem.getElements()) {
			double quality = FemTools.calculateElementQuality(elem) / maxQuality;
			if (quality <= thresholdQuality) {
				double qualityRelativeToThreshold = quality/thresholdQuality;
				colorizeElementByQuality(elem, qualityRelativeToThreshold, ColorMixer.RED, ColorMixer.BLUE);
			}
		}
	}


	public static void colorizeByQuality(FemMuscleModel fem, double maxQuality) {
		for (FemElement3d elem : fem.getElements()) {
			double quality = FemTools.calculateElementQuality(elem) / maxQuality;
			elem.getRenderProps().setFaceColor(ColorTools.createBicolor(quality, ColorMixer.RED, ColorMixer.BLUE));
		}
	}

	public static void colorizeByQuality(FemMuscleModel fem) {
		double maxQuality = getMaxElementQuality(fem);
		colorizeByQuality(fem, maxQuality);
	}

	public static ArrayList<FemNode3d> colorNodesNearFiber(FemMuscleModel fem, Muscle fiber, double inclusionRadius, Color muscleNodeColor) {
		ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
		Line line = new Line();
		Point3d p1 = fiber.getFirstPoint().getPosition();
		Point3d p2 = fiber.getSecondPoint().getPosition();

		line.setPoints(p1, p2);

		if (!line.getDirection().containsNaN()) {
			for (FemNode3d node : fem.getNodes()) {
				Point3d proj = new Point3d();
				line.nearestPoint(proj, node.getPosition());

				double distance = proj.distance(node.getPosition());

				//First check if the distance to the line is within the inclusion "capsule"
				if (distance <= inclusionRadius) {
					//Check if the projection point falls within the radius of either point, or somewhere between the two
					double p1Dist = proj.distance(p1);
					double p2Dist = proj.distance(p2);
					double p1p2Dist = p1.distance(p2);

					if (p1Dist <= inclusionRadius || p2Dist <= inclusionRadius || p1Dist + p2Dist <= p1p2Dist) {
						node.getRenderProps().setPointColor(muscleNodeColor);
						nodes.add(node);
					}
				}
			}
		}
		return nodes;
	}

	public static double getAverageNeighbourDistance(FemMuscleModel fem) {
		HashMap<Integer, ArrayList<Neighbour>> nodeNeighbourMap = NeighbourData.getNeighbourMap(fem, 1.0);
		double count = 0.0;
		double averageDistance = 0.0;

		for (FemNode3d node : fem.getNodes()) {
			Point3d pos = node.getPosition();
			ArrayList<Neighbour> neighbours = nodeNeighbourMap.get(node.getNumber());

			if (neighbours != null) {
				for (Neighbour neighbour : neighbours) {
					Point3d diff = new Point3d(neighbour.node.getPosition());
					diff.sub(pos);
					averageDistance += diff.norm();
					count++;
				}
			}
			else {
				System.err.println("Neighbours for node " + node.getNumber() + " could not be found.");
			}
		}

		if (count > 0.0) {
			averageDistance /= count;
		}

		return averageDistance;
	}

	public static void colorNeighbours(FemNode3d node) {	
		ArrayList<Neighbour> neighbours = Neighbour.findNeighbours(node, 1.0);

		for (Neighbour neighbour : neighbours) {
			if (neighbour.type == Neighbour.NeighbourType.ORTHOGONAL) {
				RenderProps.setPointColor(neighbour.node, Color.green);
			}
			else if (neighbour.type == Neighbour.NeighbourType.FACE_DIAGONAL) {
				RenderProps.setPointColor(neighbour.node, Color.cyan);
			}
			else {
				RenderProps.setPointColor(neighbour.node, Color.blue);
			}

		}
	}

	public static class FemSymmetryMap {
		public static enum SymmetryAxis {
			X(new Vector3d(1.0, 0.0, 0.0)),
			Y(new Vector3d(0.0, 1.0, 0.0)),
			Z(new Vector3d(0.0, 0.0, 1.0));
			Vector3d normal;
			SymmetryAxis(Vector3d normal) {
				this.normal = normal;
			}
			public Plane getSymmetryPlane(double symmetryValue) {
				return new Plane(normal, symmetryValue);
			}
		}

		public Color negativeSideColor = Color.blue;
		public Color positiveSideColor = Color.red;
		public Color neutralColor = Color.yellow;
		public Color noMateColor = Color.green;
		public FemMuscleModel fem;
		public SymmetryAxis symmetryAxis;
		public Plane symmetryPlane;
		public double symmetryThreshold;
		public double symmetryValue;
		public HashMap<FemNode3d, FemNode3d> nodeSymmetryForward = new HashMap<FemNode3d, FemNode3d>();
		public HashMap<FemNode3d, FemNode3d> nodeSymmetryReverse = new HashMap<FemNode3d, FemNode3d>();
		public ArrayList<FemNode3d> negativeNodes = new ArrayList<FemNode3d>();
		public ArrayList<FemNode3d> positiveNodes = new ArrayList<FemNode3d>();
		public ArrayList<FemNode3d> unclassifiedNodes = new ArrayList<FemNode3d>();

		public HashMap<FemElement3d, FemElement3d> elementSymmetryForward = new HashMap<FemElement3d, FemElement3d>();
		public HashMap<FemElement3d, FemElement3d> elementSymmetryReverse = new HashMap<FemElement3d, FemElement3d>();
		public ArrayList<FemElement3d> negativeElements = new ArrayList<FemElement3d>();
		public ArrayList<FemElement3d> positiveElements = new ArrayList<FemElement3d>();
		public ArrayList<FemElement3d> centerElements = new ArrayList<FemElement3d>();
		public ArrayList<FemNode3d> centerNodes = new ArrayList<FemNode3d>();
		public ArrayList<FemElement3d> unclassifiedElements = new ArrayList<FemElement3d>();

		public HashMap<FemNode3d, Color> nodeColorMap = new HashMap<FemNode3d, Color>();
		public HashMap<FemElement3d, Color> elementColorMap = new HashMap<FemElement3d, Color>();

		public FemSymmetryMap(FemMuscleModel fem, SymmetryAxis symmetryAxis, double symmetryThreshold) {
			this.fem = fem;
			this.symmetryAxis = symmetryAxis;
			this.symmetryThreshold = symmetryThreshold;
			createSymmetryMaps(this);
			this.colorFemBySymmetryMapping();
		}

		public void colorFemBySymmetryMapping() {
			for (FemNode3d node : negativeNodes) {
				RenderProps.setPointColor(node, negativeSideColor);
			}

			for (FemNode3d node : positiveNodes) {
				RenderProps.setPointColor(node, positiveSideColor);
			}

			for (FemNode3d node : centerNodes) {
				RenderProps.setPointColor(node, neutralColor);
			}

			for (FemElement3d element : fem.getElements()) {
				Color elementColor = elementColorMap.get(element);

				if (elementColor != null) {
					RenderProps.setFaceColor(element, elementColor);
				}
				else {
					System.err.println("Element " + element.getNumber() + " was not assigned a symmetry mapping color.");
				}

			}
			/*
                     for (FemNode3d node : negativeNodes) {
                            node.getRenderProps().setPointColor(negativeSideColor);
                     }

                     for (FemNode3d node : positiveNodes) {
                            node.getRenderProps().setPointColor(positiveSideColor);
                     }

                     for (FemNode3d node : unclassifiedNodes) {
                            node.getRenderProps().setPointColor(noMateColor);
                     }

                     for (FemElement3d element : negativeElements) {
                            element.getRenderProps().setFaceColor(negativeSideColor);
                     }

                     for (FemElement3d element : positiveElements) {
                            element.getRenderProps().setFaceColor(positiveSideColor);
                     }

                     for (FemElement3d element : centerElements) {
                            element.getRenderProps().setFaceColor(neutralColor);
                     }

                     for (FemElement3d element : unclassifiedElements) {
                            element.getRenderProps().setFaceColor(noMateColor);
                     }
			 */
		}

		public Point3d reflectPoint(Point3d pos) {
			switch (symmetryAxis) {
			case X:
				return new Point3d(-pos.x, pos.y, pos.z);
			case Y:
				return new Point3d(pos.x, -pos.y, pos.z);
			case Z:
				return new Point3d(pos.x, pos.y, -pos.z);
			default:
				System.err.println("An error occurred determining the symmetry axis of FEM '" + fem.getName() + "'");
				return null;
			}
		}

		public double getSymmetryValue(Point3d pos) {
			switch (symmetryAxis) {
			case X:
				return pos.x;
			case Y:
				return pos.y;
			case Z:
				return pos.z;
			default:
				System.err.println("An error occurred determining the symmetry axis of FEM '" + fem.getName() + "'");
				return 0.0;
			}
		}

		/** Removes unclassified elements and nodes from the FEM
		 * **/
		public void cleanFemWithSymmetryMap() {
			for (FemElement3d elem : unclassifiedElements) {
				fem.removeElement(elem);	
			}

			for (FemNode3d node : unclassifiedNodes) {
				fem.removeNode(node);
			}

		}

		/** Builds a mapping of symmetrical node pairs across the specified axis using symmetryThreshold to judge pairings. If a node cannot be assigned a pairing, then its symmetry mate will be null and it will be assigned the color green in the symmetry color map. 
		 *  Nodes falling perfectly on the symmetry axis will not be assigned symmetry matings. Elements that stradle the symmetry axis will be self-mated.
		 * **/
		private void createSymmetryMaps(FemSymmetryMap fsm) {
			System.out.println("\tCreating symmetry mappings");

			//Obtain data about the mesh
			Point3d min = new Point3d();
			Point3d max = new Point3d();
			Point3d mid = new Point3d();

			fem.getSurfaceMesh().updateBounds(min, max);
			mid.add(max, min);
			mid.scale(0.5);
			symmetryValue = getSymmetryValue(mid);
			symmetryPlane = symmetryAxis.getSymmetryPlane(symmetryValue);

			//Iterate through all nodes and, for each, locate the node laying across the axis of symmetry that is the most plausible symmetry mate
			negativeNodes.clear();
			positiveNodes.clear();
			for (FemNode3d node : fem.getNodes()) {
				Point3d pos = node.getPosition();


				double nodeSymValue = getSymmetryValue(pos);

				//Only process nodes that fall on the negative side of the symmetry axis
				if (nodeSymValue < symmetryValue - symmetryThreshold) {
					negativeNodes.add(node);

					Point3d reflection = new Point3d();
					symmetryPlane.reflect(reflection, pos);
					boolean symmetryNodeFoundFlag = false;
					HashMap<Object, Double> numberDistMap = new HashMap<Object, Double>();

					for (FemNode3d candidateNode : fem.getNodes()) {
						Point3d candPos = candidateNode.getPosition();
						double candSymValue = getSymmetryValue(candPos);

						//First rule out nodes that fall on the same side of the axis of symmetry
						if (candSymValue > symmetryValue + symmetryThreshold) {
							//Now calculate the distance between the two nodes and store the candidate index along with this distance in the map
							Point3d diff = new Point3d(candPos);
							diff.sub(reflection);
							double dist = diff.norm();
							numberDistMap.put(candidateNode.getNumber(), dist);

							if (dist < symmetryThreshold) {
								if (!positiveNodes.contains(candidateNode)) {
									symmetryNodeFoundFlag = true;
									positiveNodes.add(candidateNode);
									nodeSymmetryForward.put(node, candidateNode);
									nodeSymmetryReverse.put(candidateNode, node);
									nodeColorMap.put(node, negativeSideColor);
									nodeColorMap.put(candidateNode, positiveSideColor);
									break;
								}
								else {
									System.err.println("An error occurred forming symmetry map: Node " + candidateNode.getNumber() + " has multiple symmetry assignments.");
								}
							}
						}
						else if (candSymValue == symmetryValue){
							System.out.println("Node " + candidateNode.getNumber() + " is on the symmetry axis.");
						}
					}

					//Check if a node could not be located, in which case use a more thorough approach by finding the closest possible node
					if (!symmetryNodeFoundFlag) {
						for (FemNode3d candidateNode : fem.getNodes()) {
							Point3d candPos = candidateNode.getPosition();
							if (!numberDistMap.containsKey(candidateNode.getNumber()) && candidateNode != node && getSymmetryValue(candPos) > symmetryValue) {
								Point3d diff = new Point3d(candPos);
								diff.sub(reflection);
								numberDistMap.put(candidateNode.getNumber(), diff.norm());
							}
						}

						//Sort the distance map and choose the node that is closest
						if (numberDistMap.size() > 0) {
							LinkedHashMap<Object, Double> sortedNumberDistMap = AuxTools.sortByComparator(numberDistMap);
							List<Map.Entry<Object, Double>> list = new LinkedList<Map.Entry<Object, Double>>(sortedNumberDistMap.entrySet());

							FemNode3d symmetryMate = fem.getByNumber((Integer) list.get(0).getKey());
							if (symmetryMate != null) {
								positiveNodes.add(symmetryMate);
								nodeSymmetryForward.put(node, symmetryMate);
								nodeSymmetryReverse.put(symmetryMate, node);
								nodeColorMap.put(node, negativeSideColor);
								nodeColorMap.put(symmetryMate, positiveSideColor);
							}
							else {
								System.err.println("Node " + list.get(0).getKey() + " was contained in node-node distance map but could not be found in the FEM model");
							}
						}
						else {
							System.err.println("Node-node distance map contains no entries! Could not locate any symmetry mates for node " + node.getNumber());
						}
					}
				} else if ((nodeSymValue > symmetryValue - symmetryThreshold) && (nodeSymValue < symmetryValue + symmetryThreshold)) {
					centerNodes.add(node);
					nodeColorMap.put(node, neutralColor);
				}
			}

			//Find unassigned nodes and assign them to the appropriate array and notify that there was an incomplete assignment
			for (FemNode3d node : fem.getNodes()) {
				if (!negativeNodes.contains(node) && !positiveNodes.contains(node) && !centerNodes.contains(node)) {
					unclassifiedNodes.add(node);
					System.err.println("Node " + node.getNumber() + " could not be assigned a symmetry mate.");
					nodeColorMap.put(node, noMateColor);
				}
			}


			//Iterate through all elements and determine symmetry matings based on shared symmetry nodes
			for (FemElement3d element : fem.getElements()) {
				int negativeNodeCount = 0;
				int positiveNodeCount = 0;
				int neutralNodeCount = 0;
				for (FemNode3d node : element.getNodes()) {
					if (negativeNodes.contains(node)) {
						negativeNodeCount++;
					} else if (positiveNodes.contains(node)) {
						positiveNodeCount++;
					} else if (centerNodes.contains(node)) {
						neutralNodeCount++;
					}
				}

				//All nodes of this element are negative, assign it to the appropriate array and locate and assign its symmetry mate
				if ((negativeNodeCount == element.getNodes().length) || ((negativeNodeCount == 4) && (neutralNodeCount == 4))) {
					negativeElements.add(element);
					elementColorMap.put(element, negativeSideColor);
					ArrayList<FemNode3d> negativeNodeMates = new ArrayList<FemNode3d>();
					for (FemNode3d node : element.getNodes()) {
						FemNode3d nodeSymMate = nodeSymmetryForward.get(node);
						if (nodeSymMate != null) {
							negativeNodeMates.add(nodeSymMate);
						}
					}

					//Find the element in common
					if (!negativeNodeMates.isEmpty()) {
						ArrayList<LinkedList<FemElement3d>> depElementLists = new ArrayList<LinkedList<FemElement3d>>();

						for (FemNode3d mateNodes : negativeNodeMates) {
							depElementLists.add(mateNodes.getElementDependencies());
						}

						FemElement3d candidateElement = null;
						for (FemElement3d elem : depElementLists.get(0)) {
							candidateElement = elem;
							for (int i = 1; i < depElementLists.size(); i++) {
								if (!depElementLists.get(i).contains(elem)) {
									candidateElement = null;
									break;
								}
							}

							//Check if a candidate has been found to be in all arrays
							if (candidateElement != null) {
								break;
							}
						}

						if (candidateElement != null) {
							positiveElements.add(candidateElement);
							elementColorMap.put(candidateElement, positiveSideColor);
							elementSymmetryForward.put(element, candidateElement);
							elementSymmetryReverse.put(candidateElement, element);
						} else {
							System.err.println("A unique element symmetry pairing could not be found for element " + element.getNumber());
						}
					} else {
						System.out.println("An error occurred: found an element containing negative nodes but these none of theese could be found in the symmetry map.");
					}
				} 
				else if (negativeNodeCount >= 4){
					centerElements.add(element);
					elementColorMap.put(element, neutralColor);
				}
				else if (!centerElements.contains(element) && !positiveElements.contains(element)){
					unclassifiedElements.add(element);
					elementColorMap.put(element, noMateColor);
				}
			}
		}

		public void symmetrizeFem() {
			for (FemNode3d node : negativeNodes) {
				nodeSymmetryForward.get(node).setPosition(reflectPoint(node.getPosition()));
				nodeSymmetryForward.get(node).setVelocity(0.0, 0.0, 0.0);
			}

			for (FemNode3d node : centerNodes) {
				restrictPositionToSymmetryPlane(node);				
			}
		}

		public void restrictPositionToSymmetryPlane(FemNode3d node) {
			Point3d pos = node.getPosition();
			Vector3d vel = node.getVelocity();

			switch (symmetryAxis) {
			case X:
				node.setPosition(symmetryValue, pos.y, pos.z);
				node.setVelocity(0.0, vel.y, vel.z);
				break;
			case Y:
				node.setPosition(pos.x, symmetryValue, pos.z);
				node.setVelocity(vel.x, 0.0, vel.z);
				break;
			case Z:
				node.setPosition(pos.x, pos.y, symmetryValue);
				node.setVelocity(vel.x, vel.y, 0.0);
				break;
			default:
				System.err.println("An error occurred determining the symmetry axis of FEM '" + fem.getName() + "'");
				break;
			}
		}
	}



	public static class ForgedFem {
		public FemMuscleModel fem;
		public FemGrid fg;
		public PolygonalMesh mesh;
		public double scaleFactor, optimalOrthogonalLength, volumeThreshold;
		public HashMap<Integer, ArrayList<Neighbour>> neighbourMap;
		public HashMap<FemNode3d, FemNode3d> symmetryData;
		public static enum FemType {
			TET,
			HEX,
			SYMMETRIC_HEX;
		}

		public ForgedFem(FemMuscleModel fem, FemGrid fg, PolygonalMesh mesh, double scaleFactor, double optimalOrthogonalLength, double volumeThreshold) {
			this.fem = fem;
			this.fg = fg;
			this.mesh = mesh;
			this.scaleFactor = scaleFactor;
			this.optimalOrthogonalLength = optimalOrthogonalLength;
			this.volumeThreshold = volumeThreshold;
			this.neighbourMap = Neighbour.getNeighbourMap(fem, optimalOrthogonalLength);
		}

		public static ForgedFem forgeHexFem(String name, PolygonalMesh mesh, double scaleFactor, double volumeThreshold) { return ForgedFem.createSymmetricHexFEM(name, mesh, scaleFactor, volumeThreshold); }
		public static ForgedFem forgeFEM(String name, PolygonalMesh mesh, FemType femType, double scaleFactor, double volumeThreshold) {
			switch (femType) {
			case HEX:
				break;
			case SYMMETRIC_HEX:
				return ForgedFem.createSymmetricHexFEM(name, mesh, scaleFactor, volumeThreshold);
			case TET:
				break;
			default:
				break;
			}
			return null;
		}



		public static class NeighbourData {
			//Hex element node neighbourhood index arrays
			public static int[][] orthogonalNeighbourIndices = new int[][]{{1, 3, 4}, {0, 2, 5}, {1, 3, 6}, {0, 2, 7}, {0, 5, 7}, {1, 4, 6}, {5, 7, 2}, {4, 6, 3}};
			public static int[][] faceDiagonalNeighbourIndices = new int[][]{{2, 7, 5}, {3, 4, 6}, {0, 5, 7}, {1, 4, 6}, {1, 3, 6}, {0, 2, 7}, {1, 3, 4}, {0, 2, 5}};
			public static int[] volumeDiagonalNeighbourIndices = new int[]{6, 7, 4, 5, 2, 3, 0, 1};

			public FemNode3d node;
			public NeighbourType type;
			public double optimalDistance;

			public static enum NeighbourType {
				ORTHOGONAL,
				FACE_DIAGONAL,
				VOLUME_DIAGONAL;
			}

			public NeighbourData(FemNode3d neighbour, NeighbourType neighbourType, double optimalDistance) {
				this.node = neighbour;
				this.type = neighbourType;
				this.optimalDistance = optimalDistance;
			}

			public static boolean testForContainment(ArrayList<Neighbour> neighbours, FemNode3d testNeighbour) {
				for (Neighbour neighbour : neighbours) {
					if (neighbour.node.equals(testNeighbour)) {
						return true;
					}
				}
				return false;
			}



			public static HashMap<Integer, ArrayList<Neighbour>> getNeighbourMap(FemModel3d fem, double optimalOrthogonalLength) {
				HashMap<Integer, ArrayList<Neighbour>> neighbourMap = new HashMap<Integer, ArrayList<Neighbour>>();
				for (FemNode3d node : fem.getNodes()) {
					neighbourMap.put(node.getNumber(), findNeighbours(node, optimalOrthogonalLength));
				}
				return neighbourMap;
			}


			/** Produces an array of FEM nodes neighbour to the input node. **/
			public static ArrayList<Neighbour> findNeighbours(FemNode3d node, double optimalOrthogonalDistance) {
				ArrayList<Neighbour> neighbourNodes = new ArrayList<Neighbour>();
				int numElements = node.getElementDependencies().size();

				if (numElements > 0) {
					for (FemElement3d elem : node.getElementDependencies()) {
						int count = 0;
						FemNode3d[] elemNodes =  elem.getNodes();
						for (FemNode3d elemNode : elemNodes) {
							if (elemNode.equals(node)) {
								break;
							}
							count++;
						}

						for (int i : Neighbour.orthogonalNeighbourIndices[count]) {
							FemNode3d testNeighbour = elemNodes[i];
							if (!Neighbour.testForContainment(neighbourNodes, testNeighbour)) {
								neighbourNodes.add(new Neighbour(testNeighbour, Neighbour.NeighbourType.ORTHOGONAL, optimalOrthogonalDistance));         
							}
						}

						double faceDiagonalOrthogonalDistance = Math.sqrt(2*optimalOrthogonalDistance*optimalOrthogonalDistance);
						for (int i : Neighbour.faceDiagonalNeighbourIndices[count]) {
							FemNode3d testNeighbour = elemNodes[i];
							if (!Neighbour.testForContainment(neighbourNodes, testNeighbour)) {
								neighbourNodes.add(new Neighbour(testNeighbour, Neighbour.NeighbourType.FACE_DIAGONAL, faceDiagonalOrthogonalDistance));        
							}
						}

						double optimalVolumeDiagonalDistance = Math.sqrt(3*optimalOrthogonalDistance*optimalOrthogonalDistance);
						FemNode3d testNeighbour = elemNodes[Neighbour.volumeDiagonalNeighbourIndices[count]];
						if (!Neighbour.testForContainment(neighbourNodes, testNeighbour)) {
							neighbourNodes.add(new Neighbour(testNeighbour, Neighbour.NeighbourType.VOLUME_DIAGONAL, optimalVolumeDiagonalDistance));
						}

					}
				}


				return neighbourNodes;
			}

		}

		public static class HexData {
			HexElement hex;
			FemNode3d cornerNode;
			FemNode3d[] edgeNodes = new FemNode3d[3];
			Point3d[] edges = new Point3d[]{new Point3d(), new Point3d(), new Point3d()};
			Plane[] planes = new Plane[3];
			double[] edgePairMagnitudes = new double[3], angles = new double[3], lengths = new double[3], angleQs = new double[3], edgeQs = new double[3];
			boolean[] edgeDegeneracyFlag = new boolean[]{false, false, false};
			double optimalLength, cornerQuality = 0.0, cornerAngleQuality = 1.0, cornerEdgeQuality = 1.0;
			double optimalAngle = Math.PI*0.5; //90 degrees is optimal for all corners of a hexahedral element (perfect cube)

			public HexData(HexElement hex, FemNode3d cornerNode, FemNode3d firstEdgeNode, FemNode3d secondEdgeNode, FemNode3d thirdEdgeNode, double optimalLength) {
				this.hex = hex;
				this.cornerNode = cornerNode;

				this.edgeNodes[0] = firstEdgeNode;
				this.edgeNodes[1] = secondEdgeNode;
				this.edgeNodes[2] = thirdEdgeNode;
				this.optimalLength = optimalLength;

				calculateQuality();
			}

			public static HexElement createHex(FemNode3d[] nodes, Integer ... indices) {
				FemNode3d[] hexNodes = new FemNode3d[8];
				int n = 0;
				for (Integer i : indices) {
					hexNodes[n++] = nodes[i];
				}
				return new HexElement(hexNodes);
			}


			public void calculateQuality() {
				for (int i = 0; i < 3; i++) {
					edges[i].sub(cornerNode.getPosition(), edgeNodes[i].getPosition());
					lengths[i] = edges[i].norm();
					edgeQs[i] = 1.0 - (Math.abs(lengths[i] - optimalLength)/optimalLength);
				}

				int[][] idx = new int[][]{{1, 0, 2}, {2, 0, 1}, {2, 1, 0}};    //Indices arranged to create counterclockwise lists of nodes that define outward facing plane normals
				for (int i = 0; i < 3; i++) {
					try {
						planes[i] = new Plane(cornerNode.getPosition(), edgeNodes[idx[i][0]].getPosition(), edgeNodes[idx[i][1]].getPosition());

						double mag = (planes[i].getNormal().norm()*lengths[idx[i][2]]);
						if (mag != 0.0) {
							angles[i] = Math.abs(optimalAngle - Math.acos(Math.min(1.0, planes[i].getNormal().dot(edges[idx[i][2]]) / mag)));
							angleQs[i] = 1.0 - (Math.abs(angles[i] - optimalAngle)/optimalAngle);
						}

					} catch (IllegalArgumentException ex) {
						angleQs[i] = 0.0;
					}
				}

				//Determine corner quality (0 if any two edges are colinear, or an edge length is 0)
				for (int i = 0; i < 3; i++) {
					cornerEdgeQuality *= edgeQs[i];
					cornerAngleQuality *= angleQs[i];
				} 


				//An optimal corner scores maximally on both measures
				cornerQuality = (cornerAngleQuality + cornerEdgeQuality) / 2.0;

			}

			public static double calculateHexQuality(HexElement hex, double optimalLength) {
				FemNode3d[] nodes = hex.getNodes();
				double hexQuality = 0.0;
				boolean zeroQualityFlag = false;
				ArrayList<HexData> cornerData = new ArrayList<HexData>();
				int[][] idx = new int[][]{{1, 3, 4}, {0, 2, 5}, {1, 3, 6}, {0, 2, 7}, {5, 7, 0}, {4, 6, 1}, {5, 7, 2}, {4, 6, 3}}; //See HexElement.class for edge indices

				for (int i = 0; i < 8; i++) {
					HexData hcd = new HexData(hex, nodes[i], nodes[idx[i][0]], nodes[idx[i][1]], nodes[idx[i][2]], optimalLength);
					cornerData.add(hcd);
					hexQuality += hcd.cornerQuality;

					if (hcd.cornerQuality == 0.0) {
						zeroQualityFlag = true;
						break;
					}
				}

				return (zeroQualityFlag ? 0.0 : Math.max(hexQuality / 8.0, 0.0));
			}

			public static double reportHexQuality(HexElement hex, double optimalLength) {
				double hexQuality = calculateHexQuality(hex, optimalLength);
				hex.computeVolumes();
				System.out.println("Hex " + hex.getNumber() + ": quality is " + String.format("%.3f", hexQuality*100) + "% ... volume is " + String.format("%.3f", hex.getVolume()) + (hex.isInvertedAtRest() ? " (inverted)" : ""));
				return hexQuality;
			}
		}


		public static double computeMeshResolution(PolygonalMesh mesh, double scaleFactor) {

			//Calculate average face size to determine mesh resolution factor
			double optLength = 0.0;
			for (Face face : mesh.getFaces()) {
				double lengthTotal = 0.0;
				double edgeCount = 0.0;
				HalfEdge he0 = face.firstHalfEdge();
				HalfEdge he = he0;

				do {
					edgeCount += 1.0;
					lengthTotal += he.length();
					he = he.getNext();
				}
				while (he != he0);

				optLength += (lengthTotal/edgeCount);
			}
			optLength /= mesh.numFaces();

			optLength *= scaleFactor;

			return optLength;
		}

		public static ForgedFem createSymmetricHexFEM(String name, PolygonalMesh mesh, double scaleFactor, double volumeThreshold) {
			System.out.println("Creating symmetric hexahedral FEM '" + name + "':");
			FemMuscleModel fem = new FemMuscleModel(name);

			double optLength = computeMeshResolution(mesh, scaleFactor);
			double hOptLength = optLength*0.5;

			Vector3d min = new Vector3d();
			Vector3d max = new Vector3d();
			Vector3d mid = new Vector3d();
			mesh.getLocalBounds(min, max);
			mid.add(min, max);
			mid.scale(0.5);

			//Compute the number of cells required for one octant of the mesh with elements centered at its midlines (giving an uneven number of elements)
			Vector3d overMid = new Vector3d(mid);
			Vector3d underMid = new Vector3d(mid);
			overMid.add(hOptLength, hOptLength, hOptLength);
			underMid.sub(new Vector3d(hOptLength, hOptLength, hOptLength));

			double hNumX = Math.ceil((max.x - overMid.x) / optLength); 
			double hNumY = Math.ceil((max.y - overMid.y) / optLength); 
			double hNumZ = Math.ceil((max.z - overMid.z) / optLength); 

			Vector3d femMin = new Vector3d(mid.x - (hNumX*optLength) - hOptLength, mid.y - (hNumY*optLength) - hOptLength, mid.z - (hNumZ*optLength) - hOptLength); 
			int numX = (int)(2*(hNumX + 1));
			int numY = (int)(2*(hNumY + 1));
			int numZ = (int)(2*(hNumZ + 1));

			FemGrid fg = FemGrid.createHexGrid(numX, numY, numZ, optLength, femMin);
			FemGrid.createSymmetryMap(fg, mid.x, optLength);
			forgeHexFemFromGrid(fem, mesh, fg, volumeThreshold, optLength);
			return new ForgedFem(fem, fg, mesh, scaleFactor, optLength, volumeThreshold);
		}

		public static class NodeData {
			public InsideQuery iq;
			public Face face;
			public Face centroidFace;

			public FemNode3d node;
			public FemNode3d symmetryNode;

			public Point3d position = new Point3d();
			public Point3d projection = new Point3d();
			public Vector3d faceNormal = new Vector3d();
			public Point3d faceCentroid;
			public Point3d projPosNormal;
			public double projPosDistance;

			public NodeData(FemNode3d node, FemNode3d symmetryNode, PolygonalMesh mesh) {
				this.node = node;
				this.symmetryNode = symmetryNode;
				iq = BVFeatureQuery.isInsideMesh(mesh, node.getPosition());
				face = BVFeatureQuery.getNearestFaceToPoint(projection, null, mesh, node.getPosition());
				face.computeNormal(faceNormal);

				position = node.getPosition();
				projPosNormal = new Point3d(position);
				projPosNormal.sub(projection);
				projPosDistance = projPosNormal.norm();
				projPosNormal.normalize();
			}
			public ArrayList<ElementData> evaluateProjection(HashMap<FemNode3d, Point3d> nodeProjMap, PolygonalMesh mesh, FemGrid fg, FemMuscleModel fem) {
				//Get nearest face to node and to the centroid closest to the mesh surface of all the node's elements
				boolean splitFlag = false;
				ArrayList<ProjectionData> projData = new ArrayList<ProjectionData>();
				ArrayList<ElementData> deadElements = new ArrayList<ElementData>();
				for (FemElement3d element : node.getElementDependencies()) {
					ProjectionData pd = new ProjectionData(node, projPosNormal, position, element, mesh, NormalRule.CENTOID_PROJECTION_FACE_NORMAL);
					projData.add(pd);
					if (pd.invertedNormalFlag && !fg.elementDataMap.get(pd.element).isInternal) {
						splitFlag = true;
					}
				}                                                        

				Point3d[] avgProjs = ProjectionData.splitProjection(projection, projData, mesh);

				//nodeProjMap.put(node, avgProjs[0]);
				projectNodesToPoint(avgProjs[0]);

				if (splitFlag) {
					FemNode3d newNode = new FemNode3d();
					fem.addNode(newNode);
					newNode.setName("duplicate " + node.getName());
					newNode.setPosition(avgProjs[1]);
					RenderProps.setFaceColor(newNode, Color.cyan);
					for (ProjectionData pd : projData) {
						if (pd.invertedNormalFlag && !fg.elementDataMap.get(pd.element).isInternal) {
							//Swap out old node 
							FemNode3d[] elemNodes = pd.element.getNodes();
							for (int i = 0; i < 8; i++) {
								if (elemNodes[i] == node) {
									elemNodes[i] = newNode;
								}
							}

							//Remove the old element and add a new one with the new node
							HexElement newElement = new HexElement(elemNodes);
							fem.addElement(newElement);
							//RenderProps.setFaceColor(newElement, Color.magenta);
							newElement.setName("new element " + newElement.getNumber());
							deadElements.add(fg.elementDataMap.get(pd.element));
						}
					}
				}

				return deadElements;
			}

			public void projectNodesToNearestFace() {
				projectNodesToPoint(projection);
			}
			public void projectNodesToPoint(Point3d point) {
				node.setPosition(point);
				symmetryNode.setPosition(AuxTools.flipValue(point, FlipAxis.X));                             
			}

			public void symmetricallyProjectToAverageProjection(PolygonalMesh mesh) {
				//Create particles located at the projection points
				Point3d proj = new Point3d();
				BVFeatureQuery.getNearestFaceToPoint(proj, null, mesh, node.getPosition());

				//Get nearest face to node and to the centroid closest to the mesh surface of all the node's elements
				double count = 1.0;
				Point3d avgProj = new Point3d(proj);
				for (FemElement3d element : node.getElementDependencies()) {
					Point3d projCentroid = new Point3d();
					Point3d centroid = new Point3d();
					element.computeCentroid(centroid);

					BVFeatureQuery.getNearestFaceToPoint(projCentroid, null, mesh, centroid);
					avgProj.add(projCentroid);
					count++;
				}

				//Add the average centroid projection
				avgProj.scale(1.0/count);
				Point3d avgProjProj = new Point3d();
				BVFeatureQuery.getNearestFaceToPoint(avgProjProj, null, mesh, avgProj);

				node.setPosition(avgProj);
				symmetryNode.setPosition(AuxTools.flipValue(avgProjProj, FlipAxis.X)); 
			}
		}

		public static class ElementData {
			public InsideQuery iq;
			public Point3d proj = new Point3d();
			public Face face;
			public FemElement3d element, symmetryElement;
			public ArrayList<NodeData> nodes = new ArrayList<NodeData>();
			public Point3d centroid = new Point3d();

			public boolean isInternal = false;

			public ElementData(FemElement3d element, FemElement3d symmetryElement, PolygonalMesh mesh, FemGrid fg, double tolerance) {
				this.element = element;
				this.symmetryElement = symmetryElement;

				//Build the node data for this element
				for (FemNode3d node : element.getNodes()) {
					FemNode3d refNode = (fg.allNodes.indexOf(node) >= fg.nodeSymmetryIndex ? fg.nodeSymmetryMapReverse.get(node) : node);
					if (refNode != null) {
						nodes.add(fg.nodeDataMap.get(refNode));
					}
				}

				//Determine element containment by means of element centroid projection
				checkForCentroidContainment(mesh, tolerance);
			}

			public boolean checkForCentroidContainment(PolygonalMesh mesh, double tolerance) {
				//If the centroid was inside the mesh initially, then there is no need to check again
				if (iq != InsideQuery.INSIDE) {
					//Check again whether the centroid is inside the mesh
					return recheckForCentroidContainment(mesh, tolerance);
				}
				else {
					return true;
				}
			}

			public boolean recheckForCentroidContainment(PolygonalMesh mesh, double tolerance) {
				Point3d centroid = new Point3d();
				element.computeCentroid(centroid);

				Point3d centroidProjDiff = new Point3d(centroid);
				iq = BVFeatureQuery.isInsideMesh(mesh, centroid);
				face = BVFeatureQuery.getNearestFaceToPoint(proj, null, mesh, centroid);
				centroidProjDiff.sub(proj);
				isInternal = (iq == InsideQuery.INSIDE || centroidProjDiff.norm() <= tolerance);
				return isInternal;
			}

			public boolean checkNodeProximityToMesh(FemGrid fg, PolygonalMesh mesh, double tolerance) {
				double avgDist = 0.0;
				for (FemNode3d node : element.getNodes()) {
					NodeData nd = fg.nodeDataMap.get(node);
					if (nd.iq != InsideQuery.INSIDE) {
						Point3d proj = new Point3d();
						BVFeatureQuery.getNearestFaceToPoint(proj, null, mesh, node.getPosition());
						proj.sub(node.getPosition());
						avgDist += proj.norm();
					}                                   
				}
				avgDist /= 8.0;

				return avgDist <= tolerance;
			}

			public void addNodesToFem(FemMuscleModel fem, FemGrid fg, ArrayList<NodeData> nodeSet) {
				if (isInternal) {
					//First add the nodes of this element to the FEM
					for (FemNode3d node : element.getNodes()) {
						if (!fem.getNodes().contains(node)) {
							fem.addNode(node);

							if (fg.allNodes.indexOf(node) < fg.nodeSymmetryIndex) {
								nodeSet.add(fg.nodeDataMap.get(node));
							}
						}
					}

					//If this elements symmetry partner is not itself (such would be the case if the element is on the symmetry center line), then add the symmetry partners nodes to the FEM 
					if (element != symmetryElement) {
						for (FemNode3d node : symmetryElement.getNodes()) {
							if (!fem.getNodes().contains(node)) {
								fem.addNode(node);
							}
						}       
					}
				}
			}

			public void addElementsToFem(FemMuscleModel fem, FemGrid fg, ArrayList<ElementData> elementSet) {
				if (isInternal) {
					fem.addElement(element);
					elementSet.add(this);

					if (element != symmetryElement) {
						fem.addElement(symmetryElement);
					}
				}
			}

			public void projectNodesToMesh(PolygonalMesh mesh) {
				for (NodeData nd : nodes) {
					Point3d proj = new Point3d();
					BVFeatureQuery.getNearestFaceToPoint(proj, null, mesh, nd.node.getPosition());
					nd.projectNodesToPoint(proj);
				}
			}
		}

		public static class ProjectionData {
			public Vector3d centroidProjNormal = new Vector3d();
			public Point3d centroidProj = new Point3d();
			public Point3d centroid = new Point3d();
			public FemElement3d element;
			public FemNode3d node;
			public boolean invertedNormalFlag = false;
			public static enum NormalRule {
				CENTOID_PROJECTION_FACE_NORMAL,
				NODE_ORIGIN;
			}
			public ProjectionData(FemNode3d node, Vector3d nodeProjNormal, Point3d target, FemElement3d element, PolygonalMesh mesh, NormalRule normalRule) {
				this.node = node;
				this.element = element;
				element.computeCentroid(centroid);
				Face face = BVFeatureQuery.getNearestFaceToPoint(centroidProj, null, mesh, centroid);

				switch (normalRule) {
				case CENTOID_PROJECTION_FACE_NORMAL:
					face.computeNormal(centroidProjNormal);
					invertedNormalFlag = (centroidProjNormal.dot(nodeProjNormal) < 0.0);
					break;
				case NODE_ORIGIN:
					centroidProjNormal.sub(target, centroidProj);
					centroidProjNormal.normalize();
					invertedNormalFlag = (centroidProjNormal.dot(nodeProjNormal) < 0.0);
					break;
				default:
					break;

				}
			}

			public static Point3d[] splitProjection(Point3d mainProj, ArrayList<ProjectionData> projData, PolygonalMesh mesh) {
				double[] projCounts = new double[] {1.0, 0.0};
				Point3d[] avgProjs = new Point3d[]{mainProj, new Point3d()};
				for (ProjectionData pd : projData) {
					int idx = (pd.invertedNormalFlag ? 1 : 0);
					avgProjs[idx].add(pd.centroidProj);
					projCounts[idx]++;
				}

				avgProjs[0].scale(1.0/projCounts[0]);
				avgProjs[1].scale(1.0/(projCounts[1] > 0.0 ? projCounts[1] : 1.0));

				//Now project the averages of the projections back to the mesh
				Point3d[] avgProjProjs = new Point3d[]{new Point3d(), new Point3d()};
				BVFeatureQuery.getNearestFaceToPoint(avgProjProjs[0], null, mesh, avgProjs[0]);
				BVFeatureQuery.getNearestFaceToPoint(avgProjProjs[1], null, mesh, avgProjs[1]);
				return avgProjProjs;
			}
		}


		private static void forgeHexFemFromGrid(FemMuscleModel fem, PolygonalMesh mesh, FemGrid fg, double volumeThreshold, double optLength) {
			//Check node containment in mesh
			System.out.println("\tChecking node containment");
			for (int i = 0; i < fg.nodeSymmetryIndex; i++) {
				FemNode3d node = fg.allNodes.get(i);
				NodeData nd = new NodeData(node, fg.nodeSymmetryMapForward.get(node), mesh);
				fg.nodeDataMap.put(node, nd);
				fg.nodeDataMap.put(nd.symmetryNode, nd);
			}

			//Check element contaiment in mesh
			double optTolerance = 0.5*optLength*Math.sqrt(3);
			System.out.println("\tChecking element containment");
			for (int i = 0; i < fg.elementOverSymmetryIndex; i++) {
				FemElement3d element = fg.allElements.get(i);
				//Add elements that have centroids within the distance of the volumetric diagonal of the hex to the mesh
				fg.elementDataMap.put(element, new ElementData(element, fg.elementSymmetryMapForward.get(element), mesh, fg, optTolerance));
			}

			//Add remaining nodes to the model
			System.out.println("\tAdding internal nodes and elements to FEM");
			ArrayList<NodeData> nodeSet = new ArrayList<NodeData>();
			for (int i = 0; i < fg.elementOverSymmetryIndex; i++) {
				ElementData ed = fg.elementDataMap.get(fg.allElements.get(i));
				ed.addNodesToFem(fem, fg, nodeSet);
			}

			//Add remaining elements to the model
			ArrayList<ElementData> elementSet = new ArrayList<ElementData>();
			for (int i = 0; i < fg.elementOverSymmetryIndex; i++) {
				ElementData ed = fg.elementDataMap.get(fg.allElements.get(i));
				ed.addElementsToFem(fem, fg, elementSet);
			}

			//Project nodes of external elements to nearest faces
			for (NodeData nd : nodeSet) {
				//Only do projection for nodes that are not unequivocally inside the mesh
				if (nd.iq != InsideQuery.INSIDE) {
					//nd.evaluateProjection(nodeProjMap, mesh, fg, fem);
					nd.projectNodesToNearestFace();
				}
			}

			ArrayList<ElementData> externalElements = new ArrayList<ElementData>();
			for (ElementData ed : elementSet) {
				if (!ed.recheckForCentroidContainment(mesh, 0.0)) {
					externalElements.add(ed);
				}
			}

			//Iterate through external element node projection and element removal until no more elements remain
			do {


				//Add remaining elements to the model
				ArrayList<ElementData> removalElements = new ArrayList<ElementData>();
				for (ElementData ed : elementSet) {
					if (!ed.recheckForCentroidContainment(mesh, 0.0)) {
						//Project the nodes of the element to be remove onto the mesh
						ed.projectNodesToMesh(mesh);
						removalElements.add(ed);

						fem.removeElement(ed.element);
						fem.removeElement(ed.symmetryElement);
					}
				}

				//Remove excluded elements from the element set
				for (ElementData ed : removalElements) {
					elementSet.remove(ed);
					externalElements.remove(ed);
				}

				for (ElementData ed : elementSet) {
					if (!ed.recheckForCentroidContainment(mesh, 0.0)) {
						externalElements.add(ed);
					}
				}
			} 
			while (externalElements.size() > 0); 


			/*                   
			 * 
			 *
                     for (ElementData ed : removalElements) {
                            elementSet.remove(ed);
                            if (ed != null) {
                                   fem.removeElement(ed.element);
                                   fem.removeElement(ed.symmetryElement);
                            }
                            else {
                                   int tim = 1;
                            }
                     }

                     //Project nodes
                     for (NodeData nd : nodeSet) {
                            Point3d projLoc = nodeProjMap.get(nd.node);
                            if (projLoc != null) {
                                   nd.projectNodesToPoint(projLoc);
                            }
                     }       
			 */

			/*
                     //Since some of the nodes may have been repositioned due to projection to the surface mesh, reset the rest positions of the FEM
                     //fem.resetRestPosition();
                     /*
                     //Compute element volumes after the projection and remove any that are below the threshold
                     System.out.println("\tRemoving ill-formed elements");
                     double optVolume = optLength*optLength*optLength;
                     ArrayList<FemElement3d> elementsToRemove = new ArrayList<FemElement3d>();
                     for (FemElement3d elem : fem.getElements()) {

                            boolean invertedFlag = elem.isInvertedAtRest();
                            double vol = optVolume;//TODO there is an error in the new routine for calculating element volumes!!!! elem.computeVolumes();
                            if (vol/optVolume < volumeThreshold || invertedFlag) {
                                   System.out.println("\t\tRemoving element " + elem.getNumber() + (invertedFlag ? " - inverted" : " volume below " + String.format("%.3f", volumeThreshold*100) + "% threshold (" +  String.format("%.3f", vol/optVolume*100) +"%)"));
                                   elementsToRemove.add(elem);
                            }
                     }

                     //Perform the removal of the element from the FEM and from the elementData array
                     for (FemElement3d elem : elementsToRemove) {
                            fem.removeElement(elem);
                            ElementData.removeElementData(fg.elementSet, elem);
                     }
			 */
			//Remove any stray nodes
			System.out.println("\tRemoving stray nodes");
			FemMiscMethods.cleanFem(fem);

			//As a precaution, reset all node rest positions
			//fem.resetRestPosition();

			if (fem.getNodes().size() == 0) {
				System.err.println("Failed to generate well-formed hexahedral FEM mesh for the provided geometry. Try changing the input parameters.");
			}
			else {
				System.out.println("\tFEM '" + fem.getName() + "' has successfully been forged: " + fem.getElements().size() + " elements and " + fem.getNodes().size() + " nodes");
			}
		}



		public static class FemGrid {
			public HashMap<String, FemNode3d> stringIDnodeMap = new HashMap<String, FemNode3d>();

			public ArrayList<FemNode3d> allNodes = new ArrayList<FemNode3d>();
			public int nodeSymmetryIndex;
			public HashMap<FemNode3d, FemNode3d> nodeSymmetryMapForward = new HashMap<FemNode3d, FemNode3d>();
			public HashMap<FemNode3d, FemNode3d> nodeSymmetryMapReverse = new HashMap<FemNode3d, FemNode3d>();
			public HashMap<FemNode3d, NodeData> nodeDataMap = new HashMap<FemNode3d, NodeData>();

			public ArrayList<FemElement3d> allElements = new ArrayList<FemElement3d>();
			public int elementOverSymmetryIndex;
			public int elementMidlineSymmetryIndex;
			public HashMap<FemElement3d, FemElement3d> elementSymmetryMapForward = new HashMap<FemElement3d, FemElement3d>();
			public HashMap<FemElement3d, FemElement3d> elementSymmetryMapReverse = new HashMap<FemElement3d, FemElement3d>();
			public ArrayList<FemElement3d> elementsLeft = new ArrayList<FemElement3d>();
			public ArrayList<FemElement3d> elementsCenter = new ArrayList<FemElement3d>();
			public ArrayList<FemElement3d> elementsRight = new ArrayList<FemElement3d>();
			public HashMap<FemElement3d, ElementData> elementDataMap = new HashMap<FemElement3d, ElementData>();


			/** Creates a new fem grid data collection from null input <b>fg</b> **/
			public static FemGrid createHexGrid(int numX, int numY, int numZ, double optLength, Vector3d femMin) {
				FemGrid fg = new FemGrid();

				//Create nodes (if n is the number of elements in a row, then n + 1 is the number of nodes in the same row)
				System.out.println("\tGenerating nodes");
				for (int i = 0; i < numX; i++) {
					for (int j = 0; j < numY; j++) {
						for (int k = 0; k < numZ; k++) {
							double x = femMin.x + (i*optLength);
							double y = femMin.y + (j*optLength);
							double z = femMin.z + (k*optLength);
							FemNode3d newNode = new FemNode3d(x, y, z);
							newNode.setName("Node_" + String.valueOf(i) + "_" + String.valueOf(j) + "_" + String.valueOf(k));
							fg.allNodes.add(newNode);
							fg.stringIDnodeMap.put(newNode.getName(), newNode);
						}
					}
				}

				//Create elements
				System.out.println("\tGenerating elements");
				for (int i = 0; i < numX - 1; i++) {
					for (int j = 0; j < numY - 1; j++) {
						for (int k = 0; k < numZ - 1; k++) {
							FemNode3d[] nodes = new FemNode3d[8];

							int n = 0;
							for (int keyI = i; keyI <= i + 1; keyI++) {
								for (int keyJ = j; keyJ <= j + 1; keyJ++) {
									for (int keyK = k; keyK <= k + 1; keyK++) {
										nodes[n++] = fg.stringIDnodeMap.get("Node_" + String.valueOf(keyI) + "_" + String.valueOf(keyJ) + "_" + String.valueOf(keyK));
										//System.out.println(nodes[n - 1].getName() + ": " + AuxTools.pointAsString(nodes[n - 1].getPosition(), 3));
									}      
								}       
							}

							//To create a hex element, the first four nodes should describe a single face (in the "X-Z" plane) of the element, arranged counter-clockwise about the outward-directed normal. The last four nodes should describe the corresponding nodes on the opposite face (these will be arranged clockwise about that face's normal).
							FemElement3d hex = HexData.createHex(nodes, 4, 6, 7, 5, 0, 2, 3, 1);
							fg.allElements.add(hex);
						}
					}
				}

				return fg;
			}


			/** Builds a mapping of symmetrical node pairs across the X axis. If a node cannot be assigned a pair, it is assigned a null mate. **/
			public static void createSymmetryMap(FemGrid fg, double midX, double optLength) {
				System.out.println("\tCreating symmetry maps");
				double tol = optLength*1e-1;

				//Find the index of the first node on the other side of the axis of symmetry
				int nodeSymIdx = 0;
				for (int i = 0; i < fg.allNodes.size(); i++) {
					if (fg.allNodes.get(i).getPosition().x > midX) {
						nodeSymIdx = i;
						break;
					}
				}
				fg.nodeSymmetryIndex = nodeSymIdx;

				for (int i = 0; i < nodeSymIdx; i++) {
					Point3d pos = fg.allNodes.get(i).getPosition();

					//Find a suitable candidate node on the other side of the axis of symmetry
					for (int j = nodeSymIdx; j < fg.allNodes.size(); j++) {
						Point3d cPos = fg.allNodes.get(j).getPosition();

						if ((Math.abs(-pos.x - cPos.x) <= tol) && (pos.y == cPos.y) && (pos.z == cPos.z)) {
							fg.nodeSymmetryMapForward.put(fg.allNodes.get(i), fg.allNodes.get(j));
							fg.nodeSymmetryMapReverse.put(fg.allNodes.get(j), fg.allNodes.get(i));
							RenderProps.setPointColor(fg.allNodes.get(i), Color.blue);
							RenderProps.setPointColor(fg.allNodes.get(j), Color.red);
							break;
						}
					}
				}

				//Find the index of the first element on the other side of the axis of symmetry
				fg.elementMidlineSymmetryIndex = -1;
				fg.elementOverSymmetryIndex = -1;
				for (int i = 0; i < fg.allElements.size(); i++) {
					//Determine if the element is on the axis of symmetry or not
					int overSymIndexCount = 0;
					FemElement3d element = fg.allElements.get(i);
					for (FemNode3d node : element.getNodes()) {
						if (fg.allNodes.indexOf(node) >= fg.nodeSymmetryIndex) {
							overSymIndexCount++;
						}
					}

					if (overSymIndexCount == 4) {
						fg.elementsCenter.add(element);
						fg.elementSymmetryMapForward.put(element, element);
						fg.elementSymmetryMapReverse.put(element, element);
						if (fg.elementMidlineSymmetryIndex == -1) fg.elementMidlineSymmetryIndex = i;
						RenderProps.setFaceColor(element, Color.yellow);
					}
					else if (overSymIndexCount == 8) {
						fg.elementsRight.add(element);
						fg.elementSymmetryMapForward.put(element, element);
						fg.elementSymmetryMapReverse.put(element, element);
						if (fg.elementOverSymmetryIndex == -1) fg.elementOverSymmetryIndex = i;
						RenderProps.setFaceColor(element, Color.red);
					}
					else if (overSymIndexCount == 0) {
						fg.elementsLeft.add(element);
					}
					else if (overSymIndexCount > 0) {
						System.err.println("A symmetry error has occurred for element " + i);
					}

				}

				//Match elements on the one side of the axis of symmetry with those on the other
				for (int i = 0; i < fg.elementOverSymmetryIndex; i++) {

					FemElement3d element = fg.allElements.get(i);
					//Collect all of the symmetrizing nodes for this element
					ArrayList<FemNode3d> symNodes = new ArrayList<FemNode3d>();
					for (FemNode3d node : element.getNodes()) {
						symNodes.add(fg.nodeSymmetryMapForward.get(node));
					}

					//Search for an element that posesses these nodes
					for (FemElement3d symElement : fg.elementsRight) {
						boolean matchingElement = true;
						for (FemNode3d symNode : symElement.getNodes()) {
							if (!symNodes.contains(symNode)) {
								matchingElement = false;
								break;
							}
						}

						if (matchingElement) {
							fg.elementSymmetryMapForward.put(element, symElement);
							fg.elementSymmetryMapReverse.put(symElement, element);
							RenderProps.setFaceColor(element, Color.blue);
						}
					}
				}
			}
		}
	}

	public static ArrayList<FemElement3d> findInvertedElements(FemMuscleModel fem) {
		ArrayList<FemElement3d> invertedElements = new ArrayList<FemElement3d>();
		for (FemElement3d element : fem.getElements()) {
			if (element.isInvertedAtRest()) {
				invertedElements.add(element);
			}
		}
		return invertedElements;
	}

	/** Returns the minimum Jacobian for this element. **/
	public static double calculateElementQuality(FemElement3d element) {
		IntegrationPoint3d[] ipnts = element.getIntegrationPoints();
		double minDet = Double.MAX_VALUE;
		for (int i = 0; i < ipnts.length; i++) {
			IntegrationData3d idata = new IntegrationData3d();

			Matrix3d J0 = new Matrix3d();
			J0.setZero();
			for (int j = 0; j < element.getNodes().length; j++) {
				Vector3d pos = element.getNodes()[j].getRestPosition();
				Vector3d dNds = ipnts[i].getGNs()[j];
				J0.addOuterProduct(pos.x, pos.y, pos.z, dNds.x, dNds.y, dNds.z);
			}
			minDet = Math.min(idata.getInvJ0().fastInvert(J0), minDet);
		}

		return minDet /= ipnts.length;
	}

	/** Returns the minimum Jacobian for this element. **/
	public static double calculateMinimumJacobian(FemElement3d element) {
		IntegrationPoint3d[] ipnts = element.getIntegrationPoints();
		double minDet = Double.MAX_VALUE;
		for (int i = 0; i < ipnts.length; i++) {
			IntegrationData3d idata = new IntegrationData3d();

			Matrix3d J0 = new Matrix3d();
			J0.setZero();
			for (int j = 0; j < element.getNodes().length; j++) {
				Vector3d pos = element.getNodes()[j].getPosition();
				Vector3d dNds = ipnts[i].getGNs()[j];
				J0.addOuterProduct(pos.x, pos.y, pos.z, dNds.x, dNds.y, dNds.z);
			}
			minDet = Math.min(idata.getInvJ0().fastInvert(J0), minDet);
		}

		return minDet /= ipnts.length;
	}
}
