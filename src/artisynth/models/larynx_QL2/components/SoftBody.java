package artisynth.models.larynx_QL2.components;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemMuscleStiffener;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.femmodels.PointFem3dAttachment;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.InactiveMuscle;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.models.larynx_QL2.VocalTractBase;
import artisynth.models.larynx_QL2.components.LegacyReaders.AnsysFaceMuscleFiberReader;
import artisynth.models.larynx_QL2.components.LegacyReaders.AnsysMuscleElemReader;
import artisynth.models.larynx_QL2.components.LegacyReaders.AnsysMuscleFiberReader;
import artisynth.models.larynx_QL2.components.LegacyReaders.AnsysReadMethod;
import artisynth.models.larynx_QL2.components.Names.VocalTractType;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemIOMethods;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemMiscMethods;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemMuscleMethods;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemVolumeMethods;
import artisynth.models.larynx_QL2.tools.AuxTools;
import artisynth.models.larynx_QL2.tools.FemTools;
import artisynth.models.larynx_QL2.tools.RenderTools;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVFeatureQuery.InsideQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;

/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class SoftBody extends AbstractBody {
	private FemMuscleModel fem;
	private ArrayList<PolygonalMesh> collisionSubsurfaces = new ArrayList<PolygonalMesh>();
	private FemMuscleStiffener stiffener;
	private FemVolumeMethods volumeData;
	private HashMap<FemElement3d, Color> elementColorMap = new HashMap<FemElement3d, Color>();
	
	private String[] muscleNames;
	private boolean useStiffenerFlag;
	private boolean attachFemFlag;
	private static boolean useHardCompressibilityFlag = false;	//false for the Quantal Larynx
	private static double femMuscleForceScaling = 1.0;
	private static double femMuscleMaxForce = 200.0;
	private static double femMaterialScaling = 1.0;
	private static double bulkModulusScaling = 10.0;	//1000 works well; John Lloyd suggests 100 to 1000 times greater than elastic modulus

	public SoftBody(MechModel mech, VocalTractType vtModel, Names name) {
		super(name.getName(), vtModel.checkFemUsage(name), true, true, true, 1.0, new RigidTransform3d());
		this.useStiffenerFlag = true;
		this.attachFemFlag = true;

		addToModel(mech, vtModel, this);
	}

	/** Sets up a FEM specified in CSV format. In the model's data directory, there should be two files, one containging FEM node & element data <i><b>"[fem_name].csv"</i></b>, and another containing FEM muscle data <i><b>"[fem_name]_muscles.csv"</i></b>.
	 * The routine will load any collision subsurface OBJ files found in the model's data directory following the naming convention, <i><b>"[fem_name]_collisionSubsurface_[subsurface_number].obj"</i></b>
	 * A conventional Mooney-Rivlin material and numerical method properties are specified (see {@link artisynth.models.moisik.components.FemMethods#setMaterialToHumanFlesh setMaterialToHumanFlesh} and {@link artisynth.models.moisik.components.FemMethods#setUpDefaultFemNumerics setUpDefaultFemNumerics}).
	 * The model is then added to the current MechModel <i>mech</i>. 
	 * 
	 * @param mech main mech model
	 * @param vtModel vocal tract model
	 * @param sb soft body
	 */
	public static void addToModel(MechModel mech, VocalTractType vtModel, SoftBody sb) {
		if (sb.isUsed()) {
			System.out.println("Loading FEM '" + sb.getName() + "'");
			//Read FEM node and element data from CSV ("[fem_name].csv")
			sb.fem = FemIOMethods.readFemFromCSV(sb.getName(), vtModel.getFullFileName(sb.name) + Names.Files.FEM.getSuffix());

			//Reset rest positions and check for any inverted elements and warn if there are any
			sb.fem.resetRestPosition();
			FemMiscMethods.checkForInvertedElements(sb.fem);

			//Locate and load collision surfaces
			FemIOMethods.readCollisionSurfacesFromOBJ(sb, vtModel);

			//Read muscles from the associated CSV file ("[fem_name]_muscles.csv")
			try {
				//New method that reduces number of frame markers required
				FemIOMethods.readMusclesFromCSV_new(sb.fem, vtModel.getFullFileName(sb.name + Names.Files.FEM_MUSCLES.getSuffix()));
				
				
			} catch (Exception ex) {
				//Old method for old models
			         for (MuscleBundle mb : sb.fem.getMuscleBundles()) {
			   
			            sb.fem.removeMuscleBundle(mb);
			         }
			         FemIOMethods.readMusclesFromCSV(sb.fem, vtModel.getFullFileName(sb.name + Names.Files.FEM_MUSCLES.getSuffix()));
			}
			
			FemMuscleMethods.initializeMuscles(sb.fem);

			//Specify muscle bundle exciters
			FemMuscleMethods.createMuscleExciters(sb.fem, true);

			//Setup FEM properties: material, numerical method settings, render properties, stiffener, and element volume stats
			FemMiscMethods.setMaterialToHumanFlesh(sb.fem, femMaterialScaling, useHardCompressibilityFlag);
			FemMiscMethods.setUpDefaultFemNumerics(sb.fem);

			sb.stiffener = new FemMuscleStiffener(sb.fem);
			sb.volumeData = FemVolumeMethods.createFemVolumeData(sb.fem);

			mech.addModel(sb.fem);

			//Set collision behaviour
			mech.setCollisionBehavior(sb.fem, sb.fem, true);

			//Attach the FEM to RigidBodies & free any nodes that are connected to elements which form muscles
			FemIOMethods.readAttachmentsFromCSV(mech, sb.fem, vtModel.getFullFileName(sb.name + Names.Files.ATTACHMENTS.getSuffix()));

			//TODO how do we reconcile attached nodes that also form elements containing muscles???
			//FemMuscleMethods.freeNodesOfMuscleElements(mech, sb.fem);

			//Setup render properties and set element colors based on color data (if it exists)
			RenderTools.setupRenderProperties(sb.fem, FemMethods.defaultElementColor, FemMethods.defaultNodeColor);
			FemIOMethods.readColorFromCSV(mech, sb.fem, vtModel.getFullFileName(sb.name + Names.Files.COLORS.getSuffix()));
			FemMiscMethods.fillElementColorMap(sb.fem, sb.elementColorMap);
			
			//Remove nodes with no attachments or dependencies
			ArrayList<FemNode3d> removeNodes = new ArrayList<FemNode3d>();
			for (FemNode3d node : sb.fem.getNodes()) {
				if ((!FemMiscMethods.checkNodeFiberAttachment(sb.fem, node)) && (node.getElementDependencies().size() < 1)) {
					removeNodes.add(node);
				}
			}
			
			for (FemNode3d node : removeNodes) {
				RenderProps.setVisible(node, true);
				RenderProps.setPointColor(node, Color.green);
				//System.out.println("Removing node: " + node.getNumber()); 
				//sb.fem.removeNode(node);
			}
			
		}
	}

	public static class FemMethods {

		public static class FemMiscMethods {

			public static boolean checkNodeFiberAttachment(FemMuscleModel fem, FemNode3d node) {
				for (MuscleBundle mb : fem.getMuscleBundles()) {
					for (Muscle m : mb.getFibres()) {
						if ((m.getFirstPoint() == node) || (m.getSecondPoint() == node)) {
							return true;
						}
					}
				}
				return false;
			}
			
			public static void fillElementColorMap(FemMuscleModel fem, HashMap<FemElement3d, Color> elementColorMap) {
				for (FemElement3d element : fem.getElements()) {
					try {
						elementColorMap.put(element, element.getRenderProps().getFaceColor());
					}
					catch (Exception ex) {
						elementColorMap.put(element, Color.gray);
					}
				}
			}
			
			
			/** Sets the FEM to use a Mooney Rivlin material [as specified in Nasari 2010 and Buchaillard 2009]. The values used provided in MKS units, but can be optionally scaled.
			 * Incompressibility is turned ON.
			 * 
			 * @param fem fem model
			 * @param scale to amplify stiffness parameters
			 * @param useHardCompressibilityFlag hard incompressibility flag
			 */
			public static void setMaterialToHumanFlesh(FemMuscleModel fem, double scale, boolean useHardCompressibilityFlag) {
				if (scale <= 0.0) {
					scale = 1.0;
				}

				//Set material properties of the face (Nazari 2010 Motor Control paper)
				MooneyRivlinMaterial mrmat = new MooneyRivlinMaterial();

				mrmat.setC10(2500*scale); // Pa
				mrmat.setC20(1175*scale); // Pa
				mrmat.setBulkModulus(bulkModulusScaling*mrmat.getC10());  //John Lloyd says: bulk modulus 100 to 1000 times larger than the elastic modulus is best (and fastest) 
				//previous *10 //mrmat.setBulkModulus (100*mrmat.getC10 ()); // 100x c10 ~ ; possion=0.49

				mrmat.setJLimit(0.0);
				fem.setMaterial(mrmat);
				fem.setParticleDamping(19.0);             //[Buchaillard 2009] Rayleigh damping C = a M + b K ... a = 40 s^-1 ... b = 0.03 s
				fem.setStiffnessDamping(0.03);
				fem.setDensity(1040*scale*scale*scale);                 //[Buchaillard 2009] density = 1040 kg m^-3             

				fem.setIncompressible((useHardCompressibilityFlag ? IncompMethod.NODAL : IncompMethod.OFF));
				fem.setSoftIncompMethod(IncompMethod.NODAL);
			}

			public static void setMooneyRivlinMaterial(FemMuscleModel fem, double density, double particleDamping, double stiffnessDamping, IncompMethod incompressiblityMethod, IncompMethod softIncompressibilityMethod, double bulkModulus, double C10, double C20, double jLim) {
				//Set material properties of the face (Nazari 2010 Motor Control paper)
				MooneyRivlinMaterial mrmat = new MooneyRivlinMaterial();

				mrmat.setC10(C10); // Pa
				mrmat.setC20(C20); // Pa
				mrmat.setBulkModulus(bulkModulus);  	 


				mrmat.setJLimit(jLim);
				fem.setMaterial(mrmat);
				fem.setParticleDamping(particleDamping); 
				fem.setStiffnessDamping(stiffnessDamping);
				fem.setDensity(density);                

				fem.setIncompressible(incompressiblityMethod);
				fem.setSoftIncompMethod(softIncompressibilityMethod);
			}

			public static void setMaterialToCompressibleLinear(FemMuscleModel fem, double k, double d) {
				fem.setLinearMaterial(k, d, true);
				fem.setIncompressible(IncompMethod.OFF);
			}

			public static void checkForInvertedElements(FemMuscleModel fem) {
				for (FemElement3d element : fem.getElements()) {
					if (element.isInverted() || element.isInvertedAtRest()) {
						System.err.println("Warning: element " + element.getNumber() + " of FEM '" + fem.getName() + "' is inverted. Consider correcting the mesh.");
					}
				}

			}

			/** Turns dynamics on, makes the FEM incompressible, adds gravity, and sets the integrator to constrained backward Euler with 100 implicit iterations and 1e-4 implicit precision. **/
			public static void setUpDefaultFemNumerics(FemMuscleModel fem) {
				fem.setGravity(0.0, -9.81, 0.0);
				fem.setDynamicsEnabled(true);
				fem.setIntegrator(Integrator.ConstrainedBackwardEuler);
				fem.setImplicitIterations(100);
				fem.setImplicitPrecision(0.0001);
			}

			public static ArrayList<FemElement3d> getElements(Point point) {
				ArrayList<FemElement3d> elements = new ArrayList<FemElement3d>();
				if (point instanceof FemNode3d) {
					elements.addAll(((FemNode3d) point).getElementDependencies());
				}
				else if (point instanceof FemMarker){
					elements.add((FemElement3d) ((FemMarker) point).getElement());
				}
				return elements;
			}

			public static Point createRightSideMarker(FemMuscleModel fem, Point p, Plane midSagittalPlane) {
				Point rightPt = null;
				Point3d pos = p.getPosition();
				double dist = midSagittalPlane.distance(pos);
				double midsagittalTol = 1e-4;
				if (dist < -midsagittalTol) {
					Point3d reflect = new Point3d();
					midSagittalPlane.reflect(reflect, pos);

					FemElement3dBase elem = fem.findContainingElement(reflect);
					if (elem == null) {
						Point3d origreflect = new Point3d(reflect);
						elem = fem.findNearestSurfaceElement(reflect, origreflect);
					}

					rightPt = new FemMarker(fem.getElement(1), reflect);
					fem.addMarker((FemMarker)rightPt);
				} else {
					rightPt = p; // right-side or mid-sagittal point
				}

				return rightPt;
			}
			public static FemNode3d findNodeFromPosition(FemMuscleModel fem, Point3d pos) {
				for (FemNode3d node : fem.getNodes()) {
					Vector3d diff = new Vector3d(node.getPosition());
					diff.sub(pos);
					double norm = diff.norm();

					if (norm < 1e-10) {
						return node;
					}
				}

				System.err.println("Could not locate node in '" + fem.getName() + "' that matches the given position");
				return null;
			}

			public static PolygonalMesh createSubsurface(FemMuscleModel fem, String name, PolygonalMesh mesh) {
				PolygonalMesh subSurface = new PolygonalMesh();

				subSurface.setFixed(false);
				//subSurface.setUseDisplayList(true);
				subSurface.setRenderBuffered(true);
				subSurface.setName(name);

				HashMap<FemNode3d,Vertex3d> nodeVertexMap = new HashMap<FemNode3d,Vertex3d>();
				ArrayList<Vertex3d> vertexList = new ArrayList<Vertex3d>();

				boolean subSurfaceGenerationFailureFlag = false;
				for (Face face : mesh.getFaces()) {
					if (subSurfaceGenerationFailureFlag) {
						break;
					}

					vertexList.clear();
					for (Vertex3d vertex : face.getVertices()) {
						FemNode3d node = findNodeFromPosition(fem, vertex.getPosition());
						if (node != null) {
							Vertex3d femVertex = nodeVertexMap.get(node);
							if (femVertex == null) {
								femVertex = new Vertex3d(node.getPosition());
								nodeVertexMap.put(node, femVertex);
								subSurface.addVertex(femVertex);
							}
							vertexList.add(femVertex);
						}
						else {
							subSurfaceGenerationFailureFlag = true;
							break;
						}
					}

					if (vertexList.size() != 3) {
						System.err.println("Error building collision mesh '" + name + "' for FEM '" + fem.getName() + "': face has " + vertexList.size() + " vertices instead of 3");
					}
					else {
						subSurface.addFace(vertexList.toArray(new Vertex3d[0]));
					}
				}

				if (subSurface.numVertices() == 0) {
					System.err.println("An error occurred attempting to build collision mesh '" + name + "' for FEM '" + fem.getName() + "': face has " + vertexList.size() + " vertices instead of 3");
				}
				
				if (!subSurface.isClosed()) {
					
					System.err.println("Warning: collision mesh '" + name + "' for FEM '" + fem.getName() + "' is not closed.");
				}

				return subSurface;
			}




			/** Generates a clone FEM of thie input FEM to ensure numbering consistency (e.g. if numbering is discontinuous). This method is intended to be used for resaving modified FEM models that may have had elements removed. **/
			public static FemMuscleModel createRenumberedFem(FemMuscleModel fem) {
				//For numbering consistency, create a temporary FEM based on the input FEM
				FemMuscleModel renumberedFem = new FemMuscleModel("renumbered " + fem.getName());
				HashMap<Integer, Integer> nodeNumberMap = new HashMap<Integer, Integer>();
				HashMap<Integer, Integer> elementNumberMap = new HashMap<Integer, Integer>();

				//First add nodes with element dependencies from the input FEM
				for (FemNode3d node : fem.getNodes()) {

					int prevNumber = node.getNumber();
					//System.out.print("Input node " + node.getNumber());
					renumberedFem.addNode(node);
					//System.out.print(" becomes node " + node.getNumber());
					//System.out.print(" becomes " + renumberedFem.getNode(renumberedFem.getNodes().size() - 1).getNumber() + "\n");
					nodeNumberMap.put(prevNumber, renumberedFem.getNode(renumberedFem.getNodes().size() - 1).getNumber());

				}

				//Now add the elements
				for (FemElement3d element : fem.getElements()) {
					int prevNumber = element.getNumber();
					//System.out.print("Input element " + element.getNumber());
					renumberedFem.addElement(element);
					//System.out.print(" becomes element " + element.getNumber() + "\n");
					elementNumberMap.put(prevNumber, element.getNumber());
				}

				//Add surfaces
				for (FemMeshComp mesh : fem.getMeshComps()) {
					//renumberedFem.addMeshComp(mesh);
				}

				//Finally add all muscle bundles
				for (MuscleBundle mb : fem.getMuscleBundles()) {
					MuscleBundle newBundle = new MuscleBundle(mb.getName());
					newBundle.setFibresActive(true);
					LinkedList<Muscle> newFascicle = new LinkedList<Muscle>();
					newBundle.addFascicle(newFascicle);
					renumberedFem.addMuscleBundle(newBundle);

					for (MuscleElementDesc med : mb.getElements()) {
						newBundle.addElement(med);
						/*
                                          Integer elementNumber = elementNumberMap.get(med.getElement().getNumber());
                                          if (elementNumber != null) {
                                                 FemElement3d renumberedElement = renumberedFem.getElementByNumber(elementNumber);
                                                 MuscleElementDesc newMed = new MuscleElementDesc(renumberedElement, new Vector3d());
                                                 newBundle.addElement(newMed);
                                          }
                                          else {
                                                 System.err.println("Error creating renumbered FEM during copying of muscle '" + mb.getName() + "': original element " + med.getElement().getNumber() + " not found in renumbered FEM.");
                                          }
						 */
					}

					for (Muscle m : mb.getFibres()) {
						Muscle newMuscle = new Muscle();

						renumberedFem.addMuscleBundle(newBundle);
						if (m.getFirstPoint() instanceof FemMarker) {
							FemMarker fm1 = (FemMarker) m.getFirstPoint();
							//If the muscle fiber is missing an element (because it was removed through editing, then this muscle fiber should not be included in the model)
							if (fm1.getElement().getParent() == null) {
								continue;
							}
							FemMarker m1 = new FemMarker(m.getFirstPoint().getPosition());
							renumberedFem.addMarker(m1, fm1.getElement());
							newMuscle.setFirstPoint(m1);
						}
						else if (m.getFirstPoint() instanceof FemNode3d) {
							newMuscle.setFirstPoint(m.getFirstPoint());
						}

						if (m.getSecondPoint() instanceof FemMarker) {
							FemMarker fm2 = (FemMarker) m.getSecondPoint();
							//If the muscle fiber is missing an element (because it was removed through editing, then this muscle fiber should not be included in the model)
							if (fm2.getElement().getParent() == null) {
								continue;
							}
							FemMarker m2 = new FemMarker(m.getSecondPoint().getPosition());
							renumberedFem.addMarker(m2, fm2.getElement());
							newMuscle.setSecondPoint(m2);
						}
						else if (m.getSecondPoint() instanceof FemNode3d) {
							newMuscle.setSecondPoint(m.getSecondPoint());
						}

						try {
						newBundle.addFibre(newMuscle);
						newFascicle.add(newMuscle);
						}
						catch (Exception ex) {
							int j = 1;
						}
					}

				}

				return renumberedFem;       
			}


			
			
			public static void attachToRigidBodies(MechModel mech, FemMuscleModel fem, RigidBody[] bodies, int[][] nodeIndices, Color[] nodeColors) {
				int bodyCount = 0;
				for (RigidBody rb : bodies) {
					for (int i : nodeIndices[bodyCount]) {
						mech.attachPoint(fem.getByNumber(i), rb);
						RenderProps.setPointColor(fem.getByNumber(i), nodeColors[i]);
					}
					bodyCount++;
				}
			}




			public static void cleanFem(FemMuscleModel fem) {
				//Prune unnecessary nodes from the face model
				ArrayList<FemNode3d> strayNodes = new ArrayList<FemNode3d>();
				for (FemNode3d node : fem.getNodes()) {
					if (node.getElementDependencies().size() == 0) {
						strayNodes.add(node);
					}
				}
				for (FemNode3d stray : strayNodes) {
					fem.removeNode(stray);
				}

			}

			public static ArrayList<FemElement3d> getDependentElements(ArrayList<FemNode3d> subNodes) {
				ArrayList<FemElement3d> elementSet = new ArrayList<FemElement3d>();
				for (FemNode3d node : subNodes) {
					for (FemElement3d element : node.getElementDependencies()) {
						if (!elementSet.contains(element)) {
							elementSet.add(element);
						}
					}
				}
				return elementSet;
			}

			public static FemMuscleModel createSubFem(String name, ArrayList<FemNode3d> subNodes) {
				FemMuscleModel subFem = new FemMuscleModel(name);

				//First find all nodes not included in the sub node list (because they are attached to depenent elements not contained by the subnode capture space)
				ArrayList<FemNode3d> nodeSet = new ArrayList<FemNode3d>();
				ArrayList<FemElement3d> elementSet = new ArrayList<FemElement3d>();
				for (FemNode3d node : subNodes) {
					for (FemElement3d element : node.getElementDependencies()) {
						if (!elementSet.contains(element)) {
							elementSet.add(element);
						}

						for (FemNode3d candidateNode : element.getNodes()) {
							if (!nodeSet.contains(candidateNode)) {
								nodeSet.add(candidateNode);
							}
						}
					}
				}


				//Add nodes
				for (FemNode3d node : nodeSet) {
					subFem.addNode(node);
				}

				//Add dependent elements
				for (FemElement3d element : elementSet) {
					subFem.addElement(element);
				}

				return subFem;
			}
		}

		public static class FemMuscleMethods {

			public static void copyMusclesFromFem(FemMuscleModel sourceFem, FemMuscleModel targetFem) {

				//Finally add all muscle bundles
				for (MuscleBundle mb : sourceFem.getMuscleBundles()) {
					MuscleBundle newBundle = new MuscleBundle(mb.getName());
					newBundle.setFibresActive(true);
					LinkedList<Muscle> newFascicle = new LinkedList<Muscle>();
					newBundle.addFascicle(newFascicle);
					targetFem.addMuscleBundle(newBundle);

					for (MuscleElementDesc med : mb.getElements()) {
						newBundle.addElement(med);
						/*
                          Integer elementNumber = elementNumberMap.get(med.getElement().getNumber());
                          if (elementNumber != null) {
                                 FemElement3d renumberedElement = renumberedFem.getElementByNumber(elementNumber);
                                 MuscleElementDesc newMed = new MuscleElementDesc(renumberedElement, new Vector3d());
                                 newBundle.addElement(newMed);
                          }
                          else {
                                 System.err.println("Error creating renumbered FEM during copying of muscle '" + mb.getName() + "': original element " + med.getElement().getNumber() + " not found in renumbered FEM.");
                          }
						 */
					}

					for (Muscle m : mb.getFibres()) {
						Muscle newMuscle = new Muscle();

						targetFem.addMuscleBundle(newBundle);
						if (m.getFirstPoint() instanceof FemMarker) {
							FemMarker fm1 = (FemMarker) m.getFirstPoint();
							//If the muscle fiber is missing an element (because it was removed through editing, then this muscle fiber should not be included in the model)
							if (fm1.getElement().getParent() == null) {
								continue;
							}
							FemMarker m1 = new FemMarker(m.getFirstPoint().getPosition());
							targetFem.addMarker(m1, fm1.getElement());
							newMuscle.setFirstPoint(m1);
						}
						else if (m.getFirstPoint() instanceof FemNode3d) {
							newMuscle.setFirstPoint(m.getFirstPoint());
						}

						if (m.getSecondPoint() instanceof FemMarker) {
							FemMarker fm2 = (FemMarker) m.getSecondPoint();
							//If the muscle fiber is missing an element (because it was removed through editing, then this muscle fiber should not be included in the model)
							if (fm2.getElement().getParent() == null) {
								continue;
							}
							FemMarker m2 = new FemMarker(m.getSecondPoint().getPosition());
							targetFem.addMarker(m2, fm2.getElement());
							newMuscle.setSecondPoint(m2);
						}
						else if (m.getFirstPoint() instanceof FemNode3d) {
							newMuscle.setSecondPoint(m.getSecondPoint());
						}

						newBundle.addFibre(newMuscle);
						newFascicle.add(newMuscle);
					}
                }
			}
			
			public static void initializeMuscles(FemMuscleModel fem) {
				if (fem != null) {
					//Fibers will be used, therefore inactivate the muscle material
					fem.setMuscleMaterial(new InactiveMuscle());
					
					//Create fascicles from linked muscles
					createFasciclesInBundles(fem);
					
					//Set muscle properties for each fiber in every muscle bundle
					for (MuscleBundle mb : fem.getMuscleBundles()) {
						mb.setMaxForce(femMuscleMaxForce / mb.getFibres().size());
						mb.setFibresActive(true);
						
						for (Muscle m : mb.getFibres()) {
							/*
							PeckAxialMuscle pam = new PeckAxialMuscle();
							pam.setAxialMuscleMaterialProps(forceScaling, m.getLength(), m.getLength()*2, 0.0, 0.2, 0.1, forceScaling);	//length scaling was 1.15
							m.setMaterial(pam);
							m.resetLengthProps();
							 */

							ConstantAxialMuscle mat = new ConstantAxialMuscle();
							mat.setForceScaling(femMuscleForceScaling);
							mat.setMaxForce(femMuscleForceScaling);
							double len = m.getFirstPoint().distance(m.getSecondPoint());
							mat.setOptLength(len);
							mat.setMaxLength(2*len);
							m.setMaterial(mat);
						}
						
						setVolumeDepMaxForce(mb, femMuscleMaxForce, true);
					}
				}
			}

			public static void createFasciclesInBundles(FemMuscleModel fem) {
				for (MuscleBundle bundle : fem.getMuscleBundles()) {
					createFasciclesInBundle(bundle);
				}
			}
			
			public static void createFasciclesInBundle(MuscleBundle bundle) {
				LinkedList<Muscle> newFascicle = new LinkedList<Muscle>();
				Point lastPoint = null;
				
				for (Muscle m : bundle.getFibres()) {
					if (lastPoint != m.getFirstPoint()) {
						if (lastPoint != null) {
							bundle.addFascicle(newFascicle);
						}
						newFascicle = new LinkedList<Muscle>();
					}

					if (newFascicle != null) {
						newFascicle.add(m);
					}

					lastPoint = m.getSecondPoint();
				}
				bundle.addFascicle(newFascicle);
			}

			/**
			 * Returns volume of elements added to elems
			 **/
			public static double getVolumeOfSurroundingElements(MuscleBundle b, ArrayList<FemElement> surroundingElems, Muscle fiber) {
				HashSet<FemElement3d> elems0 = new HashSet<FemElement3d>();
				HashSet<FemElement3d> elems1 = new HashSet<FemElement3d>();
				double elemsVolume = 0.0;
				
				elems0.addAll(FemMiscMethods.getElements(fiber.getFirstPoint()));
				elems1.addAll(FemMiscMethods.getElements(fiber.getSecondPoint()));
				for (MuscleElementDesc desc : b.getElements()) {
					FemElement e = desc.getElement();
					if (elems0.contains(e) && elems1.contains(e)) {
						surroundingElems.add(e);
						e.computeVolumes();
						elemsVolume += e.getVolume();
					}
				}

				if (surroundingElems.size() == 0) {
					// fiber connected to non-adjacent elements,
					// therefore compute average volume of all elements
					int cnt = 0;
					for (FemElement e : elems0) {
						boolean contains = false;
						for (MuscleElementDesc desc : b.getElements()) {
							if (desc.getElement() == e) {
								contains = true;
								break;
							}
						}
						
						if (contains) {
							e.computeVolumes();
							elemsVolume += e.getVolume();
							cnt++;
						}
					}
					
					if (cnt > 0) {
						elemsVolume /= cnt;
					}
					else {
						elemsVolume = 0.0;
						System.err.println("Could not compute average volume of elements for muscle bundle '" + b.getName() + "'.");
					}
				}

				return elemsVolume;
			}


			/** Original method from HexTongueDemo.java. Call createFaciclesInBundles() if you wish to scaleByFascicle. **/
			public static void setVolumeDepMaxForce(MuscleBundle b, double bundleMaxForce, boolean scaleByFascicle) {
				double totalMaxForce = 0;
				
				//Compute the total volume of the muscle bundle and divide it by the number of fibers to approximate the volume per fiber
				double vol = 0.0;
				for (MuscleElementDesc med : b.getElements()) {
					vol += med.getElement().getRestVolume();
				}
				
				if (b.getFibres().size() > 0) {
					vol /= b.getFibres().size();
				}
				else {
					throw new IllegalArgumentException("Muscle bundle '" + b.getName() + "' has no muscle fibers. Cannot scale muscle force.");
				}
				
				for (Muscle f : b.getFibres()) {
					//ArrayList<FemElement> surroundingElems = new ArrayList<FemElement>();
					//double vol = getVolumeOfSurroundingElements(b, surroundingElems, f);

					Muscle.setMaxForce(f, bundleMaxForce*vol);
					totalMaxForce += Muscle.getMaxForce(f);
				}

				double S = 1.0;
				if (scaleByFascicle) {
					double fascicleMaxForce = getTotalFascicleForce(b);
					
					if (fascicleMaxForce <= 0.0) {
						System.out.println("Fascicle max force of bundle '" + b.getName() + "' must be greater than 0");
						//throw new IllegalArgumentException("Fascicle max force of bundle '" + b.getName() + "' must be greater than 0");
					}
					
					//System.out.println(b.getName() + " total/fascicle = " + totalMaxForce / fascicleMaxForce);
					S = bundleMaxForce / fascicleMaxForce;
				} else {
					
					if (totalMaxForce <= 0.0) {
						throw new IllegalArgumentException("Total muscle force of bundle '" + b.getName() + "' must be greater than 0");
					}
					
					S = bundleMaxForce / totalMaxForce;
				}

				
				
				for (Muscle f : b.getFibres()) {
					Muscle.setMaxForce(f, Muscle.getMaxForce(f)*S);
				}
			}
			
			


			public static void createMuscleExciters(FemMuscleModel fem, boolean fusePairedMusclesFlag) {
				if (fusePairedMusclesFlag) {
					//Search for left muscles first
					for (MuscleBundle mb : fem.getMuscleBundles()) {
						if (mb.getName().contains("left")) {
							//Locate the corresponding right sided muscle
							boolean pairFound = false;
							for (MuscleBundle pairCandidate : fem.getMuscleBundles()) {
								if (pairCandidate.getName().contains(mb.getName().replace("left", "right"))) {
									createPairedMuscleExciter(fem, mb, pairCandidate);
									pairFound = true;
									break;
								}
							}     

							if (!pairFound) {
								System.err.println("A muscle pair for '" + mb.getName() + "' of FEM '" + fem.getName() + "' could not be located.");
							}
							//If the muscle does not have left or right in its name, create an exciter for it
						} else if (!mb.getName().contains("right")) {
							createMuscleExciter(fem, mb);
						}
					}


				}
				else {
					for (MuscleBundle mb : fem.getMuscleBundles()) {
						createMuscleExciter(fem, mb);
					}
				}
			}

			public static void createMuscleExciter(FemMuscleModel fem, MuscleBundle mb) {
				MuscleExciter mex = new MuscleExciter(mb.getName());
				mex.addTarget(mb, 1.0);
				fem.addMuscleExciter(mex);
			}

			public static void createPairedMuscleExciter(FemMuscleModel fem, MuscleBundle left, MuscleBundle right) {
				MuscleExciter mex = new MuscleExciter(left.getName().replace("left", ""));
				mex.addTarget(left, 1.0);
				mex.addTarget(right, 1.0);
				fem.addMuscleExciter(mex);
			}

			public static void freeNodesOfMuscleElements(MechModel mech, FemMuscleModel fem) {
				for (MuscleBundle mb : fem.getMuscleBundles()) {
					for (Muscle m : mb.getFibres()) {
						ArrayList<FemElement3d> elements = FemMiscMethods.getElements(m.getFirstPoint());
						elements.addAll(FemMiscMethods.getElements(m.getSecondPoint()));

						for (FemElement3d element : elements) {
							for (FemNode3d node : element.getNodes()) {
								if (node.isAttached()) {
									mech.detachPoint(node);
								}
							}
						}

					}
				}
			}

			public static FemMarker addMuscleFemMarker(FemMuscleModel fem, Point3d position) {
				FemMarker point = new FemMarker(position);
				fem.addMarker(point);

				RenderProps.setPointStyleMode(point, PropertyMode.Explicit);
				RenderProps.setPointRadius(point, 0.2);
				RenderProps.setVisible(point, true);
				return point;
			}

			public static boolean checkForMuscleFiberContainment(FemMuscleModel fem, FemElement3d element) {

				for (MuscleBundle mb : fem.getMuscleBundles()) {
					for (Muscle m : mb.getFibres()) {
						if (m.getFirstPoint() instanceof FemMarker) {
							FemMarker m1 = (FemMarker) m.getFirstPoint();

							if (m1.getElement().equals(element)) {
								return true;
							}
						}
						if (m.getSecondPoint() instanceof FemMarker) {
							FemMarker m2 = (FemMarker) m.getSecondPoint();

							if (m2.getElement().equals(element)) {
								return true;
							}
						}
					}
				}
				return false;
			}

			
			
			/** Scales muscle fiber max force for the provided list of muscle bundles. If a fiber in a given bundle is lacking a material, then it will be given a 
			 * default constant axial muscle material.  
			 * @param bundles muscles
			 * @param maxForce maximum force
			 */
			public static void setVolumeDependentMaxForce(RenderableComponentList<MuscleBundle> bundles, double maxForce, boolean scaleByFascicle) {

				for (MuscleBundle bundle : bundles) {
					double totalMaxForce = 0;
					for (Muscle f : bundle.getFibres()) {
						//Set the material to constant axial muscle if necessary
						if (f.getMaterial() == null) {
							f.setMaterial(new ConstantAxialMuscle());
						}
						ArrayList<FemElement> surroundingElems = new ArrayList<FemElement>();
						double vol = FemVolumeMethods.getSurroundingElementVolume(bundle, surroundingElems, f);

						if (vol > 0.0) {
							Muscle.setMaxForce(f, maxForce*vol);
							totalMaxForce += Muscle.getMaxForce(f);
						}
						else {
							Muscle.setMaxForce(f, 0.0);
							System.err.println("Fiber " + f.getNumber() + " of muscle '" + bundle.getName() + "' has no element description. It's max force has been set to zero.");
						}
					}

					double S = 1.0;
					if (scaleByFascicle) {
						double fascicleMaxForce = getTotalFascicleForce(bundle);
						//System.out.println(b.getName() + " total/fascicle = " + totalMaxForce / fascicleMaxForce);

						if (fascicleMaxForce > 0.0) {
							S = maxForce / fascicleMaxForce;
						}
						else {
							S = 0.0;
							System.err.println("Error setting volume dependent max force for muscle '" + bundle.getName() + "': division by zero. Max muscle force has been set to 0.0");
						}

					} else {

						if (totalMaxForce > 0.0) {
							S = maxForce / totalMaxForce;
						}
						else {
							S = 0.0;
							System.err.println("Error setting volume dependent max force for muscle '" + bundle.getName() + "': division by zero. Max muscle force has been set to 0.0");
						}
					}

					for (Muscle f : bundle.getFibres()) {
						double maxForceActual = Muscle.getMaxForce(f)*S;
						Muscle.setMaxForce(f, (Double.isNaN(maxForceActual) ? 0.0 : maxForceActual));
						if (Double.isNaN(maxForceActual)) {
							System.err.println("Fiber " + f.getNumber() + " of muscle '" + bundle.getName() + "' has a max force that is NaN. It's max force has been set to zero.");
						}
					}
				}
			}


			/**A fascicle is a serial group of muscle fibers fasciles are arranged in parallel to form a muscle bundle the sum of fascile max forces should equal bundleMaxForce. **/
			public static double getTotalFascicleForce(MuscleBundle b) {
				double totalMaxFascicleForce = 0;
				if (b.getFascicles().size() > 0) {
					for (int i = 0; i < b.getFascicles().size(); i++) {
						double maxMaxForce = -1;
						for (Muscle f : b.getFascicles().get(i)) {
							if (Muscle.getMaxForce(f) > maxMaxForce) {
								maxMaxForce = Muscle.getMaxForce(f);
							}
						}
						totalMaxFascicleForce += maxMaxForce;
					}
				}
				else {
					System.err.println("Muscle '" + b.getName() + "' has no fascicles.");
				}
				return totalMaxFascicleForce;
			}


			public static void updateElementDirections(RenderableComponentList<MuscleBundle> bundles) {
				for (MuscleBundle bundle : bundles) {
					bundle.computeElementDirections();
				}
			}
		}
		public static class FemVolumeMethods {
			private double averageVolume, minVolume, maxVolume;
			private LinkedHashMap<Object, Double> volumeMap = new LinkedHashMap<Object,Double>();
			private HashMap<Object, Double> smallestVolumes = new HashMap<Object, Double>();
			private String smallestFew = "";

			public FemVolumeMethods(double averageVolume, LinkedHashMap<Object, Double> volumeMap) {
				this.averageVolume = averageVolume;
				this.volumeMap = volumeMap;

				List<Map.Entry<Object, Double>> list = new LinkedList<Map.Entry<Object, Double>>(volumeMap.entrySet());

				minVolume = list.get(0).getValue();
				maxVolume = list.get(list.size() - 1).getValue();

				smallestFew = "";
				for (int i = 0; i < Math.min(50, list.size()); i++) {
					smallestVolumes.put(list.get(i).getKey(), list.get(i).getValue());
					if (i < Math.min(10, list.size())) {
						smallestFew += list.get(i).getKey() + " (" + String.format("%.3e", list.get(i).getValue()) + " m^3)" + (i < 9 ? ", " : "");
					}
				}
			}

			public String getSmallest10() { return smallestFew; }
			public double getAverageVolume() { return averageVolume; }
			public double getMinVolume() { return minVolume; }
			public double getMaxVolume() { return maxVolume; }
			public double getVolume(int key) {
			       if (volumeMap.get(key) !=  null) {
			              return volumeMap.get(key);
			       }
			       else {
			              System.err.println("Volume map is null.");
			              return -1.0;
			       }
			}
			public boolean checkForSmallSize(int key) { return smallestVolumes.containsKey(key); }

			public static FemVolumeMethods createFemVolumeData(FemModel3d fem) {
				LinkedHashMap<Object, Double> volMap = new LinkedHashMap<Object, Double>(); // peter's edit
				double averageVolume = 0.0;

				for (FemElement3d elem : fem.getElements()) {
					double vol = computeVolume(elem);
					volMap.put(elem.getNumber(), vol);
					averageVolume += vol;
				}

				averageVolume /= fem.getElements().size();
				LinkedHashMap<Object, Double> volMapSorted = new LinkedHashMap<Object, Double>();  
				volMapSorted.putAll(AuxTools.sortByComparator(volMap));  
				return new FemVolumeMethods(averageVolume, volMapSorted);
			}



			/*
			 * Returns volume of elements added to elems
			 */
			public static double getSurroundingElementVolume(MuscleBundle b, ArrayList<FemElement> surroundingElems, Muscle fiber) {
				ArrayList<FemElement3d> elems0 = new ArrayList<FemElement3d>();
				ArrayList<FemElement3d> elems1 = new ArrayList<FemElement3d>();
				double elemsVolume = 0.0;

				elems0 = FemMiscMethods.getElements(fiber.getFirstPoint());
				elems1 = FemMiscMethods.getElements(fiber.getSecondPoint());

				if (elems0.size() > 0 && elems0.size() > 0) {
					for (FemElement3d elem : elems0) {
						elemsVolume += computeVolume((FemElement3d) elem);
					}
					for (FemElement3d elem : elems1) {
						elemsVolume += computeVolume((FemElement3d) elem);
					}
					surroundingElems.addAll(elems0);
					surroundingElems.addAll(elems1);
				}
				else {
					System.err.println("Fiber " + fiber.getNumber() + " of muscle '" + b.getName() + "' has no elements associated with it.");
				}

				return  (Double.isNaN(elemsVolume) ? 0.0 : elemsVolume);

				/*
				int containedCount = 0;
				if (b.getElements().size() > 0) {
					for (MuscleElementDesc desc : b.getElements()) {
						FemElement e = desc.getElement();
						if (elems0.contains(e) && elems1.contains(e)) {
							containedCount++;
							surroundingElems.add(e);
							elemsVolume += computeVolume((FemElement3d) e);
						}
					}
				}
				else {
					System.err.println("Muscle '" + b.getName() + "' has no element definitions");
				}

				if (surroundingElems.size() == 0) {
					// fiber connected to non-adjacent elements, therefore compute average volume of all elements
					int cnt = 0;
					for (FemElement e : elems0) {
						boolean contains = false;
						for (MuscleElementDesc desc : b.getElements()) {
							if (desc.getElement() == e) {
								containedCount++;
								contains = true;
								break;
							}
						}
						if (contains) {
							elemsVolume += computeVolume((FemElement3d) e);
							cnt++;
						}
					}
					elemsVolume /= cnt;
				}

				if (containedCount == 0) {
					System.err.println("Fiber " + fiber.getNumber() + " of muscle '" + b.getName() + "' has no elements associated with it.");
				}

				return (Double.isNaN(elemsVolume) ? 0.0 : elemsVolume);
				 */
			}

			public static double computeVolume(FemElement3d element) {
				FemNode3d[] n = element.getNodes();
				if (element instanceof HexElement) {
					return HexElement.computeVolume(n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7]);
				}
				else if (element instanceof WedgeElement) {
					return FemVolumeMethods.computeWedgeVolume(n[0], n[1], n[2], n[3], n[4], n[5]);
				}
				else if (element instanceof TetElement) {
					return TetElement.computeVolume(n[0].getPosition(), n[1].getPosition(), n[2].getPosition(), n[3].getPosition());
				}
				else {
					System.err.println("Error computing volume of element '" + element.getNumber() + "'");
					return 1.0;
				}
			}

			public static double computeWedgeVolume(FemNode3d n0, FemNode3d n1, FemNode3d n2, FemNode3d n3, FemNode3d n4, FemNode3d n5) {
				Point3d p0 = n0.getPosition();
				Point3d p1 = n1.getPosition();
				Point3d p2 = n2.getPosition();
				Point3d p3 = n3.getPosition();
				Point3d p4 = n4.getPosition();
				Point3d p5 = n5.getPosition();

				double vol = 0;
				// to compute the volume without bias, we take the average volume of two complementary three-tetrahedra tesselations
				vol += TetElement.computeVolume(p0, p1, p2, p4);
				vol += TetElement.computeVolume(p0, p3, p4, p5);
				vol += TetElement.computeVolume(p2, p4, p5, p0);

				vol += TetElement.computeVolume(p5, p4, p3, p1);
				vol += TetElement.computeVolume(p0, p3, p1, p2);
				vol += TetElement.computeVolume(p1, p5, p2, p3);

				return vol/2;
			}

		}

		public static class FemIOMethods {


			/** Saves a FEM in .CSV format (comma separated values). Nodes are always written before elements, which guaranetees a single read through the file 
			 * such that nodes can be created first and then populated into elements. 
			 * Zero based numbering is used. 
			 * The schema for a node line is: 'node', node_id, x, y, z
			 * The schema for an elements line is: 'element', element_id, number_of_nodes, node_ids...
			 */
			public static void writeFemToCSV(FemMuscleModel fem, String fileName) {
				try {
					BufferedWriter bw = new BufferedWriter(new FileWriter(new File(fileName), false));

					//Write the node data (excluding nodes lacking dependencies; such nodes will be added by muscles if necessary)
					for (FemNode3d node : fem.getNodes()) {
						if (node.getElementDependencies().size() > 0) {
							Point3d pos = node.getPosition();
							bw.write("node, " + node.getNumber() + ", " + pos.x + ", " + pos.y + ", " + pos.z + "\n");
						}
					}

					//Write the node data
					for (FemElement3d elem : fem.getElements()) {
						String elemData = "element, " + elem.getNumber() + ", " + elem.getNodes().length + ", ";
						for (FemNode3d node : elem.getNodes()) {
							elemData += node.getNumber() + ", ";
						}
						bw.write(elemData + "\n");
					}

					bw.close();
					System.out.println("FEM '" + fem.getName() + "' saved to " + fileName);
				} catch (IOException e) {
					System.out.println("An error occurred writing to file " + fileName + "; if the file is open, close it and try again.");
				} catch (NullPointerException e) {
					System.out.println("An unknown error occurred when attempting to save FEM data.");
				}

			}

			public static FemMuscleModel readFemFromCSV(String name, String fileName) {
				FemMuscleModel fem = new FemMuscleModel(name);
				ArrayList<String[]> rawTextLines = AuxTools.readDataFromCSV(fileName);

				//Read the node data and create FEM nodes for each entry                     
				for (String[] data : rawTextLines) {
					if (data[0].equals("node")) {
						int nodeID = Integer.valueOf(data[1].trim());
						double x = Double.valueOf(data[2].trim());
						double y = Double.valueOf(data[3].trim());
						double z = Double.valueOf(data[4].trim());

						FemNode3d newNode = new FemNode3d(x, y, z);             
						fem.addNode(newNode);   
					}
					else if (data[0].equals("element")) {
						int elementID = Integer.valueOf(data[1].trim());
						int numberOfNodes = Integer.valueOf(data[2].trim());

						FemNode3d[] nodes = new FemNode3d[numberOfNodes];
						for (int i = 0; i < numberOfNodes; i++) {
							nodes[i] = fem.getByNumber(Integer.valueOf(data[3 + i]));
						}

						FemElement3d elem = null;
						if (numberOfNodes == 4) {
							elem = new TetElement(nodes[0], nodes[1], nodes[2], nodes[3]);
						}
						else if (numberOfNodes == 6) {
							elem = new WedgeElement(nodes);
						}
						else if (numberOfNodes == 8) {
							elem = new HexElement(nodes);
						}
						else {
							System.err.println("Attempted to read FEM from file " + fileName + " but it contains an element with " + numberOfNodes + " nodes.");
						}

						try {
							elem.setNumber(elementID);
							fem.addElement(elem);
						} catch (Exception ex) {
							System.err.println("Failed to add element number " + elem.getNumber() + " of FEM '" + fem.getName() + "'.");
							ex.printStackTrace();
						}
					}
				}

				//Set any free nodes to non-dynamic
				for (FemNode3d node : fem.getNodes()) {
					if (node.getElementDependencies().size() == 0) {
						node.setDynamic(false);
						//System.out.println("Node " + node.getNumber() + " in FEM '" + fem.getName() + "' has been set to non-dynamic.");
					}
				}
				
				return fem;   
			}

			/** Reads in node and element data from a .MSH file (produced by GMSH). A node line contains the node number followed by its coordinates.
			 * An element line contains the element number followed by its type, number of tags, the tags (used in GMSH), and finally the node number list
			 * @param name name of model
			 * @param fileName file to read model from
			 * @return created model
			 */
			public static FemMuscleModel readFemFromMSH(String name, String fileName) {
				FemMuscleModel fem = new FemMuscleModel(name);
				ArrayList<String> rawTextLines = new ArrayList<String>();

				try {
					//Create a text file reader and parse the imported file for numeric data
					String line = "";
					BufferedReader br;

					br = new BufferedReader(new FileReader(fileName + ".msh"));

					while ((line = br.readLine()) != null) {
						rawTextLines.add(line.trim());
					}

					br.close();
				} catch (FileNotFoundException e) {
					e.printStackTrace();
				} catch (IOException e) {
					e.printStackTrace();
				}

				try {
					//Locate the nodes description (designated by the token "$Nodes")
					int numberOfNodesIndex = 0;
					int firstNodeDefinitionIndex = 0;
					int numberOfElementsIndex = 0;
					int firstElementDefinitionIndex = 0;
					int count = 0;
					for (String s : rawTextLines) {
						if (s.equals("$Nodes")) {
							numberOfNodesIndex = count + 1;
							firstNodeDefinitionIndex = count + 2;
						}

						if (s.equals("$Elements")) {
							numberOfElementsIndex = count + 1;
							firstElementDefinitionIndex = count + 2;
						}
						count++;
					}

					//Create an array of FEM nodes sized according to the data stored after the "$Nodes" token, which indicates the total number of nodes
					int numNodes = Integer.valueOf(rawTextLines.get(numberOfNodesIndex));
					int zeroNumbering = -1;
					for (int i = firstNodeDefinitionIndex; i < firstNodeDefinitionIndex + numNodes; i++) {
						String[] nodeDefinitionLine = rawTextLines.get(i).split(" ");
						int nodeNumber = Integer.valueOf(nodeDefinitionLine[0].trim());
						double x = Double.valueOf(nodeDefinitionLine[1].trim());
						double y = Double.valueOf(nodeDefinitionLine[2].trim());
						double z = Double.valueOf(nodeDefinitionLine[3].trim());

						FemNode3d newNode = new FemNode3d(x, y, z);
						newNode.setNumber(nodeNumber + zeroNumbering);                 //ArtiSynth uses '0' based numbering
						fem.addNode(newNode);
					}

					//Create an array of FEM elements based on the node data and the element structure data stored after the "$Elements" token in the mesh file
					int numElems = Integer.valueOf(rawTextLines.get(numberOfElementsIndex));
					int elementTypeIndex = 1;
					String elementType = "4";
					int elemNumber = 1;

					for (int i = firstElementDefinitionIndex; i < firstElementDefinitionIndex + numElems; i++) {
						//Skip to the volume elements

						String[] elementDefinitionLine = rawTextLines.get(i).split(" ");

						if (elementDefinitionLine[elementTypeIndex].equals(elementType)) {

							FemNode3d n1 = fem.getByNumber(Integer.valueOf(elementDefinitionLine[5].trim()) + zeroNumbering);
							FemNode3d n2 = fem.getByNumber(Integer.valueOf(elementDefinitionLine[6].trim()) + zeroNumbering);
							FemNode3d n3 = fem.getByNumber(Integer.valueOf(elementDefinitionLine[7].trim()) + zeroNumbering);
							FemNode3d n4 = fem.getByNumber(Integer.valueOf(elementDefinitionLine[8].trim()) + zeroNumbering);

							FemElement3d newElement = new TetElement(n1, n2, n3, n4);
							newElement.setNumber(elemNumber++);
							fem.addElement(newElement);
						}
					}

					//Define a new FEM model based on the node and element data


				}
				catch (Exception ex) {
					System.out.println("An error occurred when attempting to create a FEM from file " + fileName);
					ex.printStackTrace();

				}

				return fem;          
			}
			

			public static void readMusclesFromCSV(FemMuscleModel fem, String fileName) {
				PolygonalMesh mesh = fem.getSurfaceMesh();
				ArrayList<String[]> muscleData = AuxTools.readDataFromCSV(fileName);
				String muscleName = "";
				MuscleBundle newBundle = new MuscleBundle();
				LinkedList<Muscle> fascicle = new LinkedList<Muscle>();
				BVFeatureQuery query = new BVFeatureQuery();
				ArrayList<FemMarker> newFemMarkers = new ArrayList<FemMarker>();
				
				for (String[] fiberDescription : muscleData) {
					String muscleDataType = fiberDescription[0];

					if (muscleDataType.contains("fiber")) {
						String currentName = fiberDescription[1];

						if (!muscleName.equals(currentName)) {
							muscleName = currentName;
							newBundle = new MuscleBundle(currentName);
							newBundle.setFibresActive(true);

							fascicle = new LinkedList<Muscle>();
							newBundle.addFascicle(fascicle);
							fem.addMuscleBundle(newBundle);

							RenderProps.setVisible(newBundle, true);
							RenderProps.setLineColor(newBundle, defaultFemMuscleColor);
						}

						
						//bw.write("fiber_marker_node, " + bundle.getName() + ", " + elem1 + ", " + p1.x + ", " + p1.y + ", " + p1.z + ", " + i2 + "\n");
						//bw.write("fiber_node_marker, " + bundle.getName() + ", " + i1 + ", " + elem2 + ", " + p2.x + ", " + p2.y + ", " + p2.z + "\n");
						
						Muscle m = new Muscle();
						if (muscleDataType.contains("marker_node")) {
							int fem1 = Integer.valueOf(fiberDescription[2]);
							
							double p1x = Double.valueOf(fiberDescription[4]);
							double p1y = Double.valueOf(fiberDescription[5]);
							double p1z = Double.valueOf(fiberDescription[6]);

							double p2x = Double.valueOf(fiberDescription[7]);
							double p2y = Double.valueOf(fiberDescription[8]);
							double p2z = Double.valueOf(fiberDescription[9]);
							
							Point m1 = null;
							Point m2 = null;
							
							Point3d p1 = new Point3d(p1x, p1y, p1z);
							Point3d p2 = new Point3d(p2x, p2y, p2z);
							
							m1 = new FemMarker(p1);
							fem.addMarker((FemMarker) m1, fem.getElementByNumber(fem1));
							
							m2 = new FemNode3d(p2);
							((FemNode3d) m2).setDynamic(false);
							fem.addNode((FemNode3d) m2);
							
							m.setFirstPoint(m1);
							m.setSecondPoint(m2);
						}
						else if (muscleDataType.contains("node_marker")) {
							int fem2 = Integer.valueOf(fiberDescription[3]);

							double p1x = Double.valueOf(fiberDescription[4]);
							double p1y = Double.valueOf(fiberDescription[5]);
							double p1z = Double.valueOf(fiberDescription[6]);

							double p2x = Double.valueOf(fiberDescription[7]);
							double p2y = Double.valueOf(fiberDescription[8]);
							double p2z = Double.valueOf(fiberDescription[9]);
							
							Point m1 = null;
							Point m2 = null;
							
							Point3d p1 = new Point3d(p1x, p1y, p1z);
							Point3d p2 = new Point3d(p2x, p2y, p2z);
							
							m1 = new FemNode3d(p1);
							((FemNode3d) m1).setDynamic(false);
							fem.addNode((FemNode3d) m1);
							
							m2 = new FemMarker(p2);
							fem.addMarker((FemMarker) m2, fem.getElementByNumber(fem2));
							
							m.setFirstPoint(m1);
							m.setSecondPoint(m2);
						}
						else if (muscleDataType.contains("marker")) {
							int fem1 = Integer.valueOf(fiberDescription[2]);
							int fem2 = Integer.valueOf(fiberDescription[3]);

							double p1x = Double.valueOf(fiberDescription[4]);
							double p1y = Double.valueOf(fiberDescription[5]);
							double p1z = Double.valueOf(fiberDescription[6]);

							double p2x = Double.valueOf(fiberDescription[7]);
							double p2y = Double.valueOf(fiberDescription[8]);
							double p2z = Double.valueOf(fiberDescription[9]);
							
							Point m1 = null;
							Point m2 = null;
							
							Point3d p1 = new Point3d(p1x, p1y, p1z);
							Point3d p2 = new Point3d(p2x, p2y, p2z);

							m1 = new FemMarker(p1);
							fem.addMarker((FemMarker) m1, fem.getElementByNumber(fem1));


							m2 = new FemMarker(p2);
							fem.addMarker((FemMarker) m2, fem.getElementByNumber(fem2));

							m.setFirstPoint(m1);
							m.setSecondPoint(m2);
							//System.out.println(currentName + " " + m1.getElement().getNumber() + " = " + fem1 + " ... " + m2.getElement().getNumber() + " = " + fem2);       
						} 
						else if (muscleDataType.contains("node")) {
							int n1 = Integer.valueOf(fiberDescription[2]);
							int n2 = Integer.valueOf(fiberDescription[3]);

							m.setFirstPoint(fem.getByNumber(n1));
							m.setSecondPoint(fem.getByNumber(n2));
						}


						newBundle.addFibre(m);
						fascicle.add(m);
					}
					else if (muscleDataType.equals("element")) {
						String currentName = fiberDescription[1];

						if (!muscleName.equals(currentName)) {
							muscleName = currentName;
							newBundle = new MuscleBundle(currentName);
							fem.addMuscleBundle(newBundle);

							RenderProps.setVisible(newBundle, true);
							//RenderProps.setLineColor(newBundle, muscleColors[newBundle.getNumber()]);
						}

						MuscleElementDesc med = new MuscleElementDesc();
						int elementNumber = Integer.valueOf(fiberDescription[2]);      
						med.setElement(fem.getElementByNumber(elementNumber));
						med.setDirection(Vector3d.X_UNIT);
						newBundle.addElement(med);
						RenderProps.setVisible(med, true);
					}
				}

				//Now compute the element directions to align with muscle fibers (calls a Tetgen Tesselator)
				for (MuscleBundle mb : fem.getMuscleBundles()) {
					mb.computeElementDirections();
					if (mb.getElements().size() == 0) {
						System.err.println("Muscle '" + mb.getName() + "' of FEM '" + fem.getName() + "' contains no elements.");
					}
				}
			}

			

			public static void readMusclesFromCSV_new(FemMuscleModel fem, String fileName) {
				ArrayList<String[]> muscleData = AuxTools.readDataFromCSV(fileName);
				String muscleName = "";

				MuscleBundle newBundle = new MuscleBundle();
				LinkedList<Muscle> fascicle = new LinkedList<Muscle>();

				HashMap<Integer, FemMarker> markerNumbers = new HashMap<Integer, FemMarker>();
				HashMap<Integer, FemNode3d> nodeNumbers = new HashMap<Integer, FemNode3d>();
				
				for (String[] fiberDescription : muscleData) {
					String dataType = fiberDescription[0];

					if (dataType.equals("fem_marker") || dataType.equals("fem_node")) {
						int number = Integer.valueOf(fiberDescription[1]);
						
						double p1x = Double.valueOf(fiberDescription[2]);
						double p1y = Double.valueOf(fiberDescription[3]);
						double p1z = Double.valueOf(fiberDescription[4]);

						Point3d pos = new Point3d(p1x, p1y, p1z);
						
						if (dataType.equals("fem_marker")) {
							FemMarker marker = new FemMarker(pos);
							fem.addMarker(marker);
							markerNumbers.put(number, marker);
						}
						else {
							FemNode3d node = new FemNode3d(pos);
							node.setDynamic(false);
							fem.addNode(node);
							nodeNumbers.put(number, node);
						}
					}
					else {
						if (!muscleName.equals(dataType)) {
							muscleName = dataType;
							newBundle = new MuscleBundle(dataType);
							newBundle.setFibresActive(true);

							fascicle = new LinkedList<Muscle>();
							newBundle.addFascicle(fascicle);
							fem.addMuscleBundle(newBundle);

							RenderProps.setVisible(newBundle, true);
							RenderProps.setLineColor(newBundle, defaultFemMuscleColor);
						}
						
						if (fiberDescription[1].equals("element")) {
							MuscleElementDesc med = new MuscleElementDesc();
							int elementNumber = Integer.valueOf(fiberDescription[2]);      
							med.setElement(fem.getElementByNumber(elementNumber));
							med.setDirection(Vector3d.X_UNIT);
							newBundle.addElement(med);
							RenderProps.setVisible(med, true);
						}
						else {
							//First point
							Point p1 = null;
							int num = Integer.valueOf(fiberDescription[3]);
							p1 = (fiberDescription[2].equals("fem_marker") ? markerNumbers.get(num) : nodeNumbers.get(num));
							
							//Second point
							Point p2 = null;
							num = Integer.valueOf(fiberDescription[5]);
							p2 = (fiberDescription[4].equals("fem_marker") ? markerNumbers.get(num) : nodeNumbers.get(num));
							
							Muscle m = new Muscle(p1, p2);
							newBundle.addFibre(m);
							fascicle.add(m);
						}
					}
				}
			}

			
			public static void writeMusclesToCSV(FemMuscleModel fem, String fileName) {
				double tol = 1e-16;
				try {
					File muscleDataFile = new File(fileName);
					BufferedWriter bw = new BufferedWriter(new FileWriter(muscleDataFile, false));
					
					//Collect all frame markers and nodes that are required
					ArrayList<FemMarker> femMarkers = new ArrayList<FemMarker>();
					ArrayList<FemNode3d> femNodes = new ArrayList<FemNode3d>();
					for (MuscleBundle mb : fem.getMuscleBundles()) {
						HashMap<Muscle, Point[]> fiberPointMap = new HashMap<Muscle, Point[]>();
						
						for (Muscle m : mb.getFibres()) {
							
							Point p1 = m.getFirstPoint();
							Point p2 = m.getSecondPoint();
							
							fiberPointMap.put(m, new Point[]{p1, p2});
							
							if (p1 instanceof FemMarker) {
								femMarkers.add((FemMarker) p1);
							}
							else {
								femNodes.add((FemNode3d) p1);
							}
							
							if (p2 instanceof FemMarker) {
								femMarkers.add((FemMarker) p2);
							}
							else {
								femNodes.add((FemNode3d) p2);
							}
						}
					}
					
					//Identify markers and nodes to keep
					ArrayList<FemMarker> femMarkersKeep = new ArrayList<FemMarker>();
					HashMap<FemMarker, FemMarker> equivalentMarkers = new HashMap<FemMarker, FemMarker>();
					ArrayList<FemNode3d> femNodesKeep = new ArrayList<FemNode3d>();
					HashMap<FemNode3d, FemNode3d> equivalentNodes = new HashMap<FemNode3d, FemNode3d>();
					
					if (femMarkers.size() > 0) {
						femMarkersKeep.add(femMarkers.get(0));
						for (FemMarker fm : femMarkers) {
							boolean addFlag = true;
							for (FemMarker fmKeep : femMarkersKeep) {
								double dist = fm.getPosition().distance(fmKeep.getPosition());

								if (dist < tol) {
									addFlag = false;
									equivalentMarkers.put(fm, fmKeep);
									break;
								}
							}

							if (addFlag) {
								equivalentMarkers.put(fm, fm);
								femMarkersKeep.add(fm);
							}
						}
					}

					if (femNodes.size() > 0) {
						femNodesKeep.add(femNodes.get(0));
						for (FemNode3d fn : femNodes) {
							boolean addFlag = true;
							for (FemNode3d fnKeep : femNodesKeep) {
								double dist = fn.getPosition().distance(fnKeep.getPosition());

								if (dist < tol) {
									addFlag = false;
									equivalentNodes.put(fn, fnKeep);
									break;
								}
							}

							if (addFlag) {
								equivalentNodes.put(fn, fn);
								femNodesKeep.add(fn);
							}
						}
					}					
					
					//Write fem markers and nodes
					int count = 1;
					HashMap<FemMarker, Integer> markerNumbers = new HashMap<FemMarker, Integer>();
					for (FemMarker fm : femMarkersKeep) {
						Point3d loc = fm.getPosition();
						String muscleData = "fem_marker, " + count + ", " + loc.x + ", " + loc.y + ", " + loc.z  + "\n";
						markerNumbers.put(fm, count);
						count += 1;
						bw.write(muscleData);
					}
					
					count = 1;
					HashMap<FemNode3d, Integer> nodeNumbers = new HashMap<FemNode3d, Integer>();
					for (FemNode3d fn : femNodesKeep) {
						Point3d loc = fn.getPosition();
						String muscleData = "fem_node, " + count + ", " + loc.x + ", " + loc.y + ", " + loc.z  + "\n";
						nodeNumbers.put(fn, count);
						count += 1;
						bw.write(muscleData);
					}
					
					//Write bundle data referring to the markers and nodes above
					for (MuscleBundle bundle : fem.getMuscleBundles()) {
						for (MuscleElementDesc med : bundle.getElements()) {
							String muscleData = bundle.getName() + ", element" + ", " + med.getElement().getNumber() + "\n";
							bw.write(muscleData);
						}

						for (Muscle m : bundle.getFibres()) {
							Point p1 = m.getFirstPoint();
							Point p2 = m.getSecondPoint();
							
							String p1Type = "";
							String p2Type = "";
							
							int p1Number = -1;
							int p2Number = -1;
							
							if (p1 instanceof FemMarker) {
								p1Type = "fem_marker";
								p1Number = markerNumbers.get(equivalentMarkers.get((FemMarker) p1));
							}
							else {
								p1Type = "fem_node";
								p1Number = nodeNumbers.get(equivalentNodes.get((FemNode3d) p1));
							}

							if (p2 instanceof FemMarker) {
								p2Type = "fem_marker";
								p2Number = markerNumbers.get(equivalentMarkers.get((FemMarker) p2));
							}
							else {
								p2Type = "fem_node";
								p2Number = nodeNumbers.get(equivalentNodes.get((FemNode3d) p2));
							}
							
							String muscleData = bundle.getName() + ", fiber" + ", " + p1Type + ", " + p1Number + ", " + p2Type + ", " + p2Number + ", " + "\n";
							bw.write(muscleData);
						}
					}

					bw.close();
					System.out.println("Muscles of FEM '" + fem.getName() + "' saved to " + fileName);
				} catch (IOException e) {
				} catch (NullPointerException e) {
				}
			}
			
			/*
			public static void writeMusclesToCSV(FemMuscleModel fem, String fileName) {
				try {
					File muscleDataFile = new File(fileName);
					BufferedWriter bw = new BufferedWriter(new FileWriter(muscleDataFile, false));

					for (MuscleBundle bundle : fem.getMuscleBundles()) {
						for (MuscleElementDesc med : bundle.getElements()) {
							String muscleData = "element, " + bundle.getName() + ", " + med.getElement().getNumber() + "\n";
							bw.write(muscleData);
						}

						for (Muscle m : bundle.getFibres()) {
							if (m.getFirstPoint() instanceof FemNode3d && m.getSecondPoint() instanceof FemNode3d) {
								int i1 = ((FemNode3d) m.getFirstPoint()).getNumber();
								int i2 = ((FemNode3d) m.getSecondPoint()).getNumber();
								bw.write("fiber_node, " + bundle.getName() + ", " + i1 + ", " + i2 + "\n");
							}
							else if (m.getFirstPoint() instanceof FemMarker && m.getSecondPoint() instanceof FemMarker) {
								int elem1 = ((FemMarker) m.getFirstPoint()).getElement().getNumber();
								int elem2 = ((FemMarker) m.getSecondPoint()).getElement().getNumber();
								Point3d p1 = ((FemMarker) m.getFirstPoint()).getPosition();
								Point3d p2 = ((FemMarker) m.getSecondPoint()).getPosition();
								bw.write("fiber_marker, " + bundle.getName() + ", " + elem1 + ", " + elem2 + ", " + p1.x + ", " + p1.y + ", " + p1.z + ", " + p2.x + ", " + p2.y + ", " + p2.z + "\n");
							}
							else if (m.getFirstPoint() instanceof FemMarker && m.getSecondPoint() instanceof FemNode3d) {
								int elem1 = ((FemMarker) m.getFirstPoint()).getElement().getNumber();
								Point3d p1 = ((FemMarker) m.getFirstPoint()).getPosition();
								Point3d p2 = ((FemNode3d) m.getSecondPoint()).getPosition();
								bw.write("fiber_marker_node, " + bundle.getName() + ", " + elem1 + ", new_node, " + p1.x + ", " + p1.y + ", " + p1.z + ", " + p2.x + ", " + p2.y + ", " + p2.z + "\n");
							}
							else if (m.getFirstPoint() instanceof FemNode3d && m.getSecondPoint() instanceof FemMarker) {
								Point3d p1 = ((FemNode3d) m.getFirstPoint()).getPosition();
								Point3d p2 = ((FemMarker) m.getSecondPoint()).getPosition();
								int elem2 = ((FemMarker) m.getSecondPoint()).getElement().getNumber();
								bw.write("fiber_node_marker, " + bundle.getName() + ", new_node, " + elem2 + ", " + p1.x + ", " + p1.y + ", " + p1.z + ", " + p2.x + ", " + p2.y + ", " + p2.z + "\n");
							}
						}
					}

					bw.close();
					System.out.println("Muscles of FEM '" + fem.getName() + "' saved to " + fileName);
				} catch (IOException e) {
				} catch (NullPointerException e) {
				}
			}
*/
			
			
			public static void readMusclesFromANSYS(SoftBody sb, String fileName, AnsysReadMethod readMethod, boolean createRightSideMusclesFlag) {
				FemMuscleModel fem = sb.getFem();

				try {
					switch (readMethod) {
					case READ_ELEMENTS:
						AnsysMuscleElemReader.read(fem, new FileReader(fileName));
						break;
					case READ_FIBERS:
						AnsysMuscleFiberReader.read(fem, new FileReader(fileName)); 
						break;
					case READ_NODES:
						LinkedHashMap<Integer,Integer> nodeIdToIndex = new LinkedHashMap<Integer,Integer>();
						LinkedHashMap<Integer, Point3d> nodeMap = AnsysReader.readNodeFile(new FileReader(fileName + ".node"), true);

						for (int markerId : nodeMap.keySet()) {
							Point3d pos = nodeMap.get(markerId);
							FemMarker marker = fem.addNumberedMarker(pos, markerId);

							nodeIdToIndex.put(markerId, marker.getNumber());
						}

						AnsysFaceMuscleFiberReader.read(fem, nodeIdToIndex, new FileReader(fileName + ".mac"));
						break;
					default:
						break;

					}      
				}
				catch (IOException e) {
					e.printStackTrace();
				}                    

				if (createRightSideMusclesFlag) {
					ArrayList<MuscleBundle> bundlesToAdd = new ArrayList<MuscleBundle>();
					//Create right sided muscles
					Plane midsagittalPlane = new Plane(Vector3d.Y_UNIT, 0);
					for (MuscleBundle leftBundle : fem.getMuscleBundles()) {
						leftBundle.setName("left ");
						ArrayList<Muscle> fibersToAdd = new ArrayList<Muscle>();

						for (Muscle m : leftBundle.getFibres()) {
							Point p1 = FemMiscMethods.createRightSideMarker(fem, m.getFirstPoint(), midsagittalPlane);
							Point p2 = FemMiscMethods.createRightSideMarker(fem, m.getSecondPoint(), midsagittalPlane);

							if (p1 != null && p2 != null) {
								Muscle rightMuscle = (Muscle) m.copy(0, null);
								rightMuscle.setFirstPoint(p1);
								rightMuscle.setSecondPoint(p2);
								fibersToAdd.add(rightMuscle);
							}
						}

						MuscleBundle rightBundle = new MuscleBundle("right ");
						for (Muscle m : fibersToAdd) {
							rightBundle.addFibre(m);
						}
						bundlesToAdd.add(rightBundle);

						leftBundle.setName("left " + sb.getMuscleName(leftBundle.getNumber()));
						rightBundle.setName("right " + sb.getMuscleName(leftBundle.getNumber()));

						RenderProps.setVisible(leftBundle, true);
						RenderProps.setVisible(rightBundle, true);

						RenderProps.setLineColor(leftBundle, defaultFemMuscleColor);
						RenderProps.setLineColor(rightBundle, defaultFemMuscleColor);
					}

					//Add the right side bundles now
					for (MuscleBundle mb : bundlesToAdd) {
						fem.addMuscleBundle(mb);
					}

				}
				else {
					//Set muscle names and render properties
					for (MuscleBundle bundle : fem.getMuscleBundles()) {
						bundle.setName(sb.getMuscleName(bundle.getNumber()));

						RenderProps.setVisible(bundle, true);
						RenderProps.setLineColor(bundle, defaultFemMuscleColor);
					}
				}
			}




			public static void readAttachmentsFromCSV(MechModel mech, FemMuscleModel fem, String fileName) {
				ArrayList<String[]> attachmentData = AuxTools.readDataFromCSV(fileName);
				boolean resaveFlag = false;
				for (String[] attachmentDescription : attachmentData) {

					//TODO Temporary condition to accommodate different attachment types (nodes and muscles)
					if (attachmentDescription.length == 2) {
						String attachmentBody = attachmentDescription[0];

						AbstractBody ab = Structures.getBodyByName(attachmentBody);
						FemNode3d node = fem.getByNumber(Integer.valueOf(attachmentDescription[1]));

						if (ab instanceof HardBody) {
							mech.attachPoint(node, ((HardBody) ab).getBody());
						}

						RenderProps.setPointColor(node, Color.red);	
						if (resaveFlag) {
							FemIOMethods.writeAttachmentsToCSV(fem, fileName);
							resaveFlag = false;
						}
					}
					else {
						String attachmentType = attachmentDescription[0];

						if (attachmentType.contains("node")) {
							FemNode3d node = fem.getByNumber(Integer.valueOf(attachmentDescription[1]));
							String attachmentBody = attachmentDescription[2];
							AbstractBody ab = Structures.getBodyByName(attachmentBody);

							if (ab instanceof HardBody) {
								mech.attachPoint(node, ((HardBody) ab).getBody());
							}
							else if (ab instanceof SoftBody) {
								mech.attachPoint(node, ((SoftBody) ab).getFem());
							}

							RenderProps.setPointColor(node, Color.red);
						}
						else if (attachmentType.contains("muscle")) {

						}
					}

				}
			}

			public static void readColorFromCSV(MechModel mech, FemMuscleModel fem, String fileName) {
				ArrayList<String[]> colorData = AuxTools.readDataFromCSV(fileName);

				for (String[] colorDescription : colorData) {

					FemElement3d element = fem.getElementByNumber(Integer.valueOf(colorDescription[1]));
					float r = (float) Integer.valueOf(colorDescription[2]) / 255f;
					float g = (float) Integer.valueOf(colorDescription[3]) / 255f;
					float b = (float) Integer.valueOf(colorDescription[4]) / 255f;
					RenderProps.setFaceColor(element,  new Color(r, g, b));
					RenderProps.setFaceColor(element,  new Color(r, g, b));
				}
			}


			public static void writeAttachmentsToCSV(FemMuscleModel fem, String fileName) {
				try {
					File muscleDataFile = new File(fileName);
					BufferedWriter bw = new BufferedWriter(new FileWriter(muscleDataFile, false));
					for (FemNode3d node : fem.getNodes()) {
						if (node.isAttached()) {
							String attachmentName = "";
							if (node.getAttachment() instanceof PointFem3dAttachment) {
								attachmentName = node.getAttachment().getMasters()[0].getParent().getParent().getName();
							}
							else {
								attachmentName = node.getAttachment().getMasters()[0].getName();
							}
							bw.write("node, " + node.getNumber() + ", " + attachmentName + "\n");
						}
					}
					bw.close();
					System.out.println("Attachments of FEM '" + fem.getName() + "' saved to " + fileName);
				} catch (IOException e) {
					System.out.println("An error occurred writing to file " + fileName + "; if the file is open, close it and try again.");
				} catch (NullPointerException e) {
					System.out.println("An unknown error occurred when attempting to save FEM attachment data.");
				}
			}

			public static void writeElementColorsToCSV(FemMuscleModel fem, String fileName) {
				try {
					File elementColorFile = new File(fileName);
					BufferedWriter bw = new BufferedWriter(new FileWriter(elementColorFile, false));
					for (FemElement3d element : fem.getElements()) {
						String colorData = "element, " + element.getNumber() + ", ";
						if (element.getRenderProps() != null) {
							Color color = element.getRenderProps().getFaceColor();
							colorData += color.getRed() + ", " + color.getGreen() + ", " + color.getBlue();
						}
						else {
							colorData += defaultElementColor.getRed() + ", " +  defaultElementColor.getGreen() + ", " +  defaultElementColor.getBlue();

						}
						bw.write(colorData + "\n");
					}
					bw.close();
					System.out.println("Color pattern of FEM '" + fem.getName() + "' saved to " + fileName);
				} catch (IOException e) {
					System.out.println("An error occurred writing to file " + fileName + "; if the file is open, close it and try again.");
				} catch (NullPointerException e) {
					System.out.println("An unknown error occurred when attempting to save FEM color data.");
				}
			}


			public static void readCollisionSurfacesFromOBJ(SoftBody sb, VocalTractType vtModel) {
				File f = new File(vtModel.getModelDirectory());
				String[] files = f.list();

				for (String file : files) {
					if (file.startsWith(sb.getName()) && file.contains(Names.Files.COLLISION_SUBSURFACE.getKeyword())) {
						try {
							PolygonalMesh collisionSubsurface = FemMiscMethods.createSubsurface(sb.getFem(), file.replace(".obj", ""), new PolygonalMesh(new File(vtModel.getFullFileName(file))));
							sb.addCollisionSubsurface(collisionSubsurface);
							sb.getFem().addMesh(collisionSubsurface);
						}
						catch (IOException e) {
							e.printStackTrace();
						}
					}
				}

			}

			public static void writeAllFemDataToCSV(FemMuscleModel fem, String filePath, String fileNamePrefix) {
				String fileName = fem.getName();
				writeAllFemDataToCSV(fem, filePath, fileNamePrefix, fileName);
			}

			public static void writeAllFemDataToCSV(FemMuscleModel fem, String filePath, String fileNamePrefix, String fileName) {
				if (!filePath.endsWith("\\")) {
					filePath += "\\";
				}

				FemIOMethods.writeFemToCSV(fem, filePath + fileNamePrefix + fileName + Names.Files.FEM.getSuffix());
				FemIOMethods.writeMusclesToCSV(fem, filePath + fileNamePrefix + fileName + Names.Files.FEM_MUSCLES.getSuffix());
				FemIOMethods.writeAttachmentsToCSV(fem, filePath + fileNamePrefix + fileName + Names.Files.ATTACHMENTS.getSuffix());
				FemIOMethods.writeElementColorsToCSV(fem, filePath + fileNamePrefix + fileName + Names.Files.COLORS.getSuffix());
				AuxTools.saveMeshToOBJ(filePath + fileNamePrefix + fileName + Names.Files.SURFACE_MESH.getSuffix(), fem.getName(), fem.getSurfaceMesh());

				//Save collision subsurfaces if they exist
				int surfaceNumber = 1;
				for (FemMeshComp subsurface : fem.getMeshComps()) {
					if (!subsurface.getName().equals("surface")) {
						int idx = subsurface.getName().lastIndexOf("_");
						String subsurfaceName = subsurface.getName().substring(idx + 1, subsurface.getName().length());
						AuxTools.saveMeshToOBJ(filePath + fileNamePrefix + fileName + Names.Files.COLLISION_SUBSURFACE.getNumberedNamedSuffix(surfaceNumber++, subsurfaceName), fem.getName() + " collision subsurface " + subsurfaceName, (PolygonalMesh) subsurface.getMesh());
					}
				}
			}


			public static void writeFemDuplicateToCSV(String filePath, String fileNamePrefix, FemMuscleModel fem) {
				filePath += fileNamePrefix;
				FemMuscleModel renumberedFem = FemMiscMethods.createRenumberedFem(fem);

				//Save all soft bodies to CSV files and save all FEM muscles to "[fem_name]_muscles.csv" files
				FemIOMethods.writeFemToCSV(renumberedFem, filePath + fem.getName() + Names.Files.FEM.getSuffix());
				FemIOMethods.writeMusclesToCSV(renumberedFem, filePath + fem.getName() + Names.Files.FEM_MUSCLES.getSuffix());
				FemIOMethods.writeAttachmentsToCSV(renumberedFem, filePath + fem.getName() + Names.Files.ATTACHMENTS.getSuffix());
				FemIOMethods.writeElementColorsToCSV(renumberedFem, filePath + fem.getName() + Names.Files.COLORS.getSuffix());
				AuxTools.saveMeshToOBJ(filePath + fem.getName() + Names.Files.SURFACE_MESH.getSuffix(), fem.getName(), renumberedFem.getSurfaceMesh());

				//Save collision subsurfaces if they exist
				int surfaceNumber = 1;
				for (int i = 0; i < fem.getMeshComps().size(); i++) {
					FemMeshComp femMesh = fem.getMeshComps().get(i);
					if (!femMesh.getName().equals("surface")) {
						AuxTools.saveMeshToOBJ(filePath + fem.getName() + Names.Files.COLLISION_SUBSURFACE.getNumberedNamedSuffix(surfaceNumber++, femMesh.getName()), fem.getName() + " collision subsurface", (PolygonalMesh) femMesh.getMesh());
					}
				}
			}
		}

		public static Color defaultNodeColor = new Color(1f, 0.7f, 0.7f);
		public static Color defaultAttachedNodeColor = new Color(1f, 0f, 0f);
		public static Color defaultSurfaceElementColor = new Color(0.3f, 0.3f, 1f);
		public static Color defaultElementColor = new Color(0.9f, 0.8f, 0.75f);
		public static Color defaultFemMuscleColor = new Color(1f, 0f, 0f);

	}

	public FemMuscleModel getFem() { return fem; }
	public void setFem(FemMuscleModel fem) { this.fem = fem; }
	public RenderableComponentList<MuscleBundle> getMuscleBundles() { return fem.getMuscleBundles(); }
	public String getMuscleName(int index) { return muscleNames[index]; }
	public boolean isFemAttached() { return this.attachFemFlag; }

	@Override
	public ModelComponent getModelComponent () { return fem; }
	
	public void setLeftRightNames() {
		Vector3d centroid = new Vector3d();
		fem.getSurfaceMesh().computeCentroid(centroid);
		if (centroid.x > 0.0 && fem.getName().contains("left")) {
			fem.setName(fem.getName().replace("left", "right"));
			name = fem.getName();
		}
		else if (centroid.x < 0.0 && fem.getName().contains("right")) {
			fem.setName(fem.getName().replace("right", "left"));
			name = fem.getName();
		}

		for (MuscleBundle bundle : fem.getMuscleBundles()) {
			double avgX = 0.0;
			int count = 0;
			for (Muscle muscle : bundle.getFibres()) {
				avgX += muscle.getFirstPoint().getPosition().x;
				avgX += muscle.getSecondPoint().getPosition().x;
				count += 2;
			}

			avgX /= count;

			if (avgX > 0.0 && bundle.getName().contains("left")) {
				bundle.setName(bundle.getName().replace("left", "right"));
			}
			else if (avgX < 0.0 && bundle.getName().contains("right")) {
				bundle.setName(bundle.getName().replace("right", "left"));
			}
		}
	}

	public void detachFem(MechModel mech) { 
		for (FemNode3d node : fem.getNodes()) {
			if (node.isAttached()) {
				mech.detachPoint(node);
			}
		}
	}

	public double getElementVolume(int elementNumber) {
		return volumeData.volumeMap.get(elementNumber);
	}

	public void printCrashReport() {
		String outString = "\nInverted elements of FEM model '" + getName() + "':\n";
		boolean noInvertedElementsFlag = true;
		for (FemElement3d elem : fem.getElements()) {
			if (elem.isInverted()) {
				noInvertedElementsFlag = false;
				outString += "Inversion report for " + (elem instanceof HexElement ? "hex" : (elem instanceof TetElement ? "tet" : (elem instanceof WedgeElement ? "wedge" : ""))) + " element " + elem.getNumber() + ": " + " rest volume = " + String.format("%.3e", volumeData.getVolume(elem.getNumber())) + " m^3 " + (volumeData.checkForSmallSize(elem.getNumber()) ? "(small volume)" : "") + " ... minimum Jacobian " + String.format("%.3e", FemTools.calculateMinimumJacobian(elem)) + "\n";

				for (FemNode3d node : elem.getNodes()) {

					Vector3d vel = node.getVelocity();
					outString += "\tNode " + node.getNumber()
							+ (node.isAttached() ?  " (attached)" : " (unattached)") + ", velocity: " 
							+ "x = " + String.format("%.3e", vel.x) + ", y = " + String.format("%.3e", vel.y)  + ", z = " + String.format("%.3e", vel.z)  + "\n";
				}
			}
			else {
				elem.setElementWidgetSize(0.1);
			}
		}

		System.out.println(outString + (noInvertedElementsFlag ? "\tNone" : ""));
	}

	public boolean isStiffening() { return useStiffenerFlag; }
	public void stiffen() { stiffener.updateElemStiffnesses(); }

	public void printStatistics() {

		System.out.println("\nStatistics for FEM model '" + fem.getName() + "': " + fem.getNodes().size() + " nodes and " + fem.getElements().size() + " elements\n" 
				+ "\tTot mass: " + String.format("%.3e", fem.getMass()) + " kg\n" 
				+ "\tAvg vol.: " + String.format("%.3e", volumeData.getAverageVolume()) + " m^3\n"
				+ "\tMin vol.: " + String.format("%.3e", volumeData.getMinVolume()) + " m^3\n"
				+ "\tMax vol.: " + String.format("%.3e", volumeData.getMaxVolume()) + " m^3\n"
				+ "\tTen smallest elements: " + volumeData.getSmallest10() + "\n");
	}

	public void printMuscleInformation() {
		System.out.println("\nMuscle information for FEM model '" + fem.getName() + "':");
		for (MuscleBundle bundle : fem.getMuscleBundles()) {
			int numberOfFibers = bundle.getFibres().size();
			System.out.println("\tMuscle " + bundle.getName() + ": " + numberOfFibers + " fibers");
		}
	}
	
	public String createMaterialReport() {
		String materialReport = "";
		
		MooneyRivlinMaterial mrm = (MooneyRivlinMaterial) fem.getMaterial();
		materialReport += "[Bulk Modulus = " + String.format("%.3e", mrm.getBulkModulus()) + " ... ";
		materialReport += "C10 = " + String.format("%.3e", mrm.getC10()) + " ... ";
		materialReport += "C20 = " + String.format("%.3e", mrm.getC20()) + " ... ";
		materialReport += "Hard Incomp = " + fem.getIncompressible().toString() + " ... ";
		materialReport += "Soft Incomp = " + fem.getSoftIncompMethod().toString() + "]";
		
		return materialReport;
	}

	public String[][] createMuscleForceReport() {
		if (fem.getMuscleBundles().size() > 0) {
			String[][] muscleForceReport = new String[fem.getMuscleBundles().size()][2];
			int mbCount = 0;
			for (MuscleBundle mb : fem.getMuscleBundles()) {
				
				//Average the properties across the fibers
				double sumMaxForce = 0.0;
				double sumForceScaling = 0.0;
				double numFibers = mb.getFibres().size();
				if (numFibers > 0) {
					for (Muscle m : mb.getFibres()) {
						if (m.getMaterial() instanceof PeckAxialMuscle) {
							PeckAxialMuscle pam = (PeckAxialMuscle) m.getMaterial();
							sumMaxForce += pam.getMaxForce();
							sumForceScaling += pam.getForceScaling();
						}
						else if (m.getMaterial() instanceof ConstantAxialMuscle) {
							ConstantAxialMuscle cam = (ConstantAxialMuscle) m.getMaterial();
							sumMaxForce += cam.getMaxForce();
							sumForceScaling += cam.getForceScaling();							
						}
					}
					
					muscleForceReport[mbCount][0] = mb.getName();
					muscleForceReport[mbCount][1] = "[Max Force = " + String.format("%.3e", (sumMaxForce/numFibers)) + " ... Force Scaling = " + String.format("%.3e", (sumForceScaling/numFibers)) + "]";
				}
				mbCount++;
			}

			return muscleForceReport;
		}
		return null;
	}
	
	@Override
	public Point assemblePoint(MechModel mech, String name, Point3d point) { 
	        mech.addAttachment(fem.createPointAttachment(new Point(point), 0.0));
		return null;
	}
	@Override
	public void setDynamic(boolean dynamicFlag) {
		if (isUsed()) {
			this.dynamicFlag = dynamicFlag;
			fem.setDynamicsEnabled(dynamicFlag);
		}
	}
	@Override
	public void setVisible(boolean visibleFlag) {
		if (isUsed()) {
			RenderProps.setVisible(fem, visibleFlag);
		}
	}
	@Override
	public PolygonalMesh getMesh() {
		if (isUsed()) {
			return fem.getSurfaceMesh();
		}
		return null;
	}
	@Override
	public void transformGeometry(AffineTransform3dBase transform) { fem.transformGeometry(transform); }
	@Override
	public void setName(String name) {
		this.name = name;
		fem.setName(name);
	}
	@Override
	public void setDensity(double density) {
		if (isUsed()) {
			fem.setDensity(density);
		}
	}
	@Override
	public Point3d getKeyPoint() {
		Point3d centroid = new Point3d();
		fem.getSurfaceMesh().computeCentroid(centroid);
		return centroid; 
	}

	public ArrayList<PolygonalMesh> getCollisionSubsurfaces() { return collisionSubsurfaces; }

	/** Used when determining which nodes to attach to which RigidBodies */
	public static class NodeProximityData {
		int nodeNumber;
		FemNode3d node;
		ArrayList<ProximityData> proximityData = new ArrayList<ProximityData>();

		NodeProximityData(FemNode3d node, int nodeNumber) {
			this.node = node;
			this.nodeNumber = nodeNumber;
		}

		public static ArrayList<NodeProximityData> createNodeProximityDataArray(PointList<FemNode3d> nodes) {
			ArrayList<NodeProximityData> nodeArray = new ArrayList<NodeProximityData>();
			for (FemNode3d node : nodes) {
				nodeArray.add(new NodeProximityData(node, node.getNumber()));
			}

			return nodeArray;
		}

		public void addProximityData(RigidBody body, double distance) {
			proximityData.add(new ProximityData(body, distance));
		}

		public RigidBody getNearestBody() {
			if (proximityData.size() == 0) {
				return null;
			}
			else if (proximityData.size() > 1) {
				return proximityData.get(0).body;
			}
			else {
				RigidBody nearestBody = proximityData.get(0).body;
				double distanceToBeat = proximityData.get(0).distance;
				for (ProximityData pd : proximityData) {
					if (pd.distance < distanceToBeat) {
						distanceToBeat = pd.distance;
						nearestBody = pd.body;
					}
				}

				return nearestBody;
			}

		}

		public static class ProximityData {
			RigidBody body;
			double distance;
			ProximityData(RigidBody body, double distance) {
				this.body = body;
				this.distance = distance;
			}
		}
	}

	public void addCollisionSubsurface(PolygonalMesh collisionSubsurface) {
		collisionSubsurfaces.add(collisionSubsurface);
	}

	public Color getDefaultElementColor(FemElement3d element) {
		return elementColorMap.get(element);
	}
}

