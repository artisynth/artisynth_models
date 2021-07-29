package artisynth.models.larynx_QL2.components;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SphericalJoint;
import artisynth.core.mechmodels.SphericalRpyJoint;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
//import artisynth.models.frank.FrankModel;
import artisynth.models.larynx_QL2.VocalTractBase;
import artisynth.models.larynx_QL2.components.Contractile.MuscleData;
import artisynth.models.larynx_QL2.components.HardBody.HardBodyConstraint;
import artisynth.models.larynx_QL2.components.Names.All;
import artisynth.models.larynx_QL2.components.Names.Bodies;
import artisynth.models.larynx_QL2.components.Names.Files;
import artisynth.models.larynx_QL2.components.Names.HasOriginAndInsertion;
import artisynth.models.larynx_QL2.components.Names.Ligaments;
import artisynth.models.larynx_QL2.components.Names.Muscles;
import artisynth.models.larynx_QL2.components.Names.Region;
import artisynth.models.larynx_QL2.components.Names.VocalTractType;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemIOMethods;
import artisynth.models.larynx_QL2.tools.AuxTools;
import artisynth.models.larynx_QL2.tools.FemTools;
import artisynth.models.larynx_QL2.tools.GeometryTools;
import artisynth.models.larynx_QL2.tools.RenderTools;
import artisynth.models.larynx_QL2.tools.FemTools.FemSymmetryMap;
import artisynth.models.larynx_QL2.tools.FemTools.FemSymmetryMap.SymmetryAxis;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class Structures extends StructureBase {
	//Flags
	private static boolean stiffenerFlag = true;

	//Anchor body
	private static HardBody anchor;

	//Airway
	private static SkinMeshBody airway;
	   
	//Structure arrays
	private static ArrayList<AbstractBody> bodies = new ArrayList<AbstractBody>();
	private static ArrayList<SoftBody> softBodies = new ArrayList<SoftBody>();
	private static ArrayList<PointBody> pointBodies = new ArrayList<PointBody>();
	private static ArrayList<HardBody> hardBodies = new ArrayList<HardBody>();

	private static ArrayList<Tissue> tissues = new ArrayList<Tissue>();
	private static ArrayList<Ligamentous> ligaments = new ArrayList<Ligamentous>();
	private static ArrayList<Contractile> muscles = new ArrayList<Contractile>();	

	public Structures(MechModel mech, VocalTractType vtType) {
		clearStructureArrays();
		System.out.println("\nLoading bodies");
		
		//Anchor body
		anchor = new HardBody(mech, vtType, "anchor", false, false);
		hardBodies.add(anchor);
		bodies.add(anchor);

		//Particles
		pointBodies.add(new PointBody(mech, Names.All.NASOPHARYNX));
		pointBodies.add(new PointBody(mech, Names.All.OROPHARYNX));
		pointBodies.add(new PointBody(mech, Names.All.LARYNGOPHARYNX));

		bodies.addAll(pointBodies);
		
		//Rigid bodies
		hardBodies.add(new HardBody(mech, vtType, Names.All.ARYTENOID_LEFT));
		hardBodies.add(new HardBody(mech, vtType, Names.All.ARYTENOID_RIGHT));
		hardBodies.add(new HardBody(mech, vtType, Names.All.CRICOID));//, HardBodyConstraint.FULL_PLANAR_JOINT_X));
		hardBodies.add(new HardBody(mech, vtType, Names.All.CUNEIFORM_LEFT, false));
		hardBodies.add(new HardBody(mech, vtType, Names.All.CUNEIFORM_RIGHT, false));
		hardBodies.add(new HardBody(mech, vtType, Names.All.EPIGLOTTIS, HardBodyConstraint.FULL_PLANAR_JOINT_X));
		hardBodies.add(new HardBody(mech, vtType, Names.All.THYROID, HardBodyConstraint.FULL_PLANAR_JOINT_X));

		hardBodies.add(new HardBody(mech, vtType, Names.All.HYOID, HardBodyConstraint.FULL_PLANAR_JOINT_X));
		hardBodies.add(new HardBody(mech, vtType, Names.All.MANDIBLE, HardBodyConstraint.FULL_PLANAR_JOINT_X));
		hardBodies.add(new HardBody(mech, vtType, Names.All.MAXILLA, false, true));
		
		bodies.addAll(hardBodies);
		
		//Deformable (FEM) bodies
		softBodies.add(new SoftBody(mech, vtType, Names.All.FACE));
		softBodies.add(new SoftBody(mech, vtType, Names.All.LARYNX));
		softBodies.add(new SoftBody(mech, vtType, Names.All.TONGUE));
		softBodies.add(new SoftBody(mech, vtType, Names.All.VELUM));

		bodies.addAll(softBodies);
		
		//correctLarynxAttachments(mech);
		
		//Create airway
		//System.out.println("Loading airway");
		//addAirway(mech, vtType);
		
		//Create tissues from tissue data file (populates 'tissues', 'muscles', and 'ligaments' arrays)
		System.out.println("Loading tissues");
		Tissue.readTissuesFromCSV(mech, vtType.getFullFileName(Names.Files.TISSUES), false);
		
		System.out.println("Finalizing model behaviour");
		
		//Add muscle exciters
		Contractile.addExciters(mech, muscles);
	
		//Add collision behaviour
		addCollisionBehaviour(mech, vtType);
	}

	private void addCollisionBehaviour(MechModel mech, VocalTractType vtType) {
	       //mech.setCollisionBehavior(getThyroidRigidBody(), getHyoidRigidBody(), true);
	     //  mech.setCollisionBehavior(getArytenoidLeftRigidBody(), getArytenoidRightRigidBody(), true);
	       //mech.setCollisionBehavior(getCricoidRigidBody(), getArytenoidRightRigidBody(), true);
	       //mech.setCollisionBehavior(getCricoidRigidBody(), getArytenoidLeftRigidBody(), true);
	       
	       /*
	       RigidBody spineBox = new RigidBody("spine plane");
	       PolygonalMesh pm = MeshFactory.createBox(0.05, 0.05, 0.005);
	       spineBox.setMesh(pm);
	       spineBox.setRenderProps(new RenderProps());
	       spineBox.getRenderProps().setVisible(true);
	       spineBox.getRenderProps().setAlpha(0.4);
	       spineBox.transformGeometry(new RigidTransform3d(new Vector3d(0, -0.0223578, 0.0130165), new AxisAngle(1.0, 0.0, 0.0, -0.1)));
	       spineBox.setDynamic(false);
	       mech.addRigidBody(spineBox);
	       mech.setCollisionBehavior(getCricoidRigidBody(), spineBox, true);
	       */
	       
	       
		//Set collision behaviors
		try {
			if (vtType.isMaal()) {
				mech.setCollisionBehavior(getMandibleRigidBody(), getMaxilla().createCollisionBox(mech, vtType), true);
			}
			else {
				mech.setCollisionBehavior(getMandibleRigidBody(), getMaxillaRigidBody(), true, 0.2);
			}
			
			
			if (vtType.useFaceFEM()) {
				mech.setCollisionBehavior(getFaceFem(), getMaxillaRigidBody(), true);
				mech.setCollisionBehavior(getFaceFem(), getMandibleRigidBody(), true);
			}

			if (vtType.useTongueFEM()) {
				mech.setCollisionBehavior(getTongueFem(), getMaxillaRigidBody(), true);
				mech.setCollisionBehavior(getTongueFem(), getMandibleRigidBody(), true);
				mech.setCollisionBehavior(getTongueFem(), getEpiglottisRigidBody(), true);

				if (vtType.useLarynxFEM()) {
					mech.setCollisionBehavior(getLarynxFem(), getTongueFem(), true);
				}
			}
			
			
		} catch (Exception ex) {
			if (!getMandible().isUsed()) {
				System.err.println("Could not set collision behaviour for '" + getMandible().getName() + "'.");
			}
			if (!getMandible().isUsed()) {
				System.err.println("Could not set collision behaviour for '" + getMaxilla().getName() + "'.");
			}
		}	
	}
	
	
	private void correctLarynxAttachments(MechModel mech) {
	       FemSymmetryMap fsm = new FemSymmetryMap(getLarynxFem(), SymmetryAxis.X, 1e-10);
	       
	       //Remove nodes attached to left arytenoid
	       for (FemNode3d fn : getLarynxFem().getNodes()) {
	              if (fn.isAttached()) {
	                     if ((RigidBody) ((PointFrameAttachment) fn.getAttachment()).getFrame() == getArytenoidLeftRigidBody()) {
	                            mech.detachPoint(fn);
	                     }
	              }
	       }
	       
	       //Attach symmetric nodes to left arytenoid
	       int count = 0;
	       for (FemNode3d fn : getLarynxFem().getNodes()) {
	              if (fn.isAttached()) {
	                     if ((RigidBody) ((PointFrameAttachment) fn.getAttachment()).getFrame() == getArytenoidRightRigidBody()) {
	                            FemNode3d nodeF = fsm.nodeSymmetryForward.get(fn);
	                            FemNode3d nodeR = fsm.nodeSymmetryReverse.get(fn);
	                            if (nodeR != null) {
	                                   mech.attachPoint(nodeR, getArytenoidLeftRigidBody());
	                                   count++;
	                            }
	                            else {
	                                   System.err.println("No symmetry pairing for node: " + fn.myNumber);
	                            }
	                     }
	              }
	       }
	       System.out.println("Number of nodes attached: " + count);
	       
	       FemIOMethods.writeAttachmentsToCSV(getLarynxFem(), VocalTractBase.dataDirectory + File.separator + "new_larynx_attachments.csv");
	       
	}
	
	private void correctGeometry() {

              //Correct arytenoids
              Vector3d right = getArytenoidRightRigidBody().getCenterOfMass();
              Vector3d pL = new Vector3d(getArytenoidLeftRigidBody().getCenterOfMass());
              Vector3d pR = new Vector3d(-right.x, right.y, right.z);
              
              Vector3d diff = new Vector3d(pR);
              diff.sub(pL);
              
              Point3d cL = new Point3d();
              Point3d cR = new Point3d();
              getArytenoidLeftRigidBody().getMesh().computeCentreOfVolume(cL);
              getArytenoidRightRigidBody().getMesh().computeCentreOfVolume(cR);
              System.out.println(cL);
              System.out.println(cR);
              
              getArytenoidLeftRigidBody().getMesh().transform(new RigidTransform3d(diff.x, diff.y, diff.z));
              
              cL.setZero();
              cR.setZero();
              getArytenoidLeftRigidBody().getMesh().computeCentreOfVolume(cL);
              getArytenoidRightRigidBody().getMesh().computeCentreOfVolume(cR);

              AuxTools.saveMeshToOBJ(VocalTractBase.dataDirectory + "\\corrected_arytenoid_L.obj", "transformed arytenoid L", getArytenoidLeftRigidBody().getMesh());
              
              System.out.println(cL);
              System.out.println(cR);
	}
	
	
	private void addAirway(MechModel mech, VocalTractType vtType) {
		String meshFileName = vtType.getFullFileName("airway") + Files.AIRWAY.getSuffix();

		try {
			//Load the airway mesh
			PolygonalMesh mesh = new PolygonalMesh(new File(meshFileName));
			airway = new SkinMeshBody(mesh);
			airway.setName("airway");
		}
		catch (IOException e) {
			System.err.println("Airway could not be loaded. Could not locate airway mesh file " + meshFileName);
		} 

		if (airway != null) {
			mech.addMeshBody(airway);
			
			//Define the boundary bodies/weights
			if (getMandible().isUsed()) airway.addFrame(getMandibleRigidBody());
			if (getMaxilla().isUsed()) airway.addFrame(getMaxillaRigidBody());

			if (vtType.useFaceFEM()) airway.addFemModel(getFaceFem());
			if (vtType.useLarynxFEM()) airway.addFemModel(getLarynxFem());
			if (vtType.useTongueFEM()) airway.addFemModel(getTongueFem());
			if (vtType.useVelumFEM()) airway.addFemModel(getVelumFem());

			airway.computeWeights();

			//Define the rendering
			airway.getRenderProps().setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);
			airway.getRenderProps().setFaceColor(Color.white);
			airway.getRenderProps().setAlpha(0.7);
			//airway.getRenderProps().setShading(Renderer.Shading.SMOOTH);
		}
		       
	}
	
	private void clearStructureArrays() {
		bodies.clear();
		pointBodies.clear();
		softBodies.clear();
		hardBodies.clear();

		ligaments.clear();
		muscles.clear();
		tissues.clear();
	}

	public void printStats() {
		for (SoftBody sb : softBodies) {
			if (sb.isUsed()) {
				sb.printStatistics();
			}
		}
	}

	public boolean isStiffeningUsed() { return stiffenerFlag; }

	public static void setSoftBodyDensity(double density) {
		for (SoftBody sb : softBodies) {
			sb.setDensity(density);
		}
	}


	public static void setHardBodyProperties(double density, double frameDamping, double rotaryDamping) {
		for (HardBody hb : hardBodies) {
			hb.getBody().setDensity(density);
			hb.getBody().setFrameDamping(frameDamping);
			hb.getBody().setRotaryDamping(rotaryDamping);
		}
		
	}
	
	public static void setDefaultSoftBodyColors() {
		if (getFace().isUsed()) RenderTools.setupRenderProperties(getFaceFem(), new Color(0.9f, 0.8f, 0.75f), FemMethods.defaultNodeColor);
		if (getLarynx().isUsed()) RenderTools.setupRenderProperties(getLarynxFem(), new Color(0.9f, 0.75f, 0.75f), FemMethods.defaultNodeColor);
		if (getTongue().isUsed())  RenderTools.setupRenderProperties(getTongueFem(), new Color(0.8f, 0.5f, 0.5f), FemMethods.defaultNodeColor);
	}

	public void detachAllFems(MechModel mech) { 
		for (SoftBody sb : softBodies) {
			sb.detachFem(mech);
		}
	}

	public static SkinMeshBody getAirway() { return airway; }
	public static void setAirwayVisibility(boolean visibleFlag) { if (airway != null) airway.getRenderProps().setVisible(visibleFlag); }
	public static void removeAirway() { if (airway != null) VocalTractBase.mech.remove(airway); }
	
	public static SoftBody getFace() { return getSoftBodyByName(Names.Bodies.FACE); }
	public static SoftBody getLarynx() { return getSoftBodyByName(Names.Bodies.LARYNX); }
	public static SoftBody getTongue() { return getSoftBodyByName(Names.Bodies.TONGUE); }
	public static SoftBody getVelum() { return getSoftBodyByName(Names.Bodies.VELUM); }

	public static FemMuscleModel getFaceFem() { return getFace().getFem(); }
	public static FemMuscleModel getLarynxFem() { return getLarynx().getFem(); }
	public static FemMuscleModel getTongueFem() { return getTongue().getFem(); }
	public static FemMuscleModel getVelumFem() { return getVelum().getFem(); }
	
	public static HardBody getMaxilla() { return getHardBodyByName(Names.Bodies.MAXILLA); }
	public static HardBody getMandible() { return getHardBodyByName(Names.Bodies.MANDIBLE); }
	public static HardBody getHyoid() { return getHardBodyByName(Names.Bodies.HYOID); }
	public static HardBody getCricoid() { return getHardBodyByName(Names.Bodies.CRICOID); }
	public static HardBody getThyroid() { return getHardBodyByName(Names.Bodies.THYROID); }
	public static HardBody getArytenoidLeft() { return getHardBodyByName(Names.Bodies.ARYTENOID_LEFT); }
	public static HardBody getArytenoidRight() { return getHardBodyByName(Names.Bodies.ARYTENOID_RIGHT); }
	public static HardBody getEpiglottis() { return getHardBodyByName(Names.Bodies.EPIGLOTTIS); }

	public static RigidBody getMaxillaRigidBody() { return getMaxilla().getBody(); }
	public static RigidBody getMandibleRigidBody() { return getMandible().getBody(); }
	public static RigidBody getHyoidRigidBody() { return getHyoid().getBody(); }
	public static RigidBody getCricoidRigidBody() { return getCricoid().getBody(); }
	public static RigidBody getEpiglottisRigidBody() { return getEpiglottis().getBody(); }
	public static RigidBody getArytenoidLeftRigidBody() { return getArytenoidLeft().getBody(); }
	public static RigidBody getArytenoidRightRigidBody() { return getArytenoidRight().getBody(); }
	public static RigidBody getThyroidRigidBody() { return getThyroid().getBody(); }
	
	public static RigidBody getRigidBodyByName(String name) { 
		HardBody hb = (HardBody) getBodyByName(name);
		return hb.getBody();
	}

	public static HasCommonProperties getStructureByName(String name) {
		HasCommonProperties struct = null;
		for (AbstractBody body : bodies) {
			if (body.getName().equals(name)) {
				struct = body;
			}
		}		
		if (struct != null) {
			return struct;
		}
		
		
		for (Tissue t : tissues) {
			if (t.name.equals(name)) {
				struct = t;
				break;
			}	
		}
		if (struct != null) {
			return struct;
		}
		
		System.err.println("Could not find structure '" + name + "'");
		return null;
	}
	
	public static void setMuscleForceProperties(Muscles muscle, double maxForce, double forceScaling) {
              MuscleData md = Structures.getMusclesByName(muscle);
              for (ExcitationComponent ec : md.excitationSet) {
                     if (ec instanceof MuscleBundle) {
                            for (Muscle m : ((MuscleBundle) ec).getFibres()) {
                                   if (m.getMaterial() instanceof PeckAxialMuscle) {
                                          PeckAxialMuscle pam = (PeckAxialMuscle) m.getMaterial();
                                          pam.setMaxForce(maxForce);
                                          pam.setForceScaling(forceScaling);
                                   }
                                   else if (m.getMaterial() instanceof ConstantAxialMuscle) {
                                          ConstantAxialMuscle cam = (ConstantAxialMuscle) m.getMaterial();
                                          cam.setMaxForce(maxForce);
                                          cam.setForceScaling(forceScaling);
                                   }
                                   else {
                                          System.err.println("Cannot scale force of muscle '" + muscle.getName() + "'. Material is not of type PeckAxialMuscle or ConstantAxialMuscle.");
                                   }
                            }
                     }
                     else if (ec instanceof Muscle) {
                            Muscle m = (Muscle) ec;
                            if (m.getMaterial() instanceof PeckAxialMuscle) {
                                   PeckAxialMuscle pam = (PeckAxialMuscle) m.getMaterial();
                                   pam.setMaxForce(maxForce);
                                   pam.setForceScaling(forceScaling);
                            }
                            else if (m.getMaterial() instanceof ConstantAxialMuscle) {
                                   ConstantAxialMuscle cam = (ConstantAxialMuscle) m.getMaterial();
                                   cam.setMaxForce(maxForce);
                                   cam.setForceScaling(forceScaling);
                            }
                            else {
                                   System.err.println("Cannot scale force of muscle '" + muscle.getName() + "'. Material is not of type PeckAxialMuscle or ConstantAxialMuscle.");
                            }
                     }
              }
       }
       
	
	public static Tissue getTissueByName(Names name) { return getTissueByName(name.getName()); }
	public static Tissue getTissueByName(String name) {
		for (Tissue t : tissues) {
			if (t.name.equals(name)) {
				return t;
			}	
		}
		
		System.err.println("Could not find tissue '" + name + "'");
		return null;
	}


	public static AbstractBody getBodyByName(Names name) { return getBodyByName(name.getName()); }
	public static AbstractBody getBodyByName(String name) {
		for (AbstractBody body : bodies) {
			if (body.getName().equals(name)) {
				return body;
			}
		}
		return null;
	}

	public static SoftBody getSoftBodyByName(Names name) {
		return (SoftBody) getBodyByName(name);
	}

	public static HardBody getHardBodyByName(Names name) {
		return (HardBody) getBodyByName(name);
	}


	public static HardBody getAnchor() { return anchor; }
	public static void addPointBody(PointBody pb) { 
		pointBodies.add(pb);
		bodies.add(pb);
	}

	public static ArrayList<AbstractBody> getBodies() { return bodies; }
	public static ArrayList<SoftBody> getSoftBodies() { return softBodies; }
	public static ArrayList<HardBody> getHardBodies() { return hardBodies; }
	public static ArrayList<Tissue> getTissues() { return tissues; }
	public static void addTissue(Tissue tissue) { 
		tissues.add(tissue);

		if (tissue instanceof Contractile) {
			muscles.add((Contractile) tissue);
		}
		else if (tissue instanceof Ligamentous) {
			ligaments.add((Ligamentous) tissue);
		}
	}

	public static Contractile getMuscleByName(Names name) {
		for (Contractile c : muscles) {
			if (c.getName().contains(name.getName())) {
				return c;
			}
		}
		
		System.err.println("Could not locate muscle '" + name.getName() + "'");
		return null;
	}
	
	public static void setTissueVisiblityBySharedName(String name, boolean visibleFlag) {
		ArrayList<Tissue> tissues = getTissuesBySharedName(name);
		
		for (Tissue tissue : tissues) {
			if (tissue != null) {
				tissue.setVisible(visibleFlag);
			}
		}
	}
	
	
	public static ArrayList<Tissue> getTissuesBySharedName(String sharedName) {
		ArrayList<Tissue> tissuesWithSharedName = new ArrayList<Tissue>();
		for (Tissue t : tissues) {
			if (t.getName().contains(sharedName)) {
				tissuesWithSharedName.add(t);
			}
		}
		return tissuesWithSharedName;
	}
	
	public static ArrayList<Contractile> getMuscleTissues() { return muscles; }
	public static ArrayList<MuscleExciter> getMuscleTissueExciters() {
		ArrayList<MuscleExciter> exciters = new ArrayList<MuscleExciter>();
		for (Contractile c : muscles) {
			if (!exciters.contains(c.exciter)) {
				exciters.add(c.exciter);
			}
		}
		return exciters; 
	}
	
	public FemMuscleModel getFEM(String name) { 
		for (SoftBody body : softBodies) {
			if (body.getName().equals(name)) {
				return body.getFem();
			}
		}
		return null;
	}
	public static RigidBody getRigidBody(String name) { 
		for (HardBody body : hardBodies) {
			if (body.getName().equals(name)) {
				return body.getBody();
			}
		}
		return null;
	}

	public static MuscleData getMusclesByName(Names.Muscles muscleType) {
		ArrayList<Contractile> contractileSet = new ArrayList<Contractile>();
		ArrayList<ExcitationComponent> excitationComponents = new ArrayList<ExcitationComponent>();

		for (Contractile c : muscles) {
			if (c.name.contains(muscleType.getName().trim())) {
				contractileSet.add(c);
				excitationComponents.add((ExcitationComponent) c.muscle);
			}
		}

		for (SoftBody sb : softBodies) {
			if (sb.isUsed()) {
				for (MuscleBundle mb : sb.getMuscleBundles()) {
					if (mb.getName().contains(muscleType.getName().trim())) {
						excitationComponents.add(mb);
					}
				}
			}
		}

		if (excitationComponents.size() == 0) {
			System.err.println("Warning: Could not find muscles associated with the name '" + muscleType.getName() + "'.");
		}
		return new MuscleData(muscleType.getName(), contractileSet, excitationComponents);
	}

	/** Applies <i>transform</i> to all model components. **/
	public static void transformAll(RigidTransform3d transform) {
		for (AbstractBody body : bodies) {
			body.transformGeometry(transform);
		}
	}

	/** Changes visibility of all model components. **/
	public static void setGlobalVisibility(MechModel mech, boolean visibleFlag) {
	       for (AbstractBody body : bodies) {
	              body.setVisible(visibleFlag);
	       }
	       
	       for (Tissue tissue : tissues) {
	              tissue.setVisible(visibleFlag);
	       }
	       
	       RenderProps.setVisible(mech.frameMarkers(), visibleFlag);
	       RenderProps.setVisible(mech.particles(), visibleFlag);

	       if (airway != null) airway.getRenderProps().setVisible(visibleFlag);
	}
	
       public static void setGlobalDynamics(MechModel mech, boolean dynamicFlag) {
              for (AbstractBody body : bodies) {
                     body.setDynamic(dynamicFlag);
              }
              
              for (Particle particle : mech.particles()) {
                     particle.setDynamic(dynamicFlag);
              }
       }
	
	public void setLeftRightNames() {
		for (AbstractBody body : bodies) {
			if (body.isUsed()) {
				body.setLeftRightNames();
			}
		}

		for (Tissue tissue : tissues) {
			tissue.setLeftRightNames();
		}
	}

	public Point getOriginPointByName(HasOriginAndInsertion tissue) {
		for (Tissue t : tissues) {
			if (t.getName().contains(tissue.getName()) && t.getSide().equals(tissue.getSide())) {
				return t.originPoint;
			}
		}
		return null;
	}

	public Point getInsertionPointByName(HasOriginAndInsertion tissue) {
		for (Tissue t : tissues) {
			if (t.getName().contains(tissue.getName()) && t.getSide().equals(tissue.getSide())) {
				return t.insertionPoint;
			}
		}
		return null;
	}

	public static void scaleContractileTissueProperties(double forceScaling, double maxForce, double damping, double passiveFraction) {
		for (Contractile c : muscles) {
			c.setForceProperties(forceScaling, maxForce, damping, passiveFraction);
		}
	}
	
	public static void scaleLigamentousTissueProperties(double stiffness, double damping) {
		for (Ligamentous l : ligaments) {
			l.setProperties(stiffness, damping);
		}
	}

	public static PolygonalMesh getCollisionMesh(All surfaceName) {
		for (SoftBody sb : softBodies) {
			for (PolygonalMesh subMesh : sb.getCollisionSubsurfaces()) {
				if (subMesh.getName().contains(surfaceName.getName())) {
					return subMesh;
				}
			}
		}
		return null;
	}

	public static ArrayList<HasCommonProperties> getRegion(Region region) {
		ArrayList<HasCommonProperties> components = new ArrayList<HasCommonProperties>();
		
		for (Tissue tissue : tissues) {
			if (region.checkName(tissue.getName())) {
				components.add(tissue);
			}
		}
		
		for (AbstractBody body : bodies) {
			if (region.checkName(body.getName())) {
				components.add(body);
			}
		}
		
		return components;
	}
	
	public static PolygonalMesh getCollisionMeshByRegion(Region region) {
		for (SoftBody sb : softBodies) {
			for (PolygonalMesh pm : sb.getCollisionSubsurfaces()) {
				if (region.checkName(pm.getName())) {
					return pm;
				}
			}
		}
		
		System.err.println("Could not locate mesh associated with region '" + region.getName() + "'.");
		return null;
	}
	
	public interface HasCommonProperties {
		public String getName();
		public void setVisible(boolean visibleFlag);
		public void setDynamic(boolean dynamicFlag);
		public ModelComponent getModelComponent();
	}

	
	public static Ligamentous getLigamentByName(Ligaments ligament) {
		for (Ligamentous lig : ligaments) {
			if (ligament.checkNameAndSide(lig)) {
				return lig;
			}
		}
		return null;
	}
	
	public static ArrayList<Ligamentous> getLigamentsByName(Ligaments ligament) {
		ArrayList<Ligamentous> ligamentSet = new ArrayList<Ligamentous>();
		for (Ligamentous lig : ligaments) {
			if (lig.getName().contains(ligament.getName())) {
				ligamentSet.add(lig);
			}
		}
		return ligamentSet;
	}

	public static void setLigamentVisibility(boolean visibleFlag, String ... nameParts) {
		for (Ligamentous lig : ligaments) {
			for (String namePart : nameParts) {
				if (lig.getName().contains(namePart)) {
					lig.setVisible(visibleFlag);
				}
			}
		}
	}
	
	
	/** Creates a bare larynx mech model lacking any of the suprathyroid musculature or structures. A hyoid can be supplied; otherwise a default hyoid is used. 
	 * 
	 * @param hyoid externally defined hyoid. If <b>null</b> then a default hyoid is used.
	 * @param worldTransform externally defined world transform. No transform is applied if <b>null</b>
	 * @param scale indicates an alternative scaling should be used (the model is by default in MKS: meters, kilograms, seconds; so 1 cm is 0.01 m in this model)
	 * @param addLarynxMucosaFemFlag indicates whether the mucosa should be added to the model
	 * **/
	public static MechModel createLarynx(RootModel root, RigidBody hyoid, RigidTransform3d worldTransform, double scale, boolean addLarynxMucosaFemFlag) {
		System.out.println("Creating larynx model" + (addLarynxMucosaFemFlag ? " with mucosa" : " (cartilaginous framework only)"));
		MechModel larynxMech = new MechModel("larynx" + (addLarynxMucosaFemFlag ? " with mucosa" : " cartilaginous framework"));
		VocalTractType vtType = VocalTractType.BARE_LARYNX;

		//Anchor body
		anchor = new HardBody(larynxMech, vtType, "anchor", false, false);
		hardBodies.add(anchor);
		bodies.add(anchor);
		
		if (hyoid == null) {
			hardBodies.add(new HardBody(larynxMech, vtType, Names.All.HYOID));
		}
		else {
		   RigidTransform3d worldTransformI = worldTransform.copy();
                   worldTransformI.invert();
                   hyoid.transformGeometry(worldTransformI);
                   hyoid.scaleDistance(1.0/scale);
                   
		   hardBodies.add(new HardBody(larynxMech, Names.Bodies.HYOID.getName(), hyoid));
		}
		
		//Add rigid bodies
		hardBodies.add(new HardBody(larynxMech, vtType, Names.All.ARYTENOID_LEFT));
		hardBodies.add(new HardBody(larynxMech, vtType, Names.All.ARYTENOID_RIGHT));
		hardBodies.add(new HardBody(larynxMech, vtType, Names.All.CRICOID));
		hardBodies.add(new HardBody(larynxMech, vtType, Names.All.CUNEIFORM_LEFT, false));
		hardBodies.add(new HardBody(larynxMech, vtType, Names.All.CUNEIFORM_RIGHT, false));
		hardBodies.add(new HardBody(larynxMech, vtType, Names.All.EPIGLOTTIS));
		hardBodies.add(new HardBody(larynxMech, vtType, Names.All.THYROID));
		bodies.addAll(hardBodies);
		
		//Add laryngeal mucosa
		if (addLarynxMucosaFemFlag) {
			softBodies.add(new SoftBody(larynxMech, vtType, Names.All.LARYNX));
		}
		bodies.addAll(softBodies);
		
		//Create tissues from tissue data file (populates 'tissues', 'muscles', and 'ligaments' arrays)
		System.out.println("Loading laryngeal tissues");
		Tissue.readTissuesFromCSV(larynxMech, vtType.getFullFileName(Names.Files.TISSUES), true);

		//Add muscle exciters
		Contractile.addExciters(larynxMech, muscles);

		larynxMech.scaleDistance(scale);
		
		if (worldTransform != null) {
			larynxMech.transformGeometry(worldTransform);
		}
		
//		if (hyoid != null) {
//			if (scale > 0.0) {
//				hyoid.scaleDistance(1/scale);
//			}
//			worldTransform.invert();
//			hyoid.transformGeometry(worldTransform);
//		}

		return larynxMech;
	}
	
	/** 
	 * This is peter's re-hacking...
	 * **/
        public static void createLarynx_peter(RootModel root, MechModel mechModel, RigidTransform3d worldTransform, double scale, boolean addLarynxMucosaFemFlag) {
                System.out.println("Creating larynx model" + (addLarynxMucosaFemFlag ? " with mucosa" : " (cartilaginous framework only)"));
                //MechModel mechModel = new MechModel("larynx" + (addLarynxMucosaFemFlag ? " with mucosa" : " cartilaginous framework"));
                VocalTractType vtType = VocalTractType.LARYNX_QL2;
                
                // Transform model to moisik space
                if (worldTransform != null) 
                {
                   RigidTransform3d worldTransformI = worldTransform.copy();
                   worldTransformI.invert();
                   //worldTransform.invert();
                   mechModel.transformGeometry(worldTransformI);
                   //worldTransform.invert();
                }
                //mechModel.scaleMass(s); // do w need to scale mass too? I think we work in the same mass units...
                mechModel.scaleDistance(1.0/scale);
                

                //Anchor body
                anchor = new HardBody(mechModel, vtType, "anchor", false, false);
                hardBodies.add(anchor);
                bodies.add(anchor);
                
                RigidBody hyoid = mechModel.rigidBodies().get("hyoid");
                if (hyoid == null) {
                        hardBodies.add(new HardBody(mechModel, vtType, Names.All.HYOID));
                }
                else {
                        hardBodies.add(new HardBody(Names.Bodies.HYOID.getName(), hyoid));
                }
                
                //Add rigid bodies
                hardBodies.add(new HardBody(mechModel, vtType, Names.All.ARYTENOID_LEFT));
                hardBodies.add(new HardBody(mechModel, vtType, Names.All.ARYTENOID_RIGHT));
                hardBodies.add(new HardBody(mechModel, vtType, Names.All.CRICOID));
                hardBodies.add(new HardBody(mechModel, vtType, Names.All.CUNEIFORM_LEFT, false));
                hardBodies.add(new HardBody(mechModel, vtType, Names.All.CUNEIFORM_RIGHT, false));
                hardBodies.add(new HardBody(mechModel, vtType, Names.All.EPIGLOTTIS));
                hardBodies.add(new HardBody(mechModel, vtType, Names.All.THYROID));
                bodies.addAll(hardBodies);
                
                //Add laryngeal mucosa
                if (addLarynxMucosaFemFlag) {
                        softBodies.add(new SoftBody(mechModel, vtType, Names.All.LARYNX));
                }
                bodies.addAll(softBodies);

                // up to this point I think it is fine if there is no hyoid, so perhaps I can perform the transform here...
                
                //Create tissues from tissue data file (populates 'tissues', 'muscles', and 'ligaments' arrays)
                System.out.println("Loading laryngeal tissues");
                Tissue.readTissuesFromCSV(mechModel, vtType.getFullFileName(Names.Files.TISSUES), true);

                
                //Add muscle exciters
                Contractile.addExciters(mechModel, muscles);

                // transform back to frank space
                mechModel.scaleDistance(scale);                
                if (worldTransform != null) {
                        mechModel.transformGeometry(worldTransform);
                }
                
        }

	public static void resetSoftBodyRendering() {
		for (SoftBody sb : softBodies) {
			if (sb.isUsed()) {
				sb.getFem().setElementWidgetSize(1.0);
				for (FemElement3d element : sb.getFem().getElements()) {
					element.setElementWidgetSize(1.0);
					element.setElementWidgetSizeMode(PropertyMode.Inherited);
					element.getRenderProps().setFaceColor(sb.getDefaultElementColor(element));

				}
			}
		}
	}


	public static MuscleBundle getFemMuscleByName(String name) {
		for (SoftBody sb : softBodies) {
			if (sb.isUsed()) {
				for (MuscleBundle mb : sb.getMuscleBundles()) {
					if (mb.getName().contains(name.trim())) {
						return mb;
					}
				}
			}
		}
		return null;
	}


}
