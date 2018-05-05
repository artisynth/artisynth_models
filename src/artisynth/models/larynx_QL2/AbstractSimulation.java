package artisynth.models.larynx_QL2;

import java.util.ArrayList;

import maspack.matrix.Point3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.materials.AxialMaterial;
import artisynth.core.materials.BlemkerMuscle;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.models.larynx_QL2.VocalTractBase;
import artisynth.models.larynx_QL2.components.AbstractBody;
import artisynth.models.larynx_QL2.components.Contractile;
import artisynth.models.larynx_QL2.components.HardBody;
import artisynth.models.larynx_QL2.components.Ligamentous;
import artisynth.models.larynx_QL2.components.Names;
import artisynth.models.larynx_QL2.components.Contractile.MuscleData;
import artisynth.models.larynx_QL2.components.Names.Bodies;
import artisynth.models.larynx_QL2.components.Names.Diarthrosis;
import artisynth.models.larynx_QL2.components.Names.Ligaments;
import artisynth.models.larynx_QL2.components.Names.Muscles;
import artisynth.models.larynx_QL2.components.Names.Region;
import artisynth.models.larynx_QL2.components.PointBody;
import artisynth.models.larynx_QL2.components.SoftBody;
import artisynth.models.larynx_QL2.components.Structures;
import artisynth.models.larynx_QL2.components.Names.All;
import artisynth.models.larynx_QL2.components.Names.VocalTractType;
import artisynth.models.larynx_QL2.components.Structures.HasCommonProperties;
import artisynth.models.larynx_QL2.tools.RenderTools;
import artisynth.models.larynx_QL2.components.Tissue;

/** Base class to be inherited by all simulation classes. This class ensures that all simulation classes implement methods for 
 * interaction with the ArtiSynth driver and provides access to a number of convenience objects for use in building and manipulating the simulation.
 * 
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public abstract class AbstractSimulation extends VocalTractBase implements DriverListener {
	public AbstractSimulation(String name, VocalTractType vocalTractType) {
		super(name, vocalTractType);

		addDriverListener(this);
	}

	public SkinMeshBody getAirway() { return Structures.getAirway(); }

	public HardBody getHyoid() { return Structures.getHyoid(); }
	public HardBody getCricoid() { return Structures.getCricoid(); }
	public HardBody getMandible() { return Structures.getMandible(); }
	public HardBody getMaxilla() { return Structures.getMaxilla(); }
	public HardBody getEpiglottis() { return Structures.getEpiglottis(); }
	public HardBody getThyroid() { return Structures.getThyroid(); }
	public HardBody getArytenoidLeft() { return Structures.getArytenoidLeft(); }
	public HardBody getArytenoidRight() { return Structures.getArytenoidRight(); }


	public SoftBody getFace() { return Structures.getSoftBodyByName(All.FACE); }
	public SoftBody getLarynx() { return Structures.getSoftBodyByName(All.LARYNX); }
	public SoftBody getTongue() { return Structures.getSoftBodyByName(All.TONGUE); }
	public SoftBody getVelum() { return Structures.getSoftBodyByName(All.VELUM); }

	//public MeasurementTools mt = new MeasurementTools();
	//public Observers observers = mt.observers;
	
	public FemMuscleModel getFaceFem() {
		SoftBody face = getFace();
		if (face.isUsed()) {
			return face.getFem();	
		}
		else {
			System.err.println("FEM 'face' is not in use. Specify a vocal tract type that includes the face.");
			return null;
		}
	}

	public FemMuscleModel getLarynxFem() {
		SoftBody larynx = getLarynx();
		if (larynx.isUsed()) {
			return larynx.getFem();	
		}
		else {
			System.err.println("FEM 'larynx' is not in use. Specify a vocal tract type that includes the larynx.");
			return null;
		}
	}

	public FemMuscleModel getTongueFem() {
		SoftBody tongue = getTongue();
		if (tongue.isUsed()) {
			return tongue.getFem();	
		}
		else {
			System.err.println("FEM 'tongue' is not in use. Specify a vocal tract type that includes the tongue.");
			return null;
		}
	}

	public FemMuscleModel getVelumFem() {
		SoftBody velum = getVelum();
		if (velum.isUsed()) {
			return velum.getFem();	
		}
		else {
			System.err.println("FEM 'velum' is not in use. Specify a vocal tract type that includes the velum.");
			return null;
		}
	}



	public void focusOnFace() {
		SoftBody face = Structures.getSoftBodyByName(All.FACE);
		if (face.isUsed()) {
			RenderTools.centerCamera(face.getFem().getSurfaceMesh());
		}
		else { 
			System.err.println("FEM 'face' is not in use. Specify a vocal tract type that includes the face.");
		}
	}

	public void focusOnLarynx() {
		SoftBody larynx = Structures.getSoftBodyByName(All.LARYNX);
		if (larynx.isUsed()) {
			RenderTools.centerCamera(larynx.getFem().getSurfaceMesh());
		}
		else { 
			System.err.println("FEM 'larynx' is not in use. Specify a vocal tract type that includes the larynx.");
		}
	}

	public void focusOnTongue() {
		SoftBody tongue = Structures.getSoftBodyByName(All.TONGUE);
		if (tongue.isUsed()) {
			RenderTools.centerCamera(tongue.getFem().getSurfaceMesh());
		}
		else { 
			System.err.println("FEM 'tongue' is not in use. Specify a vocal tract type that includes the tongue.");
		}
	}

	/** Sets density of the arytenoids, cuneiforms, and epiglottis. **/
	public void setInternalLaryngealCartilageDensity(double density) {
		Structures.getHardBodyByName(All.ARYTENOID_LEFT).setDensity(density);
		Structures.getHardBodyByName(All.ARYTENOID_RIGHT).setDensity(density);
		Structures.getHardBodyByName(All.CUNEIFORM_LEFT).setDensity(density);
		Structures.getHardBodyByName(All.CUNEIFORM_RIGHT).setDensity(density);
		Structures.getHardBodyByName(All.EPIGLOTTIS).setDensity(density);
	}

	/** Sets dynamic status of the thyroid, criocid, arytenoids, cuneiforms, and epiglottis. **/
	public void setLaryngealCartilageDynamics(boolean dynamicFlag) {
		Structures.getHardBodyByName(All.ARYTENOID_LEFT).setDynamic(dynamicFlag);
		Structures.getHardBodyByName(All.ARYTENOID_RIGHT).setDynamic(dynamicFlag);
		Structures.getHardBodyByName(All.CUNEIFORM_LEFT).setDynamic(dynamicFlag);
		Structures.getHardBodyByName(All.CUNEIFORM_RIGHT).setDynamic(dynamicFlag);
		Structures.getHardBodyByName(All.CRICOID).setDynamic(dynamicFlag);
		Structures.getHardBodyByName(All.THYROID).setDynamic(dynamicFlag);
		Structures.getHardBodyByName(All.EPIGLOTTIS).setDynamic(dynamicFlag);
	}

	/** Sets visibility status of the thyroid, criocid, arytenoids, cuneiforms, and epiglottis. **/
	public void setLaryngealCartilageVisibility(boolean visibleFlag) {
		Structures.getHardBodyByName(All.ARYTENOID_LEFT).setVisible(visibleFlag);
		Structures.getHardBodyByName(All.ARYTENOID_RIGHT).setVisible(visibleFlag);
		Structures.getHardBodyByName(All.CUNEIFORM_LEFT).setVisible(visibleFlag);
		Structures.getHardBodyByName(All.CUNEIFORM_RIGHT).setVisible(visibleFlag);
		Structures.getHardBodyByName(All.CRICOID).setVisible(visibleFlag);
		Structures.getHardBodyByName(All.THYROID).setVisible(visibleFlag);
		Structures.getHardBodyByName(All.EPIGLOTTIS).setVisible(visibleFlag);
		Structures.setLigamentVisibility(false, "hyo", "vocal", "crico", "thyro");

	}

	public void setMuscleVisiblity(SoftBody sb, boolean visibleFlag) { setMuscleVisiblity(sb.getFem(), visibleFlag); }
	public void setMuscleVisiblity(FemMuscleModel fem, boolean visibleFlag) {
		if (fem != null) {
			for (MuscleBundle mb : fem.getMuscleBundles()) {
				for (Muscle m : mb.getFibres()) {
					m.getRenderProps().setVisible(visibleFlag);
				}
			}
		}
	}

	public void setupMuscleDescriptionMaterial(FemMuscleModel fem, double maxStress) {
		if (fem != null) {
			for (MuscleBundle mb : fem.getMuscleBundles()) {
				mb.setMuscleMaterial(new BlemkerMuscle(2.1, 1.0, maxStress, 0.00005, 6.6));
			}
		}
	}


	public void setMuscleForceProperties(Muscles muscle, double forceScaling) {
		setMuscleForceProperties(muscle, forceScaling, forceScaling);
	}

	public void setMuscleForceProperties(FemMuscleModel fem, double maxForce, double forceScaling) {
		for (MuscleBundle mb : fem.getMuscleBundles()) {

			for (Muscle m : mb.getFibres()) {
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
					System.err.println("Cannot scale force of muscle '" + mb.getName() + "'. Material is not of type PeckAxialMuscle or ConstantAxialMuscle.");
				}
			}
		}
	}

	public void setMuscleForceProperties(Muscles muscle, double maxForce, double forceScaling) {
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

	public void setIntrinsicLaryngealMuscleForceScaling(double forceScaling) {
		class Assistant {
			public void scaleMuscleForce(ArrayList<Contractile> muscles, double scale) {
				for (Contractile c : muscles) {
					c.scaleForce(scale);
				}
			}
		}
		Assistant assistant = new Assistant();

		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.CRICOTHYROID_PARS_OBLIQUE).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.CRICOTHYROID_PARS_RECTA).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.INTERARYTENOID_OBLIQUE).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.INTERARYTENOID_INFERIOR_TRANSVERSE).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.LATERALCRICOARYTENOID).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.POSTERIOR_CRICOARYTENOID).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.THYROEPIGLOTTIC).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Laryngeal.THYROARYTENOID_VOCALIS).contractileSet, forceScaling);
	}

	/** Scales the force of the masseter, internal and external pterygoid, temporalis, digastric, and geniohyoid muscles. **/
	public void scaleJawMuscleForce(double forceScaling) {
		class Assistant {
			public void scaleMuscleForce(ArrayList<Contractile> muscles, double scale) {
				for (Contractile c : muscles) {
					c.scaleForce(scale);
				}
			}
		}
		Assistant assistant = new Assistant();

		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.MASSETER).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.INTERNAL_PTERYGOID).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.EXTERNAL_PTERYGOID).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.GENIOHYOID).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.TEMPORALIS_ANTERIOR).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.TEMPORALIS_MEDIAL).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.TEMPORALIS_POSTERIOR).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.DIGASTRIC_ANTERIOR).contractileSet, forceScaling);
		assistant.scaleMuscleForce(Structures.getMusclesByName(Muscles.Mandibular.DIGASTRIC_POSTERIOR).contractileSet, forceScaling);
	}

	public Point3d getLeftMandibularCondylePosition() {
		Point3d leftCondyle = Structures.getLigamentsByName(Ligaments.TEMPOROMANDIBULAR).get(0).insertionPoint.getPosition();
		leftCondyle.x = -Math.abs(leftCondyle.x);
		return leftCondyle;
	}

	public Point3d getRightMandibularCondylePosition() {
		Point3d rightCondyle = Structures.getLigamentsByName(Ligaments.TEMPOROMANDIBULAR).get(0).insertionPoint.getPosition();
		rightCondyle.x = Math.abs(rightCondyle.x);
		return rightCondyle;
	}


	public void setTemporomandibularJointProperties(double stiffness, double damping) {
		for (Ligamentous ligament : Structures.getLigamentsByName(Ligaments.TEMPOROMANDIBULAR)) {
			ligament.setProperties(stiffness, damping);
		}
	}

	public void setDynamic(boolean cricoidFlag, boolean thyroidFlag, boolean epiglottisFlag, boolean hyoidFlag, boolean mandibleFlag) {
		getCricoid().setDynamic(cricoidFlag);
		getThyroid().setDynamic(thyroidFlag);
		getMandible().setDynamic(mandibleFlag);
		getHyoid().setDynamic(hyoidFlag);
		getEpiglottis().setDynamic(epiglottisFlag);
	}

	public void setAxialLaryngealMuscleForces(double CT, double PCA, double LCA, double IA, double TE, double TH, double TA) {
		setMuscleForceProperties(Muscles.Laryngeal.INTERARYTENOID_OBLIQUE, IA);
		setMuscleForceProperties(Muscles.Laryngeal.INTERARYTENOID_INFERIOR_TRANSVERSE, IA);
		setMuscleForceProperties(Muscles.Laryngeal.INTERARYTENOID_SUPERIOR_TRANSVERSE, IA);
		setMuscleForceProperties(Muscles.Laryngeal.LATERALCRICOARYTENOID, LCA);
		setMuscleForceProperties(Muscles.Laryngeal.POSTERIOR_CRICOARYTENOID, PCA);
		setMuscleForceProperties(Muscles.Laryngeal.CRICOTHYROID_PARS_OBLIQUE, CT);
		setMuscleForceProperties(Muscles.Laryngeal.CRICOTHYROID_PARS_RECTA, CT);
		setMuscleForceProperties(Muscles.Laryngeal.THYROEPIGLOTTIC, TE);
		setMuscleForceProperties(Muscles.Laryngeal.THYROHYOID_INFERIOR, TH);
		setMuscleForceProperties(Muscles.Laryngeal.THYROHYOID_SUPERIOR, TH);
		setMuscleForceProperties(Muscles.Laryngeal.THYROARYTENOID_VOCALIS, TA);

	}

	public void setFemLaryngealMuscleForces(double eTA, double iTA, double TE, double ventAL, double ventAM, double ventPL) {
		setMuscleForceProperties(Muscles.Laryngeal.FEM_THYROARYTENOID_MUSCULARIS, eTA);
		setMuscleForceProperties(Muscles.Laryngeal.FEM_THYROARYTENOID_VOCALIS, iTA);
		setMuscleForceProperties(Muscles.Laryngeal.FEM_THYROEPIGLOTTIC, TE);
		setMuscleForceProperties(Muscles.Laryngeal.FEM_VENTRICULARIS_ANTEROLATERAL, ventAL);
		setMuscleForceProperties(Muscles.Laryngeal.FEM_VENTRICULARIS_ANTEROMEDIAL, ventAM);
		setMuscleForceProperties(Muscles.Laryngeal.FEM_VENTRICULARIS_POSTEROLATERAL, ventPL);
	}

	public void setFemLingualMuscleForces(double IL, double SL, double GGA, double GGM, double GGP, double HG, double Trans, double Vert) {
		setMuscleForceProperties(Muscles.Lingual.INFERIOR_LONGITUDINAL, IL);
		setMuscleForceProperties(Muscles.Lingual.SUPERIOR_LONGITUDINAL, SL);
		setMuscleForceProperties(Muscles.Lingual.GENIOGLOSSUS_ANTERIOR, GGA);
		setMuscleForceProperties(Muscles.Lingual.GENIOGLOSSUS_MEDIAL, GGM);
		setMuscleForceProperties(Muscles.Lingual.GENIOGLOSSUS_POSTERIOR, GGP);
		setMuscleForceProperties(Muscles.Lingual.HYOGLOSSUS, HG);
		setMuscleForceProperties(Muscles.Lingual.TRANSVERSUS, Trans);
		setMuscleForceProperties(Muscles.Lingual.VERTICALIS, Vert);
	}


	public void setGravity(boolean gravityFlag, FemMuscleModel ... fems) {
		for (FemMuscleModel fem : fems) {
			if (fem != null) {
				fem.setGravity(0.0, (gravityFlag ? -9.81 : 0.0), 0.0);
			}
		}
	}

	public void setAirwayVisiblity(boolean visibleFlag) { Structures.setAirwayVisibility(visibleFlag); }

	public void setCricoarytenoidPlaneVisiblity(boolean visibleFlag) {
		for (Tissue t : Structures.getTissuesBySharedName(Diarthrosis.CRICOARYTENOID_JOINT_PLANE.getName())) {
			t.setVisible(visibleFlag);	
		}
	}

	public void setPharynxPlanesVisibility(boolean visibleFlag) {
		Structures.getTissueByName(Diarthrosis.SOFT_PALATE_PLANE).setVisible(false);
		Structures.getTissueByName(Diarthrosis.POSTERIOR_PHARYNGEAL_WALL_PLANE).setVisible(false);
	}


	public void setPharynxVisibility(boolean visibleFlag) {
		ArrayList<HasCommonProperties> pharynx = Structures.getRegion(Region.PHAYNX);
		for (HasCommonProperties hcp : pharynx) {
			hcp.setVisible(visibleFlag);
		}
	}

	public void setSkeletonVisibility(boolean visibleFlag) {
		for (AbstractBody body : Structures.getBodies()) {
			if (body instanceof PointBody || body instanceof HardBody) {
				body.setVisible(visibleFlag);	
			}
		}

		for (Tissue tissue : Structures.getTissues()) {
			tissue.setVisible(visibleFlag);
		}
	}


	public void setIntrinsicLaryngealMuscleVisibility(boolean visibleFlag) {
		Structures.setTissueVisiblityBySharedName(Names.Muscles.Laryngeal.INTERARYTENOID.getName(), visibleFlag);
		Structures.setTissueVisiblityBySharedName(Names.Muscles.Laryngeal.LATERALCRICOARYTENOID.getName(), visibleFlag);
		Structures.setTissueVisiblityBySharedName(Names.Muscles.Laryngeal.POSTERIOR_CRICOARYTENOID.getName(), visibleFlag);
		Structures.setTissueVisiblityBySharedName(Names.Muscles.Laryngeal.THYROEPIGLOTTIC.getName(), visibleFlag);
		Structures.setTissueVisiblityBySharedName(Names.Muscles.Laryngeal.THYROARYTENOID_VOCALIS.getName(), visibleFlag);
	}
	
	public void setRigidBodyMuscleVisibility(boolean visibleFlag) {
	   setIntrinsicLaryngealMuscleVisibility(visibleFlag);
	   
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.DIGASTRIC_ANTERIOR.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.DIGASTRIC_POSTERIOR.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.EXTERNAL_PTERYGOID.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.INTERNAL_PTERYGOID.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.GENIOHYOID.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.MASSETER.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.MYLOHYOID.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.TEMPORALIS_ANTERIOR.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.TEMPORALIS_MEDIAL.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Mandibular.TEMPORALIS_POSTERIOR.getName(), visibleFlag);
           

           Structures.setTissueVisiblityBySharedName(Names.Muscles.Pharyngeal.INFERIOR_PHARYNGEAL_CONSTRICTOR.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Pharyngeal.MIDDLE_PHARYNGEAL_CONSTRICTOR.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Pharyngeal.STYLOPHARYNGEUS.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Pharyngeal.SUPERIOR_PHARYNGEAL_CONSTRICTOR.getName(), visibleFlag);


           Structures.setTissueVisiblityBySharedName(Names.Muscles.Hyoid.OMOHYOID.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Hyoid.STERNOHYOID.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Hyoid.STYLOHYOID.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Hyoid.THYROHYOID_INFERIOR.getName(), visibleFlag);
           Structures.setTissueVisiblityBySharedName(Names.Muscles.Hyoid.THYROHYOID_SUPERIOR.getName(), visibleFlag);
   }

	public void setOrofacialSkeletonAlpha(double alpha) {
		Structures.getMandibleRigidBody().getRenderProps().setAlpha(alpha);
		Structures.getMaxillaRigidBody().getRenderProps().setAlpha(alpha);
	}

	/*
	public void createMandibleAdjustmentController(int referenceVertexIndex, Point3d mandibleTargetPosition, double startTime, double stopTime) {
		Structures.setGlobalDynamics(mech, false);
		Structures.getMandible().setDynamic(true);

		RigidBody mandible = Structures.getMandibleRigidBody();
		Point3d mandibleStartPosition = mandible.getMesh().getVertex(referenceVertexIndex).getPosition();

		TargetSequence ts = InverseTools.createSimpleRigidBodyTargetSequence(mech, mandible, mandibleStartPosition, mandibleTargetPosition, startTime, stopTime);
		addController(new GeneralInverseController(mech, "mandible repositioning inverse model", ts));	
	}
*/
}
