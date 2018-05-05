package artisynth.models.larynx_QL2;

import artisynth.core.workspace.DriverInterface;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.models.larynx_QL2.components.Names.Ligaments;
import artisynth.models.larynx_QL2.components.Names.Muscles;
import artisynth.models.larynx_QL2.components.Names.VocalTractType;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemMiscMethods;
import artisynth.models.larynx_QL2.components.Names;
import artisynth.models.larynx_QL2.components.Structures;

/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class LarynxQL2 extends AbstractSimulation {
	FemMuscleModel larynx, tongue;

	boolean editModeFlag = true;

	String larynxType = "";

	public LarynxQL2(String name) {
		super(name, VocalTractType.LARYNX_QL2);
		larynx = getLarynxFem();
		tongue = getTongueFem();
		setupTissueProperties();
			
		mech.getCollisionManager().setDamping(1.0);
		mech.getCollisionManager().setCompliance(1.0);
		mech.setPenetrationTol(3e-3);
		
		double maxStepSize = 1e-3;
		mech.setMaxStepSize(maxStepSize);
		mech.setGravity(0.0, 0.0, 0.0);
		
		if (larynx != null) {
			FemMiscMethods.setMooneyRivlinMaterial(getLarynxFem(), 1040, 19.0, 0.03, IncompMethod.OFF, IncompMethod.NODAL, 1.037e4, 1.037e3, 4.86e2, 0.0);
			
			larynx.setMaxStepSize(maxStepSize);
			setGravity(false, larynx);
		}

		//Sets muscle force scaling; tweak as desired
		double mforce = 2.55;
		setAxialLaryngealMuscleForces(1.5*mforce, 4.5*mforce, 1.5*mforce, 1.25*mforce, 2.5*mforce, 1.25*mforce, mforce);
		setFemLaryngealMuscleForces(1.25*mforce, 1.25*mforce, mforce, 2.0*mforce, 2.0*mforce, 2.0*mforce);
	}

	public void setupTissueProperties() {
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_LEFT).setDynamic(true);
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_RIGHT).setDynamic(true);

		setMuscleVisiblity(larynx, false);
		setMuscleVisiblity(tongue, false);
		setOrofacialSkeletonAlpha(0.35);
		setIntrinsicLaryngealMuscleVisibility(false);
		
		Structures.getBodyByName(Names.Bodies.ARYTENOID_LEFT).setVisible(false);
		Structures.getBodyByName(Names.Bodies.ARYTENOID_RIGHT).setVisible(false);
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_LEFT).setVisible(false);
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_RIGHT).setVisible(false);
		Structures.getLigamentByName(Ligaments.VOCAL_LIGAMENT_LEFT).setVisible(false);
		Structures.getLigamentByName(Ligaments.VOCAL_LIGAMENT_RIGHT).setVisible(false);
		Structures.getLigamentByName(Ligaments.CRICOTHYROID_MEDIAL).setVisible(false);
		setCricoarytenoidPlaneVisiblity(false);
		setPharynxVisibility(false);

		//setGravity(false, larynx, tongue);

		Structures.getBodyByName(Names.Bodies.ARYTENOID_LEFT).setVisible(false);
		Structures.getBodyByName(Names.Bodies.ARYTENOID_RIGHT).setVisible(false);
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_LEFT).setVisible(false);
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_RIGHT).setVisible(false);
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_LEFT).setDynamic(true);
		Structures.getBodyByName(Names.Bodies.CUNEIFORM_RIGHT).setDynamic(true);
		Structures.getLigamentByName(Ligaments.VOCAL_LIGAMENT_LEFT).setVisible(false);
		Structures.getLigamentByName(Ligaments.VOCAL_LIGAMENT_RIGHT).setVisible(false);
		Structures.getLigamentByName(Ligaments.CRICOTHYROID_MEDIAL).setVisible(false);
		setIntrinsicLaryngealMuscleForceScaling(0.1);	//2.5 works well

		setMuscleForceProperties(Muscles.Lingual.HYOGLOSSUS, 1e2);
		setMuscleForceProperties(Muscles.Lingual.TRANSVERSUS, 1e1);
		setMuscleForceProperties(Muscles.Lingual.GENIOGLOSSUS_ANTERIOR, 1e1);
		setMuscleForceProperties(Muscles.Lingual.GENIOGLOSSUS_MEDIAL, 1e1);
		setMuscleForceProperties(Muscles.Lingual.GENIOGLOSSUS_POSTERIOR, 1e3);
		setMuscleForceProperties(Muscles.Mandibular.MASSETER, 5.0);
		setMuscleForceProperties(Muscles.Mandibular.INTERNAL_PTERYGOID, 5.0);

		setMuscleForceProperties(Muscles.Hyoid.ANTERIOR_DIGASTRIC, 1.75);
		setMuscleForceProperties(Muscles.Hyoid.POSTERIOR_DIGASTRIC, 1.75);

		setMuscleForceProperties(Muscles.Laryngeal.STYLOPHARYNGEUS, 2.5);
		setMuscleForceProperties(Muscles.Pharyngeal.INFERIOR_PHARYNGEAL_CONSTRICTOR, 1.5);
		setMuscleForceProperties(Muscles.Hyoid.STERNOHYOID, 0.3);
		setMuscleForceProperties(Muscles.Hyoid.STERNOTHYROID, 2.75);		

		setMuscleForceProperties(Muscles.Laryngeal.FEM_THYROARYTENOID_MUSCULARIS, 1.0);
		setMuscleForceProperties(Muscles.Laryngeal.FEM_THYROARYTENOID_VOCALIS, 1.0);

	}



	@Override
	public StepAdjustment driverAdvance(double t0, double t1, int flags) {
		return null;
	}

	@Override
	public void driverAttach(DriverInterface driver) {
	   

	}

	@Override
	public void driverDetach(DriverInterface driver) {
		// TODO Auto-generated method stub

	}

	@Override
	public void addInverseController() {
		// TODO Auto-generated method stub

	}

}
