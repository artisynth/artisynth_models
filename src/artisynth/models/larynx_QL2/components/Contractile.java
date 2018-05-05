package artisynth.models.larynx_QL2.components;

import java.awt.Color;
import java.util.ArrayList;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointSpringBase;
import artisynth.core.modelbase.ModelComponent;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class Contractile extends Tissue{
	public PointSpringBase muscle;
	public MuscleExciter exciter; 
	public ArrayList<Ligamentous> ligaments = new ArrayList<Ligamentous>();
	
	public static class MuscleData {
		public ArrayList<ExcitationComponent> excitationSet = new ArrayList<ExcitationComponent>();
		public ArrayList<Contractile> contractileSet = new ArrayList<Contractile>();
		public String name;
		
		public MuscleData(String name, ArrayList<Contractile> contractileSet, ArrayList<ExcitationComponent> excitationSet) {
			this.name = name;
			this.contractileSet = contractileSet;
			this.excitationSet = excitationSet;
		}
		public double getExcitation() {
			if (excitationSet.size() > 0) {
				return excitationSet.get(0).getExcitation();
			}
			else {
				System.err.println("Could not obtain excitation data from muscle " + name);
				return 0.0;
			}
		}
		public double getForce() {
			if (excitationSet.size() > 0) {
				double force = 0.0;
				for (ExcitationComponent ec : excitationSet) {
					if (ec instanceof MuscleBundle) {
						MuscleBundle mb = (MuscleBundle) ec;
						for (Muscle m : mb.getFibres()) {
							double fiberForce = m.getForceNorm();
							if (!Double.isNaN(fiberForce)) {
								force += fiberForce;
							}
							else {
								System.err.println("NaN value detected for force of fiber of muscle '" + mb.getName() + "'");
							}
						}
					}
					else if (ec instanceof AxialSpring) {
						AxialSpring as = (AxialSpring) ec;
						Vector3d forceVector = new Vector3d();
						as.computeForce(forceVector);
						double springForce = forceVector.norm();
						if (!Double.isNaN(springForce)) {
							force += springForce;
						}
						else {
							System.err.println("NaN value detected for force of fiber of muscle '" + as.getName() + "'");
						}
						
					}
				}
				return force;
			}
			else {
				System.err.println("Could not obtain force data from muscle " + name);
				return 0.0;
			}
		}
		
	}
	
	
	public Contractile(String name, String side, MechModel mech, AbstractBody[] bodies, Point3d[] locs, double damping, double passiveFraction, double tendonRatio, double forceScaling) {
		super(name, "muscle", side, bodies, locs);

		originPoint = originBody.assemblePoint(mech, side + " " + name + " origin", originLoc);
		insertionPoint = insertionBody.assemblePoint(mech, side + " " + name + " insertion", insertionLoc);
		restLength = originLoc.copyAndSub(insertionLoc).norm();

		//Determine if this is a single-segment or multi-segment muscle
		if (numberOfPoints > 2) {
			MultiPointMuscle multiPointMuscle = new MultiPointMuscle(side + " " + name);
			points = new Point[numberOfPoints];

			int count = 0;
			for (Point3d loc : locs) {
				String muscleNodeName = side + " " + name + " - Point " + count;

				if (count == 0) {
					points[count] = originBody.assemblePoint(mech, muscleNodeName, loc);
				}
				else if (count == locs.length - 1) {
					points[count] = insertionBody.assemblePoint(mech, muscleNodeName, loc);
				}
				else {
					PointBody pb = new PointBody(mech, muscleNodeName);
					points[count] = pb.assemblePoint(mech, muscleNodeName, loc);
					Structures.addPointBody(pb);

					//Add ligament attachment for the particle (set saveableFlag to false because these tissues should not be saved; they are generated upon instantiation of a saved multipoint contractile tisssue)
					Structures.addTissue(new Ligamentous(muscleNodeName + " (ligament)", "", mech, new AbstractBody[]{pb, Structures.getAnchor()}, new Point3d[]{loc, loc}, genericStiffness, genericDamping, false, false));
				}

				multiPointMuscle.addPoint(points[count]);
				count++;
			}       

			PeckAxialMuscle peckMuscle = new PeckAxialMuscle();
			peckMuscle.setAxialMuscleMaterialProps(forceScaling, multiPointMuscle.getLength(), multiPointMuscle.getLength()*1.2, passiveFraction, tendonRatio, damping, forceScaling);
			multiPointMuscle.setMaterial(peckMuscle);
			//multiPointMuscle.resetLengthProps();

			RenderProps.setVisible(multiPointMuscle, true);
			RenderProps.setLineColor(multiPointMuscle, Color.RED);
			RenderProps.setLineStyle(multiPointMuscle, LineStyle.SPINDLE);
			RenderProps.setLineWidth(multiPointMuscle, 2);

			//Store the tissue type and add it to the model
			muscle = multiPointMuscle;
			mech.addMultiPointSpring(multiPointMuscle);
		}
		else {
			Muscle simpleMuscle = new Muscle(side + " " + name);
			simpleMuscle.setSecondPoint(originPoint);
			simpleMuscle.setFirstPoint(insertionPoint);
			setPoints(originPoint, insertionPoint);

			PeckAxialMuscle peckMuscle = new PeckAxialMuscle();
			peckMuscle.setAxialMuscleMaterialProps(forceScaling, simpleMuscle.getLength(), simpleMuscle.getLength()*1.2, passiveFraction, tendonRatio, damping, forceScaling);
			simpleMuscle.setMaterial(peckMuscle);
			//simpleMuscle.resetLengthProps();

			RenderProps.setVisible(simpleMuscle, true);
			RenderProps.setLineColor(simpleMuscle, Color.RED);
			RenderProps.setLineStyle(simpleMuscle, LineStyle.SPINDLE);
			RenderProps.setLineWidth(simpleMuscle, 2);

			//Store the tissue type and add it to the model
			muscle = simpleMuscle;
			mech.addAxialSpring(simpleMuscle);
		}
	}

	@Override
	public void removeComponents(MechModel mech) {
		if (muscle instanceof MultiPointMuscle) {
			mech.removeMultiPointSpring((MultiPointMuscle) muscle);
		}
		else if (muscle instanceof Muscle) {
			mech.removeAxialSpring((Muscle) muscle);
		}
		
		for (Point point : points) {
			if (point instanceof Particle) {
				mech.removeParticle((Particle) point);
			}
			else if (point instanceof FrameMarker) {
				mech.removeFrameMarker((FrameMarker) point);
			}
		}
	}
	@Override
	public void setVisible(boolean visibleFlag) {
		RenderProps.setVisible(muscle, visibleFlag);
		RenderProps.setVisible(originPoint, visibleFlag);
		RenderProps.setVisible(insertionPoint, visibleFlag);
		
		for (Point point : points) {
			RenderProps.setVisible(point, visibleFlag);
		}
		
		for (Ligamentous ligament : ligaments) {
			ligament.setVisible(visibleFlag);
		}
	}
	
	@Override
	public void setName(String name) {
		this.name = name;
		muscle.setName(name);
	}
	@Override
	public void setLeftRightNames() {
		if ((originPoint.getPosition().x > 0.0 || insertionPoint.getPosition().x > 0.0) && muscle.getName().contains("left")) {
			name.replace("left", "right");
			side = "right";
			muscle.setName(muscle.getName().replace("left", "right"));
			originPoint.setName(originPoint.getName().replace("left", "right"));
			insertionPoint.setName(insertionPoint.getName().replace("left", "right"));
		}
		else if ((originPoint.getPosition().x < 0.0 || insertionPoint.getPosition().x < 0.0) && muscle.getName().contains("right")) {
			name.replace("right", "left");
			side = "left";
			muscle.setName(muscle.getName().replace("right", "left"));
			originPoint.setName(originPoint.getName().replace("right", "left"));
			insertionPoint.setName(insertionPoint.getName().replace("right", "left"));
		}
	}

	@Override
	public void setRenderRadius(double size) {
		RenderProps.setLineRadius(muscle, size);
	}

	public double getDamping() { return ((PeckAxialMuscle) muscle.getMaterial()).getDamping(); } 		
	public double getPassiveFraction() { return ((PeckAxialMuscle) muscle.getMaterial()).getPassiveFraction();	}	
	public double getTendonRatio() { return ((PeckAxialMuscle) muscle.getMaterial()).getTendonRatio(); }	
	public double getForceScaling() { return ((PeckAxialMuscle) muscle.getMaterial()).getForceScaling(); }

	public ModelComponent getModelComponent() { return muscle; }
	
	public static void addExciters(MechModel mech, ArrayList<Contractile> muscles) {
		for (Contractile musc : muscles) {
			if (musc.exciter == null) {
				musc.exciter = new MuscleExciter(musc.name);
				musc.exciter.addTarget((ExcitationComponent) musc.muscle, 1.0);
				if (musc.pairedFlag) {
					musc.exciter.addTarget((ExcitationComponent) ((Contractile) musc.pair).muscle, 1.0);
					((Contractile) musc.pair).exciter = musc.exciter;
				}

				mech.addMuscleExciter(musc.exciter);
			}
		}
	}

	public static void addPair(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertion, Point3d[] points, double forceScaling) {
		Contractile left = new Contractile(name, "left", mech, new AbstractBody[]{origin, insertion}, points, 1e-4, 0.1, 0.1, forceScaling);
		Contractile right = new Contractile(name, "right", mech, new AbstractBody[]{origin, insertion}, right(points), 1e-4, 0.1, 0.1, forceScaling);

		left.pair = right;
		right.pair = left;

		tissues.add(left);
		tissues.add(right);
	}

	public static void addPair(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertionLeft, AbstractBody insertionRight, Point3d[] points, double forceScaling) {
		Contractile left = new Contractile(name, "left", mech, new AbstractBody[]{origin, insertionLeft}, points, muscleDamping, musclePassive, muscleTendoRatio, forceScaling);
		Contractile right = new Contractile(name, "right", mech, new AbstractBody[]{origin, insertionRight}, right(points), muscleDamping, musclePassive, muscleTendoRatio, forceScaling);

		left.pair = right;
		right.pair = left;

		tissues.add(left);
		tissues.add(right);
	}

	public static void addPair(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody originLeft, AbstractBody originRight, AbstractBody insertionLeft, AbstractBody insertionRight, Point3d originLoc, Point3d insertionLoc, double forceScaling) {
		Contractile left = new Contractile(name, "left", mech, new AbstractBody[]{originLeft, insertionLeft}, new Point3d[]{originLoc, insertionLoc}, muscleDamping, musclePassive, muscleTendoRatio, forceScaling);
		Contractile right = new Contractile(name, "right", mech, new AbstractBody[]{originRight, insertionRight}, new Point3d[]{right(originLoc), right(insertionLoc)}, muscleDamping, musclePassive, muscleTendoRatio, forceScaling);

		left.pair = right;
		right.pair = left;

		tissues.add(left);
		tissues.add(right);
	}

	public static ArrayList<Contractile> addPair(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertion, Point3d originLoc, Point3d insertionLoc, double forceScaling) {
		Contractile left = new Contractile(name, "left", mech, new AbstractBody[]{origin, insertion}, new Point3d[]{originLoc, insertionLoc}, muscleDamping, musclePassive, muscleTendoRatio, forceScaling);
		Contractile right = new Contractile(name, "right", mech, new AbstractBody[]{origin, insertion}, new Point3d[]{right(originLoc), right(insertionLoc)}, muscleDamping, musclePassive, muscleTendoRatio, forceScaling);

		left.pair = right;
		right.pair = left;

		tissues.add(left);
		tissues.add(right);
		
		ArrayList<Contractile> pair = new ArrayList<Contractile>();
		pair.add(left);
		pair.add(right);
		return pair;
	}

	public static Contractile add(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertion, Point3d originLoc, Point3d insertionLoc, double forceScaling) {
		Contractile c = new Contractile(name, "", mech, new AbstractBody[]{origin, insertion}, new Point3d[]{originLoc, insertionLoc}, muscleDamping, musclePassive, muscleTendoRatio, forceScaling);
		tissues.add(c);
		return c;
	}

	public void setForceProperties(double forceScaling, double maxForce, double damping, double passiveFraction) {
		PeckAxialMuscle pam = (PeckAxialMuscle) muscle.getMaterial();
		pam.setForceScaling(forceScaling);
		pam.setMaxForce(maxForce);
		pam.setDamping(damping);
		pam.setPassiveFraction(passiveFraction);
	}
	
	public void scaleForce(double forceScaling) {
		PeckAxialMuscle pam = (PeckAxialMuscle) muscle.getMaterial();
		pam.setMaxForce(pam.getMaxForce()*forceScaling);
		pam.setForceScaling(pam.getForceScaling()*forceScaling);
	}

	public String createMuscleForceReport() {
		String muscleForceReport = "";
		PeckAxialMuscle pam = (PeckAxialMuscle) muscle.getMaterial();
		muscleForceReport += "[Max Force = " + String.format("%.3e", pam.getMaxForce()) + " ... ";
		muscleForceReport += "Force Scaling = " + String.format("%.3e", pam.getForceScaling()) + "]";
		return muscleForceReport;
	}

}
