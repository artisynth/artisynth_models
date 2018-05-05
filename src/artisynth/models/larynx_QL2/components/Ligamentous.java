package artisynth.models.larynx_QL2.components;

import java.awt.Color;
import java.util.ArrayList;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.AxialSpring;
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
public class Ligamentous extends Tissue {
	public AxialSpring ligament;

	Ligamentous(String name, String side, MechModel mech, AbstractBody[] bodies, Point3d[] locs, double stiffness, double damping, boolean zeroRestLengthFlag, boolean saveableFlag) {
		this(name, side, mech, bodies, locs, stiffness, damping, zeroRestLengthFlag);
		this.saveableFlag = saveableFlag;
	}
	Ligamentous(String name, String side, MechModel mech, AbstractBody[] bodies, Point3d[] locs, double stiffness, double damping, boolean zeroRestLengthFlag) {
		super(name, "ligament", side, bodies, locs);

		//Define ligmament
		originPoint = originBody.assemblePoint(mech, side + " " + name + " origin", originLoc);
		insertionPoint = insertionBody.assemblePoint(mech, side + " " + name + " insertion", insertionLoc);
		setPoints(originPoint, insertionPoint);

		if (!zeroRestLengthFlag) {
			restLength = originLoc.copyAndSub(insertionLoc).norm();
		}
		else {
			restLength = 0.0;
		}
		
		ligament = new AxialSpring(side + " " + name, stiffness, damping, restLength);
		ligament.setFirstPoint(originPoint);
		ligament.setSecondPoint(insertionPoint);

		RenderProps.setVisible(ligament, true);
		RenderProps.setLineStyle(ligament, LineStyle.CYLINDER);
		RenderProps.setLineColor(ligament, Color.WHITE);

		//Store the tissue type and add it to the model
		mech.addAxialSpring(ligament);
	}

	@Override
	public void setVisible(boolean visibleFlag) {
		RenderProps.setVisible(ligament, visibleFlag);

		for (Point point : points) {
			RenderProps.setVisible(point, visibleFlag);
		}
	}
	@Override
	public void setName(String name) {
		this.name = name;
		ligament.setName(name);
	}
	@Override
	public void removeComponents(MechModel mech) {
		mech.removeAxialSpring(ligament);
		
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
	public void setLeftRightNames() {
		
		if ((originPoint.getPosition().x > 0.0 || insertionPoint.getPosition().x > 0.0) && ligament.getName().contains("left")) {
			name.replace("left", "right");
			side = "right";
			ligament.setName(ligament.getName().replace("left", "right"));
			originPoint.setName(originPoint.getName().replace("left", "right"));
			insertionPoint.setName(insertionPoint.getName().replace("left", "right"));
		}
		else if ((originPoint.getPosition().x < 0.0 || insertionPoint.getPosition().x < 0.0) && ligament.getName().contains("right")) {
			name.replace("right", "left");
			side = "left";
			ligament.setName(ligament.getName().replace("right", "left"));
			originPoint.setName(originPoint.getName().replace("right", "left"));
			insertionPoint.setName(insertionPoint.getName().replace("right", "left"));
		}
	}
	@Override
	public void setRenderRadius(double size) {
		RenderProps.setLineRadius(ligament, size);
	}

	public ModelComponent getModelComponent() { return ligament; }
	
	public double getDamping() { return ((LinearAxialMaterial) ligament.getMaterial()).getDamping(); }
	public double getStiffness() { return ((LinearAxialMaterial) ligament.getMaterial()).getStiffness(); }

	public static ArrayList<Ligamentous> addPair(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertion, double stiffness, double damping, boolean zeroRestLengthFlag, Point3d originLoc, Point3d insertionLoc) {
		Ligamentous left = new Ligamentous(name, "left", mech, new AbstractBody[]{origin, insertion}, new Point3d[]{originLoc, insertionLoc}, stiffness, damping, zeroRestLengthFlag);
		Ligamentous right = new Ligamentous(name, "right", mech, new AbstractBody[]{origin, insertion}, right(new Point3d[]{originLoc, insertionLoc}), stiffness, damping, zeroRestLengthFlag);

		left.pair = right;
		right.pair = left;

		tissues.add(left);
		tissues.add(right);
		
		ArrayList<Ligamentous> pair = new ArrayList<Ligamentous>();
		pair.add(left);
		pair.add(right);
		return pair;
	}

	public static void addPair(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertionLeft, AbstractBody insertionRight, double stiffness, double damping, boolean zeroRestLengthFlag, Point3d originLoc, Point3d insertionLoc) {
		Ligamentous left = new Ligamentous(name, "left", mech, new AbstractBody[]{origin, insertionLeft}, new Point3d[]{originLoc, insertionLoc}, stiffness, damping, zeroRestLengthFlag);
		Ligamentous right = new Ligamentous(name, "right", mech, new AbstractBody[]{origin, insertionRight}, right(new Point3d[]{originLoc, insertionLoc}), stiffness, damping, zeroRestLengthFlag);

		left.pair = right;
		right.pair = left;

		tissues.add(left);
		tissues.add(right);
	}

	public static Ligamentous add(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertion, double stiffness, double damping, boolean zeroRestLengthFlag, Point3d originLoc, Point3d insertionLoc) {
		Ligamentous lig = new Ligamentous(name, "", mech, new AbstractBody[]{origin, insertion}, new Point3d[]{originLoc, insertionLoc}, stiffness, damping, zeroRestLengthFlag);
		tissues.add(lig);
		return lig;
	}

	public void setProperties(double stiffness, double damping) {
		PointSpringBase.setStiffness(ligament, stiffness);
		PointSpringBase.setDamping(ligament, damping);
	}
	public double computeForceNorm() {
		Vector3d force = new Vector3d();
		ligament.computeForce(force);
		Vector3d dir = ligament.getDir();
		return force.dot(dir);
	}

	/** If the ligament has a rest length other than zero, return -1 to signify that rest length will be calculate using the starting length. **/
	public String getRestLength() {
		if (ligament.getRestLength() > 0.0) {
			return Double.toString(-1.0);
		}
		else {
			return Double.toString(0.0);
		}
	}
	public double computeLength() {
		return ligament.getLength();
	}



}
