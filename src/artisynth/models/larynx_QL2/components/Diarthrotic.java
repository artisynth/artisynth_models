package artisynth.models.larynx_QL2.components;

import java.awt.Color;
import java.util.ArrayList;

import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RevoluteJoint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.modelbase.ModelComponent;
import artisynth.models.larynx_QL2.VocalTractBase;
//import artisynth.models.larynx_QL2.tools.GeometryTools;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderable;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class Diarthrotic extends Tissue {
	private BodyConnector constraint;
	private double minAngle, maxAngle, angle, size;

	/** Creates a simple revolute joint. Angles must be given in degrees. **/
        public Diarthrotic(String name, String side, MechModel mech, AbstractBody[] bodies, Point3d[] locs, double linearCompliance, double rotaryCompliance, double minAngle, double maxAngle) {
                super(name, "joint", side, bodies, locs);
                this.minAngle = minAngle;
                this.maxAngle = maxAngle;

                if (originBody instanceof HardBody && insertionBody instanceof HardBody) {
                        RigidBody originRB = ((HardBody) originBody).getBody();
                        RigidBody insertionRB = ((HardBody) insertionBody).getBody();

                        Point3d mid = new Point3d(originLoc);
                        mid.add(insertionLoc);
                        mid.scale(0.5);

                        Point3d diff = new Point3d(originLoc);
                        diff.sub(insertionLoc);
                        double axisLength = diff.norm();
                        diff.normalize();

                        RigidTransform3d jointTransform = new RigidTransform3d();
                        jointTransform.p.set(mid);
                        jointTransform.R.rotateZDirection(diff);
                        RevoluteJoint newJoint = new RevoluteJoint(originRB, insertionRB, jointTransform);

                        newJoint.setName(side + " " + name);
                        newJoint.setAxisLength(axisLength);
                        newJoint.setMinTheta(minAngle);
                        newJoint.setMaxTheta(maxAngle);
                        newJoint.setLinearCompliance(linearCompliance);
                        newJoint.setRotaryCompliance(rotaryCompliance);
                        
                        //Temporary fix: Numerical errors when two revolute joints are used; right-side (arbitrary) joint disabled; rotation still works
                        if (name.contains("cricothyroid joint") && side.contains("right")) {
                           newJoint.setEnabled(false);
                        }
                        
                        constraint = newJoint;
                        mech.addBodyConnector(constraint);

                }
                else {
                        System.err.println("A joint can only be attached to rigid bodies.");
                }
        }

	
	/** Creates a simple planar constraint. Angle must be given in degrees. The first point in locs stores the plane center. The second point stores the plane normal. **/
	public Diarthrotic(String name, String side, AbstractBody[] bodies, Point3d[] locs, double angle, double size, double compliance) {
		super(name, "plane", side, bodies, locs);
		this.angle = angle;
		this.size = size;

		RigidBody originRB = ((HardBody) originBody).getBody();
		RigidBody insertionRB = ((HardBody) insertionBody).getBody();
		
		PlanarConnector plane = new PlanarConnector(insertionRB, new Vector3d(originLoc), originRB, new RigidTransform3d(originLoc, new AxisAngle(insertionLoc, angle)));
		plane.setName(side + " " + name);
		plane.setPlaneSize(size);
		plane.setLinearCompliance(compliance);
		constraint = plane;
		VocalTractBase.mech.addBodyConnector(plane);
	}


	public Point3d[] getLocations() {
		if (type.equals("joint")) {
			return new Point3d[]{originLoc, insertionLoc};
		}
		else {
			return new Point3d[]{new Point3d(constraint.getCurrentTDW().p), new Point3d(constraint.getCurrentTDW().R.getAxisAngle().axis)};
		}
	}
	public ModelComponent getModelComponent() { return constraint; }
	
	public double getLinearCompliance() { return constraint.getLinearCompliance(); }
	public double getRotatryCompliance() { return constraint.getRotaryCompliance(); }
	public double getMinAngle() { return minAngle; }
	public double getMaxAngle() { return maxAngle; }
	public double getSize() { return size; }
	public double getAngle() { 
		if (type.equals("joint")) {
			return angle;
		}
		else {
			return constraint.getCurrentTDW().R.getAxisAngle().angle;
		}
	}
	public Renderable getConstraint() { return constraint; }

	public void setLinearCompliance(double compliance) { constraint.setLinearCompliance(compliance); }
	public void setRotaryCompliance(double rotaryCompliance) { constraint.setRotaryCompliance(rotaryCompliance); }
	public void setScale(double size) {
		this.size = size;
		((PlanarConnector) constraint).setPlaneSize(size); 
	}
	
	public void setColor(Color color, double alpha) {
		constraint.getRenderProps().setAlpha(alpha);
		if (constraint instanceof JointBase) {
			constraint.getRenderProps().setLineColor(color);
		}
		else if (constraint instanceof PlanarConnector) {
			constraint.getRenderProps().setFaceColor(color);
		}
	}

	@Override
	public void setVisible(boolean visibleFlag) {
		constraint.getRenderProps().setVisible(visibleFlag);
	}
	@Override
	public void setName(String name) {
		this.name = name;
		constraint.setName(name);
	}
	@Override
	public void removeComponents(MechModel mech) {
		mech.removeBodyConnector(constraint);

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

		if ((originPoint.getPosition().x > 0.0 || insertionPoint.getPosition().x > 0.0) && constraint.getName().contains("left")) {
			name.replace("left", "right");
			side = "right";
			constraint.setName(constraint.getName().replace("left", "right"));
			originPoint.setName(originPoint.getName().replace("left", "right"));
			insertionPoint.setName(insertionPoint.getName().replace("left", "right"));
		}
		else if ((originPoint.getPosition().x < 0.0 || insertionPoint.getPosition().x < 0.0) && constraint.getName().contains("right")) {
			name.replace("right", "left");
			side = "left";
			constraint.setName(constraint.getName().replace("right", "left"));
			originPoint.setName(originPoint.getName().replace("right", "left"));
			insertionPoint.setName(insertionPoint.getName().replace("right", "left"));
		}
	}
	@Override
	public void setRenderRadius(double size) {
		RenderProps.setLineRadius(constraint, size);
	}

	public void setRenderProperties(double radiusSize, Color color) {
		if (type.equals("joint")) {
			constraint.getRenderProps().setPointRadius(radiusSize);	
		}
		else if (type.equals("plane")) {
			constraint.getRenderProps().setAlpha(0.8);
		}
		constraint.getRenderProps().setLineColor(color);

	}


	public static Diarthrotic addSinglePlane(ArrayList<Tissue> tissues, String name, MechModel mech, AbstractBody origin, AbstractBody insertion, Point3d location, Point3d normal) {
		Diarthrotic diarthrotic = new Diarthrotic(name, "", new AbstractBody[]{origin, insertion}, new Point3d[]{location, normal}, 0.0, 1.0, 0.0);
		tissues.add(diarthrotic);
		return diarthrotic;
	}




}
