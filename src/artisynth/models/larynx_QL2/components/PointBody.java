package artisynth.models.larynx_QL2.components;

import java.awt.Color;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.ModelComponent;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
/**
  *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class PointBody extends AbstractBody {

	private Particle particle = new Particle();
	private double pointMass = 1e-6;
	private double pointDamping = 1.0;

	PointBody(MechModel mech, String name) {
		super(name, true, true, true, true, 1.0, new RigidTransform3d());
		particle.setName(name);
		addToModel(mech);
	}

	PointBody(MechModel mech, String name, boolean addComponentFlag, boolean addMusclesFlag, boolean dynamicFlag, boolean visibleFlag, double scale, RigidTransform3d worldTransform) {
		super(name, addComponentFlag, addMusclesFlag, dynamicFlag, visibleFlag, scale, worldTransform);
		particle.setName(name);
		addToModel(mech);
	}

	public PointBody(MechModel mech, Names.All name) {
		this(mech, name.getName());
	}

	public Point3d getPosition() { return particle.getPosition(); }

	@Override
	public void printCrashReport() {}
	@Override
	public void printStatistics() {}
	@Override
	public void setVisible(boolean visibleFlag) {
		if (isUsed()) {
			RenderProps.setVisible(particle, visibleFlag);
		}
	}
	@Override
	public Point assemblePoint(MechModel mech, String name, Point3d point) {
		Particle newParticle = new Particle(pointMass);
		newParticle.setName(name + " anchor particle");
		newParticle.setPosition(point);
		newParticle.setPointDamping(pointDamping);
		mech.addParticle(newParticle);

		//Set the location for this pointbody
		particle.setPosition(point);
		mech.attachPoint(newParticle, particle);
		return newParticle;
	}
	@Override
	public void setDynamic(boolean dynamicFlag) {
	       if (isUsed()) {
	              particle.setDynamic(dynamicFlag);
	       }
	}
	@Override
	public PolygonalMesh getMesh() {
		System.err.println("Particles do not have meshes");
		return null; 
	}
	@Override
	public void transformGeometry(AffineTransform3dBase transform) { particle.transformGeometry(transform); }
	@Override
	public void setName(String name) {
		this.name = name;
		particle.setName(name);
	}
	@Override
	public void setLeftRightNames() {

		if (particle.getPosition().x > 0.0 && particle.getName().contains("left")) {
			particle.setName(particle.getName().replace("left", "right"));
			name = particle.getName();
		}
		else if (particle.getPosition().x < 0.0 && particle.getName().contains("right")) {
			particle.setName(particle.getName().replace("right", "left"));
			name = particle.getName();
		}
	}
	@Override
	public void setDensity(double density) {}

	public void addToModel(MechModel mech) { 
		particle.setMass(pointMass);
		particle.setPointDamping(pointDamping);
		mech.addParticle(particle);      	 

		RenderProps.setPointRadius(particle, 1.0);
		RenderProps.setPointColor(particle, Color.orange);
		RenderProps.setVisible(particle, true);
	}

	@Override
	public Point3d getKeyPoint() { return particle.getPosition(); }
	public Particle getParticle() { return particle; }
	
	@Override
	public ModelComponent getModelComponent () {
	       return particle;
	}
}
