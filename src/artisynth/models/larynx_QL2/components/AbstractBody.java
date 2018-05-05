package artisynth.models.larynx_QL2.components;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.ModelComponent;
import artisynth.models.larynx_QL2.components.Structures.HasCommonProperties;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public abstract class AbstractBody implements HasCommonProperties {
	protected String name;
	protected boolean usageFlag;
	protected boolean addMusclesFlag;
	protected boolean dynamicFlag;
	protected boolean visibleFlag;
	protected double scale;
	protected RigidTransform3d worldTransform;

	AbstractBody(String name, boolean usageFlag, boolean addMusclesFlag, boolean dynamicFlag, boolean visibleFlag, double scale, RigidTransform3d worldTransform) {
		this.name = name;
		this.usageFlag = usageFlag;
		this.addMusclesFlag = addMusclesFlag;
		this.dynamicFlag = dynamicFlag;
		this.visibleFlag = visibleFlag;
		this.scale = scale;
		this.worldTransform = worldTransform;
	}

	public abstract void printCrashReport();
	public abstract void printStatistics();
	public abstract Point assemblePoint(MechModel mech, String name, Point3d point);
	public abstract void setDynamic(boolean dynamicFlag);
	public abstract void setVisible(boolean visibleFlag);
	public abstract PolygonalMesh getMesh();
	public abstract void transformGeometry(AffineTransform3dBase transform);
	public abstract void setName(String name);
	public abstract void setDensity(double density);
	public abstract void setLeftRightNames(); 
	public abstract Point3d getKeyPoint();
	public abstract ModelComponent getModelComponent();
	
	public void setUsage(boolean usedFlag) { this.usageFlag = usedFlag; }
	public String getName() { return name; }
	
	public double getScale() { return scale; }
	public RigidTransform3d getWorldTransform() { return worldTransform; }
	public boolean isUsed() { return usageFlag; }
	public boolean musclesEnabled() { return addMusclesFlag; }
	public boolean isDynamic() { return dynamicFlag; }
	public boolean isVisible() { return visibleFlag; }

       

}
