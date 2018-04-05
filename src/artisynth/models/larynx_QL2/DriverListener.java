package artisynth.models.larynx_QL2;

import artisynth.core.workspace.DriverInterface;
import artisynth.core.modelbase.StepAdjustment;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public interface DriverListener {
	public StepAdjustment driverAdvance(double t0, double t1, int flags);
	public void driverAttach(DriverInterface driver);
	public void driverDetach(DriverInterface driver);
	public void addInverseController();
}
