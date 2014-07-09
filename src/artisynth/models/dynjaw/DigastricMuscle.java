package artisynth.models.dynjaw;

import artisynth.core.materials.AxialMuscleMaterial;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;

public class DigastricMuscle extends MultiPointMuscle {

   /*
    * specialized multi-point muscle used for modeling the digastric
    * the anterior belly is activated and the posterior belly is passive
    */
   public DigastricMuscle(String name, Muscle anteriorBelly, Muscle posteriorBelly) {
      super(name);
      addPoint (anteriorBelly.getFirstPoint ()); // jaw attach
      addPoint (anteriorBelly.getSecondPoint ()); // hyoid attach
      addPoint (posteriorBelly.getFirstPoint ()); // maxilla attach
      setSegmentPassive (1, true);
      setMuscleProps(anteriorBelly);
   }
   
   private void setMuscleProps(Muscle muscle) {
//      myType = muscle.getMuscleType ();
//      myOptLength = muscle.getOptLength ();
//      myMaxLength = muscle.getMaxLength ();
//      myPassiveFraction = muscle.getPassiveFraction ();
//      myTendonRatio = muscle.getTendonRatio ();
//      setForceScaling (muscle.getForceScaling ());
//      setMaxForce(muscle.getMaxForce ());
//      setDamping (muscle.getDamping ());
      if (muscle.getMaterial() instanceof AxialMuscleMaterial) {
         setMaterial(muscle.getMaterial());
      }
      else {
         System.err.println("setMuscleProps(), muscle's material not AxialMuscleMaterial");
      }
	 
   }
   
   // /*
   //  * only use anterior belly to compute lengths
   //  * 
   //  */
   // public void applyForces (double t) {
   //    updateSegsIfNecessary();
   //    Point pnt0 = getPoint (0);
   //    Point pnt1 = getPoint (1);
   //    double len = computeU (myTmp, pnt0, pnt1);
   //    double dldt = myTmp.dot (pnt1.getVelocity()) - myTmp.dot (pnt0.getVelocity());

   //    double F = computeF (len, dldt);
   //    for (int i = 0; i < myPnts.size() - 1; i++) {
   //       pnt0 = getPoint (i);
   //       pnt1 = getPoint (i + 1);
   //       computeSegmentForce (myTmp, i, F);
   //       pnt0.addForce (myTmp);
   //       pnt1.subForce (myTmp);
   //    }
   //    myForce.set (myTmp);
   // }
   
   // public double getActiveLength() {
   //    updateSegsIfNecessary();
   //    return getPoint(0).distance (getPoint(1));
   // }
   
}
