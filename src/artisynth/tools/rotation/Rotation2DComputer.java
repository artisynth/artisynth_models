package artisynth.tools.rotation;

import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import artisynth.core.mechmodels.Frame;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.util.TimeBase;
import artisynth.tools.rotation.RotationAxis.AxisValidity;

public class Rotation2DComputer extends MonitorBase implements RotationAxisComputer {

   public static double DEFAULT_EPSILON = 1e-10;
   public static int DEFAULT_AVERAGE_POINTS = 10;
   public static double DEFAULT_UPDATE_RATE = 0;
   
   public double eps = DEFAULT_EPSILON;
   public int nAverage = DEFAULT_AVERAGE_POINTS;
   public double updateRate = DEFAULT_UPDATE_RATE;
   
   private static class Storage {
      public double t;
      public Vector3d w;
      public Point3d p;
   }
   int storageIdx=0;
   
   public Storage[] storage;

   public RotationAxis2D RA;   
   boolean follow; // not yet implemented

   Frame myFrame = null;

   public Rotation2DComputer(Frame frame, Point3d pnt, Vector3d normal, int nAverage) {
      myFrame = frame;
      
      // initialize plane
      // RA = new PlaneIntersectionRotationAxis(normal, pnt);
      RA = new PlaneProjectionRotationAxis(normal, pnt);
      this.nAverage = nAverage;
      storage = new Storage[nAverage];
   }
   
   public Rotation2DComputer (Frame frame, Point3d pnt, Vector3d normal) {
      this(frame, pnt, normal, DEFAULT_AVERAGE_POINTS);
   }
   
   public Frame getFrame() {
      return myFrame;
   }
   
   public void setNumAverage(int nAverage) {
      this.nAverage = nAverage;
      storage = new Storage[nAverage];
   }
   
   public void apply(double t0, double t1) {

      
      if (updateRate <=0 || TimeBase.modulo(t0, updateRate)==0) {
         Vector3d w = myFrame.getVelocity().w;
   
         if (w.norm ()<eps) {
            RA.invalidate();
            return;
         }
         
         Vector3d v0 = myFrame.getVelocity().v;
         Vector3d p0 = myFrame.getPosition();
         
         Point3d icr_prev = new Point3d(); 
         RA.getICR(icr_prev);
         
         RA.compute (w, v0, p0);
         
         // store information
         Storage storeThis = new Storage();
         storeThis.p = new Point3d();
         RA.getICR(storeThis.p);
         storeThis.t = t0;
         storeThis.w = new Vector3d(w);
         
         storage[storageIdx] = storeThis;
         storageIdx = (storageIdx+1) % nAverage;
      }
   }

   public void setUpdateRate(double rate) {
      updateRate = rate;
   }
   
   public double getUpdateRate() {
      return updateRate;
   }
   
   public void setThreshold(double epsilon) {
      eps = epsilon;
   }

   public AxisValidity getRotationAxis(Vector3d axis, Point3d pnt) {
      
      RA.getDirection (axis);
      return RA.getICR (pnt);

   }
   
   /**
    * Returns time of estimated CoR
    * @param pnt average CoR based on nAverage time steps
    * @return the time of the CoR, which is the mean simulation time
    *    for which we have stored values
    */
   public double getAverageCoR(Point3d pnt) {
      
      double wTotal = 0;
      double minT = Double.POSITIVE_INFINITY;
      double maxT = 0;
      Point3d cor = new Point3d();
      Point3d pMax = cor;
      double wMax = 0;
      
      for (int i=0; i<storage.length; i++) {
         if (storage[i] != null) {
            
            if (storage[i].t < minT) {
               minT = storage[i].t;
            } else if (storage[i].t > maxT) {
              maxT = storage[i].t;
            }
            
            double w = storage[i].w.norm();
            if (w > wMax) {
               wMax = w;
               pMax = storage[i].p;
            }
            wTotal += w;
            cor.scaledAdd(w, storage[i].p);
//            System.out.println("w: " + w + ", p: " + storage[i].p.toString());
            
         }
      }
      
      if (wTotal < eps) {
         cor.set(pMax);
      } else {
         cor.scale(1.0/wTotal);
      }
      
      double t = (maxT+minT)/2; // mean time
      pnt.set(cor);
      
      return t;
      
   }
   
   public RotationAxis getRotationAxis() {
      return RA;
   }

}
