package artisynth.tools.exReader;

import java.util.ArrayList;

public class ExNodeComponent {

   private ArrayList<double[]> vals;
   private int nDerivatives;
   private int nVersions;
   private String name;

   public int getNumDerivatives() {
      return nDerivatives;
   }

   public int getNumVersions() {
      return nVersions;
   }

   public ExNodeComponent () {
      setData(1, 0, new double[1]);
   }
   
   public void removeVersion(int idx) {
      vals.remove(idx);
   }
   
   public void removeVersion(double[] val) {
      vals.remove(val);
   }
   
   public void addVersion(double[] val) {
      vals.add(val);
   }

   public ExNodeComponent (int nVer, int nDeriv, double data[], int offset) {
      setData(nVer, nDeriv, data, offset);
   }

   public ExNodeComponent (int nVer, int nDeriv, double data[]) {
      setData(nVer, nDeriv, data);
   }

   public double getValue(int version) {
      double[] val = vals.get(version);
      return val[0];
   }

   double getDerivative(int version, int dIdx) {
      double[] val = vals.get(version);
      return val[dIdx];
   }
   
   void setDerivative(int version, int dIdx, double val) {
      double[] entries = vals.get(version);
      entries[dIdx] = val;
   }

   String getName() {
      return name;
   }

   double get(int version, int idx) {
      return getDerivative(version, idx);
   }
   
   void set(int version, int idx, double val) {
      setDerivative(version, idx, val);
   }

   double get(int idx) { // defaults to version 0
      return vals.get(0)[idx];
   }

   public void setName(String compName) {
      name = compName;
   }

   public void setData(int nVer, int nDeriv, double data[]) {
      setData(nVer, nDeriv, data, 0);
   }

   public void setData(int nVer, int nDeriv, double data[], int offset) {
      nVersions = nVer;
      nDerivatives = nDeriv;
      vals = new ArrayList<double[]>();
      
      for (int i=0; i<nVersions; i++) {
         double [] val = new double[nDeriv+1];
         for (int j=0; j<nDeriv+1; j++) {
            val[j] = data[i*(nDeriv+1) + j + offset];
         }
         vals.add(val);
      }
      
   }

}
