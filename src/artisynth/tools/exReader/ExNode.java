package artisynth.tools.exReader;

import java.util.ArrayList;

import maspack.matrix.Point3d;

public class ExNode {
   private int idx;

   ArrayList<ExNodeField> myFields;
   ArrayList<ExLine> dependentLines;
   public static final String COORDINATE_FIELD = "coordinates";
   public static final String[] COORDINATE_COMPONENTS = { "x", "y", "z" };

   public ExNode () {
      myFields = new ArrayList<ExNodeField>();
      dependentLines = new ArrayList<ExLine>();
   }

   public ExNode (Point3d pnt) {
      this();

      ExNodeField coordField =
         new ExNodeField(
            COORDINATE_FIELD, "coordinate", "rectangular cartesian");
      double val[] = new double[1];

      for (int i = 0; i < 3; i++) {
         val[0] = pnt.get(i);
         ExNodeComponent comp = new ExNodeComponent(1, 0, val);
         comp.setName(COORDINATE_COMPONENTS[i]);
         coordField.addComponent(comp);
      }
      addField(coordField);

   }

   public int getNodeIdx() {
      return idx;
   }

   public void setIdx(int index) {
      idx = index;
   }

   public void addField(ExNodeField nf) {
      myFields.add(nf);
   }

   public ArrayList<ExNodeField> getFields() {
      return myFields;
   }

   public boolean addDependentLine(ExLine line) {
      if (!dependentLines.contains(line)) {
         dependentLines.add(line);
         return true;
      }
      return false;
   }

   public boolean removeDependentLine(ExLine line) {
      if (dependentLines.contains(line)) {
         dependentLines.remove(line);
         return true;
      }
      return false;
   }

   public ArrayList<ExLine> getDependentLines() {
      return dependentLines;
   }

   public ArrayList<ExFace> getDependentFaces() {
      ArrayList<ExFace> ret = new ArrayList<ExFace>();

      for (ExLine line : dependentLines) {
         ArrayList<ExFace> lf = line.getDependenFaces();

         for (ExFace face : lf) {
            if (!ret.contains(face)) {
               ret.add(face);
            }
         }
      }

      return ret;
   }

   public ArrayList<ExElement> getDependentElements() {
      ArrayList<ExElement> ret = new ArrayList<ExElement>();

      for (ExLine line : dependentLines) {
         ArrayList<ExElement> le = line.getDependentElements();

         for (ExElement elem : le) {
            if (!ret.contains(elem)) {
               ret.add(elem);
            }
         }
      }
      return ret;
   }

   public ExNodeField getField(String name) {
      for (ExNodeField field : myFields) {
         if (name.equals(field.getFieldName())) {
            return field;
         }
      }
      return null;
   }

   public ExNodeComponent getComponent(String fieldName, String componentName) {
      ExNodeField field = getField(fieldName);
      if (field != null) {
         return field.getComponent(componentName);
      }
      return null;
   }

   public void removeCoordinateVersion(int version) {
      
      ExNodeField field = getField(COORDINATE_FIELD);
      for (ExNodeComponent comp : field.getComponents()) {
         comp.removeVersion(version);
      }
      
   }
   
   public void addCoordinateVersion(double[][] vals) {
      
      ExNodeField field = getField(COORDINATE_FIELD);
      ArrayList<ExNodeComponent> comps = field.getComponents();
      int dim = comps.size();
      
      for (int i=0; i<dim; i++) {
         ExNodeComponent comp = field.getComponent(ExNode.COORDINATE_COMPONENTS[i]);
         comp.addVersion(vals[i]);
      }
      
   }
   
   public boolean equalCoordinateVersions(int ver1, int ver2) {
      
      final double F = 1e-5; 
      int nDeriv = getField(COORDINATE_FIELD).getComponent(COORDINATE_COMPONENTS[0]).getNumDerivatives();
      for (int i=0; i<nDeriv+1; i++) {
         double[] pnt1 = getDerivativeCoordinate(ver1, i);
         double[] pnt2 = getDerivativeCoordinate(ver2, i);
         
         for (int j=0; j<pnt1.length; j++) {
            if (Math.abs( (pnt1[j]-pnt2[j]) )/(Math.abs(pnt2[j])+1e-12) > F) {
               return false;
            }
         }
      }
      
      return true;
      
   }
   
   
   public double[] getCoordinate() {

      ExNodeField field = getField("coordinates");
      int dim = field.getComponents().size();
      double coords[] = new double[dim];

      for (int i = 0; i < dim; i++) {
         coords[i] = field.getComponentVal(COORDINATE_COMPONENTS[i]);
      }

      return coords;

   }

   public int getNumCoordinateVersions() {
      return getField(COORDINATE_FIELD).getComponent(COORDINATE_COMPONENTS[0]).getNumVersions();
   }
   
   public int getNumCoordinateDerivatives() {
      return getField(COORDINATE_FIELD).getComponent(COORDINATE_COMPONENTS[0]).getNumDerivatives();
   }
   
   public double[] getDerivativeCoordinate(int versionIdx, int derivIdx) {

      ExNodeField field = getField(COORDINATE_FIELD);
      int dim = field.getComponents().size();
      double coords[] = new double[dim];

      for (int i = 0; i < dim; i++) {
         coords[i] =
            field.getComponentVal(
               COORDINATE_COMPONENTS[i], versionIdx, derivIdx);
      }

      return coords;
   }
   
   public void setDerivativeCoordinate(int versionIdx, int derivIdx, double[] coords) {
      
      ExNodeField field = getField(COORDINATE_FIELD);
      int dim = field.getComponents().size();

      for (int i = 0; i < dim; i++) {
         field.setComponentVal(COORDINATE_COMPONENTS[i], versionIdx, derivIdx, coords[i]);
      }
      
   }

   public boolean isOnSurface() {
      // check if any of the dependent lines are on surface
      for (ExLine line : dependentLines) {
         if (line.isOnSurface()) {
            return true;
         }
      }
      return false;

   }
}
