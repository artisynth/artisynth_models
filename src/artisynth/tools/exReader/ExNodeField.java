package artisynth.tools.exReader;

import java.util.ArrayList;

public class ExNodeField {

   private String fieldName;
   private String fieldType;
   private String fieldStructure;

   ArrayList<ExNodeComponent> components;

   public ExNodeField () {
      components = new ArrayList<ExNodeComponent>();
   }

   public ExNodeField (String name, String type, String structure) {
      this();
      setFieldName(name);
      setFieldType(type);
      setFieldStructure(structure);
   }

   public void setFieldName(String name) {
      fieldName = name;
   }

   public void setFieldType(String type) {
      fieldType = type;
   }

   public void setFieldStructure(String structure) {
      fieldStructure = structure;
   }

   public void addComponent(ExNodeComponent comp) {
      components.add(comp);
   }

   public void addComponent(int idx, ExNodeComponent comp) {
      components.add(idx, comp);
   }

   public int getNumComponents() {
      return components.size();
   }

   public String getFieldName() {
      return fieldName;
   }

   public String getFieldType() {
      return fieldType;
   }

   public String getFieldStructure() {
      return fieldStructure;
   }

   public ArrayList<ExNodeComponent> getComponents() {
      return components;
   }

   public ExNodeComponent getComponent(String componentName) {
      for (ExNodeComponent comp : components) {
         if (componentName.equals(comp.getName())) {
            return comp;
         }
      }
      return null;
   }

   public double getComponentVal(String compName, int version) {
      ExNodeComponent comp = getComponent(compName);
      if (comp != null) {
         return comp.getValue(version);
      }
      System.out.println("Error: component '" + compName + "' does not exist!!");
      return 0;
   }

   public double getComponentVal(String compName, int version, int derivIdx) {
      ExNodeComponent comp = getComponent(compName);
      if (comp != null) {
         return comp.get(version, derivIdx);
      }
      System.out.println("Error: component '" + compName + "' does not exist!!");
      return 0;
   }
   
   public void setComponentVal(String compName, int version, int derivIdx, double val) {
      
      ExNodeComponent comp = getComponent(compName);
      if (comp != null) {
         comp.set(version, derivIdx, val);
      }
      
   }

   public double getComponentVal(String compName) {
      return getComponentVal(compName, 0);
   }

}
