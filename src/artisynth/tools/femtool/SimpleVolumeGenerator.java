package artisynth.tools.femtool;

import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;

public class SimpleVolumeGenerator extends VolumetricMeshGenerator implements HasProperties {

   public enum TemplateType {
      BEAM, CYLINDER, SPINDLE
   }
   public static TemplateType DEFAULT_TEMPLATE = TemplateType.BEAM;
   private TemplateType templateType = DEFAULT_TEMPLATE;
   
   public static PropertyList myProps =
      new PropertyList(SimpleVolumeGenerator.class);
   
   static {
      myProps.add("templateType * *", "", DEFAULT_TEMPLATE);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public Property getProperty(String path) {
      return PropertyList.getProperty(path, this);
   }

   
   public SimpleVolumeGenerator() {
      setTemplateType(DEFAULT_TEMPLATE);
   }
   
   public SimpleVolumeGenerator(TemplateType type) {
      setTemplateType(type);
   }
   
   public TemplateType getTemplateType() {
      return templateType;
   }

   public void setTemplateType(TemplateType templateType) {
      this.templateType = templateType;
   }
   
   public GeneratorType getType() {
      return GeneratorType.SIMPLE;
   }
   
   public static FemModel3d createBoundingBeam(PolygonalMesh surface, int nx, int ny, int nz) {
      
      RigidTransform3d principle = getPrincipalAxes(surface);
      
      Point3d[] tightBox = getTightBox(surface, principle);
      
      double dx = tightBox[0].distance(tightBox[4]);
      double dy = tightBox[0].distance(tightBox[3]);
      double dz = tightBox[0].distance(tightBox[1]);
      FemModel3d model = new FemModel3d();
      FemFactory.createHexGrid(model, dx, dy, dz, nx, ny, nz);
      
      RigidTransform3d trans = new RigidTransform3d(principle);
//      principle.setTranslation(tightBox[6]);
      Vector3d p = new Vector3d(tightBox[0]);
      p.add(tightBox[6]);
      p.scale(0.5);
      trans.setTranslation(p);
      
      model.transformGeometry(trans);
      
      return model;
      
   }

   @Override
   public FemModel3d generate(PolygonalMesh surface, int[] resolution) {
      
      switch(templateType) {
         case BEAM:
            return createBoundingBeam(surface, resolution[0], resolution[1], resolution[2]);
         case CYLINDER:
            return null;
         case SPINDLE:
            return null;
         default:
      }
      
      return null;
      
   }
}
