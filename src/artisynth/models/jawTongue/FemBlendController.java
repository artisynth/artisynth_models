package artisynth.models.jawTongue;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.properties.PropertyList;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.modelbase.ComponentState;
import artisynth.core.modelbase.CompositeState;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.NumericState;

public class FemBlendController extends ControllerBase {

   PolygonalMesh mesh0;
   PolygonalMesh mesh1;
   PolygonalMesh blendedMesh;
   
   FemModel3d fem;
   Point3d[] femPos0;
   Point3d[] femPos1;
   

   double blendFactor = 0;
   
   public static PropertyList myProps = new PropertyList (
      FemBlendController.class, ControllerBase.class);

   static {
      myProps.add (
         "blendFactor", "number of priciple components used", 0, "[0,1]");
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }
   
   public FemBlendController (FemModel3d fem, File state0, File state1) {
      this.fem = fem;
      this.femPos0 = readNodePos (state0);
      this.femPos1 = readNodePos (state1);
   }
   
   private Point3d[] readNodePos(File stateFile) {
      try {
         ComponentState state = fem.createState (null);
         state.readBinary (new DataInputStream (new FileInputStream (stateFile)));
         fem.setState (state);
         Point3d[] femNodePos = new Point3d[fem.numNodes ()];
         for (int i = 0; i < fem.numNodes (); i++) {
            femNodePos[i] = new Point3d(fem.getNode (i).getPosition ());
         }
         return femNodePos;
      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      return null;
   }
   
   public PolygonalMesh getBlendedMesh() {
      return blendedMesh;
   }

   Point3d pnt = new Point3d();
   @Override
   public void apply (double t0, double t1) {
      for (int i = 0; i < fem.numNodes (); i++) {
         pnt.interpolate (femPos0[i], blendFactor, femPos1[i]);
         fem.getNode (i).setPosition (pnt);
      }
   }

   public double getBlendFactor () {
      return blendFactor;
   }

   public void setBlendFactor (double blendFactor) {
      this.blendFactor = blendFactor;
   }
   
   

}
