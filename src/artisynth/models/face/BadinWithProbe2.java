package artisynth.models.face;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.LinkedList;
import java.util.Locale;
import java.util.Scanner;
import java.util.ArrayList;

import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyMode;
import maspack.render.GL.GLViewer;
import maspack.render.RenderProps;
import maspack.render.Renderer.Shading;
import artisynth.core.driver.Main;
import artisynth.core.driver.ViewerManager;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.FungMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.jawTongue.JawHyoidFemMuscleTongue;
import artisynth.models.tongue3d.HexTongueDemo;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.femmodels.MuscleElementDesc;

public class BadinWithProbe2 extends BadinJawFaceDemo {

   public BadinWithProbe2 () {
      super ();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      RigidBody hyoid = myJawModel.rigidBodies ().get ("hyoid");
      hyoid.setDynamic (false);

      RigidBody jaw = myJawModel.rigidBodies().get("jaw");
      jaw.setDynamic (false);

      myJawModel.setGravity(Vector3d.ZERO);

      myJawModel.setStabilization (PosStabilization.GlobalStiffness);
      
      // Refine mesh near the probes
      double probeZone = 22.5;
      FemNode3d nodeA1 = face.getNode(5938);
      RenderProps.setPointColor(nodeA1, Color.RED);
      refineMeshNearProbes(nodeA1, probeZone);
      refineMeshNearProbes(nodeA1, probeZone);
//      refineMeshNearProbes(nodeA1, probeZone);
      
      for (FemNode3d n : face.getNodes()) {
         n.setDynamic(true);
         if ( n.getPosition().distance(nodeA1.getPosition()) > probeZone ) {
            n.setDynamic(false);
            RenderProps.setPointColor (n, Color.CYAN);
         }
         
      }

   }

   private void refineMeshNearProbes(FemNode3d node, double radius){

      LinkedList<HexElement> hexElementsToRefine     = new LinkedList<HexElement>();
      LinkedList<WedgeElement> wedgeElementsToRefine = new LinkedList<WedgeElement>();

      Point3d pA1  = node.getPosition();
      
      for (FemElement3d e : face.getElements()) {
        Point3d elemCentroid  = new Point3d();;
        if(e instanceof HexElement) {
           FemNode3d[] nodes = e.getNodes();
           for (int i = 0; i < e.numNodes(); i++){           
              elemCentroid.add( nodes[i].getRestPosition() );    
           }
           elemCentroid.scale(1.0/8.0);
           if ( pA1.distance(elemCentroid) < radius ) {
              hexElementsToRefine.add ((HexElement)e);
           }
        }
        else if (e instanceof WedgeElement) {
           FemNode3d[] nodes = e.getNodes();
           for (int i = 0; i < e.numNodes(); i++){           
              elemCentroid.add(nodes[i].getRestPosition());    
           }
           elemCentroid.scale(1.0/8.0);
           if ( pA1.distance(elemCentroid) < radius ) {
              wedgeElementsToRefine.add ((WedgeElement)e);
           }
        }
      }

      face.subdivideHexs (hexElementsToRefine);
//      face.subdivideWedge (wedgeElementsToRefine);
      
   }
   
}