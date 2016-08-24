package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.*;

import javax.swing.JComponent;
import javax.swing.JMenuItem;

import java.util.*;

import maspack.matrix.*;
import maspack.geometry.*;
import maspack.properties.*;
import maspack.render.*;
import maspack.render.GL.*;
import artisynth.demos.fem.*;
import artisynth.core.workspace.*;
import artisynth.core.femmodels.*;
import artisynth.core.mechmodels.*;
import artisynth.core.driver.*;
import artisynth.core.gui.*;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.*;
import artisynth.core.util.*;
import artisynth.demos.mech.*;

import java.awt.event.ActionEvent;

public class DeformedJawDemoX extends JawDemo {

   FemModelDeformer myDeformer;
   boolean myDeformed = false;
   PointForce myPointForce;
   ArrayList<FemNode3d> myControlNodes;

   public void build (String[] args) throws IOException {
      super.build (args);

      myDeformer = new FemModelDeformer ("deformer", myJawModel, 10);
      addModel (myDeformer);
      addControlPanel (myDeformer.createControlPanel());

      double maxy = Double.NEGATIVE_INFINITY;
      double miny = Double.POSITIVE_INFINITY;
      for (FemNode3d n : myDeformer.getNodes()) {
         Point3d p = n.getPosition();
         if (p.y < miny) {
            miny = p.y;
         }
         else if (p.y > maxy) {
            maxy = p.y;
         }
      }
      myControlNodes = new ArrayList<FemNode3d>();
      for (FemNode3d n : myDeformer.getNodes()) {
         Point3d p = n.getPosition();
         if (p.y == miny || p.y == maxy) {
            n.setDynamic (false);
         }
         if (p.y == miny) {
            myControlNodes.add (n);
         }
      }

      FrameMarker mkr =
         (FrameMarker)myJawModel.findComponent ("frameMarkers/lowerincisor");
      myPointForce = new PointForce (mkr);
      GLViewer viewer = getMainViewer();
      viewer.setOrthographicView (true);
      setDefaultViewOrientation (
         AxisAlignedRotation.X_Z.getAxisAngle());
      RenderProps.setVisible (myDeformer, false);

      myJawModel.setGravity (new Vector3d());
      myJawModel.addForceEffector (myPointForce);

      RenderProps.setLineStyle (myPointForce, Renderer.LineStyle.CYLINDER);
      RenderProps.setLineRadius (myPointForce, 2.0);
      RenderProps.setLineColor (myPointForce, Color.GREEN);
      myPointForce.setAxisLength (30.0);
   }



   public StepAdjustment advance (double t0, double t1, int flags) {

      double tpan1 = 2.0;
      double tforce1 = 4.0;
      double tgrid = 5.0;
      double tdeform = 6.0;
      double tforce2 = 8.0;

      GLViewer viewer = getMainViewer();

      if (t0 == 0) {
         myJawModel.setDynamicsEnabled (true);
         myDeformer.setDynamicsEnabled (false);
         RenderProps.setVisible (myDeformer, false);

         viewer.setOrthographicView (true);
         setViewerCenter (new Point3d (0, 0, 70.0));
         //setViewerCenter (new Point3d (33.8894, 14.5473, 69.0211));
         setViewerEye (new Point3d (0, -895.0, 70.0));
         viewer.zoom (0.35);
         //setViewerEye (new Point3d (-875.657, 14.5473, 69.0211));
      }
      if (t1 <= tpan1) {
         double ang = (t1/tpan1)*Math.PI/2.0;
         double sin = Math.sin(ang);
         double cos = Math.cos(ang);
         setViewerEye (new Point3d (sin*(-895.0), cos*(-895), 70.0));
      }
      else if (t1 <= tforce1) {
         double s = (t1-tpan1)/(tforce1-tpan1);
         Vector3d fmax = new Vector3d (0, 2000, -2000);
         Vector3d f = new Vector3d();
         double lam = s < 0.5 ? 2*s : 2*(1-s);
         f.scale (lam, fmax);
         myPointForce.setForce (f);
      }
      else if (t1 <= tgrid) {
         if (t0 == tforce1) {
            myJawModel.setDynamicsEnabled (false);
            myDeformer.setDynamicsEnabled (true);
            RenderProps.setVisible (myDeformer, true);
         }
         double s = (t1-tforce1)/(tgrid-tforce1);
         if (s > 0.4) {
            for (FemNode3d n : myControlNodes) {
               Point3d p = n.getPosition();
               p.y -= 0.6;
               p.z += 0.1;
               n.setPosition (p);
            }
         }
      }
      else if (t1 <= tdeform) {
         double s = (t1-tgrid)/(tdeform-tgrid);
         if (s >= 0.5 && !myDeformed) {
            System.out.println ("applying ...");
            FemGeometryTransformer xformer = myDeformer.getTransformer();
            xformer.setUndoState (GeometryTransformer.UndoState.SAVING);
            TransformGeometryContext.transform (myJawModel, xformer, 0);
            System.out.println ("done");
            myDeformed = true;
         }
         else if (t1 == tdeform) {
            myJawModel.setDynamicsEnabled (true);
            myDeformer.setDynamicsEnabled (false);
            //RenderProps.setVisible (myDeformer, false);
         }
      }
      else if (t1 <= tforce2) {
         double s = (t1-tdeform)/(tforce2-tdeform);
         Vector3d fmax = new Vector3d (0, 2000, -2000);
         Vector3d f = new Vector3d();
         double lam = s < 0.5 ? 2*s : 2*(1-s);
         f.scale (lam, fmax);
         myPointForce.setForce (f);
      }
      // else if (t1 <= tpan2) {
      //    double s = (t1-tforce2)/(tpan2-tforce2);
      //    double ang = (1-s)*Math.PI/2.0;
      //    double sin = Math.sin(ang);
      //    double cos = Math.cos(ang);
      //    setViewerEye (new Point3d (sin*(-895.0), cos*(-895), 70.0));
      // }
      
      StepAdjustment sa = super.advance (t0, t1, flags);
      return sa;
   }
}
