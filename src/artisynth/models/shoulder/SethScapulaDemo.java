/**
 * Copyright (c) 2023, by the Authors: John E Lloyd (UBC), Ian Stavness (USask)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 * 
 * Implementation of Ajay Seth's Scapulothoracic Joint from OpenSim: 
 * Seth A, Matias R, Veloso AP and Delp SL. A biomechanical model of the 
 * scapulothoracic joint to accurately capture scapular kinematics during 
 * shoulder movements. PLoS ONE. (2015)
 */

package artisynth.models.shoulder;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.EllipsoidJoint;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.opensim.OpenSimParser;
import artisynth.core.opensim.components.Coordinate;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.render.Renderer.Shading;
import maspack.util.PathFinder;


public class SethScapulaDemo extends RootModel {


   File osimFile = ArtisynthPath.getSrcRelativeFile (this, 
      "osim/ScapulothoracicJoint_Shoulder_nohand_noscapclav.osim");
    
   protected static final double RTOD = 180.0/Math.PI; 
   protected static final double DTOR = Math.PI/180.0; 
    
   // default coordinate values from XXX
   double DEFAULT_SCAPULA_ABDUCTION = RTOD * -0.31156113764349197;
   double DEFAULT_SCAPULA_ELEVATION = RTOD * -0.050497870894562202;
   double DEFAULT_SCAPULA_ROT = RTOD * 0.20820491154386678;
   double DEFAULT_SCAPULA_WINGING = RTOD * 0.27358519522900637;
    
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      String localPath = PathFinder.findSourceDir(SethScapulaDemo.class);
      String geometryPath = localPath + "/osim/Geometry/";
           
      MechModel mech = new MechModel("SethScapula");
      OpenSimParser parser = new OpenSimParser (osimFile);

      parser.setGeometryPath (new File(geometryPath));
      
      parser.createModel (mech);
            
      RenderableComponentList<RigidBody> bodies = 
         (RenderableComponentList<RigidBody>)mech.get ("bodyset");

      RigidBody thorax = bodies.get ("thorax");
      for (RigidBody body : bodies) {
         body.setAxisLength (0.1);
      }
//      scapula.setAxisLength (0.1);
      thorax.setDynamic (false);
      addModel(mech);
      
      EllipsoidJoint joint = (EllipsoidJoint)bodies.get ("scapula").getConnectors ().get (0);

      // set default coorindate values to zero
      joint.setCoordinate (EllipsoidJoint.LONGITUDE_IDX, 0);
      joint.setCoordinate (EllipsoidJoint.LATITUDE_IDX, 0 );
      joint.setCoordinate (EllipsoidJoint.THETA_IDX, 0);
      joint.setCoordinate (EllipsoidJoint.PHI_IDX, 0);
      joint.setCoordinate (EllipsoidJoint.LONGITUDE_IDX, 0); // TODO - why do we need to re-zero X coord?

//      joint.setCoordinate (EllipsoidJoint.X_IDX, DEFAULT_SCAPULA_ABDUCTION);
//      joint.setCoordinate (EllipsoidJoint.Y_IDX, DEFAULT_SCAPULA_ELEVATION );
//      joint.setCoordinate (EllipsoidJoint.THETA_IDX, DEFAULT_SCAPULA_ROT);
//      joint.setCoordinate (EllipsoidJoint.PHI_IDX, DEFAULT_SCAPULA_WINGING);
//      
      System.out.println("scapula_abduction = "+joint.getCoordinate (EllipsoidJoint.LONGITUDE_IDX));
      System.out.println("scapula_elevation = "+joint.getCoordinate (EllipsoidJoint.LATITUDE_IDX));
      System.out.println("scapula_rot = "+joint.getCoordinate (EllipsoidJoint.THETA_IDX));
      System.out.println("scapula_winging = "+joint.getCoordinate (EllipsoidJoint.PHI_IDX));
      
      // create control panel to interactively adjust properties
      ControlPanel panel = new ControlPanel();
      panel.addWidget (joint, "x");
      panel.addWidget (joint, "xRange");
      panel.addWidget (joint, "y");
      panel.addWidget (joint, "yRange");
      panel.addWidget (joint, "theta");
      panel.addWidget (joint, "thetaRange");
      panel.addWidget (joint, "phi");
      panel.addWidget (joint, "phiRange");
      panel.addWidget (joint, "drawFrameC");
      panel.addWidget (joint, "drawFrameD");
      panel.addWidget (joint, "axisLength");
      panel.addWidget (joint, "linearCompliance");
      panel.addWidget (joint, "rotaryCompliance");
      panel.addWidget (joint, "compliance");
      panel.addWidget (joint, "damping");
      addControlPanel (panel);
      
      // set render properties
      RenderProps.setShading(this, Shading.SMOOTH);
      joint.setDrawFrameC (AxisDrawStyle.ARROW);
      joint.setDrawFrameD (AxisDrawStyle.ARROW);
      joint.setAxisLength (0.15);

      System.out.println("done seth scapula");
   }
   
   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);
      
      setDefaultViewOrientation (AxisAngle.ROT_Y_90);
   }
   
}
