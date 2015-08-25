package artisynth.models.dynjaw;

import java.io.IOException;
import java.io.PrintWriter;

import javax.media.opengl.GL2;

import maspack.matrix.AffineTransform3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.PolarDecomposition3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.properties.PropertyList;
import maspack.render.GLRenderer;
import maspack.render.GLViewer;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.RenderProps.Faces;
import maspack.util.ReaderTokenizer;
import artisynth.core.driver.Main;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBodyConnector;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.CompositeComponentBase;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.util.TransformableGeometry;

public class ScarSpring extends OrthoSpring implements TransformableGeometry
{

   protected FrameMarker collidingPointA;

   public static final double defaultPlaneSize = 1.0;
   protected double myPlaneSize;
   
   private Point3d[] planeRenderPnts = new Point3d[4];
   float[] myRenderOriginC = new float[3];
   float[] myRenderOriginD = new float[3];
   
   public static PropertyList myProps =
      new PropertyList(ScarSpring.class, OrthoSpring.class);

   private RigidTransform3d X = new RigidTransform3d();
   private RotationMatrix3d Rx = new RotationMatrix3d(
      new AxisAngle(0,1,0, Math.PI/2));
   private RotationMatrix3d Ry = new RotationMatrix3d(
      new AxisAngle(1,0,0, Math.PI/2));
   private RotationMatrix3d Rz = new RotationMatrix3d();
   private float[] red = new float[]{1f,0f,0f};
   private float[] green = new float[]{0f,1f,0f};
   private float[] blue = new float[]{0f,0f,1f};
   
   static
    { 
      myProps.add (
         "planeSize * *", "size for renderer planes", 1.0);
    }

   public PropertyList getAllPropertyInfo()
    {
      return myProps;
    }
   
   public ScarSpring()
   {
      super ();
//      if (myX1A != null)
//         myX1A.setIdentity ();
//      else
         myX1A = new RigidTransform3d();
      
      myX2B = new RigidTransform3d();
      for (int i=0; i<planeRenderPnts.length; i++)
       { planeRenderPnts[i] = new Point3d();
       }
      setPlaneSize (defaultPlaneSize);
      RenderProps.setFaceStyle (this, Faces.FRONT_AND_BACK);
   }

   public ScarSpring(String name)
   {
      this();
      setName (name);
   }
   
   /*
    * colliding point defines frame A
    */
   public void setCollidingPoint(FrameMarker m)
   {
      collidingPointA = m;
      setFrameA (m.getFrame ());
      RigidTransform3d newCA = new RigidTransform3d();
      newCA.set (myFrameA.getPose ());
      newCA.p.set (m.getLocation ());
      setAttachFrameA (newCA);
   }
   
   

   @Override
   public void applyForces (double t)
   {
      if (collidingPointA != null)
      {
         if (collidingPointA.getFrame () != myFrameA)
         {
            setFrameA(collidingPointA.getFrame ());
         }
         // update CA incase frame marker location has changed
         myX1A.set (myFrameA.getPose ());
         myX1A.p.set (collidingPointA.getLocation ());
      }
      super.applyForces (t);
   }

   public void transformGeometry(AffineTransform3dBase X,
      TransformableGeometry topObject, int flags)
   {
      /*
       * only transform attach frame B because attach frame A 
       * is defined by the frame marker on frame A
       */
      AffineTransform3d Xlocal = new AffineTransform3d();
      myX2B.mulAffineLeft (X, Xlocal.A);
   }

   public void transformGeometry(AffineTransform3dBase X)
   {
      transformGeometry (X, this, 0);
   }
   
   public double getPlaneSize()
   {
      return myPlaneSize;
   }

   public void setPlaneSize(double planeSize)
   {
      this.myPlaneSize = planeSize;
      computeRenderPoints ();
   }
   
   private void computeRenderPoints ()
   {
     planeRenderPnts[0].set ( myPlaneSize/2,  myPlaneSize/2, 0);
     planeRenderPnts[1].set (-myPlaneSize/2,  myPlaneSize/2, 0);
     planeRenderPnts[2].set (-myPlaneSize/2, -myPlaneSize/2, 0);
     planeRenderPnts[3].set ( myPlaneSize/2, -myPlaneSize/2, 0);
//     for (int i=0; i<planeRenderPnts.length; i++)
//      { planeRenderPnts[i].transform (X);
//      }
   }   
   
   public void updateBounds (Point3d pmin, Point3d pmax)
   {
      Point3d tmp = new Point3d();
      for (int i=0; i<planeRenderPnts.length; i++)
      { tmp.set (planeRenderPnts[i]);
        tmp.transform (myRenderXDW);
        tmp.updateBounds (pmin, pmax);
      }
   }
   
   

   @Override
   public void prerender(RenderList list)
   {
      super.prerender (list);
      myRenderOriginC[0] = (float)myRenderXCW.p.x;
      myRenderOriginC[1] = (float)myRenderXCW.p.y;
      myRenderOriginC[2] = (float)myRenderXCW.p.z;
      myRenderOriginD[0] = (float)myRenderXDW.p.x;
      myRenderOriginD[1] = (float)myRenderXDW.p.y;
      myRenderOriginD[2] = (float)myRenderXDW.p.z;

   }

   public void render (GLRenderer renderer, int flags)
   { 
     renderer.setMaterialAndShading (myRenderProps, myRenderProps.getFaceMaterial(), isSelected());
     renderer.setFaceMode (Faces.FRONT_AND_BACK);
//     renderer.setFaceMode (myRenderProps.getFaceStyle());
     renderer.setLineWidth (myRenderProps.getLineWidth());
//     Frame.drawAxes (gl, myRenderXCW, 1f);
//     Frame.drawAxes (gl, myRenderXDW, 1f);
//     drawOrthoPlanes (renderer, myRenderXDW);
     drawOrthoLines (renderer, myRenderXDW, myRenderProps);
     renderer.drawLine (
        myRenderProps, myRenderOriginC, myRenderOriginD, isSelected ());
     renderer.setLineWidth (1);
     renderer.restoreShading (myRenderProps);
   }
   
   private void drawOrthoPlanes (
      GLRenderer renderer, RigidTransform3d XFrameToWorld)
    { 

       X.p.set (XFrameToWorld.p);
       X.R.mul (XFrameToWorld.R, Rx);
       drawPlane(renderer, X, red);
       X.R.mul (XFrameToWorld.R, Ry);
       drawPlane(renderer, X, green);
       drawPlane(renderer, XFrameToWorld, blue); // normal == z-axis
    }
   
   private void drawPlane (
      GLRenderer renderer, RigidTransform3d XFrameToWorld, float[] color)
    { 
      GL2 gl = renderer.getGL2 ();
      gl.glPushMatrix();
//      renderer.setLightingEnabled (false);
      GLViewer.mulTransform (gl, XFrameToWorld);
      gl.glBegin (GL2.GL_POLYGON);
      gl.glNormal3f (0f, 0f, 1f);
      renderer.setColor (color[0], color[1], color[2]);
      for (int i=0; i<planeRenderPnts.length; i++)
      { Point3d p = planeRenderPnts[i];
        gl.glVertex3d (p.x, p.y, p.z);
      }
      gl.glEnd();
//      renderer.setLightingEnabled (true);
      gl.glPopMatrix();
    }
   
   private void drawOrthoLines (
      GLRenderer renderer, RigidTransform3d XFrameToWorld, RenderProps props)
    { 
      GL2 gl = renderer.getGL2 ();
      gl.glPushMatrix();
      renderer.setLightingEnabled (false);
      GLViewer.mulTransform (gl, XFrameToWorld);
      renderer.setLineWidth (props.getLineWidth());
      gl.glBegin (GL2.GL_LINES);
      renderer.setColor (props.getLineColorArray(), isSelected());
      float l = (float)myPlaneSize/2;
      gl.glVertex3f (0f, 0f, l);
      gl.glVertex3f (0f, 0f, -l);
      gl.glVertex3f (0f, l, 0f);
      gl.glVertex3f (0f, -l, 0f);
      gl.glVertex3f (l, 0f, 0f);
      gl.glVertex3f (-l, 0f, 0f);
      gl.glEnd();
      renderer.setLightingEnabled (true);
      gl.glPopMatrix();
    }
}
