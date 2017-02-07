package artisynth.models.dynjaw;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Map;
import java.util.List;

import maspack.geometry.GeometryTransformer;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.PolarDecomposition3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.Renderer;
import maspack.render.GL.GLViewer;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;
import maspack.util.ReaderTokenizer;
import artisynth.core.driver.Main;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.CompositeComponentBase;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;

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
      RenderProps.setFaceStyle (this, FaceStyle.FRONT_AND_BACK);
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

   public void transformGeometry (
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {
      
      /*
       * only transform attach frame B because attach frame A 
       * is defined by the frame marker on frame A
       */
      gtr.transform (myX2B);
   }   
   
   public void addTransformableDependencies (
      TransformGeometryContext context, int flags) {
      // no dependencies
   }

   public void transformGeometry(AffineTransform3dBase X)
   {
      TransformGeometryContext.transform (this, X, 0);
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
   
   public void updateBounds (Vector3d pmin, Vector3d pmax)
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

   public void render (Renderer renderer, int flags)
   { 
     renderer.setPropsShading (myRenderProps);
     renderer.setFaceColoring (myRenderProps, isSelected());
     renderer.setFaceStyle (FaceStyle.FRONT_AND_BACK);
     renderer.setLineWidth (myRenderProps.getLineWidth());
     drawOrthoLines (renderer, myRenderXDW, myRenderProps);
     renderer.drawLine (
        myRenderProps, myRenderOriginC, myRenderOriginD, isSelected ());
   }
   
   private void drawOrthoPlanes (
      Renderer renderer, RigidTransform3d XFrameToWorld)
    { 

       X.p.set (XFrameToWorld.p);
       X.R.mul (XFrameToWorld.R, Rx);
       drawPlane(renderer, X, red);
       X.R.mul (XFrameToWorld.R, Ry);
       drawPlane(renderer, X, green);
       drawPlane(renderer, XFrameToWorld, blue); // normal == z-axis
    }
   
   private void drawPlane (
      Renderer renderer, RigidTransform3d XFrameToWorld, float[] color)
    { 
       renderer.pushModelMatrix();
       renderer.mulModelMatrix (XFrameToWorld);
       renderer.beginDraw (Renderer.DrawMode.TRIANGLES);
       renderer.setNormal (0f, 0f, 1f);
       renderer.setColor (color[0], color[1], color[2]);
       renderer.addVertex (planeRenderPnts[0]);
       renderer.addVertex (planeRenderPnts[1]);
       renderer.addVertex (planeRenderPnts[2]);
       renderer.addVertex (planeRenderPnts[0]);
       renderer.addVertex (planeRenderPnts[2]);
       renderer.addVertex (planeRenderPnts[3]);
       renderer.endDraw();
       renderer.popModelMatrix();
    }
   
   private void drawOrthoLines (
      Renderer renderer, RigidTransform3d XFrameToWorld, RenderProps props)
    { 
       renderer.pushModelMatrix();
       Shading savedShading = renderer.setShading (Shading.NONE);
       renderer.mulModelMatrix (XFrameToWorld);
      renderer.setLineWidth (props.getLineWidth());
      renderer.beginDraw (Renderer.DrawMode.LINES);
      renderer.setLineColoring (props, isSelected());
      float l = (float)myPlaneSize/2;
      renderer.addVertex (0f, 0f, l);
      renderer.addVertex (0f, 0f, -l);
      renderer.addVertex (0f, l, 0f);
      renderer.addVertex (0f, -l, 0f);
      renderer.addVertex (l, 0f, 0f);
      renderer.addVertex (-l, 0f, 0f);
      renderer.endDraw();
      renderer.setShading (savedShading);
      renderer.popModelMatrix();
    }
}
