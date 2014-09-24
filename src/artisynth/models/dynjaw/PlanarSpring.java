package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Deque;

import javax.media.opengl.GL2;

import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.PolarDecomposition3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SparseBlockMatrix;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.PropertyList;
import maspack.properties.PropertyInfo.Edit;
import maspack.render.GLRenderer;
import maspack.render.RenderProps;
import maspack.render.RenderProps.LineStyle;
import maspack.util.IndentingPrintWriter;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.PlanarComponent;
import artisynth.core.mechmodels.PlanarPoint;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointState;
import artisynth.core.materials.*;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.CompositeComponentBase;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.util.*;

public class PlanarSpring extends AxialSpring implements PlanarComponent,
   TransformableGeometry, ScalableUnits

{
   
   private double myPlaneSize = 1.0;
   private Point3d[] myRenderPnts;
   RigidTransform3d myPlaneToWorld;
   
   boolean myUnilateralP = false;
   boolean myEnabledP = true;
   
   Point3d myTmpPnt = new Point3d();
   PointState myTmpPntState = new PointState();
   
   Plane myPlane;
   
   /*
    * myPnt0 = colliding point
    * myPnt1 = colliding point projected to plane
    */
   
   public static PropertyList myProps =
      new PropertyList(PlanarSpring.class,
         AxialSpring.class);

   static
    { 
      myProps.add (
         "planeSize * *", "renderable size of the plane", 1.0);
      myProps.add (
         "enabled isEnabled *", "spring is enabled", true);
      myProps.add (
         "unilateral isUnilateral *", "planar spring is one-sided", false);
      myProps.get("restLength").setEditing(Edit.Never);
    }

   public PropertyList getAllPropertyInfo()
    {
      return myProps;
    }
   
   public PlanarSpring()
   {
      super();
      myPlaneToWorld = new RigidTransform3d();
      myPlane = new Plane();
      myRenderPnts = new Point3d[4];
      for (int i=0; i<myRenderPnts.length; i++)
      { myRenderPnts[i] = new Point3d(); 
      }
      myRenderProps = createRenderProps();
      setMaterial (new LinearAxialMaterial(10.0, 1.0));
      setRestLength (0);
      //myStiffness = 10.0;
      //myDamping = 1.0;
   }
   
   public PlanarSpring(Point collidingPnt)
   {
      this();
      myPnt0 = collidingPnt;
      myPnt1 = new Point();
   }


   public void applyForces (double t)
   {
      if (isEnabled())
      {        
         computeForce (myTmp);
         myPnt0.subForce (myTmp); // spring pulls toward plane
      }
   }
   
   public void computeForce (Vector3d f)
   {
      super.computeForce(f);
      if (!isActive())
         f.setZero();
   }
   
   protected void updateU ()
   { 
     projectPnt1ToPlane();
     // unit vector points from projPnt to collidingPnt
     myU.sub (myPnt0.getPosition(), myPnt1.getPosition());
     myLength = myU.norm();
     if (myLength != 0)
      { myU.scale (1/myLength);
      }
   }
   
   public void render (GLRenderer renderer, int flags)
   {
      if (!isEnabled())
         return;
      
     Vector3d nrm = new Vector3d(0, 0, 1);
     RigidTransform3d TDW = getPlaneToWorld();

     computeRenderPoints (TDW);
     nrm.transform (TDW);

     GL2 gl = renderer.getGL2().getGL2();
     RenderProps props = myRenderProps;

     renderer.setMaterialAndShading (props, props.getFaceMaterial(), isSelected());
     renderer.setFaceMode (props.getFaceStyle());
     gl.glBegin (GL2.GL_POLYGON);
     gl.glNormal3d (nrm.x, nrm.y, nrm.z);
     for (int i=0; i<myRenderPnts.length; i++)
      { Point3d p = myRenderPnts[i];
        gl.glVertex3d (p.x, p.y, p.z);
      }
     gl.glEnd ();
     renderer.setDefaultFaceMode ();
     renderer.restoreShading (props);
     
     if (isActive())
     {
        renderer.drawLine (
           myRenderProps,
           myPnt1.myRenderCoords,
           myPnt0.myRenderCoords,
           isSelected());
     }

//     renderer.drawPoint(myRenderProps, 
//        myPnt1.myRenderCoords, 
//        isSelected());
   }
   
   protected void computeRenderPoints (RigidTransform3d TDW)
   {
     myRenderPnts[0].set ( myPlaneSize/2,  myPlaneSize/2, 0);
     myRenderPnts[1].set (-myPlaneSize/2,  myPlaneSize/2, 0);
     myRenderPnts[2].set (-myPlaneSize/2, -myPlaneSize/2, 0);
     myRenderPnts[3].set ( myPlaneSize/2, -myPlaneSize/2, 0);
     for (int i=0; i<myRenderPnts.length; i++)
      { myRenderPnts[i].transform (TDW);
      }
   }    
   
   public void updateBounds (Point3d pmin, Point3d pmax)
   {
     computeRenderPoints (getPlaneToWorld());
     for (int i=0; i<myRenderPnts.length; i++)
      { myRenderPnts[i].updateBounds (pmin, pmax); 
      }
   }
   
   protected void projectPnt1ToPlane()
   {
      myPlane.project(myPnt1.getPosition(), 
         myPnt0.getPosition());
      myPnt1.setVelocity(myPnt0.getVelocity()); // XXX velocity should be projected to plane also
//      myPnt1.prerender(null);
   }
   
   public void setPlaneToWorld(RigidTransform3d newX)
   {
      myPlaneToWorld.set(newX);
   }

   public RigidTransform3d getPlaneToWorld()
   {
      return myPlaneToWorld;
   }

   public void setCollidingPnt(Point collidingPnt)
   {
      if (collidingPnt != null)
         myPnt0 = collidingPnt;
   }

   public double getPlaneSize()
   {
      return myPlaneSize;
   }

   public void setPlaneSize(double len)
   {
      myPlaneSize = len;
   }

   public void setUnilateral(boolean unilateral)
   {
      myUnilateralP = unilateral;
   }
   
   public boolean isUnilateral()
   {
      return myUnilateralP;
   }
   
   public boolean isActive()
   {
      return !isUnilateral() || myU.dot(myPlane.getNormal())<0.0;
   }
   
   public void setEnabled(boolean enabled)
   {
      myEnabledP = enabled;
   }
   
   public boolean isEnabled()
   {
      return myEnabledP;
   }
  
   public void transformGeometry(AffineTransform3dBase X)
   {
      transformGeometry(X, this, 0);
   }

  public void transformGeometry (
    AffineTransform3dBase X, TransformableGeometry topObject, int flags)
  {
     myPlane.transform(X);
     
     PolarDecomposition3d PD = new PolarDecomposition3d();
     PD.factorLeft (X.getMatrix());

     RotationMatrix3d R = new RotationMatrix3d();
     Matrix3d P = new Matrix3d();

     PD.getR(R);
     PD.getP(P);

     myPlaneToWorld.R.mul (R, myPlaneToWorld.R);
     myPlaneToWorld.p.mulAdd (
        X.getMatrix(), myPlaneToWorld.p, X.getOffset());
  }
  
  /**
   * Nothing to do for scale mass.
   */
  public void scaleMass(double m)
   { 
   }

  public void scaleDistance (double s)
   { 
     myPlaneSize *= s;
   }

  
  public RenderProps createRenderProps()
  {
    RenderProps props = new RenderProps();
    props.setFaceStyle (RenderProps.Faces.FRONT_AND_BACK);
    props.setFaceColor(Color.WHITE);
    props.setAlpha(0.5);
    props.setLineStyle(LineStyle.LINE);
    props.setLineColor(Color.CYAN);
    props.setLineWidth(2);
//    props.setPointStyle(PointStyle.POINT);
//    props.setPointColor(Color.MAGENTA);
//    props.setPointSize(4);
    return props;
  }
  
   @Override
   public void getHardReferences (List<ModelComponent> refs) {
      /*
       * colliding point is only reference, projected point
       * created locally
       */ 
      super.getHardReferences (refs);
      if (myPnt0 != null)
       { refs.add (myPnt0);
       }
    }
   
   public void printPointReferences (
      PrintWriter pw, CompositeComponent ancestor)
      throws IOException
    { 
      pw.print (ComponentUtils.getWritePathName (
                   ancestor, myPnt0)+" ");
    }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {
      
      rtok.nextToken();      
      if (scanAndStoreReference (rtok, "point", tokens)) {
         return true;
      }
      else if (scanAttributeName (rtok, "planeToWorld")) {
         myPlaneToWorld.scan (rtok);
         myPlane.set (0, 0, 1, 0);
         myPlane.transform (myPlaneToWorld);           
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }
   
   protected boolean postscanItem (
   Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {

      if (postscanAttributeName (tokens, "point")) {
         setFirstPoint (postscanReference (tokens, Point.class, ancestor));
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }
   
   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      pw.println ("point=" +
                  ComponentUtils.getWritePathName (ancestor, myPnt0));
      pw.println ("planeToWorld=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      pw.println(myPlaneToWorld.toString (fmt));
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");
      super.writeItems (pw, fmt, ancestor);
   }   

   public void addPosJacobian(SparseBlockMatrix M, double s)
   {
      // no jacobian information for planar spring
   }

   public void addSolveBlocks(SparseBlockMatrix M)
   {
      // no jacobian information for planar spring
   }

   public void addVelJacobian(SparseBlockMatrix M, double s)
   {
      // no jacobian information for planar spring
   }

   public int getJacobianType()
   {
      // no jacobian information for planar spring
      return Matrix.SPD; 
   }

   
}
