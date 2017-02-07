package artisynth.models.dynjaw;

import java.util.LinkedList;

import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.IsRenderable;
import maspack.render.Renderer;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.spatialmotion.Twist;
import maspack.spatialmotion.Wrench;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.util.ScalableUnits;

public class OrthoSpring extends FrameSpring 
   implements ScalableUnits, RenderableComponent
{
   public static final Vector3d defaultStiffnessVec = new Vector3d(0,0,0);
   protected Vector3d myStiffnessVec = new Vector3d();
   protected RigidTransform3d myRenderXCW = new RigidTransform3d();
   protected RigidTransform3d myRenderXDW = new RigidTransform3d();
   
   protected Wrench myFrameAForce = new Wrench();
   
//   private Matrix3d myKmat = new Matrix3d();
//   private Wrench myF = new Wrench();
//   private Wrench myFTmp = new Wrench();
//   private Vector3d myU = new Vector3d();
//   private AxisAngle myAxisAngle = new AxisAngle();
//
//   private Twist myVelA = new Twist();
//   private Twist myVelB = new Twist();
//   private Twist myVTmp = new Twist();

//   private RigidTransform3d myDC = new RigidTransform3d();

   public static PropertyList myProps =
      new PropertyList(OrthoSpring.class,
                       FrameSpring.class);

   static
    { 
//      myProps.add (
//         "renderProps * *", "renderer properties", null);
//      myProps.add (
//         "stiffnessVector * *", "vector of orthogonal linear spring stiffnesses", defaultStiffnessVec);
      myProps.addReadOnly ("force", "total force wrench applied to frameA");
    }

   public PropertyList getAllPropertyInfo()
    {
      return myProps;
    }
   
   public OrthoSpring ()
   {
      this(null);
   }
   
   public OrthoSpring (String name)
    {
      super (name);
      myRenderProps = createRenderProps ();
//      /setStiffnessVector (defaultStiffnessVec);
    }

//   public OrthoSpring (
//      String name, Vector3d kvec, double kRot, double d, double dRot)
//    { 
//      super (name);
//      myRenderProps = createRenderProps ();
//      setStiffnessVector (kvec);
//      myDamping = d;
//      myRotaryStiffness = kRot;
//      myRotaryDamping = dRot;
//    }

//   public Vector3d getStiffnessVector ()
//    { 
//      return myStiffnessVec;
//    }
//
//   public void setStiffnessVector (Vector3d k)
//    {
//      myStiffnessVec.set (k);
//    }
//   
//   public void setStiffness(double k)
//   {
//      super.setStiffness (k);
//      myStiffnessVec.set (k, k, k);
//   }
   
   public Wrench getForce()
   {
      return myFrameAForce;
   }

   public void applyForces (double t) {

      computeForces();
      
      myFTmp.transform (myX1A, myF);
      myFTmp.transform (myFrameA.getPose().R); // put into rotated world coords
      myFrameA.addForce (myFTmp);
      myFrameAForce.set (myFTmp);
      myF.inverseTransform (myX21.R);
      myFTmp.transform (myX2B, myF);
      myFTmp.transform (myFrameB.getPose().R); // put into rotated world coords
      myFTmp.negate();
      myFrameB.addForce (myFTmp);
   }

//   public void applyForces (double t, StepAdjustment stepAdjust)
//   {
//     myDC.mulInverseBoth (myX1A, myFrameA.getPose());
//     myDC.mul (myFrameB.getPose());
//     myDC.mul (myX2B);
//
//     myU.set (myDC.p);
//     myDC.R.getAxisAngle (myAxisAngle);
//
////     double len = myU.norm();
////     if (len != 0)
////      { myU.scale (1/len);
////      }
////     myF.f.scale (myStiffness*len, myU);
//     myKmat.setZero ();
//     myKmat.setDiagonal (myStiffnessVec);
//     myF.f.mul (myKmat, myU);
//     myF.m.scale (myRotaryStiffness*myAxisAngle.angle, myAxisAngle.axis);
//
////      System.out.println ("axis=" + myAxisAngle.axis.toString("%8.3f") +
////                          " ang=" + Math.toDegrees(myAxisAngle.angle));
////      System.out.println ("f=" + myF.toString("%8.3f"));
//
//     myFrameA.getBodyVelocity (myVelA);
//     myFrameB.getBodyVelocity (myVelB);
//     myVTmp.sub (myVelB, myVelA);
//     myF.f.scaledAdd (myDamping, myVTmp.v);
//     myF.m.scaledAdd (myRotaryDamping, myVTmp.w);
//
//     myVelA.inverseTransform (myX1A);
//
//     myFTmp.transform (myX1A, myF);
//     myFTmp.transform (myFrameA.getPose().R);
//     myFrameA.addForce (myFTmp);
//     myFrameAForce.set (myFTmp);
//     myF.inverseTransform (myDC.R);
//     myFTmp.transform (myX2B, myF);
//     myFTmp.transform (myFrameB.getPose().R);
//     myFTmp.negate();
//     myFrameB.addForce (myFTmp);
//   }

   public void scaleMass (double s)
    { 
      super.scaleMass (s);
      myStiffnessVec.scale(s);
      myDamping *= s;
      myRotaryStiffness *= s;
      myRotaryDamping *= s;
    }
   
   /*======== Renderable implementation =======*/

   protected RenderProps myRenderProps = null;

   public RenderProps getRenderProps()
    {
      return myRenderProps;
    }

   public void setRenderProps (RenderProps props)
    {
      myRenderProps =
         RenderableComponentBase.updateRenderProps(this, myRenderProps, props);
    }

   public RenderProps createRenderProps()
    {
      return RenderProps.createRenderProps (this);
    }

   public void prerender (RenderList list)
    {
      myRenderXCW.mul (myFrameA.getPose (), myX1A);
      myRenderXDW.mul (myFrameB.getPose (), myX2B);
    }

   public void updateBounds (Vector3d pmin, Vector3d pmax)
    {
       myFrameA.updateBounds (pmin, pmax);
       myFrameB.updateBounds (pmin, pmax);
    }

   public int getRenderHints()
    {
      int code = 0;
      if (myRenderProps != null && myRenderProps.getAlpha() != 1)
       { code |= TRANSPARENT;
       }
      return code;
    }
   
   public boolean isSelectable()
    { return true;      
    }

   public int numSelectionQueriesNeeded() {
      return -1;
   }

   public void render (Renderer renderer, int flags)
    { 
       int width = myRenderProps.getLineWidth();
       renderer.drawAxes (myRenderXCW, 1f, width, isSelected());
       renderer.drawAxes (myRenderXDW, 1f, width, isSelected());
    }

   public void getSelection (LinkedList<Object> list, int qid) {
   }
 }
