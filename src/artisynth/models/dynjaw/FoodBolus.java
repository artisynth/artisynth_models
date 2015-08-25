package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Deque;

import javax.media.opengl.GL2;

import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Matrix;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SparseBlockMatrix;
import maspack.matrix.SparseNumberedBlockMatrix;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.matrix.VectorNi;
import maspack.properties.PropertyList;
import maspack.render.GLRenderer;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.util.*;
import artisynth.core.mechmodels.ForceComponent;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.HasAuxState;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.CompositeComponentBase;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.util.*;

public class FoodBolus extends RenderableComponentBase 
   implements ForceComponent, ScalableUnits, TransformableGeometry, HasAuxState
{

   boolean useNormalToPlane = true; // if false, calculate normal b/w plane origin and colliding point
   
   protected double distanceThreshold = 0.001;
   protected boolean debug = false;

   PlanarConnector myPlane = null;
   protected Point3d myCenter = new Point3d();
   protected double mySize = 50;
   protected Point collidingPt = new Point();

   protected double myDiameter = 1.0;
   protected double myDamping;
   protected double myMaxResistance = 1.0;
   private double myStiffness;	
   protected double myForceNorm;
   protected boolean myActiveP = false;
   protected boolean myCrushedP = true;

   protected double myLastStepTime = -1;
	
   // 	converts Newtons, which are used externally, to internal model
   // force units
   private static final double defaultNewtonsToForce = 1000;
   private double NewtonsToForce = defaultNewtonsToForce;
	


   public static PropertyList myProps = new PropertyList(FoodBolus.class,
                                                         RenderableComponentBase.class);

   static
    {
      myProps.add("renderProps", "render properties", null);
      myProps.addReadOnly("forceNorm *",
                  "norm of total force applied by bolus (N)", "%8.2f");
      myProps.addReadOnly("normal *",
         "direction of force applied by bolus", "%8.4f");
      myProps.add("resistance * *",
                  "maximum force produced by bolus (N)", 1.0);	      
      myProps.add("diameter * *",
                  "diameter of bolus (mm)", 1.0);
      myProps.add("damping * *",
                  "linear damping for bolus force (kg/s)", 0.1);	      
      myProps.add("active isActive *", 
                  "active status of bolus", false);	     
      myProps.add("crushed isCrushed *", 
                  "crushed status of bolus", true);
    }

   public PropertyList getAllPropertyInfo()
    {
      return myProps;
    }

   public FoodBolus()
    {
      super();
      myRenderProps = createRenderProps ();
    }

   public FoodBolus (String name, PlanarConnector plane, double diam, double resist, double damp)
    { 
      this();
      setName (name);
      myPlane = plane;
      myDiameter = diam;	   
      setResistance(resist);
      myDamping = damp;
    }
	
   public void setCollidingPoint(Point pt)
    {
      if (pt != null)
       {
         collidingPt = pt;
       }
    }

   public Point getCollidingPoint() {
      return collidingPt;
   }

   public RenderProps createRenderProps()
    {
      RenderProps props = new RenderProps();
      props.setFaceColor (new Color(0.5f, 0.5f, 0.5f));
      props.setAlpha (1.0f);
      props.setShading(RenderProps.Shading.PHONG);
      props.setPointSlices(30);
      return props;
    }

   public PlanarConnector getPlane()
    {
      return myPlane;
    }
   
   public void setPlane(PlanarConnector p)
    {
      myPlane = p;
    }

   public void setRenderPosition (Point3d center, double size)
    {
      setCenter (center);
      setSize (size);
    }

   public void setCenter (Point3d center)
    {
      myCenter.set (center);
    }

   public Point3d getCenter()
    {
      return myCenter;
    }

   public void setSize (double size)
    {
      mySize = size;
    }

   public double getSize()
    {
      return mySize;
    }
   
   public void addPosJacobian (SparseNumberedBlockMatrix M, double s)
   {
      // No jacobian information for foodbolus
   }

   public void addSolveBlocks (SparseNumberedBlockMatrix M)
   {
      // No jacobian information for foodbolus
   }

   public void addVelJacobian (SparseNumberedBlockMatrix M, double s)
   {
      // No jacobian information for foodbolus
   }

   public int getJacobianType()
   {
      // No jacobian information for foodbolus
      return Matrix.SPD; 
   }

   Plane plane = new Plane ();
   Vector3d myNormal = new Vector3d ();

   private double getPlaneNormal(Vector3d nrm) {
      return myPlane.getPlaneNormal (nrm);
   }
   
   public Vector3d getNormal() {
      return myNormal;
   }
   
   public double getDistanceToPlane(Vector3d normal) {
      double offset = myPlane.getPlaneNormal (myNormal);
//      RigidTransform3d XPB = myPlane.getTDB();
//      myNormal.set (XPB.R.m02, XPB.R.m12, XPB.R.m22);
//      double offset = myNormal.dot(XPB.p);
      plane.set (myNormal, offset);
      myNormal.normalize ();
      return plane.distance (collidingPt.getPosition());
   }
   

   public double getDistanceToPlaneOrigin(Vector3d normal) {
      RigidTransform3d TDW = myPlane.getCurrentTDW();
      myNormal.sub(collidingPt.getPosition (), TDW.p);
      double dist = myNormal.norm ();
      myNormal.normalize ();
      return dist;
   }
   
   private boolean isTime(double t) {
      return (t >= 0.5547 && t <= 0.5549);      
   }

   public void applyForces (double t) {

      if (!isActive()) {
         return;
      }

      double dist;
      if (useNormalToPlane) {
         dist = getDistanceToPlane (myNormal);
      }
      else {
         dist = getDistanceToPlaneOrigin (myNormal);
      }
      
      if (isCrushed()) {
         if (dist > myDiameter) {
            setCrushed(false);
         }
         else {
            myForceNorm = 0.0;
            //collidingPt.zeroExternalForces();
            // external force set to zero
         }
         return;
      }
	   
      double v = collidingPt.getVelocity().dot(myNormal);

//  	   System.out.println("dist = " + dist +"; v = " + v + ";diam = " + myDiameter);
      if (dist < myDiameter) {
         if (dist < distanceThreshold) {
            setCrushed (true);
            myForceNorm = 0.0;
         }
         else {
            Vector3d extForce = new Vector3d();
            if (v < 0) { // jaw is closing apply bolus damping
               myForceNorm = myStiffness*(myDiameter - dist) - myDamping*v;
            }
            else  {
               myForceNorm = myStiffness*(myDiameter - dist);
            }
            // System.out.println("Bolus Newtons = " + bolusForceNorm);
            
            extForce.scaledAdd (newtonsToForce(myForceNorm),
                                myNormal, extForce);
            // System.out.println("F_bolus=" + extForce.toString("%8.2f"));
            //collidingPt.setExternalForce(extForce);
            collidingPt.addForce(extForce);
            myRenderProps.setLineColor(Color.RED);
            // if (myForceNorm > 0.9*myMaxResistance)
         }
      }
      else { 
         myForceNorm = 0.0; 
         //collidingPt.zeroExternalForces();
         myRenderProps.setLineColor(Color.GREEN);
      }
   }
        
   public void newtonsToForce(Vector3d f)
    {
      f.scale(NewtonsToForce);
    }

   public double newtonsToForce(double fNorm)
    {
      return fNorm * NewtonsToForce;
    }

   public void printPointReferences(PrintWriter pw, CompositeComponent ancestor)
      throws IOException
    {
      pw.print(ComponentUtils.getWritePathName(ancestor,
                                                         collidingPt)
               + " ");
    }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();      
      if (scanAndStoreReference (rtok, "point", tokens)) {
         return true;
      }
      else if (scanAttributeName (rtok, "planarConnector")) {
         myPlane = new PlanarConnector();
         tokens.offer (new StringToken ("planarConnector", rtok.lineno()));
         myPlane.scan (rtok, tokens);
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      pw.println (
         "point=" + ComponentUtils.getWritePathName(ancestor, collidingPt));
      pw.print ("planarConnector=");
      myPlane.write (pw, fmt, ancestor);
      super.writeItems (pw, fmt, ancestor);
   }

   // /**
   //  * Scans this component from a ReaderTokenizer. The expected text format is
   //  * assumed to be compatible with that produced by {@link #write write}.
   //  * 
   //  * @param rtok
   //  *            Tokenizer from which to scan the component
   //  * @throws IOException
   //  *             if an I/O or formatting error occured
   //  */
   // public void scan(ReaderTokenizer rtok, Object ref) throws IOException {
   //    super.scan (rtok, ref);
   //    if (useNewScan) {
   //       Deque<ScanToken> tokens = (Deque<ScanToken>)ref;
   //       tokens.offer (ScanToken.END);
   //    }
   // }

   protected boolean postscanItem (
   Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {

      if (postscanAttributeName (tokens, "point")) {
         collidingPt = postscanReference (tokens, Point.class, ancestor);
         return true;
      }
      else if (ScanWriteUtils.postscanAttributeName (tokens, "planarConnector")) {
         myPlane.postscan (tokens, ancestor);
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }
   

   public void updateBounds (Point3d pmin, Point3d pmax)
    {
      myCenter.updateBounds (pmin, pmax);
    }
	
   //TODO - project colliding point onto bite plane and draw sphere normal to bite plane
   public void drawSolid(GLRenderer renderer, RenderProps props)
    {
      float[] bolusCentre = collidingPt.myRenderCoords.clone();
      bolusCentre[2] += myDiameter/2.0;
      props.setPointRadius(myDiameter/2.0);
//      props.drawSphere(renderer, bolusCentre);
      renderer.drawSphere (props, bolusCentre);
    }
	
   //TODO - make Bolus appear to be squished - for now just render sphere
   public void render (GLRenderer renderer, int flags)
    { renderer.setMaterialAndShading (
          myRenderProps, myRenderProps.getLineMaterial(), isSelected());       
      if (myRenderProps.isVisible())
	 drawSolid (renderer, myRenderProps);
      renderer.restoreShading (myRenderProps);
    }

   public void scaleDistance (double s)
    {
      myCenter.scale (s);
      mySize *= s;
//      myPlane.offset *= s;
      myDiameter *= s;
      myMaxResistance *= s;
    }

   public void scaleMass (double s)
    { 
      myMaxResistance *= s;
      myStiffness *= s;
      myDamping *= s;
    }

   public double getForceNorm()
    {
      return myForceNorm;
    }

   public double getDiameter()
    {
      return myDiameter;
    }

   public void setDiameter(double diameter)
    {
      myStiffness = myMaxResistance / diameter;
      myDiameter = diameter;
    }

   public boolean isActive()
    {
      return myActiveP;
    }

   public void setActive(boolean active)
    {
      this.myActiveP = active;
      myRenderProps.setVisible(active);
      if (!active)
      {
         setCrushed (true);
         myForceNorm = 0.0;
         collidingPt.zeroExternalForces();
      }
    }

   public double getResistance()
    {
      return myMaxResistance;
    }

   public void setResistance(double resistance)
    {
      myStiffness = resistance / myDiameter;
      myMaxResistance = resistance;
    }

   public double getDamping()
    {
      return myDamping;
    }

   public void setDamping(double damping)
    {
      myDamping = damping;
    }

   public boolean isCrushed()
    {
      return myCrushedP;
    }

   public void setCrushed(boolean crushed)
    {
      this.myCrushedP = crushed;
      if (crushed)
         myRenderProps.setLineColor(Color.GRAY);
      else
         myRenderProps.setLineColor(Color.GREEN);
    }

   public void transformGeometry (AffineTransform3dBase X) {
      transformGeometry (X, this, 0);
      
   }

   public void transformGeometry (
      AffineTransform3dBase X, TransformableGeometry topObject, int flags) {
      
      // transform done in mech model
//      myPlane.transform (X);
      
   }

   public void advanceAuxState (double t0, double t1) {
   }

   /** 
    * {@inheritDoc}
    */
   public void skipAuxState (DataBuffer data) {
      data.dskip (1);
      data.zskip (1);
   }

   public void getInitialAuxState (DataBuffer newData, DataBuffer oldData) {
      if (oldData == null) {
         getAuxState (newData);
      }
      else {
         newData.putData (oldData, 1, 1);
      }
   }

   public void getAuxState (DataBuffer data) {
      data.dput (myLastStepTime);
      data.zput (isCrushed() ? 1 : 0);
   }

   public void setAuxState (DataBuffer data) {
      myLastStepTime = data.dget();
      setCrushed (data.zget() != 0);
   }
   
   public boolean hasState() {
      return true;
   }

}
