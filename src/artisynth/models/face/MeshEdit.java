package artisynth.models.face;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;

public class MeshEdit extends RootModel {

   PolygonalMesh morphmesh, datamesh;
   MechModel mech;
   BVTree databvh;
   
   ArrayList<Point> leftpts = new ArrayList<Point>();
   ArrayList<Point> rightpts = new ArrayList<Point>();
   
   Plane midSagittalPlane = new Plane(Vector3d.Y_UNIT, 0);
   
   double midsagittalTol = 1e-6;
   double tol = 1e-5;
   Point3d reflect = new Point3d();

   
   public MeshEdit() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);      

      mech = new MechModel("mech");
      addModel(mech);
      
      RenderProps.setFaceStyle(mech, FaceStyle.FRONT_AND_BACK);
      RenderProps.setPointColor(mech, Color.MAGENTA);
      RenderProps.setPointRadius(mech, 0.00025);
      RenderProps.setPointStyle(mech, PointStyle.SPHERE);
      
      RigidBody data = addBody("ctdata", BadinFaceDemo.faceGeometryDir+"/badinskull.obj");
      RenderProps.setFaceColor(data, new Color(0.5f, 0.5f, 0.5f));
      mech.addRigidBody(data);
      datamesh = data.getMesh();
      databvh = datamesh.getBVTree();
      
      RigidBody morph = addBody("morph", BadinFaceDemo.faceGeometryDir+"/badin_maxilla_teeth_extended.obj");
//      RigidBody morph = addBody("morph", BadinFaceDemo.faceGeometryDir+"/badin_jaw_teeth_condyles_extended.obj");
      RenderProps.setFaceColor(morph, new Color(0.6f, 0.6f, 1f));
      mech.addRigidBody(morph);
      morphmesh = morph.getMesh();
      morphmesh.setFixed(false);
      
      for (Vertex3d v : morphmesh.getVertices()) {
	 Particle p = new Particle(1, v.getPosition());
	 mech.addParticle(p);
      }
      
      for (Particle p : mech.particles()) {
	 if (midSagittalPlane.distance(p.getPosition()) < -midsagittalTol) {
	    leftpts.add(p);
	    RenderProps.setPointColor(p, Color.MAGENTA.brighter());
	 }
      }
      
      ArrayList<Point> nomatch = new ArrayList<Point>();
      for (Point lp : leftpts) {
	 Point rp = findRightSidePoint(lp);
	 if (rp == null) {
	    System.out.println("no right side point found for p"+lp.getNumber());
	    RenderProps.setPointColor(lp, Color.RED);
	    RenderProps.setPointRadius(lp, 0.001);
	    nomatch.add(lp);
	 }
	 else { 
	    rightpts.add(rp);
	    RenderProps.setPointColor(rp, Color.MAGENTA.darker());
	 }
	    
      }
      
      leftpts.removeAll(nomatch);
   }
   
   public void selp() {
      Main.getMain().getSelectionManager().clearSelections();
      for (int i = 0; i < mech.particles().size(); i++) {
	 Main.getMain().getSelectionManager().addSelected(mech.particles().get(i));
      }
   }
   
   public void downmesh() {
      for (int i = 0; i < morphmesh.numVertices(); i++) {
	 mech.particles().get(i).setPosition(morphmesh.getVertices().get(i).pnt);
      }
      rerender();
   }
   
   public void upmesh() {
      for (int i = 0; i < morphmesh.numVertices(); i++) {
	 morphmesh.getVertices().get(i).pnt.set(mech.particles().get(i).getPosition());
      }
      rerender();
   }
   
   public void project() {
      for (ModelComponent comp : Main.getMain().getSelectionManager().getCurrentSelection()) {
	 if (comp instanceof Particle) {
	    project((Particle)comp);
	 }
      }
      rerender();
   }
   
   public void mirror() {
      for (int i = 0; i < rightpts.size(); i++) {
	 midSagittalPlane.reflect(reflect, rightpts.get(i).getPosition());
	 leftpts.get(i).setPosition(reflect);
      }
      rerender();
   }
   
   
   public void project(Particle p) {
      Point3d proj = new Point3d ();
      Vector2d coords = new Vector2d ();
      BVFeatureQuery query = new BVFeatureQuery();
      query.nearestFaceToPoint (proj, coords, databvh, p.getPosition());
      p.setPosition(proj);
   }
   

   
   public RigidBody addBody(String name, String meshName) {
      RigidBody body = new RigidBody(name);
    
      try {
	 body.setMesh(new PolygonalMesh(new File(meshName)), meshName);
      } catch (IOException e) {
	 // TODO Auto-generated catch block
	 e.printStackTrace();
      }
      
      body.setDynamic(false);
      
      
      
      return body;
   }

   
   private Point findRightSidePoint(Point p) {
      Point rightPt = null;
      Point3d pos = p.getPosition();
      midSagittalPlane.reflect(reflect, pos);
      for (Particle part : mech.particles()) {
	 if (reflect.distance(part.getPosition())< tol) {
	    rightPt = part;
	    break;
	 }
      }
      return rightPt;
   }
   

   @Override
   public void attach(DriverInterface driver) {
      // TODO Auto-generated method stub
      super.attach(driver);
      
      ControlPanel panel = BadinFaceDemo.createVisibilityPanel(this, mech);
      panel.addWidget("particles", mech, "particles:renderProps.visible");
      panel.pack ();
      panel.setVisible (true);
   }

   
   
}
