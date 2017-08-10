package artisynth.models.registration;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemNode;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.PointFem3dAttachment;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.PointAttachment;
import artisynth.core.mechmodels.PointParticleAttachment;
import artisynth.core.modelbase.ControllerBase;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.properties.PropertyList;
import maspack.render.LineRenderProps;
import maspack.render.RenderList;
import maspack.render.RenderObject;
import maspack.render.Renderer;
import maspack.render.Renderer.LineStyle;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.PropertyWidget;

/**
 * Controller to apply forces from an embedded FemMesh to a target mesh
 * @author Antonio
 *
 */
public class FemMeshICPController extends ControllerBase {

   public static boolean DEFAULT_ENABLED = true;
   RegistrationPressureFunctionBase pressureFunction;
   
   private PolygonalMesh targetSurface = null;
   private FemMeshComp sourceSurface = null;
   HashMap <FemNode3d, Vector3d> forceMap;
   double renderScale = 0;
   
   private static class VertexInfo {
      Vertex3d vtx;
      PointAttachment pa;
      double[] wgts;
      double area;
      FemNode3d[] nodes;
      Vector3d pressure;
   }
   VertexInfo[] vtxInfo = null;
   
   private boolean enabled = DEFAULT_ENABLED;
   
   private boolean zero = false;
   
   public static PropertyList myProps = new PropertyList(FemMeshICPController.class);
   static {
      myProps.add("enabled * *", "Controller enabled", DEFAULT_ENABLED);
      myProps.add("pressureFunction * *", "registration pressure function", createDefaultPressureFunction());
      myProps.add("renderProps * *", "render properties", new LineRenderProps());
      myProps.add("forceRenderScale * *", "render scale of forces", 0.0);
      // myProps.add("smoothing * *", "Force smoothing", DEFAULT_SMOOTHING, "NW [0,1]");
   }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public FemMeshICPController(FemMeshComp source, PolygonalMesh target) {
      
      this.targetSurface = target;
      this.sourceSurface = source;
      this.forceMap = new HashMap<>();
      
      vtxInfo = new VertexInfo[sourceSurface.numVertices()];
      for (int i=0; i<sourceSurface.numVertices(); i++) {
         vtxInfo[i] = new VertexInfo();
         vtxInfo[i].vtx = sourceSurface.getVertex(i);
         vtxInfo[i].pa = sourceSurface.getAttachment(i);
         vtxInfo[i].pressure = new Vector3d();
         if (vtxInfo[i].pa instanceof PointFem3dAttachment) {
            PointFem3dAttachment pf3d = (PointFem3dAttachment)(vtxInfo[i].pa);
            FemNode[] nodes = pf3d.getNodes();
            VectorNd weights = pf3d.getCoordinates();
            vtxInfo[i].nodes = new FemNode3d[nodes.length];
            vtxInfo[i].wgts = new double[nodes.length];
            for (int j=0; j<nodes.length; j++) {
               vtxInfo[i].nodes[j] = (FemNode3d)(nodes[j]);
               vtxInfo[i].wgts[j] = weights.get(j);
            }
            
         } else if (vtxInfo[i].pa instanceof PointParticleAttachment) {
            PointParticleAttachment pp = (PointParticleAttachment)vtxInfo[i].pa;
            vtxInfo[i].nodes = new FemNode3d[1];
            vtxInfo[i].nodes[0] = (FemNode3d)(pp.getParticle());
            vtxInfo[i].wgts = new double[1];
            vtxInfo[i].wgts[0] = 1;
         }
         
         // divide attached face areas
         vtxInfo[i].area = 0;
         Iterator<HalfEdge> hedges = vtxInfo[i].vtx.getIncidentHalfEdges();
         while (hedges.hasNext()) {
            HalfEdge he = hedges.next();
            double fa = he.getFace().computeArea();
            fa = fa/he.getFace().numVertices();
            vtxInfo[i].area += fa;
         }
      }
      
      // create point forces
      for (VertexInfo vi : vtxInfo) {
         for (int j=0; j<vi.nodes.length; j++) {
            Vector3d pf = forceMap.get(vi.nodes[j]);
            if (pf == null) {
               pf = new Vector3d();
               forceMap.put(vi.nodes[j], pf);
            }
         }
      }
   }
   
   public void apply(double t0, double t1) {
      if (!enabled) {
         zeroForces();  
         return;
      }
      updateForces();
      
     
      // add forces to nodes
      for (Entry<FemNode3d,Vector3d> entry : forceMap.entrySet()) {
         FemNode3d node = entry.getKey();
         Vector3d force = entry.getValue();
         node.setExternalForce(force);
      }
   }
   
   private void zeroForces() {
      
      if (zero) {
         return;
      }
      
      for (Vector3d force : forceMap.values()) {
         force.setZero();
      }
      zero = true;
   }
   
   private void updateForces() {
      
      zero = false;
      
      BVFeatureQuery query = new BVFeatureQuery();
      Vector2d coords = new Vector2d();
      Point3d nearest = new Point3d();
      Vector3d nSource = new Vector3d();
      Vector3d nTarget = new Vector3d();
      
      // zero out node forces
      for (Vector3d pf : forceMap.values()) {
         pf.setZero();
      }
      
      RegistrationPressureFunction pfunc = getPressureFunction();
      
      for (VertexInfo vi : vtxInfo) {
         // project to surface
         Point3d pSource = vi.vtx.getWorldPoint();
         vi.vtx.computeWorldNormal(nSource);
         
         Face f = query.nearestFaceToPoint(nearest, coords, targetSurface, pSource);
         f.getWorldNormal(nTarget);
         
         
         pfunc.computeCorrespondencePressure(pSource, nSource, nearest, nTarget, vi.pressure);
         
         vi.pressure.scale(vi.area); // scale by area
         
         // distribute force around to attached nodes
         for (int i=0; i<vi.nodes.length; i++) {
            Vector3d pf = forceMap.get(vi.nodes[i]);
            pf.scaledAdd(vi.wgts[i]*vi.area, vi.pressure);
         }
      }

      
   }   
   
   public void setEnabled(boolean enabled) {
      this.enabled = enabled;
   }
   
   public boolean getEnabled() {
      return enabled;
   }

   public static void addControls(
      ControlPanel controlPanel, FemMeshICPController controller) {
      
      for (PropertyInfo propInfo : myProps) {
         Property prop = controller.getProperty(propInfo.getName());
         LabeledComponentBase widget = PropertyWidget.create (prop);
         controlPanel.addWidget(widget);
      }
   }


   private static RegistrationPressureFunctionBase createDefaultPressureFunction() {
      return new GravityPressureFunction();
   }
   
   public RegistrationPressureFunctionBase getPressureFunction() {
      if (pressureFunction == null) {
         pressureFunction = createDefaultPressureFunction();
      }
      return pressureFunction;
   }


   public void setPressureFunction(RegistrationPressureFunctionBase pfunc) {
      this.pressureFunction = pfunc;
      updateForces();
   }
   
   public double getForceRenderScale() {
      return renderScale;
   }
   
   public void setForceRenderScale(double s) {
      renderScale = s;
   }
   
   // render implementation
   RenderObject robj;
   
   @Override
   public void prerender(RenderList list) {
      super.prerender(list);
      
      Point3d tmp = new Point3d();
      if (robj == null) {
         robj = new RenderObject();
         for (Entry<FemNode3d,Vector3d> entry : forceMap.entrySet()) {
            FemNode3d node = entry.getKey();
            Vector3d force = entry.getValue();
            int p0 = robj.addPosition(node.getPosition());
            tmp.scaledAdd(renderScale, force, node.getPosition());
            int p1 = robj.addPosition(tmp);
            int v0 = robj.addVertex(p0);
            int v1 = robj.addVertex(p1);
            robj.addLine(v0, v1);
         }
      } else {
         // update points
         int pidx = 0;
         for (Entry<FemNode3d,Vector3d> entry : forceMap.entrySet()) {
            FemNode3d node = entry.getKey();
            Vector3d force = entry.getValue();
            robj.setPosition(pidx++, node.getPosition());
            tmp.scaledAdd(renderScale, force, node.getPosition());
            robj.setPosition(pidx++, tmp);
         }
      }
      
   }
   
   @Override
   public void render(Renderer renderer, int flags) {
      super.render(renderer, flags);
      
      if (renderScale > 0) {
         renderer.setLineShading(myRenderProps);
         renderer.setLineColoring(myRenderProps, isSelected());
         LineStyle s = myRenderProps.getLineStyle();
         switch(s) {
            case LINE:
               renderer.drawLines(robj, s, myRenderProps.getLineWidth());
               break;
            case SOLID_ARROW:
            case SPINDLE:
            case CYLINDER:
               renderer.drawLines(robj, s, myRenderProps.getLineRadius());
               break;
            
         }

      }
      
   }
   
   @Override
   public void initialize(double t0) {
      super.initialize(t0);
      zeroForces();
   }
      
}
