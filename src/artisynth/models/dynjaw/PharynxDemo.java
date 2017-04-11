package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemFactory.FemElementType;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.materials.AxialMaterial;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.AxialSpringList;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;

public class PharynxDemo extends JawLarynxDemo {

   PointList<Particle> pharynxNodes = new PointList<Particle>(Particle.class, "pharynxNodes");
   AxialSpringList<AxialSpring> pharynxNet = new AxialSpringList<AxialSpring> (AxialSpring.class, "pharynxNet", "pn");
   HashMap<Vertex3d,Particle> vmap = new HashMap<Vertex3d,Particle> ();

   public PharynxDemo () {
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
      RigidBody pharynx_surf = myJawModel.createAndAddBody (
         "pharynx", "pharynx/meshes/pharynx.obj");
//      RenderProps.setVisible (pharynx_surf, false);
      
      RigidBody pharynx_net = myJawModel.createAndAddBody (
         "pharynx_net", "pharynx/meshes/pharynx_wrap.obj");
//      RenderProps.setVisible (pharynx_net, false);
      
//      RigidBody sup = myJawModel.createAndAddBody ("pharynx_sup", "pharynx/meshes/sup.obj");
//      addFemSheet(sup);

//      RigidBody mid = myJawModel.createAndAddBody ("pharynx_mid", "pharynx/meshes/mid.obj");
//      addFemSheet(mid);
//
//      RigidBody inf = myJawModel.createAndAddBody ("pharynx_inf", "pharynx/meshes/inf.obj");
//      addFemSheet(inf);

//      RigidBody all = myJawModel.createAndAddBody ("pharynx_combined", "pharynx/meshes/pharynx_wrap.obj");
//      addFemSheet(all);

      
      addMuscleLines();
      
      RenderProps.setLineRadius (myJawModel, 0.2);

      myJawModel.add(pharynxNodes);
      RenderProps.setPointRadius (pharynxNodes, 0.5);
      RenderProps.setPointColor (pharynxNodes, Color.BLUE);
      
      myJawModel.add(pharynxNet);
      RenderProps.setLineStyle (myJawModel, LineStyle.CYLINDER);
      RenderProps.setLineRadius (myJawModel, 0.25);
      RenderProps.setLineColor (pharynxNet, new Color(.75f, .4f, .4f));

      
//      createPharyngealNet(net.getMesh ());
//      attachSup (67.07);
//      attachInf (-23.98);
   }
   
   public void addMuscleLines() {
      String[] groups = new String[]{
         "spc",
         "mpc",
         "ipc",
         "lp"
      };

      for (String group : groups) {
         String dirName = ArtisynthPath.getSrcRelativePath (this, "geometry/pharynx/bundles/")+group;
         File dir = new File(dirName);
         for (String file : dir.list ()) {
            if (!file.endsWith (".pts"))
               continue;
            String name = file.substring (0, file.length ()-4);
            addMuscleLines (dirName, name, false, group.endsWith ("pc"));
            addMuscleLines (dirName, name, true, group.endsWith ("pc"));
         }
      }
      
      int cnt = 0;
      float num = (float)myJawModel.meshBodies ().size ();
      for (MeshComponent comp : myJawModel.meshBodies ()) {
         RenderProps.setLineColor (comp, Color.getHSBColor ((cnt++)/num, 1f, 1f));
      }
   }

   public void addMuscleLines(String dirName, String name, boolean isLeftSide, boolean originatesAtMidline) {
      PolylineMesh mesh = new PolylineMesh ();
      for (int j = 1; j < 10; j++) {
         String filename = dirName+"_split/"+name+"_"+j+".pts";
         if (!(new File(filename).exists ())) {
            break;
         }
         Point3d[] pnts = readPointList (filename);
         Vertex3d[] vtxs = new Vertex3d[pnts.length];
         for (int i = 0; i < pnts.length; i++) {
            Point3d pnt = pnts[i];
            if (originatesAtMidline && i == 0) {
               pnt.x = 0; // origin pnt on midline
            } else if (isLeftSide) {
               pnt.x = -pnt.x;
            }
            vtxs[i] = mesh.addVertex (pnt);
         }
         mesh.addLine (vtxs);
      }
      MeshComponent comp = new MeshComponent (mesh, null, null);
      String meshname;
      if (isLeftSide) {
         meshname = "l"+name.substring (1, name.length ());
      } else {
         meshname = name.substring (0, name.length ());
      }
      comp.setName (meshname);
      myJawModel.addMeshBody (comp);
   }
   
   public void addMuscleLine(String dirName, String file, boolean flip_x) {
      PolylineMesh mesh = new PolylineMesh ();
      for (Point3d pnt : readPointList (dirName+"_split"+file)) {
         if (flip_x) {
            pnt.x = -pnt.x;
         }
         mesh.addVertex (pnt);
         
      }
      mesh.addLine (mesh.getVertices ().toArray (new Vertex3d[0]));
      MeshComponent comp = new MeshComponent (mesh, null, null);
      String name;
      if (flip_x) {
         name = "l"+file.substring (1, file.length ()-4);
      } else {
         name = file.substring (0, file.length ()-4);
      }
      comp.setName (name);
      myJawModel.addMeshBody (comp);
   }
   
   public static Point3d[] readPointList(String fileName) {
      ArrayList<Point3d> intList = new ArrayList<Point3d>();
      try {
         ReaderTokenizer rtok = new ReaderTokenizer(new FileReader(fileName));
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            rtok.pushBack();
            Point3d pnt = new Point3d();
            pnt.scan (rtok);
            intList.add(pnt);
         }
      } catch (IOException e) {
         e.printStackTrace();
      }
      return intList.toArray(new Point3d[intList.size()]);
   }
   
   public void addFemSheet(RigidBody surf) {
      FemMuscleModel fem = new FemMuscleModel ();
      FemFactory.createExtrusion (
         (FemModel3d)fem, FemElementType.Tet, 1, 1d, 0, surf.getMesh());
      fem.setName (surf.getName ());
      setFemRenderProps(fem);
      myJawModel.addModel (fem);
      
      RenderProps.setVisible(fem.getNodes (), false);
      // for extruded FEM the first n nodes are co-incident 
      // with the n vertices of the surface mesh
      for (int i = 0; i < surf.getMesh ().numVertices (); i++) {
         RenderProps.setVisible (fem.getNode (i), true);
      }
   }
   
   public void setFemRenderProps(FemMuscleModel fem) {
      RenderProps.setPointRadius (fem, 0.4);
      RenderProps.setVisible (fem.getElements (), false);
      RenderProps.setFaceColor (fem, new Color(.75f, .4f, .4f));
      RenderProps.setLineRadius (fem, 0.2);
      fem.setSurfaceRendering (SurfaceRender.Shaded);
      
   }
   
   public void setMass(double m) {
      for (Particle p : pharynxNodes) {
         p.setMass(m);
      }
   }
   
   public void attachSup(double zthreshold) {
      for (Particle p : pharynxNodes) {
         if (p.getPosition ().z >= zthreshold) {
            p.setDynamic (false);
            RenderProps.setPointColor (p, Color.GRAY);
         }
      }
   }
   
   public void attachInf(double zthreshold) {
      for (Particle p : pharynxNodes) {
         if (p.getPosition ().z <= zthreshold) {
            p.setDynamic (false);
            RenderProps.setPointColor (p, Color.GRAY);
         }
      }
   }
   
   public void createPharyngealNet(PolygonalMesh mesh) {
      
      double mass = 0.01;
      AxialMaterial mat = new LinearAxialMaterial (1000, 10);
      pharynxNet.setMaterial (mat);
      
      for (Vertex3d v : mesh.getVertices ()) {
         Particle p = new Particle (mass);
         p.setPosition (v.pnt);
         vmap.put (v, p);
         pharynxNodes.add (p);
      }
      
      for (Face f : mesh.getFaces ()) {
         for (int i = 0; i < f.numEdges (); i++) {
            HalfEdge e = f.getEdge (i);
            addSpring(e.head, e.tail);
         }
      }
      
   }
   
   private void addSpring(Vertex3d v0, Vertex3d v1) {
      AxialSpring s = new AxialSpring ();
      s.setMaterial (null);
      s.setFirstPoint (vmap.get (v0));
      s.setSecondPoint (vmap.get (v1));
      pharynxNet.add(s);
   }
   

}
