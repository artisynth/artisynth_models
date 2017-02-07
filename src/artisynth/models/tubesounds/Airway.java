package artisynth.models.tubesounds;

import java.io.*;
import maspack.matrix.*;
import maspack.render.*;
import maspack.geometry.*;
import java.util.ArrayList;

import artisynth.core.modelbase.*;
import artisynth.core.driver.Main;
import java.awt.Color;

public class Airway extends RenderableModelBase
{
    private int larynxPlane; // only sections from here on are part of sound production
    private static final String LARYNXNAME = "larynx";
	SectionedMesh mesh;
    TractCache cache;
    String textureFileName = "";


    public StepAdjustment advance (double t0, double t1, int flags) { 
       return null;
    }
    
    public String getName() {
        return "AirwayMesh";
    }
    

    //IsRenderable
    public void render(
       Renderer renderer, int flags) {
        mesh.render(renderer, flags);
    }
    
    public void prerender (RenderList list)
    {
        mesh.prerender(list);
    }   
    
    public int getRenderHints() {
        return mesh.getRenderHints();
    }

    public void updateBounds (Vector3d pmin, Vector3d pmax) {
        mesh.updateBounds(pmin,pmax);
    }

    class TractCache {
        Point3d[] centroids0; // initially
        double tractLength0; // initially (from larynx to end)
        double[] area0; // initially
        double volume0;
        double[] sectionDist0; // initial distances between centroids
        Point3d[] centroids; // cache
        double tractLength; // cache (from larynx to end)
        double[] area; // cache
        double[] sectionDist;
        double volume;
        double lengthConversion=1; // TubeShape.getLength()*lengthConversion = mesh length
        double widthConversion=1; // TubeShape.getRadius()*widthConversion = mesh radius
        
        TractCache() {
            int n = mesh.getNumSections();
            centroids0 = new Point3d[n];
            centroids = new Point3d[n];
            area0 = new double[n];
            area = new double[n];
            sectionDist0 = new double[n];
            sectionDist = new double[n];
            PolygonalSection[]  ps = mesh.getSections();        
            for(int i=0;i<n;i++) {
                centroids0[i] = new Point3d();
                centroids[i] = new Point3d();
                ps[i].getCentroid(centroids0[i]);
                ps[i].getCentroid(centroids[i]);
                area[i] = ps[i].estimateArea(centroids[i]);
                area0[i] = area[i];
            }
            Point3d c0 = new Point3d();
            Point3d c1 = new Point3d();
            ps[0].getCentroid(c0);
            for(int k=1;k<n;k++) {
                ps[k].getCentroid(c1);
                double d = c0.distance(c1);
                sectionDist[k-1] = d;
                sectionDist0[k-1] = d;
                c0.set(c1);
            }
            sectionDist[n-1]=0;
            sectionDist0[n-1]=0;
            double xx = 0;
            volume0 = 0;
            volume = 0;
            for(int k=larynxPlane;k<n-2;k++) {
                volume += sectionDist[k]*(area[k+1]+area[k])/2;
                xx += sectionDist[k];
            }
            xx += sectionDist[n-2];
            volume0 = volume;
            tractLength0 = xx;
            tractLength = xx;
        }

        void prt() {
            int n = centroids0.length;
            System.out.println("len="+tractLength0+" vol="+volume0);
            for(int i=0;i<n-1;i++) {
                System.out.println(i+"  "+sectionDist0[i]+"  "+ Math.sqrt((area0[i]/Math.PI)));
            }
            System.out.println(n-1+"  "+" -- " +"  "+Math.sqrt((area0[n-1]/Math.PI)));
        }

    }
    
    private void init() {
        cache = new TractCache();
//        cache.prt();
    }

    public void scale() {
        Point3d c = new Point3d();
        Point3d pc = new Point3d();
        PolygonalSection[]  ps = mesh.getSections();
        //for(int k=ps.length-1;k<ps.length;k++) {
        for(int k=larynxPlane;k<ps.length;k++) {
            ArrayList<Vertex3d> vl = ps[k].getVerticesVector();
            ps[k].getCentroid(c);
            double r = 2*Math.random()-1;
            for(Vertex3d v: vl) {
                Point3d p = v.pnt;
                pc.combine(1.,p,-1.,c); 
                pc.scale(1 + .05*r);
                p.add(pc,c);
            }
        }
        Main.getMain().rerender();
    }

    public void init(jass.generators.TubeShape tubeShape) {
        cache.lengthConversion = cache.tractLength0/tubeShape.getLength();
        double sumMeshAreas=0,sumTubeShapeAreas=0;
        double x=0;
        int n = mesh.getNumSections();
        for(int k=larynxPlane;k<n-2;k++) {
            sumMeshAreas += cache.area0[k];
            double r = tubeShape.getRadius(x);
            sumTubeShapeAreas += r*r/Math.PI;
            x += cache.sectionDist0[k];
        }
        cache.widthConversion = Math.sqrt(sumMeshAreas/sumTubeShapeAreas);
        System.out.println("lenc="+cache.lengthConversion+" wc="+cache.widthConversion);
    }

    private void moveVerticesInPlaneNormal(int k,double d) {
        PolygonalSection ps = mesh.getSections()[k];
        Vector3d n = ps.getPlane().getNormal();
        ArrayList<Vertex3d> vl = ps.getVerticesVector();
        Point3d np = new Point3d(n);
        //double[] vals = new double[3];
        for(Vertex3d v: vl) {
            Point3d p = v.pnt;
            p.combine(1,p,d,np);
        }
    }
    
    private void scaleLength(double deltaL) {

        //ns = n-larynxPlane # active sections. Let's index them by
        // i = 0,....,ns-1. Each section has to move by
        // D = i * (deltaL/(ns-1)) in the direction normal to its section
        double prev_d = 0; // how much previous section moved
        int n = mesh.getNumSections();
        for(int k=larynxPlane+1;k<n;k++) {
            int i = k-larynxPlane;
            double d = i * deltaL/(n-larynxPlane-1); // to move
            moveVerticesInPlaneNormal(k,d);
            cache.sectionDist[k-1] += d - prev_d;
            prev_d = d;
        }
        cache.tractLength += deltaL;
    }
    
    public void scale(jass.generators.TubeShape tubeShape) {
        double deltaL = tubeShape.getLength()*cache.lengthConversion - cache.tractLength;
        if(Math.abs(deltaL)>.001) {
            scaleLength(deltaL);
        }
        Point3d c = new Point3d();
        Point3d pc = new Point3d();
        PolygonalSection[]  ps = mesh.getSections();
        double x = 0;
        for(int k=larynxPlane;k<ps.length;k++) {
            double oldArea = cache.area[k];
            double r = tubeShape.getRadius(x/cache.lengthConversion)*cache.widthConversion;
            double newArea = r*r/Math.PI;
            //System.out.println("old="+oldArea+" new="+newArea);
            ArrayList<Vertex3d> vl = ps[k].getVerticesVector();
            ps[k].getCentroid(c);
            for(Vertex3d v: vl) {
                Point3d p = v.pnt;
                pc.combine(1.,p,-1.,c); 
                double scaleFactor = Math.sqrt(newArea/oldArea);
                pc.scale(scaleFactor);
                p.add(pc,c);
            }
            x += cache.sectionDist[k];
            cache.area[k] = newArea;
        }
    }


   public void display()
    { 
      RenderProps props = mesh.createRenderProps();
      props.setDrawEdges (false);
      props.setLineColor (new Color (0.5f,0.5f,0.5f));
      props.setFaceStyle (Renderer.FaceStyle.FRONT_AND_BACK);
        //mesh.setRenderFaces(PolygonalMesh.FRONT_FACE);
      props.setShading (Renderer.Shading.SMOOTH);
      props.setBackColor (Color.GREEN);
        
      ColorMapProps cprops = new ColorMapProps();
      cprops.setEnabled (true);
      cprops.setFileName (textureFileName);
      //cprops.setSphereMappingEnabled(true);
      cprops.setColorMixing(Renderer.ColorMixing.MODULATE);
      props.setColorMap (cprops);
        
        /*Material mat = new Material();
        mat.setAmbient (0, 0, 0, 1f);
        mat.setShininess (128.0f);
        mat.setSpecular (1f, 1f, 1f, 1f);
        mat.setDiffuse (125/255f, 125/255f, 125/255f, 1f);*/
      props.setFaceColor (Color.WHITE);
        //double s = .05;
        //double[] p_t = {s*.9,-.3*s,-.2*s,s/10};
        //double[] p_s = {-.2*s,.2*s,s,-.3*s};

//        double s = .05;
//        double[] p_t = {s*.9,-.3*s,-.2*s,s/10};
//        double[] p_s = {-.2*s,.2*s,s,-.3*s};
        //mesh.setAutomaticTextureMappingParmameters(p_s,p_t);
      mesh.setRenderProps (props);
      mesh.setFixed (false);
    }
    
    private void demoCode() {
        while(true) {
            try {
                Thread.sleep(50);
            } catch(Exception e) {}
            scale();

        }
    }

    
	public Airway(String meshFileName,String textureFileName)	 throws IOException {

        this.textureFileName = textureFileName;
	   mesh = new SectionedMesh (new File(meshFileName));
	   PolygonalSection[] sections = mesh.getSections();
       for(int i=0;i<sections.length;i++) {
           String name = sections[i].getName();
           if(name !=null && name.compareToIgnoreCase(LARYNXNAME) == 0) {
               larynxPlane = i;
               System.out.println("larynx located at i="+i);
           }
       }
       init();
       display();
       //demoCode();
	 }


    public static void main (String args[]) throws Exception {

    }

}
