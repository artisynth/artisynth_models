package artisynth.models.tubesounds;

import java.io.File;
import java.io.IOException;
import maspack.matrix.Point3d;
import jass.generators.TubeShape;
import artisynth.core.modelbase.*;

public class AirwayMesh extends SectionedMesh implements TubeShape {
    
    protected int larynxPlane; // only sections from here on are part of sound production
    protected static final String LARYNXNAME = "larynx";
    protected TractCache tractCache;
    
	public AirwayMesh () {
        super();
    }

	public AirwayMesh (File file) throws IOException {
        super (file);
        PolygonalSection[] sections = getSections();
        for(int i=0;i<sections.length;i++) {
            String name = sections[i].getName();
            if(name !=null && name.compareToIgnoreCase(LARYNXNAME) == 0) {
                larynxPlane = i;
                System.out.println("larynx located at i="+i);
            }
        }
        tractCache = new TractCache();
    }
    
    class TractCache {
        Point3d[] centroids0; // initially (size N array)
        double tractLength0; // initially (from larynx to end)
        double[] area0; // initially (size N array)
        double volume0;
        double[] sectionDist0; // initial distances between centroids (size N-1 array)
        Point3d[] centroids; // cache (size N array)
        double tractLength; // cache (from larynx to end)
        double[] area; // cache (size N array)
        double[] sectionDist; // (size N-1 array)
        double volume;
        double lengthConversion=1; // TubeShape.getLength()*lengthConversion = mesh length
        double widthConversion=1; // TubeShape.getRadius()*widthConversion = mesh radius

        private Point3d c0 = new Point3d();
        private Point3d c1 = new Point3d();
        
        TractCache() {
            init(); 
        }
        
        public void update() {
            int n = getNumSections();
            PolygonalSection[]  ps = getSections();        
            for(int i=0;i<n;i++) {
                ps[i].getCentroid(centroids[i]);
                area[i] = ps[i].estimateArea(centroids[i]);
            }
            ps[0].getCentroid(c0);
            for(int k=1;k<n;k++) {
                ps[k].getCentroid(c1);
                double d = c0.distance(c1);
                sectionDist[k-1] = d;
                c0.set(c1);
            }
            sectionDist[n-1]=0;
            double xx = 0;
            volume = 0;
            for(int k=larynxPlane;k<n-2;k++) {
                volume += sectionDist[k]*(area[k+1]+area[k])/2;
                xx += sectionDist[k];
            }
            xx += sectionDist[n-2];
            tractLength = xx;
        }
        
        public void init() {
            int n = getNumSections();
            centroids0 = new Point3d[n];
            centroids = new Point3d[n];
            area0 = new double[n];
            area = new double[n];
            sectionDist0 = new double[n];
            sectionDist = new double[n];
            PolygonalSection[]  ps = getSections();        
            for(int i=0;i<n;i++) {
                centroids0[i] = new Point3d();
                centroids[i] = new Point3d();
                ps[i].getCentroid(centroids0[i]);
                ps[i].getCentroid(centroids[i]);
                area[i] = ps[i].estimateArea(centroids[i]);
                area0[i] = area[i];
            }
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

    // TubeShape interface implementation
    
    public double getLength() {
        return tractCache.tractLength/tractCache.lengthConversion;
    }

    public double getRadius(double x) {
        double t=0;
        double ret = 0;
        int n = getNumSections();
        x *= tractCache.lengthConversion;
        if(x<0) {
            return Math.sqrt((tractCache.area[larynxPlane])/Math.PI)/tractCache.widthConversion;
        }
        if(x>tractCache.tractLength) {
            return Math.sqrt((tractCache.area0[n-1])/Math.PI)/tractCache.widthConversion;
        }
        for(int k=larynxPlane;k<n-1;k++) {
            t += tractCache.sectionDist[k];
            if(t>x) {
                double t0 = t - tractCache.sectionDist[k];
                double a = (x-t0)/(t-t0);
                double b = (t-x)/(t-t0);
                ret = Math.sqrt((a*tractCache.area[k-1] + b*tractCache.area[k])/Math.PI);
                ret /= tractCache.widthConversion;
                break;
            }
        }
        return ret;
    }

    private void test() {
        tractCache.prt();
        double len = getLength();
        System.out.println("len="+len);
        for(double x=-5;x<160;x+=2) {
            double r = getRadius(x);
            System.out.println("r("+x+")="+r);
        }
    }
    
    public static void main(String[] argv) {
        AirwayMesh a=null;
        try {
            a = new AirwayMesh(new File("airway.obj"));
        } catch(Exception e){}
        a.test();
    }

	public void notifyParentOfChange (ComponentChangeEvent e)
	 {
	   // no parent, so do nothing
	 }

	public void componentChanged (ComponentChangeEvent e)
	 {
	   notifyParentOfChange (e);
	 }
}
