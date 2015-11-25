package artisynth.models.tubesounds;

import jass.render.*;
import jass.generators.*;
import jass.utils.*;
import java.awt.*;
import javax.swing.*;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.tubesounds.Airway;
import maspack.geometry.*;
import maspack.matrix.*;

public class AreaFunctionPlotter{
    protected PlotGraph plotGraph = null;
    protected int n;
    protected double[][] plotData = null;
    protected int topleft_x=600;
    protected int topleft_y=0;

    public AreaFunctionPlotter(int n) {
        this.n = n;
        plotData = new double[2][n];
    }

    // public AreaFunctionPlotter(CoupledAirwayMesh a,boolean foo) {
    //     this.n = a.getNumSections();
    //     plotData = new double[2][n];
    // }
    
    // public void dumpPolygonalSections(CoupledAirwayMesh a) {
    //     int n = a.getNumSections();
    //     PolygonalSection[]  ps = a.getSections();
    //     for(int i=0;i<n;i++) {
    //         System.out.println("ps{"+(i+1)+"}=[");
    //         int nv = ps[i].numVertices();
    //         for(int k=0;k<nv;k++) {
    //     	Vertex3d v = ps[i].getVertex(k);
    //     	Point3d p = v.pnt;
    //     	System.out.println(p.x+" "+p.y+" "+p.z);
    //         }
    //         Vertex3d v = ps[i].getVertex(0);
    //         Point3d p = v.pnt;
    //         System.out.println(p.x+" "+p.y+" "+p.z);
    //         System.out.println("];");
    //     }
    // }

    // public void plot(CoupledAirwayMesh a,boolean foo) {
    //     if(plotData==null) {
    //         plotData = new double[2][this.n];
    //     }
    //     //dumpPolygonalSections(a);
    //     double x = 0;
    //     double[] dist = a.getSectionDistances();
    //     double[] areas = a.getTractAreas();
    //     for(int i=0;i<n;i++) {
    //         plotData[0][i] = i;//x;
    //         if(i<n-1) {
    //     	x += dist[i];
    //         }
    //         double area = areas[i];
    //         plotData[1][i] = area;
    //         //System.out.println(i+":"+x+" "+area);
    //     }
    //     if(plotGraph == null) {
    //         System.out.println("CREATE new sectioned mesh AREA FUNCTION graph");
    //         plotGraph = new PlotGraph(plotData);
    //         plotGraph.setCloseChoice(0); //0 hide, 1 exit
    //         plotGraph.rescaleX(.5);
    //         plotGraph.rescaleY(.5);
    //     } else {
    //         plotGraph.initialise(plotData);
    //     }
    //     plotGraph.setLocation(topleft_x,topleft_y);
    //     plotGraph.setLine(0);
    //     plotGraph.setPoint(1);
    //     plotGraph.plot();
    // }

    public void plot(TubeShape ts) {
        if(plotData==null) {
            plotData = new double[2][this.n];
        }
        double len = ts.getLength();
        for(int i=0;i<n;i++) {
            double x =  (i*len)/(n-1);
            plotData[0][i] = x;
            double r = ts.getRadius(x);
            plotData[1][i] = r*r; //y
        }
        if(plotGraph == null) {
            System.out.println("CREATE new tubeModel AREA FUNCTION graph");
            plotGraph = new PlotGraph(plotData);
            plotGraph.setCloseChoice(0); //0 hide, 1 exit
            plotGraph.rescaleX(.5);
            plotGraph.rescaleY(.5);
        } else {
            plotGraph.initialise(plotData);
        }
        plotGraph.setLocation(topleft_x,topleft_y);
        plotGraph.setLine(1);
        plotGraph.setPoint(0);
        plotGraph.plot();
    }

    public void setLocation(int topleft_x,int topleft_y) {
        this.topleft_x = topleft_x;
        this.topleft_y = topleft_y;
    }

    public void close() {
        plotGraph.close();
        plotGraph = null;
        plotData = null;
    }
}

