package artisynth.models.tubesounds;

import maspack.geometry.*;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

import java.util.ArrayList;
import java.util.Iterator;

public class PolygonalSection
{
	private ArrayList<Vertex3d>  vertices;
	private ArrayList<Point3d>  origPoints;
	private Plane plane;
	private boolean closed;
	private String name;

	// DEBUGm
	Point3d[] blendPnts;
	int numBlendPnts;

    public ArrayList<Vertex3d> getVerticesVector() {
        return vertices;
    }

    public ArrayList<Point3d> getOrigPoints() {
        return origPoints;
    }

	/**
	 * Creates an empty polygonal section.
	 */
	public PolygonalSection()
        { vertices = new ArrayList<Vertex3d>();
        origPoints = new ArrayList<Point3d>();
        plane = new Plane();

        blendPnts = new Point3d[100];
        for (int i=0; i<blendPnts.length; i++)
            { blendPnts[i] = new Point3d(); 
            }
        }

	void setBlendPoints (Point3d[] pnts, int num)
        {
            for (int i=0; i<num; i++)
                { blendPnts[i].set (pnts[i]); 
                }
            numBlendPnts = num;
        }

	/**
	 * Creates a polygonal section with no vertices and
	 * a specific plane.
	 *
	 * @param plane plane defining the section
	 */
	public PolygonalSection (Plane plane)
        {
            this();
            this.plane.set (plane);
        }

	public String getName ()
        {
            return name;
        }

	public void setName (String str)
        {
            if (str != null)
                { name = new String(str);
                }
            else
                { name = null; 
                }
        }

	private Vertex3d closestAdjacentVertex (
        Vertex3d vtxBase, Vertex3d vtxPrev, Plane plane,
        boolean[] marked)
        {
            // search all vertices attached to incident edges
            // and return the one closest to the plane
            double minDist = Double.POSITIVE_INFINITY;
            Vertex3d nearestVtx = null;
            for (Iterator it=vtxBase.getIncidentHalfEdges(); it.hasNext(); ) 
                { HalfEdge he = (HalfEdge)it.next();
                Vertex3d vtx = he.getTail();
                if (!marked[vtx.getIndex()] && vtx != vtxPrev)
                    { double d = Math.abs(plane.distance(vtx.pnt));
                    if (d < minDist)
                        { minDist = d;
                        nearestVtx = he.getTail();
                        }
                    }
                }
            return nearestVtx;
        }

	private boolean counterClockwiseCheck ()
        {
            Vertex3d vtx0 = getVertex(0);
            Vertex3d vtx1 = getVertex(1);
            Vector3d tmp = new Vector3d();
            Vector3d u = new Vector3d();
            HalfEdge he = null;
            for (Iterator it=vtx0.getIncidentHalfEdges(); it.hasNext(); ) 
                { he = (HalfEdge)it.next();
                if (he.getTail() == vtx1)
                    { break;
                    }
                }
            he.computeEdgeUnitVec (u);
            tmp.cross (he.getFace().getNormal(), u);
            double sign = he.isPrimary() ? 1 : -1;
            return (tmp.dot (plane.getNormal())*sign < 0);
        }

	private void reverseVertices()
        {
            int size = vertices.size();
            for (int i=0; i<size/2; i++)
                { Vertex3d tmp = vertices.get(i);
                vertices.set (i, vertices.get(size-1-i));
                vertices.set (size-1-i, tmp);
                }
        }
	
	/**
	 * Sets the vertices of this polygonal section by finding a loop of
	 * vertices within a mesh that stays close to a plane. This is done by
	 * first finding a vertex that is closest to the plane, and then
	 * searching along adjacent vertices until we either hit a dead-end or
	 * return to the starting vertex. Note that this routine does not
	 * guarantee that any loop found is "closest" in any particular
	 * mathematical sense.
	 *
	 * <p> The vertices should be arranged counter-clockwise around
	 * the plane's normal.
	 *
	 * <p>This function is still experimental and may not work that
	 * effectively.
	 *
	 * @param mesh triangular mesh providing the vertices
	 * @param plane plane defining the section
	 * @return true if a loop was found
	 */
	public boolean setVertices (PolygonalMesh mesh, Plane plane)
        {
            ArrayList verts = mesh.getVertices();
            boolean[] marked = new boolean[mesh.numVertices()];

            vertices.clear();
            // start by finding the closest vertex to the plane
            double minDist = Double.POSITIVE_INFINITY;
            Vertex3d startingVtx = null;
            for (Iterator it=verts.iterator(); it.hasNext(); )
                { Vertex3d vtx = (Vertex3d)it.next();
                double d = Math.abs(plane.distance(vtx.pnt));
                if (d < minDist)
                    { minDist = d;
                    startingVtx = vtx;
                    }
                }
//	   marked[startingVtx.idx] = true;

            Vertex3d prevVtx = null;
            Vertex3d vtx = startingVtx;
            int k = 0;
            do
                { vertices.add (vtx);
                Vertex3d nextVtx =
                    closestAdjacentVertex (vtx, prevVtx, plane, marked);
                prevVtx = vtx;
                vtx = nextVtx;
                if (vtx != null)
                    { marked[vtx.getIndex()] = true;
                    }
                }
            while (vtx != null && vtx != startingVtx);
            closed = (vtx == startingVtx);

            if (closed)
                { // make sure vertices run counter-clockwise around
                    // the normal
                    if (!counterClockwiseCheck())
                        { reverseVertices();
                        }
                }
	   
            // make a copy of the original vertex points
            for (Iterator it=vertices.iterator(); it.hasNext(); ) 
                { origPoints.add (new Point3d(((Vertex3d)it.next()).pnt));
                }

            return closed;
        }
	
	/**
	 * Returns an interator over the vertices defined for
	 * this section.
	 *
	 * @return vertex iterator
	 */
	public Iterator<Vertex3d> getVertices()
        {
            return vertices.iterator();
        }

	/**
	 * Returns the number of vertices defined for this section.
	 *
	 * @return number of vertices
	 */
	public int numVertices()
        {
            return vertices.size();
        }

	/**
	 * Gets a specific vertex for this section.
	 *
	 * @param num vertex number
	 * @return vertex
	 */
	public Vertex3d getVertex (int num)
        {
            return (Vertex3d)vertices.get(num);
        }

	/**
	 * Returns the plane defining this section.
	 *
	 * @return plane defining the section
	 */
	public Plane getPlane()
        {
            return plane;
        }

	/**
	 * Returns true if the vertices associated with
	 * this section form a closed loop.
	 *
	 * @return true if vertex loop is closed
	 */
	public boolean isClosed()
        {
            return closed;
        }

	/**
	 * Returns the length of this polygonal section.
	 *
	 * @return length of the section
	 */
	public double getLength ()
        { 
            double len = 0;
            Vertex3d prevVtx = null; 
            for (Iterator it=vertices.iterator(); it.hasNext(); ) 
                { Vertex3d vtx = (Vertex3d)it.next();
                if (prevVtx != null)
                    { len += vtx.pnt.distance (prevVtx.pnt);
                    }
                prevVtx = vtx;
                }
            if (prevVtx != null)
                { len += ((Vertex3d)vertices.get(0)).pnt.distance(prevVtx.pnt);
                }
            return len;
        }

	/**
	 * Gets the centroid of this polygonal section.
	 *
	 * @param centroid returns the centroid
	 */
	public void getCentroid (Point3d centroid)
        {
            int numVtxs = 0;
            centroid.setZero();
            for (Iterator it=vertices.iterator(); it.hasNext(); ) 
                { centroid.add (((Vertex3d)it.next()).pnt);
                numVtxs++;
                }
            centroid.scale (1/(double)numVtxs);
        }

	/**
	 * Estimates the area of this polygonal section by adding the areas of
	 * triangles formed by adjacent vertices and a reference point.
	 *
	 * @param ref reference point (typically the centroid)
	 */
	public double estimateArea (Point3d ref)
        {
            Vector3d xprod = new Vector3d();
            Vector3d d1 = new Vector3d();
            Vector3d d2 = new Vector3d();

            if (vertices.size() < 3)
                { return 0;
                }
            Iterator it=vertices.iterator();
            d2.sub (((Vertex3d)it.next()).pnt, ref);
            while (it.hasNext())
                { d1.set (d2);
                d2.sub (((Vertex3d)it.next()).pnt, ref);
                xprod.crossAdd (d1, d2, xprod);
                }
            d1.set (d2);
            d2.sub (getVertex(0).pnt, ref);
            xprod.crossAdd (d1, d2, xprod);
            return xprod.norm();
        }

	/**
	 * Returns the vertex with the largest signed distance from
	 * a given plane.
	 */
	public Vertex3d getMaxFromPlane (Plane plane)
        {
            double dmax = Double.NEGATIVE_INFINITY;
            Vertex3d vtxMax = null;
            for (Iterator it=vertices.iterator(); it.hasNext(); ) 
                { Vertex3d vtx = (Vertex3d)it.next();
                double d = plane.distance (vtx.pnt);
                if (d > dmax)
                    { dmax = d;
                    vtxMax = vtx;
                    }
                }
            return vtxMax;
        }

	/**
	 * Returns the vertex with the smaller signed distance from
	 * a given plane.
	 */
	public Vertex3d getMinFromPlane (Plane plane)
        {
            double dmin = Double.POSITIVE_INFINITY;
            Vertex3d vtxMin = null;
            for (Iterator it=vertices.iterator(); it.hasNext(); ) 
                { Vertex3d vtx = (Vertex3d)it.next();
                double d = plane.distance (vtx.pnt);
                if (d < dmin)
                    { dmin = d;
                    vtxMin = vtx;
                    }
                }
            return vtxMin;
        }

	/**
	 * Returns an array of points distributed along this
	 * polygon with a spacing of len.
	 */
	public Point3d[] getEvenlySpacedPoints (double spacing)
        {
            if (vertices.size() == 0)
                { return null;
                }

            double length = getLength();
            if (length == 0)
                { return new Point3d[] { new Point3d(getVertex(0).pnt) };
                }

            // adjust spacing so that the curve length is an integral multiple
            // of it.

            ArrayList<Point3d> newPnts = new ArrayList<Point3d>(16);
            int numNewPnts = (int)Math.max(length/spacing, 3);
            spacing = length/numNewPnts;
            double pntToVtxDist = 0;
            double vtxToVtxDist = 0;

            Iterator<Vertex3d> vit = vertices.iterator();
            Vertex3d vtx = vit.next();
            Vertex3d prevVtx = null;
            newPnts.add (vtx.pnt);
	   
            int k = 1;
            while (k < numNewPnts)
                { 
                    while (pntToVtxDist < spacing && vit.hasNext())
                        { prevVtx = vtx;
                        vtx = (Vertex3d)vit.next();
                        vtxToVtxDist = vtx.pnt.distance (prevVtx.pnt);
                        pntToVtxDist += vtxToVtxDist;
                        }
                    Point3d pnt = new Point3d();
                    pnt.interpolate (
                        prevVtx.pnt, 1-(pntToVtxDist-spacing)/vtxToVtxDist, vtx.pnt);
                    newPnts.add (pnt);
                    k++;
	   
                    pntToVtxDist -= spacing;
                }
            return (Point3d[])newPnts.toArray(new Point3d[0]);	   
        }

	private double blendPathLength (Point3d[] blendPnts, int num)
        {
            double len = 0;
            for (int i=0; i<num-1; i++)
                { len += blendPnts[i].distance(blendPnts[i+1]); 
                }
            return len;
        }

	public void blendPoints (
        Point3d[] blendPnts, int numb,
        Vertex3d leftVtx, Vertex3d rightVtx, double blendSize)
        {
            double blendLen = blendPathLength (blendPnts, numb);
            Point3d pnt = new Point3d();

            int leftIdx = vertices.indexOf (leftVtx);
            int rightIdx = vertices.indexOf (rightVtx);

            int numv = rightIdx - leftIdx + 1;
            if (numv <= 0)
                { numv += vertices.size();
                } 

            int k = 0;
            double kdist = 0;
            double kprevDist = 0;
            for (int i=1; i<numv-1; i++)
                { int idx = (leftIdx+i)%vertices.size();
                Vertex3d vtx = (Vertex3d)vertices.get (idx);
                double l = i/(double)(numv-1)*blendLen;
                while (kdist < l && k < numb-1)
                    { kprevDist = blendPnts[k].distance(blendPnts[k+1]);
                    kdist += kprevDist;
                    k++;
                    }
                if (k == numb-1)
                    { pnt.set (blendPnts[k]);
                    }
                else
                    { pnt.interpolate (
                        blendPnts[k-1], 1-(kdist-l)/kprevDist, blendPnts[k]);
                    }
                double s = Math.min(1,i/blendSize);
                s = Math.min(s, (numv-1-i)/blendSize);

                if (s == 1)
                    { vtx.pnt.set (pnt);
                    }
                else
                    { vtx.pnt.interpolate ((Point3d)origPoints.get(idx), s, pnt);
                    }
                }
        }
	
}
