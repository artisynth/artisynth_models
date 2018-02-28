package artisynth.models.tubesounds;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StreamTokenizer;
import java.util.Iterator;
import java.util.Vector;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.WavefrontReader;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.VectorTransformer3d;
import maspack.matrix.Plane;
import maspack.util.*;
import maspack.matrix.Vector3d;

public class SectionedMesh extends PolygonalMesh
{
    public SectionedMesh () {
        super();
    }

    public SectionedMesh (File file) throws IOException {
        this();
        read (new BufferedReader (new FileReader (file)), false);
    }

    private PolygonalSection[] sections;

    public PolygonalSection[] getSections() {
        return sections;
    }

    public int getNumSections() {
        return sections.length;
    }

    private class Reader extends WavefrontReader 
    {
        Vector<PolygonalSection> sectionList = new Vector<PolygonalSection>(10);

        Reader (java.io.Reader reader) {
            super (reader);
        }

        @Override
        protected boolean processLine (ReaderTokenizer stok) throws IOException {
            if (!super.processLine (stok)) {
                if (stok.sval.equals ("sec")) {
                    Plane p = new Plane();
                    Vector3d nrml = new Vector3d();
                    double offset;
                    if ((nrml.x = scanDouble(stok)) != nrml.x ||
                    (nrml.y = scanDouble(stok)) != nrml.y ||
                    (nrml.z = scanDouble(stok)) != nrml.z ||
                    (offset = scanDouble(stok)) != offset) {
                        throw new IOException (
                            "plane parameters expected, line " + stok.lineno()); 
                    }
                    String name = null;
                    if (nextToken(stok) == StreamTokenizer.TT_WORD) {
                        name = stok.sval;
                    }
                    PolygonalSection sec =
                    new PolygonalSection(new Plane (nrml, offset));
                    sec.setName (name);
                    sectionList.add (sec);
                    return true;
                }
                else {
                    return false; 
                }
            }
            else {
                return true; 
            }
        }
    }

    public void read (java.io.Reader reader, boolean zeroIndexed) throws IOException {
        //System.out.println ("sectioned mesh reader");
        SectionedMesh.Reader wfr = new SectionedMesh.Reader(reader);
        if (zeroIndexed) {
            wfr.setZeroIndexed(true);
        }
        wfr.readMesh (this);
        System.out.println ("num sections " + wfr.sectionList.size());
        // post process section list
        for (Iterator it=wfr.sectionList.iterator(); it.hasNext(); ) {
            PolygonalSection sec = (PolygonalSection)it.next();
            sec.setVertices (this, sec.getPlane());
        }
        sections = (PolygonalSection[])wfr.sectionList.toArray(
            new PolygonalSection[0]);
        //System.out.println ("sections.length="+ sections.length);
    }

    public boolean isWritable() {
        return true;
    }

    public void write (PrintWriter pw, NumberFormat fmt, boolean zeroIndexed) throws IOException {
        super.write (pw, fmt, zeroIndexed);
        // post process section list
        for (int k=0; k<sections.length; k++) {
            pw.print ("s " + sections[k].getPlane().toString (fmt));
            if (sections[k].getName() != null) {
                pw.print ("  " + sections[k].getName()); 
            }
            pw.println ("");
        }
        pw.flush();
    }


    public void write(PrintWriter pw, NumberFormat fmt, boolean zeroIndexed, boolean closed) throws IOException {

        super.write(pw, fmt, zeroIndexed);
        // Write out the closing faces on the end to create closed Volume
        if (closed) {
            pw.print("f ");
            for (int i = 0; i < sections[0].numVertices(); i++) {
                if (zeroIndexed) {
                    pw.print(sections[0].getVertex(i).getIndex() + " ");
                }
                else {
                    pw.print(sections[0].getVertex(i).getIndex() + 1 + " ");
                }
            }
            pw.print("\n");
            pw.print("f ");
            for (int i=sections[sections.length-1].numVertices()-1; i>=0 ; i--) {
                if (zeroIndexed) {
                    pw.print(sections[sections.length - 1].getVertex(i).getIndex()
                        + " ");
                }
                else {
                    pw.print(sections[sections.length - 1].getVertex(i).getIndex()
                        + 1 + " ");
                }
            }
            pw.print("\n");
        }

        // post process section list
        for (int k = 0; k < sections.length; k++) {
            pw.print("s " + sections[k].getPlane().toString(fmt));
            if (sections[k].getName() != null) {
                pw.print("  " + sections[k].getName());
            }
            pw.println("");
        }
        pw.flush();
    }



    /**
     * Applies an affine transformation to the vertices
     * of this mesh. The topology of the mesh remains unchanged.
     *
     * @param X affine transformation
     */
    public void transform (AffineTransform3dBase X) {
        super.transform(X);
        for (int i=0; i<sections.length; i++) {
            sections[i].getPlane().transform (X);
        }
    }

    /**
     * Applies an inverse affine transformation to the vertices of this
     * mesh. The topology of the mesh remains unchanged.
     *
     * @param X affine transformation
     */
    public void inverseTransform (AffineTransform3dBase X) {
        super.inverseTransform(X);
        for (int i=0; i<sections.length; i++) {
            sections[i].getPlane().inverseTransform (X);
        }
    }

    /**
     * Applies a transformation to the vertices
     * of this mesh. The topology of the mesh remains unchanged.
     *
     * @param T transformation
     */
    public void transform (VectorTransformer3d T) {
        super.transform(T);
        for (int i=0; i<sections.length; i++) {
            sections[i].getPlane().transform (T);
        }
    }

    /**
     * Applies an inverse transformation to the vertices of this
     * mesh. The topology of the mesh remains unchanged.
     *
     * @param T transformation
     */
    public void inverseTransform (VectorTransformer3d T) {
        super.inverseTransform(T);
        for (int i=0; i<sections.length; i++) {
            sections[i].getPlane().inverseTransform (T);
        }
    }

    public void scale (double s) {
        super.scale (s);
        for (int i=0; i<sections.length; i++) {
            sections[i].getPlane().scale (s);
        }
    }

}
