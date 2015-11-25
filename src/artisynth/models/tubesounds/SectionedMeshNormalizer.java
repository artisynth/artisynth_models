package artisynth.models.tubesounds;


import maspack.matrix.*;
import argparser.*;
import java.io.*;

/**
 * Reads in a sectioned mesh, and then writes out the vertices
 * that would result if they were all projected onto their
 * respective planes and renormalized with a give spacing.
 */

public class SectionedMeshNormalizer
{
	static StringHolder meshFileName = new StringHolder(null);
	static DoubleHolder spacing = new DoubleHolder (1);
	static BooleanHolder clockwise = new BooleanHolder (false);

	public static void main (String[] args)
	 { 
	   ArgParser parser = new ArgParser("java <classname>");

	   parser.addOption ("-meshFile %s #mesh file name", meshFileName);   
	   parser.addOption ("-spacing %f #desired mesh spacing", spacing);
	   parser.addOption (
"-clockwise %v #output points clockwise about plane normal", clockwise);
	   
	   parser.matchAllArgs(args);

	   if (meshFileName.value == null)
	    { parser.printErrorAndExit ("File name missing");
	    }

	   SectionedMesh mesh = null;
	   try
	    { mesh = new SectionedMesh (new File(meshFileName.value));
	    }
	   catch (Exception e)
	    { e.printStackTrace();
	      System.exit(1);
	    }
	   PolygonalSection[] secs = mesh.getSections();

	   for (int i=0; i<secs.length; i++)
	    { Point3d[] pnts =
		 secs[i].getEvenlySpacedPoints(spacing.value);
	      System.out.println ("[");
	      if (clockwise.value)
	       { for (int j=pnts.length-1; j>=0; j--)
		  { secs[i].getPlane().project (pnts[j], pnts[j]);
		    System.out.println (pnts[j].toString ("%11.6f"));
		  }
	       }
	      else
	       { for (int j=0; j<pnts.length; j++)
		  { secs[i].getPlane().project (pnts[j], pnts[j]);
		    System.out.println (pnts[j].toString ("%11.6f"));
		  }
	       }
	      System.out.println ("]");
	    }

	 }
}

