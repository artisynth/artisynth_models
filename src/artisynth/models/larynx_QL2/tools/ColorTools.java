package artisynth.models.larynx_QL2.tools;

import java.awt.Color;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.BVFeatureQuery.InsideQuery;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.HexElement;
import artisynth.models.larynx_QL2.components.SoftBody.FemMethods.FemVolumeMethods;
import artisynth.models.larynx_QL2.tools.FemTools.ForgedFem.HexData;
import artisynth.models.larynx_QL2.tools.GeometryTools.NearestFaceProps;
/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public class ColorTools {

	public static Color[] createColorSet(int number) {
		Color[] colors = new Color[number];

		for (int i = 0; i < number; i++) {
			colors[i] = Color.getHSBColor((float)i / (float) number, 1f, 0.75f);	
		}

		return colors;
	}

	public static void colorizeFemByQuadrature(FemMuscleModel fem) {
		for (FemElement3d elem : fem.getElements()) {
			Color color = new Color(0f, 0f, 0f);

			double quadrature = elem.computeVolumes();
			if (quadrature < 0.0) {
				color = Color.green;
			}
			else {
				float value = (float) Math.max(Math.min(quadrature, 1.0), 0.0);
				color = new Color(1f - value, 0f, value);
			}
			RenderProps.setFaceColor(elem, color);
		}
	}

	public static void colorizeFemByCentroidContainment(FemMuscleModel fem, PolygonalMesh mesh, double distanceTolerance) {
		for (FemElement3d element : fem.getElements()) {
			Point3d centroid = new Point3d();
			element.computeCentroid(centroid);

			if (BVFeatureQuery.isInsideMesh(mesh, centroid) == InsideQuery.INSIDE) {
				RenderProps.setFaceColor(element, Color.pink);
			}
			else {
				NearestFaceProps nfp = new NearestFaceProps(mesh, centroid);
				float value = (float) Math.max(Math.min(nfp.projPosDistance / distanceTolerance, 1.0), 0.0);
				RenderProps.setFaceColor(element, new Color(value, 0f, 1f));
			}
		}
	}

	public static void colorizeFemByVolume(FemMuscleModel fem, FemVolumeMethods fvd) {
		double range = fvd.getMaxVolume() - fvd.getMinVolume();
		if (range == 0) { 
			System.err.println("FEM '" + fem.getName() + "' has an ill-defined volume range.");
		}
		else {
			for (FemElement3d elem : fem.getElements()) {
				Color color = new Color(0f, 0f, 0f);

				if (elem.isInvertedAtRest()) {
					color = Color.green;
				}
				else {
					float value = (float)Math.max(Math.min((fvd.getVolume(elem.getNumber()) - fvd.getMinVolume()) / range, 1.0), 0.0);
					color = new Color(1f - value, 0f, value);
				}
				RenderProps.setFaceColor(elem, color);
			}
		}
	}


	public static void colorizeHexFemByQuality(FemMuscleModel fem, double optimalLength) {

		for (FemElement3d elem : fem.getElements()) {
			Color color = new Color(0f, 0f, 0f);

			if (elem.isInvertedAtRest()) {
				color = Color.green;
			}
			else {
				double quality = HexData.calculateHexQuality((HexElement) elem, optimalLength);
				//quality = Math.log(quality*10 + (1.0/Math.E)) + 1.0;
				float value = (float) Math.max(Math.min(quality, 1.0), 0.0);
				color = new Color(1f - value, 0f, value);
			}
			RenderProps.setFaceColor(elem, color);
		}
	}

	public static enum ColorMixer {
		RED, GREEN, BLUE;
	}

	public static Color getIncrementalColor(float r, float g, float b, int currentColor, int maxColors, ColorMixer colorMixer) {

		switch (colorMixer) {
		case BLUE:
			float bDiff = 1f - b;
			float newBlue = Math.min(Math.max(b + bDiff*((float) currentColor / (float) maxColors), b), 1f);
			return new Color(r, g, newBlue);
		case GREEN:
			float gDiff = 1f - g;
			float newGreen = Math.min(Math.max(g + gDiff*((float) currentColor / (float) maxColors), g), 1f);
			return new Color(r, newGreen, b);
		case RED:
			float rDiff = 1f - r;
			float newRed = Math.min(Math.max(r + rDiff*((float) currentColor / (float) maxColors), r), 1f);
			return new Color(newRed, g, b);
		default:
			return new Color(1f, 1f, 1f);
		}
	}

	public static void colorizeHexByQuality(HexElement hex, double quality) {
		//quality = Math.log(quality*10 + (1.0/Math.E)) + 1.0;
		float value = (float) Math.max(Math.min(quality, 1.0), 0.0);
		RenderProps.setFaceColor(hex, new Color(1f - value, 0f, value));
	}


	/** Quality is a proportion ranging from -1.0 to 1.0 **/
	public static Color createPolarityColor(double quality, Color neutralColor, Color negativeColor, Color positiveColor) {
		float r = 0f;
		float g = 0f;
		float b = 0f;
		
		if (quality < 0.0) {
			int tim = 1;
		}
		quality = Math.min(Math.max(quality, -1.0), 1.0);
		
		float[] neu = neutralColor.getColorComponents(null);
		float[] neg = negativeColor.getColorComponents(null);
		float[] pos = positiveColor.getColorComponents(null);
		
		float fp = ((float) Math.abs(quality));
		float bp = 1 - ((float) Math.abs(quality));
		
		if (quality < 0.0) {
			r = neg[0]*fp + neu[0]*bp;
			g = neg[1]*fp + neu[1]*bp;
			b = neg[2]*fp + neu[2]*bp;
		}
		else if (quality > 0.0) {
			r = pos[0]*fp + neu[0]*bp;
			g = pos[1]*fp + neu[1]*bp;
			b = pos[2]*fp + neu[2]*bp;
		}
		else {
			return neutralColor;
		}
		
		r = Math.min(Math.max(r, 0f), 1f);
		g = Math.min(Math.max(g, 0f), 1f);
		b = Math.min(Math.max(b, 0f), 1f);
		return new Color(r, g, b, 1f);
	}

	
	/** Quality is a proportion ranging from 0.0 to 1.0 **/
	public static Color createBicolor(double quality, ColorMixer lowColor, ColorMixer highColor) {
		float r = 0f;
		float g = 0f;
		float b = 0f;
		switch (lowColor) {
		case BLUE:
			b = 1f - ((float) quality);
			break;
		case GREEN:
			g = 1f - ((float) quality);
			break;
		case RED:
			r = 1f - ((float) quality);
			break;
		default:
			break;
		}

		switch (highColor) {
		case BLUE:
			b = ((float) quality);
			break;
		case GREEN:
			g = ((float) quality);
			break;
		case RED:
			r = ((float) quality);
			break;
		default:
			break;
		}
		r = Math.min(Math.max(r, 0f), 1f);
		g = Math.min(Math.max(g, 0f), 1f);
		b = Math.min(Math.max(b, 0f), 1f);
		return new Color(r, g, b, 1f);
	}

}
