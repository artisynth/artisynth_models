package artisynth.models.larynx_QL2.components;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import maspack.matrix.Vector3d;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.larynx_QL2.VocalTractBase;

public class LegacyReaders {

	public static enum AnsysReadMethod {
		READ_NODES,
		READ_ELEMENTS,
		READ_FIBERS;
	}

	/**
	 * A class to read muscle fiber definitions from ICP's Fibres.mac
	 * 
	 */
	public static class AnsysMuscleFiberReader {

		public static final int rStyInsertionAnsys = 4037;
		public static final int lStyInsertionAnsys = 4042;
		public static final int rStyInsertionArtiSynth = 947;
		public static final int lStyInsertionArtiSynth = 946;


		/**
		 * for testing
		 */
		public static void main (String[] args) {

			FemMuscleModel model = new FemMuscleModel ();
			try {
				AnsysMuscleFiberReader.read (model, new FileReader (
						ArtisynthPath.getSrcRelativePath (VocalTractBase.class, "geometry/Fibers.mac")));
			}
			catch (IOException e) {
				e.printStackTrace ();
			}

			System.out.println("done MacMuscleDef test");

		}


		/** 
		 * Adds muscle fibers and associated with bundles to the specified FEM model
		 * 
		 * @param model FEM model to be populated by muscle bundles
		 * @param reader reader from which to read .mac file
		 * @throws IOException if this is a problem reading the file
		 */
		public static void read (
				FemMuscleModel model, Reader reader)
						throws IOException {


			FibreIndicesInfo fibreInfo = new FibreIndicesInfo ();
			readFibreIndices (fibreInfo, reader);
			addMuscles(model, fibreInfo);

		}


		public static void readFibreIndices (
				FibreIndicesInfo fibreInfo, Reader reader)
						throws IOException {


			ReaderTokenizer rtok = new ReaderTokenizer (new BufferedReader (reader));
			rtok.commentChar ('!');
			rtok.wordChar ('_');

			findSubString ("nb_max_fibres", '=', rtok);
			int max_fibers = rtok.scanInteger ();

			findSubString ("nb_max_noeuds", '=', rtok);
			int max_nodes = rtok.scanInteger ();

			findSubString ("nb_muscles", '=', rtok);
			int max_muscles = rtok.scanInteger ();


			fibreInfo.nodeIndices = new int[max_muscles][max_fibers][max_nodes];
			fibreInfo.numNodesPerFiber = new int[max_muscles][max_fibers];
			fibreInfo.numFibersPerMuscle = new int[max_muscles];
			fibreInfo.numMuscles = 0;

			String strtok = null;
			while ((strtok = findSubString ("FIBER", '(', rtok)) != null) {
				if (strtok.compareTo ("FIBER") == 0.0) {
					int ni = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (',');
					int fi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (',');
					int mi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (')');
					rtok.scanCharacter ('=');
					int nodeIndex = rtok.scanInteger ();
					fibreInfo.nodeIndices[mi][fi][ni] = AnsysToArtisynthNodeIdx(nodeIndex);
				}
				if (strtok.compareTo ("NB_NODES_FIBER") == 0.0) {
					int fi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (',');
					int mi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (')');
					rtok.scanCharacter ('=');
					int numNodes = rtok.scanInteger ();
					fibreInfo.numNodesPerFiber[mi][fi] = numNodes;
				}
				else if (strtok.compareTo ("NB_FIBERS") == 0.0) {
					int mi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (')');
					rtok.scanCharacter ('=');
					int numFibers = rtok.scanInteger ();
					fibreInfo.numFibersPerMuscle[mi] = numFibers;
					fibreInfo.numMuscles++;
				}
			}


		}

		private static int AnsysToArtisynthNodeIdx(int ansysNodeIdx) {
			switch(ansysNodeIdx) {
			case rStyInsertionAnsys:
				return rStyInsertionArtiSynth;
			case lStyInsertionAnsys:
				return lStyInsertionArtiSynth;
			default:
				return ansysNodeIdx-1;
			}
		}



		private static String findSubString (String str, Character expectedNextToken, ReaderTokenizer rtok)
				throws IOException {

			while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
				if (rtok.ttype == ReaderTokenizer.TT_WORD && rtok.sval.contains (str)) {
					String strtok = rtok.sval;
					if (expectedNextToken != null) {
						try {
							// try to scan next character token
							rtok.scanCharacter (expectedNextToken);
						}
						catch (Exception e) {
							rtok.pushBack (); // wrong next character found, continue search
							continue;
						}
					}
					return strtok;
				}

			}
			return null;
		}

		public static void addMuscles (
				FemMuscleModel fem, FibreIndicesInfo fibreInfo) {

			if (fibreInfo.numMuscles > fem.getMuscleBundles ().size ()) {
				return;
			}

			for (int mi = 0; mi < fibreInfo.numMuscles; mi++) {
				MuscleBundle b = fem.getMuscleBundles ().get (mi);
				for (int fi = 0; fi < fibreInfo.numFibersPerMuscle[mi]; fi++) {
					LinkedList<Muscle> fascicle = new LinkedList<Muscle>();
					for (int ni = 0; ni < fibreInfo.numNodesPerFiber[mi][fi]-1; ni++) {
						Muscle m = new Muscle();
						m.setFirstPoint (fem.getNode (fibreInfo.nodeIndices[mi][fi][ni]));
						m.setSecondPoint (fem.getNode (fibreInfo.nodeIndices[mi][fi][ni+1]));
						b.addFibre (m);
						fascicle.add (m);

						// generic default muscle parameters
						ConstantAxialMuscle mat = new ConstantAxialMuscle();
						//               PeckAxialMuscle mat = new PeckAxialMuscle();
						//               LinearAxialMuscle mat = new LinearAxialMuscle();
						mat.setForceScaling (1.0);

						//               BlemkerAxialMuscle mat = new BlemkerAxialMuscle();
						//               mat.setExpStressCoeff(0);

						mat.setMaxForce (1.0);
						double len = m.getFirstPoint ().distance (m.getSecondPoint ());
						mat.setOptLength (len);
						mat.setMaxLength (2*len);
						m.setMaterial(mat);
					}
					b.addFascicle (fascicle);
				}
			}

		}


		public static class FibreIndicesInfo {
			public int[][][] nodeIndices;
			public int[][] numNodesPerFiber;
			public int[] numFibersPerMuscle;
			public int numMuscles;


		}

	}

	/**
	 * A class to read muscle group and element definitions ICP's CreateMuscles.mac
	 * 
	 */
	public static class AnsysMuscleElemReader {

		/**
		 * for testing
		 */
		public static void main(String[] args) {

			FemMuscleModel model = new FemMuscleModel ();
			try {
				AnsysMuscleElemReader.read (model, new FileReader(ArtisynthPath.getSrcRelativePath(VocalTractBase.class, "geometry/CreateMuscles.mac")));
			}
			catch (IOException e) {
				e.printStackTrace ();
			}

			System.out.println("done MacMuscleDef test");

		}


		/** 
		 * Adds muscle bundles and associated elements to the specified FEM model
		 * 
		 * @param model FEM model to be populated by muscle bundles
		 * @param reader reader from which to read .mac file
		 * @throws IOException if this is a problem reading the file
		 */
		public static void read(FemMuscleModel model, Reader reader) throws IOException {

			ArrayList<String> bundleNames = new ArrayList<String> ();
			ArrayList<ArrayList<Integer>> bundleElemIdxs = new ArrayList<ArrayList<Integer>> ();

			readBundleNames (reader, bundleNames, bundleElemIdxs);

			for (int i = 0; i < bundleNames.size (); i++) {
				MuscleBundle mb = new MuscleBundle ();
				mb.setName (bundleNames.get (i));
				model.addMuscleBundle(mb);

				ArrayList<Integer> elemIdxs = bundleElemIdxs.get (i);
				for (Integer idx : elemIdxs) {
					// ansys elems are one indexed, convert to zero-indexed
					if (model.numElements () >= idx) {
						MuscleElementDesc mf = new MuscleElementDesc ();
						mf.setElement (model.getElement (idx - 1));
						mf.setDirection(Vector3d.X_UNIT);
						mb.addElement(mf);
					}
				}
			}
		}


		private static void readBundleNames (Reader reader, ArrayList<String> bundleNames, ArrayList<ArrayList<Integer>> bundleElemIdxs) throws IOException {


			ReaderTokenizer rtok = new ReaderTokenizer (new BufferedReader (reader));
			rtok.commentChar ('!');
			int numbundles = 0;
			if (findString("nb_muscles", rtok)) {
				rtok.scanCharacter ('=');
				numbundles = rtok.scanInteger ();
			}



			for (int i = 0; i < numbundles; i++) {
				findString ("muscleName", '(', rtok);
				rtok.scanInteger ();
				rtok.scanCharacter (')');
				rtok.scanCharacter ('=');
				rtok.nextToken ();
				bundleNames.add (rtok.sval);

			}

			for (int i = 0; i < numbundles; i++) {
				ArrayList<Integer> elemIdxs = new ArrayList<Integer> ();
				while (rtok.nextToken () != ReaderTokenizer.TT_WORD
						|| rtok.sval.compareTo ("CM") != 0.0) {
					findString ("ELEM", ',', rtok);
					rtok.scanCharacter (',');
					int startidx = rtok.scanInteger ();
					rtok.nextToken ();
					if (rtok.ttype != ',') {
						// single idx only, continue to next ELEM token
						elemIdxs.add (startidx);
						rtok.pushBack ();
						continue;
					}
					// idx range specified, read all
					int endidx = rtok.scanInteger ();
					rtok.nextToken ();
					int inc;
					if (rtok.ttype != ',') {
						inc = 1;
						rtok.pushBack ();
					}
					else { // explicit increment specified, read inc 
						inc = rtok.scanInteger ();
					}

					for (int idx = startidx; idx <= endidx; idx+=inc) {
						elemIdxs.add (idx);
					}

				}
				rtok.scanCharacter (',');
				rtok.nextToken ();
				//String groupName = rtok.sval;

				bundleElemIdxs.add (elemIdxs);
			}

		}

		private static boolean findString (String str, ReaderTokenizer rtok) throws IOException { return findString (str, null, rtok); }

		private static boolean findString (String str, Character expectedNextToken, ReaderTokenizer rtok) throws IOException {

			boolean foundString = false;
			while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
				if (rtok.ttype == ReaderTokenizer.TT_WORD && rtok.sval.compareTo (str) == 0.0) {
					if (expectedNextToken != null) {
						try {
							// try to scan next character token
							rtok.scanCharacter (expectedNextToken);
						}
						catch (Exception e) {
							rtok.pushBack (); // wrong next character found, continue search
							continue;
						}
					}
					foundString = true;
					break;
				}

			}
			return foundString;
		}

	}

	/**
	 * A class to read muscle fiber definitions from ICP's Fibres.mac
	 * 
	 */
	public static class AnsysFaceMuscleFiberReader {


		/**
		 * Adds muscle fibers to the face FEM model. Based on AnsysMuscleFiberReader
		 * for tongue model, with following modifications: 
		 * -- "nb_muscles" is first in file 
		 * -- order of FIBER, and NB_NODES_FIBER indices are reversed 
		 * -- nodeIds are non-sequential, therefore require hashmap argument, 
		 *    which is generated in AnsysReader.readNodeFile()
		 * 
		 * @param model
		 * FEM face model to be populated by muscle bundles
		 * @param nodeIdToIndex
		 * mapping from node ids to fem marker indices
		 * @param fibreFileReader
		 * reader for the face muscle topology .mac file
		 * @throws IOException
		 * if this is a problem reading the file
		 */
		public static void read(FemMuscleModel model, HashMap<Integer,Integer> nodeIdToIndex, Reader fibreFileReader) throws IOException {


			ReaderTokenizer rtok = new ReaderTokenizer (new BufferedReader (fibreFileReader));
			rtok.commentChar ('!');
			rtok.wordChar ('_');


			findSubString ("nb_muscles", '=', rtok);
			int max_muscles = rtok.scanInteger ();

			findSubString ("nb_max_fibres", '=', rtok);
			int max_fibers = rtok.scanInteger ();

			findSubString ("nb_max_noeuds", '=', rtok);
			int max_nodes = rtok.scanInteger ();



			int[][][] nodeIds = new int[max_muscles][max_fibers][max_nodes];
			int[][] numNodesPerFiber = new int[max_muscles][max_fibers];
			int[] numFibersPerMuscle = new int[max_muscles];
			int numMuscles = 0;

			String strtok = null;
			while ((strtok = findSubString ("FIBER", '(', rtok)) != null) {
				if (strtok.compareTo ("FIBER") == 0.0) {
					int mi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (',');
					int fi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (',');
					int ni = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (')');
					rtok.scanCharacter ('=');
					int nodeId = rtok.scanInteger ();
					nodeIds[mi][fi][ni] = nodeId;
				}
				if (strtok.compareTo ("NB_NODES_FIBER") == 0.0) {
					int mi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (',');
					int fi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (')');
					rtok.scanCharacter ('=');
					int numNodes = rtok.scanInteger ();
					numNodesPerFiber[mi][fi] = numNodes;
				}
				else if (strtok.compareTo ("NB_FIBERS") == 0.0) {
					int mi = rtok.scanInteger ()-1; // zero-based indexing
					rtok.scanCharacter (')');
					rtok.scanCharacter ('=');
					int numFibers = rtok.scanInteger ();
					numFibersPerMuscle[mi] = numFibers;
					numMuscles++;
				}
			}


			/* debugging */
			//      for (int mi = 0; mi < numMuscles; mi++) {
			//         for (int fi = 0; fi < numFibersPerMuscle[mi]; fi++) {
			//            for (int ni = 0; ni < numNodesPerFiber[mi][fi] - 1; ni++) {
			//               if (nodeIdToIndex.get (nodeIds[mi][fi][ni]) != null)
			//                  System.out.printf("%d %d %d : %d = %d\n",mi,fi,ni,nodeIds[mi][fi][ni],nodeIdToIndex.get (nodeIds[mi][fi][ni]));
			//            }
			//         }
			//      }

			addMuscleFibers(model, nodeIdToIndex, nodeIds, numNodesPerFiber, numFibersPerMuscle, numMuscles);

		}


		private static String findSubString (String str, Character expectedNextToken, ReaderTokenizer rtok)
				throws IOException {

			while (rtok.nextToken () != ReaderTokenizer.TT_EOF) {
				if (rtok.ttype == ReaderTokenizer.TT_WORD && rtok.sval.contains (str)) {
					String strtok = rtok.sval;
					if (expectedNextToken != null) {
						try {
							// try to scan next character token
							rtok.scanCharacter (expectedNextToken);
						}
						catch (Exception e) {
							rtok.pushBack (); // wrong next character found, continue search
							continue;
						}
					}
					return strtok;
				}

			}
			return null;
		}

		public static void addMuscleFibers (
				FemMuscleModel fem, HashMap<Integer,Integer> nodeIdToIndex,
				int[][][] nodeIds, int[][] numNodes, int[] numFibers, int numMuscles) {

			for (int mi = 0; mi < numMuscles; mi++) {
				MuscleBundle b = new MuscleBundle ();
				fem.addMuscleBundle (b);
				for (int fi = 0; fi < numFibers[mi]; fi++) {
					LinkedList<Muscle> fascicle = new LinkedList<Muscle> ();
					for (int ni = 0; ni < numNodes[mi][fi] - 1; ni++) {
						Muscle m = new Muscle ();
						Integer mkr0= nodeIdToIndex.get (nodeIds[mi][fi][ni]);
						Integer mkr1 = nodeIdToIndex.get (nodeIds[mi][fi][ni + 1]);
						if (mkr0 == null || mkr1 == null) {
							continue;
						}
						m.setFirstPoint (fem.markers ().getByNumber (mkr0));
						m.setSecondPoint (fem.markers ().getByNumber (mkr1));
						b.addFibre (m);
						fascicle.add (m);

						setDefaultMuscleFibreProperties(m);

					}
					b.setFibresActive(true);
					b.addFascicle (fascicle);
				}
			}

		}

		public static void setDefaultMuscleFibreProperties(Muscle m) {
			// generic default muscle parameters
			double optLen = m.getFirstPoint ().distance (m.getSecondPoint ());
			ConstantAxialMuscle mat = new ConstantAxialMuscle();
			mat.setAxialMuscleMaterialProps(/*maxForce=*/1.0, optLen, /*maxLen=*/2*optLen, 
					0, 0, 0, /*forceScaling=*/1.0);
			m.setMaterial(mat);
		}

	}
}
