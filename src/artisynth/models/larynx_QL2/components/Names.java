package artisynth.models.larynx_QL2.components;

import java.io.File;

import maspack.geometry.PolygonalMesh;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.models.larynx_QL2.VocalTractBase;

/**
 *  <p>
 *   @author Scott Moisik (Scott.Moisik@ntu.edu.sg), 2017, Nanyang Technological University, Singapore. 
 *   <p>
 *   If you use this model in your research, please cite: 
 *   Moisik, S. R., & Gick, B. (2017). The quantal larynx: the stable regions of laryngeal biomechanics and implications for speech production. Journal of Speech Language and Hearing Research, 60(3), 540-560. https://doi.org/10.1044/2016_JSLHR-S-16-0019

 *  **/
public interface Names {	

	public String getName();
	public String toString();
	
	public static enum Files {
		AIRWAY(".obj", "airway"),
		ATTACHMENTS("_attachments.csv", "attachment"),
		COLLISION_BOX("_collisionBox.obj", "collisionBox"),
		COLLISION_SUBSURFACE("_collisionSubsurface_[num]_[name].obj", "collisionSubsurface"),
		FEM("_fem.csv", "fem"),
		FEM_MUSCLES("_muscles.csv", "muscle"),
		FEM_FILTERS("_filters.csv", "filter"),
		RIGIDBODY(".obj", "rigidbody"),
		SURFACE_MESH("_surfaceMesh.obj", "surface"),
		TISSUES("tissues.csv", "tissue"), 
		INVERSE_TARGETS("_inverseTargets.csv", "inverse"), 
		COLORS("_colors.csv", "color"), 
		MEASUREMENT_POINTS("_measurementPoints.csv", "measurement"), 
		OUTPUT("_simulationOutput.csv", "simulation"), 
		MUSCLE_DESIGNER_DATA("_muscleDesignerData.csv", "muscle designer data"), 
		SCREENSHOT(".png", "png"), 
		MOVIE(".mov", "mov"), 
		MESH_WARP_DESIGNER_DATA("_meshWarpDesignerData.csv", "mesh warp designer data"), 
		AIRWAY_DATA(".csv", "airway"), 
		AUDIO(".wav", "wave audio file");
              		
		String suffix;
		String keyWord;
		Files(String suffix, String keyWord) {
			this.suffix = suffix;
			this.keyWord = keyWord;
		}

		public String toString() { return suffix; }
		public String getSuffix() { return suffix; }
		public String getKeyword() { return keyWord; }
		public String getNumberedSuffix(int number) { return suffix.replace("[num]", Integer.toString(number)); }
		public String getNumberedNamedSuffix(int number, String name) { return getNumberedSuffix(number).replace("[name]", name); }
		/** Concatenates the folders provided in the input argument list and adds the suffix matching the specific enum used. The final string is assumed to be the file name. **/
		public String createFullFileName(String ... pathElements) {
			String path = "";
			for (int i = 0; i < pathElements.length - 1; i++) {
				if (!pathElements[i].endsWith(File.separator)) {
					path += pathElements[i] + File.separator;
				}
				else {
					path += pathElements[i];
				}
			}
			return path + pathElements[pathElements.length - 1] + getSuffix();
		}
		public static String createPath(String ... folders) {
			String path = "";
			for (String folder : folders) {
				if (!folder.endsWith(File.separator)) {
					path += folder + File.separator;
				}
				else {
					path += folder;
				}
			}
			return path;
		}
	}

	public static enum Region implements Names {
		LIP_LOWER("lower lip", new String[]{"lowerLip"}),
		LIP_UPPER("upper lip", new String[]{"upperLip"}),
		VOCAL_FOLD_RIGHT("right vocal fold", new String[]{"rightVocalFold"}),
		VOCAL_FOLD_LEFT("left vocal fold", new String[]{"leftVocalFold"}),
		ARYEPIGLOTTIC_FOLD_RIGHT("right aryepiglottic fold", new String[]{"rightAEFold"}),
		ARYEPIGLOTTIC_FOLD_LEFT("left aryepiglottic fold", new String[]{"leftAEFold"}),
		FALSE_FOLD_RIGHT("right ventricular fold", new String[]{"rightFalseFold"}),
		FALSE_FOLD_LEFT("left ventricular fold", new String[]{"leftFalseFold"}),
		EPIGLOTTIC_TUBERCLE("epiglottic tubercle", new String[]{"epiglotticTubercle"}),
		PHAYNX("pharynx", new String[]{"pharyn"});

		private String name;
		private String[] sharedNames;
		Region(String name, String[] sharedNames) {
			this.name = name;
			this.sharedNames = sharedNames;
		}
		public String getName() { return name; }
		public String[] getSharedNameComponent() { return sharedNames; }
		public boolean checkName(String name) {
			for (String sharedName : sharedNames) {
				if (name.contains(sharedName)) {
					return true;
				}
			}
			return false;
		}
		public PolygonalMesh getMesh() {
			return Structures.getCollisionMeshByRegion(this);
		}
		
	}
	
	/** An enumeration which defines all predetermined vocal tract configurations. 
	 * 	Each member of the enum specifies takes an argument list with one string and several booleans indicating FEM usage: 
	 * 	<b>("model folder name", face, larynx, tongue, velum)</b>   
	 *  
	 */
	public static enum VocalTractType {
		NEW_TONGUE("newstructures", false, false, true, false),
		
		//Badin definitions
		BADIN_SKELETAL("badin", false, false, false, false),

		BADIN_FACE("badin", true, false, false, false),
		BADIN_FACE_LARYNX("badin", true, true, false, false),
		BADIN_FACE_TONGUE("badin", true, false, true, false),

		BADIN_LARYNX("badin", false, true, false, false),
		BARIN_LARYNX_TONGUE("badin", false, true, true, false),

		BADIN_TONGUE("badin", false, false, true, false),

		BADIN_ALL("badin", true, true, true, false),

		//Frank definitions
		FRANK_ALL("frank", true, true, true, true),
		FRANK_VELUM("frank", false, false, false, true),
		
		//Maal definitions
		MAAL_SKELETAL("maal", false, false, false, false),

		MAAL_FACE("maal", true, false, false, false),
		MAAL_FACE_LARYNX("maal", true, true, false, false),
		MAAL_FACE_TONGUE("maal", true, false, true, false),

		MAAL_LARYNX("maal", false, true, false, false),
		MAAL_LARYNX_TONGUE("maal", false, true, true, false),

		MAAL_TONGUE("maal", false, false, true, false),

		MAAL_ALL("maal", true, true, true, false),

		//Simulation definitions
		SIM_LABIODENTAL_SKELETAL("simulations\\labiodental\\original", false, false, false, false),
		SIM_LABIODENTAL_ORIGINAL("simulations\\labiodental\\original", true, false, false, false),
		
		SIM_LABIODENTAL_CLASS_II("simulations\\labiodental\\classII", true, false, false, false),
		SIM_LABIODENTAL_CLASS_II_V2("simulations\\labiodental\\classIIV2", true, false, false, false),
		SIM_LABIODENTAL_CLASS_III("simulations\\labiodental\\classIII", true, false, false, false),
		SIM_LABIODENTAL_EDGE_TO_EDGE("simulations\\labiodental\\edge2edge", true, false, false, false),
		
		SIM_LABIODENTAL_EDGE_TO_EDGE_V2("simulations\\labiodental\\edge2edgeV2", true, false, false, false),
		SIM_LABIODENTAL_EDGE_TO_EDGE_SKELETAL("simulations\\labiodental\\edge2edge", false, false, false, false),
		SIM_LABIODENTAL_REMODELING("simulations\\labiodental\\remodeling", true, false, false, false),
		
		//Bare larynx
		BARE_LARYNX("larynx", false, true, false, false),
		
		//Click geometry
		CLICKS_SKELETAL("simulations" + File.separatorChar + "clicks", false, false, false, false),
		CLICKS_TONGUE("simulations" + File.separatorChar + "clicks", false, false, true, false),
		CLICKS_2_TONGUE("simulations" + File.separatorChar + "clicks_2", false, false, true, false),
		
		//Quantal larynx
		QUANTAL_LARYNX_SIZE1_VENTRICLE("quantal larynx" + File.separatorChar + "size 1 ventricle", "QL_VentricleSize1_", false, true, false, false),
		QUANTAL_LARYNX_SIZE1_VENTRICLE_SKELETON_ONLY("quantal larynx" + File.separatorChar + "size 1 ventricle", "QL_VentricleSize1_", false, false, false, false),
		QUANTAL_LARYNX_SIZE1_VENTRICLE_WITH_TONGUE("quantal larynx" + File.separatorChar + "size 1 ventricle", "QL_VentricleSize1_", false, true, true, false),
		
		QUANTAL_LARYNX_SIZE2_VENTRICLE("quantal larynx" + File.separatorChar + "size 2 ventricle", "QL_VentricleSize2_", false, true, false, false),
		QUANTAL_LARYNX_SIZE2_VENTRICLE_SKELETON_ONLY("quantal larynx" + File.separatorChar + "size 2 ventricle", "QL_VentricleSize2_", false, false, false, false),
		QUANTAL_LARYNX_SIZE2_VENTRICLE_WITH_TONGUE("quantal larynx" + File.separatorChar + "size 2 ventricle", "QL_VentricleSize2_", false, true, true, false),
		
		QUANTAL_LARYNX_SIZE3_VENTRICLE("quantal larynx" + File.separatorChar + "size 3 ventricle", "QL_VentricleSize3_", false, true, false, false),
		QUANTAL_LARYNX_SIZE3_VENTRICLE_SKELETON_ONLY("quantal larynx" + File.separatorChar + "size 3 ventricle", "QL_VentricleSize3_", false, false, false, false),
		QUANTAL_LARYNX_SIZE3_VENTRICLE_WITH_TONGUE("quantal larynx" + File.separatorChar + "size 3 ventricle", "QL_VentricleSize3_", false, true, true, false),
		
		LARYNX_QL2("larynx_QL2", "QL_", false, true, false, false),
		LARYNX_QL2_SKELETON_ONLY("larynx_QL2", "QL_", false, false, false, false),
		LARYNX_QL2_WITH_TONGUE("larynx_QL2", "QL_", false, true, true, false),
		
		
		//Test definitions
		TEST_SKELETAL("test", false, false, false, false),

		TEST_FACE("test", true, false, false, false),
		TEST_FACE_LARYNX("test", true, true, false, false),
		TEST_FACE_TONGUE("test", true, false, true, false),

		TEST_LARYNX("test", false, true, false, false),
		TEST_LARYNX_TONGUE("test", false, true, true, false),

		TEST_TONGUE("test", false, false, true, false),

		TEST_ALL("test", true, true, true, false), 
		
		EMPTY("", false, false, false, false); 
		

		private String modelDirectory;
		private String fileNamePrefix = "";
		private boolean useFaceFEM = false;
		private boolean useLarynxFEM = false;
		private boolean useTongueFEM = false;
		private boolean useVelumFEM = false;
		
		VocalTractType(String name, boolean useFaceFEM, boolean useLarynxFEM, boolean useTongueFEM, boolean useVelumFEM) {
			//this.modelDirectory = name + "\\";
			this.modelDirectory = name + File.separator; // peter's edit
			this.useFaceFEM = useFaceFEM;
			this.useLarynxFEM = useLarynxFEM;
			this.useTongueFEM = useTongueFEM;
			this.useVelumFEM = useVelumFEM;
		}
		
		VocalTractType(String name, String shortName, boolean useFaceFEM, boolean useLarynxFEM, boolean useTongueFEM, boolean useVelumFEM) {
			this(name, useFaceFEM, useLarynxFEM, useTongueFEM, useVelumFEM);
			this.fileNamePrefix = shortName;
		}
		
		public String getFileNamePrefix() { return fileNamePrefix; }
		public String getFullFileName(String fileName) { return VocalTractBase.dataDirectory + modelDirectory + fileName; }
		public String getFullFileName(Names.Files fileName) { return VocalTractBase.dataDirectory + modelDirectory + fileName.getSuffix(); }
		public String getModelDirectory() { return VocalTractBase.dataDirectory + modelDirectory; } 
		public boolean useFaceFEM() { return useFaceFEM; }
		public boolean useLarynxFEM() { return useLarynxFEM; }
		public boolean useTongueFEM() { return useTongueFEM; }
		public boolean useVelumFEM() { return useVelumFEM; }
		public boolean isEmpty() { return modelDirectory.equals(File.separator); }
		public boolean isBadin() { return modelDirectory.contains("badin"); }
		public boolean isMaal() { return modelDirectory.contains("maal"); }
              public boolean checkFemUsage(Names name) {
                     if (name.getName().equals(Names.All.FACE.getName()) && useFaceFEM) {
                            return true;
                     }
                     else if (name.getName().equals(Names.All.LARYNX.getName()) && useLarynxFEM) {
                            return true;
                     }
                     else if (name.getName().equals(Names.All.TONGUE.getName()) && useTongueFEM) {
                            return true;
                     }
                     else if (name.getName().equals(Names.All.VELUM.getName()) && useVelumFEM) {
                             return true;
                      }
                     return false;
              }

			
	}

	public static enum All implements Names {
		ANCHOR("anchor"),

		OROPHARYNX("oropharynx"),
		NASOPHARYNX("nasopharynx"),
		LARYNGOPHARYNX("laryngopharynx"),

		ARYTENOID_LEFT("arytenoid_L"),
		ARYTENOID_RIGHT("arytenoid_R"),
		CRICOID("cricoid"),
		CUNEIFORM_LEFT("cuneiform_L"),
		CUNEIFORM_RIGHT("cuneiform_R"),
		EPIGLOTTIS( "epiglottis"),
		THYROID("thyroid"),

		HYOID("hyoid"),
		MANDIBLE("mandible"),
		MAXILLA("maxilla"),

		FACE("face"),
		LARYNX("larynx"),
		TONGUE("tongue"),
		VELUM("velum"),
		
		CRICOTHYROID_PARS_RECTA("criothyroid (pars recta)", "CT"),
		CRICOTHYROID_PARS_OBLIQUE("criothyroid (pars oblique)", "CT"),

		INTERARYTENOID_SUPERIOR_TRANSVERSE("interarytenoid (superior transverse)", "IA"),
		INTERARYTENOID_INFERIOR_TRANSVERSE("interarytenoid (inferior transverse)", "IA"),
		INTERARYTENOID_OBLIQUE(" interarytenoid (oblique)", "IA"),

		THYROHYOID_SUPERIOR("thyrohyoid (superior fibers)", "TH"),
		THYROHYOID_INFERIOR("thyrohyoid (inferior fibers)", "TH"),

		THYROEPIGLOTTIC("thyroepiglottic", "TE"),
		THYROARYTENOID_VOCALIS("thyroarytenoid (vocalis)", "TAv"),

		LATERALCRICOARYTENOID("lateral cricoarytenoid", "LCA"),
		POSTERIOR_CRIOARYTENOID("posterior cricoarytenoid", "PCA"),
		STYLOPHARYNGEUS("stylopharyngeus", "SP"),

		STERNOHYOID("sternohyoid", "SH"),
		STERNOTHYROID("sternothyroid", "ST"),
		STYLOHYOID("stylohyoid", "StyH"),

		POSTERIOR_DIGASTRIC("posterior digastric", "pDG"),
		ANTERIOR_DIGASTRIC("anterior digastric", "aDG"),

		SUPERIOR_PHARYNGEAL_CONSTRICTOR("superior pharyngeal constrictor"),
		MIDDLE_PHARYNGEAL_CONSTRICTOR("middle pharyngeal constrictor"),
		INFERIOR_PHARYNGEAL_CONSTRICTOR("inferior pharyngeal constrictor"),

		MASSETER("masseter"),
		TEMPORALIS_ANTERIOR("temporalis (anterior)"),
		TEMPORALIS_MEDIAL("temporalis (medial)"),
		TEMPORALIS_POSTERIOR("temporalis (posterior)"),
		DIGASTRIC_ANTERIOR("anterior digastric"),
		DIGASTRIC_POSTERIOR("posterior digastric"),
		INTERNAL_PTERYGOID("internal (medial) pterygoid"),
		EXTERNAL_PTERYGOID("external (lateral) pterygoid"),

		GENIOGLOSSUS_POSTERIOR("genioglossus posterior"),
		GENIOGLOSSUS_MEDIAL("genioglossus medial"),
		GENIOGLOSSUS_ANTERIOR("genioglossus anterior"),
		STYLOGLOSSUS("styloglossus"),
		GENIOHYOID("geniohyoid"),
		MYLOHYOID("mylohyoid"),
		OMOHYOID("omohyoid", "OH"),
		HYOGLOSSUS("hyoglossus"),
		VERTICALIS("verticalis"),
		TRANSVERSUS("transversus"),
		INFERIOR_LONGITUDINAL("inferior longitudinal"),
		SUPERIOR_LONGITUDINAL("superior longitudinal"),

		BUCCINATOR("buccinator"),
		DEPRESSOR_LABII_INFERIORIS("depressor labii inferioris"),
		MENTALIS("mentalis"),
		ORBICULARIS_ORIS_MARGINAL("obicularis oris marginal"),
		ORBICULARIS_ORIS_PERIPHERAL("obicularis oris peripheral"),
		LEVATOR_LABII_SUPERIORIS_ALAEQUE_NASI("levator labii superioris alaeque nasi"),
		LEVATOR_ANGULI_ORIS("levator anguli oris"),
		RISORIUS("risorius"),
		ZYGOMATIC("zygomatic"), 
		LIP_LOWER("lowerLip"),
		LIP_UPPER("upperLip"); 
		
		

		String name;
		String abbreviation;
		All(String name) {
			this.name = name;
			this.abbreviation = name;
		}

		All(String name, String abbreviation) {
			this.name = name;
			this.abbreviation = abbreviation;
		}

		public String toString() { return name; }
		public String getName() { return name; }	
		public String getAbbreviation() { return abbreviation; }
	}

	public static enum Points {
		MEASUREMENT("measurement point");
		String name;
		Points(String name) {
			this.name = name;
		}
		public String getName(int number) {
			return name + " " + number;
		}
	}
	
	public interface HasOriginAndInsertion {
		public String getName();
		public String getSide();
	}
	
	public static enum Diarthrosis implements Names {
		CRICOARYTENOID_JOINT_PLANE("cricoarytenoid joint"),
		SOFT_PALATE_PLANE("soft palate"),
		POSTERIOR_PHARYNGEAL_WALL_PLANE("posterior pharyngeal wall");
		
		String name;
		Diarthrosis(String name) {
			this.name = name;
		}
		public String getName() { return name; }
	}
	
	public static enum Ligaments implements Names, HasOriginAndInsertion {
		CRICOTRACHEAL_LIGAMENT_LEFT("cricotracheal ligament", "left"),
		CRICOTRACHEAL_LIGAMENT_RIGHT("cricotracheal ligament", "right"),
		CRICOTRACHEAL_LIGAMENT_ANTERIOR("cricotracheal ligament anterior", ""),
		CRICOTRACHEAL_LIGAMENT_POSTERIOR("cricotracheal ligament posterior", ""),
		VOCAL_LIGAMENT_LEFT("vocal ligament", "left"),
		VOCAL_LIGAMENT_RIGHT("vocal ligament", "right"), 
		TEMPOROMANDIBULAR("temporomandibular", ""), 
		CRICOTHYROID_MEDIAL("medial cricothyroid ligament", "");
		
		String name;
		String side;
		Ligaments(String name, String side) {
			this.name = name;
			this.side = side;
		}

		public String getName() { return name; }
		public String getSide() { return side; }

		public Ligamentous get() {
			return Structures.getLigamentByName(this);
		}
		
		public boolean checkNameAndSide(Ligamentous ligament) {
			if (ligament.getName().contains(name) && ligament.getSide().contains(side)) {
				return true;
			}
			else {
				return false;
			}
		}
	}
	
	public static enum Bodies implements Names {
		ANCHOR("anchor"),

		OROPHARYNX("oropharynx"),
		NASOPHARYNX("nasopharynx"),
		LARYNGOPHARYNX("laryngopharynx"),

		ARYTENOID_LEFT("arytenoid_L"),
		ARYTENOID_RIGHT("arytenoid_R"),
		CRICOID("cricoid"),
		CUNEIFORM_LEFT("cuneiform_L"),
		CUNEIFORM_RIGHT("cuneiform_R"),
		EPIGLOTTIS( "epiglottis"),
		THYROID("thyroid"),

		HYOID("hyoid"),
		MANDIBLE("mandible"),
		MAXILLA("maxilla"),

		FACE("face"),
		LARYNX("larynx"),
		TONGUE("tongue"), 
		VELUM("velum");

		String name;
		Bodies(String name) {
			this.name = name;
		}

		public String getName() { return name; }	
	}

	public interface Muscles extends Names {
		public String getName();
		public String getAbbreviation();

		public static enum Laryngeal implements Muscles {
			CRICOTHYROID("cricothyroid", "CT"),
			CRICOTHYROID_PARS_RECTA("cricothyroid (pars recta)", "CT"),
			CRICOTHYROID_PARS_OBLIQUE("cricothyroid (pars oblique)", "CT"),
			
			INTERARYTENOID("interarytenoid", "IA"),
			INTERARYTENOID_SUPERIOR_TRANSVERSE("interarytenoid (superior transverse)", "IA"),
			INTERARYTENOID_INFERIOR_TRANSVERSE("interarytenoid (inferior transverse)", "IA"),
			INTERARYTENOID_OBLIQUE("interarytenoid (oblique)", "IA"),

			THYROHYOID("thyrohyoid", "TH"),
			THYROHYOID_SUPERIOR("thyrohyoid (superior fibers)", "TH"),
			THYROHYOID_INFERIOR("thyrohyoid (inferior fibers)", "TH"),

			THYROEPIGLOTTIC("thyroepiglottic", "TE"),
			THYROARYTENOID_VOCALIS("thyroarytenoid (vocalis)", "TAv"),

			FEM_THYROARYTENOID_VOCALIS("thyroarytenoid (vocalis; FEM intrinsic", "TAv (fem)"),
			FEM_THYROARYTENOID_MUSCULARIS("thyroarytenoid (muscularis; FEM intrinsic", "TAm (fem)"),
			FEM_THYROEPIGLOTTIC("thyroepiglottic (FEM intrinsic", "TE (fem)"),
			
			FEM_VENTRICULARIS("ventricularis", "Vent"),
			FEM_VENTRICULARIS_POSTEROLATERAL("ventricularis (posterolateral; FEM intrinsic", "Vpl"),
			FEM_VENTRICULARIS_ANTEROLATERAL("ventricularis (anterolateral; FEM intrinsic", "Val"),
			FEM_VENTRICULARIS_ANTEROMEDIAL("ventricularis (anteromedial; FEM intrinsic", "Vam"),
			
			LATERALCRICOARYTENOID("lateral cricoarytenoid", "LCA"),
			POSTERIOR_CRICOARYTENOID("posterior cricoarytenoid", "PCA"),
			STYLOPHARYNGEUS("stylopharyngeus", "SP");

			String name;
			String abbreviation;
			Laryngeal(String name) {
				this.name = name;
				this.abbreviation = name;
			}

			Laryngeal(String name, String abbreviation) {
				this.name = name;
				this.abbreviation = abbreviation;
			}

			public String getName() { return name; }
			public MuscleBundle getFemLeft() {
				return Structures.getFemMuscleByName("left " + name);
			}
			public MuscleBundle getFemRight() {
				return Structures.getFemMuscleByName("right " + name);
			}
			
			public String toString() { return name; }
			public String getAbbreviation() { return abbreviation; }
		}

		public static enum Hyoid implements Muscles {
			STERNOHYOID("sternohyoid", "SH"),
			STERNOTHYROID("sternothyroid", "ST"),
			STYLOHYOID("stylohyoid", "StyH"),
			GENIOHYOID("geniohyoid"),
			MYLOHYOID("mylohyoid"),
			POSTERIOR_DIGASTRIC("posterior digastric"),
			ANTERIOR_DIGASTRIC("anterior digastric"),
			HYOGLOSSUS("hyoglossus"),
			THYROHYOID_SUPERIOR("thyrohyoid (superior fibers)"),
			THYROHYOID_INFERIOR("thyrohyoid (inferior fibers)"),
			OMOHYOID("omohyoid", "OH");
			
			String name;
			String abbreviation;
			Hyoid(String name) {
				this.name = name;
				this.abbreviation = name;
			}

			Hyoid(String name, String abbreviation) {
				this.name = name;
				this.abbreviation = abbreviation;
			}

			public String getName() { return name; }	
			public String getAbbreviation() { return abbreviation; }
		}


		public static enum Pharyngeal implements Muscles {

			STYLOPHARYNGEUS("stylopharyngeus"),
			SUPERIOR_PHARYNGEAL_CONSTRICTOR("superior pharyngeal constrictor"),
			MIDDLE_PHARYNGEAL_CONSTRICTOR("middle pharyngeal constrictor"),
			INFERIOR_PHARYNGEAL_CONSTRICTOR("inferior pharyngeal constrictor");

			String name;
			String abbreviation;
			Pharyngeal(String name) {
				this.name = name;
				this.abbreviation = name;
			}

			Pharyngeal(String name, String abbreviation) {
				this.name = name;
				this.abbreviation = abbreviation;
			}

			public String getName() { return name; }	
			public String getAbbreviation() { return abbreviation; }

		}

		public static enum Mandibular implements Muscles {
			MASSETER("masseter"),
			INTERNAL_PTERYGOID("internal (medial) pterygoid"),
			EXTERNAL_PTERYGOID("external (lateral) pterygoid"),
			TEMPORALIS_ANTERIOR("temporalis (anterior)"),
			TEMPORALIS_MEDIAL("temporalis (medial)"),
			TEMPORALIS_POSTERIOR("temporalis (posterior)"),
			DIGASTRIC_ANTERIOR("anterior digastric"),
			DIGASTRIC_POSTERIOR("posterior digastric"),
			GENIOHYOID("geniohyoid"),
			MYLOHYOID("mylohyoid");


			String name;
			String abbreviation;
			Mandibular(String name) {
				this.name = name;
				this.abbreviation = name;
			}

			Mandibular(String name, String abbreviation) {
				this.name = name;
				this.abbreviation = abbreviation;
			}

			public String getName() { return name; }	
			public String getAbbreviation() { return abbreviation; }

		}


		public static enum Lingual implements Muscles {
			GENIOGLOSSUS_POSTERIOR("genioglossus posterior"),
			GENIOGLOSSUS_MEDIAL("genioglossus medial"),
			GENIOGLOSSUS_ANTERIOR("genioglossus anterior"),
			STYLOGLOSSUS("styloglossus"),
			GENIOHYOID("geniohyoid"),
			MYLOHYOID("mylohyoid"),
			HYOGLOSSUS("hyoglossus"),
			VERTICALIS("verticalis"),
			TRANSVERSUS("transversus"),
			INFERIOR_LONGITUDINAL("inferior longitudinal"),
			SUPERIOR_LONGITUDINAL("superior longitudinal");

			String name;
			String abbreviation;
			Lingual(String name) {
				this.name = name;
				this.abbreviation = name;
			}

			Lingual(String name, String abbreviation) {
				this.name = name;
				this.abbreviation = abbreviation;
			}

			public String getName() { return name; }	
			public String getAbbreviation() { return abbreviation; }
		}


		public static enum Facial implements Muscles {
			BUCCINATOR("buccinator"),
			DEPRESSOR_LABII_INFERIORIS("depressor labii inferioris"),
			MENTALIS("mentalis"),
			ORBICULARIS_ORIS_MARGINAL("obicularis oris marginal"),
			ORBICULARIS_ORIS_PERIPHERAL("obicularis oris peripheral"),
			LEVATOR_LABII_SUPERIORIS_ALAEQUE_NASI("levator labii superioris alaeque nasi"),
			LEVATOR_ANGULI_ORIS("levator anguli oris"),
			RISORIUS("risorius"),
			ZYGOMATIC("zygomatic");

			String name;
			String abbreviation;
			Facial(String name) {
				this.name = name;
				this.abbreviation = name;
			}

			Facial(String name, String abbreviation) {
				this.name = name;
				this.abbreviation = abbreviation;
			}

			public String getName() { return name; }	
			public String getAbbreviation() { return abbreviation; }
		}
	}

}
