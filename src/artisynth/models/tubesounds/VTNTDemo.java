package artisynth.models.tubesounds;
import jass.generators.FilterContainer;
import jass.generators.GlottalWave;
import jass.generators.RandOut;
import jass.generators.Silence;
import jass.generators.TubeModel;
import jass.generators.TwoMassModel;
import jass.render.Controller;
import jass.render.FormatUtils;
import jass.render.SourcePlayer;
import jass.utils.FormantsPlotter;

import java.awt.FileDialog;
import java.awt.Frame;
import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Vector;

import javax.swing.JProgressBar;

import maspack.interpolation.Interpolation;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import artisynth.core.driver.Main;
import artisynth.core.modelbase.HasAudio;
import artisynth.core.modelbase.ModelBase;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.moviemaker.MovieMaker;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.Probe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
//import artisynth.models.tongueAirway.AreaFunctionPlotter;

public class VTNTDemo extends RootModel implements Runnable, HasAudio {

    Main myMain;
    TextSpeechDlg textToSpeech;
    boolean renderAudioToFile = false;
    boolean renderText = false;
    boolean normalize = false;
    String tmpAudioFilePath;
    boolean updateTubeShapes;
    double tubeLength; 
    int  nTubeSections;    
    double tubeLengthNasal;
    int nTubeSectionsNasal;
    TubeModel tmNasal;
    double[] tract;// = new double[nTubeSections]; // for presets
    //static final double tubeLength=-1;
    String[] args = {".17","44100",".10","8","40"};
    SourcePlayer player;
    int bufferSize = 512; // JASS buffersize
    FilterContainer filterContainer;
    Controller a_controlPanel; // VT
    // control panel stuff
    String[] names;
    double[] val;
    double[] min;
    double[] max;
    int tubelengthSliderIndex;
    int nAuxSliders;
    int nSliders;
    int currentVowelState = -1; // encodes vowels
    Controller a_controlPanelNasal; // nasal tract
    Controller a_controlPanelRosenberg; // Rosenberg glottal model
    // values of sliders
    double[] valRosenberg;
    double[] minRosenberg;
    double[] maxRosenberg;
    String[] namesRosenberg;
    Controller a_controlPanelTwoMass; // Ishizak-Flanagan model
    int nbuttonsTwoMass;
    int nSlidersTwoMass;
    String[] namesTwoMass;
    double[] valTwoMass;
    double[] minTwoMass;
    double[] maxTwoMass;
    Airway airway=null;
    FormantsPlotter formantsPlotter;
    TwoMassModel twoMassSource; 
    boolean useTwoMassModel =true; // or if false use Rosenberg model
    float srate;
    GlottalWave source;
    TubeModel tm;
    jass.generators.RightLoadedWebsterTube filter;
    jass.generators.RightLoadedWebsterTube filterCopy;
    ProbeHolder probeHolder; // coz Root model can't have probes??

   public static PropertyList myProps =
      new PropertyList (VTNTDemo.class, RootModel.class);

   static VectorNd DEFAULT_NOISE_LEVEL =
      new VectorNd (new double[] { 1, 500, 1000 });

   // XXX Hack! This is a bogus value for DEFAULT_TUBE_SHAPE
   static VectorNd DEFAULT_TUBE_SHAPE = new VectorNd(1);

   static {
      myProps.add ("lungPressure", "lung pressure", 500);
      myProps.add ("qfactor", "q factor", 1);
      myProps.add ("ag0", "ag 0", -0.005);
      myProps.add ("twoMassNoiseLevel", "two mass noise level",
                   DEFAULT_NOISE_LEVEL);
      myProps.add ("tubeNoiseLevel", "tube noise level", DEFAULT_NOISE_LEVEL);
      myProps.add ("vowel", "vowel", 0);
      myProps.add ("tubeShape", "tube shape", DEFAULT_TUBE_SHAPE);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   private void resetControlPanelSliders() {
      a_controlPanelTwoMass.setSliders(
         valTwoMass,minTwoMass,maxTwoMass,namesTwoMass);
   }

   public void setLungPressure (double p) {
      if (valTwoMass != null) {
         valTwoMass[1] = p;
         resetControlPanelSliders();
      }
   }

   public double getLungPressure() {
      if (valTwoMass != null) {
         return valTwoMass[1];
      }
      else {
         return 0;
      }
   }

   public void setQfactor (double q) {
      if (valTwoMass != null) {
         valTwoMass[0] = q;
         resetControlPanelSliders();
      }
   }

   public double getQfactor() {
      if (valTwoMass != null) {
         return valTwoMass[0];
      }
      else {
         return 0;
      }
   }

   public void setAg0 (double ag0) {
      if (valTwoMass != null) {
         valTwoMass[2] = ag0;
         resetControlPanelSliders();
      }
   }

   public double getAg0() {
      if (valTwoMass != null) {
         return valTwoMass[2];
      }
      else {
         return 0;
      }
   }

   public void setTwoMassNoiseLevel (VectorNd vals) {
      if (valTwoMass != null) {
         valTwoMass[3] = vals.get(0);
         valTwoMass[4] = vals.get(1);
         valTwoMass[5] = vals.get(2);
         resetControlPanelSliders();
      }
   }

   public VectorNd getTwoMassNoiseLevel() {
      VectorNd vals = new VectorNd (3);
      if (valTwoMass != null) {
         vals.set (0, valTwoMass[3]);
         vals.set (1, valTwoMass[4]);
         vals.set (2, valTwoMass[5]);
      }
      return vals;
   }

   public void setTubeNoiseLevel (VectorNd vals) {
      if (val != null) {
         val[0] = vals.get(0);
         val[1] = vals.get(1);
         val[2] = vals.get(2);
         resetControlPanelSliders();
      }
   }

   public VectorNd getTubeNoiseLevel() {
      VectorNd vals = new VectorNd (3);
      if (val != null) {
         vals.set (0, val[0]);
         vals.set (1, val[1]);
         vals.set (2, val[2]);
      }
      return vals;
   }

   public void setVowel (int state) {
      if (state != currentVowelState) {
         String stringRep = null;
         double tubeLen = 0;
         switch (state) {
            case 0: {
               stringRep = "a";
               tubeLen = 0.17;
               break;
            }
            case 1: {
               stringRep = "o";
               tubeLen = 0.185;
               break;
            }
            case 2: {
               stringRep = "u";
               tubeLen = 0.195;
               break;
            }
            case 3: {
               stringRep = "i_";
               tubeLen = 0.19;
               break;
            }
            case 4: {
               stringRep = "i";
               tubeLen = 0.165;
               break;
            }
            case 5: {
               stringRep = "e";
               tubeLen = 0.165;
               break;
            }
            case 6: {
               stringRep = "-";
               tubeLen = 0.17;
               break;
            }
            default: {
               System.out.println ("Unknown vowel state "+state+", ignoring");
            }
         }
         if (stringRep != null) {
            System.out.println("new vowel state="+state);            
            currentVowelState = state;            
            preset(stringRep);
            System.out.println(stringRep);
            handlePresetChangeGlobal(tubeLen);
         }
      }
   }

   public int getVowel() {
      return currentVowelState;
   }

   public void setTubeShape (VectorNd vec) {
      if (vec.size() < nTubeSections+1) {
         System.out.println (
            "Warning: setTubeShape with only "+vec.size()+" params; ignoring");
         return;
      }
      if (val != null) {
         double vals[] = vec.getBuffer();
         double tubeLen = vals[nTubeSections];
         tm.setLength(tubeLen);
         val[tubelengthSliderIndex] = tubeLen; //(just to maintain consistency with sliders)
         for(int k=0;k<nTubeSections;k++) {
            double area = vals[k];
            val[k+nAuxSliders] = area; // in cm^2 (just to maintain consistency with sliders)
            tract[k] = Math.sqrt(area)/100; // radius in meters
            tm.setRadius(k,tract[k]);
         }
         if (updateTubeShapes) {
            a_controlPanel.setSliders(val,min,max,names); //(to move sliders)
         }
         filter.changeTubeModel();
      }
   }

   public VectorNd getTubeShape() {
      VectorNd vec = new VectorNd (nTubeSections+1);
      if (val != null) {
         for (int k=0;k<nTubeSections;k++) {
            vec.set (k, val[k+nAuxSliders]);
         }
         vec.set (nTubeSections, val[tubelengthSliderIndex]);
      }
      return vec;
   }

    public void setRenderAudioToFile(boolean b)
    {
       //System.out.println("Setting render to file = "+b);
       if (player != null)
       {
	  System.out.println("stopping and removing filterCont. from player");
	  player.stopPlaying();
	  player.removeSource(filterContainer);
	  try
	  {
	     Thread.sleep(50);
	  }
	  catch (Exception e)
	  {
	     
	  }
       }       
       renderAudioToFile = b;
       initializePlayer();
    }

    public void onStop()
    {
       if (Main.getMain().getMovieMaker().isAudioNormalized())
       {
	  Vector<Float> floatData = new Vector<Float>();
	  System.out.println("normalizing audio... file name ="+tmpAudioFilePath);
          try
          {
             File file = new File(tmpAudioFilePath);
             BufferedReader in = new BufferedReader(new FileReader(file));
             String line = null;
             while ((line = in.readLine()) != null)
             {
        	//System.out.println(line);
        	floatData.add(Float.valueOf(line));
             }
             in.close();
          }
          catch (IOException e)
          {
             System.err.println(e.getMessage());
          }
          float max = 0.0f;
          long maxI = 0;
          for (int i=0; i<floatData.size(); i++)
          {
             //System.out.println(floatData.get(i));
             if (Math.abs(floatData.get(i)) > max)
             {
        	max = Math.abs(floatData.get(i));
        	maxI = i;
             }
          }
          System.out.println("max value was "+max+" at "+maxI);
          float[] fdata = new float[floatData.size()];
          for (int i=0; i<fdata.length; i++)
          {
             fdata[i] = floatData.get(i)/max;
          }
          if (renderText)
          {  //print as ASCII TEXT
             System.out.println("rendering normalized float text");
             try
             {	
                FileOutputStream outFile = new FileOutputStream(new File(tmpAudioFilePath));
                PrintStream printStream = new PrintStream(outFile);
                for (int i=0; i<fdata.length; i++)
                {
                   printStream.println(fdata[i]);
                }
                printStream.close();
                outFile.close();
             }
             catch (Exception e)
             {
        	System.out.println(e.getMessage());
             }
          }
          else
          {  //print as SHORT BYTES.
             try
             {
        	System.out.println("writing raw file to "+tmpAudioFilePath);
        	System.out.println("length = "+fdata.length);
                FileOutputStream outFile = new FileOutputStream(new File(tmpAudioFilePath));
                byte[] byteData = new byte [2*fdata.length];
                FormatUtils.floatToByte(byteData, fdata);
                System.out.println(byteData.length);
                outFile.flush();
                outFile.write(byteData);
                outFile.close();
             }
             catch (Exception e)
             {
        	
             }
          }
       }
    }
    

    public String getName() {
        return "VTNTDemo rootmodel";
    }
    
// 	public Property getProperty (String name) {
//         Property prop = null;
//         int size = 1;
//         if(name.equals ("lungPressure")) {
//             // lung pressure in two mass model
//             prop = new NumericPropertyBase (name, probeHolder, size) {
//                     public void set (Object obj) {
//                         double vals[] = (double[])obj;
//                         valTwoMass[1] = vals[0];
//                         a_controlPanelTwoMass.setSliders(valTwoMass,minTwoMass,maxTwoMass,namesTwoMass);
//                     }
//                     public Object get() {	
//                         return myValues;	
//                     }		
//                 };
//         } else if(name.equals ("qfactor")) {
//             // q factor (freq.) in two mass model
//             prop = new NumericPropertyBase (name, probeHolder, size) {
//                     public void set (Object obj) {
//                         double vals[] = (double[])obj;
//                         valTwoMass[0] = vals[0];
//                         a_controlPanelTwoMass.setSliders(valTwoMass,minTwoMass,maxTwoMass,namesTwoMass);
//                     }
//                     public Object get() {	
//                         return myValues;	
//                     }		
//                 };
//         } else if(name.equals ("ag0")) {
//             // glottal rest area in cm^2 in two mass model
//             prop = new NumericPropertyBase (name, probeHolder, size) {
//                     public void set (Object obj) {
//                         double vals[] = (double[])obj;
//                         valTwoMass[2] = vals[0];
//                         a_controlPanelTwoMass.setSliders(valTwoMass,minTwoMass,maxTwoMass,namesTwoMass);
//                     }
//                     public Object get() {	
//                         return myValues;	
//                     }		
//                 };
//         } else if (name.equals("twoMassNoiseLevel")) {
//             prop = new NumericPropertyBase (name, probeHolder, 3) {
//                	  public void set (Object obj) {
//                	     double vals[] = (double[])obj;
//                	     valTwoMass[3] = vals[0];
//                	     valTwoMass[4] = vals[1];
//                	     valTwoMass[5] = vals[2];
//                	     a_controlPanelTwoMass.setSliders(valTwoMass,minTwoMass,maxTwoMass,namesTwoMass);
//                	  }
//                	  public Object get() {
//                	     return myValues;
//                	  }
//             };
//         } else if (name.equals("tubeNoiseLevel")) {
//             prop = new NumericPropertyBase (name, probeHolder, 3) {
//                	 public void set (Object obj) {
//                	    double vals[] = (double[])obj;
//                	    val[0] = vals[0];
//                	    val[1] = vals[1];
//                	    val[2] = vals[2];
//                	 }
//                	 public Object get() {
//                	    return myValues;
//                	 }
//             };
//         }
// //        else if (name.equals("nasal")) {
// //            prop = new NumericPropertyBase (name, probeHolder, 2) {
// //               	public void set(Object obj) {
// //               	   double vals[] = (double[])obj;
// //               	   filter.velumNasal = vals[0];
// //               	   filterCopy.velumNasal = vals[0];
// //               	   filter.mouthNoseBalance = vals[1];
// //               	   filterCopy.mouthNoseBalance = vals[1];
// //               	}
// //               	public Object get() {
// //               	   System.out.println("getting nasal values...");
// //               	   return myValues;
// //               	}
// //            };
// //        }
//         else if(name.equals ("vowel")) {
//             // encode 7 vowels in order a o u _i_ i e - to intervals [0 1][1 2]etc.
//             prop = new NumericPropertyBase (name, probeHolder, size) {
//                     public void set (Object obj) {
//                         double vals[] = (double[])obj;
//                         double q = vals[0];
//                         // System.out.println(q);
//                         int newVowelState = (int)q;
//                         if(newVowelState != currentVowelState) {
//                             System.out.println("vowelstate="+currentVowelState);
//                             currentVowelState = newVowelState;
//                             if(0<=q && q<1) {
//                                 preset("a");
//                                 System.out.println("a");
//                                 double tubeLen = .17;
//                                 handlePresetChangeGlobal(tubeLen);
//                             } else if(1<=q && q<2) {
//                                 preset("o");
//                                 System.out.println("o");
//                                 double tubeLen = .185;
//                                 handlePresetChangeGlobal(tubeLen);
//                             } else if(2<=q && q<3) {
//                                 preset("u");
//                                 double tubeLen = .195;
//                                 handlePresetChangeGlobal(tubeLen);
//                             } else if(3<=q && q<4) {
//                                 preset("i_");
//                                 double tubeLen = .19;
//                                 handlePresetChangeGlobal(tubeLen);
//                             } else if(4<=q && q<5) {
//                                 preset("i");
//                                 double tubeLen = .165;
//                                 handlePresetChangeGlobal(tubeLen);
//                             } else if(5<=q && q<6) {
//                                 preset("e");
//                                 double tubeLen = .165;
//                                 handlePresetChangeGlobal(tubeLen);
//                             } else if(6<=q && q<7) {
//                                 preset("-");
//                                 double tubeLen = .17;
//                                 handlePresetChangeGlobal(tubeLen);
//                             }
//                         }
//                     }
//                     public Object get() {
//                         return myValues;
//                     }
//                 };
//         } else if(name.equals ("tubeShape")) {
//             // set areas (cm^2)  at controlpoints (nTubeSections of them
//             // starting at glottis) and length (m) there is a boolean in
//             // handlePresetChangeGlobal()  to  get  the  preset  vlaue  to  be
//             // exported to an input probe
//             prop = new NumericPropertyBase (name, probeHolder, nTubeSections+1) {
//                     public void set (Object obj) {
//                         double vals[] = (double[])obj;
//                         double tubeLen = vals[vals.length-1];
//                         tm.setLength(tubeLen);
//                         val[tubelengthSliderIndex] = tubeLen; //(just to maintain consistency with sliders)
//                         for(int k=nAuxSliders;k<nSliders;k++) {
//                             val[k] = vals[k-nAuxSliders]; // in cm^2 (just to maintain consistency with sliders)
//                             tract[k-nAuxSliders] = Math.sqrt(val[k])/100; // radius in meters
//                             tm.setRadius(k-nAuxSliders,tract[k-nAuxSliders]);
//                         }
//                         if (updateTubeShapes)
//                         {
//                            a_controlPanel.setSliders(val,min,max,names); //(to move sliders)
//                         }
//                         filter.changeTubeModel();
//                     }
//                     public Object get() {
//                         return myValues;
//                     }
//                 };
//         } 
//         return prop;
//     }

    public void addInputProbes() {
        
        //lung pressure in cm H2O for two mass model
        {
            Property prop = getProperty ("lungPressure");
            System.out.println ("prop=" + prop);
            NumericInputProbe probe =
               new NumericInputProbe(prop,probeHolder);
            probe.setName("lung pressure (cm H2O)");
            String pn = "HI_lung.txt";
            String fn= ArtisynthPath.getSrcRelativePath(this, pn);
            try {  
                probe.setAttachedFileName(fn, "%8.3f");
                probe.setStartStopTimes (1, 20);
                probe.setInterpolationOrder(Interpolation.Order.Linear);
                probe.load();
            } catch(Exception e) {
                System.out.println("Probe file not found. "+fn+" "+e);
            }
            addInputProbe (probe);
        }
        
        //q factor (freq.) for two mass model
        {
            Property prop = getProperty ("qfactor");
            NumericInputProbe probe =  new NumericInputProbe(prop,probeHolder);
            probe.setName("pitch factor (q)");
            String pn = "HI_pitch.txt";
            String fn= ArtisynthPath.getSrcRelativePath(this, pn);
            try {  
                probe.setAttachedFileName(fn, "%8.3f");
                probe.setStartStopTimes (1, 20);
                probe.setInterpolationOrder(Interpolation.Order.Linear);
                probe.load();
            } catch(Exception e) {
                System.out.println("Probe file not found. "+fn+" "+e);
            }
            addInputProbe (probe);
        }
        
        // glottal rest area in cm^2 in two mass model
        {
            Property prop = getProperty ("ag0");
            NumericInputProbe probe =  new NumericInputProbe(prop,probeHolder);
            probe.setName("glottal ret area (cm^2)");
            String pn = "HI_glot.txt";
            String fn= ArtisynthPath.getSrcRelativePath(this, pn);
            try {  
                probe.setAttachedFileName(fn, "%8.3f");
                probe.setStartStopTimes (1, 20);
                
                probe.load();
                probe.setInterpolationOrder(Interpolation.Order.Linear);
            } catch(Exception e) {
                System.out.println("Probe file not found. "+fn+" "+e);
            }
            addInputProbe (probe);
        }
//      Noise attributes in twomass model
        {
            Property prop = getProperty ("twoMassNoiseLevel");
            NumericInputProbe probe =
               new NumericInputProbe(prop,probeHolder);
            probe.setName("twoMass noise attributes");
            String pn = "HI_noise_tm.txt";
            String fn= ArtisynthPath.getSrcRelativePath(this, pn);
            try {  
                probe.setAttachedFileName(fn, "%8.3f");
                probe.setStartStopTimes (1, 20);
                
                probe.load();
                probe.setInterpolationOrder(Interpolation.Order.Linear);
            } catch(Exception e) {
                System.out.println("Probe file not found. "+fn+" "+e);
            }
            addInputProbe (probe);
        }
        //  NoiseLevel attributes in main tube
        {
            Property prop = getProperty ("tubeNoiseLevel");
            NumericInputProbe probe =
               new NumericInputProbe(prop,probeHolder);
            probe.setName("tube noise attributes");
            String pn = "HI_noise_main.txt"; //for now load the same default file to avoid clutter
            String fn= ArtisynthPath.getSrcRelativePath(this, pn);
            try {  
                probe.setAttachedFileName(fn, "%8.3f");
                probe.setStartStopTimes (1, 20);
                
                probe.load();
                probe.setInterpolationOrder(Interpolation.Order.Linear);
            } catch(Exception e) {
                System.out.println("Probe file not found. "+fn+" "+e);
            }
            addInputProbe (probe);
        }
        //vowel
//        {
//            Property prop = getProperty ("vowel");
//            NumericInputProbe probe =  new NumericInputProbe(prop,probeHolder);
//            probe.setName("vowel");
//            String pn = "vowelIP.txt";
//            String fn= ArtisynthPath.getHomeRelativePath(
//                                                  "src/artisynth/models/tubesounds/"+pn,".");
//            try {  
//                probe.setAttachedFileName(fn, "%8.3f");
//                probe.setStartStopTimesSec (1, 20);
//                probe.setInterpolationOrder(Interpolation.Order.Step);
//                probe.load();
//            } catch(Exception e) {
//                System.out.println("Probe file not found. "+fn+" "+e);
//            }
//            addInputProbe (probe);
//            probe.setActive(false);
//        }
        
        //tubeshape
        {
            Property prop = getProperty ("tubeShape");
            NumericInputProbe probe =  new NumericInputProbe(prop,probeHolder);
            probe.setName("tubeShape");
            String pn = "HI_tube.txt";
            String fn= ArtisynthPath.getSrcRelativePath(this, pn);
            try {  
                probe.setAttachedFileName(fn, "%8.3f");
                probe.setStartStopTimes (1, 20);
                probe.setInterpolationOrder(Interpolation.Order.Linear);
                probe.load();
            } catch(Exception e) {
                System.out.println("Probe file not found. "+fn+" "+e);
            }
            addInputProbe (probe);
        }
//        //nasal
//        {
//           Property prop = getProperty("nasal");
//           NumericInputProbe probe =  new NumericInputProbe(prop,probeHolder);
//           probe.setName("nasal");
//           String pn = "nasal.txt";
//           String fn= ArtisynthPath.getHomeRelativePath(
//                    "src/artisynth/models/tubesounds/"+pn,".");
//           try {  
//              	probe.setAttachedFileName(fn, "%8.3f");
//              	probe.setStartStopTimesSec (1, 20);
//              	probe.setInterpolationOrder(Interpolation.Order.Linear);
//              	probe.load();
//           } catch(Exception e) {
//              	System.out.println("Probe file not found. "+fn+" "+e);
//           }
//           addInputProbe (probe); 
//        }
    }
    
    public void loadInputProbeFiles(String baseFile) throws Exception
    {
       System.out.println("loading basefile: "+baseFile);
//       ArrayList<Probe> inputProbes = Main.getWorkspace().getInputProbeList();
       for (Probe probe: getInputProbes())
       {
	  System.out.println(probe.getName());
	  if (probe.getName() == "lung pressure (cm H2O)")
	  {
	     probe.setAttachedFileName(baseFile+"_lung.txt");
	  }
	  if (probe.getName() == "pitch factor (q)")
	  {
	     probe.setAttachedFileName(baseFile+"_pitch.txt");
	  }
	  if (probe.getName() == "glottal ret area (cm^2)")
	  {
	     probe.setAttachedFileName(baseFile+"_glot.txt");
	  }
	  if (probe.getName() == "tubeShape")
	  {
	     probe.setAttachedFileName(baseFile+"_tube.txt");
	  }
	  try
	  {
	     probe.load();
	  }
	  catch (IOException e)
	  {
	     System.out.println("EXCEPTION CAUGHT!!!!!");
	     throw new Exception("One or more files did not load, or was invalid", e);
	  }
       }
       Main.getMain().getTimeline().requestUpdateDisplay();
    }


    public String getAbout() {
        return artisynth.core.util.TextFromFile.getTextOrError (
           ArtisynthPath.getSrcRelativeFile (this, "VTNTDemo.txt"));
    }

    private boolean haltMe=false;
    
   @Override
    public void detach(DriverInterface driver) {
        halt();
        System.out.println("halt!!");
    }

    public void halt() {
        haltMe=true;
        player.stopPlaying();
        a_controlPanel.dispose();
        a_controlPanelRosenberg.dispose();
        a_controlPanelTwoMass.dispose();
        a_controlPanelNasal.dispose();
        textToSpeech.dispose();
        if(formantsPlotter != null) {
            formantsPlotter.close();
        }
        if(areaFunctionPlotter != null) {            
           areaFunctionPlotter.close();
        }
    }

    public VTNTDemo() {
        super();
    }
    
    public VTNTDemo(String name) {
        this();
        setName(name);
        Thread thread = new Thread(this);
        thread.setPriority(Thread.MAX_PRIORITY);
        thread.start();
        while(airway==null) {
            try {
                Thread.sleep(50);
            } catch(Exception e){}
        }
        addModel(airway);
        probeHolder = new ProbeHolder();
        addModel(probeHolder);
        addInputProbes();
    }
    
    public VTNTDemo(String[] args) {
        super();
        this.args=args;
        Thread thread = new Thread(this);
        thread.setPriority(Thread.MAX_PRIORITY);
        thread.start();
        while(airway==null) {
            try {
                Thread.sleep(50);
            } catch(Exception e){}
        }
        addModel(airway);
    }
    
    public static void main(String[] args) {
        new VTNTDemo(args);
    }

    private void handleReset() {
        player.setMute(true);
        filter.reset();
        twoMassSource.reset();
        player.resetAGC();
        try {
            Thread.sleep(50);
        } catch(Exception e){};
        player.setMute(false);
        player.resetAGC();
    }
    
    private void handlePresetChangeGlobal(double tubeLen) {
        boolean exportPresets = false;
        tm.setLength(tubeLen);
        val[tubelengthSliderIndex] = tubeLen;
        for(int k=nAuxSliders;k<nSliders;k++) {
            val[k] = 100*tract[k-nAuxSliders]; // in cm!
            val[k] *= val[k];
            val[k] *= Math.PI;
        }
        a_controlPanel.setSliders(val,min,max,names);
        for(int i=0;i<nTubeSections;i++) {
            tm.setRadius(i,tract[i]);
        }
        filter.changeTubeModel();
        handleReset();
        //updateFormantsPlot();
        
        if(exportPresets) {
            for(int k=nAuxSliders;k<nSliders;k++) {
                System.out.printf("%f ",val[k]);
            }
            System.out.println(tubeLen+"\n");
        }
    }
    
    private void updateFormantsPlot() {
        filterCopy.changeTubeModel();
        filterCopy.reset();
        if(formantsPlotter == null) {
            formantsPlotter = new FormantsPlotter();
            formantsPlotter.setLocation(300,500);
        }
        formantsPlotter.plotFormants(filterCopy,srate);
    }

                     
    AreaFunctionPlotter areaFunctionPlotter;
    
    private void updateAreaFunctionPlot() {
        if(areaFunctionPlotter == null) {
            int npoints = 100;
             areaFunctionPlotter = new AreaFunctionPlotter(npoints);
             areaFunctionPlotter.setLocation(300,800);
        }
        areaFunctionPlotter.plot(tm);
    }
    
    private void initializePlayer()
    {
       System.out.println("initializing SourcePlayer...");
       int bufferSizeJavaSound = 1024*10;

       MovieMaker movieMaker = Main.getMain().getMovieMaker();
       renderText = movieMaker.isRenderingAudioToText();
       normalize = movieMaker.isAudioNormalized();
        
       if (renderAudioToFile || renderText)
       {
	  System.out.println(
             "audio file name: "+ movieMaker.getAudioFileName());
	  //if normalizing, write to a temp file first
	  if (normalize)
	  {
	     //System.out.println("normalize. adding .txt to filename");
	     tmpAudioFilePath = movieMaker.getAudioFileName();//+".txt";
	  }
	  else
	  {
	     tmpAudioFilePath = movieMaker.getAudioFileName();
	  }
	  player = new SourcePlayer(bufferSize,srate, tmpAudioFilePath);
       }
       else
       {
	  player = new SourcePlayer(bufferSize,bufferSizeJavaSound,srate);
       }
       try
       {
	  filter.reset();
	  filterContainer.setTime(0);
	  source.setTime(0);
	  twoMassSource.setTime(0);
	  player.addSource(filterContainer);	  
       }
       catch (Exception e) 
       {
	  System.out.println(e.getMessage());
       };
       
       player.AGCOff(); // turn off automatic gain control
       if (!renderAudioToFile)
       {
	  player.start();
       }
       System.out.println("ok.");
       filter.reset();
       player.resetAGC();
       player.setVolume(0.1f);
    }
    
    public void run() {
       
        updateTubeShapes = true;
        textToSpeech = new TextSpeechDlg(this);
        textToSpeech.setVisible(true);
        
        tubeLength = Double.parseDouble(args[0]);
        srate = (float) Double.parseDouble(args[1]);
        tubeLengthNasal = Double.parseDouble(args[2]);
        nTubeSectionsNasal = Integer.parseInt(args[3]); // only for control
        nTubeSections = Integer.parseInt(args[4]); // only for control
        
        //        int nchannels = 1;
        try {
            airway = new Airway(ArtisynthPath.getSrcRelativePath(this, "airway_t_sec.obj"),
                                ArtisynthPath.getSrcRelativePath(this, "fissured-tongue.jpg"));
        } catch(Exception e) {
            System.out.println("File not found: " + e);
        }

        if(args.length != 5) {
            System.out.println("Usage: java VTNTDemo .17 srate nasalLen nNasalSections nTubeSections");
            return;
        }
        tract = new double[nTubeSections]; // radii in m at gridpoints

        // TubeModel will decide how many segments are needed and interpolate
        tm = new TubeModel(nTubeSections);
        tmNasal = new TubeModel(nTubeSectionsNasal);
        //final TubeModel tmNasal = new TubeModel(nTubeSectionsNasal);
        tm.setLength(tubeLength);
        tmNasal.setLength(tubeLengthNasal);



        double c= 350; // vel. of sound
        double minLen = .15;
        double minLenNasal = tubeLengthNasal;
        double theCFLNumber = 0.5; // 0.5 is more stable but coarser grid.
        filter = new jass.generators.RightLoadedWebsterTube(srate,tm,minLen,tmNasal,minLenNasal,theCFLNumber);
        //filter.useLipModel = !filter.useLipModel; // set to false
        filterCopy = new jass.generators.RightLoadedWebsterTube(srate,tm,minLen,tmNasal,minLenNasal,theCFLNumber);
        //filterCopy.useLipModel = filter.useLipModel; // set to false
        filterCopy.setOutputVelocity(true); // to display formants correctly
        filterContainer = new FilterContainer(srate,bufferSize,filter);
        source = new GlottalWave(srate,bufferSize);
        twoMassSource = new TwoMassModel(bufferSize,srate);
        final RandOut randOut= new RandOut(bufferSize);
        final Silence silence= new Silence(bufferSize);
        initializePlayer();
        try {
            if(useTwoMassModel) {
                filter.setTwoMassModel(twoMassSource);
                filterContainer.addSource(source); // add Rosenberg source also
            } else {
                filterContainer.addSource(source);
            }

        } catch(Exception e) {}
        
        preset("a");
        for(int i=0;i<nTubeSections;i++) {
            tm.setRadius(i,tract[i]);
        }
        airway.init(tm);
        filter.changeTubeModel();
        filterCopy.changeTubeModel();
        // set up control panels

        // Vocal tract control panel:        
        int nbuttons = 4 + 7 + 2 + 1;
        nAuxSliders = 9;
        nSliders = nTubeSections+nAuxSliders;
        names = new String[nSliders];
        val = new double[nSliders];
        min = new double[nSliders];
        max = new double[nSliders];
        tubelengthSliderIndex = nAuxSliders-3;
        names[0] = "noiseLevel";
        val[0] = 1; min[0] = 0; max[0] = 10;
        names[1] = "noiseFreq.";
        val[1] = 500; min[1] = 200; max[1] = 10000;
        names[2] = "noiseBW";
        val[2] = 1000; min[2] = 250; max[2] = 10000;
        
        names[3] = "u_xx mult";
        val[3] = 1; min[3] = 0.0; max[3] = 20;
        names[4] = "u mult";
        val[4] = 1; min[4] = 0.0; max[4] = 100;
        names[5] = "wall coeff ";
        val[5] = 1; min[5] = 0; max[5] = 5;
        names[6] = "length ";
        val[6] = tubeLength; min[6] = .15; max[6] = tubeLength*4;
        names[7] = "lipCoeff. ";
        val[7] = 1; min[7] = .05; max[7] = 10;
        names[8] = "VOL ";
        val[8] = .1; min[8] = 0; max[8] = .1;

        double minA = .0;
        double maxA = 20;
        for(int k=nAuxSliders;k<nSliders;k++) {
            names[k] = "A("+new Integer(k-nAuxSliders).toString() + ") ";
            val[k] = 1;
            min[k] = minA;
            max[k] = maxA;
            double r=Math.sqrt(val[k]/Math.PI);
            tm.setRadius(k-nAuxSliders,r/100); // in meters
            //tmAirway.setRadius(k-nAuxSliders,r); // in cm!
        }

        a_controlPanel = new Controller(new java.awt.Frame ("Vocal Tract"),
                                        false,val.length,nbuttons) {
                private static final long serialVersionUID = 1L;

                boolean muted=false;
                

                public void onButton(int k) {
                    switch(k) {
                    case 0:
			//System.out.println("ACG="+player.getAGC());
                        handleReset();
                        break;
                    case 1: {
                        FileDialog fd = new FileDialog(new Frame(),"Save");
                        fd.setMode(FileDialog.SAVE);
                        fd.setVisible(true);
                        saveToFile(fd.getFile());
                    }
                        break;
                    case 2: {
                        FileDialog fd = new FileDialog(new Frame(),"Load");
                        fd.setMode(FileDialog.LOAD);
                        fd.setVisible(true);
                        loadFromFile(fd.getFile());
                        handleReset();
                    }
                        break;
                    case 3: {
                        muted = !muted;
                        player.setMute(muted);
                        player.resetAGC();
                    }
                        break;
                        
                    case 4: {
                        preset("a");
                        double tubeLen = .17;
                        handlePresetChange(tubeLen);
                    }
                        break;
                    case 5: {
                        preset("o");
                        double tubeLen = .185;
                        handlePresetChange(tubeLen);
                    }
                        break;
                    case 6: {
                        preset("u");
                        double tubeLen = .195;
                        handlePresetChange(tubeLen);
                    }
                        break;
                    case 7: {
                        preset("i_");
                        double tubeLen = .19;
                        handlePresetChange(tubeLen);
                    }
                        break;
                    case 8: {
                        preset("i");
                        double tubeLen = .165;
                        handlePresetChange(tubeLen);
                    }
                        break;
                    case 9: {
                        preset("e");
                        double tubeLen = .165;
                        handlePresetChange(tubeLen);
                    }
                        break;
                    case 10: {
                        preset("-");
                        double tubeLen = .17;
                        handlePresetChange(tubeLen);
                    }
                        break;
                    case 11: { //plot formants
                        updateFormantsPlot();
                        updateAreaFunctionPlot();
                    }
                        break;
                    case 12: { //toggle lipmodel
                        filter.useLipModel = !filter.useLipModel;
                        filterCopy.useLipModel = filter.useLipModel;
                        //System.out.println("useLipModel="+filter.useLipModel);
                        handleReset();
                        String name;
                        if(filter.useLipModel) {
                            name = "Lipmodel (is on)";
                        } else {
                            name = "Lipmodel (is off)";
                        }
                        a_controlPanel.setButtonName(name, 12);                       
                    }
                        break;
                    case 13: { //toggle update of tubeshape via probes
                       updateTubeShapes = !updateTubeShapes;
                       String name;
                       if(updateTubeShapes) {
                           name = "no-update";
                       } else {
                           name = "update";
                       }
                       a_controlPanel.setButtonName(name, 13);                       
                    }
                        break;
                    }
                }
                
                private void handlePresetChange(double tubeLen) {
                    boolean exportPresets = false;
                    tm.setLength(tubeLen);
                    val[tubelengthSliderIndex] = tubeLen;
                    for(int k=nAuxSliders;k<nSliders;k++) {
                        val[k] = 100*tract[k-nAuxSliders]; // in cm!
                        val[k] *= val[k];
                        val[k] *= Math.PI;
                    }
                    a_controlPanel.setSliders(val,min,max,names);
                    for(int i=0;i<nTubeSections;i++) {
                        tm.setRadius(i,tract[i]);
                    }
                    filter.changeTubeModel();
                    handleReset();
                    //updateFormantsPlot();
                    
                    if(exportPresets) {
                        for(int k=nAuxSliders;k<nSliders;k++) {
                            System.out.printf("%f ",val[k]);
                        }
                        System.out.println(tubeLen+"\n");
                    }
                }
                
                
                public void onSlider(int k) {
                    switch(k) {
                    case 0:
                        filter.setFlowNoiseLevel(this.val[k]);
                        break;
                    case 1:
                        filter.setFlowNoiseFrequency(this.val[k]);
                        break;
                    case 2:
                        filter.setFlowNoiseBandwidth(this.val[k]);
                        break;
                    case 3:
                        filter.multDSecond = this.val[k];
                        filterCopy.multDSecond = this.val[k];
                        filter.changeTubeModel();
                        break;
                    case 4:
                        filter.multDWall = this.val[k];
                        filterCopy.multDWall = this.val[k];
                        break;
                    case 5:
                        filter.setWallPressureCoupling((double)this.val[k]);
                        filterCopy.setWallPressureCoupling((double)this.val[k]);
                        filter.changeTubeModel();
                        break;
                    case 6:
                        tm.setLength((double)this.val[k]);
                        filter.changeTubeModel();
                        break;
                    case 7:
                        filter.lipAreaMultiplier = (double)this.val[k];
			filterCopy.lipAreaMultiplier = (double)this.val[k];
			filter.changeTubeModel();
                        break;
                    case 8:
                        player.setVolume((float)this.val[k]);
			System.out.println("set vol="+this.val[k]);
                        break;
                    default:
                        double r=Math.sqrt(val[k]/Math.PI);
                        tm.setRadius(k-nAuxSliders,r/100);// in meters
                        //tmAirway.setRadius(k-nAuxSliders,r);// in cm
                        //System.out.println("set r="+r+" at"+k);
                        filter.changeTubeModel();
                        break;
                    }
                }           
            };
        
        a_controlPanel.addWindowListener(new java.awt.event.WindowAdapter() {
                public void windowClosing(java.awt.event.WindowEvent e) {
                    System.out.println("Close handler called");
                    player.stopPlaying();
                    
                }
            });
        
        a_controlPanel.setSliders(val,min,max,names);
        a_controlPanel.setButtonNames (new String[] {"Reset","Save","Load","(Un)mute","[a]","[o]","[u]","[i-]","[i]","[e]","[-]","Formants","LipModel (is on)", "update"});
        a_controlPanel.setVisible(true);
        a_controlPanel.onButton(nbuttons-1); // put up formants

        // End Vocal tract control panel:        

        // Nasal tract control panel:
        int nbuttonsNasal = 2;
        final int nAuxSlidersNasal = 3;
        int nSlidersNasal = nTubeSectionsNasal+nAuxSlidersNasal;
        String[] namesNasal = new String[nSlidersNasal];
        final double[] valNasal = new double[nSlidersNasal];
        double[] minNasal = new double[nSlidersNasal];
        double[] maxNasal = new double[nSlidersNasal];
        namesNasal[0] = "Velum(0noNasal)";
        valNasal[0] = 0; minNasal[0] = 0; maxNasal[0] = 1;
        namesNasal[1] = "M-N Bal";
        valNasal[1] = .5; minNasal[1] = 0; maxNasal[1] = 1;
        namesNasal[2] = "NasalLen";
        valNasal[2] = .11; minNasal[2] = .1; maxNasal[2] = .18;
        double minANasal = .01;
        double maxANasal = 10;
        double[] dangHondaFig6 = {.7,1.5,5,1,.8,.5,.6,.8}; // 8 sliders
        int ii=0;
        for(int k=nAuxSlidersNasal;k<nSlidersNasal;k++,ii++) {
            namesNasal[k] = "A("+new Integer(k-nAuxSlidersNasal).toString() + ") ";
            valNasal[k] = dangHondaFig6[ii];
            minNasal[k] = minANasal;
            maxNasal[k] = maxANasal;
            double r=Math.sqrt(valNasal[k]/Math.PI);
            tmNasal.setRadius(k-nAuxSlidersNasal,r/100); // in meters
        }
        
        a_controlPanelNasal = new Controller(new java.awt.Frame ("Nasal Tract"),
                                        false,valNasal.length,nbuttonsNasal) {
                private static final long serialVersionUID = 2L;

                boolean muted=false;
                
                public void onButton(int k) {
                    switch(k) {
                    case 0: {
                        FileDialog fd = new FileDialog(new Frame(),"Save");
                        fd.setMode(FileDialog.SAVE);
                        fd.setVisible(true);
                        saveToFile(fd.getFile());
                    }
                        break;
                    case 1: {
                        FileDialog fd = new FileDialog(new Frame(),"Load");
                        fd.setMode(FileDialog.LOAD);
                        fd.setVisible(true);
                        loadFromFile(fd.getFile());
                    }
                        break;
                        
                    }
                }
            
                public void onSlider(int k) {
                    switch(k) {
                    case 0:
                        filter.velumNasal = this.val[k];
                        filterCopy.velumNasal = this.val[k];
                        break;
                    case 1:
                        filter.mouthNoseBalance = this.val[k];
                        filterCopy.mouthNoseBalance = this.val[k];
                        break;
                    case 2:
                        tmNasal.setLength((double)this.val[k]);
                        filter.changeTubeModel();
                        break;
                    default:
                        double r=Math.sqrt(this.val[k]/Math.PI);
                        tmNasal.setRadius(k-nAuxSlidersNasal,r/100);// in meters
                        filter.changeTubeModel();
                        break;
                    }
                }
            
            };
        
        a_controlPanelNasal.setSliders(valNasal,minNasal,maxNasal,namesNasal);
        a_controlPanelNasal.setButtonNames (new String[] {"Save","Load"});
        a_controlPanelNasal.setVisible(true);

        // End Nasal tract control panel:

        // Rosenberg glottal source control panel:
        int nbuttonsRosenberg = 2;
        int nSlidersRosenberg = 4;
        namesRosenberg = new String[nSlidersRosenberg];
        valRosenberg = new double[nSlidersRosenberg];
        minRosenberg = new double[nSlidersRosenberg];
        maxRosenberg = new double[nSlidersRosenberg];

        namesRosenberg[0] = "freq";
        valRosenberg[0] = 100; minRosenberg[0] = 20; maxRosenberg[0] = 1000;
        namesRosenberg[1] = "openQ";
        valRosenberg[1] = .5; minRosenberg[1] = 0.001; maxRosenberg[1] = 1;
        namesRosenberg[2] = "slopeQ";
        valRosenberg[2] = 4; minRosenberg[2] = .15; maxRosenberg[2] = 10;
        namesRosenberg[3] = "gain";
        valRosenberg[3] = 0; minRosenberg[3] = 0; maxRosenberg[3] = 1;
        
        a_controlPanelRosenberg = new Controller(new java.awt.Frame ("Rosenberg Glottal Model"),
                                        false,valRosenberg.length,nbuttonsRosenberg) {
                private static final long serialVersionUID = 1L;
                
                public void onButton(int k) {
                    switch(k) {
                    case 0: {
                        FileDialog fd = new FileDialog(new Frame(),"Save");
                        fd.setMode(FileDialog.SAVE);
                        fd.setVisible(true);
                        saveToFile(fd.getFile());
                    }
                        break;
                    case 1: {
                        FileDialog fd = new FileDialog(new Frame(),"Load");
                        fd.setMode(FileDialog.LOAD);
                        fd.setVisible(true);
                        loadFromFile(fd.getFile());
                    }
                        break;
                    }
                }

                public void onSlider(int k) {
                    switch(k) {
                    case 0:
                        source.setFrequency((float)this.val[k]);
                        break;
                    case 1:
                        source.setOpenQuotient((float)this.val[k]);
                        break;
                    case 2:
                        source.setSpeedQuotient((float)this.val[k]);
                        break;
                    case 3:
                        source.setVolume((float)this.val[k]);
                        break;
                    default:
                        break;
                    }
                }
            };
        
        a_controlPanelRosenberg.setSliders(valRosenberg,minRosenberg,maxRosenberg,namesRosenberg);
        a_controlPanelRosenberg.setButtonNames (new String[] {"Save","Load"});
        a_controlPanelRosenberg.setVisible(true);
        // end Rosenberg panel

        //Ishizak-Flanagan twomass model panel
        nbuttonsTwoMass = 2;
        nSlidersTwoMass = 6;
        namesTwoMass = new String[nSlidersTwoMass];
        valTwoMass = new double[nSlidersTwoMass];
        minTwoMass = new double[nSlidersTwoMass];
        maxTwoMass = new double[nSlidersTwoMass];
        
        namesTwoMass[0] = "q(freq)";
        valTwoMass[0] = 1; minTwoMass[0] = .05; maxTwoMass[0] = 6;
        namesTwoMass[1] = "p-lung";
        valTwoMass[1] = 500; minTwoMass[1] = 0; maxTwoMass[1] = 6000;
        namesTwoMass[2] = "Ag0(cm^2)";
        valTwoMass[2] = -.005; minTwoMass[2] = -.5; maxTwoMass[2] = .5;
	namesTwoMass[3] = "noiseLevel";
        valTwoMass[3] = 1; minTwoMass[3] = 0; maxTwoMass[3] = 10;
        namesTwoMass[4] = "noiseFreq.";
        valTwoMass[4] = 500; minTwoMass[4] = 200; maxTwoMass[4] = 10000;
        namesTwoMass[5] = "noiseBW";
        valTwoMass[5] = 1000; minTwoMass[5] = 250; maxTwoMass[5] = 10000;
        

        
        a_controlPanelTwoMass = new Controller(new java.awt.Frame ("TwoMass Glottal Model"),
                                        false,valTwoMass.length,nbuttonsTwoMass) {
                private static final long serialVersionUID = 1L;

                public void onButton(int k) {
                    switch(k) {
                    case 0: {
                        FileDialog fd = new FileDialog(new Frame(),"Save");
                        fd.setMode(FileDialog.SAVE);
                        fd.setVisible(true);
                        saveToFile(fd.getFile());
                    }
                        break;
                    case 1: {
                        FileDialog fd = new FileDialog(new Frame(),"Load");
                        fd.setMode(FileDialog.LOAD);
                        fd.setVisible(true);
                        loadFromFile(fd.getFile());
                    }
                        break;
                    }
                }

                public void onSlider(int k) {
                    switch(k) {
                    case 0:
                        // q factor of two mass model
                        twoMassSource.getVars().q = this.val[k];
                        //twoMassSource.getVars().setVars(); 
                        break;
                    case 1:
                        // lung pressure
                        twoMassSource.getVars().ps = this.val[k];
                        //twoMassSource.getVars().setVars(); 
                        break;
                    case 2:
                        // glottal rest area (displayed in cm^2)
                        twoMassSource.getVars().Ag0 = 1.e-4 * this.val[k];
                        //twoMassSource.getVars().setVars(); 
                        break;
                    case 3:
                        twoMassSource.setFlowNoiseLevel(this.val[k]);
                        break;
                    case 4:
                        twoMassSource.setFlowNoiseFrequency(this.val[k]);
                        break;
                    case 5:
                        twoMassSource.setFlowNoiseBandwidth(this.val[k]);
                        break;
                    default:
                        break;
                    }
                }
            };
        
        a_controlPanelTwoMass.setSliders(valTwoMass,minTwoMass,maxTwoMass,namesTwoMass);
        a_controlPanelTwoMass.setButtonNames (new String[] {"Save","Load"});
        a_controlPanelTwoMass.setVisible(true);
        // end Ishizak-Flanagan twomass model panel

        // add airflow monitor
        final JProgressBar progressBar = new JProgressBar(0, 1000);
        progressBar.setValue(0);
        progressBar.setStringPainted(true);
        a_controlPanelTwoMass.add(progressBar);
        a_controlPanelTwoMass.getContentPane ().setLayout (new java.awt.GridLayout (nSlidersTwoMass+(nbuttonsTwoMass+2)/2, 2));
        a_controlPanelTwoMass.pack();
        
                
        // set locations of panels on screen
        a_controlPanel.setLocation(new Point(740,10));
        Point p = a_controlPanel.getLocation();
        p.translate(430,0);
        a_controlPanelNasal.setLocation(p);
        p.translate(0,300);
        a_controlPanelRosenberg.setLocation(p);
        p.translate(0,150);
        a_controlPanelTwoMass.setLocation(p);
        p.translate(0, 240);
        textToSpeech.setLocation(p);
        textToSpeech.setSize(a_controlPanelNasal.getWidth(), 350);
        //textToSpeech.setSize(a_controlPanelNasal.getWidth(), textToSpeech.getHeight());
//        if (!renderAudioToFile)
//        {
//           player.start();
//        }
	/*
	boolean foo = true;
	while(foo) {
	    try {
		Thread.sleep(100);
	    } catch(Exception e) {}
	}
	*/
        filter.reset();
        twoMassSource.reset();
        player.resetAGC();

        class UGAverager {
            int n = 0; // # elements in it
            int ip=0; // oldest element
            int nmax =0; // max allowed
            double [] vals; // values
            
            UGAverager(int n) {
                this.nmax=n;
                vals = new double[n];
            }

            double take(double u) {
                vals[ip] = u;
                ip++;
                if(ip==nmax) {
                    ip = 0;
                }
                if(n!=nmax) {
                    n++;
                }
                double av = 0;
                for(int i=0;i<n;i++) {
                    av += vals[i];
                }
                av /= n;
                return av;
            }
        }
        
        int sleepms = 1;
        double maxug = 0.00000000001;
        
        UGAverager ugAverager = new UGAverager(100);
        while(!haltMe) {
            try {
                Thread.sleep(sleepms);
                double ug = twoMassSource.getUg();
                if(ug<0) {
                    //System.out.println(ug);
                }
                ug = ugAverager.take(ug);
                if(ug>maxug) {
                    maxug=ug;
                }
                int pval = (int)(1000*ug/maxug);
                progressBar.setValue(pval);
                progressBar.setString("ug="+String.valueOf(ug));
            } catch(Exception e) {}
            airway.scale(tm);
        }
        
    }

    double[] fantData_a =  new double[] {5, 5, 5, 5, 6.5,       8, 8, 8, 8, 8,
                                                8, 8, 8, 6.5, 5,       4, 3.2, 1.6, 2.6, 2.6,
                                                2, 1.6, 1.3, 1, .65,   .65, .65, 1, 1.6, 2.6,
                                                4, 1, 1.3, 1.6, 2.6};
    double[] fantData_o =  new double[] {3.2,3.2,3.2,3.2,6.5,   13,13,16,13,10.5,
                                                10.5,8,8,6.5,6.5,       5,5,4,3.2,2,
                                                1.6,2.6,1.3,.65,.65,    1,1,1.3,1.6,2,
                                                3.2,4,5,5,1.3,          1.3,1.6,2.6};
    double[] fantData_u =  new double[] {.65,.65,.32,.32,2,  5,10.5,13,13,13,
                                                13,10.5,8,6.5,5,    3.2,2.6,2,2,2,
                                                1.6,1.3,2,1.6,1,     1,1,1.3,1.6,3.2,
                                                5,8,8,10.5,10.5,    10.5,2,2,2.6,      2.6};
    double[] fantData_i_ =  new double[] {6.5,6.5,2,6.5,8,   8,8,5,3.2,2.6,
                                                 2,2,1.6,1.3,1,     1,1.3,1.6,2.6,2,
                                                 4,5,6.5,6.5,8,     10.5,10.5,10.5,10.5,10.5,
                                                 13,13,10.5,10.5,6, 3.2,3.2,3.2,3.2};
    double[] fantData_i =  new double[] {4,4,3.2,1.6,1.3,              1,.65,.65,.65,.65,
                                                .65,.65,.65,1.3,2.6,          4,6.5,8,8,10.5,
                                                10.5,10.5,10.5,10.5,10.5,     10.5,10.5,10.5,8,8,
                                                2,2,2.6,3.2};
    double[] fantData_e =  new double[] {8,8,5,5,4,               2.6,2,2.6,2.6,3.2,
                                                4,4,4,5,5,               6.5,8,6.5,8,10.5,
                                                10.5,10.5,10.5,10.5,8,   8,6.5,6.5,6.5,6.5,
                                                1.3,1.6,2,2.6};
    
    double[] fantData__ =  new double[] {5, 5, 5, 5, 5,       5, 5, 5, 5, 5,
                                                5, 5, 5, 5, 5,       5, 5, 5, 5, 5,
                                                5, 5, 5, 5, 5,   5, 5, 5, 5, 5,
                                                5, 5, 5, 5, 5};

    public void preset(String p) {
        double[] f_a=null;
        if (p=="a") {
            f_a = fantData_a;
        }
		if (p=="o") {
            f_a = fantData_o;
        }
        if (p=="u") {
            f_a = fantData_u;
        }
        if (p=="i_") {
            f_a = fantData_i_;
        }
        if (p=="i") {
            f_a = fantData_i;
        }
        if (p=="e") {
            f_a = fantData_e;
        }
        if (p=="-") {
            f_a = fantData__;
        }
        // interpolate and invert Fant data
        double C = (f_a.length-1.)/(tract.length-1);
        for(int i=0;i<tract.length;i++) {
            double k = i*C;
            int ki = (int)k;
            double kfrac = k-ki;
            int i1 = ki;
            int i2 = i1+1;
            if(i2>f_a.length-1) {
                i2 = i1;
            }
            // radii in meters
            tract[tract.length-i-1] = Math.sqrt((f_a[i1]*(1-kfrac)+f_a[i2]*kfrac)/Math.PI)/100;
        }
    }
    
//  this class exists only to hold the input probes
    class ProbeHolder extends ModelBase {
        
       public StepAdjustment advance (double t0, double t1, int flags) {
           
           if(renderAudioToFile) {
              try {
        	 //System.out.println("advancing player from t0="+TimeBase.ticksToSeconds(t0)+" to t1="+TimeBase.ticksToSeconds(t1));
        	 // for either the rendering of text or normalize options, we render using ASCII to file
        	 if (renderText || normalize) 
        	 {
        	    player.advanceTime (t1, true);
        	 }
        	 else
        	 {
        	    player.advanceTime (t1);
        	 }
              } catch(Exception e) {
              }
          }
           return null;
        }
        
//       public void setDefaultInputs(double t0, double t1) {}
        
        public String getName() {
            return "VTNTDemo probeholder helper";
        }
    }
}


