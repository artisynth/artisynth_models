package artisynth.models.tubesounds;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Hashtable;


import artisynth.core.util.ArtisynthPath;
import artisynth.models.tubesounds.TubeData.PhonemeSpec;




public class VoiceProbeMaker
{

   private Hashtable<String, String> dictionary;
   
   private static final String retChar = "\r\n";
   	
   private String[] input;
   private String inputWord;
   private String fileName;
   
   //list of output files to be written
   private static BufferedWriter[] outFiles;
   
   //default length of each segment
   static final int segLength = 250;   
   // interval between each segment
   static final int segInt = 100; 
   // interval between each word
   static final int wordPause = 500; 
   
   //list of file types (also used as the file suffix for each probe file)
   static final String fileTypes[] = {"lung", "pitch", "glot", "tube"};
   
   //default header for each probe
   static final String probe_header = "1000000000 20000000000 1.0 \r\nLinear 1 explicit" + retChar;
	
   //constructor for commandline testing
   public VoiceProbeMaker(String[] args)
   {
      input = args;
   }
   
   //constructor for 'normal' usage
   public VoiceProbeMaker(String word, String filename)
   {
      inputWord = word;
      fileName = filename;
   }
   
   //test routine
   public void testRun()
   {
      loadDictionary();
      TubeData.initTable();
      PhonemeSpec pair = TubeData.phonemeToIndex.get("HH");
      System.out.println(pair.A);
      String fullBasePath = ArtisynthPath.getSrcRelativePath(
	       this, input[0]);
      setupOutputFiles(fullBasePath);
      writeFileHeaders();
      try
      {
         String inputLine = "";
         InputStreamReader converter = new InputStreamReader(System.in);
         BufferedReader reader = new BufferedReader(converter);

         System.out.println("Enter word:");
         inputLine = reader.readLine();
         VoiceSegment[] segs = parseWord(inputLine);
         writeSegments(segs);
      }
      catch (Exception e)
      {
	 System.out.println("exception");
	 System.out.println(e.getMessage());
      }		
      closeOutputFiles();
   }
   
   /**
    * main method for creating an input probe from input text
    * @return true if successful
    */
   public boolean makeProbe()
   {
      loadDictionary();
      TubeData.initTable();
      PhonemeSpec pair = TubeData.phonemeToIndex.get("HH");
      System.out.println(pair.A);
      setupOutputFiles(fileName);
      writeFileHeaders();
      try
      {
	 String[] words = inputWord.split(" ");
	 System.out.println("parsed "+words.length+" word(s)");
	 ArrayList<VoiceSegment> allSegs = new ArrayList<VoiceSegment>();
	 Collections.addAll(allSegs, parseWord(words[0])); //add first word
	 for (int i=1; i<words.length; i++)
	 {
	    VoiceSegment prev = allSegs.get(allSegs.size()-1);
	    
	    long offset =  prev.getStartTimeMS() + prev.getDurationMS() + wordPause;
	    VoiceSegment[] segs = parseWord(words[i], offset);
	    Collections.addAll(allSegs, segs);
	 }
	 System.out.println(allSegs.size() + " segments in total");
         writeSegments(allSegs.toArray(new VoiceSegment[0]));
      }
      catch (Exception e)
      {
	 System.out.println("exception");
	 System.out.println(e.getMessage());
	 return false;
      }		
      closeOutputFiles();
      return true;
   }
   
   /**
    * sets up the input probe data files according to a base name; appends file type suffix for each
    * input probe (e.g. _lung, _pitch, etc)
    * clears existing file, if present
    * @param base_name the base filename for the probe files
    */
   private void setupOutputFiles(String base_name)
   {
//      String fullBasePath = ArtisynthPath.getHomeRelativePath(
//                    "src/artisynth/models/tubesounds/"+base_name,".");
      outFiles = new BufferedWriter[4];
      for (int i=0; i<fileTypes.length; i++)
      {
	 String fileName = base_name+ "_"+fileTypes[i]+".txt";
   	 try
   	 {
   	    BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
            System.out.println("opening file: "+ fileName);
            outFiles[i] = writer;
            writer.flush();
   	 }
   	 catch (IOException e)
   	 {
   	    System.out.println(e.getMessage());
   	 }
      }
   }
   
   /**
    * close all files that are open for writing during the creation of probes
    *
    */
   private void closeOutputFiles()
   {
      for (int i=0; i<outFiles.length; i++)
      {
	 try
	 {
	    outFiles[i].close();
	 }
	 catch (IOException e)
	 {
	    System.out.println(e.getMessage());
	 }
      }
   }
   /**
    * inserts the default headers to each input probe file
    *
    */
   private void writeFileHeaders()
   {
      for (int i=0; i<outFiles.length; i++)
      {
	 try
	 {
	    outFiles[i].write(probe_header);
	 }
	 catch (IOException e)
	 {
	    System.out.println(e.getMessage());		
	 }
      }
   }
   
   /**
    * traverses through list of all segments created for the given text input, and writes them
    * to the input probe files that are currently being created.
    * @param segments
    */
   private void writeSegments(VoiceSegment[] segments)
   {
      System.out.println("writing " + segments.length+" segments...");
      if (segments == null)
      {
         System.out.println("segments are null!");
      }
      for (int i=0; i< segments.length; i++)
      {
         VoiceSegment curr = segments[i];
         if (curr == null)
         {
            System.out.println(i+" is null!");
         }
         try
         {
            VoiceSegment.ProbePoint pt;
            double start = (double)curr.getStartTimeMS()/1000;
            //writing segments for LUNG values
            System.out.println("lung");
            for (int j=0; j < curr.getLungValues().length; j++)
            {
               pt = curr.getLungValues()[j];
               outFiles[0].append((start+pt.getTimeS())+" "+(pt.getValue()+curr.getStressLungMod())+retChar);
            }
            //writing segments for PITCH values
            System.out.println("pitch");
            for (int j=0; j < curr.getPitchValues().length; j++)
            {
               pt = curr.getPitchValues()[j];
               outFiles[1].append((start+pt.getTimeS())+" "+(pt.getValue()+curr.getStressPitchMod())+retChar);
            }
            //writing segments for GLOTTAL values
            System.out.println("glot");
            for (int j=0; j < curr.getGlotValues().length; j++)
            {
               pt = curr.getGlotValues()[j];
               outFiles[2].append((start+pt.getTimeS())+" "+pt.getValue()+retChar);
            }
            //writing TUBE values.
            // The tube values are stored as an index, so we need to call a method
            // to return all the values for a tube as an array, and write them individually
            // to the probe file.
            for (int j=0; j < curr.getTubeIndicies().length; j++)
            {
               String tmpStr = "";
               pt = curr.getTubeIndicies()[j];
               double[] vals = TubeData.getTubeValsFromIndex((int)pt.getValue());
               for (int k=0; k< vals.length; k++)
               {
        	  tmpStr += vals[k]+" ";
               }
               outFiles[3].append((start+pt.getTimeS())+" "+tmpStr+retChar);
            }
         }
         catch (IOException e)
         {
            System.err.println(e.getMessage());
         }
         catch (NullPointerException e)
         {
            System.err.println(e.getMessage());
         }
      }
   }
   /**
    * loads the CMU pronouncing dictionary as a hash table
    *
    */
   private void loadDictionary()
   {
      dictionary = new Hashtable<String, String>();
      String fullFilePath = ArtisynthPath.getSrcRelativePath(
               this, "c0.6_.txt");
      try
      {
	 File file = new File(fullFilePath);
	 BufferedReader in = new BufferedReader(new FileReader(file));
	 String line = null;
	 while ((line = in.readLine()) != null)
	 {
	    if (line.indexOf("  ") != -1)
	    {
	       String word = line.substring(0, line.indexOf("  "));
	       String phonemes = line.substring(line.indexOf("  ")+2);
               //System.out.println("word is: "+word);
               //System.out.println("   phonemes:"+phonemes);
               dictionary.put(word, phonemes);
	    }
	 }
	 in.close();
      }
      catch (IOException e)
      {
	 System.err.println(e.getMessage());
      }
   }
   
   private VoiceSegment[] parseWord(String word)
   {
      return parseWord(word, 0);
   }
   /**
    * method for parsing a single word string
    * @param word the word to be parsed
    * @param offset the absolute start time of this word
    * @return
    */
   private VoiceSegment[] parseWord(String word , long offset)
   {
      String phonemesString = dictionary.get(word.toUpperCase());
      System.out.println("word: "+word+ " phonemes: "+phonemesString);
      if (phonemesString != null)
      {
	 String[] phonemes = phonemesString.split(" ");
	 VoiceSegment[] segs = new VoiceSegment[phonemes.length]; //add one more to the end for the 'dummy' segment
	 //System.out.println("START PHONE LIST, "+phonemes.length+" phonemes");
	 for (int i=0; i<phonemes.length; i++)
	 {
	    long start = i*(segLength+segInt) + offset; 
	    //long end = (2*i+1)*segLength + offset;
	    VoiceSegment currentSeg = phonemeToSegment(phonemes[i], start, segLength);
	    //System.out.println("start time = "+start);
	    currentSeg.setStart(start);
	    //currentSeg.setEnd(end);
	    segs[i] = currentSeg;
	 }
	 //System.out.println("END PHONE LIST. parse word successful");
	 segs[segs.length-1].addTail();
	 segs[0].addHead();
	 return segs;
      }
      else
      {
	 return null;
      }
   }
   /**
    * coverts a given phoneme into a VoiceSegment, which contains all the necessary data
    * to create a segment of an input probe for that phoneme.
    * @param phoneme the phoneme to be converted
    * @param start start time of phoneme (relative to beginning of word)
    * @param duration length of phoneme
    * @return
    */
   private VoiceSegment phonemeToSegment(String phoneme, long start, long duration)
   {
      //it is possible for the phoneme to have stress value at the end, represented by a single numerical digit
      VoiceSegment seg = new VoiceSegment(start, duration);
      String phone;
      int stress = -1;
      
      if (phoneme.matches("\\D*\\d")) // one or more letters, followed by number
      {
	 phone = phoneme.substring(0, phoneme.length()-1);
	 stress = Integer.parseInt(phoneme.substring(phoneme.length()-1, phoneme.length()));
      }
      else if (phoneme.matches("\\D\\D") || phoneme.matches("\\D")) //either one or two letters
      {
	 phone = phoneme;
      }
      else
      {
	 System.out.println("Error: invalid phoneme!");
	 return null;
      }
      System.out.println("phone: "+phone+" stress: "+stress);
      try
      {
	 int tubeA, tubeB;
         tubeA = TubeData.phonemeToIndex.get(phone).A;
         tubeB = TubeData.phonemeToIndex.get(phone).B;
         System.out.println("making seg. for phoneme "+phoneme+"; shapes ="+tubeA+":"+tubeB);
         seg.setTubeA(tubeA);
         seg.setTubeB(tubeB);
         if (TubeData.phonemeToIndex.get(phone).isVoiced)
         {
            seg.setGlot(0.01);
         }
         else
         {
            seg.setGlot(0.4);
         }
         if ((phone.equals("B") || (phone.equals("P"))))
         {
            seg.setTubeMid(tubeA);
         }
         seg.setLung(900);
      }
      catch (Exception e)
      {
	 System.out.println("Error: invalid phoneme!"+e.getMessage());
	 return null;
      }
      seg.setStress(stress);
      
      System.out.println("done phoneme");
      return seg;
   }
   

   /**
    * for commandline testing
    */
   public static void main(String[] args) 
   {
      if (args.length != 1)
      {
	 System.out.println("usage: VoiceProbe [file_name]");
	 return;
      }
      else
      {
	 VoiceProbeMaker v = new VoiceProbeMaker(args);
	 v.testRun();
      }
   }
   /**
    * VoiceSegment class. Contains enough information to create input probe segments that represents
    * a single phoneme.
    *
    */
   public class VoiceSegment
   {
      /**
       * ProbePoint class. contains a time:value pair that stores single points within a probe.
       * For tube shapes, it simply stores a single value that refers to an index position that can
       * be used to retreive the entire array that represents the input probe values to drive the tube shape
       *
       */
      private class ProbePoint
      {
	 private long timeValue;
	 private double probeValue;
	 public ProbePoint(long t, double v)
	 {
	    timeValue = t;
	    probeValue = v;
	 }
	 public void setValue(double value) {probeValue = value;}
	 public void setTime(long value) {timeValue= value;}
	 public double getTime() {return timeValue;}
	 public double getTimeS() { return (double)timeValue/1000;}
	 public double getValue() {return probeValue;}
      }

      //stress modifier for lung pressure
      private final float stress_base_lung = 200;
      private final float stress_base_pitch = 0.2f;

      private int stress_index;
      private long start_time;
      private long length;
      
      
      // lists of values for all the input probes used to represent a phoneme
      ArrayList<ProbePoint> lungValues;
      ArrayList<ProbePoint> pitchValues;
      ArrayList<ProbePoint> glotValues;
      ArrayList<ProbePoint> tubeValues;
      

      /**
       * default constructor. expects values to be edited later.
       */
      public VoiceSegment(long start, long duration)
      {
	 start_time = start;
	 length = duration;
	 
	 lungValues = new ArrayList<ProbePoint>();
         lungValues.add(new ProbePoint(0, 0));
         lungValues.add(new ProbePoint(length, 0));
         
         pitchValues = new ArrayList<ProbePoint>();
         pitchValues.add(new ProbePoint(0, 1));
         pitchValues.add(new ProbePoint(length, 1));
         
         glotValues = new ArrayList<ProbePoint>();
         glotValues.add(new ProbePoint(0, 0));
         glotValues.add(new ProbePoint(length, 0));
         
         tubeValues = new ArrayList<ProbePoint>();
         tubeValues.add(new ProbePoint(0, 0));
         tubeValues.add(new ProbePoint(length, 0));
         System.out.println("Created Voice Seg with start = "+start+ " len=" + duration);
	 
      }
//      //not used
//      public VoiceSegment(double lungP, double pitch, double glot, int shapeA, int shapeB, long start, long stop)
//      {
//	 this(lungP, pitch, glot, shapeA, shapeB, start, stop, 0);
//      }
//      //not used
//      public VoiceSegment(double lungP, double pitch, double glot, int shape, long start, long stop)
//      {
//	 this(lungP, pitch, glot, shape, shape, start, stop, 0);
//      }
//      //not used
//      public VoiceSegment()
//      {
//	 this(0, 1, 0, 0, 0, 0, 0);
//      }
//    public VoiceSegment(double lungP, double pitch, double glot, int shapeA, int shapeB, long start, long duration, int stress)
//    {
//       this(start, duration);
//       setLung(lungP);
//       setPitch(pitch);
//       setGlot(glot);
//       setTubeA(shapeA);
//       setTubeB(shapeB);
//       stress_index = stress;
//    }
      
      public long getStartTimeMS() { return start_time; }
      public double getStartTimeS() { return (double) start_time / 1000; }
      
      public long getDurationMS() { return length; }
      public double getDurationS() { return (double) length / 1000; }
      
      public ProbePoint[] getLungValues()
      {
	 return lungValues.toArray(new ProbePoint[0]);
      }
      public ProbePoint[] getPitchValues()
      {
	 return pitchValues.toArray(new ProbePoint[0]);
      }
      public ProbePoint[] getGlotValues()
      {
	 return glotValues.toArray(new ProbePoint[0]);
      }
      public ProbePoint[] getTubeIndicies()
      {
	 return tubeValues.toArray(new ProbePoint[0]);
      }
      
      public void addTail()
      {
	 lungValues.add(new ProbePoint(length+10, 0));
      }
      public void addHead()
      {
	 lungValues.add(0, new ProbePoint(-1, 0));
      }
      private int getStressFactor()
      {  //note: stress factor is the amount of stress to be applied
	 //      stress level is the 'type' of stress as defined by the dictionary
	 //        (0 = no stress; 1 = primary; 2 = secondary)
	 switch (stress_index)
	 {
	 case 0:
	    return 0;
	 case 1:
	    return 2;
	 case 2:
	    return 1;
	 }
	 return 0;
      }
      
      public float getStressPitchMod()
      {
	 return stress_base_pitch*getStressFactor();
      }
      
      public float getStressLungMod()
      {
	 return stress_base_lung*getStressFactor();
      }
      public void setLung(double value) 
      {
	 setAllValues(lungValues, value);
      }
      public void setPitch(double value) 
      {
	 setAllValues(pitchValues, value);
      }
      public void setGlot(double value) 
      {
	 setAllValues(glotValues, value);
      }
      public void setTubeA(int A) 
      {
	 tubeValues.get(0).setValue(A);
      }
      public void setTubeMid(int T)
      {  //assumes only 3 sections max, and this is only called once ever for a given segment
	 System.out.println("adding a mid tube section");
	 tubeValues.add(1,new ProbePoint(length/2, T));
      }
      public void setTubeB(int B) 
      { 
	 tubeValues.get(tubeValues.size()-1).setValue(B);
	 tubeValues.get(tubeValues.size()-1).setTime(segLength);
      }
      public void setStress(int level) {stress_index = level;}
      
      public void setStart(long start) {start_time = start;}
      
      private void setAllValues(ArrayList<ProbePoint> list, double value)
      {
	 for (ProbePoint pt : list)
	 {
	    pt.setValue(value);
	 }
      }  
   }
}