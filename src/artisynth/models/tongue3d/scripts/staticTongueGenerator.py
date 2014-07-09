### USER DEFINED SETTINGS VVVVVV

# shouldn't modify to order to be in-sync with toMatrix.py script
tonguePath = "models/0/models/0/"
tongueExciterListPath = tonguePath + "exciters/"
tongueNodeListPath = tonguePath + "nodes/"

excitIncrement = 0.1             # how much should the excitation levels increment per iteration? No trailing zeros
excitIncrementDeciPlaces = 1     # how many decimal places does excitIncrement have? e.g. 0.2 -> 1 deciPlaces

excitMin = 0        # minimum excitation value
excitMax = 1        # maximum excitation value

# shouldn't modify in order to be in-sync with controller. Note that controller only uses waypoints
asciiOutputStartDir = "asciiOutput/"
wpOutputStartDir = "wpOutput/"

# have the script also generate an ascii version of the simulation results? That is, record
# all the tongue node positions in Point3D.toString() format in a text file per simulation?
makeAsciiAlso = False

# Use this to enforce the maximum duration (seconds) of each simulation. Some simulations
# will go on too long before being settled, so this variable was implemented.
lengthPerSimulation = 2

# used to compare two floats. Shouldn't have to modify as long as value is more precise
# than exciteMin/Max/Increment
ELIPSON = 0.0001

### USER DEFINED SETTINGS ^^^^^^^




### WHAT DOES THIS SCRIPT DO?
"""
It will iterate through every different muscle excitation combination, and for each
combination, it will run a simulation and record the resulting tongue position in a 
waypoint file (and ascii file)

For example, if the tongue only had 3 muscle exciters with values 0.545, 0.266, 2.455,
the location of the generated waypoint file will be:  wpOutput/0.545/0.266/2.455_wp.txt
If enabled, the location of the generated ascii file would similarly be:
  asciiOutput/0.545/0.266/2.455_ascii.txt.


Each ascii .txt file will list positions of all the tongue nodes that can be indexed, beginning
with node#1 on the first line

This script is intended to be use with the StaticTongueGenerator model. See tongue3d/TongueViewerController.java
for detailed instructions on this script.
"""






import java.io.File

### FUNCTION DEFINITIONS VVVVVVV

"""
Save excitIncrement and excitIncrementDeciPlaces values to <outputPath>/inc.txt.
This is required for the controller during playback later.

@param:
    increment :: double - value between excitation configurations
    incrementDeciPlaces :: integer - how many decimal places does increment have?
    outputPath :: string - main directory path of waypoint folder
"""
def recordIncrementValue(increment, incrementDeciPlaces, outputPath):
    outputFolder = File(outputPath)    
    if not outputFolder.exists():
        outputFolder.mkdir()        # ensure folder for .txt file exists
    incFile = open(outputPath + "inc.txt", "w")
    incFile.write(str(increment))
    incFile.write("\n")
    incFile.write(str(incrementDeciPlaces))
    incFile.close()





"""
This will write data values that's required toMatrix.py script.
The data will be written to <outputPath>/toMatrixVariables.txt.

@param:
    min :: double - the minimum value of the excitation values
    inc :: double - the increment defined as excitation values increase
    max :: double - the maximum value of the excitation values
    exciterList :: list - references to the exciters
    outputPath :: string - main directory path of waypoint folder
"""
def recordDataReqForToMatrixScript(min, inc, max, exciterList, outputPath):
    outputFolder = File(outputPath)    
    if not outputFolder.exists():
        outputFolder.mkdir()        # ensure folder for .txt file exists
    varFile = open(outputPath + "toMatrixVariables.txt", "w")
    varFile.write(str(min) + "\n")
    varFile.write(str(inc) + "\n")
    varFile.write(str(max) + "\n")
    varFile.write( str(exciterList.size()) )
    varFile.close()






"""
Has the simulation time reached the imposed simulation length limit?
@param:
    ELIPSON :: global double - small value used to compare 2 floats (e.g. 0.0001)
@return: true if so, false otherwise
"""
def hasReachedMaxDuration():
    if Main.getTime() > lengthPerSimulation - ELIPSON:
        return True
    else:
        return False





"""
Function that behaves similar to a C++ style "for" loop involving <=
@param:
    start :: double - minimum number
    end :: double - maximum number
    step :: double - how much do you want to increment at every iteration
    
    ELIPSON :: global double - small value used to compare 2 floats (e.g. 0.0001)
@return:
    yields all the intervals from start to end (inclusively), incremented by "step"
"""
def CLessEqualLoop(start, end, step):
    while start <= end + ELIPSON:
        yield start
        start += step





"""
Set all excitation levels to zero.
@param:
    exciterList :: list - holds references to all exciters
@return:
    None
"""
def setAllExcitationToZero(exciterList):
    for exciterIdx in range(0, exciterList.size()):            # for each exciter
        exciterList[exciterIdx].setExcitation(0)




"""
Does a waypoint (and ascii file) file already exist for the current excitation configuration?
@param:
    makeAsciiAlso :: boolean - generate ascii version also?
    asciiOutputStartDir :: string - starting directory location of where ascii files should be stored
    wpOutputStartDir :: string - starting directory location of where waypoint files should be stored
    excitePath :: string - relative directory that the waypoint/ascii file would be located in
                  e.g. 0.0/0.0/0.0/0.2/
    lastExciteValue :: double - excitation value of the last exciter
@return:
    if makeAsciiAlso == false:
        true if waypoint already exists for current excitation configuration
        false if waypoint doesn't exist
    if makeAsciiAlso == true:
        true if both waypoint and ascii files exist for current excitation configuration
        false if otherwise
"""
def doSimResultsAlreadyExist(makeAsciiAlso, asciiOutputStartDir, wpOutputStartDir, excitePath, lastExciteValue):
    wpPath = wpOutputStartDir + excitePath + str(lastExciteValue) + "_wp.txt"
    wpFile = File(wpPath)
    if not makeAsciiAlso:
        if wpFile.exists():
            System.out.println("\nScript :: simulation results for current excitation configuration already exists")
            System.out.println("Script :: current configuration path:  " + wpPath)
            System.out.println("Script :: skipping to next configuration\n")
            return True
        else:
            return False    
    else:
        asciiPath = asciiOutputStartDir + excitePath + str(lastExciteValue) + "_ascii.txt"
        asciiFile = File(asciiPath)
        if wpFile.exists() and asciiFile.exists():
            System.out.println("\nScript :: simulation results for current excitation configuration already exists")
            System.out.println("Script :: current configuration path:  " + wpPath)
            System.out.println("Script :: skipping to next configuration\n")
            return True
        elif wpFile.exists() and not asciiFile.exists():
            System.out.println("\nScript :: ascii results don't exist. Running simulation")
            System.out.println("Script :: current configuration path:  " + wpPath + "\n")
            return False
        elif not wpFile.exists() and asciiFile.exists():
            System.out.println("\nScript :: waypoint results don't exist. Running simulation")
            System.out.println("Script :: current configuration path:  " + wpPath + "\n")
            return False
        else:
            System.out.println("\nScript :: ascii and waypoint results don't exist. Running simulation")
            System.out.println("Script :: current configuration path:  " + wpPath + "\n")
            return False





"""
Record tongue node positions as a waypoint (and as ascii file).
@param:
    tongueNodeList :: list - holds references to all tongue nodes
    asciiOutputStartDir :: string - starting directory location of where ascii files should be stored
    wpOutputStartDir :: string - starting directory location of where waypoint files should be stored
    excitePath :: string - relative directory that the waypoint/ascii file would be located in
                  e.g. 0.0/0.0/0.0/0.2/
    lastExciteValue :: double - the excitation value of the final exciter
    makeAsciiAlso :: boolean - generate ascii version also?
@return: none
"""
def recordResults(tongueNodeList, asciiOutputStartDir, wpOutputStartDir, excitePath, lastExciteValue, makeAsciiAlso):
    Main.getMain().clearWayPoints();                # clear previous waypoint. Only 1 waypoint per waypoint file
    Main.getMain().addWayPoint(Main.getTime())
    Main.getRootModel().getWayPoints().saveas(wpOutputStartDir + excitePath + str(lastExciteValue) + "_wp.txt")
    if makeAsciiAlso:
        asciiFile = open(asciiOutputStartDir + excitePath + str(lastExciteValue) + "_ascii" + ".txt", "w")    # create a new file to store positions of tongue
        for nodeIdx in range(0, tongueNodeList.size()):                    # for each node that define tongue shape
            nodePosition = str( tongueNodeList[nodeIdx].getPosition().toString("%g") ) + "\n"
            asciiFile.write( nodePosition )
        asciiFile.close()






"""
Recursive method to iterate through every possible muscle excitation configuration.
For every configuration, run simulator and record results.
@param
    exciterList :: list - references to exciters
    tongueNodeList :: list - references to nodes that define tongue shape
    min :: double - minimum excitation level
    cur :: double - the current exciter idx that's being iterated through. First call should pass 0
    increment :: double - how much do you want to increment the excitation level per iteration?
    max :: double - maximum excitation level
    excitePath :: string - relative directory path so far using excitation values only. (e.g. 0.0/0.1/...). First call should pass empty string ""
    asciiOutputStartDir :: string - starting directory location of where ascii files should be stored
    wpOutputStartDir :: string - starting directory location of where waypoint files should be stored
    makeAsciiAlso :: boolean - generate ascii version also?
    
    lengthPerSimulation :: global double - maximum length per simulation (seconds)
@precond
    Call "setAllExcitationToZero(exciterList)" before executing this function
@return:
    none
"""
def forAllMuscleExcitmentConfig(exciterList, tongueNodeList, min, cur, increment, max, excitePath, asciiOutputStartDir, wpOutputStartDir, makeAsciiAlso):
    if cur == exciterList.size() - 1:
        for i in CLessEqualLoop(min, max, increment):
            exciterList[cur].setExcitation(i)
            lastExciteValueBeforeRun = exciterList[cur].getExcitation()
            if not doSimResultsAlreadyExist(makeAsciiAlso, asciiOutputStartDir, wpOutputStartDir, excitePath, lastExciteValueBeforeRun):
                origExciterLevels = []
                for excitIdx in range(0, exciterList.size()):        # save excitation levels before run() & reset()
                    origExciterLevels.append( exciterList[excitIdx].getExcitation() )
                run(lengthPerSimulation)        
                waitForStop()
                if Main.getRootModel().getMonitors().get("RunUntilSettled").isSettled() or hasReachedMaxDuration():
                    System.out.println("\nScript :: recording results now because tongue has settled or reached max duration.\n")
                    recordResults(tongueNodeList, asciiOutputStartDir, wpOutputStartDir, excitePath, lastExciteValueBeforeRun, makeAsciiAlso)
                else:
                    System.out.println("\nScript :: exception detected. Will not record results.\n")
                reset()
                for excitIdx in range(0, exciterList.size()):        # restore excitation levels after reset()
                    exciterList[excitIdx].setExcitation( origExciterLevels[excitIdx] )
    else:
        for i in CLessEqualLoop(min, max, increment):            # for THIS excitation slider, iterate through
            exciterList[cur].setExcitation(i)
            if makeAsciiAlso:
                asciiNextFolder = File(asciiOutputStartDir + excitePath + str(exciterList[cur].getExcitation()))        
                if not asciiNextFolder.exists():
                    asciiNextFolder.mkdir()
            forAllMuscleExcitmentConfig(exciterList, tongueNodeList, min, cur+1, increment, max, excitePath + str(exciterList[cur].getExcitation()) + "/", asciiOutputStartDir, wpOutputStartDir, makeAsciiAlso)    #recursively handle others

### FUNCTION DEFINITIONS ^^^^^^^^




### MAIN CODE BODY VVVVVV

reset()
Main.getRootModel().getWayPoints().clear()
recordIncrementValue(excitIncrement, excitIncrementDeciPlaces, wpOutputStartDir)

# this list below will hold reference to all the tongue nodes
tongueNodeList = find( tongueNodeListPath )

# this list below will hold reference to all the tongue muscles (exciters) 
exciterList = find( tongueExciterListPath )

recordDataReqForToMatrixScript(excitMin, excitIncrement, excitMax, exciterList, wpOutputStartDir)

# ensure all excitations are 0 before calling forAllMuscleExcitmentConfig()
setAllExcitationToZero(exciterList)

# must manually create starting folder for ascii files
if makeAsciiAlso:
    asciiOutputRootFolder = File(asciiOutputStartDir)        
    if not asciiOutputRootFolder.exists():
        asciiOutputRootFolder.mkdir()

# for all possible muscle excitement configurations, run simulator and record results
forAllMuscleExcitmentConfig(exciterList, tongueNodeList, excitMin, 0, excitIncrement, excitMax, "", asciiOutputStartDir, wpOutputStartDir, makeAsciiAlso)
