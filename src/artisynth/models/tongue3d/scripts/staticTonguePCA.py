### USER DEFINED SETTINGS

tonguePath = "models/0/models/0/"
tongueNodeListPath = tonguePath + "nodes/"

# these variables will be initialized from a data file called "toMatrixVariables.txt" in the wp folder
excitIncrement = None
excitMin = None
excitMax = None
numExciters = None

# starting directory to waypoint files. Shouldn't modify.
wpOutputStartDir = "wpOutput/"

# used to compare two floats. Shouldn't have to modify
ELIPSON = 0.0001





"""
What does this script do?

It will iterate through the pre-generated waypoint files and dump the surface tongue node positions
into a matrix. The resulting data matrix can be passed on for PCA.

The data matrix layout:
    
          Node #1         Node #2   ....       Last Surface Node (#m)
      __________________________________________________________
Sim1 | x11, y11, z11,    x12, y12, z12    ....,    x1m, y1m, z1m |
Sim2 | x21, y21, z21,    x22, y22, z22    ....,    x2m, y2m, z2m |
Sim3 | x31  ...                ....                              |
Sim4 | x41  ...                ....                              |
Sim5 | x51
 ..                                                                 |
Last |                                                              |
Sim  | xn1, yn1, zn1,                     .....,   xnm, ynm, znm |
(#n) |                                                             |
      ------------------------------------------------------------
"""




numSurfaceNodes = None

import java.io.File
import java.io.DataInputStream
import java.io.FileInputStream

import maspack.matrix.MatrixNd

### FUNCTION DEFINITIONS

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
Initialize global variables from data file. The data will be generated from
staticTongueGenerator.py, which creates a file called toMatrixVariables.txt, located
in <folderLocation>.
@param:
    folderLocation :: string - directory path to toMatrixVariables.txt. Should be
                               wpOutput/ by default.
    
    excitIncrement :: global
    excitMin :: global
    excitMax :: global
    numExciters :: global
@return: none
"""
def initGlobalVariables(folderLocation):
    scanner = java.util.Scanner( File(folderLocation + "toMatrixVariables.txt") )
    global excitMin
    global excitIncrement
    global excitMax
    global numExciters
    excitMin = scanner.nextDouble()
    excitIncrement = scanner.nextDouble()
    excitMax = scanner.nextDouble()
    numExciters = scanner.nextInt()
    scanner.close()





"""
Does the specified waypoint file exist?
@param:
    startingDir :: string - starting directory of where all the waypoint files
                            are located.
    excitePath :: string - starting from startingDir, excitePath leads to
                           location of waypoint file. e.g. <startingDir>0.0/0.1/0.1/<lastExciteValue>
    lastExciteValue :: double - the final value of supposedly exciter
@return:
    True if waypoint file exists, false otherwise
"""
def waypointExists(startingDir, excitePath, lastExciteValue):
    wpPath = startingDir + excitePath + str(lastExciteValue) + "_wp.txt"
    wpFile = File(wpPath)
    return wpFile.exists()




"""
Adjust the tongue model as specified by the waypoint.
@param:
    startingDir :: string - starting directory of where all the waypoint files
                            are located.
    excitePath :: string - starting from startingDir, excitePath leads to
                           location of waypoint file. e.g. <startingDir>0.0/0.1/0.1/<waypointFile>
    lastExciteValue :: double - the final value of supposedly exciter
@return: None
"""
def loadModelStateFromWaypoint(startingDir, excitePath, lastExciteValue):
    wpPath = startingDir + excitePath + str(lastExciteValue) + "_wp.txt"
    Main.getRootModel().getWayPoints().setAttachedFileName(wpPath)
    waypointFile = Main.getRootModel().getWayPoints().getAttachedFile()
    dis = DataInputStream( FileInputStream (waypointFile) )
    dis.readInt()
    dis.readDouble()
    cs = Main.getRootModel().createState()
    cs.readBinary(dis)
    dis.readDouble()
    cs = Main.getRootModel().createState()
    cs.readBinary(dis)
    Main.getRootModel().setState(cs)
    Main.getRootModel().rerender()




"""
Append a row of node data to the matrix
@param:
    tongueNodeList :: list - references to nodes that define tongue shape
    matrix :: maspack.matrix.MatrixNd - Matrix used to store tongue node positions.
                                        Should pass empty 0x0 matrix on first call
    numSurfaceNodes :: global int - number of surface nodes on the tongue
@return: none
"""
def addNodesPosToMatrix(tongue, tongueNodeList, matrix):
    global numSurfaceNodes
    x = 0
    y = 1
    z = 2
    matrix.setSize(matrix.rowSize()+1, numSurfaceNodes*3)
    curColBlockIdx = 0                # the next 1x3 column of where the next x,y,z data is to be inserted in the appended row
    for nodeIdx in range(0, tongueNodeList.size()):            #for each node, add its x,y,z values into appended row
        if tongue.isSurfaceNode( tongueNodeList.get(nodeIdx) ):
            curCol = curColBlockIdx * 3
            matrix.set(matrix.rowSize()-1, curCol, tongueNodeList.get(nodeIdx).getPosition().get(x))
            matrix.set(matrix.rowSize()-1, curCol+1, tongueNodeList.get(nodeIdx).getPosition().get(y))
            matrix.set(matrix.rowSize()-1, curCol+2, tongueNodeList.get(nodeIdx).getPosition().get(z))
            curColBlockIdx += 1
    wordedNodesList = ["top", "tip", "base"]
    for wordedNodeIdx in range(0, len(wordedNodesList)):            # add these 3 nodes to the appended row also
        curCol = curColBlockIdx * 3
        matrix.set(matrix.rowSize()-1, curCol, tongueNodeList.get(wordedNodesList[wordedNodeIdx]).getPosition().get(x))
        matrix.set(matrix.rowSize()-1, curCol+1, tongueNodeList.get(wordedNodesList[wordedNodeIdx]).getPosition().get(y))
        matrix.set(matrix.rowSize()-1, curCol+2, tongueNodeList.get(wordedNodesList[wordedNodeIdx]).getPosition().get(z))
        curColBlockIdx += 1





"""
Get the number of surface nodes of the tongue model.
@param:
    tongueNodeList :: list - references to every tongue node
    tongue :: FemMuscleModel - reference to the tongue
@return: number of surface nodes on the tongue
"""
def getNumSurfaceNodes(tongueNodeList, tongue):
    numSurfNodes = 0
    for nodeIdx in range(0, tongueNodeList.size()):
        if tongue.isSurfaceNode( tongueNodeList.get(nodeIdx) ):
            numSurfNodes += 1
    numSurfNodes += 3            # account for "top", "tip", and "base" nodes
    return numSurfNodes





"""
It will iterate through the pre-generated waypoint files and dump the surface tongue node positions
into a matrix.
@param:
    matrix :: maspack.matrix.MatrixNd - Matrix used to store tongue node positions.
                                        Should pass empty 0x0 matrix on first call
    numExciters :: int - number of exciters
    tongueNodeList :: list - references to nodes that define tongue shape
    tongue :: FemMuscleModel - reference to the tongue
    min :: double - minimum excitation level
    cur :: double - the current exciter idx that's being iterated through. First call should pass 0
    increment :: double - how much do you want to increment the excitation level per iteration?
    max :: double - maximum excitation leveL
    excitePath :: string - relative directory path so far using excitation values only.
                           (e.g. 0.0/0.1/...). First call should pass empty string ""
    wpOutputStartDir :: string - starting directory location of where waypoint files should be stored
"""
def iterateThroughWaypointsAndRecordToMatrix(matrix, numExciters, tongue, tongueNodeList, min, cur, increment, max, excitePath, wpOutputStartDir):
    if cur == numExciters - 1:
        for i in CLessEqualLoop(min, max, increment):
            if waypointExists(wpOutputStartDir, excitePath, i):
                System.out.println("WP exist for: " + excitePath + str(i))
                loadModelStateFromWaypoint(wpOutputStartDir, excitePath, i)
                addNodesPosToMatrix(tongue, tongueNodeList, matrix)
                printMatrixToFile(matrix)
            else:
                System.out.println("No WP for: " + excitePath + str(i))
    else:
        for i in CLessEqualLoop(min, max, increment):
            iterateThroughWaypointsAndRecordToMatrix(matrix, numExciters, tongue, tongueNodeList, min, cur+1, increment, max, excitePath + str(i) + "/", wpOutputStartDir)







### MAIN CODE BODY

# load data to initialize the global variable
initGlobalVariables(wpOutputStartDir)

# this list below will hold reference to all the tongue nodes
tongueNodeList = find( tongueNodeListPath )

# create a matrix to store the node positions
matrix = MatrixNd()

tongue = find( tonguePath )
numSurfaceNodes = getNumSurfaceNodes(tongueNodeList, tongue)

iterateThroughWaypointsAndRecordToMatrix(matrix, numExciters, tongue, tongueNodeList, excitMin, 0, excitIncrement, excitMax, "", wpOutputStartDir)

