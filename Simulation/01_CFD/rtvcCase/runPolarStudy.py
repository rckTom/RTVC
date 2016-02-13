import PyFoam
import math
import time
import numpy as np
from PyFoam.RunDictionary.SolutionDirectory import SolutionDirectory
from PyFoam.RunDictionary.SolutionFile import SolutionFile
from PyFoam.RunDictionary.ParsedParameterFile import ParsedParameterFile
from PyFoam.Execution.AnalyzedRunner import AnalyzedRunner
from PyFoam.LogAnalysis.StandardLogAnalyzer import StandardLogAnalyzer
from PyFoam.Execution.BasicRunner import BasicRunner
from PyFoam.Execution.GnuplotRunner import GnuplotRunner

case = "polarStudy"
caseDir = SolutionDirectory(".",archive="polars")
caseDir.addBackup("postProcessing/forceCoeffs/0/forceCoeffs.dat")
caseDir.addBackup("PyFoamSolve.logfile")
caseDir.addBackup("PyFoamSolve.analyzed")
parameters = ParsedParameterFile("boundaryConditions",noHeader=True,preserveComments=False)

vRange =   np.arange(25,31,2.5)
alphaRange = np.arange(0,11,1)
print vRange
caseDir.clearResults()
caseDir.clear(functionObjectData=True)

valueList = []

for i in range(len(vRange)):
    for j in range(len(alphaRange)):
        valueList.append({'velocity':vRange[i],'alpha':alphaRange[j]})

valueList.append({'velocity':10,'alpha':4})
valueList.append({'velocity':5,'alpha':10})
valueList.append({'velocity':7.5,'alpha':10})
print valueList

for k in range(6,len(valueList)):
    vel = valueList[k]['velocity']
    alpha = valueList[k]['alpha']
    caseDir.clear(functionObjectData=True)
    caseDir.clearOther()
    caseDir.clearPattern("postProcessing")
    caseDir.clearResults(functionObjectData=True)
    velocity = [0,math.sin(alpha*math.pi/180)*vel,-math.cos(alpha*math.pi/180)*vel]
    print "Simulation {:d} from {:d}: velocity = {:f} alpha = {:f}".format(k,len(valueList),vel,alpha)
    parameters['flowVelocity'] = "({:f} {:f} {:f})".format(velocity[0],velocity[1],velocity[2])
    parameters['magUInf'] = "{:f}".format(vel)
    parameters.writeFile()
    decompose = BasicRunner(argv=["decomposePar"],silent=True)
    print " Decompose Dictionary"
    start = time.clock()
    decompose.start()
    print " Run SimpleFoam"
    run = AnalyzedRunner(StandardLogAnalyzer(),argv=["mpirun -np 4","simpleFoam","-parallel"],silent=True)
    run.start()
    run.picklePlots()
    print " Reconstruct Par"
    reconstruct = BasicRunner(argv=["reconstructPar"],silent=True)
    reconstruct.start()
    end = time.clock()
    print " Save Data"
    print " Run finalized. Ellapsed time: {:f}s".format(end-start)
    caseDir.lastToArchive("vel={:f}alpha={:f}".format(vel,alpha))