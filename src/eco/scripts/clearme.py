import os 

def getExecClearList(solDir):
  retList = []
  for curDir in os.listdir(solDir):
    if curDir.endswith(".log") or curDir.endswith(".rects") or curDir.endswith(".db") or \
        curDir.endswith(".smt2") or curDir.endswith(".def") or curDir.endswith(".done") or \
        curDir.endswith(".helper") or curDir.endswith(".route") or curDir.endswith("result"):
      curLen = (len(curDir.split("_")))
      retList.append("rm -rf " + solDir + "/" + curDir)
  return retList

def getSolClearList(key, solDir):
  retList = []
  for curDir in os.listdir(solDir):
    if curDir.endswith(".log") or curDir.endswith(".rects") or curDir.endswith(".db") or \
        curDir.endswith(".smt2") or curDir.endswith(".def") or curDir.endswith(".done") or \
        curDir.endswith(".helper") or curDir.endswith(".route") or curDir.endswith("result"):
      if not key in curDir: 
        retList.append("rm -rf " + solDir + "/" + curDir)
  return retList


allList = []
for curDir in sorted(os.listdir(".")):
  if curDir.endswith(".db"):
    if curDir[0].isnumeric():
      fullDir = os.path.realpath(curDir)

      solutionKey = fullDir.split("/")[-1][:-3]
      solDir = "/".join(fullDir.split("/")[:-1])
      execDir = "/".join(fullDir.split("/")[:-2])
      clearList1 = getExecClearList(execDir)
      clearList2 = getSolClearList(solutionKey, solDir)
      # print(solutionKey, solDir)

      allList = allList + clearList1 + clearList2

print("erase", len(allList), "files")
f = open("clear.sh", 'w')
f.write("\n".join(allList))
f.close()


