import os
import odb_py as odb
import sys

class CircuitInfo:
  def __init__(self, text):
    f = open(text, 'r')
    cont = f.read()
    f.close()

    isNetFirst = True 
    curNet = None
    curNetCnt = -1
    iterNetCnt = 0
    curNetDict = dict()

    for i, curLine in enumerate(cont.split("\n")):
      curLine = curLine.strip()
      if curLine == "":
        continue


      arr = curLine.split()
      
      if i == 0: 
        self.die = curLine.split()
      elif i == 1:
        self.core = curLine.split()
      elif i == 2:
        self.fromRouteLayer = int(arr[0])
        self.toRouteLayer = int(arr[1])
        self.totalRouteLayers = int(arr[2])
      elif hasattr(self, 'totalRouteLayers'):
        if i == 3:
          self.routeInfo = [] 
          self.routeInfo.append(arr)
        elif i > 3 and i < 3+self.totalRouteLayers:
          self.routeInfo.append(arr)
        elif i == 3+self.totalRouteLayers:
          self.siteWidth = int(arr[0])
          self.siteHeight = int(arr[1])
        elif i == 4+self.totalRouteLayers:
          self.rowCnt = int(arr[0])
          self.colCnt = int(arr[1])
          self.rowLx = int(arr[2])
          self.rowLy = int(arr[3])
        elif i == 5+self.totalRouteLayers:
          self.orients = arr
        elif i == 6+self.totalRouteLayers:
          self.totalNets = int(arr[0])
      if hasattr(self, 'totalNets'):
        if isNetFirst:
          isNetFirst = False
        else:
          if curNet is None: 
            curNet = arr[1]
            curNetCnt = int(arr[2])
            iterNetCnt = 0
          else:
            if iterNetCnt <= curNetCnt:
              iterNetCnt += 1 
              if hasattr(self, 'netList') == False:
                self.netList = dict()
                self.netList[curNet] = [[arr[1], arr[2]]]
              else:
                if curNet in self.netList:
                  self.netList[curNet].append([arr[1], arr[2]])
                else:
                  self.netList[curNet] = [[arr[1], arr[2]]]
              
              if iterNetCnt == curNetCnt:
                curNet = None
                curNetCnt = -1
                iterNetCnt = 0

    print("total nets:", len(self.netList.items()))
    for key, val in self.netList.items():
      print(" ", key)
      for v in val:
        print("  ", v[0], v[1])

    print()


  def dump(self):
    print("DIE:", self.die)
    print("CORE:", self.core)
    print("route layer:", self.fromRouteLayer, self.toRouteLayer)
    print("route info:")
    for r in self.routeInfo:
      print(r)
    print("site:", self.siteWidth, self.siteHeight)
    print("DP row/col:", self.rowCnt, self.colCnt)
    print("DP lx/ly:", self.rowLx, self.rowLy)
    print("DP orients:", self.orients)


def GetValue(smt2ResStr):
  smt2ResStr = smt2ResStr.strip()
  # print(smt2ResStr)
  if smt2ResStr.startswith("false"):
    return 0
  elif smt2ResStr.startswith("true"):
    return 1
  elif smt2ResStr.startswith("#b"):
    return int(smt2ResStr[2:-1], 2)
    pass
  elif smt2ResStr.startswith("#x"):
    return int(smt2ResStr[2:-1])

def CombineXY(listX, listY, listF):
  ret = dict()
  for x in listX:
    ret[x[0][2:]] = [x[1]]

  for y in listY:
    ret[y[0][2:]].append(y[1])

  for f in listF:
    ret[f[0][3:]].append(f[1])

  return ret

def CheckStr(forbiddenStrs, givenStr):
  for fStr in forbiddenStrs:
    if fStr in givenStr:
      return False
  return True

def GetIndex(curList, inputStr):
  for i, curStr in enumerate(curList):
    if curStr == inputStr:
      return i
  return -1

def GetVertices(newArr):
  if len(newArr) == 2:
    return (newArr[0], newArr[1])

  if len(newArr) == 3 and newArr[1] == "ep":
    return (newArr[0], "_".join(newArr[1:3]))
  elif newArr[1] == "ep":
    return (newArr[0], "_".join(newArr[1:len(newArr)]))

  if len(newArr) == 4 and newArr[1] == "it":
    return (newArr[0], "_".join(newArr[1:4]))
  elif newArr[1] == "it":
    return (newArr[0], "_".join(newArr[1:len(newArr)]))

  print("Error found at GetVertices:", newArr)
  exit()

  return ("ERROR", "ERROR")
  

def BuildRouteMap(allStrs):
  retMap = dict()
  for curRoute in allStrs:
    routeArr = curRoute.split("_")
    eIdx = GetIndex( routeArr, "E" )

    # print(routeArr, eIdx, len(routeArr[eIdx-1]))
    # skip for non _C{N}_ route net cases
    if len(routeArr[eIdx-1]) == 0 or routeArr[eIdx-1][0] != "C":
      continue

    netName = "_".join(routeArr[0:eIdx-1])
    cIdx = int(routeArr[eIdx-1][1:])
    newArr = routeArr[eIdx+1:]
    vert1, vert2 = GetVertices(newArr)

    key = netName + "_" + str(cIdx)
    if key in retMap:
      retMap[key].append([vert1, vert2])
    else:
      retMap[key] = [[vert1, vert2]]
    # print(eIdx, netName, cIdx, vert1, vert2)
  return retMap

def GetNextPin(connList, point): 
  if point in connList:
    if point == connList[0]: 
      return connList[1]
    elif point == connList[1]:
      return connList[0]
  else:
    return None

def GetMRC(mrcString):
  newStr = mrcString
  newStr = newStr.replace("m", "")
  newStr = newStr.replace("r", "!")
  newStr = newStr.replace("c", "!")
  return [int(l) for l in newStr.split("!")]


def GetRouteSegs(connLists, begin, end, steiners = None):
  #print("begin:", begin, "end:", end)
  #print("given conn list:", connLists)

  curPoint = begin
  isEnd = False

  visitor = [0] * len(connLists)
  totalList = [begin]

  while True:
    isFound = False
    for i, connList in enumerate(connLists):
      if visitor[i] == 0 and curPoint in connList:
        nextPin = GetNextPin(connList, curPoint)
        isFound = True
        visitor[i] = 1
        break
    
    if isFound == False:
      break
    else:
      totalList.append(nextPin)
      curPoint = nextPin
      
  # print("Found routing:", "\n".join(totalList))

  # summarized the whole routing
  shortTotalList = []

  # either VIA/HORIZONTAL/VERTICAL segment increasing
  viaDiff = 0
  horDiff = 0
  verDiff = 0
  prevElem = None
  
  for i in range(len(totalList)-1):
    curElem = totalList[i]
    nextElem = totalList[i+1]

    if curElem.startswith("it") or curElem.startswith("ep") or \
        nextElem.startswith("it") or nextElem.startswith("ep"):
      shortTotalList.append(curElem)
      shortTotalList.append(nextElem)
    else:
      curMrc = GetMRC(curElem)
      nextMrc = GetMRC(nextElem)
      
      newViaDiff = nextMrc[0] - curMrc[0]
      newHorDiff = nextMrc[1] - curMrc[1]
      newVerDiff = nextMrc[2] - curMrc[2]

      # outlier handling (M3 <-> M4)
      if (curMrc[0] == 3 and nextMrc[0] == 4) or (curMrc[0] == 4 and nextMrc[0] == 3): 
        newVerDiff = newHorDiff = 0

      # now, only one elem is different
      # ignore first case
      # the via diff cannot be shortened
      if i != 1 and (newViaDiff != 0 or newHorDiff != horDiff or newVerDiff != verDiff or (steiners != None and curElem in steiners)):
        #print("Diff happen", curElem, nextElem)
        shortTotalList.append(curElem)

      #print(curMrc, nextMrc, newViaDiff, newHorDiff, newVerDiff)
      viaDiff = newViaDiff
      horDiff = newHorDiff
      verDiff = newVerDiff

  # print("ShortTotalList:", "\n".join(shortTotalList))
  return totalList, shortTotalList

def GetSteiners(totalSegsList):
  steinerResDict = dict() 
  steinerDict = dict()
  for segs in totalSegsList:
    
    prevCnt = -1
    prevSeg = None
    for seg in segs:
      
      curCnt = 0
      if seg in steinerDict:
        curCnt = steinerDict[seg]
        steinerDict[seg] += 1
      else:
        curCnt = 0
        steinerDict[seg] = 1

      # steiner point detected
      if prevCnt != -1 and prevCnt != curCnt: 
        steinerResDict[prevSeg] = 1

      prevCnt = curCnt
      prevSeg = seg

  steinerList = []
  for key, val in steinerResDict.items():
    steinerList.append(key)
  print("found steiner points:", steinerList)
  return steinerList

print("Given DB:", sys.argv[1])
print("SMT2 loc:", sys.argv[2])

db = odb.dbDatabase.create()
odb.read_db(db, sys.argv[1])
block = db.getChip().getBlock()

# curFile: smt2
def WriteRouteFile(curFile):
  print("parsing: " + curFile + ".helper")
  ci = CircuitInfo(curFile + ".helper")
  ci.dump()

  f = open(curFile + ".result", 'r')
  cont = f.read()
  f.close()

  allTrueRouteStr = []
  xStr = []
  rStr = []
  fStr = []

  conts = cont.split("\n")
  # skip for errors
  conts = [l for l in conts if l.startswith("(error") == False]

  # skip for unsat smt2
  if conts[0] == "unsat" or conts[0] == "unknown":
    return

  for i, curLine in enumerate(conts):
    curLine = curLine.strip()

    if curLine.startswith("(define-fun") == False:
      continue

    if "BitVec" in curLine:
      varStr = conts[i].strip().split()[1]
      if " x_" in curLine:
        xStr.append([varStr, GetValue(conts[i+1])])
      elif " r_" in curLine:
        rStr.append([varStr, GetValue(conts[i+1])])

    if " ff_" in curLine:
      varStr = conts[i].strip().split()[1]
      fStr.append([varStr, GetValue(conts[i+1])])

    if conts[i+1].strip().startswith("false"):
      continue

    varStr = conts[i].strip().split()[1]
    if "_E_" in varStr: 
      allTrueRouteStr.append(varStr)
 
  # print(allTrueRouteStr)
  xyDict = CombineXY(xStr, rStr, fStr)
  instResult = []

  for key, val in xyDict.items():
    inst = block.findInst(key)
    newLx = ci.rowLx + val[0]*ci.siteWidth 
    newLy = ci.rowLy + val[1]*ci.siteHeight

    if val[2] == 0:
      orient = "R0" if ci.orients[val[1]] == "N" else "MX"
    elif val[2] == 1:
      orient = "MY" if ci.orients[val[1]] == "N" else "R180"

    #print(inst.getConstName(), val)
    #print("prev:", inst.getLocation())
    #inst.setOrient(orient)
    #inst.setLocation(newLx, newLy)

    instResult.append([inst, orient, newLx, newLy])
    #print("new:", inst.getLocation())
  
  print("placement updated")

  routeMap = BuildRouteMap(allTrueRouteStr)

  retStr = ""
  # write PLACEMENT
  retStr += "PLACEMENT " + str(len(instResult)) + "\n"
  for inst, orient, newLx, newLy in instResult:
    retStr += " %s %s %d %d\n" % (inst.getConstName(), orient, newLx, newLy)

  # write ROUTE
  for curNet, epPair in ci.netList.items():

    retStr += curNet + " " + str(len(epPair)) + "\n"
    
    totalSegsList = []
    for i, (begin, end) in enumerate(epPair):
      key = curNet + "_" + str(i)
      totalSegs, _ = GetRouteSegs(routeMap[key], begin, end, None)
      totalSegsList.append(totalSegs)

    if len(totalSegsList) == 1:
      steinerList = None
    else:
      steinerList = GetSteiners(totalSegsList)

    for i, (begin, end) in enumerate(epPair):
      key = curNet + "_" + str(i)
      _, summSegs = GetRouteSegs(routeMap[key], begin, end, steinerList)
      retStr += " " + str(i) + " " + str(len(summSegs)) + "\n"
      retStr += "   "
      retStr += "\n   ".join(summSegs)
      retStr += "\n"
    
    if len(totalSegsList) == 1:
      retStr += " STEINER 0\n" 
    else:
      retStr += " STEINER " + str(len(steinerList)) + "\n"
      for stn in steinerList:
        retStr += "  " + stn + "\n"


  f = open(curFile + ".route", 'w')
  f.write(retStr)
  f.close()

  print('')

# pass smt2
WriteRouteFile(sys.argv[2])

# finish python in a normal way - stdout needs to be flushed
sys.stdout.flush()
os._exit(0)

