import subprocess as sp 
import os
import copy
import glob

def GetDrcs(tclFile):
  f = open(tclFile, 'r')
  cont = f.read()
  f.close()

  cont = cont.split("set drc_list")[1]
  retList = []

  for curVal in cont.split("}"):
    arr = curVal.split()
    if len(arr) != 4:
      continue

    if arr[0].startswith("{{"):
      arr[0] = arr[0][2:]
    elif arr[0].startswith("{"):
      arr[0] = arr[0][1:]
   
    arr = [int(i) for i in arr]
    retList.append(arr)
  return retList

def getDxDyName(val):
  if val < 0:
    return "n"+str(-1*val)
  else:
    return "p"+str(val)

def getDbFiles(filesCandi): 
  for curDb in sorted(filesCandi, key=os.path.getctime):
    c = curDb[:-3]
    if os.path.exists(c + ".rects") and os.path.exists(c + ".def"):
      return curDb
  return None


# clear all parallel runs
# sp.call("ps -ef | grep mgwoo | grep parallel | tr -s ' ' | cut -d' ' -f2 | xargs kill -9", shell=True)

comEnv = os.environ.copy() 

# default RECT set
comEnv["MGWOO_RUN_RECTS"] = ""

numTrial = int(comEnv["MGWOO_TRIAL_DIR"][-1])

if numTrial == 0:
  mxPitches = [i for i in range(6,20,2)]
  myPitches = [i for i in range(6,20,4)]
  dxList = [i for i in range(-10, 11, 2)]
  dyList = [i for i in range(-10, 11, 4)]
elif numTrial == 1:
  mxPitches = [i for i in range(7,20,2)]
  myPitches = [i for i in range(7,20,4)]
  dxList = [i for i in range(-10, 11, 2)]
  dyList = [i for i in range(-10, 11, 4)]
else:
  mxPitches = [i for i in range(6,20,2)]
  myPitches = [i for i in range(8,20,4)]
  dxList = [i for i in range(-10, 11, 2)]
  dyList = [i for i in range(-10, 11, 4)]


#24 36 25 - original setup


drcList = GetDrcs(comEnv["MGWOO_DRC_TCL"])
#drcList = GetDrcs("/path/to/example/ldpc/drc/location/ldpc_core_eco_0_drc.tcl")

# create trial run dir
sp.call("rm -rf %s" % (comEnv["MGWOO_TRIAL_DIR"]), shell=True)
sp.call("mkdir -p %s" % (comEnv["MGWOO_TRIAL_DIR"]), shell=True)

newFile = None
solutions = []
for xy in drcList:
  cx = (xy[0] + xy[2])/2
  cy = (xy[1] + xy[3])/2
  newDir = "%s/%d_%d" % (comEnv["MGWOO_TRIAL_DIR"], cx, cy)
  sp.call("rm -rf %s" % (newDir), shell=True)
  sp.call("mkdir -p %s" % (newDir), shell=True)
  sp.call("ln -s $(readlink -f .)/../ref/parallel_single_run.* %s/" %(newDir), shell=True)
  sp.call("ln -s $(readlink -f .)/../ref/node* %s/" %(newDir), shell=True)
  sp.call("ln -s $(readlink -f .)/../ref/runme.py %s/" %(newDir), shell=True)
  sp.call("ln -s $(readlink -f .)/../ref/z3 %s/" %(newDir), shell=True)
  sp.call("ln -s $(readlink -f .)/../openroad %s/" %(newDir), shell=True)
  sp.call("ln -s $(readlink -f .)/%s %s/" %(comEnv["MGWOO_RUN_DB"], newDir), shell=True)

  if comEnv["MGWOO_RUN_RECTS"] != "":
    sp.call("ln -s $(readlink -f .)/%s %s/" %(comEnv["MGWOO_RUN_RECTS"], newDir), shell=True)

  drvBox = "%d_%d_%d_%d" % (xy[0], xy[1], xy[2], xy[3])

  runList = []


  for mx in mxPitches:
    for my in myPitches:
      solveRangeX = 80 * mx + 40
      solveRangeY = 80 * my + 40

      for dx in dxList:
        xdx = dx * 80
        for dy in dyList:
          ydy = dy * 80

          solveBox = [cx + xdx - solveRangeX/2, \
              cy + ydy - solveRangeY/2, \
              cx + xdx + solveRangeX/2, \
              cy + ydy + solveRangeY/2]

          if xy[0] >= solveBox[0] and \
              xy[1] >= solveBox[1] and \
              xy[2] <= solveBox[2] and \
              xy[3] <= solveBox[3]:

            drvPnt = "%d_%d" % (cx + xdx, cy+ydy)
            runName = "%d_%d_%s_%s" % (mx, my, getDxDyName(xdx), getDxDyName(ydy))
            runList.append("MGWOO_TRIAL_DIR=%s MGWOO_SINGLE_RUN=%s MGWOO_DRV_RUN_PNT=%s MGWOO_DRV_BOX=%s \
                python3 parallel_single_run.py" % (newDir, runName, drvPnt, drvBox))

  f = open("%s/parallel_run.sh" %(newDir), 'w')
  f.write("\n".join(runList))
  f.close()

  sp.call("cd %s && /path/to/gnu/parallel --bar \
      --env PATH --env LM_LICENSE_FILE --env LD_LIBRARY_PATH \
      --env MGWOO_TRIAL_DIR --env MGWOO_DRC_TCL \
      --env MGWOO_RUN_DB --env MGWOO_RUN_RECTS --env PYTHONPATH \
      --sshloginfile node_mgwoo.txt --workdir $PWD < parallel_run.sh" % (newDir), shell=True, env=comEnv)

  files = glob.glob("%s/solution/*.db" % (newDir))
  dbFile = getDbFiles(files)
  if dbFile != None:
    rectFile = dbFile[:-3] + ".rects" 
    defFile = dbFile[:-3] + ".def"

    solutions.append(dbFile)
    newFile = "%d_%d_%d_%s" % (len(solutions), cx, cy, dbFile.split("/")[-1][:-3])

    cnt = 0
    while os.path.exists(newFile + ".rects") == False:
      cnt += 1
      sp.call("ln -s %s ./%s.rects" % (rectFile, newFile), shell=True)
      if cnt >= 3:
        break

    # THE FOLLOWING CRASHED A LOT -> WHILE
    cnt = 0
    while os.path.exists(newFile + ".def") == False:
      cnt += 1
      sp.call("ln -s %s ./%s.def" % (defFile, newFile), shell=True)
      if cnt >= 3:
        break

    cnt = 0
    while os.path.exists(newFile + ".db") == False:
      cnt += 1
      sp.call("ln -s %s ./%s.db" % (dbFile, newFile), shell=True)
      if cnt >= 3:
        break

    comEnv["MGWOO_RUN_DB"] = newFile + ".db"
    comEnv["MGWOO_RUN_RECTS"] = newFile + ".rects" 

    print("Success!", newFile)

if newFile is not None:
  sp.call("ln -s %s.db %s" % (newFile, comEnv["MGWOO_WRITE_DB"]), shell=True)
  sp.call("ln -s %s.def %s" % (newFile, comEnv["MGWOO_WRITE_DEF"]), shell=True)
