import sys
import os

f = open(sys.argv[1], 'r')
cont = f.read()
f.close()

stringList = []
dbu = 2000

drvDict = dict()

for curLine in cont.split("\n"):
  if "Bounds" in curLine:
    arrs = curLine.strip().split()
    lx = float(arrs[3][:-1])
    ly = float(arrs[4])
    ux = float(arrs[7][:-1])
    uy = float(arrs[8])

    # avoid the duplicated DRV find issues
    sumX = int(dbu*lx) + int(dbu*ux)
    sumY = int(dbu*ly) + int(dbu*uy)
    key = "%d_%d" % (sumX, sumY)

    if not (key in drvDict):
      stringList.append( "{%d %d %d %d} " % (int(dbu*lx), int(dbu*ly), int(dbu*ux), int(dbu*uy)) )
      drvDict[key] = 1



def getSloVios(fileName, drvDict):
  f = open(fileName,'r')
  cont = f.read()
  f.close()

  gdsDbu = 1000

  retList = []
  for i, curLine in enumerate(cont.split("\n")):
    if i == 0 or curLine == "":
      continue
    arr = curLine.split(",")

    cx = int(arr[0])
    cy = int(arr[1])

    # 2cx * 2 (def/gds dbu)
    key = "%d_%d" % (cx*4, cy*4)

    if not (key in drvDict):
      retList.append("{%d %d %d %d} " % ( (cx-80)*2, (cy-80)*2, (cx+80)*2, (cy+80)*2 ))
      drvDict[key] = 1
  return retList


comEnv = os.environ.copy()
m2Vios1 = getSloVios(comEnv["MGWOO_ECLAIR_OUTPUT"] + "/search_m2_p1/output_m2_og_p1/results_m2_p1.csv", drvDict)
m3Vios1 = getSloVios(comEnv["MGWOO_ECLAIR_OUTPUT"] + "/search_m3_p1/output_m3_og_p1/results_m3_p1.csv", drvDict)

m2Vios2 = getSloVios(comEnv["MGWOO_ECLAIR_OUTPUT"] + "/search_m2_p2/output_m2_og_p2/results_m2_p2.csv", drvDict)
m3Vios2 = getSloVios(comEnv["MGWOO_ECLAIR_OUTPUT"] + "/search_m3_p2/output_m3_og_p2/results_m3_p2.csv", drvDict)

#m2Vios = getSloVios("eclair_output_0/search_m2_p1/output_m2_og_p1/results_m2_p1.csv", drvDict)
#m3Vios = getSloVios("eclair_output_0/search_m3_p1/output_m3_og_p1/results_m3_p1.csv", drvDict)

totalStr = "".join(stringList) + "".join(m2Vios1) + "".join(m3Vios1) + "".join(m2Vios2) + "".join(m3Vios2)

f = open(sys.argv[2], 'w')
f.write("set drc_list {%s}" % (totalStr))
f.close()
