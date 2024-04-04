import sys

def ParseDrc(fileName):
  retDict = dict()

  f = open(fileName, 'r')
  cont = f.read()
  f.close()
  

  key = ""
  drc = -1
  for curLine in cont.split('\n'):
    if curLine.startswith("#") or curLine == "":
      continue

    curLine = curLine.strip()
    arr = curLine.split(" ")
    if curLine.startswith("Total"):
      drc = arr[3]
    elif curLine.startswith("Bounds"):
      val = ' '.join([arr[3][:-1], arr[4], arr[7][:-1], arr[8]])
      # print("VAL", val)
      if not key in retDict:
        retDict[key] = val 
    else:
      key = arr[0][:-1]
      firstEndIdx = -1
      secondStartIdx = -1
      atIdx = -1

      for i in range(1, len(arr)):
        if firstEndIdx == -1 and arr[i] == ')': 
          firstEndIdx = i

        if atIdx == -1 and arr[i] == '&': 
          atIdx = i
        
        if firstEndIdx != -1 and arr[i] == '(':
          secondStartIdx = i
          break

      if atIdx == -1:
        key = key + '/' + ' '.join(arr[firstEndIdx+1:secondStartIdx-1])
      else:
        firstElem = ' '.join(arr[firstEndIdx+1:atIdx])
        secondElem = ' '.join(arr[atIdx+1:secondStartIdx-1])
        sortArr = [firstElem, secondElem]
        sortArr.sort()
        key = key + '/' + ' '.join(sortArr)

  print("Parsing", fileName, "is finished. #DRC=", drc)
  return retDict 

def CheckBreakDown(givenDict):
  newDict = dict()
  for key, val in givenDict.items():
    newKey = key.split("/")[0]
    if newKey in newDict:
      newDict[newKey] += 1
    else:
      newDict[newKey] = 1

  for key, val in sorted(newDict.items()):
    print(key, val)


prev = ParseDrc(sys.argv[1])
print("PREV BREAKDOWN")
CheckBreakDown(prev)
print()

after = ParseDrc(sys.argv[2])
print("NEW BREAKDOWN")
CheckBreakDown(after)
print()

for key, val in after.items():
  if key in prev:
    continue
  else:
    print("NEW DRV:", key, val)

print()

# for i in range(3,11):
#   after = ParseDrc('after_drc_v3_%d.rpt' %(i))
# 
#   print("NEW BREAKDOWN")
#   CheckBreakDown(after)
#   print()
#   
#   for key, val in after.items():
#     if key in prev:
#       continue
#     else:
#       print("NEW DRV:", key, val)
# 
#   print()
