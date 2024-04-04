import subprocess as sp 
import os
import copy

comEnv = os.environ.copy() 

curRun = comEnv["MGWOO_SINGLE_RUN"]
curDrvPath = comEnv["MGWOO_TRIAL_DIR"]

sp.call("./openroad parallel_single_run.tcl | tee %s.log" % (curRun), env=comEnv, shell=True)

# solution EXISTS!!!
if os.path.isfile(curRun + ".done") == True:
  sp.call("mkdir -p ./solution/", shell=True)
  sp.call("mv %s.*  ./solution/" % (curRun), shell=True)
  sp.call("ps -ef | grep mgwoo | grep %s | tr -s ' ' | cut -d' ' -f2 | xargs kill -9" %(curRun), shell=True)
  sp.call("ps -ef | grep mgwoo | grep %s | tr -s ' ' | cut -d' ' -f2 | xargs kill -9" %(curDrvPath), shell=True)
else:
  sp.call("rm -rf %s*" % (curRun), shell=True)

