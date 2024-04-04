import subprocess as sp 
import os
import copy


comEnv = os.environ.copy()


design = comEnv["MGWOO_INIT_DESIGN"]
cDef = comEnv["MGWOO_INIT_DEF"]
cLef = comEnv["MGWOO_INIT_LEF"]

#design = aes
#cDef = "aes_0.73_6T_3MPO_EL_198_eco20.def"
#cLef = "6T_2F_40CPP_40MP_3MPO_EL_M1"

comName = ".".join(cDef.split(".")[:-1]).replace(".","_")
print(cLef, cDef)
print("current def comm:", comName)

sp.call("rm -rf " + comName, shell=True)
sp.call("mkdir -p " + comName, shell=True)
sp.call("cd %s && ln -s ../ref/* ./" %(comName), shell=True)
sp.call("cd %s && ln -s ../openroad ./" %(comName), shell=True)
sp.call("cd %s && ln -s ../%s ./" %(comName, cDef), shell=True)

sp.call("cd %s && cp /path/to/tech/lef/location/%s.lef ./%s_tech.lef" %(comName, cLef, cLef), shell=True)
sp.call('cd %s && sed -i "s/LAYER LEF58_CORNEREOLKEEPOUT STRING/#LAYER LEF58_CORNEREOLKEEPOUT STRING/g" ./%s_tech.lef' %(comName, cLef), shell=True)
sp.call('cd %s && sed -i "s/PROPERTY LEF58_EOLKEEPOUT/#PROPERTY LEF58_EOLKEEPOUT/g" ./%s_tech.lef' %(comName, cLef), shell=True)

comEnv["MGWOO_TECH_LEF"] = "%s_tech.lef" %(cLef)
comEnv["MGWOO_CELL_LEFS"] = " ".join(["%s_ext%d.lef" %(cLef, i) for i in range(4)])

cellLefs = ["/path/to/cell/lef/location/ext%d/%s/%s.lef" %(i,cLef,cLef) for i in range(4)]
for i, cellLef in enumerate(cellLefs):
  sp.call("cd %s && ln -s %s ./%s_ext%d.lef" %(comName, cellLef, cLef, i), shell=True)

comEnv["MGWOO_CORE_ECO_INIT_DEF"] = cDef 
comEnv["MGWOO_CORE_ECO_INIT_DB"] = comName + "_prepared.db"

sp.call("cd %s && ./openroad gen_ordb.tcl | tee gen_ordb_init.log" % (comName), env=comEnv, shell=True)

for i in range(5):
  newEnv = copy.deepcopy(comEnv)

  if i == 0:
    drvChkDef = cDef
    runDb = comName + "_prepared.db"
  else:
    drvChkDef = design + "_core_eco_" + str(i-1) + ".def"
    runDb = design + "_core_eco_" + str(i-1) + ".db" 

  rptName = design + "_core_eco_" + str(i) + ".rpt"
  rptTclName = design + "_core_eco_" + str(i) + "_drc.tcl"
  writeDb = design + "_core_eco_" + str(i) + ".db"
  writeDef = design + "_core_eco_" + str(i) + ".def"

  eclairOutput = "eclair_output_%d" % (i)

  newEnv["MGWOO_CORE_ECO_DRV_DEF"] = drvChkDef 
  newEnv["MGWOO_CORE_ECO_DRV_RPT"] = rptName 
  newEnv["MGWOO_TRIAL_DIR"] = "inc_run_%d" % (i)
  newEnv["MGWOO_DRC_TCL"] = rptTclName
  newEnv["MGWOO_RUN_DB"] = runDb 
  newEnv["MGWOO_WRITE_DB"] = writeDb 
  newEnv["MGWOO_WRITE_DEF"] = writeDef 
  newEnv["MGWOO_ECLAIR_OUTPUT"] = eclairOutput 

  sp.call("cd %s && innovus -init gen_drv.tcl | tee drv_run_%d.log" % (comName, i), 
      env=newEnv, shell=True)

  # the eclair must be in centos7 machine 
  f = open("%s/eclair_run.sh" %(comName), 'w')
  f.write("MGWOO_ECLAIR_OUTPUT=%s MGWOO_CORE_ECO_DRV_RPT=%s eclair cafe pm_core_eco_common.py --num-workers 2 --verbose | tee eclair_run_%d.log" % (eclairOutput, rptName, i))
  f.close()

  sp.call("cd %s && /path/to/gnu/parallel --bar \
    --env PATH --env LM_LICENSE_FILE --env LD_LIBRARY_PATH \
    --env PYTHONPATH \
    --sshloginfile node_eclair.txt --workdir $PWD < eclair_run.sh" % (comName), shell=True, env=comEnv)

  # parallel run func
  sp.call("cd %s && python3 parse_drv.py %s %s" % (comName, rptName, rptTclName), 
      env=newEnv, shell=True)
  #sp.call("cd %s && ./openroad incremental_gen.tcl | tee %s_inc_%d.log" % (comName, design, i), 
  #    env=newEnv, shell=True)

  sp.call("cd %s && python3 parallel_gen.py | tee %s_par_%d.log" % (comName, design, i), 
      env=newEnv, shell=True)

  # EXCLUDE RUNTIME FOR THESE
  sp.call("cd %s && touch clean_start_%d" % (comName, i), shell=True)
  sp.call("cd %s && python3 clearme.py" %(comName), shell=True)
  sp.call("cd %s && chmod a+x ./clear.sh" %(comName), shell=True)
  sp.call("cd %s && ./clear.sh" %(comName), shell=True)
  sp.call("cd %s && touch clean_done_%d" % (comName, i), shell=True)


newEnv["MGWOO_CORE_ECO_DRV_DEF"] = design + "_core_eco_4.def"
newEnv["MGWOO_CORE_ECO_DRV_RPT"] = design + "_core_eco_5.rpt"

eclairOutput = "eclair_output_5"
rptName = design + "_core_eco_5.rpt"
newEnv["MGWOO_ECLAIR_OUTPUT"] = eclairOutput
sp.call("cd %s && innovus -init gen_drv.tcl | tee drv_run_5.log" % (comName), 
    env=newEnv, shell=True)

# the eclair must be in centos7 machine 
f = open("%s/eclair_run.sh" %(comName), 'w')
f.write("MGWOO_ECLAIR_OUTPUT=%s MGWOO_CORE_ECO_DRV_RPT=%s eclair cafe pm_core_eco_common.py --num-workers 2 --verbose | tee eclair_run_5.log" % (eclairOutput, rptName))
f.close()

sp.call("cd %s && /path/to/gnu/parallel --bar \
  --env PATH --env LM_LICENSE_FILE --env LD_LIBRARY_PATH \
  --env PYTHONPATH \
  --sshloginfile node_eclair.txt --workdir $PWD < eclair_run.sh" % (comName), shell=True, env=comEnv)

sp.call("cd %s && python3 clearme.py" %(comName), shell=True)
sp.call("cd %s && chmod a+x ./clear.sh" %(comName), shell=True)
sp.call("cd %s && ./clear.sh" %(comName), shell=True)

