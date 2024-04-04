loadLefFile $::env(MGWOO_TECH_LEF)
foreach lef $::env(MGWOO_CELL_LEFS) {
  loadLefFile $lef
}
loadDefFile $::env(MGWOO_CORE_ECO_DRV_DEF)

verify_drc -report $::env(MGWOO_CORE_ECO_DRV_RPT) -layer_range {M1 M3}
violationBrowserReport -all -no_display_false -report $::env(MGWOO_CORE_ECO_DRV_RPT).fororig

streamOut $::env(MGWOO_CORE_ECO_DRV_RPT).gds -unit 1000
exec mv streamOut.map $::env(MGWOO_CORE_ECO_DRV_RPT).gds.map

exit

#6T_2F_40CPP_40MP_3MPO_EL_M1.lef
# defOut -netlist issue5.test.def
