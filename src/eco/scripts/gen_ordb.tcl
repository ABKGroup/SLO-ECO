read_lef $::env(MGWOO_TECH_LEF)

foreach lef $::env(MGWOO_CELL_LEFS) {
  read_lef $lef
}

read_def $::env(MGWOO_CORE_ECO_INIT_DEF) -continue_on_errors 
# pin_access
write_db $::env(MGWOO_CORE_ECO_INIT_DB)

exit
