read_db $::env(MGWOO_RUN_DB)

eco::read_rects -file_name $::env(MGWOO_RUN_RECTS)

proc run_core_eco { cx cy orig mx_pitch my_pitch file_name } {
  puts "eco::gen_smt2_core_eco -metal_route_range [list 2 3] "
  puts "  -swbox_solve_range [list [expr int(80*$mx_pitch + 40)] [expr int(80*$my_pitch + 40)]]"
  puts "  -swbox_obs_range [list [expr int(80*$mx_pitch + 360)] [expr int(80*$my_pitch + 360)]]"
  puts "  -cx $cx -cy $cy -orig $orig -out_file $file_name"

  set res [eco::gen_smt2_core_eco -metal_route_range [list 2 3] \
  -swbox_solve_range [list [expr int(80*$mx_pitch + 40)] [expr int(80*$my_pitch + 40)]] \
  -swbox_obs_range [list [expr int(80*$mx_pitch + 360)] [expr int(80*$my_pitch + 360)]] \
  -cx $cx -cy $cy -orig $orig -out_file $file_name]
  puts "gen_result: $res"
  if { $res == 0 } {
    return false
  }
  
  # catch {exec -ignorestderr ./z3 smt.threads=1 ${new_dir}/test.smt2 | tee /dev/tty} results
  puts "./z3 smt.threads=1 timeout=50000 ${file_name} | tee ${file_name}.result"

  # some error happened
  set i 0
  while {![file exists ${file_name}.result]} {
    catch {exec -ignorestderr ./z3 smt.threads=1 timeout=200000 ${file_name} | tee ${file_name}.result} results
    incr i
    if {$i >= 3} {
      return false
    }
  }

  set fp [open "${file_name}.result" "r"]
  set i 0
  # only get first line
  while { [gets $fp line] >= 0 } {
    if {$line == "unsat" || $line == "unknown"}  {
      close $fp
      return false
    }
    break
  }
  close $fp

  puts "./openroad -python runme.py $::env(MGWOO_RUN_DB) ${file_name}"
  catch {exec ./openroad -python runme.py $::env(MGWOO_RUN_DB) ${file_name}} results
  puts "$results"

  catch {exec ls ${file_name}.route} results
  if {[string first "No such file or directory" $results] != -1} {
    puts "SMT2 failed at generating route file"
    return false
  }

  puts "eco::read_smt2_core_eco -in_file ${file_name}"

  return [eco::read_smt2_core_eco -in_file ${file_name}]
}

proc get_dxdy_name { delta } {
  if {$delta < 0} {
    set pos [expr int($delta * -1)]
    return "n${pos}"
  } else {
    return "p${delta}"
  }
}


set file_name $::env(MGWOO_SINGLE_RUN).smt2
set names [split $::env(MGWOO_SINGLE_RUN) "_"]
set mx_pitch [lindex $names 0]
set my_pitch [lindex $names 1]

set drv_box_list [split $::env(MGWOO_DRV_BOX) "_"]
set drv_box_pnt [split $::env(MGWOO_DRV_RUN_PNT) "_"]

set drv_cx [lindex $drv_box_pnt 0]
set drv_cy [lindex $drv_box_pnt 1]

set trial [run_core_eco $drv_cx $drv_cy $drv_box_list $mx_pitch $my_pitch $file_name]
if { $trial == 1 } {
  puts "ROUTING SUCCESS!"
  eco::run_save_cur_rect_cmd
  
  set_debug_level ECO core_eco 10
  eco::post_process_smt2_eco_cmd
  # update pin access for next DB
  # pin_access
  write_db $::env(MGWOO_SINGLE_RUN).db 
  write_def $::env(MGWOO_SINGLE_RUN).def
  eco::write_rects -file_name $::env(MGWOO_SINGLE_RUN).rects
  exec touch $::env(MGWOO_SINGLE_RUN).done
}
exit
