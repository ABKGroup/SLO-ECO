read_db $::env(MGWOO_RUN_DB)

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
    catch {exec -ignorestderr ./z3 smt.threads=1 timeout=100000 ${file_name} | tee ${file_name}.result} results
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

source $::env(MGWOO_DRC_TCL)

# source drc.tcl

# set drc_list {{32420 59760 32460 59840} {36240 62660 36240 62700} {45360 56340 45360 56380} {46480 58820 46480 58860} {44980 57360 45020 57440} {44980 59280 45020 59360} {46020 60160 46060 60320} {47460 60880 47500 61040} {61760 26980 61920 27020} {33540 21040 33580 21360} {34960 21860 34960 21900} {35440 23620 35440 23660} {34260 20800 34300 21120} {34500 24400 34540 24480} {36340 23040 36380 23040} {5020 8180 5060 8220} {6460 7380 6500 7420} {6540 6260 6580 6300} {29180 3060 29220 3100} {6300 7380 6340 7420} {29540 3500 29580 3540} {33020 23940 33060 23980} {3920 6020 3920 6060} {4560 6660 4720 6700} {4800 7780 4800 7820} {4960 6980 4960 7020} {5040 8180 5040 8220} {6320 7380 6320 7420} {6480 7380 6480 7420} {6560 6260 6560 6300} {7760 8820 7920 8860} {23760 5780 23840 5820} {28080 5380 28080 5420} {29040 2180 29120 2220} {29200 3060 29200 3100} {33040 23940 33040 23980} {5700 9200 5740 9280} {6020 8480 6060 8640} {28340 5360 28380 5360} {29540 3520 29580 3520} {32259 24640 32299 24960} {33140 20320 33180 20480} }

# set drc_list {{32420 59760 32460 59840} \
#   {36240 62660 36240 62700} \
#   {45360 56340 45360 56380} \
#   {46480 58820 46480 58860} \
#   {44980 57360 45020 57440} \
#   {44980 59280 45020 59360} \
#   {46020 60160 46060 60320} \
#   {47460 60880 47500 61040} \
#   {61760 26980 61920 27020} \
#   {33540 21040 33580 21360} \
#   {34960 21860 34960 21900} \
#   {35440 23620 35440 23660} \
#   {34260 20800 34300 21120} \
#   {34500 24400 34540 24480} \
#   {36340 23040 36380 23040} \
#   {5020 8180 5060 8220} \
#   {6460 7380 6500 7420} \
#   {6540 6260 6580 6300} \
#   {29180 3060 29220 3100} \
#   {6300 7380 6340 7420} \
#   {29540 3500 29580 3540} \
#   {33020 23940 33060 23980} \
#   {3920 6020 3920 6060} \
#   {4560 6660 4720 6700} \
#   {4800 7780 4800 7820} \
#   {4960 6980 4960 7020} \
#   {5040 8180 5040 8220} \
#   {6320 7380 6320 7420} \
#   {6480 7380 6480 7420} \
#   {6560 6260 6560 6300} \
#   {7760 8820 7920 8860} \
#   {23760 5780 23840 5820} \
#   {28080 5380 28080 5420} \
#   {29040 2180 29120 2220} \
#   {29200 3060 29200 3100} \
#   {33040 23940 33040 23980} \
#   {5700 9200 5740 9280} \
#   {6020 8480 6060 8640} \
#   {28340 5360 28380 5360} \
#   {29540 3520 29580 3520} \
#   {32259 24640 32299 24960} \
#   {33140 20320 33180 20480} }

# clear the running dir 
exec rm -rf $::env(MGWOO_TRIAL_DIR)/
exec mkdir -p $::env(MGWOO_TRIAL_DIR)/

set success_cnt 0

foreach xy $drc_list {
  set orig_lx [lindex $xy 0]
  set orig_ly [lindex $xy 1]
  set orig_ux [lindex $xy 2]
  set orig_uy [lindex $xy 3]

  set cx [expr int( ($orig_lx + $orig_ux)/2.0 )]
  set cy [expr int( ($orig_ly + $orig_uy)/2.0 )]

  set new_dir $::env(MGWOO_TRIAL_DIR)/${cx}_${cy}
  exec rm -rf $new_dir
  exec mkdir -p $new_dir

  set is_found false
  foreach mx_pitch [list 2 3 4 5 6 7 8 9] {
    set my_pitch $mx_pitch
    foreach dx [list -10 -9 -8 -7 -6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10] {
      foreach dy [list -10 -8 -6 -4 -2 0 2 4 6 8 10] {

        # site col/row
        set xdx [expr int($dx * 80)]
        set ydy [expr int($dy * 80)]

        set name_x [get_dxdy_name $xdx]
        set name_y [get_dxdy_name $ydy]

        set file_name $new_dir/${mx_pitch}_${my_pitch}_${name_x}_${name_y}.smt2

  
        # set trial [run_core_eco 45360 56360 $m_pitch $file_name]
        set trial [run_core_eco [expr int($cx + $xdx)] [expr int($cy + $ydy)] \
        [list $orig_lx $orig_ly $orig_ux $orig_uy] $mx_pitch $my_pitch $file_name]
        if { $trial == 1 } {
          puts "ROUTING SUCCESS! $cx $cy"
          eco::run_save_cur_rect_cmd
          set is_found true
          incr success_cnt
          break
        }
      }
      # end dy
      if {$is_found == true} {
        break
      }
    } 
    # end dx
    if {$is_found == true} {
      break
    }
  } 
  # end mx_pitch
}

puts "success cnt: $success_cnt"

set_debug_level ECO core_eco 10
eco::post_process_smt2_eco_cmd
# update pin access for next DB
# pin_access
write_db $::env(MGWOO_WRITE_DB) 
write_def $::env(MGWOO_WRITE_DEF)

exit


