###############################################################################
## BSD 3-Clause License
##
## Copyright (c) 2022, The Regents of the University of California
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and#or other materials provided with the distribution.
##
## * Neither the name of the copyright holder nor the names of its
##   contributors may be used to endorse or promote products derived from
##   this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE
## DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
## FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
## DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
## SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
## CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
## OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

sta::define_cmd_args "core_eco" {\
    [-swbox_solve_range box]\
    [-swbox_obs_range box]\
    [-metal_block_range range]\
    [-metal_route_range range]\
    [-swbox_local_step range]\
    [-swbox_local_count range]\
    [-set_timing_slack_threshold threshold]\
    [-drc_report file_name]\
    [-verbose level]\
}

proc core_eco { args } {
  sta::parse_key_args "core_eco" args \
    keys {
      -swbox_solve_range \
      -swbox_obs_range \
      -metal_block_range \
      -metal_route_range \
      -swbox_local_step \
      -swbox_local_count \
      -set_timing_slack_threshold \
      -drc_report \
      -write_z3 \
      -read_z3 \
      -verbose} \
    flags {
      -gui }

  if { [info exists keys(-swbox_solve_range)] } {
    set swbox_solve_range $keys(-swbox_solve_range)
    eco::set_swbox_solve_range_cmd [lindex $swbox_solve_range 0] [lindex $swbox_solve_range 1] 
  } 
  
  if { [info exists keys(-swbox_obs_range)] } {
    set swbox_obs_range $keys(-swbox_obs_range)
    eco::set_swbox_obs_range_cmd [lindex $swbox_obs_range 0] [lindex $swbox_obs_range 1]
  }

  if { [info exists keys(-metal_block_range)] } {
    set metal_block_range $keys(-metal_block_range)
    eco::set_metal_block_range_cmd [lindex $metal_block_range 0] [lindex $metal_block_range 1]
  }

  if { [info exists keys(-metal_route_range)] } {
    set metal_route_range $keys(-metal_route_range)
    eco::set_metal_route_range_cmd [lindex $metal_route_range 0] [lindex $metal_route_range 1]
  }

  if { [info exists keys(-swbox_local_step)] } {
    set local_step $keys(-swbox_local_step)
    eco::set_local_step_cmd [lindex $local_step 0] [lindex $local_step 1]
  }

  if { [info exists keys(-swbox_local_count)] } { 
    set local_cnt $keys(-swbox_local_count)
    eco::set_local_cnt_cmd [lindex $local_cnt 0] [lindex $local_cnt 1]
  }

  if { [info exists keys(-verbose)] } { 
    set verbose $keys(-verbose)
    eco::set_verbose_level_cmd $verbose
  }

  eco::set_gui_mode_cmd [info exists flags(-gui)]

  if {[info exists keys(-write_z3)]} {
    eco::set_write_z3_file_mode_cmd true
    eco::set_write_z3_file_dir_cmd $keys(-write_z3)
  }

  if {[info exists keys(-read_z3)]} {
    eco::set_read_z3_file_mode_cmd true
    eco::set_read_z3_file_dir_cmd $keys(-read_z3)
  }

  if { [ord::db_has_rows] } {
    sta::check_argc_eq0 "core_eco" $args
    eco::run_eco_cmd
    eco::reset_eco_cmd
  } else {
    utl::error ECO 150 "No rows defined in design. Use initialize_floorplan to add rows."
  }
}


namespace eval eco {
proc add_drc_coordinates { args } {
  sta::parse_key_args "add_drc_coordinates" args \
      keys {-x -y} \
      flags {}

  if { [info exists keys(-x)] && [info exists keys(-y)] } {
    eco::add_drc_coordinates_cmd $keys(-x) $keys(-y)
  } else {
    utl::error ECO 200 "Please input -x and -y keys to add drc coordinates"
  }
}


proc write_rects { args } {
  sta::parse_key_args "write_rects" args \
      keys { -file_name }
  set file_name $keys(-file_name)
  eco::write_rects_cmd $file_name
}

proc read_rects { args } { 
  sta::parse_key_args "read_rects" args \
      keys { -file_name }
  set file_name $keys(-file_name)
  eco::read_rects_cmd $file_name
}


proc gen_smt2_core_eco { args } {
  sta::parse_key_args "gen_smt2_core_eco" args \
      keys {\
      -metal_block_range \
      -metal_route_range \
      -swbox_solve_range \
      -swbox_obs_range \
      -cx -cy -orig -out_file } \
      flags {}

  if { [info exists keys(-swbox_solve_range)] } {
    set swbox_solve_range $keys(-swbox_solve_range)
    eco::set_swbox_solve_range_cmd [lindex $swbox_solve_range 0] [lindex $swbox_solve_range 1] 
  } 
  
  if { [info exists keys(-swbox_obs_range)] } {
    set swbox_obs_range $keys(-swbox_obs_range)
    eco::set_swbox_obs_range_cmd [lindex $swbox_obs_range 0] [lindex $swbox_obs_range 1]
  }

  if { [info exists keys(-metal_block_range)] } {
    set metal_block_range $keys(-metal_block_range)
    eco::set_metal_block_range_cmd [lindex $metal_block_range 0] [lindex $metal_block_range 1]
  }

  if { [info exists keys(-metal_route_range)] } {
    set metal_route_range $keys(-metal_route_range)
    eco::set_metal_route_range_cmd [lindex $metal_route_range 0] [lindex $metal_route_range 1]
  }

  if {[info exists keys(-out_file)]} {
    set out_file $keys(-out_file)
  } else {
    utl::error ECO 201 "Please set -out_file."
  }

  if {![info exists keys(-cx)] || ![info exists keys(-cy)] || ![info exists keys(-orig)]} {
    utl::error ECO 202 "Please set -cx -cy -orig." 
  }

  set orig_arr $keys(-orig)
  if {[llength $orig_arr] != 4} {
    utl::error ECO 203 "-orig must have four coordi (lx ly ux uy)." 
  }

  set orig_lx [lindex $orig_arr 0]
  set orig_ly [lindex $orig_arr 1]
  set orig_ux [lindex $orig_arr 2]
  set orig_uy [lindex $orig_arr 3]

  return [eco::generate_smt2_eco_cmd $out_file $keys(-cx) $keys(-cy) $orig_lx $orig_ly $orig_ux $orig_uy] 
}

proc read_smt2_core_eco { args } {
  sta::parse_key_args "read_smt2_core_eco" args \
      keys { -in_file } \
      flags {}

  if {[info exists keys(-in_file)]} {
    set in_file $keys(-in_file)
  } else {
    utl::error ECO 204 "Please set -in_file."
  }

  # extract core coordinate from helper file
  set fp [open "${in_file}.helper" "r"]
  set i 0
  while { [gets $fp line] >= 0 } {
    if {$i == 1} {
      set data [split $line]
      set lx [lindex $data 0]
      set ly [lindex $data 1]
      set ux [lindex $data 2]
      set uy [lindex $data 3]
      break
    }
    incr i
  }
  close $fp

  return [eco::read_smt2_eco_cmd ${in_file}.route $lx $ly $ux $uy]
}

}
