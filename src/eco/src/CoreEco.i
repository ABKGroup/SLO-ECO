///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2022, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

%{
#include "ord/OpenRoad.hh"
#include "eco/CoreEco.h"

namespace ord {
OpenRoad*
getOpenRoad();

eco::CoreEco*
getCoreEco();

}

using ord::getOpenRoad;
using ord::getCoreEco;
using eco::CoreEco;

%}

%include "../../Exception.i"

%inline %{

namespace eco {

void 
reset_eco_cmd() 
{
  CoreEco* coreEco = getCoreEco();  
  coreEco->reset();
  coreEco->init(getOpenRoad()->getDb(),
    getOpenRoad()->getSta(),
    getOpenRoad()->getLogger());
}

void 
run_eco_cmd()
{
  CoreEco* coreEco = getCoreEco();
  coreEco->runCoreEco();
}

bool
generate_smt2_eco_cmd(const char* fileName, int cx, int cy, int orig_lx, int orig_ly, int orig_ux, int orig_uy)
{
  CoreEco* coreEco = getCoreEco();
  return coreEco->runGenSMT2CoreEco(fileName, cx, cy, orig_lx, orig_ly, orig_ux, orig_uy);
}

bool
read_smt2_eco_cmd(const char* fileName, int lx, int ly, int ux, int uy)
{
  CoreEco* coreEco = getCoreEco();
  return coreEco->runReadSMT2CoreEco(fileName, lx, ly, ux, uy);
}

void
run_save_cur_rect_cmd() 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->runSaveCurRect();
}

void
post_process_smt2_eco_cmd()
{
  CoreEco* coreEco = getCoreEco();
  coreEco->runPostProcess();
}

void
set_verbose_level_cmd(int verbose)
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setVerboseLevel(verbose);
}

void
add_drc_coordinates_cmd(int x, int y)
{
  CoreEco* coreEco = getCoreEco();
  coreEco->addDrcCoordi(x, y);
}

void 
set_gui_mode_cmd(bool mode)
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setGuiMode(mode);
}

void
set_write_z3_file_mode_cmd(bool mode)
{ 
  CoreEco* coreEco = getCoreEco();
  coreEco->setWriteZ3FileMode(mode);
}

void
set_write_z3_file_dir_cmd(const char* name)
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setWriteZ3FileDir(name);
}

void
write_rects_cmd(const char* name) 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->writeRects(name);
}

void
read_rects_cmd(const char* name) 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->readRects(name);
}

void
set_read_z3_file_mode_cmd(bool mode)
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setReadZ3FileMode(mode);
}

void
set_read_z3_file_dir_cmd(const char* name)
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setReadZ3FileDir(name);
}

void
set_metal_route_range_cmd(int lower, int upper) 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setMetalRouteRange(lower, upper);
}

void
set_metal_block_range_cmd(int lower, int upper) 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setMetalBlockRange(lower, upper);
}

void
set_swbox_obs_range_cmd(int width, int height) 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setSwitchBoxObsRange(width, height);
}

void
set_swbox_solve_range_cmd(int width, int height) 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setSwitchBoxSolveRange(width, height);
} 

void
set_local_step_cmd(int x, int y) 
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setLocalStep(x,y);
}

void
set_local_cnt_cmd(int x, int y)
{
  CoreEco* coreEco = getCoreEco();
  coreEco->setLocalCnt(x,y); 
}

}

%} // inline
