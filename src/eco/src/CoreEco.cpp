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

#include "odb/db.h"
#include "utl/Logger.h"

#include <tcl.h>
#include "sta/StaMain.hh"
#include "ord/OpenRoad.hh"
#include "eco/MakeCoreEco.h"
#include "eco/CoreEco.h"
#include "EcoBase.h"

#include <iostream>

namespace eco {

using utl::ECO;

CoreEco::CoreEco() 
  : db_(nullptr), sta_(nullptr), log_(nullptr),
  sw_box_solve_range_x_(0), sw_box_solve_range_y_(0),
  sw_box_obs_range_x_(0), sw_box_obs_range_y_(0),
  from_metal_block_layer_(0), to_metal_block_layer_(0),
  from_metal_route_layer_(0), to_metal_route_layer_(0),
  local_step_x_(0), local_step_y_(0), local_cnt_x_(0), local_cnt_y_(0),
  slack_threshold_(0.0f), 
  write_z3_file_mode_(false),
  read_z3_file_mode_(false),
  gui_mode_(false),
  verbose_(0), 
  write_z3_dir_name_("smt2"),
  read_z3_dir_name_("smt2"),
  rect_file_("") {} 

CoreEco::~CoreEco() {
  reset();
}

void 
CoreEco::reset() {
  drc_coordis_.clear();
  drc_coordis_.shrink_to_fit();
  eb_.reset();

  sw_box_solve_range_x_ = sw_box_solve_range_y_ = 0;
  sw_box_obs_range_x_ = sw_box_obs_range_y_ = 0;

  from_metal_block_layer_ = to_metal_block_layer_ = 0;
  from_metal_route_layer_ = to_metal_route_layer_ = 0;

  local_step_x_ = local_step_y_ = local_cnt_x_ = local_cnt_y_ = 0;
  slack_threshold_ = 0;

  write_z3_file_mode_ = false;
  read_z3_file_mode_ = false;
  gui_mode_ = false;
  verbose_ = 0;

  write_z3_dir_name_ = read_z3_dir_name_ = "smt2";
  rect_file_ = "";
}

void
CoreEco::loadVars() {
  if( db_ ) {
    odb::dbTech* tech = db_->getTech();
    if( tech ) {
  	  int maxLayer = tech->getRoutingLayerCount();
      if( maxLayer >= 4 ) {
        odb::dbTechLayer* m1 = tech->findRoutingLayer(1);
        odb::dbTechLayerDir m1Dir = m1->getDirection();

        odb::dbTechLayer* m2 = tech->findRoutingLayer(2);
        odb::dbTechLayerDir m2Dir = m2->getDirection();

        if( sw_box_solve_range_x_ == 0 ) {
          sw_box_solve_range_x_ = 
            (m1Dir == odb::dbTechLayerDir::HORIZONTAL)?
            m1->getPitchX() * 10.5 : m1->getPitchY() * 10.5;
        }

        if( sw_box_solve_range_y_ == 0 ) {
          sw_box_solve_range_y_ = 
            (m2Dir == odb::dbTechLayerDir::VERTICAL)?
            m2->getPitchY() * 10.5 : m2->getPitchX() * 10.5;
        }

        if( sw_box_obs_range_x_ == 0 ) {
          sw_box_obs_range_x_ = 
            (m1Dir == odb::dbTechLayerDir::HORIZONTAL)?
            m1->getPitchX() * 14.5 : m1->getPitchY() * 14.5; 
        }

        if( sw_box_obs_range_y_ == 0 ) {
          sw_box_obs_range_y_ = 
            (m2Dir == odb::dbTechLayerDir::VERTICAL)?
            m2->getPitchY() * 14.5 : m2->getPitchX() * 14.5;
        }

        if( from_metal_route_layer_ == 0 ) {
          from_metal_route_layer_ = 1;
        }

        if( to_metal_route_layer_ == 0 ) {
          to_metal_route_layer_ = 4;
        }

        if( from_metal_block_layer_ == 0 ) {
          from_metal_block_layer_ = std::min(5, maxLayer);
        }

        if( to_metal_block_layer_ == 0 ) {
          to_metal_block_layer_ = maxLayer;
        }
      }
    }
  }
}

void
CoreEco::init(odb::dbDatabase* odb,
    sta::dbSta* sta, 
    utl::Logger* log,
    int verbose) {
  db_ = odb;
  sta_ = sta;
  log_ = log;
  verbose_ = verbose;

  reset();
}

void
CoreEco::runCoreEco() {
  loadVars();

  EcoBaseVars ebVars;
  ebVars.swBoxSolveRangeX = sw_box_solve_range_x_;
  ebVars.swBoxSolveRangeY = sw_box_solve_range_y_;
  ebVars.swBoxObsRangeX = sw_box_obs_range_x_;
  ebVars.swBoxObsRangeY = sw_box_obs_range_y_;

  ebVars.fromMetalBlockLayer = from_metal_block_layer_;
  ebVars.toMetalBlockLayer = to_metal_block_layer_;

  ebVars.fromMetalRouteLayer = from_metal_route_layer_; 
  ebVars.toMetalRouteLayer = to_metal_route_layer_;

  ebVars.localStepX = local_step_x_;
  ebVars.localStepY = local_step_y_;

  ebVars.localCntX = local_cnt_x_;
  ebVars.localCntY = local_cnt_y_;

  ebVars.slackThreshold = slack_threshold_;
  ebVars.writeZ3FileMode = write_z3_file_mode_;
  ebVars.readZ3FileMode = read_z3_file_mode_;
  ebVars.guiMode = gui_mode_;

  ebVars.writeZ3DirName = write_z3_dir_name_;
  ebVars.readZ3DirName = read_z3_dir_name_;

  // initialize EcoBase
  eb_ = std::make_shared<EcoBase>(ebVars, db_, sta_, log_, verbose_); 

  eb_->setDrcCoordis(drc_coordis_);
  eb_->run();
}


bool 
CoreEco::runGenSMT2CoreEco(std::string fileName, 
    int cx, int cy, int orig_lx, int orig_ly, int orig_ux, int orig_uy) {
  loadVars();

  EcoBaseVars ebVars;
  ebVars.swBoxSolveRangeX = sw_box_solve_range_x_;
  ebVars.swBoxSolveRangeY = sw_box_solve_range_y_;
  ebVars.swBoxObsRangeX = sw_box_obs_range_x_;
  ebVars.swBoxObsRangeY = sw_box_obs_range_y_;

  ebVars.fromMetalBlockLayer = from_metal_block_layer_;
  ebVars.toMetalBlockLayer = to_metal_block_layer_;

  ebVars.fromMetalRouteLayer = from_metal_route_layer_; 
  ebVars.toMetalRouteLayer = to_metal_route_layer_;

  // initialize EcoBase
  if( !eb_ ) {
    eb_ = std::make_shared<EcoBase>(ebVars, db_, sta_, log_, verbose_); 
  }
  else {
    eb_->setEbVars(ebVars);
  }

  if( rect_file_ != "" ) {
    eb_->readRects(rect_file_);
  }
  return eb_->runGenSMT2(fileName, cx, cy, orig_lx, orig_ly, orig_ux, orig_uy);
}

bool 
CoreEco::runReadSMT2CoreEco(std::string fileName, 
    int lx, int ly, int ux, int uy) {
  loadVars();

  EcoBaseVars ebVars;
  ebVars.swBoxSolveRangeX = sw_box_solve_range_x_;
  ebVars.swBoxSolveRangeY = sw_box_solve_range_y_;
  ebVars.swBoxObsRangeX = sw_box_obs_range_x_;
  ebVars.swBoxObsRangeY = sw_box_obs_range_y_;

  ebVars.fromMetalBlockLayer = from_metal_block_layer_;
  ebVars.toMetalBlockLayer = to_metal_block_layer_;

  ebVars.fromMetalRouteLayer = from_metal_route_layer_; 
  ebVars.toMetalRouteLayer = to_metal_route_layer_;

  // initialize EcoBase
  if( !eb_ ) {
    eb_ = std::make_shared<EcoBase>(ebVars, db_, sta_, log_, verbose_); 
  }
  else {
    eb_->setEbVars(ebVars);
  }

  if( rect_file_ != "" ) {
    eb_->readRects(rect_file_);
  }

  return eb_->runReadSMT2(fileName, lx, ly, ux, uy);
}

void
CoreEco::runSaveCurRect() {
  if( eb_ ) {
    eb_->runSaveCurRect();
  }
}

void
CoreEco::writeRects(std::string fileName) {
  if( !eb_ ) {
    log_->error(ECO, 7520, "EcoBase was not initialized.") ;
  }
  eb_->writeRects(fileName);
}

void
CoreEco::readRects(std::string fileName) {
  rect_file_ = fileName;
}


void
CoreEco::runPostProcess() {
  if( !eb_ ) {
    log_->error(ECO, 7523, "EcoBase was not initialized.") ;
  }
  eb_->runPostProcessRectNets();
}

void
CoreEco::setSwitchBoxSolveRange(int width, int height) {
  sw_box_solve_range_x_ = width;
  sw_box_solve_range_y_ = height;
}

void
CoreEco::setSwitchBoxObsRange(int width, int height) {
  sw_box_obs_range_x_ = width;
  sw_box_obs_range_y_ = height;
}

void
CoreEco::setMetalBlockRange(int from, int to) {
  from_metal_block_layer_ = from;
  to_metal_block_layer_ = to;
}

void
CoreEco::setMetalRouteRange(int from, int to) {
  from_metal_route_layer_ = from;
  to_metal_route_layer_ = to;
}

void
CoreEco::setVerboseLevel(int verbose) {
  verbose_ = verbose;
  log_->setDebugLevel(ECO, "core_eco", verbose_);
}

void 
CoreEco::setWriteZ3FileMode(bool mode) {
  write_z3_file_mode_ = mode;
}

void
CoreEco::setReadZ3FileMode(bool mode) {
  read_z3_file_mode_ = mode;
}

void
CoreEco::setWriteZ3FileDir(std::string dirName) {
  write_z3_dir_name_ = dirName;
}

void
CoreEco::setReadZ3FileDir(std::string dirName) {
  read_z3_dir_name_ = dirName;
}

void
CoreEco::setLocalStep(int x, int y) {
  local_step_x_ = x;
  local_step_y_ = y;
}

void
CoreEco::setLocalCnt(int x, int y) {
  local_cnt_x_ = x;
  local_cnt_y_ = y;
}

void
CoreEco::setGuiMode(bool mode) {
  gui_mode_ = mode;
}

void
CoreEco::addDrcCoordi(int x, int y) {
  drc_coordis_.push_back(std::make_pair(x,y));
}

}
