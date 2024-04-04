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
#include "odb/dbWireGraph.h"
#include "odb/dbWireCodec.h"
#include "odb/dbObject.h"
#include "odb/wOrder.h"
#include "utl/Logger.h"
#include "ord/OpenRoad.hh"

#include "eco/CoreEco.h"
#include "EcoBase.h"

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <set>
#include <string>
#include <unordered_map>

#include <iostream>
#include <fstream>
using std::cout;
using std::endl; 
using std::to_string;
using std::string;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// for easier coding with boost
typedef bg::model::point<int, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef bg::model::segment<point> segment;

// save point and dbiTerm* pointer
typedef std::pair<box, odb::dbITerm*> valueITerm;
typedef std::pair<box, odb::dbNet*> valueWire;
typedef bgi::rtree< valueITerm, bgi::quadratic<6> > ITermRTree;
typedef bgi::rtree< valueWire, bgi::quadratic<6> > WireRTree;

using std::vector;
using std::set;
using std::pair;
using std::make_pair;
using std::tuple;
using utl::ECO;

namespace eco {

static utl::Logger* logger = nullptr;

static OverlapMark 
getWireOverlapInfo(odb::Rect& wireRect, odb::Rect& solveRect);

static std::string
getString(eco::OverlapMark mark);

static std::string
getString(eco::ClipLayerPin::Direction dir); 

static std::string
getString(eco::ClipLayerPin::IO io); 

static bool 
isWithInBox(odb::dbBox* box, 
    odb::Rect& rect);

static bool 
isWithInBox(odb::Rect& rect1, 
    odb::Rect& rect2);

static std::string
getDebugString(eco::Clip::EcoNode& node);

static std::string
getDebugString(eco::Clip::EcoEdge& edge);

static std::string
getDebugString(eco::Clip::EcoPath& path);

static void
ParseDbBox(odb::dbBox* box, eco::Pin& curPin, utl::Logger* log);

static int itermLayerNum = 0;

static bool
isGoInside(eco::OverlapMark mark);

static bool
isGoOutside(eco::OverlapMark mark);

static bool
isInside(eco::OverlapMark mark);

static bool
isOutside(eco::OverlapMark mark);

static bool
is2DIntersect(eco::OverlapMark mark);

static bool
is3DIntersect(eco::OverlapMark mark);

static odb::dbWire* 
getNewWireForRects(odb::dbWire* prevWire, EcoOdbNet& odbNetInfo);

RoutedPoint::RoutedPoint(std::string input, odb::dbBlock* block) 
  : m_(0), r_(0), c_(0), 
  is_external_(false),
  iterm_(nullptr) {

  // parse the m/r/c segment 
  int result = sscanf(input.c_str(), 
      "m%dr%dc%d", &m_, &r_, &c_);
  // successful
  if( result != 0 ) {
    return;
  }

  // try ep_
  result = sscanf(input.c_str(), 
      "ep_m%dr%dc%d", &m_, &r_, &c_);
  
  // successful (external pin)
  if( result != 0 ) {
    is_external_ = true;
    return;
  }

  std::stringstream ss(input);

  std::stringstream ss2(input);
  std::string tmp;
  std::vector<std::string> splitted;
  while( getline(ss2, tmp, '_') ) {
    splitted.push_back(tmp);
  }

  if( splitted.size() <= 1 ) {
    logger->error(ECO, 4830, "Failed to parse"); 
  }


  std::string instName = "", mTermName = "";
  for(size_t i=1; i<splitted.size()-2; i++) {
    instName += splitted[i] + "_";
  }
  instName += splitted[splitted.size()-2];
  mTermName = splitted[splitted.size()-1];

  odb::dbInst* inst = block->findInst(instName.c_str());
  if( !inst ) {
    logger->error(ECO, 4828, "Cannot find instance: {}", 
        instName);
  }

  iterm_ = inst->findITerm(mTermName.c_str());
  if( !iterm_ ) { 
    logger->error(ECO, 4829, "Cannot find iTerm: {}/{}", 
        instName, mTermName);
  }

  cout << "retrieved: " << inst->getConstName() << "/" << iterm_->getMTerm()->getConstName() << endl;

  
  // // try it_(instName)_(pinName)
  // result = sscanf(input.c_str(), 
  //     "it_%s_%s", instStr, pinStr);
 
  // // successful (iterm) 
  // if( result != 0 ) {
  //   odb::dbInst* inst = block->findInst(instStr);
  //   if( !inst ) {
  //     logger->error(ECO, 432512345, "Cannot find instance: {}", 
  //         instStr);
  //   }

  //   iterm_ = inst->findITerm(pinStr);
  //   if( !iterm_ ) { 
  //     logger->error(ECO, 1234123413, "Cannot find iTerm: {}/{}", 
  //         instStr, pinStr);
  //   }
  // }
  // else {
  // }
}

std::string
RoutedPoint::string() const {
  if( m_ != 0 || r_ != 0 || c_ != 0 ) {
    if( is_external_ ) {
      return "external_m" + to_string(m_) 
        + "r" + to_string(r_) 
        + "c" + to_string(c_);
    }
    else {
      return "m" + to_string(m_) 
        + "r" + to_string(r_) 
        + "c" + to_string(c_);
    }
  }
  else if ( iterm_ ) {
    return "iterm_" + std::string(iterm_->getInst()->getConstName()) 
      + "/" + std::string(iterm_->getMTerm()->getConstName());
  }

  return "";
}

RNet::RNet(odb::dbNet* net) 
: net_(net), net_idx_(0), nets_idx_(0), 
  is_single_comm_net_(false),
  need_special_handling_(false) {}


void
RNet::addPin(ClipLayerPin* clPin) {
  // input pin case
  if( clPin->io() == ClipLayerPin::IO::INPUT) {
    // allow multiple call
    if( ipins_map_.find(clPin) == ipins_map_.end() ) {
      ipins_.push_back(clPin);
      ipins_map_.insert(clPin);
      pins_.push_back(clPin);
    }
  }
  else if( clPin->io() == ClipLayerPin::IO::OUTPUT) {
    opins_.push_back(clPin);
    pins_.push_back(clPin);
  }
  else {
    logger->error(ECO, 13, "Received IO NONE pins in RNet::addPin");
  }
}

void
RNet::setIdx(int netIdx) {
  net_idx_ = netIdx;
}

std::string
RNet::getName() const {
  if( net_idx_ == 0 ) {
    return net_->getConstName();
  }
  else {
    return std::string(net_->getConstName()) + "_P" + std::to_string(net_idx_);
  }
}

void
RNet::setNetsIdx(int netsIdx) {
  nets_idx_ = netsIdx;
}

void
RNet::print() {
  debugPrint(logger, ECO, "core_eco", 6, "RNet from RNet: {}, Net: {}", 
      getName(),
      net_->getConstName());
  for(auto& oPin : opins_) {
    debugPrint(logger, ECO, "core_eco", 6, "  outPin: {} {} {}",
      oPin->x(), oPin->y(), oPin->layer()->getConstName());
  }

  for(auto& iPin : ipins_) {
    debugPrint(logger, ECO, "core_eco", 6, "  inPin: {} {} {}",
      iPin->x(), iPin->y(), iPin->layer()->getConstName());
  }
}

void 
RNet::buildCommodityIndices() {
  if( pins().size() == 0 ) {
    logger->warn(ECO, 141, "Found pin size == 0. skip this net: {}", 
        getName());
    return;
  }

  ClipLayerPin* fromPin = nullptr;
  std::vector<ClipLayerPin*> toPins;

  // outlier case
  // where opin didn't exist
  if( oPins().size() != 1 ) { 
    logger->warn(ECO, 142, "Found opin size != 1. Assume pin[0] as opin");
    if( pins().size() < 2 ) {
      logger->warn(ECO, 143, 
          "Found pin size == {}. skip this net: {}", 
          pins().size(), getName());
      return;
    }

    fromPin = pins()[0]; 

    // expected elem
    toPins.reserve(pins().size()-1);

    // populate toPins
    // exclude first elem
    bool isFirst = true;
    for(auto& pin : pins()) {
      // skip for first pin 
      if( isFirst ){
        isFirst = false;
        continue;
      }
      toPins.push_back(pin);
    }
  }
  // normal case
  else {
    fromPin = oPins()[0];
    toPins = iPins();
  }

  for(auto& toPin : toPins) {
    c_opins_.push_back(fromPin);
    c_ipins_.push_back(toPin);
  }
}

int 
RNet::getCommodityCnt() const {
  return c_opins_.size();
}

ClipLayerPin*
RNet::commodityOutPin(int idx) {
  return c_opins_[idx];
}

ClipLayerPin*
RNet::commodityInPin(int idx) {
  return c_ipins_[idx];
}

void
RNet::addRoutedPoints(std::vector<RoutedPoint> routedPoints) {
  routed_paths_.push_back(routedPoints);
}

void
RNet::addRoutedSteinerPoint(RoutedPoint point) {
  steiner_points_.push_back(point);
}

// needed for remove duplicated external pins.
// re fill the whole pins/ipins/opins
void
RNet::clearPins() {
  // clear vectors
  pins_.clear();
  ipins_.clear();
  opins_.clear();

  pins_.shrink_to_fit();
  ipins_.shrink_to_fit();
  opins_.shrink_to_fit();

  // clear map
  ipins_map_.clear();
}

void RNet::setSingleCommNet(bool mode) {
  is_single_comm_net_ = mode;
}

bool
RNet::isFeasible(int from, int to) {
  if( pins().size() <= 1 ) {
    return false;
  }

  if( pins().size() == 2 ) {
    ClipLayerPin* pin1 = pins()[0];
    ClipLayerPin* pin2 = pins()[1];

    // EXTPIN, same loc, but outside of metal routing layers
    if( pin1->x() == pin2->x() &&
        pin1->y() == pin2->y() &&
        pin1->layer() == pin2->layer() &&
        pin1->type() == pin2->type() &&
        (from > pin1->layer()->getRoutingLevel() ||
         to < pin1->layer()->getRoutingLevel()) &&
        pin1->type() == ClipLayerPin::ClipLayerPinType::EXTPIN ) {
      return false;
    }
    else {
      return true;
    }
  }
  return true;
}



ClipLayerPin::ClipLayerPin(
    ClipLayerPinType type,
    int x, int y, int dbuX, int dbuY,
    odb::dbTechLayer* layer) 
: type_(type), io_(IONONE), 
  iterm_(nullptr), bterm_(nullptr), 
  layer_(layer),
  dir_(SAME),
  x_(x), y_(y), 
  dbu_x_(dbuX), dbu_y_(dbuY),
  is_fixed_(0),
  is_obs_area_(0),
  has_obs_left_(0),
  has_obs_right_(0),
  has_obs_up_(0),
  has_obs_down_(0),
  has_obs_z_up_(0),
  has_obs_z_down_(0) {}

void
ClipLayerPin::setIo(IO io) {
  io_ = io;
}

void
ClipLayerPin::setIo(odb::dbIoType io) {
  switch(io) {
    case odb::dbIoType::INPUT:
      io_ = ClipLayerPin::IO::INPUT;
      break;
    case odb::dbIoType::OUTPUT:
      io_ = ClipLayerPin::IO::OUTPUT;
      break;
    default:
      logger->error(ECO, 1010, "Cannot find Io type");
      break;
  }
}

void
ClipLayerPin::setType(ClipLayerPinType type) {
  type_ = type; 
}

void
ClipLayerPin::setITerm(odb::dbITerm* iTerm) {
  iterm_ = iTerm;
}

void
ClipLayerPin::setBTerm(odb::dbBTerm* bTerm) {
  bterm_ = bTerm;
}

void
ClipLayerPin::setDbLayer(odb::dbTechLayer* layer) {
  layer_ = layer;
}

void
ClipLayerPin::setIsFixed(bool isFixed) {
  is_fixed_ = isFixed;
}

void
ClipLayerPin::setIsObsArea(bool isObsArea) {
  is_obs_area_ = isObsArea;
}

void
ClipLayerPin::setHasObsLeft(bool hasObs) {
  has_obs_left_ = hasObs;
}

void
ClipLayerPin::setHasObsRight(bool hasObs) {
  has_obs_right_ = hasObs;
}

void
ClipLayerPin::setHasObsUp(bool hasObs) {
  has_obs_up_ = hasObs;
}

void
ClipLayerPin::setHasObsDown(bool hasObs) {
  has_obs_down_ = hasObs;
}

void
ClipLayerPin::setHasObsZUp(bool hasObs) {
  has_obs_z_up_ = hasObs;
}
void
ClipLayerPin::setHasObsZDown(bool hasObs) {
  has_obs_z_down_ = hasObs;
}

void
ClipLayerPin::setDir(Direction dir) {
  dir_ = dir;
}

bool
ClipLayerPin::hasObs() const {
  return (hasObsLeft() || hasObsRight() || hasObsUp() || hasObsDown() || hasObsZUp() || hasObsZDown());
}

bool 
ClipLayerPin::isSameClPin(const ClipLayerPin* clPin) {
  if( this->x_ == clPin->x() &&
      this->y_ == clPin->y() &&
      this->dbu_x_ == clPin->dbuX() &&
      this->dbu_y_ == clPin->dbuY() &&
      this->layer_ == clPin->layer() &&
      this->type_ == clPin->type() &&
      this->io_ == clPin->io() &&
      this->iterm_ == clPin->iTerm() ) {
    return true;
  }
  return false;
}



// define new int (32bit) key 
//
// where
// 29-31 bit: ITERM/BTERM/EXTPIN
// 28 bit: isFixed
// 25-27 bit: I/O  (0/1)
// 17-24 bit: metal layer
// 9-16 bit: row idx 
// 1-8 bit: col idx

int
ClipLayerPin::getKey() const {
  int key = (type() << 29);
  key += (isFixed() << 28);
  key += (io() << 25);
  key += (layer()->getRoutingLevel() << 17);
  key += (y() << 9);
  key += (x() << 1);

  return key;
}

std::string
ClipLayerPin::getVertName() const {
  if( type_ == ClipLayerPinType::ITERM ) {
    return "it_" + iterm_->getInst()->getName() 
      + "_" + iterm_->getMTerm()->getName();
  }
  else if( type_ == ClipLayerPinType::EXTPIN ) {
    return "ep_" + getMrc();
  }
  return "WRONG_" + getMrc();
}

std::string
ClipLayerPin::getMrc() const {
  return "m"
    + to_string(layer_->getRoutingLevel())
    + "r" + to_string(y_)
    + "c" + to_string(x_);
}

std::string 
ClipLayerPin::getObsString() const {
  if( hasObs() == false ) {
    return "";
  }

  std::string ret = "";
  if( hasObsLeft() ) { ret += "LEFT "; }
  if( hasObsRight() ) { ret += "RIGHT "; }
  if( hasObsUp() ) { ret += "UP "; }
  if( hasObsDown() ) { ret += "DOWN "; }
  if( hasObsZUp() ) { ret += "ZUP "; }
  if( hasObsZDown() ) { ret += "ZDOWN "; }
  return ret;
}

ClipLayer::ClipLayer(
    odb::dbDatabase* db,
    odb::Rect die,
    odb::Rect core,
    odb::dbTechLayer* layer,
    odb::dbTechLayer* nextLayer)
  : db_(db), layer_(layer), 
  next_layer_(nextLayer), 
  die_(die), core_(core), is_failed_(false) {
  init();
}  

void
ClipLayer::init() {
  // reserve 100 CL backup pins
  backup_clpins_stor_.reserve(100);

  odb::dbBlock* block = db_->getChip()->getBlock();

  std::vector<int> gridX, gridY;

  // retrieve gridXY
  odb::dbTrackGrid* trackGrid = block->findTrackGrid(layer_);
  trackGrid->getGridX(gridX);
  trackGrid->getGridY(gridY);
  
  // update grid: grid_x_, grid_y_
  for(auto y: gridY) {
    if( die_.yMin() < y && y < die_.yMax() ) {
      grid_y_.push_back(y);
    }

    if( core_.yMin() == y || core_.yMax() == y ) {
      logger->warn(ECO, 35, "The core box boundaries (y) lie on the track. Please re-adjust");
      is_failed_ = true;
      return;
    }
  }

  for(auto x: gridX) {
    if( die_.xMin() < x && x < die_.xMax() ) {
      grid_x_.push_back(x);
    }
    if( core_.xMin() == x || core_.xMax() == x ) {
      logger->error(ECO, 36, "The core box boundaries (x) lie on the track. Please re-adjust");
      is_failed_ = true;
      return;
    }

  }

  int coreIdxXMin = INT_MAX, coreIdxYMin = INT_MAX;
  int coreIdxXMax = -1, coreIdxYMax = -1;

  // boundary indices extraction where laid inside core box!
  for(int i=0; i<grid_x_.size(); i++) {
    if( core_.xMin() < grid_x_[i] && grid_x_[i] < core_.xMax() ) {
      if( coreIdxXMin > i ) {
        coreIdxXMin = i;
      }
      if( coreIdxXMax < i ) {
        coreIdxXMax = i;
      }
    }
  }

  for(int i=0; i<grid_y_.size(); i++) {
    if( core_.yMin() < grid_y_[i] && grid_y_[i] < core_.yMax() ) {
      if( coreIdxYMin > i ) {
        coreIdxYMin = i;
      }
      if( coreIdxYMax < i ) {
        coreIdxYMax = i;
      }
    }
  }

  grid_core_idx_ = odb::Rect(coreIdxXMin, coreIdxYMin, 
      coreIdxXMax, coreIdxYMax);

  if( grid_core_idx_.xMin() == -1 ||
      grid_core_idx_.xMax() == INT_MAX || 
      grid_core_idx_.xMin() == grid_core_idx_.xMax() ) {
    logger->warn(ECO, 37, "The core idx extraction is faced with fatal errors (x-axis). "
       "Check box sizes again: {} {} {}",
       layer_->getConstName(), 
        grid_core_idx_.xMin(), 
        grid_core_idx_.xMax());
    is_failed_ = true;
    return;
  }

  if( grid_core_idx_.yMin() == -1 ||
      grid_core_idx_.yMax() == INT_MAX || 
      grid_core_idx_.yMin() == grid_core_idx_.yMax() ) {
    logger->warn(ECO, 38, "The core idx extraction is faced with fatal errors (y-axis). "
       "Check box sizes again: {} {} {}",
       layer_->getConstName(), 
        grid_core_idx_.yMin(), 
        grid_core_idx_.yMax());
    is_failed_ = true;
    return;
  }

  // initialize x index map
  // to enable query from dbu to idx
  for(auto& x: grid_x_) {
    int idxX = &x - &grid_x_[0];
    idx_x_map_[x] = idxX;

    debugPrint(logger, ECO, "core_eco", 5, "   clip_idx_x_ idx: {} coordi: {}", 
       idxX, x); 
  }
  for(auto& y: grid_y_) {
    int idxY = &y - &grid_y_[0];
    idx_y_map_[y] = idxY;

    debugPrint(logger, ECO, "core_eco", 5, "   clip_idx_y_ idx: {} coordi: {}", 
       idxY, y); 
  }

  // initialize vertex grid
  for(auto& y: grid_y_) {
    int idxY = &y - &grid_y_[0];
    
    vector<ClipLayerPin> yLayers; 
    for(auto& x: grid_x_) {
      int idxX = &x - &grid_x_[0];
      yLayers.push_back(
          ClipLayerPin(ClipLayerPin::ClipLayerPinType::EMPTY, 
            idxX, idxY, x, y, layer_)); 
    }
    
    grid_.push_back(yLayers);
  }

  // update obs_area_clpins_
  int numObsAreaClPins = 0;
  for(auto& gridY : grid_) {
    for(auto& clPin : gridY) {
      // outside of core boundary
      if( ! ( core_.xMin() <= clPin.dbuX() && 
            clPin.dbuX() <= core_.xMax() &&
            core_.yMin() <= clPin.dbuY() && 
            clPin.dbuY() <= core_.yMax() ) ) {
        clPin.setIsObsArea(true);
        numObsAreaClPins++;
      }
    }
  }
  
  logger->report("  layer {}:  numTrackX: {} -> {}, numTrackY: {} -> {}",
      layer_->getConstName(), 
      gridX.size(), grid_x_.size(),
      gridY.size(), grid_y_.size());
  logger->report("  total clip layer pins: {}, obs area clip layer pins: {}", 
      grid_y_.size() * grid_x_.size(), 
      numObsAreaClPins);

  if( grid_x_.size() == 0 ) {
    logger->warn(ECO, 31, "Detected numTrackX = 0. Please consider larger clip or exclude layer: {}",
        layer_->getConstName());
    is_failed_ = true;
    return;
  }

  if( grid_y_.size() == 0 ) {
    logger->warn(ECO, 32, "Detected numTrackY = 0. Please consider larger clip or exclude layer: {}",
        layer_->getConstName());
    is_failed_ = true;
    return;
  }
}

void
ClipLayer::addBackupClipLayerPin(ClipLayerPin clPin) {
  backup_clpins_stor_.reserve(100);

  if( backup_clpins_stor_.size() >= 100 ) {
    logger->error(ECO, 33, "exceed assumed size");
  }
  backup_clpins_stor_.push_back(clPin);
  backup_clpins_.push_back(
      &backup_clpins_stor_[backup_clpins_stor_.size()-1]);
}

std::vector<ClipLayerPin*> 
ClipLayer::allClipLayerPins() {
  std::vector<ClipLayerPin*> ret;
  ret.reserve(gridX().size() * gridY().size() 
      + backupClipLayerPins().size());

  // traverse all 2D clipLayer Grid
  for(auto &clPinsY : grid()) {
    for(auto &clPin : clPinsY) {
      ret.push_back(&clPin);
    }
  }

  for(auto& clPin : backupClipLayerPins()) {
    ret.push_back(clPin);
  }
  
  return ret;
}

ClipLayerPin*
ClipLayer::clipLayerPin(int x, int y, 
    ClipLayerPin::ClipLayerPinType type) {
  // grid boundary check
  if( x < 0 || x >= grid_x_.size() ) {
    return nullptr;
  }
  if( y < 0 || y >= grid_y_.size() ) {
    return nullptr;
  }

  // usual cases -- type is empty
  if( type == ClipLayerPin::ClipLayerPinType::EMPTY ) {
    return &(grid_[y][x]);
  }
  // special cases -- type is not empty
  else {
    if( grid_[y][x].type() == type ) {
      return &(grid_[y][x]);
    }
    // check for backup clip layer pins
    else {
      for(auto& clPin : backup_clpins_) {
        if( clPin->x() == x && clPin->y() == y &&
            clPin->type() == type ) {
          return clPin;
        }
      }
      return nullptr;
    }
  }
} 

static int 
getCloseVar(int var, std::vector<int>& grid) {
  if( var <= grid[0] ) {
    return grid[0];
  }
  if( var >= grid[grid.size()-1] ) {
    return grid[grid.size()-1];
  }

  int prevVar = 0, curVar = 0;

  for(int i=1; i<grid.size(); i++) {
    prevVar = grid[i-1]; 
    curVar = grid[i];
    if( prevVar <= var && var <= curVar ) {
      break;
    }
  }

  // return the var that is close to grid
  return (abs(prevVar - var) > abs(curVar - var))?
    curVar : prevVar;
}

ClipLayerPin*
ClipLayer::clipLayerPinDbu(int x, int y, bool allowNearest) {
  auto xPtr = idx_x_map_.find(x);
  if( xPtr == idx_x_map_.end() ) {
    if( allowNearest ) {
      int val = getCloseVar(x, grid_x_);
      logger->warn(ECO, 52, "Cannot find dbuX: {} -> changed to dbuX: {}", 
          x, val);
  
      xPtr = idx_x_map_.find(val);
    }
    else {
      return nullptr;
    }
  }
  int idxX = xPtr->second;

  auto yPtr = idx_y_map_.find(y);
  if( yPtr == idx_y_map_.end() ) { 
    if( allowNearest ) {
      int val = getCloseVar(y, grid_y_);
      logger->warn(ECO, 53, "Cannot find dbuY: {} -> changed to dbuY: {}", 
          y, val);
  
      yPtr = idx_y_map_.find(val);
    }
    else {
      return nullptr;
    }
  }
  int idxY = yPtr->second;
  
  return &(grid_[idxY][idxX]);
}

static std::string 
getOrientStr(odb::dbOrientType type) {
  if( type == odb::dbOrientType::R0 ) { 
    return " N";
  }
  else if( type == odb::dbOrientType::MX) { 
    return "FS";
  }
  else if( type == odb::dbOrientType::R180) {
    return " S";
  }
  else if( type == odb::dbOrientType::MY) { 
    return "FN";
  }
  else {
    return "ORIENT ERROR";
  }
}


// Dp Site def
DpSite::DpSite(odb::dbInst* inst, 
    bool isFixed,
    odb::dbOrientType orient, 
    int gLx, int gLy,
    int dbuLx, int dbuLy) 
  : inst_(inst), 
  is_fixed_(isFixed), 
  orient_(orient),
  glx_(gLx), gly_(gLy), 
  dbulx_(dbuLx), dbuly_(dbuLy) {}

std::string 
DpSite::printType() const {
  if( isEmpty() ) { 
    return "E";
  }
  else if( isFixed() ) { 
    return "F";
  }
  return "TYPE ERROR";
}

std::string
DpSite::printOrient() const {
  return getOrientStr(orient_);
}

void
DpSite::setInst(odb::dbInst* inst) {
  inst_ = inst;
}

void
DpSite::setFixed(bool isFixed) {
  is_fixed_ = isFixed;
}

bool
DpSite::isPlace() const {
  return isEmpty() && !isFixed();
}

Clip::Clip()
  : db_(nullptr),
 from_route_layer_(0),
 to_route_layer_(999),
 site_lower_x_(0), site_upper_x_(0), site_lower_y_(0), site_upper_y_(0),
 is_failed_(false),
 iterm_layer_(nullptr) {
 }



Clip::Clip(odb::dbDatabase* db,
    EcoBaseVars ebVars,
    odb::Rect die, 
    odb::Rect core,
    std::vector<odb::dbInst*>& instList,
    std::vector<odb::dbInst*>& movableInstList,
    std::vector<odb::dbInst*>& fixedInstList,
    std::vector<odb::dbNet*>& netList)
: die_(die), core_(core), db_(db),
  from_route_layer_(ebVars.fromMetalRouteLayer),
  to_route_layer_(ebVars.toMetalRouteLayer),
  site_lower_x_(0), site_upper_x_(0), site_lower_y_(0), site_upper_y_(0),
  is_failed_(false),
  iterm_layer_(nullptr),
  insts_(instList),
  movable_insts_(movableInstList),
  fixed_insts_(fixedInstList),
  nets_(netList) {
  // for isFixedInsts()
  for(auto& fixedInst : fixed_insts_) {
    fixed_insts_map_.insert(fixedInst);
  }
  initClipLayers(ebVars);
  // handling clip layer failure
  if( is_failed_ ) {
    return;
  }
  initDpGrid();
  initMasters();

  // reserve sizes to avoid dangling ptr with std::vector
  int edgeCnt = 0, nodeCnt = 0;
  for(auto& net : nets_) {
    odb::dbWire* wire = net->getWire();

    odb::dbWireGraph graph;
    graph.decode(wire);

    for(odb::dbWireGraph::edge_iterator 
        edgeIter = graph.begin_edges();
        edgeIter != graph.end_edges();
        edgeIter ++) {
      edgeCnt ++;
    }

    for(odb::dbWireGraph::node_iterator 
        nodeIter = graph.begin_nodes();
        nodeIter != graph.end_nodes();
        nodeIter ++) {
      nodeCnt ++;
    }
  }

  nodes_.reserve(std::max(static_cast<int>(nodeCnt*1.3), 200));
  edges_.reserve(std::max(static_cast<int>(edgeCnt*1.3), 200));
}

void
Clip::reset() {
  clip_layer_stor_.clear();
  clip_layer_stor_.shrink_to_fit();
  clip_layers_.clear();
  clip_layers_.shrink_to_fit();
  clip_layer_map_.clear();

  nodes_.clear();
  nodes_.shrink_to_fit();
  new_nodes_.clear();
  new_nodes_.shrink_to_fit();

  edges_.clear();
  edges_.shrink_to_fit();
  new_edges_.clear();
  new_edges_.shrink_to_fit();

  eco_paths_.clear();
  eco_paths_.shrink_to_fit();

  insts_.clear();
  insts_.shrink_to_fit();
  movable_insts_.clear();
  movable_insts_.shrink_to_fit();
  fixed_insts_.clear();
  fixed_insts_.shrink_to_fit();

  nets_.clear();
  nets_.shrink_to_fit();
  rnets_.clear();
  rnets_.shrink_to_fit();

  dp_grid_.clear();
  dp_grid_.shrink_to_fit();

  dp_grid_place_pairs_.clear();
  dp_grid_place_pairs_.shrink_to_fit();

  db_masters_.clear();
  db_masters_.shrink_to_fit();

  eco_master_stor_.clear();
  eco_master_stor_.shrink_to_fit();

  eco_masters_.clear();
  eco_masters_.shrink_to_fit();

  coordi_info_stor_.clear();
  coordi_info_stor_.shrink_to_fit();
}

void
Clip::setIsFailed(bool isFailed) {
  is_failed_ = isFailed;
}

void 
Clip::setEcoPaths(std::vector<std::vector<std::vector<eco::Clip::EcoPath>>>& ecoPaths) {
  eco_paths_ = ecoPaths;
}

eco::ClipLayer* 
Clip::dbToEb(odb::dbTechLayer* layer) const {
  auto cmPtr = clip_layer_map_.find(layer);
  if( cmPtr == clip_layer_map_.end() ) {
    logger->error(ECO, 45, 
        "Cannot find layer {} in clip_layer_map_", 
       layer->getConstName()); 
  }
  return cmPtr->second;
}

eco::ClipLayer*
Clip::dbToEb(int layerNum) const {
  odb::dbTechLayer* layer = db_->getTech()->findRoutingLayer(layerNum);
  auto cmPtr = clip_layer_map_.find(layer);
  if( cmPtr == clip_layer_map_.end() ) {
    logger->error(ECO, 46, 
        "Cannot find layer {} in clip_layer_map_", 
       layer->getConstName()); 
  }
  return cmPtr->second;
}

void 
Clip::initClipLayers(EcoBaseVars ebVars) {
  odb::dbTech* tech = db_->getTech();
  odb::dbBlock* block = db_->getChip()->getBlock();

  // below / upper metals
  for(int i=ebVars.fromMetalRouteLayer-1; i<=ebVars.toMetalRouteLayer+1; i++) {
    odb::dbTechLayer* layer = tech->findRoutingLayer(i);
    odb::dbTechLayer* nextLayer = tech->findRoutingLayer(i+1);

    ClipLayer cLayer(db_, die_, core_, layer, nextLayer);
    if( cLayer.isFailed() ) {
      is_failed_ = true;
      return;
    }
    clip_layer_stor_.push_back(cLayer);
  }

  clip_layer_map_.clear();
  for(auto& clipLayer: clip_layer_stor_) {
    clip_layers_.push_back(&clipLayer);
    clip_layer_map_[clipLayer.layer()] = &clipLayer;

    // iterm_layer_ set
    // TODO: read opendb tech and determine it
    if( clipLayer.layer()->getRoutingLevel() == 1 ) {
      iterm_layer_ = &clipLayer;
    }

    std::cout << clipLayer.layer()->getConstName() << " is initialized" << std::endl;
  }


  // Assumption: 
  // a) there is no M1 routing and all inst pin is lied on M1
  // b) M1-M2 pin via is therefore starting point of router/dbWireGraph
  // c) The dbWireGraph traverse will be compatible 
  //    with M1-M2 starting pin points. 
  //    - please take a look at RNet class structure
  
  for(auto& net : nets_) {
    odb::dbWire* wire = net->getWire();
    odb::dbWireGraph graph;
    graph.decode(wire);
    for(odb::dbWireGraph::edge_iterator edgeIter = graph.begin_edges();
        edgeIter != graph.end_edges();
        edgeIter ++) {

      odb::dbWireGraph::Edge* edge = *edgeIter;

      int x1=-1, y1=-1, x3=-1, y3=-1;
      edge->source()->xy(x1,y1);
      edge->target()->xy(x3,y3);

      // means VIA
      if( x1 == x3 && y1 == y3 ) {
        int from = edge->source()->layer()->getRoutingLevel();
        int to = edge->target()->layer()->getRoutingLevel();

        odb::dbITerm* iTerm = nullptr;

        // AccessPoint VIA
        // The from layer = M1 means ITERM must be existed
        if( from == 1 && to == 2 ) {
          odb::dbObject* obj = edge->source()->object();
          if (obj && obj->getObjName() == std::string("dbITerm")) {
            iTerm = odb::dbITerm::getITerm(block, obj->getId());
          }
        }
        else if (from == 2 && to == 1 ) {
          odb::dbObject* obj = edge->target()->object();
          if( obj && obj->getObjName() == std::string("dbITerm")) {
            iTerm = odb::dbITerm::getITerm(block, obj->getId());
          }
        }

        if( iTerm ) {
          odb::Rect box(x1,y1,x1,y1);

          // ignore AP outside of die_
          if( isWithInBox( box, die_ ) ) {
            odb::dbTechLayer* m2 = db_->getTech()->findRoutingLayer(2);

            // All pin must be initialized at M2 AP
            ClipLayer* clipLayer = dbToEb(m2);
            ClipLayerPin* clPin = clipLayer->clipLayerPinDbu(x1, y1, false);

            if( clPin ) {
              clPin->setIo(iTerm->getIoType());
              clPin->setType(ClipLayerPin::ClipLayerPinType::ITERM);
              clPin->setITerm(iTerm);
              if( isFixedInsts(iTerm->getInst()) ) {
                clPin->setIsFixed(true);
              }
              debugPrint(logger, ECO, "core_eco", 5, 
                  "  iterm update: type:{} io:{} x:{} y:{} dbuX:{} dbuY:{} iterm:{}/{}", 
                  clPin->type(), clPin->io(), clPin->x(), clPin->y(), clPin->dbuX(), clPin->dbuY(), 
                  iTerm->getInst()->getConstName(), iTerm->getMTerm()->getConstName());
            }
          }
        }
      }
    }
  }
}

bool
Clip::isFixedInsts(odb::dbInst* inst) {
  if( fixed_insts_map_.find(inst) != fixed_insts_map_.end() ) {
    return true;
  }
  return false;
}

static odb::dbOrientType 
getRowOrient(odb::dbSet<odb::dbRow>& rows, int y) {
  for(auto row : rows) {
    odb::Rect rowRect = row->getBBox();
    if( rowRect.yMin() == y ) {
      return row->getOrient();
    }
  }
  logger->error(ECO, 950, 
      "Cannot find any y coordinates within given rows {}", 
      y);
  return odb::dbOrientType::R0;
}

void 
Clip::initDpGrid() {
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();

  // retrieve row/site
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  // retrieve gridCntX and gridCntY
  odb::Rect coreBox = block->getCoreArea();
  site_lower_x_ = coreBox.xMin(), site_upper_x_ = coreBox.xMax();
  while(site_lower_x_ < core_.xMin()) {
    site_lower_x_ += site->getWidth();
  }
  while(site_upper_x_ > core_.xMax()) {
    site_upper_x_ -= site->getWidth();
  }

  // get the lower site Y inside Core Rect
  site_lower_y_ = coreBox.yMin(), site_upper_y_ = coreBox.yMax();
  // site_lower_y_ could be the same as core_.yMin()
  while(site_lower_y_ < core_.yMin()) {
    site_lower_y_ += site->getHeight(); 
  }

  // get the upper site Y inside Core Rect
  // site_upper_y_ could be the same as core_.yMax()
  while(site_upper_y_ > core_.yMax()) {
    site_upper_y_ -= site->getHeight();
  }

  // all possible x and y counts
  
  int gridCntX = 0, gridCntY = 0;
  if (site_upper_x_ > site_lower_x_ ) {
    gridCntX = (site_upper_x_ - site_lower_x_)/site->getWidth();
  }
  if (site_upper_y_ > site_lower_y_ ) { 
    gridCntY = (site_upper_y_ - site_lower_y_)/site->getHeight();
  }

  logger->info(ECO, 951, "Current core's site box: ({} {} - {} {}) (cnt: {} {})", 
      site_lower_x_, site_lower_y_, 
      site_upper_x_, site_upper_y_, 
      gridCntX, gridCntY);

  // clear
  if( dp_grid_.size() > 0 ) {
    dp_grid_.clear();
    dp_grid_.shrink_to_fit();
  }

  // initialize DP grid
  for(int i=0; i<gridCntY; i++) {
    std::vector<DpSite> y; 
    int curRowLy = site_lower_y_ + i * site->getHeight();
    odb::dbOrientType rowOrient = getRowOrient(rows, curRowLy);

    for(int j=0; j<gridCntX; j++) { 
      // odb::dbInst*, isFixed, orient, glx, gly
      y.push_back(DpSite(nullptr, false, 
            rowOrient, 
            j, i, 
            j * site->getWidth() + site_lower_x_,
            i * site->getHeight() + site_lower_y_));
    }
    dp_grid_.push_back(y);
  }

  for(auto& inst: fixed_insts_) { 
    // get grid (lx, ly, ux uy)
    odb::dbBox* iBox = inst->getBBox();
    
    int gLx = (iBox->xMin() - site_lower_x_) / static_cast<int>(site->getWidth());
    int gLy = (iBox->yMin() - site_lower_y_) / static_cast<int>(site->getHeight());
    int gUx = (iBox->xMax() - site_lower_x_) / static_cast<int>(site->getWidth());
    int gUy = (iBox->yMax() - site_lower_y_) / static_cast<int>(site->getHeight());

    // update fixed insts in 2d dp grid. 
    for( int j=std::max(gLy, 0); 
        j<std::min(gUy, gridCntY); j++) {
      for( int i=std::max(gLx, 0); 
          i< std::min(gUx, gridCntX); i++) {
        dp_grid_[j][i].setInst(inst);
        dp_grid_[j][i].setFixed(true);
      }
    }
  }

  // update dp_grid_place_pairs_ for SMT2 filling
  for(auto& y: dp_grid_) {
    std::vector<std::pair<int, int>> rowGridPlacePair;
    int startIdx = -1, endIdx = -1;

    for(int i=0; i<y.size()-1; i++) {  
      // begin case
      if(i == 0 && y[i].isPlace()) {
        startIdx = i;
      }
      if(!y[i].isPlace() && y[i+1].isPlace() ) {
        startIdx = i+1;
      }

      if(y[i].isPlace() && !y[i+1].isPlace() ) {
        endIdx = i;
        rowGridPlacePair.push_back(make_pair(startIdx, endIdx));
        startIdx = endIdx = -1;
      }

      // end case
      if(i+1 == y.size()-1 && y[i+1].isPlace() && startIdx != -1) {
        endIdx = i+1;
        rowGridPlacePair.push_back(make_pair(startIdx, endIdx));
        startIdx = endIdx = -1;
      }
    }
    dp_grid_place_pairs_.push_back(rowGridPlacePair);
  }
}

void
Clip::initMasters() {
  set<odb::dbMaster*> dbMasterMap;
  for(auto& inst : insts_) {
    dbMasterMap.insert(inst->getMaster());
  }

  // retrieve access points
  for(odb::dbMaster* master : dbMasterMap) {
    db_masters_.push_back(master);

    debugPrint(logger, ECO, "core_eco", 3, "initMasters: master {}", master->getConstName());
    eco::Master curMaster(master);

    for(odb::dbMTerm* mTerm : master->getMTerms()) {
      if( mTerm->getSigType() != odb::dbSigType::POWER &&
          mTerm->getSigType() != odb::dbSigType::GROUND &&
          mTerm->getSigType() != odb::dbSigType::CLOCK) {
        debugPrint(logger, ECO, "core_eco", 3, "initMasters:   mterm {}", mTerm->getConstName());

        // create Pin
        eco::Pin curPin(mTerm);

        for(odb::dbMPin* mPin : mTerm->getMPins()) {
          for(odb::dbBox* box : mPin->getGeometry()) {
            ParseDbBox(box, curPin, logger);
          }
        }
        curMaster.addPin(curPin);
      }
    }

    // obs parse
    for(odb::dbBox* box : master->getObstructions()) {
      debugPrint(logger, ECO, "core_eco", 3, "initMasters:   obs");
      eco::Pin curPin(nullptr);
      ParseDbBox(box, curPin, logger);
      curMaster.addPin(curPin);
    }
    eco_master_stor_.push_back(curMaster);
  }

  // init masters_, dbmaster_to_ebmaster_map_
  for(auto& master: eco_master_stor_) {
    eco_masters_.push_back(&master);
    db_to_eco_master_map_[master.dbMaster()] = &master;
  }

  // update eco::Pin* pointer
  for(auto& master : eco_masters_) {
    for(auto& pin : master->pins()) {
      for(eco::PinCoordi& coordi: pin.pinCoordis()) {
        coordi.setPin(&pin);
      }
      db_to_eco_pin_map_[pin.mTerm()] = &pin;
    }
  }
}

eco::Master*
Clip::dbToEb(odb::dbMaster* master) const {
  auto ptr = db_to_eco_master_map_.find(master);
  if( ptr == db_to_eco_master_map_.end() ) {
    return nullptr;
  }
  return ptr->second;
}

eco::Pin*
Clip::dbToEb(odb::dbMTerm* mterm) const {
  auto ptr = db_to_eco_pin_map_.find(mterm);
  if( ptr == db_to_eco_pin_map_.end() ) {
    return nullptr;
  }
  return ptr->second;
}

// return DBU
// takes wire segment and track coordinate
static std::pair<int, int> 
getLeftMark(odb::Rect rect, eco::ClipLayer* cLayer) {
  // don't care direction
  int y = rect.yMin();
  return std::make_pair(cLayer->gridX()[cLayer->gridCoreIdx().xMin()], y);
}

static std::pair<int, int>
getRightMark(odb::Rect rect, eco::ClipLayer* cLayer) {
  int y = rect.yMin();
  return std::make_pair(cLayer->gridX()[cLayer->gridCoreIdx().xMax()], y);
}

static std::pair<int, int>
getDownMark(odb::Rect rect, eco::ClipLayer* cLayer) {
  int x = rect.xMin();
  return std::make_pair(x, cLayer->gridY()[cLayer->gridCoreIdx().yMin()]);
}

static std::pair<int, int>
getUpMark(odb::Rect rect, eco::ClipLayer* cLayer) {
  int x = rect.xMin();
  return std::make_pair(x, cLayer->gridY()[cLayer->gridCoreIdx().yMax()]);
}

static bool
isWithInRouteLayer(odb::dbTechLayer* layer, EcoBaseVars& ebVars) {
  if( layer->getRoutingLevel() >= ebVars.fromMetalRouteLayer 
      && layer->getRoutingLevel() <= ebVars.toMetalRouteLayer ) {
    return true;
  }
  return false;
}

void 
Clip::updateEdgeMark(EcoBaseVars& ebVars, 
    EcoNode& prevNode, EcoNode& nextNode, 
    eco::OverlapMark& mark,
    odb::Rect rect) {

  // make wireRect
  odb::Rect wireRect;
  wireRect.set_xlo(prevNode.x);
  wireRect.set_ylo(prevNode.y);

  wireRect.set_xhi(nextNode.x);
  wireRect.set_yhi(nextNode.y);

  // means VIA segments
  if( prevNode.x == nextNode.x &&
      prevNode.y == nextNode.y ) {

    int fromLayer = prevNode.layer->getRoutingLevel();
    int toLayer = nextNode.layer->getRoutingLevel();

    // outside of routinglayer 
    if( (fromLayer < ebVars.fromMetalRouteLayer
          && toLayer < ebVars.fromMetalRouteLayer) ||
        (fromLayer > ebVars.toMetalRouteLayer 
         && toLayer > ebVars.toMetalRouteLayer) ) {
      mark = eco::OverlapMark::OUTSIDE;
    }
    // inside of routing layer
    else {
      // overlap with 2D rect
      if( isWithInBox( wireRect, rect ) ) {

        // VIA touch case - 3D axis in/out
        if( fromLayer == ebVars.fromMetalRouteLayer - 1
            && toLayer == ebVars.fromMetalRouteLayer ) { 
          mark = eco::OverlapMark::ZLOWERIN;
        }
        else if( fromLayer  == ebVars.fromMetalRouteLayer
            && toLayer == ebVars.fromMetalRouteLayer - 1 ) {
          mark = eco::OverlapMark::ZLOWEROUT;
        }
        else if( fromLayer == ebVars.toMetalRouteLayer
            && toLayer == ebVars.toMetalRouteLayer + 1) {
          mark = eco::OverlapMark::ZUPPEROUT;
        }
        else if( fromLayer == ebVars.toMetalRouteLayer +1
            && toLayer == ebVars.toMetalRouteLayer) {
          mark = eco::OverlapMark::ZUPPERIN;
        }
        // within routing layer range
        else {
          mark = eco::OverlapMark::INSIDE;
        }
      } 
      // not overlap with 2D rect
      else {
        mark = eco::OverlapMark::OUTSIDE;
      }
    }
  }
  // means 2D segments
  // (x1,y1)-(x3,y3), where layer is the same
  else {
    // within routing layer range
    // note that prevNode->layer == nextNode->layer
    if (isWithInRouteLayer(prevNode.layer, ebVars)) {
      // update wireMarks;
      mark = getWireOverlapInfo (wireRect, rect); 
    }
    // oustide of routing layer range 
    else {
      mark = eco::OverlapMark::OUTSIDE;
    }
  }
}

void
Clip::updateRNet(odb::dbNet* net, 
    std::unordered_map<std::pair<int, int>, int, PairHash>& rNetOPinMap, 
    std::vector<EcoPath>& path, 
    EcoBaseVars& ebVars) {

  eco::ClipLayerPin* oPin = nullptr;
  eco::ClipLayerPin* iPin = nullptr;
  RNet* curRNet = nullptr;

  using ClipLayerPinType = eco::ClipLayerPin::ClipLayerPinType;
  using IO = eco::ClipLayerPin::IO; 

  // update mark on every edge
  for(int i=0; i<path.size(); i++) {
    EcoPath* curPath = &path[i];
    if( curPath->edge ) { 
      EcoNode* prevNode = path[i-1].node;       
      EcoNode* nextNode = path[i+1].node;

      updateEdgeMark(ebVars, *prevNode, *nextNode, 
          curPath->edge->coreMark, 
          core_);
      updateEdgeMark(ebVars, *prevNode, *nextNode, 
          curPath->edge->dieMark, 
          die_);
    }
  }

  // avoid outlier case where
  // oPin is initialized, but iPin didn't initialized.
  // This happened when iPin is ITERM's M2 pin, etc. 
  // -> outlier
  bool isOpinInit = false, isIpinInit = false;

  // retrieve opin / ipin
  for(int i=0; i<path.size(); i++) {
    EcoPath* curPath = &path[i];
    if( curPath->node ) {
      debugPrint(logger, ECO, "core_eco", 5, 
        "    pathInfo: {} {}",
       i, getDebugString(*(curPath->node))); 
    }
    else if (curPath->edge) {
      debugPrint(logger, ECO, "core_eco", 5, 
        "    pathInfo: {} {}", 
        i, getDebugString(*(curPath->edge)));

      // skip for outside seg
      // no need to consider
      if( curPath->edge->coreMark == eco::OverlapMark::OUTSIDE ) {
        continue;
      }

      EcoNode* prevNode = path[i-1].node;       
      EcoNode* nextNode = path[i+1].node;

      std::pair<int, int> iPtr = {0,0}, oPtr = {0,0};

      // make wireRect
      odb::Rect wireRect;
      wireRect.set_xlo(prevNode->x);
      wireRect.set_ylo(prevNode->y);

      wireRect.set_xhi(nextNode->x);
      wireRect.set_yhi(nextNode->y);

      // be aware of iPin/oPin's VIA direction
      //
      // e.g., suppose M2-M3 are only allowed.
      // does iPin/oPin go to Z-DOWN (to M12-VIA) 
      // or Z-UP (to M34-VIA)
      // 
      // dir code:
      // -1 : goes to down metal (e.g., to M12 VIA and M1)
      // 0 : no change
      // 1 : goes to up metal (e.g., to M34 VIA and M4)
      //
      ClipLayerPin::Direction 
        iPinDir = ClipLayerPin::Direction::SAME, 
                oPinDir = ClipLayerPin::Direction::SAME;

      // segment edge
      if( prevNode->layer == nextNode->layer ) {
        ClipLayer* cLayer = dbToEb(prevNode->layer);

        // 2D intersect detection
        switch( curPath->edge->coreMark ) {
          // outpin hit case
          case OverlapMark::LEFTIN:
            oPtr = getLeftMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);
            break;

          case OverlapMark::RIGHTIN:
            oPtr = getRightMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);
            break;

          case OverlapMark::UPIN:
            oPtr = getUpMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);
            break;

          case OverlapMark::DOWNIN:
            oPtr = getDownMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);
            break;

            // inpin hit case
          case OverlapMark::LEFTOUT:
            iPtr = getLeftMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

          case OverlapMark::RIGHTOUT:
            iPtr = getRightMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

          case OverlapMark::UPOUT:
            iPtr = getUpMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

          case OverlapMark::DOWNOUT:
            iPtr = getDownMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

            // case where oPin and iPin happened simultaneously
          case OverlapMark::LEFTINRIGHTOUT:
            oPtr = getLeftMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);

            iPtr = getRightMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

          case OverlapMark::RIGHTINLEFTOUT:
            oPtr = getRightMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);

            iPtr = getLeftMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

          case OverlapMark::UPINDOWNOUT:
            oPtr = getUpMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);

            iPtr = getDownMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

          case OverlapMark::DOWNINUPOUT:
            oPtr = getDownMark(wireRect, cLayer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);

            iPtr = getUpMark(wireRect, cLayer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

          default:
            break;
        }
      }
      // VIA edge
      else {
        ClipLayer* cLayer = nullptr;
        // 3D VIA intersect detection
        switch( curPath->edge->coreMark ) {
          // outpin hit case
          case OverlapMark::ZLOWERIN:
            oPinDir = ClipLayerPin::Direction::UP;
            oPtr = std::make_pair(wireRect.xMin(), wireRect.yMin());
            cLayer = dbToEb(nextNode->layer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);
            break;
          case OverlapMark::ZUPPERIN:
            oPinDir = ClipLayerPin::Direction::DOWN;
            oPtr = std::make_pair(wireRect.xMin(), wireRect.yMin());
            cLayer = dbToEb(nextNode->layer);
            oPin = cLayer->clipLayerPinDbu(oPtr.first, oPtr.second);
            break;
          
          // inpin hit case
          case OverlapMark::ZLOWEROUT:
            iPinDir = ClipLayerPin::Direction::DOWN;
            iPtr = std::make_pair(wireRect.xMin(), wireRect.yMin());
            cLayer = dbToEb(prevNode->layer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;
          case OverlapMark::ZUPPEROUT:
            iPinDir = ClipLayerPin::Direction::UP;
            iPtr = std::make_pair(wireRect.xMin(), wireRect.yMin());
            cLayer = dbToEb(prevNode->layer);
            iPin = cLayer->clipLayerPinDbu(iPtr.first, iPtr.second);
            break;

        default:
          break;
        }
      }

      // both case
      // == go through case
      //
      if (iPin && oPin) {
        debugPrint(logger, ECO, "core_eco", 5, "      oiPin Created"); 

        // outlier case: it means the location of (Opin and Ipin) are the exactly same.
        // Need special procedure to escape this failure. 
        if( iPin == oPin ) {
          debugPrint(logger, ECO, "core_eco", 5, "      oiPin Found Corner cases"); 
          // create new input pin
          if( oPin->io() == IO::OUTPUT ) {
            ClipLayerPin clPin = ClipLayerPin(ClipLayerPinType::EXTPIN, 
                oPin->x(), oPin->y(),
                oPin->dbuX(), oPin->dbuY(),
                oPin->layer());
            ClipLayer* cLayer = dbToEb(oPin->layer());
            cLayer->addBackupClipLayerPin(clPin);
            iPin = cLayer->backupClipLayerPins().back();
            iPin->setIo(IO::INPUT);
            iPin->setDir(iPinDir);
          }
          // create new output pin
          else if( oPin->io() == IO::INPUT ) {
            // create new output pin
            ClipLayerPin clPin = ClipLayerPin(ClipLayerPinType::EXTPIN, 
                oPin->x(), oPin->y(),
                oPin->dbuX(), oPin->dbuY(),
                oPin->layer());

            ClipLayer* cLayer = dbToEb(oPin->layer());
            cLayer->addBackupClipLayerPin(clPin);
            oPin = cLayer->backupClipLayerPins().back();
            oPin->setIo(IO::OUTPUT);
            oPin->setDir(oPinDir);
          }
        }

        if( oPin->type() == ClipLayerPinType::EMPTY) {
          oPin->setIo(IO::OUTPUT);
          oPin->setType(ClipLayerPinType::EXTPIN);
          oPin->setDir(oPinDir);
        }

        debugPrint(logger, ECO, "core_eco", 5, "        oPin type:{} io:{} x:{} y:{} d:{}", 
            oPin->type(), oPin->io(), oPin->x(), oPin->y(), oPin->dir());


        auto rmPtr = rNetOPinMap.find(std::make_pair(oPtr.first, oPtr.second));

        // first time to see this output pin.
        // keep curRNet for future use
        if( rmPtr == rNetOPinMap.end() ) {
          // newly define RNet
          RNet rNet(net);
          rNet.addPin(oPin);

          rnets_.push_back(rNet);
          rNetOPinMap[oPtr] = rnets_.size()-1;
          curRNet = &rnets_[rnets_.size()-1];
        }
        else {
          curRNet = &rnets_[rmPtr->second];
        }

        if( iPin->type() == ClipLayerPinType::EMPTY) { 
          iPin->setIo(IO::INPUT);
          iPin->setType(ClipLayerPinType::EXTPIN);
          iPin->setDir(iPinDir);
        }

        debugPrint(logger, ECO, "core_eco", 5, "        iPin type:{} io:{} x:{} y:{} d:{}", 
            iPin->type(), iPin->io(), iPin->x(), iPin->y(), iPin->dir());

        curRNet->addPin(iPin);
        curPath->clPins.push_back(oPin);
        curPath->clPins.push_back(iPin);

        oPin = nullptr;
        iPin = nullptr;
        isOpinInit = isIpinInit = true;
      }
      // special outlier handling 
      // oPin is appeared first, and iterm, but 
      // has extpin and iterm at the sametime.
      //
      // oPin / iPin must be created twice
      else if( oPin && 
          oPin->type() == ClipLayerPinType::ITERM && 
          prevNode->iterm != oPin->iTerm() && 
          nextNode->iterm != oPin->iTerm()) {

        debugPrint(logger, ECO, "core_eco", 5, "      oiPin must be created (outlier)"); 

        ClipLayerPin* tmpPin = oPin;

        // the iPin must be oPin here (iPin is ITERM)
        iPin = oPin;
        
        // retrieve EXTPIN at the same location 
        // - This must be new outpin
        ClipLayer* cLayer = dbToEb(tmpPin->layer());
        oPin = cLayer->clipLayerPin(tmpPin->x(), tmpPin->y(), 
            ClipLayerPinType::EXTPIN);
                
        if( oPin == nullptr ) {
          debugPrint(logger, ECO, "core_eco", 5, "      oPin is null -> create and assign new EXTPIN"); 
          ClipLayerPin newPin(ClipLayerPinType::EXTPIN,
              tmpPin->x(), tmpPin->y(),
              tmpPin->dbuX(), tmpPin->dbuY(),
              tmpPin->layer());
          cLayer->addBackupClipLayerPin(newPin);

          // update oPin pointer
          oPin = cLayer->clipLayerPin(tmpPin->x(), tmpPin->y(), 
              ClipLayerPinType::EXTPIN); 

          // the direction is already set previously 
          oPin->setDir(oPinDir);
        }

        oPin->setIo(IO::OUTPUT);
        iPin->setIo(IO::INPUT);

        debugPrint(logger, ECO, "core_eco", 5, "        oPin type:{} io:{} x:{} y:{} d:{}", 
            oPin->type(), oPin->io(), oPin->x(), oPin->y(), oPin->dir());

        debugPrint(logger, ECO, "core_eco", 5, "        iPin type:{} io:{} x:{} y:{} d:{}", 
            iPin->type(), iPin->io(), iPin->x(), iPin->y(), iPin->dir());

        auto rmPtr = rNetOPinMap.find(std::make_pair(oPtr.first, oPtr.second));

        // first time to see this output pin.
        // keep curRNet for future use
        if( rmPtr == rNetOPinMap.end() ) {
          // newly define RNet
          RNet rNet(net);
          rNet.addPin(oPin);

          rnets_.push_back(rNet);
          rNetOPinMap[oPtr] = rnets_.size()-1;
          curRNet = &rnets_[rnets_.size()-1];
        }
        else {
          curRNet = &rnets_[rmPtr->second];
        }

        curRNet->addPin(iPin);
        curPath->clPins.push_back(oPin);
        curPath->clPins.push_back(iPin);

        oPin = nullptr;
        iPin = nullptr;
        isOpinInit = isIpinInit = true;
      }
      else if( oPin ) {
        debugPrint(logger, ECO, "core_eco", 5, "      oPin Created"); 
        // initialize when empty is found
        if( oPin->type() == ClipLayerPinType::EMPTY) { 
          oPin->setIo(IO::OUTPUT);
          oPin->setType(ClipLayerPinType::EXTPIN);
          oPin->setDir(oPinDir);
        }
        // iterm handling - the input ITERM pin can be "output" pin
        else if ( oPin->type() == ClipLayerPinType::ITERM ) {
          oPin->setIo(IO::OUTPUT);
        }
        debugPrint(logger, ECO, "core_eco", 5, "        oPin type:{} io:{} x:{} y:{} d:{}", 
            oPin->type(), oPin->io(), oPin->x(), oPin->y(), oPin->dir());

        if( oPin->type() == ClipLayerPinType::ITERM ) {
          debugPrint(logger, ECO, "core_eco", 5, "        oPin ITERM:{}/{} ", 
            oPin->iTerm()->getInst()->getConstName(), 
            oPin->iTerm()->getMTerm()->getConstName());
        }

        auto rmPtr = rNetOPinMap.find(std::make_pair(oPtr.first, oPtr.second));

        // first time to see this output pin.
        // keep curRNet for future use
        if( rmPtr == rNetOPinMap.end() ) {
          // newly define RNet
          RNet rNet(net);
          rNet.addPin(oPin);

          rnets_.push_back(rNet);
          rNetOPinMap[oPtr] = rnets_.size()-1;
          curRNet = &rnets_[rnets_.size()-1];
        }
        else {
          curRNet = &rnets_[rmPtr->second];
        }

        // reset oPin
        curPath->clPins.push_back(oPin);
        oPin = nullptr;
        
        // failure happened at previous output pin case !!!
        if( isOpinInit == true && isIpinInit == false ){ 
          logger->warn(ECO, 92, "The Opin found twice. Skip");
          is_failed_ = true; 
          break;
        }

        isOpinInit = true;
        isIpinInit = false;
      }
      else if( iPin ) {
        debugPrint(logger, ECO, "core_eco", 5, "      iPin Created"); 
        if( curRNet == nullptr ) { 
          logger->warn(ECO, 99, "The RNet is empty at iPin addition");
          is_failed_ = true;
          break;
        }

        if( iPin->type() == ClipLayerPinType::EMPTY) { 
          iPin->setIo(IO::INPUT);
          iPin->setType(ClipLayerPinType::EXTPIN);
          iPin->setDir(iPinDir);
        }
        // outlier cases - cut at at ITERM but has EXTPIN - 
        // can be determined is NOT end edge ( i != paths.size()-2 )
        // 
        // 10/05/23: added DOWN case must be avoided
        else if( iPin->type() != ClipLayerPinType::EXTPIN 
            && i != path.size()-2 ) {
          // need special handling in the future!
          curRNet->setSpecialHandling(true);
          if( iPinDir != ClipLayerPin::Direction::DOWN ) {
            debugPrint(logger, ECO, "core_eco", 5, "      iPin Found Corner cases"); 
            ClipLayerPin clPin = ClipLayerPin(ClipLayerPinType::EXTPIN, 
                iPin->x(), iPin->y(),
                iPin->dbuX(), iPin->dbuY(),
                iPin->layer());
            ClipLayer* cLayer = dbToEb(iPin->layer());
            cLayer->addBackupClipLayerPin(clPin);

            // update iPin pointer as new one
            iPin = cLayer->backupClipLayerPins().back();
            iPin->setIo(IO::INPUT);
            iPin->setType(ClipLayerPinType::EXTPIN);
            iPin->setDir(iPinDir);
          }
        }

        debugPrint(logger, ECO, "core_eco", 5, "        iPin mrc:{} type:{} io:{} x:{} y:{} d:{}", 
            iPin->getMrc(), iPin->type(), iPin->io(), iPin->x(), iPin->y(), iPin->dir());

        curRNet->addPin(iPin);
        curPath->clPins.push_back(iPin);
        iPin = nullptr;
        isIpinInit = true;
      }
    } // end if curPath->edge
    if( is_failed_ ) {
      break;
    }
  } // end for path

  // dangling opin case 
  // outlier case - mark the current clip as the failure
  if( isOpinInit == true && isIpinInit == false) {
    is_failed_ = true;
    logger->warn(ECO, 93, "The Opin found twice. Skip");
  }
}

void 
Clip::postProcessRNets() {
  updateObsClPin();
  removeDuplicatedPins();
  updateRNetNames();
  updateRNetSingleComm();
  updateRNetIsFixed();
  updateRNetDirectionLayer();
  updateRNetClPinAPs();
  buildCommodityIndices();
  updateCoordiInfo();
  updateRNetMap();
}

void
Clip::removeDuplicatedPins() {
  // for the compilcated reason, rnet may have duplicated external pins
  for(auto& rNet : rnets_ ) {
    std::map<int, ClipLayerPin*> clPinMap;
    for(auto& pin : rNet.pins()) {
      clPinMap[pin->getKey()] = pin;
    }

    rNet.clearPins();
    for(auto& clPinPair : clPinMap) {
      rNet.addPin(clPinPair.second);
    } 
  }
}

// update rnets_ duplicated definitions 
// so that RNet::getName() works properly for SMT2 gen
//
// Also update netsIdx for CoordiInfo + SMT2ClipLayerPin's key generation
void
Clip::updateRNetNames() {
  std::map<odb::dbNet*, int> netChecker;

  int netsIdx = 0;
  for(auto& rNet : rnets_ ){
    // update nets idx
    rNet.setNetsIdx(netsIdx);
    netsIdx++;

    auto ncPtr = netChecker.find(rNet.net());
    if( ncPtr == netChecker.end()) {
      netChecker.insert(std::make_pair(rNet.net(), 0));
    }
    else {
      // get next idx
      int idx = ncPtr->second + 1;

      // update next idx
      rNet.setIdx(idx);
      netChecker[rNet.net()] = idx;
    }
  }
}

// fix the single commodity case (outlier)
// need to define a new external pin for awareness
// at the same location
//
// However, routed metal wire could be skipped later 
// at readBackSMT2 func
//
void
Clip::updateRNetSingleComm() {
  for(auto& rNet : rnets_) {
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      // for future use - read back from SMT2 - could ignore errors
      rNet.setSingleCommNet(true);

      ClipLayerPin* clPin = rNet.pins()[0];
      ClipLayer* cLayer = nullptr;
      int x = 0, y = 0;

      logger->info(ECO, 1932, 
          "Single comm net found: {}, x:{} y:{} layer:{} dir:{} io:{} clType:{}", 
          rNet.getName(),
          clPin->x(),
          clPin->y(),
          clPin->layer()->getConstName(),
          getString(clPin->dir()),
          getString(clPin->io()),
          clPin->type()
          );

      // skip for outlier cases - EXTPIN cannot be considered.
      if( clPin->type() != ClipLayerPin::ClipLayerPinType::ITERM ) {
        is_failed_ = true;
        continue;
      }

      // create previous layer's pin
      if( clPin->dir() == ClipLayerPin::Direction::DOWN ) {
        ClipLayer* curClipLayer = dbToEb(clPin->layer());
        cLayer = getPrevClipLayer( curClipLayer );
        ClipLayerPin* prevClPin = getPrevLayerClPin( curClipLayer, 
            clPin->y(), clPin->x());
        y = prevClPin->y();
        x = prevClPin->x();
      }
      // create next layer's pin
      else if( clPin->dir() == ClipLayerPin::Direction::UP ) {
        ClipLayer* curClipLayer = dbToEb(clPin->layer());
        cLayer = getNextClipLayer( curClipLayer );
        ClipLayerPin* nextClPin = getNextLayerClPin( curClipLayer, 
            clPin->y(), clPin->x());
        y = nextClPin->y();
        x = nextClPin->x();
      }
      // same layer's pin
      else if( clPin->dir() == ClipLayerPin::Direction::SAME ) {
        cLayer = dbToEb(clPin->layer());
        x = clPin->x();
        y = clPin->y();
      } 

      ClipLayerPin newClPin(
          ClipLayerPin::ClipLayerPinType::EXTPIN, 
          x,
          y,
          clPin->dbuX(),
          clPin->dbuY(),
          clPin->layer());

      // this doesn't need to be modified further - same layer
      newClPin.setDir( ClipLayerPin::Direction::SAME );

      // if input exist -> output pin must exist
      if( clPin->io() == ClipLayerPin::IO::INPUT ) {
        newClPin.setIo(ClipLayerPin::IO::OUTPUT);
      }
      // if output exist -> input pin must exist
      else if ( clPin->io() == ClipLayerPin::IO::OUTPUT) {
        newClPin.setIo(ClipLayerPin::IO::INPUT);
      } 

      cLayer->addBackupClipLayerPin(newClPin);
      ClipLayerPin* newClPinPtr 
        = cLayer->backupClipLayerPins().back();

      rNet.addPin(newClPinPtr);

      // now insert cliplayerpin object inside eco_paths_!
      // This is needed to insert a new VIA after reading back the SMT2 solution
      //
      
      vector<vector<EcoPath>>& paths = getPath(rNet.net());

      for(auto& path : paths) {
        bool isContained = false;
        for(int i=0; i<path.size(); i++) {
          if(path[i].node && path[i].node->iterm == clPin->iTerm()) {
            isContained = true;
            break;
          }
        }
        
        if( isContained ) {
          // means output pin is created - search from front
          if( clPin->io() == ClipLayerPin::IO::INPUT ) {
            for(int i=0; i<path.size(); i++){
              int changeCnt = 0;
              for(auto& pin : path[i].clPins) { 
                if( pin == clPin ) {
                  changeCnt += 1;
                  pin = newClPinPtr;
                }
              } 

              if( changeCnt > 0 ) {
                debugPrint(logger, ECO, "core_eco", 6, "  found replacement (front)! i:{} changeCnt:{}", 
                      i, changeCnt);
                break;
              }
            }
          }
          // means input pin is created - search from back
          else if( clPin->io() == ClipLayerPin::IO::OUTPUT ) {
            for(int i=path.size()-1; i>=0; i--) {
              int changeCnt = 0;
              for(auto& pin : path[i].clPins) { 
                if( pin == clPin ) {
                  changeCnt += 1;
                  pin = newClPinPtr;
                }
              } 

              if( changeCnt > 0 ) {
                debugPrint(logger, ECO, "core_eco", 6, "  found replacement (back)! i:{} changeCnt:{}", 
                      i, changeCnt);
                break;
              }
            }
          }
        }
      } 
    }
  }
}

void
Clip::updateRNetIsFixed() {
  // if the direction goes down / up, it must be fixed
  // external pins must be fixed.
  for(auto& rNet : rnets_) {
    for(auto& pin : rNet.pins()) {
      if( pin->dir() == ClipLayerPin::Direction::DOWN ||
          pin->dir() == ClipLayerPin::Direction::UP ||
          pin->type() == ClipLayerPin::ClipLayerPinType::EXTPIN ) {
        pin->setIsFixed(true);
      }
    }
  }
}

void
Clip::updateRNetDirectionLayer() {
  // The ITERM must be M1, not M2 
  // transfer all M2 into M1 if ITERM is discovered in ClipLayerPin

  // in the same way,
  // if segment is ended at M2 and goes down, the ClipLayerPin must go to M1
  // if segment is ended at M3 and goes up, the ClipLayerPin must go to M4
 
  ClipLayer* m1cLayer = nullptr, *m4cLayer = nullptr;
  std::map<ClipLayerPin*, ClipLayerPin*> m2m1ClPinMap, m3m4ClPinMap;

  for(auto& clipLayer : clip_layers_ ) {
    if( clipLayer->layer()->getRoutingLevel() == 1 ) {
      m1cLayer = clipLayer;
    }

    if( clipLayer->layer()->getRoutingLevel() == 4 ) {
      m4cLayer = clipLayer;
    }
  }

  if( m1cLayer == nullptr ) {
    logger->error(ECO, 1092, 
        "Cannot find metal 1 in clip_layers_. "
        "Please double check your input metal ranges");
  }

  if( m4cLayer == nullptr ) {
    logger->error(ECO, 1093, 
        "Cannot find metal 4 in clip_layers_. "
        "Please double check your input metal ranges");
  }

  for(auto& clipLayer : clip_layers_ ) {
    if( clipLayer->layer()->getRoutingLevel() == 2 ) {
      // for all clip layer pins
      for(auto& clPin : clipLayer->allClipLayerPins()) {
        if( clPin->type() == ClipLayerPin::ClipLayerPinType::ITERM 
            || (clPin->type() == ClipLayerPin::ClipLayerPinType::EXTPIN 
                && clPin->isFixed()
                && clPin->dir() == ClipLayerPin::Direction::DOWN) ) {

          // retrieve corresponding M1 CL pin
          ClipLayerPin* m1clPin 
            = m1cLayer->clipLayerPinDbu( clPin->dbuX(), clPin->dbuY() ); 
          
          if( m1clPin ) {
            m1clPin->setType( clPin->type() );
            m1clPin->setIsFixed( clPin->isFixed() );

            if( clPin->type() == ClipLayerPin::ClipLayerPinType::ITERM ) {
              m1clPin->setITerm( clPin->iTerm() );
            }
          }
          else {
            logger->error(ECO, 1094,
                "Fatal error happened. Cannot find M1 pin at {} {}", clPin->dbuX(), clPin->dbuY());
          }

          m2m1ClPinMap[clPin] = m1clPin;

          // reset the M2 point
          clPin->setType( ClipLayerPin::ClipLayerPinType::EMPTY );
          clPin->setIsFixed( false );
          clPin->setITerm(nullptr);
        }
      }
    }
             
    if( clipLayer->layer()->getRoutingLevel() == 3 ) {
      // traverse all 2D clipLayer Grid
      for(auto& clPin : clipLayer->allClipLayerPins()) {
        if( clPin->type() == ClipLayerPin::ClipLayerPinType::ITERM 
            || (clPin->type() == ClipLayerPin::ClipLayerPinType::EXTPIN 
                && clPin->isFixed()
                && clPin->dir() == ClipLayerPin::Direction::UP) ) {

          // retrieve corresponding M1 CL pin
          ClipLayerPin* m4clPin 
            = m4cLayer->clipLayerPinDbu( clPin->dbuX(), clPin->dbuY() ); 
          
          if( m4clPin ) {
            m4clPin->setType( clPin->type() );
            m4clPin->setIsFixed( clPin->isFixed() );
          }
          else {
            logger->error(ECO, 1095,
                "Fatal error happened. Cannot find M4 pin at {} {}", clPin->dbuX(), clPin->dbuY());
          }

          m3m4ClPinMap[clPin] = m4clPin;

          // reset the M2 point
          clPin->setType( ClipLayerPin::ClipLayerPinType::EMPTY );
          clPin->setIsFixed( false );
        }
      }
    }
  }

  // update rNets_ pointer
  for(auto& rNet : rnets_) {
    // pin
    for(auto& pin : rNet.pins()) {
      auto ptr = m2m1ClPinMap.find(pin);
      // replace the m2 empty pin as m1 ITERM ClipLayerPin
      if( ptr != m2m1ClPinMap.end() ) {
        pin = ptr->second;
      }

      ptr = m3m4ClPinMap.find(pin);
      if( ptr != m3m4ClPinMap.end() ) {
        pin = ptr->second;
      }
    }

    // ipin
    for(auto& pin : rNet.iPins()) {
      auto ptr = m2m1ClPinMap.find(pin);
      if( ptr != m2m1ClPinMap.end() ) {
        pin = ptr->second;
      }

      ptr = m3m4ClPinMap.find(pin);
      if( ptr != m3m4ClPinMap.end() ) {
        pin = ptr->second;
      }
    }
    
    // opin
    for(auto& pin : rNet.oPins()) {
      auto ptr = m2m1ClPinMap.find(pin);
      if( ptr != m2m1ClPinMap.end() ) {
        pin = ptr->second;
      }

      ptr = m3m4ClPinMap.find(pin);
      if( ptr != m3m4ClPinMap.end() ) {
        pin = ptr->second;
      }
    }
  }

  // update eco_paths_ clPins pointer
  for(auto& netPaths : eco_paths_) {
    for(auto& paths : netPaths) {
      for(auto& path : paths) {
        for(auto& pin : path.clPins) {
          auto ptr = m2m1ClPinMap.find(pin);
          if( ptr != m2m1ClPinMap.end() ) {
            pin = ptr->second;
          }

          ptr = m3m4ClPinMap.find(pin);
          if( ptr != m3m4ClPinMap.end() ) {
            pin = ptr->second;
          }
        }
      }
    }
  }
}

void
Clip::updateRNetClPinAPs() {
  // retrieve row/site
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  for(auto& clPin : iterm_layer_->allClipLayerPins()) {
    // total access points
    // note that this int has 8bit shift for y and x coordinate
    //
    // x: 0-7 bits (max 256)
    // y: 8-15 bits (max 256)
    //
    std::set<int> totalAPs;

    if( clPin->type() == ClipLayerPin::ClipLayerPinType::ITERM ) {
      // for the fixed pins, consider the current inst location
      if( clPin->isFixed() ) {
        for(auto& ap : getFixedInstAccessPoints(clPin)) {
          totalAPs.insert(ap);
        }
      }
      else {
        odb::dbBox* iBox = clPin->iTerm()->getInst()->getBBox();
        int instSiteWidth = (iBox->getDX()) 
          / static_cast<int>(site->getWidth());

        // check if current instance can be placed
        // with in given y, [lx, ux]
        for(auto& dpPairsY : dp_grid_place_pairs_) {
          int y = &dpPairsY - &dp_grid_place_pairs_[0];
          for(auto& dpPairX : dpPairsY) {
            // can be placed, within this row (row width const)
            if( dpPairX.second - dpPairX.first + 1 >= instSiteWidth ) {
              // possible x loc
              for(int x=dpPairX.first; 
                  x<=dpPairX.second - instSiteWidth + 1; 
                  x++) {
                // all flip variable
                for(int f = 0; f<2; f++) {
                  for(auto& ap : getAccessPoints(clPin, y, x, f)) {
                    totalAPs.insert(ap);
                  } 
                } 
              }
            }  
          }
        } 
      }
    }
    
    std::vector<std::pair<int, int>> ret;
    // 111111111111..00000000
    // 1 <- 16 bits
    // 0 <- 16 bits
    int32_t markY = ~0 << 16;
    for(auto& ap : totalAPs) {
      int32_t y = (markY & ap) >> 16;
      int32_t x = (~markY & ap);
      ret.push_back(make_pair(y,x));
    }
    // set the total accesspoints as ret
    clPin->accessPointPairs() = ret; 
  }
}

void
Clip::buildCommodityIndices() {
  for(auto& rNet : rnets_) {
    rNet.buildCommodityIndices();
  }
}

void
Clip::updateCoordiInfoAddCoordiAPEP(
    ClipLayerPin* clPin, 
    RNet& rNet,
    int cIdx, 
    std::set<int32_t>& allKeys, 
    std::map<int32_t, int32_t>& tmpMap) {

  if( clPin->type() 
      == ClipLayerPin::ClipLayerPinType::ITERM ) {
    for(auto& ap: clPin->accessPointPairs()) {
      CoordiInfo curCoordi (itermLayerNum, 
          ap.first, ap.second, this);

      int32_t key = curCoordi.getKey();
      allKeys.insert(key);

      // add new coordi!
      auto ptr = tmpMap.find(key);
      if( ptr == tmpMap.end() ) {
        tmpMap[key] = coordi_info_stor_.size();
        curCoordi.addSMT2ClipLayerPin(
            SMT2ClipLayerPin(&rNet, cIdx, clPin));
        coordi_info_stor_.push_back(curCoordi);
      }
      else {
        coordi_info_stor_[ptr->second].addSMT2ClipLayerPin(
            SMT2ClipLayerPin(&rNet, cIdx, clPin));
      }

      // VIA12 consideration -- add SMT2 pin on upper layer's coordi info
      // get next layer's RC
      std::pair<int, int> newPair 
        = getNextLayerRowCol(iterm_layer_, ap.first, ap.second);

      // if exists
      if( newPair.first != -1 && newPair.second != -1) {
        // next layer's coordi
        CoordiInfo newCoordi (itermLayerNum+1, 
            newPair.first, newPair.second, this);

        int32_t key = newCoordi.getKey();
        allKeys.insert(key);

        // add new coordi!
        auto ptr = tmpMap.find(key);
        if( ptr == tmpMap.end() ) {
          tmpMap[key] = coordi_info_stor_.size();
          newCoordi.addSMT2ClipLayerPin(
              SMT2ClipLayerPin(&rNet, cIdx, clPin));
          coordi_info_stor_.push_back(newCoordi);
        }
        else {
          coordi_info_stor_[ptr->second].addSMT2ClipLayerPin(
              SMT2ClipLayerPin(&rNet, cIdx, clPin));
        }
      }
    }
  }
  else if( clPin->type() 
      == ClipLayerPin::ClipLayerPinType::EXTPIN 
      && clPin->isFixed() ) {

    CoordiInfo curCoordi (clPin->layer()->getRoutingLevel(), 
        clPin->y(), clPin->x(), this);

    int32_t key = curCoordi.getKey();
    allKeys.insert(key);

    auto ptr = tmpMap.find(key);
    if( ptr == tmpMap.end() ) {
      tmpMap[key] = coordi_info_stor_.size();
      curCoordi.addSMT2ClipLayerPin(
          SMT2ClipLayerPin(&rNet, cIdx, clPin));
      coordi_info_stor_.push_back(curCoordi);
    }
    else {
      coordi_info_stor_[ptr->second].addSMT2ClipLayerPin(
          SMT2ClipLayerPin(&rNet, cIdx, clPin));
    }


    // if EXT PIN is placed at ITERM, VIA must be defined
    if( clPin->layer()->getRoutingLevel() == itermLayerNum) {
      // VIA12 consideration -- add SMT2 pin on upper layer's coordi info
      // get next layer's RC
      std::pair<int, int> newPair 
        = getNextLayerRowCol(iterm_layer_, clPin->y(), clPin->x());

      // if exists
      if( newPair.first != -1 && newPair.second != -1) {
        // next layer's coordi
        CoordiInfo newCoordi (itermLayerNum+1, 
            newPair.first, newPair.second, this);

        int32_t key = newCoordi.getKey();
        allKeys.insert(key);

        // add new coordi!
        auto ptr = tmpMap.find(key);
        if( ptr == tmpMap.end() ) {
          tmpMap[key] = coordi_info_stor_.size();
          newCoordi.addSMT2ClipLayerPin(
              SMT2ClipLayerPin(&rNet, cIdx, clPin));
          coordi_info_stor_.push_back(newCoordi);
        }
        else {
          coordi_info_stor_[ptr->second].addSMT2ClipLayerPin(
              SMT2ClipLayerPin(&rNet, cIdx, clPin));
        }
      }
    }
  }
}

void
Clip::updateCoordiInfoAddCoordiAdj(
    ClipLayerPin* clPin1, ClipLayerPin* clPin2,
    std::set<int32_t>& allKeys,
    std::map<int32_t, int32_t>& tmpMap) {

  // no need to run the following logics 
  if( clPin1 == nullptr || clPin2 == nullptr ) {
    return;
  }

  // obs pin cannot be added
  if( (clPin1->isObsArea() && clPin1->hasObs()) ||
      (clPin2->isObsArea() && clPin2->hasObs()) ) {
    return;
  }

  CoordiInfo coordi1(clPin1->layer()->getRoutingLevel(), 
      clPin1->y(), clPin1->x(), this);

  CoordiInfo coordi2(clPin2->layer()->getRoutingLevel(),
      clPin2->y(), clPin2->x(), this);

  allKeys.insert(coordi1.getKey());
  allKeys.insert(coordi2.getKey());

  auto ptr = tmpMap.find(coordi1.getKey());  
  if( ptr == tmpMap.end()) {
    tmpMap[coordi1.getKey()] = coordi_info_stor_.size();
    coordi1.allAdjSegments()
      .push_back(make_pair(clPin1, clPin2));
    coordi_info_stor_.push_back(coordi1);
  }
  else {
    coordi_info_stor_[ptr->second].allAdjSegments()
      .push_back(make_pair(clPin1, clPin2));
  }

  ptr = tmpMap.find(coordi2.getKey());
  if( ptr == tmpMap.end() ) {
    tmpMap[coordi2.getKey()] = coordi_info_stor_.size();
    coordi2.allAdjSegments()
      .push_back(make_pair(clPin1, clPin2));
    coordi_info_stor_.push_back(coordi2);
  }
  else {
    coordi_info_stor_[ptr->second].allAdjSegments()
      .push_back(make_pair(clPin1, clPin2));
  }
}


void
Clip::updateCoordiInfo() {
  itermLayerNum = iterm_layer_->layer()->getRoutingLevel();

  // key: CoordiInfo->key()
  // value: coordi_info_stor_ index
  std::map<int32_t, int32_t> tmpMap; 
  std::set<int32_t> allKeys;

  // traverse the whole RNets structure and update all APs and EPs
  for(auto& rNet : rnets_) {
    // retrieve commodity variables from rNet
    int numCommCnt = rNet.getCommodityCnt();

    for(int i=0; i<numCommCnt; i++) {
      ClipLayerPin* fromPin = rNet.commodityOutPin(i);
      ClipLayerPin* toPin = rNet.commodityInPin(i);

      updateCoordiInfoAddCoordiAPEP(fromPin, 
          rNet, i, allKeys, tmpMap);

      updateCoordiInfoAddCoordiAPEP(toPin, 
          rNet, i, allKeys, tmpMap);
    }
  }

  // traverse the whole layer and add all adjacent segments
  // (no APs, no EPs)
  for(int layer=from_route_layer_-1; 
      layer<=to_route_layer_; layer++) {
    odb::dbTechLayer* dbLayer 
      = db_->getTech()->findRoutingLayer(layer);
    ClipLayer* clipLayer = dbToEb(dbLayer);
    ClipLayer* nextClipLayer 
      = dbToEb(db_->getTech()->findRoutingLayer(layer+1));

    std::vector<int>& gridX = clipLayer->gridX();
    std::vector<int>& gridY = clipLayer->gridY();

    // considers:
    // a) upper VIA (if exists)
    // b) either HOR/VERT two metal segments (+- 1)
    //
    if( layer != itermLayerNum ) {
      // a) upper VIA
      if( nextClipLayer ) {
        // (l, m, n) to (l+1, m, n) segment
        for(int m=0; m<gridY.size(); m++) {
          for(int n=0; n<gridX.size(); n++) {
            ClipLayerPin* lowClPin 
              = clipLayer->clipLayerPin(n,m);
            ClipLayerPin* highClPin 
              = getNextLayerClPin(clipLayer, m, n);

            updateCoordiInfoAddCoordiAdj(lowClPin, highClPin,
                allKeys, tmpMap);
          }
        }
      }

      // b-1) VERTICAL
      // (l, m, n) to (l, m+1, n) segment
      if(dbLayer->getDirection() 
        == odb::dbTechLayerDir::VERTICAL ) { 

        for(int m=0; m<gridY.size()-1; m++) {
          for(int n=0; n<gridX.size(); n++) {
            ClipLayerPin* lowClPin 
              = clipLayer->clipLayerPin(n,m);
            ClipLayerPin* highClPin 
              = clipLayer->clipLayerPin(n,m+1);

            updateCoordiInfoAddCoordiAdj(lowClPin, highClPin,
                allKeys, tmpMap);
          }
        }
      }
      // b-2) HORIZONTAL 
      // (l, m, n) to (l, m, n+1) segment
      else if(dbLayer->getDirection()
        == odb::dbTechLayerDir::HORIZONTAL ) {

        for(int m=0; m<gridY.size(); m++) {
          for(int n=0; n<gridX.size()-1; n++) {
            ClipLayerPin* lowClPin 
              = clipLayer->clipLayerPin(n,m);
            ClipLayerPin* highClPin 
              = clipLayer->clipLayerPin(n+1,m);

            updateCoordiInfoAddCoordiAdj(lowClPin, highClPin,
                allKeys, tmpMap);
          }
        }
      }
    }
  }

  // now coordi_info_stor_'s pointer is finalized 
  // update coordi_to_clip_layer_pins_ map
  coordi_info_map_.clear();
  for(auto& key : allKeys) {
    auto ptr = tmpMap.find(key);
    if( ptr == tmpMap.end() ) {
      logger->error(ECO, 179, "wrong map initialization");
    }

    coordi_info_map_[key] = &(coordi_info_stor_[ptr->second]);
  }
}

void
Clip::updateRNetMap() {
  rnet_map_.clear();
  for(auto& rNet : rnets_) {
    rnet_map_[rNet.getName()] = &rNet;
  }
}

void
Clip::updateObsClPin() {
  // update eco_paths_ clPins pointer
  for(auto& netPaths : eco_paths_) {
    for(auto& paths : netPaths) {
      for(int i=0; i<paths.size(); i++) {
        debugPrint(logger, ECO, "core_eco", 5, " obsClPin: {}",
            getDebugString(paths[i]));

        if(paths[i].edge) {
          eco::OverlapMark dieMark = paths[i].edge->dieMark;
          eco::OverlapMark coreMark = paths[i].edge->coreMark;


          // OBS happened only with the following cases:
          // 1. intersect 3D
          if (is3DIntersect(dieMark) && !is3DIntersect(coreMark)) {
            EcoNode* node = nullptr;

            switch(dieMark) {
              // current pin
              case eco::OverlapMark::ZLOWEROUT:
              case eco::OverlapMark::ZUPPEROUT:
                node = paths[i-1].node;
                break;

                // next pin
              case eco::OverlapMark::ZLOWERIN:
              case eco::OverlapMark::ZUPPERIN:
                node = paths[i+1].node;
                break;

              default:
                break;
            }

            // node is not null - 3D intersect cases
            if( node != nullptr ) {
              ClipLayer* layer = dbToEb(node->layer);
              ClipLayerPin* clPin 
                = layer->clipLayerPinDbu(node->x, node->y, true);
              if( clPin ) {
                switch(dieMark) {
                  case eco::OverlapMark::ZLOWERIN:
                  case eco::OverlapMark::ZLOWEROUT:
                    clPin->setHasObsZDown(true);
                    break;
                  case eco::OverlapMark::ZUPPERIN:
                  case eco::OverlapMark::ZUPPEROUT:
                    clPin->setHasObsZUp(true);
                    break;
                  default:
                    break;
                }
              }
            }
          }
          // 2. intersect 2D
          else if (is2DIntersect(dieMark) ||
              (!isOutside(dieMark) && isOutside(coreMark)) ||
              (isInside(dieMark) && !isInside(coreMark))) {
            EcoNode* prevNode = paths[i-1].node;
            EcoNode* nextNode = paths[i+1].node;

            ClipLayer* layer = dbToEb(prevNode->layer);
            // y seg
            if( prevNode->x == nextNode->x && 
                prevNode->y != nextNode->y ) {
              int minY = std::min(prevNode->y, nextNode->y);
              int maxY = std::max(prevNode->y, nextNode->y);

              if( maxY < layer->gridY()[0] || 
                  minY > layer->gridY()[layer->gridY().size()-1] ) {
                logger->error(ECO, 32487, 
                    "dieMark is marked in a wrong way layer: {}, minY: {}, maxY: {}, coreMark:{}, dieMark:{}",
                    prevNode->layer->getConstName(),
                    minY, maxY, getString(coreMark), getString(dieMark));
              }

              minY = getCloseVar(minY, layer->gridY());
              maxY = getCloseVar(maxY, layer->gridY());

              ClipLayerPin* lowerPin 
                = layer->clipLayerPinDbu(prevNode->x, minY, false);

              ClipLayerPin* upperPin 
                = layer->clipLayerPinDbu(prevNode->x, maxY, false);
              
              if( lowerPin == nullptr || upperPin == nullptr ) {
                logger->error(ECO, 6000, 
                    "ClipLayerPin is nullptr, which is not accepted");
              }

              debugPrint(logger, ECO, "core_eco", 5, "segYcase: coreMark {}, dieMark {}: mark obs {} - {}",
                 getString(coreMark), getString(dieMark),
                 lowerPin->getMrc(), upperPin->getMrc());

              for(int j = lowerPin->y(); j <= upperPin->y(); j++) {
                ClipLayerPin* curClPin 
                  = layer->clipLayerPin(lowerPin->x(), j);
                if( curClPin-> isObsArea() ) {
                  if( j != lowerPin->y() ) {
                    curClPin->setHasObsDown(true);
                  }
                  if( j != upperPin->y() ) {
                    curClPin->setHasObsUp(true);
                  }
                }
              }
            }
            // x seg
            else if( prevNode->x != nextNode->x &&
                prevNode->y == nextNode->y) {
              int minX = std::min(prevNode->x, nextNode->x);
              int maxX = std::max(prevNode->x, nextNode->x);

              if( maxX < layer->gridX()[0] || 
                  minX > layer->gridX()[layer->gridX().size()-1] ) {
                logger->error(ECO, 32488, 
                    "dieMark is marked in a wrong way layer:{}, minX: {}, maxX: {}, coreMark:{}, dieMark:{}",
                    prevNode->layer->getConstName(),
                    minX, maxX, getString(coreMark), getString(dieMark));
              }


              minX = getCloseVar(minX, layer->gridX());
              maxX = getCloseVar(maxX, layer->gridX());

              ClipLayerPin* lowerPin 
                = layer->clipLayerPinDbu(minX, prevNode->y, false);

              ClipLayerPin* upperPin 
                = layer->clipLayerPinDbu(maxX, prevNode->y, false);

              if( lowerPin == nullptr || upperPin == nullptr ) {
                logger->error(ECO, 6001, 
                    "ClipLayerPin is nullptr, which is not accepted");
              }

              debugPrint(logger, ECO, "core_eco", 5, "segXcase: coreMark {}, dieMark {}: mark obs {} - {}",
                 getString(coreMark), getString(dieMark),
                 lowerPin->getMrc(), upperPin->getMrc());

              for(int j = lowerPin->x(); j <= upperPin->x(); j++) {
                ClipLayerPin* curClPin 
                  = layer->clipLayerPin(j, lowerPin->y());
                if( curClPin-> isObsArea() ) {
                  if( j != lowerPin->x() ) {
                    curClPin->setHasObsLeft(true);
                  }
                  if( j != upperPin->x() ) {
                    curClPin->setHasObsRight(true);
                  }
                }
              }
            }
            // 3d via segments
            else if( prevNode->x == nextNode->x &&
                prevNode->y == nextNode->y ) {

              ClipLayer* nextLayer = dbToEb(nextNode->layer);
              
              // the 3D via case happen when core: outside, die: inside

              int prevLayerNum = prevNode->layer->getRoutingLevel();
              int nextLayerNum = nextNode->layer->getRoutingLevel();

              // both prev/next node must be obs
              ClipLayerPin* clPin 
                = layer->clipLayerPinDbu(prevNode->x, prevNode->y, false);
              if( clPin->isObsArea() ) {
                if( prevLayerNum < nextLayerNum ) {
                  // prevPin, lower layer
                  clPin->setHasObsZUp(true);
                }
                else {
                  // prevPin, upper layer
                  clPin->setHasObsZDown(true);
                }
              }
              ClipLayerPin* nextClPin 
                = nextLayer->clipLayerPinDbu(nextNode->x, nextNode->y, false);
              if( nextClPin->isObsArea() ) {
                if( prevLayerNum < nextLayerNum ) {
                  // nextPin, lower layer
                  nextClPin->setHasObsZDown(true);
                }
                else {
                  // nextPin, upper layer
                  nextClPin->setHasObsZUp(true);
                }
              }

              debugPrint(logger, ECO, "core_eco", 5, "3DVIA case: coreMark {}, dieMark {}: mark obs {} - {}",
                 getString(coreMark), getString(dieMark),
                 clPin->getMrc(), nextClPin->getMrc());
            }
          }
        }
      }
    }
  }

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();
  for(auto& inst: fixed_insts_) {
    // get instance lx ly ux uy
    odb::dbBox* iBox = inst->getBBox();
    int gLx = (iBox->xMin() - site_lower_x_) / static_cast<int>(site->getWidth());
    int gLy = (iBox->yMin() - site_lower_y_) / static_cast<int>(site->getHeight());

    // get flipX/Y of fixed instance
    // see LEF/DEF manual
    bool isFlipX = false,
         isFlipY = false;

    switch(inst->getOrient()) {
      case odb::dbOrientType::R0:
        isFlipX = isFlipY = false;  
        break;
      case odb::dbOrientType::R180:
        isFlipX = isFlipY = true;
        break;
      case odb::dbOrientType::MX:
        isFlipX = false;
        isFlipY = true;
        break;
      case odb::dbOrientType::MY:
        isFlipX = true;
        isFlipY = false;
        break;
      default:
        break;
    }

    // update fixedSegs
    updateInstRoutingPinBlockage(inst, gLx, gLy, isFlipX, isFlipY);
  }


  // report
  int numObsClPins = 0;
  int numHasObsClPins = 0;
  for(auto& cLayer : clip_layers_) {
    for(auto& gridY : cLayer->grid()) {
      for(auto& clPin : gridY) {
        if( clPin.hasObs() ) {
          numHasObsClPins ++;
        }
        numObsClPins++;
      }
    }
  }
    
  logger->report("Total obs #ClipLayerPin: {}, hasObs #ClipLayerPin: {}",
      numObsClPins, numHasObsClPins);
}


void
Clip::updateCoordiInfoClipPtr() {
  for(auto& coordi : coordi_info_stor_) {
    coordi.setClip(this);
  }
}

CoordiInfo*
Clip::getCoordiInfo(int layer, int y, int x) {
  CoordiInfo tmp(layer, y, x, this);
  auto ptr = coordi_info_map_.find( tmp.getKey() );
  if( ptr == coordi_info_map_.end() ) {
    return nullptr;
  }
  else {
    return ptr->second;
  }
}



void
Clip::printRNet() {
  for(auto& rNet : rnets_) {
    rNet.print();
  }
}

std::set<int>
Clip::getAccessPoints(ClipLayerPin* clPin,
    int y, int x, bool isFlipX, bool isFlipY) {
  // y: dp grid y-coordinate
  // x: dp grid x-coordinate
  // flip: is instance flipped?  (N vs flipped south)
  //

  int itermLayerPitchX = iterm_layer_->gridX()[1] - iterm_layer_->gridX()[0];
  int itermLayerPitchY = iterm_layer_->gridY()[1] - iterm_layer_->gridY()[0];

  odb::dbMTerm* mTerm = clPin->iTerm()->getMTerm(); 
  eco::Pin* ecoPin = dbToEb(mTerm);
  eco::Master* ecoMaster = dbToEb(mTerm->getMaster());

  // check for master w/h is multiple of iterm layer's pitch
  // otherwise, some other logic is needed
  if( ecoMaster->width() % itermLayerPitchX != 0 ||
      ecoMaster->height() % itermLayerPitchY != 0 ) {
    logger->error(ECO, 162, "iterm layer: {} pitch: {} {} is not multiple of master size: {} {}",
        iterm_layer_->layer()->getConstName(),
        itermLayerPitchX, 
        itermLayerPitchY,
        ecoMaster->width(),
        ecoMaster->height());
  } 

  int pitchMasterWidth = ecoMaster->width() / itermLayerPitchX; 
  int pitchMasterHeight = ecoMaster->height() / itermLayerPitchY;

  // retrieve row/site
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  int pitchSiteWidth = site->getWidth() / itermLayerPitchX;
  int pitchSiteHeight = site->getHeight() / itermLayerPitchY;

  int pitchMarginX = (site_lower_x_ - die_.xMin()) / itermLayerPitchX;
  int pitchMarginY = (site_lower_y_ - die_.yMin()) / itermLayerPitchY;
  
  debugPrint(logger, ECO, "core_eco", 5, "getAccessPoints {}/{}, pmWidth: {}, pmHeight:{}, given y: {}, x: {}, isFlipX :{}, isFlipY: {}", 
      mTerm->getMaster()->getConstName(), 
      mTerm->getConstName(),
      pitchMasterWidth, 
      pitchMasterHeight,
      y, x, 
      isFlipX, isFlipY);

  std::set<int32_t> retSet;
  // pc: pin Coordi
  for(auto& pc : ecoPin->pinCoordis()) {
    for(int cx = pc.lx(); cx<=pc.ux(); cx++) {
      debugPrint(logger, ECO, "core_eco", 7, "  cx: {}", cx);
      for(int cy = pc.ly(); cy<=pc.uy(); cy++) {
        debugPrint(logger, ECO, "core_eco", 7, "    cy: {}", cy);
        int32_t newX = (isFlipX)? (pitchMasterWidth-1) - cx : cx ;
        int32_t newY = (isFlipY)? (pitchMasterHeight-1) - cy : cy;
        newX += x * pitchSiteWidth + pitchMarginX;
        newY += y * pitchSiteHeight + pitchMarginY;
        debugPrint(logger, ECO, "core_eco", 7, "      x: {} y: {}", newX, newY);
        retSet.insert((newY << 16) | newX);
      }
    }
  }
  return retSet;
}


std::set<int>
Clip::getAccessPoints(ClipLayerPin* clPin,
    int y, int x, int flip) {

  bool isFlipX = static_cast<bool>(flip), 
       isFlipY = (dp_grid_[y][x].orient() != odb::dbOrientType::R0 );

  return getAccessPoints(clPin, y, x, isFlipX, isFlipY);
}

// use OpenDB API to retrieve fixed instance on grid APs
std::set<int>
Clip::getFixedInstAccessPoints(ClipLayerPin* clPin) {
  odb::dbInst* inst = clPin->iTerm()->getInst();

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();

  // retrieve row/site
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  // get instance lx ly ux uy
  odb::dbBox* iBox = inst->getBBox();
  int gLx = (iBox->xMin() - site_lower_x_) / static_cast<int>(site->getWidth());
  int gLy = (iBox->yMin() - site_lower_y_) / static_cast<int>(site->getHeight());

  // get flipX/Y of fixed instance
  // see LEF/DEF manual
  bool isFlipX = false,
       isFlipY = false;

  switch(inst->getOrient()) {
    case odb::dbOrientType::R0:
      isFlipX = isFlipY = false;  
      break;
    case odb::dbOrientType::R180:
      isFlipX = isFlipY = true;
      break;
    case odb::dbOrientType::MX:
      isFlipX = false;
      isFlipY = true;
      break;
    case odb::dbOrientType::MY:
      isFlipX = true;
      isFlipY = false;
      break;
    default:
      break;
  }

  debugPrint(logger, ECO, "core_eco", 7, "    curFixedITerm: {}/{}, flipX: {}, flipY: {}", 
      clPin->iTerm()->getInst()->getConstName(),
      clPin->iTerm()->getMTerm()->getConstName(), isFlipX, isFlipY);

  std::set<int> res = getAccessPoints(clPin, gLy, gLx, isFlipX, isFlipY);
  std::set<int> filteredRes;
  int32_t markY = ~0 << 16;

  for(auto& ap : res) {
    int y = ((ap & markY) >> 16);
    int x = (ap & ~markY);

    // if clPin exists -- means within clip boundary
    if( iterm_layer_->clipLayerPin(x,y) ) {
      filteredRes.insert(ap);
    }
  }

  return filteredRes;
}

// return row, col indices pair
std::pair<int, int>
Clip::getPrevLayerRowCol(ClipLayer* curClipLayer, int r, int c) {
  ClipLayerPin* clPin = getPrevLayerClPin(curClipLayer, r, c);
  if( clPin ) {
    return make_pair(clPin->y(), clPin->x());
  } 
  else {
    return make_pair(-1,-1);
  }
}

// return row, col indices pair
std::pair<int, int>
Clip::getNextLayerRowCol(ClipLayer* curClipLayer, int r, int c) {
  ClipLayerPin* clPin = getNextLayerClPin(curClipLayer, r, c);
  if( clPin ) {
    return make_pair(clPin->y(), clPin->x());
  } 
  else {
    return make_pair(-1,-1);
  }
}

ClipLayer*
Clip::getPrevClipLayer(ClipLayer* curClipLayer) {
  int num = curClipLayer->layer()->getRoutingLevel();
  ClipLayer* prevClipLayer = dbToEb(db_->getTech()->findRoutingLayer(num-1));
  return prevClipLayer;
}

ClipLayer*
Clip::getNextClipLayer(ClipLayer* curClipLayer) {
  int num = curClipLayer->layer()->getRoutingLevel();
  ClipLayer* nextClipLayer = dbToEb(db_->getTech()->findRoutingLayer(num+1));
  return nextClipLayer;
}

ClipLayerPin* 
Clip::getPrevLayerClPin(ClipLayer* curClipLayer, int r, int c) {
  ClipLayer* prevClipLayer = getPrevClipLayer(curClipLayer);
  if( prevClipLayer == nullptr ) {
    return nullptr;
  }

  ClipLayerPin* clPin 
    = prevClipLayer->clipLayerPinDbu(
        curClipLayer->gridX()[c], 
        curClipLayer->gridY()[r], 
        false);

  return clPin;
}

ClipLayerPin* 
Clip::getNextLayerClPin(ClipLayer* curClipLayer, int r, int c) {
  ClipLayer* nextClipLayer = getNextClipLayer(curClipLayer);
  if( nextClipLayer == nullptr ) {
    return nullptr; 
  }

  ClipLayerPin* clPin 
    = nextClipLayer->clipLayerPinDbu(
        curClipLayer->gridX()[c], 
        curClipLayer->gridY()[r], 
        false);

  return clPin;
}

RNet* 
Clip::getRNet(std::string name) const {
  auto rptr = rnet_map_.find(name);
  if( rptr == rnet_map_.end()) {
    return nullptr;
  }
  return rptr->second;
}


void
Clip::writeSMT2FlowCapControl( std::ofstream& out,
    RNet& rNet,
    int cIdx,
    ClipLayerPin* clPin ) {

  if( clPin->isFixed() ) { 
    if (clPin->accessPointPairs().size() > 0) {
      std::string allApStr = "";
      for(auto& ap : clPin->accessPointPairs()) {
        allApStr += rNet.getName() 
          + "_C" + to_string(cIdx) 
          + "_E_m" + to_string(itermLayerNum) 
          + "r" + to_string(ap.first) + "c" + to_string(ap.second)
          + "_" + clPin->getVertName() + " ";
      }
      out << "(assert (and ((_ at-least 1) " << allApStr 
       << ") ((_ at-most 1) " << allApStr << ")))" << endl; 
    }
  }
  else {
    // retrieve row/site
    odb::dbBlock* block = db_->getChip()->getBlock();
    odb::dbSet<odb::dbRow> rows = block->getRows();
    odb::dbRow* row = *(rows.begin());
    odb::dbSite* site = row->getSite();

    // check if current instance can be placed
    // with in given y, [lx, ux]
    //
    odb::dbBox* iBox = clPin->iTerm()->getInst()->getBBox();
    int instSiteWidth = (iBox->getDX()) 
      / static_cast<int>(site->getWidth());
    
    for(auto& dpPairsY : dp_grid_place_pairs_) {
      int y = &dpPairsY - &dp_grid_place_pairs_[0];
      for(auto& dpPairX : dpPairsY) {
        // can be placed, within this row (row width const)
        if( dpPairX.second - dpPairX.first + 1 >= instSiteWidth ) {
          // possible x loc
          for(int x=dpPairX.first; 
              x<=dpPairX.second - instSiteWidth + 1; 
              x++) {
            
            // foreach y, x, f 
            for(int f = 0; f<2; f++) {

              std::set<int> usedCoordi; 
              std::vector<pair<int, int>> unusedCoordi;

              for(auto& ap : getAccessPoints(clPin, y, x, f)) {
                usedCoordi.insert(ap);
              }

              for(auto& ap: clPin->accessPointPairs() ) {
                int32_t key = ap.first << 16 | ap.second;
                auto cPtr = usedCoordi.find(key); 
                if( cPtr == usedCoordi.end() ) {
                  unusedCoordi.push_back(make_pair(ap.first, ap.second));
                }
              }

              string instName 
                = std::string(clPin->iTerm()->getInst()->getConstName());

              string xInst = "x_" + instName;
              string rInst = "r_" + instName;
              string fInst = "ff_" + instName;
              string fString = (f == 0)? "false" : "true";

              string allApNames = "";
              int32_t markY = ~0 << 16;
              for(auto& ap : usedCoordi) {
                allApNames += rNet.getName() 
                  + "_C" + to_string(cIdx)
                  + "_E_" + "m" + to_string(itermLayerNum)
                  + "r" + to_string((ap & markY) >> 16)
                  + "c" + to_string(ap & ~markY)
                  + "_" + clPin->getVertName() + " ";
              }

              out << "(assert (ite (and (= " 
                << xInst << " (_ bv" << x << " 9)) (= " 
                << rInst << " (_ bv" << y << " 4)) (= "
                << fInst << " " << fString << ")) (and ((_ at-least 1) "
                << allApNames << ") ((_ at-most 1) " 
                << allApNames << ") ";

              for(auto& ap : unusedCoordi) {
                out << "(= " << rNet.getName()
                  << "_C" << cIdx << "_E_m" << itermLayerNum
                  << "r" << ap.first << "c" << ap.second
                  << "_" << clPin->getVertName() << " false) ";
              }
              out << ") (= 1 1)))" << endl;
            }
          }
        }
      }
    }
  }
}

void 
Clip::writeSMT2CommFlowConserv(std::ofstream& out,
    RNet& rNet, int cIdx, std::vector<SMT2Segment>& strings) {
  if( strings.size() == 0 ) {
    return;
  }
  // the single tip case must be false.
  else if( strings.size() == 1 ) {
    out << "(assert (= " 
      << rNet.getName() << "_C" << cIdx << "_E_"
      << strings[0].str() << " false))" << endl; 
  }
  else if( strings.size() == 2 ) {
    // implements NXOR
    out << "(assert (= (or (not "
      << rNet.getName() << "_C" << cIdx << "_E_"
      << strings[0].str() << ") " 
      << rNet.getName() << "_C" << cIdx << "_E_" 
      << strings[1].str() << ") true ))" << endl;

    out << "(assert (= (or "
      << rNet.getName() << "_C" << cIdx << "_E_"
      << strings[0].str() << " (not " 
      << rNet.getName() << "_C" << cIdx << "_E_" 
      << strings[1].str() << ")) true ))" << endl;
  }
  else {
    // implements, at-most 2 is true or forcely makes all falses
    std::vector<std::string> newStrings;
    newStrings.reserve(strings.size());
    for(auto& str : strings) {
      newStrings.push_back( rNet.getName() + "_C" + to_string(cIdx)
          + "_E_" + str.str() );
    }

    out << "(assert ((_ at-most 2) ";
    for(auto& str : newStrings) {
      out << str << " ";
    }
    out << "))" << endl;

    for(int i=0; i<newStrings.size(); i++) {
      out << "(assert (= (or ";
      for(int j=0; j<newStrings.size(); j++) {
        if( i == j ) {
          out << "(not " << newStrings[j] << ") ";
        }
        else {
          out << newStrings[j] << " ";
        }
      }
      out << ") true ))" << endl;
    }
  }
}

void 
Clip::writeSMT2VertexExclusive(
    std::ofstream& out, 
    RNet& rNet, 
    CoordiInfo& coordi,
    std::vector<SMT2Segment>& strings) {
  if( strings.size() == 0 ){
    return;
  }

  out << "(assert (= V_" << rNet.getName() 
    << "_m" << coordi.layer() 
    << "r" << coordi.y() 
    << "c" << coordi.x()
    << " ";
  
  out << "(or ";
  for(auto& str : strings) {
    out << rNet.getName() << "_E_" << str.str() << " ";
  }
  out << ")))" << endl;
}

void
Clip::writeSMT2EdgeAssignments(
    std::ofstream& out,
    std::string& commStr, 
    std::vector<std::string>& indivStrs) {

  if( indivStrs.size() == 0 ) {
    return;
  } 

  for(auto& iStr : indivStrs) {
    out << "(assert (ite (= " << iStr << " true) (= " << commStr << " true) (= 1 1)))" << endl;
  }

  out << "(assert (ite (= " << commStr << " true) ((_ at-least 1) ";
  for(auto& iStr : indivStrs) {
    out << iStr << " ";
  }
  out << ") (= 1 1)))" << endl;
}


void
Clip::writeSMT2(std::string fileName) {
  logger->info(ECO, 5231, "Started to write the SMT2 file: {}", fileName); 

  // clip ptr sanity check
  updateCoordiInfoClipPtr();

  std::ofstream myFile;
  myFile.open(fileName);

  myFile << ";Layout Information" << endl;
  int dbu = db_->getChip()->getBlock()->getDbUnitsPerMicron(); 
  myFile << "; gui::zoom_to " << 
    static_cast<float>(die().xMin())/dbu << " " << 
    static_cast<float>(die().yMin())/dbu << " " << 
    static_cast<float>(die().xMax())/dbu << " " << 
    static_cast<float>(die().yMax())/dbu << endl;

  myFile << "; gui::zoom_to " << 
    static_cast<float>(core().xMin())/dbu << " " << 
    static_cast<float>(core().yMin())/dbu << " " << 
    static_cast<float>(core().xMax())/dbu << " " << 
    static_cast<float>(core().yMax())/dbu << endl;

  myFile << endl;
  myFile << "; #Nets: " << nets_.size() << endl;
  myFile << "; #RNets: " << rnets_.size() << endl;
  myFile << "; #movableInsts: " << movable_insts_.size() << endl;
  myFile << "; #fixedInsts: " << fixed_insts_.size() << endl;
  myFile << endl;
  myFile << ";DP grid Information" << endl;
  if( dp_grid_.size() > 0 ) {
    myFile << "; GridCntX: " << dp_grid_[0].size() << endl;
  }
  else {
    myFile << "; GridCntX: 0" << endl;
  }
  myFile << "; GridCntY: " << dp_grid_.size() << endl;
  myFile << endl;

  outStreamDpGrid(myFile);
  
  myFile << "; #clipLayers: " << clip_layers_.size() << endl;
  myFile << "; layerName gridCntX gridCntY pitchX pitchY" << endl;
  for(auto& clipLayer : clip_layers_ ) {
    myFile << ";  " << clipLayer->layer()->getConstName() << " ";
    myFile << clipLayer->gridX().size() << " ";
    myFile << clipLayer->gridY().size() << " ";
    myFile << clipLayer->gridX()[1] - clipLayer->gridX()[0] << " ";
    myFile << clipLayer->gridY()[1] - clipLayer->gridY()[0] << endl;
  }
  myFile << endl;

  myFile << "; OBS clipLayerPin info" << endl;
  for(auto& clipLayer : clip_layers_ ) {
    for(auto& clPin : clipLayer->allClipLayerPins()) {
      // if(clPin->isObsArea() && clPin->hasObs()) {
      if(clPin->hasObs()) {
        myFile << "; " << clPin->getMrc() << " " << clPin->getObsString() << endl;
      }
    }
  }
  

  
  myFile << "; 1. Placement Formulation" << endl;
  myFile << "; 1.A. Variables for Placement" << endl;
  // declare constraints name for DP grid
  // x: location x (col)
  // r: location y (row)
  // ff: flip, 0 means orig, 1 means flipped
  for(auto& inst: movable_insts_) {
    myFile << "(declare-const x_" << inst->getConstName() << " (_ BitVec 9))" << endl;
    myFile << "(declare-const r_" << inst->getConstName() << " (_ BitVec 4))" << endl;
    myFile << "(declare-const ff_" << inst->getConstName() << " Bool)" << endl;
  } 
  myFile << endl;

  // retrieve row/site
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  myFile << "; 1.B. Constraints for Placement" << endl; 
  myFile << "; 1.B.1. Row Constraints" << endl; 

  for(auto& inst: movable_insts_) {
    // 0 <= r_ <= numDpRows-1
    myFile << "(assert (and (bvsge r_" << inst->getConstName() 
      << " (_ bv0 4)) (bvsle r_" << inst->getConstName() 
      << " (_ bv" << dp_grid_.size()-1 << " 4))))" << endl;
    

    // get grid (lx, ly, ux uy)
    odb::dbBox* iBox = inst->getBBox();
    
    int gDx = (iBox->getDX()) / static_cast<int>(site->getWidth());
    //int gDy = (iBox->getDY()) / static_cast<int>(site->getHeight());

    // common string
    std::string xStr = std::string("x_") + inst->getConstName();
    std::string rStr = std::string("r_") + inst->getConstName();

    int iteCnt = 0;
    myFile << "(assert " << endl;

    for(auto& row : dp_grid_place_pairs_) {
      int rowIdx = &row - &dp_grid_place_pairs_[0]; 
      for(auto& pair : row) {
        // means current inst can be placed within this fragmented row pair
        if(pair.second - pair.first + 1 >= gDx) {
           myFile << "    (ite (= " << rStr  
            << " (_ bv" << rowIdx << " 4))";

           // (x_ == LB || x_ >= LB+2) && (x_ + dx == UB || x_ + dx <= UB-2)
           myFile << " (and";
           myFile << " (or (= " << xStr 
             << " (_ bv" << pair.first << " 9))" 
             << " (bvsge " << xStr
             << " (_ bv" << pair.first + 2 << " 9)))"; 
           myFile << " (or (= " << xStr
             << " (_ bv" << pair.second - gDx + 1 << " 9))";
           
           int ubSub2 = pair.second - gDx - 1;
           // avoid (_ bv-1 9) string -- negative value cause syntax error
           if( ubSub2 >= 0 ) {
             myFile  << " (bvsle " << xStr
               << " (_ bv" << pair.second - gDx - 1 << " 9))";
           }
           myFile << "))";
           iteCnt += 1;
        }
        myFile << endl;
      }
    }
    if( iteCnt >= 1 ) {
      // make this condition as unsat --> Forced condition
      myFile << "(= #b1 #b0)";
      for(int i=0; i<iteCnt; i++) {
        myFile << ")";
      }
    }
    myFile << ")" << endl;
  }

  myFile << endl;

  myFile << "; 1.B.2. Relative Positioning Constraint (RPC)" << endl;

  for(auto& inst1: movable_insts_) {
    int idx1 = &inst1 - &movable_insts_[0];
    for(auto& inst2: movable_insts_) { 
      int idx2 = &inst2 - &movable_insts_[0];

      // for every possible two pairs from insts
      if( idx1 > idx2 ) {
        std::string rStr1 = std::string("r_") + inst1->getConstName();
        std::string rStr2 = std::string("r_") + inst2->getConstName();

        std::string xStr1 = std::string("x_") + inst1->getConstName();
        std::string xStr2 = std::string("x_") + inst2->getConstName();

        odb::dbBox* iBox = inst1->getBBox();
        int w1 = (iBox->getDX()) / static_cast<int>(site->getWidth());
        iBox = inst2->getBBox();
        int w2 = (iBox->getDX()) / static_cast<int>(site->getWidth());


        // if row are different each other, ignore
        myFile << "(assert (ite (not (= " << rStr1 << " " << rStr2 << ")) (= #b0 #b0)" << endl;

        // RHS1 - if( x1 + w1 == x2 ), then x1 + w1 == x2
        myFile << "    (ite (= (bvadd " << xStr1 
          << " (_ bv" << w1 << " 9)) " << xStr2 
          << ") (= (bvadd " << xStr1 << " (_ bv" << w1 << " 9)) " << xStr2 << ")" << endl;

        // RHS2 - if( x1 + w1 > x2 ), then x1 + w1 + 2 >= x2 (filler cell const)
        myFile << "    (ite (bvslt (bvadd " << xStr1
          << " (_ bv" << w1 << " 9)) " << xStr2 
          << ") (bvsle (bvadd " << xStr1 << " (_ bv" << w1+2 << " 9)) " << xStr2 << ")" << endl;

        // RHS3 - if( x1 - w2 == x2 ), then x1 - w2 == x2 
        myFile << "    (ite (= (bvsub " << xStr1 
          << " (_ bv" << w2 << " 9)) " << xStr2 
          << ") (= (bvsub " << xStr1 << " (_ bv" << w2 << " 9)) " << xStr2 << ")" << endl;

        // RHS4 - if( x1 - w2 < x2 ), then x1 - w2 + 2 <= x2 (filler cell const)
        myFile << "    (ite (bvsgt (bvsub " << xStr1
          << " (_ bv" << w2 << " 9)) " << xStr2 
          << ") (bvsge (bvsub " << xStr1 << " (_ bv" << w2+2 << " 9)) " << xStr2 << ")" << endl;
         
        // The RHS1~4 must be satisfied. Otherwise, UNSAT
        myFile << "    (= #b1 #b0)))))))" << endl;
      }
    }
  }

  myFile << "; 2. Routing Formulation" << endl;
  myFile << "; 2.1. Declare routing variables" << endl;
  myFile << "; 2.1.A. vertex - V_{net}_{loc}" << endl;

  // define variables
  // 1D: each route layer
  // 2D: set for clPin
  
  int numLayers = db_->getTech()->getRoutingLayerCount();
  std::vector<std::set<ClipLayerPin*>> 
    movableClPins(numLayers, 
        std::set<ClipLayerPin*>());

  std::vector<std::set<ClipLayerPin*>>
    fixedClExtPins(numLayers,
        std::set<ClipLayerPin*>());

  for(auto& rNet: rnets_ ){
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      continue;
    }
    // update movable clip layer pins
    for(auto& pin : rNet.pins()) {
      int layer = pin->layer()->getRoutingLevel();
      if( !pin->isFixed() ) {
        movableClPins[layer].insert(pin);
      } 
      else {
        fixedClExtPins[layer].insert(pin);
      }
    }

    for(auto& clipLayer : clip_layers_ ) {
      for(auto& gridY : clipLayer->grid()) {
        for(auto& clPin : gridY) {
          // if there is no obs, define a new clpin var
          if( !(clPin.isObsArea() && clPin.hasObs()) ) {
            myFile << "(declare-const V_" << rNet.getName() << "_" 
              << clPin.getMrc() << " Bool)" << endl;
          }
        }
      }
    }
  }


  myFile << "; 2.1.B. commodity net variable gen - {net}_C{N}_E_{fromLoc}_{toLoc} and {net}_E_{fromLoc}_{toLoc}" << endl;
  set<string> totalCommStrs;
  set<string> totalIndivStrs;
  
  // for each rNet
  for(auto& rNet : rnets_ ) {
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      continue;
    }
    // for each coordi
    for(auto& curCoordi : coordi_info_stor_ ) {
      for(auto& pair : curCoordi.getSMT2ClipLayerPinsMap(&rNet)) {
          
        // common string : {netName}_E_{fromLoc}_{toLoc} -- union of C{N}
        totalCommStrs.insert(rNet.getName() 
          + "_E_m" + to_string( curCoordi.layer() )
          + "r" + to_string( curCoordi.y() )
          + "c" + to_string( curCoordi.x() )
          + "_" + pair.first->getVertName());

        // individual strings : {netName}_C{N}_E_{fromLoc}_{toLoc} 
        for(auto& smt2Pin : pair.second) {
          if( curCoordi.layer() 
              != smt2Pin->clipLayerPin()->layer()->getRoutingLevel()) {
            continue;
          }
          totalIndivStrs.insert( 
              rNet.getName() + "_C" + to_string( smt2Pin->commodityIndex() )
              + "_E_m" + to_string( curCoordi.layer() )
              + "r" + to_string( curCoordi.y() )
              + "c" + to_string( curCoordi.x() )
              + "_" + pair.first->getVertName() );
        }

        // VIA12 consideration
        if( pair.first->layer()->getRoutingLevel() == itermLayerNum ) {
          std::pair<int, int> nextRc 
            = getNextLayerRowCol(iterm_layer_, curCoordi.y(), curCoordi.x());
          if( nextRc.first != -1 && nextRc.second != -1) {

            // common string : {netName}_E_{fromLoc}_{toLoc} -- union of C{N}
            totalCommStrs.insert(rNet.getName() 
              + "_E_m" + to_string( curCoordi.layer() )
              + "r" + to_string( curCoordi.y() )
              + "c" + to_string( curCoordi.x() )
              + "_m" + to_string(itermLayerNum+1)
              + "r" + to_string( nextRc.first)
              + "c" + to_string( nextRc.second));

            // individual strings : {netName}_C{N}_E_{fromLoc}_{toLoc} 
            for(auto& smt2Pin : pair.second) {
              if( curCoordi.layer() 
                  != smt2Pin->clipLayerPin()->layer()->getRoutingLevel()) {
                continue;
              }
              totalIndivStrs.insert( 
                  rNet.getName() + "_C" + to_string( smt2Pin->commodityIndex() )
                  + "_E_m" + to_string( curCoordi.layer() )
                  + "r" + to_string( curCoordi.y() )
                  + "c" + to_string( curCoordi.x() )
                  + "_m" + to_string(itermLayerNum+1)
                  + "r" + to_string( nextRc.first)
                  + "c" + to_string( nextRc.second) );
            }
          }
        }
      }

      for(auto& adjSeg : curCoordi.getAdjStrings()) {
        totalCommStrs.insert(rNet.getName() + "_E_" + adjSeg.str());
        for(int i=0; i<rNet.getCommodityCnt(); i++) {
          totalIndivStrs.insert(rNet.getName() + "_C" + to_string(i) + 
              "_E_" + adjSeg.str());
        }
      }
    }
  }

  for(auto& str : totalCommStrs) {
    myFile << "(declare-const " << str << " Bool)" << endl;
  }

  for(auto& str : totalIndivStrs) {
    myFile << "(declare-const " << str << " Bool)" << endl;
  }

  int itermLayerNum = iterm_layer_->layer()->getRoutingLevel();

  myFile << "; 2.1.C. global view of movable/fixed clip layer pins def - M_{loc}_{pinName} | M_{fromLoc}_{toLoc}" << endl;
  
  // get all segments. (SMT2Pin + adjSegments)
  std::set<string> allSegmentsSet;

  // get all common adjacent segments. (only adjSegments)
  std::set<string> allAdjSegmentsSet;

  // saves (level(int)) -> (M_strings)
  // Needed for the objectives 
  // expected it to have the (M1AP -> VIA12 -> M2 -> VIA23 -> M3 -> VIA34 -> M4) order
  std::vector<std::set<string>> totalSegmentsStor;
  totalSegmentsStor.resize(SMT2Segment::maxSegmentOrder, std::set<string>());

  // traverse coordi_info_stor to define M_{loc}_{pinName}
  // update totalSegmentsStor
  for(auto& coordi : coordi_info_stor_) {
    for(auto& rNet : rnets_) {
      if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
        continue;
      }

      for(auto& str : coordi.getAllStrings(&rNet)) {
        allSegmentsSet.insert(str.str()); 
        totalSegmentsStor[str.intOrder()].insert(str.str());
      }
    }

    for(auto& str : coordi.getAdjStrings()) { 
      // update allAdjSegmentsSet
      allAdjSegmentsSet.insert(str.str());
      totalSegmentsStor[str.intOrder()].insert(str.str());
    }
  }

  // write M_{loc}_{pinName} || M_{fromLoc}_{toLoc}
  for(auto& str : allSegmentsSet) {
    myFile << "(declare-const M_" << str << " Bool)" << endl;
  }

  myFile << "; 2.1.D. Geometric variables (L:left, R:right, U:up, D:down)" << endl;

  // horizontal
  std::vector<std::string> horiGvStrs = {"GL", "GR"};
  // vertical
  std::vector<std::string> vertGvStrs = {"GU", "GD"};

  for(auto& clipLayer: clip_layers_) {
    if( clipLayer->layer()->getRoutingLevel() >= from_route_layer_ && 
        clipLayer->layer()->getRoutingLevel() <= to_route_layer_ ) { 

      auto layerDir = clipLayer->layer()->getDirection();
      std::vector<std::string>* gvStrs = nullptr;

      //change gvStrs ptr
      if( layerDir == odb::dbTechLayerDir::VERTICAL) {
        gvStrs = &vertGvStrs;
      }
      else if( layerDir == odb::dbTechLayerDir::HORIZONTAL) {
        gvStrs = &horiGvStrs;
      }

      for(auto& gvStr : *gvStrs) {
        for(auto& gridY: clipLayer->grid()) {
          for(auto& clPin : gridY) {
            myFile << "(declare-const " << gvStr 
              << "_V_" << clPin.getMrc() << " Bool)" << endl;
          }
        }
      }
    }
  }


  myFile << "; 2.2. Routing Constraints " << endl;
  myFile << "; 2.2.1. [Virtual Edge] Flow capacity control, exactly-1 vertex exists(VE), just true (EXT)" << endl;
  myFile << "; Instance physical info (location + flip) -> corresponding edges assignment" << endl;

  for(auto& rNet : rnets_) {
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      continue;
    }

    int numCommCnt = rNet.getCommodityCnt();
    for(int i=0; i<numCommCnt; i++) {
      ClipLayerPin* fromPin = rNet.commodityOutPin(i);
      ClipLayerPin* toPin = rNet.commodityInPin(i);

      if( fromPin->type() 
          == ClipLayerPin::ClipLayerPinType::ITERM) {
        writeSMT2FlowCapControl(myFile, rNet, i, fromPin);
      }

      if( toPin->type() 
          == ClipLayerPin::ClipLayerPinType::ITERM) {
        writeSMT2FlowCapControl(myFile, rNet, i, toPin);
      }
    }
  }

  myFile << "; 2.2.2. Commodity flow conservation for each vertex and every connected edge to the vertex" << endl; 

  for(auto& rNet : rnets_) {
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      continue;
    }

    for(int i=0; i<rNet.getCommodityCnt(); i++) {
      // for all coordinates
      for(auto& curCoordi : coordi_info_stor_) {
        std::vector<SMT2Segment> strings = curCoordi.getAllStringsCIdx(&rNet, i);
        writeSMT2CommFlowConserv( myFile, rNet, i, strings);
      }
    }
  }

  myFile << "; 2.2.3. Vertex exclusiveness" << endl;

  for(auto& curCoordi : coordi_info_stor_ ) {
    std::set<RNet*> survivedNets;
    for(auto& rNet : rnets_ ) {
      if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
        continue;
      }
      std::vector<SMT2Segment> strings = curCoordi.getAllStrings(&rNet);
      // skip for empty case
      if( strings.size() == 0 ) {
        continue;
      }

      writeSMT2VertexExclusive(myFile, rNet, curCoordi, strings);
      survivedNets.insert(&rNet);
    } 

    //  at most 1 on every vertices
    if( survivedNets.size() > 1) {
      myFile<< "(assert ((_ at-most 1) ";
      for(auto& rNet : survivedNets) {
        myFile << "V_" << rNet->getName() 
          << "_m" << curCoordi.layer() 
          << "r" << curCoordi.y()
          << "c" << curCoordi.x() << " ";
      }
      myFile << "))" << endl;
    }
  }

  myFile << "; 2.2.4. Edge Assigmnets" << endl; 

  // for each rNet
  for(auto& rNet : rnets_ ) {
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      continue;
    }
    // for each coordi
    for(auto& curCoordi : coordi_info_stor_ ) {
      for(auto& pair : curCoordi.getSMT2ClipLayerPinsMap(&rNet)) {
          
        // common string : {netName}_E_{fromLoc}_{toLoc} -- union of C{N}
        string commStr = rNet.getName() 
          + "_E_m" + to_string( curCoordi.layer() )
          + "r" + to_string( curCoordi.y() )
          + "c" + to_string( curCoordi.x() )
          + "_" + pair.first->getVertName();

        // individual strings : {netName}_C{N}_E_{fromLoc}_{toLoc} 
        std::vector<string> indivStrs;
        for(auto& smt2Pin : pair.second) {
          if( curCoordi.layer() 
              != smt2Pin->clipLayerPin()->layer()->getRoutingLevel()) {
            continue;
          }
          indivStrs.push_back( 
              rNet.getName() + "_C" + to_string( smt2Pin->commodityIndex() )
              + "_E_m" + to_string( curCoordi.layer() )
              + "r" + to_string( curCoordi.y() )
              + "c" + to_string( curCoordi.x() )
              + "_" + pair.first->getVertName() );
        }

        writeSMT2EdgeAssignments(myFile, commStr, indivStrs);

        // VIA12 consideration
        if( pair.first->layer()->getRoutingLevel() == itermLayerNum ) {
          std::pair<int, int> nextRc 
            = getNextLayerRowCol(iterm_layer_, curCoordi.y(), curCoordi.x());
          if( nextRc.first != -1 && nextRc.second != -1) {

            // common string : {netName}_E_{fromLoc}_{toLoc} -- union of C{N}
            commStr = rNet.getName() 
              + "_E_m" + to_string( curCoordi.layer() )
              + "r" + to_string( curCoordi.y() )
              + "c" + to_string( curCoordi.x() )
              + "_m" + to_string(itermLayerNum+1)
              + "r" + to_string( nextRc.first)
              + "c" + to_string( nextRc.second);

            // individual strings : {netName}_C{N}_E_{fromLoc}_{toLoc} 
            indivStrs.clear();
            indivStrs.shrink_to_fit();
            for(auto& smt2Pin : pair.second) {
              if( curCoordi.layer() 
                  != smt2Pin->clipLayerPin()->layer()->getRoutingLevel()) {
                continue;
              }
              indivStrs.push_back( 
                  rNet.getName() + "_C" + to_string( smt2Pin->commodityIndex() )
                  + "_E_m" + to_string( curCoordi.layer() )
                  + "r" + to_string( curCoordi.y() )
                  + "c" + to_string( curCoordi.x() )
                  + "_m" + to_string(itermLayerNum+1)
                  + "r" + to_string( nextRc.first)
                  + "c" + to_string( nextRc.second) );
            }
            writeSMT2EdgeAssignments(myFile, commStr, indivStrs);
          }
        }
      }

      for(auto& adjSeg : curCoordi.getAdjStrings()) {
        string commStr = rNet.getName() + "_E_" + adjSeg.str();
        std::vector<string> indivStrs;  
        for(int i=0; i<rNet.getCommodityCnt(); i++) {
          indivStrs.push_back( rNet.getName() + "_C" + to_string(i) + 
              "_E_" + adjSeg.str());
        }
        writeSMT2EdgeAssignments(myFile, commStr, indivStrs);
      }
    }
  }
  
  myFile << "; 2.2.5. Exclusiveness use of each edge + Metal segment assignment by using edge usage information" << endl;


  // print all of {loc}_{pinName} pairs  from coordi_info_stor_'s
  // SMT2Pin structures
  for(auto& coordi : coordi_info_stor_) {
    // first: {loc}_{pinName}
    // second: {netName}_E_{loc}_{pinName}
    std::map<string, set<string>> smt2PinStrMap; 
  
    // for each net 
    for(auto& rNet : rnets_) {
      if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
        continue;
      }

      for(auto& str : coordi.getSMT2Strings(&rNet)) {
        auto ptr = smt2PinStrMap.find(str.str());
        // not found key from map
        if( ptr == smt2PinStrMap.end() ) {
          std::set<string> tmp;
          tmp.insert(rNet.getName() + "_E_" + str.str());
          smt2PinStrMap[str.str()] = tmp;
        }
        // found
        else {
          ptr->second.insert(
              rNet.getName() + "_E_" + str.str());
        }
      }
    }
    
    // at most 1
    for(auto& strPair : smt2PinStrMap) {
      if( strPair.second.size() >= 1) {
        myFile << "(assert (= M_" << strPair.first << " (or ";
        for(auto& str : strPair.second) {
          myFile << str << " ";
        }
        myFile << ")))" << endl;

        if( strPair.second.size() >= 2) {
          myFile << "(assert ((_ at-most 1) ";
          for(auto& str : strPair.second) {
            myFile << str << " ";
          }
          myFile << "))" << endl;
        }
      }
    }

  }

  // reuse allAdjSegmentsSet
  for(auto& str : allAdjSegmentsSet) {
    if( rnets_.size() >= 1 ) {
      myFile << "(assert (= M_" << str << " (or ";
      for(auto& rNet : rnets_) {
        if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
          continue;
        }
        myFile << rNet.getName() << "_E_" << str << " ";
      }
      myFile << ")))" << endl;

      if( rnets_.size() >= 2 ) {
        myFile << "(assert ((_ at-most 1) ";
        for(auto& rNet : rnets_) {
          if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
            continue;
          }
          myFile << rNet.getName() << "_E_" << str << " ";
        }
        myFile << "))" << endl;
      }
    }
  }

  myFile << "; 2.2.6. External pins - must be occupied. Always" << endl; 
  for(auto& rNet: rnets_) {
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      continue;
    }

    set<ClipLayerPin*> isPrintedClPin;
    for(int i=0; i<rNet.getCommodityCnt(); i++) {
      ClipLayerPin* from = rNet.commodityOutPin(i);
      ClipLayerPin* to = rNet.commodityInPin(i);

      if( from->type() == ClipLayerPin::ClipLayerPinType::EXTPIN ) {
        isPrintedClPin.insert(from);
        myFile << "(assert (= " << rNet.getName() << "_C" << i 
          << "_E_"
          << "m" << from->layer()->getRoutingLevel()
          << "r" << from->y()
          << "c" << from->x() << "_" << from->getVertName() << " true))"<< endl;
      }

      if( to->type() == ClipLayerPin::ClipLayerPinType::EXTPIN ) {
        isPrintedClPin.insert(to);
        myFile << "(assert (= " << rNet.getName() << "_C" << i 
          << "_E_"
          << "m" << to->layer()->getRoutingLevel()
          << "r" << to->y()
          << "c" << to->x() << "_" << to->getVertName() << " true))"<< endl;
      }
    }

    for(auto& clPin : isPrintedClPin) {
      myFile << "(assert (= " << rNet.getName() << "_E_"
        << "m" << clPin->layer()->getRoutingLevel()
        << "r" << clPin->y()
        << "c" << clPin->x() << "_" << clPin->getVertName() << " true))"<< endl;
    }
  }

  myFile << "; 2.2.7. Geometric Constraints (L:left, R:right, U:up, D: down)" << endl; 

  // writeSMT2GeomConsts function has mode variable
  //
  // mode variable
  // 0: LEFT (GL)
  // 1: RIGHT (GR)
  // 2: DOWN (GD)
  // 3: UP (GU)
  //
  //
  myFile << "; 2.2.7.1. Geometric Constraints - Left Tip" << endl; 
  // LEFT
  for(auto& clipLayer : clip_layers_ ) {
    writeSMT2GeomConsts(myFile, 0, *clipLayer);
  }

  myFile << "; 2.2.7.2. Geometric Constraints - Right Tip" << endl; 
  // RIGHT 
  for(auto& clipLayer : clip_layers_ ) {
    writeSMT2GeomConsts(myFile, 1, *clipLayer);
  }

  myFile << "; 2.2.7.3. Geometric Constraints - Down Tip" << endl; 
  // RIGHT 
  for(auto& clipLayer : clip_layers_ ) {
    writeSMT2GeomConsts(myFile, 2, *clipLayer);
  }

  myFile << "; 2.2.7.4. Geometric Constraints - Up Tip" << endl; 
  // RIGHT 
  for(auto& clipLayer : clip_layers_ ) {
    writeSMT2GeomConsts(myFile, 3, *clipLayer);
  }

  myFile << "; 2.2.8. Minimum Area Rule (MAR)" << endl;
  for(auto& clipLayer : clip_layers_) {
    if( clipLayer->layer()->getRoutingLevel() >= from_route_layer_ && 
        clipLayer->layer()->getRoutingLevel() <= to_route_layer_ ) { 
      auto layerDir = clipLayer->layer()->getDirection();
      if( layerDir == odb::dbTechLayerDir::VERTICAL) {
        for(auto& gridY : clipLayer->grid()) {
          for(auto& clPin : gridY) {
            myFile << "(assert ((_ at-most 1) GU_V_" << clPin.getMrc() 
              << " GD_V_" << clPin.getMrc() << "))" << endl;
          }
        }
      }
      else if( layerDir == odb::dbTechLayerDir::HORIZONTAL) {
        for(auto& gridY : clipLayer->grid()) {
          for(auto& clPin : gridY) {
            myFile << "(assert ((_ at-most 1) GL_V_" << clPin.getMrc() 
              << " GR_V_" << clPin.getMrc() << "))" << endl;
          }
        }
      }
    }
  }

  myFile << "; 2.2.9. Tip-to-Tip Spacing Rule" << endl;
  for(auto& clipLayer : clip_layers_) {
    if( clipLayer->layer()->getRoutingLevel() >= from_route_layer_ && 
        clipLayer->layer()->getRoutingLevel() <= to_route_layer_ ) { 
      auto layerDir = clipLayer->layer()->getDirection();
      if( layerDir == odb::dbTechLayerDir::VERTICAL) {
        for(int y=0; y<= clipLayer->gridY().size()-2; y++) {
          for(int x=0; x<=clipLayer->gridX().size()-1; x++) {
            ClipLayerPin* prevPin = clipLayer->clipLayerPin(x, y);
            ClipLayerPin* nextPin = clipLayer->clipLayerPin(x, y+1);
            
            myFile << "(assert ((_ at-most 1) GU_V_" << prevPin->getMrc()
              << " GD_V_" << nextPin->getMrc() << "))" << endl;
          }
        }
      }
      else if( layerDir == odb::dbTechLayerDir::HORIZONTAL) {
        for(int y=0; y<= clipLayer->gridY().size()-1; y++) {
          for(int x=0; x<=clipLayer->gridX().size()-2; x++) {
            ClipLayerPin* prevPin = clipLayer->clipLayerPin(x, y);
            ClipLayerPin* nextPin = clipLayer->clipLayerPin(x+1, y);
            
            myFile << "(assert ((_ at-most 1) GR_V_" << prevPin->getMrc()
               << " GL_V_" << nextPin->getMrc() << "))" << endl;
          }
        }
      }
    }
  }

  myFile << "; 2.2.10. Via Enclosure - no standalone via is allowed. " << endl;
  myFile << "; This will also prevent M2 min-area-rule(MAR)"  <<  endl;

  for(auto& coordi : coordi_info_stor_) {
    // local segment stor variable in each coordi object
    // VIA12, VIA23, VIA34 must be considered for this rule
    //
    std::vector<std::set<string>> curSegmentStor;
    curSegmentStor.resize(SMT2Segment::maxSegmentOrder, std::set<string>());

    for(auto& rNet : rnets_) {
      if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
        continue;
      }

      // via enclosure mode
      for(auto& str : coordi.getAllStrings(&rNet, true)) {
        curSegmentStor[str.intOrder()].insert(str.str());
      }
    }

    for(auto& str : coordi.getAdjStrings()) { 
      // update allAdjSegmentsSet
      curSegmentStor[str.intOrder()].insert(str.str());
    }


    // VIA Enclosure writing
    // note that 1 3 5 7 9 is VIA12, VIA23, VIA34, VIA45, ...
    // 2*maxRouteLayer  == last upper layer element.
    // (e.g., metal3 -> index 6 (3*2) is M4)
    // (e.g., metal3 -> index 5 (3*2-1) is VIA34)
    for(int i=1; i<=2*to_route_layer_; i+=2) {
      if( curSegmentStor[i].size() >= 1 ) {
        // lower metal layer consideration
        if( i != 1 
            && curSegmentStor[i-1].size() >= 1 ) {
          for(auto& seg1: curSegmentStor[i]) {
            myFile << "(assert (ite (= M_" << seg1 << " true ) "; 
            myFile << "((_ at-least 1) ";
            for(auto& seg2: curSegmentStor[i-1]) {
              myFile << "M_" << seg2 << " "; 
            }
            myFile << ") (= 1 1)))"  << endl;
          }
        }

        // upper metal layer consideration
        // if "i" is not the maximum VIA layer 
        // (note that, metal3 -> index 5 (3*2-1) is VIA34)
        if( i != 2*to_route_layer_-1 
            && curSegmentStor[i+1].size() >= 1 ) {
          for(auto& seg1: curSegmentStor[i]) {
            myFile << "(assert (ite (= M_" << seg1 << " true ) "; 
            myFile << "((_ at-least 1) ";
            for(auto& seg2: curSegmentStor[i+1]) {
              myFile << "M_" << seg2 << " "; 
            }
            myFile << ") (= 1 1)))"  << endl;
          }
        }
      }
    }
  }

  myFile << "; 2.2.11. Boundary routing blockage - no routing outside of coreBox is allowed. " << endl;

  for(auto& clipLayer: clip_layers_) {
    if( clipLayer->layer()->getRoutingLevel() >= from_route_layer_ && 
        clipLayer->layer()->getRoutingLevel() <= to_route_layer_ ) { 
      auto layerDir = clipLayer->layer()->getDirection();
      int layerNum = clipLayer->layer()->getRoutingLevel();
      odb::Rect rect = clipLayer->gridCoreIdx();

      if( layerDir == odb::dbTechLayerDir::VERTICAL) {
        for(int i=rect.xMin(); i<=rect.xMax(); i++) {
          string key = "m" + to_string(layerNum) 
            + "r" + to_string(rect.yMin()-1) + "c" + to_string(i) 
            + "_m" + to_string(layerNum)
            + "r" + to_string(rect.yMin()) + "c" + to_string(i);

          if( allSegmentsSet.find(key) != allSegmentsSet.end() ) {
            myFile << "(assert (= M_" << key << " false))" << endl;
          }

          key = "m" + to_string(layerNum) 
            + "r" + to_string(rect.yMax()) + "c" + to_string(i) 
            + "_m" + to_string(layerNum)
            + "r" + to_string(rect.yMax()+1) + "c" + to_string(i);

          if( allSegmentsSet.find(key) != allSegmentsSet.end() ) {
            myFile << "(assert (= M_" << key << " false))" << endl;
          }
        } 
      }
      else if( layerDir == odb::dbTechLayerDir::HORIZONTAL) {
        for(int i=rect.yMin(); i<=rect.yMax(); i++) {
          string key = "m" + to_string(layerNum) 
            + "r" + to_string(i) + "c" + to_string(rect.xMin()-1) 
            + "_m" + to_string(layerNum)
            + "r" + to_string(i) + "c" + to_string(rect.xMin());

          if( allSegmentsSet.find(key) != allSegmentsSet.end() ) {
            myFile << "(assert (= M_" << key << " false))" << endl;
          }

          key = "m" + to_string(layerNum) 
            + "r" + to_string(i) + "c" + to_string(rect.xMax()) 
            + "_m" + to_string(layerNum)
            + "r" + to_string(i) + "c" + to_string(rect.xMax()+1);

          if( allSegmentsSet.find(key) != allSegmentsSet.end() ) {
            myFile << "(assert (= M_" << key << " false))" << endl;
          }
        } 
      }
    }
  }

  myFile << "; 2.2.12. Instance routing pin blockage - Cannot route the metal - instance's pin already occupied" << endl;

  std::set<std::string> fixedSegs;
  for(auto& inst: fixed_insts_) {
    // get instance lx ly ux uy
    odb::dbBox* iBox = inst->getBBox();
    int gLx = (iBox->xMin() - site_lower_x_) / static_cast<int>(site->getWidth());
    int gLy = (iBox->yMin() - site_lower_y_) / static_cast<int>(site->getHeight());

    // get flipX/Y of fixed instance
    // see LEF/DEF manual
    bool isFlipX = false,
         isFlipY = false;

    switch(inst->getOrient()) {
      case odb::dbOrientType::R0:
        isFlipX = isFlipY = false;  
        break;
      case odb::dbOrientType::R180:
        isFlipX = isFlipY = true;
        break;
      case odb::dbOrientType::MX:
        isFlipX = false;
        isFlipY = true;
        break;
      case odb::dbOrientType::MY:
        isFlipX = true;
        isFlipY = false;
        break;
      default:
        break;
    }

    // update fixedSegs
    getInstRoutingPinBlockageSegs(inst, gLx, gLy, isFlipX, isFlipY, fixedSegs);
  }
  // write out
  for(auto& seg : fixedSegs) {
    if( allSegmentsSet.find(seg) != allSegmentsSet.end() ) {
      myFile << "(assert (= M_" << seg << " false))" << endl;
    }
  }

  // movable instance awareness -> ite
  for(auto& inst : movable_insts_) {
    // get instance lx ly ux uy
    odb::dbBox* iBox = inst->getBBox();
    int instSiteWidth = (iBox->getDX()) 
      / static_cast<int>(site->getWidth());

    // get all possible x/y coordinates
    for(auto& dpPairsY : dp_grid_place_pairs_) {
      int y = &dpPairsY - &dp_grid_place_pairs_[0];

      for(auto& dpPairX : dpPairsY) {
        // can be placed, within this row (row width const)
        if( dpPairX.second - dpPairX.first + 1 >= instSiteWidth ) {

          // possible x loc
          for(int x=dpPairX.first; 
              x<=dpPairX.second - instSiteWidth + 1; 
              x++) {
            // all flip variable
            for(int f = 0; f<2; f++) {
              bool isFlipX = static_cast<bool>(f);
              bool isFlipY = (dp_grid_[y][x].orient() != odb::dbOrientType::R0 );

              // update moveableSegs 
              std::set<std::string> movableSegs;
              getInstRoutingPinBlockageSegs(inst, x, y, isFlipX, isFlipY, movableSegs);
              
              std::set<std::string> filteredMovableSegs;
              for(auto& seg : movableSegs) {
                if( allSegmentsSet.find(seg) != allSegmentsSet.end() ) { 
                  filteredMovableSegs.insert(seg);
                }
              } 

              // ite with (y, x, f) pair
              if( filteredMovableSegs.size() > 0 ) {
                string instName = inst->getConstName();
                string xInst = "x_" + instName;
                string rInst = "r_" + instName;
                string fInst = "ff_" + instName;
                string fString = (f == 0)? "false" : "true";

                myFile << "(assert (ite (and (= " 
                  << xInst << " (_ bv" << x << " 9)) (= " 
                  << rInst << " (_ bv" << y << " 4)) (= "
                  << fInst << " " << fString << ")) (and ";

                for(auto& seg : filteredMovableSegs) {
                  myFile << "(= " << seg << " false) ";
                }
                myFile << ") (= 1 1)))" << endl;
              }
            }
          }
        }
      }
    }
  }

  myFile << "; 2.2.13. SLO violations" << endl;

  for(auto& clipLayer: clip_layers_) {
    if( clipLayer->layer()->getRoutingLevel() >= from_route_layer_ && 
        clipLayer->layer()->getRoutingLevel() <= to_route_layer_ ) { 
      writeSMT2SloConstraints(myFile, allSegmentsSet, clipLayer);
    }
  }
  
  myFile << endl;
  

  myFile << "; 3. Optimization Objective" << endl;
  myFile << "; 3.1. y-axis(row) displacement" << endl;

  if( movable_insts_.size() >= 1 ) {
    myFile << "(minimize (bvadd ";
    for(auto& inst : movable_insts_) {
      // get grid (lx, ly, ux uy)
      odb::dbBox* iBox = inst->getBBox();

      int gLy = (iBox->yMin() - site_lower_y_) 
        / static_cast<int>(site->getHeight());

      string rVar = "r_" + inst->getName();
      string targetVar = "(_ bv" + to_string(gLy) + " 4)";

      myFile << "(ite (bvsge " << rVar << " " << targetVar << ") "
        << " (bvsub " << rVar << " " << targetVar << " ) "
        << " (bvsub " << targetVar << " " << rVar << ")) "; 
    }
    myFile << "))" << endl;
  }

  myFile << "; 3.2. x-axis(site) displacement" << endl;

  if( movable_insts_.size() >= 1 ) {
    myFile << "(minimize (bvadd ";
    for(auto& inst : movable_insts_) {
      // get grid (lx, ly, ux uy)
      odb::dbBox* iBox = inst->getBBox();

      int gLx = (iBox->xMin() - site_lower_x_) 
        / static_cast<int>(site->getWidth());

      string xVar = "x_" + inst->getName();
      string targetVar = "(_ bv" + to_string(gLx) + " 9)";

      myFile << "(ite (bvsge " << xVar << " " << targetVar << ") "
        << " (bvsub " << xVar << " " << targetVar << " ) "
        << " (bvsub " << targetVar << " " << xVar << ")) "; 
    }
    myFile << "))" << endl;
  }

  myFile << "; 3.3. flip displacement" << endl;
  if( movable_insts_.size() >= 1 ) {
    myFile << "(minimize (bvadd ";
    for(auto& inst : movable_insts_) {
      odb::dbBox* iBox = inst->getBBox();

      int gLy = (iBox->yMin() - site_lower_y_) 
        / static_cast<int>(site->getHeight());

      string fVar = "ff_" + inst->getName();
      string trueVar = "", falseVar = "";

      if( dp_grid_[gLy][0].orient() == odb::dbOrientType::R0 ) {
        if( inst->getOrient() == odb::dbOrientType::R0 ) {
          trueVar = "(_ bv1 6)";
          falseVar = "(_ bv0 6)";
        }
        else if( inst->getOrient() == odb::dbOrientType::MY ) {
          trueVar = "(_ bv0 6)";
          falseVar = "(_ bv1 6)";
        }
      }
      else if( dp_grid_[gLy][0].orient() == odb::dbOrientType::MX ) {
        if( inst->getOrient() == odb::dbOrientType::MX ) {
          trueVar = "(_ bv1 6)";
          falseVar = "(_ bv0 6)";
        }
        else if( inst->getOrient() == odb::dbOrientType::R180 ) {
          trueVar = "(_ bv0 6)";
          falseVar = "(_ bv1 6)";
        }
      }

      myFile << "(ite (= " << fVar << " true) " << trueVar << " " << falseVar << ") ";
    }
    myFile << "))" << endl;
  } 
  myFile << "; 3.4. M1 - VIA12 - M2 - VIA23 - M3 - VIA34 - M4 order" << endl;
  for(auto& strs: totalSegmentsStor) {
    int curIdx = &strs - &totalSegmentsStor[0];
    myFile << "; 3.4." << curIdx+1 << " case" << endl;

    int cutUnit = 128;
    int printed = 0;
    bool isFirst = true;
    
    for(auto& str : strs) {
      if( isFirst ) {
        myFile << "(minimize (bvadd ";
        isFirst = false;
      }

      if (printed >= cutUnit) {
        myFile << "))" << endl;
        myFile << "(minimize (bvadd ";
        printed = 0;
      }
      myFile << "(ite (= M_" << str << " true) (_ bv1 32) (_ bv0 32)) ";
      printed++;
    }
    if(strs.size() >= 1) {
      myFile << "))" << endl;
    }
  }
  
  myFile << "(check-sat)" << endl 
    << "(get-model)" << endl 
    << "(get-objectives)" << endl;

  myFile.close();

  logger->info(ECO, 5232, "Finished writing the SMT2 file: {}", fileName); 
}

void
Clip::updateInstRoutingPinBlockage(odb::dbInst* inst,
    int gLx, int gLy, 
    bool isFlipX, bool isFlipY) {

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  int itermLayerPitchX = iterm_layer_->gridX()[1] - iterm_layer_->gridX()[0];
  int itermLayerPitchY = iterm_layer_->gridY()[1] - iterm_layer_->gridY()[0];

  int pitchSiteWidth = site->getWidth() / itermLayerPitchX;
  int pitchSiteHeight = site->getHeight() / itermLayerPitchY;

  int pitchMarginX = (site_lower_x_ - die_.xMin()) / itermLayerPitchX;
  int pitchMarginY = (site_lower_y_ - die_.yMin()) / itermLayerPitchY;

  eco::Master* ecoMaster = dbToEb(inst->getMaster());

  int pitchMasterWidth = ecoMaster->width() / itermLayerPitchX; 
  int pitchMasterHeight = ecoMaster->height() / itermLayerPitchY;

  // for both pin and OBS
  for(auto& ecoPin : ecoMaster->pins()) {
    for(auto& pc : ecoPin.pinCoordis()) {
      // if ecoPin is within routing layer range
      odb::dbTechLayer* layer = pc.dbTechLayer();
      eco::ClipLayer* cLayer = dbToEb(layer);

      if( layer->getRoutingLevel() >= from_route_layer_ &&
          layer->getRoutingLevel() <= to_route_layer_ ) {
        if( layer->getDirection() == odb::dbTechLayerDir::VERTICAL ) {
          // // pc.lx() == pc.ux()
          // int32_t newX = (isFlipX)? 
          //   (pitchMasterWidth-1) - pc.lx() : pc.lx() ;

          // newX += gLx * pitchSiteWidth + pitchMarginX;

          // // within grid indices
          // if( 0 <= newX && newX <= cLayer->gridX().size()-1) {
          //   int32_t newLY = (isFlipY)? 
          //     (pitchMasterHeight-1) - pc.uy() : pc.ly();
          //   int32_t newUY = (isFlipY)? 
          //     (pitchMasterHeight-1) - pc.ly() : pc.uy();

          //   newLY += gLy * pitchSiteHeight + pitchMarginY;
          //   newUY += gLy * pitchSiteHeight + pitchMarginY;

          //   // ly/uy
          //   // within grid indices
          //   for(int i=std::max(newLY,0); 
          //       i<=std::min(newUY,static_cast<int>(cLayer->gridY().size()-2)); i++) {
          //     // add (x, i) ~ (x,i+1) vertical segment
          //     // y is ROW
          //     std::string seg = 
          //       "m" + to_string(layer->getRoutingLevel()) 
          //       + "r" + to_string(i) 
          //       + "c" + to_string(newX) + "_"
          //       + "m" + to_string(layer->getRoutingLevel())
          //       + "r" + to_string(i+1)
          //       + "c" + to_string(newX);
          //     // add seg
          //   }
          // }
        }
        else if( layer->getDirection() == odb::dbTechLayerDir::HORIZONTAL ) {
          // pc.ly() == pc.uy()
          int32_t newY = (isFlipY)? 
            (pitchMasterHeight-1) - pc.ly() : pc.ly() ;
          newY += gLy * pitchSiteHeight + pitchMarginY; 

          // within grid indices
          if( 0 <= newY && newY <= cLayer->gridY().size()-1) {
            int32_t newLX = (isFlipX)? 
              (pitchMasterWidth-1) - pc.ux() : pc.lx() ;

            int32_t newUX = (isFlipX)? 
              (pitchMasterWidth-1) - pc.lx() : pc.ux() ;

            newLX += gLx * pitchSiteWidth + pitchMarginX;
            newUX += gLx * pitchSiteWidth + pitchMarginX;

            newLX = std::max(newLX, 0);
            newUX = std::min(newUX, 
                static_cast<int>(cLayer->gridX().size()-1));

            // ly/uy
            // within grid indices
            for(int i=newLX; i<=newUX; i++) {
              // add (x, i) ~ (x,i+1) hori segment
              // x is COL 
              ClipLayerPin* clPin
               = cLayer->clipLayerPin(i, newY);
              if( i != newLX ) {
                clPin->setHasObsLeft(true);
              } 
              if( i != newUX ) {
                clPin->setHasObsRight(true);
              }
            }
          }
        }
      }
    }
  }
}

void
Clip::getInstRoutingPinBlockageSegs(odb::dbInst* inst,
    int gLx, int gLy, 
    bool isFlipX, bool isFlipY,
    std::set<std::string>& fixedSegs) {

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  int itermLayerPitchX = iterm_layer_->gridX()[1] - iterm_layer_->gridX()[0];
  int itermLayerPitchY = iterm_layer_->gridY()[1] - iterm_layer_->gridY()[0];

  int pitchSiteWidth = site->getWidth() / itermLayerPitchX;
  int pitchSiteHeight = site->getHeight() / itermLayerPitchY;

  int pitchMarginX = (site_lower_x_ - die_.xMin()) / itermLayerPitchX;
  int pitchMarginY = (site_lower_y_ - die_.yMin()) / itermLayerPitchY;

  eco::Master* ecoMaster = dbToEb(inst->getMaster());

  int pitchMasterWidth = ecoMaster->width() / itermLayerPitchX; 
  int pitchMasterHeight = ecoMaster->height() / itermLayerPitchY;

  // for both pin and OBS
  for(auto& ecoPin : ecoMaster->pins()) {
    for(auto& pc : ecoPin.pinCoordis()) {
      // if ecoPin is within routing layer range
      odb::dbTechLayer* layer = pc.dbTechLayer();
      eco::ClipLayer* cLayer = dbToEb(layer);

      if( layer->getRoutingLevel() >= from_route_layer_ &&
          layer->getRoutingLevel() <= to_route_layer_ ) {
        if( layer->getDirection() == odb::dbTechLayerDir::VERTICAL ) {
          // pc.lx() == pc.ux()
          int32_t newX = (isFlipX)? 
            (pitchMasterWidth-1) - pc.lx() : pc.lx() ;

          newX += gLx * pitchSiteWidth + pitchMarginX;

          // within grid indices
          if( 0 <= newX && newX <= cLayer->gridX().size()-1) {
            int32_t newLY = (isFlipY)? 
              (pitchMasterHeight-1) - pc.uy() : pc.ly();
            int32_t newUY = (isFlipY)? 
              (pitchMasterHeight-1) - pc.ly() : pc.uy();

            newLY += gLy * pitchSiteHeight + pitchMarginY;
            newUY += gLy * pitchSiteHeight + pitchMarginY;

            // ly/uy
            // within grid indices
            for(int i=std::max(newLY,0); 
                i<=std::min(newUY,static_cast<int>(cLayer->gridY().size()-2)); i++) {
              // add (x, i) ~ (x,i+1) vertical segment
              // y is ROW
              std::string seg = 
                "m" + to_string(layer->getRoutingLevel()) 
                + "r" + to_string(i) 
                + "c" + to_string(newX) + "_"
                + "m" + to_string(layer->getRoutingLevel())
                + "r" + to_string(i+1)
                + "c" + to_string(newX);
              // add seg
              fixedSegs.insert(seg);
            }
          }
        }
        else if( layer->getDirection() == odb::dbTechLayerDir::HORIZONTAL ) {
          // pc.ly() == pc.uy()
          int32_t newY = (isFlipY)? 
            (pitchMasterHeight-1) - pc.ly() : pc.ly() ;
          newY += gLy * pitchSiteHeight + pitchMarginY; 

          // within grid indices
          if( 0 <= newY && newY <= cLayer->gridY().size()-1) {
            int32_t newLX = (isFlipX)? 
              (pitchMasterWidth-1) - pc.ux() : pc.lx() ;

            int32_t newUX = (isFlipX)? 
              (pitchMasterWidth-1) - pc.lx() : pc.ux() ;

            newLX += gLx * pitchSiteWidth + pitchMarginX;
            newUX += gLx * pitchSiteWidth + pitchMarginX;

            // ly/uy
            // within grid indices
            for(int i=std::max(newLX,0); 
                i<=std::min(newUX,static_cast<int>(cLayer->gridX().size()-2)); i++) {

              // add (x, i) ~ (x,i+1) hori segment
              // x is COL 
              std::string seg = 
                "m" + to_string(layer->getRoutingLevel()) 
                + "r" + to_string(newY) 
                + "c" + to_string(i) + "_"
                + "m" + to_string(layer->getRoutingLevel())
                + "r" + to_string(newY)
                + "c" + to_string(i+1);
              // add seg
              fixedSegs.insert(seg);
            }
          }
        }
      }
    }
  }
}

void
Clip::writeSMT2GeomConsts(std::ofstream& out,
    int mode,
    ClipLayer& clipLayer) { 

  // without route layer
  if( !(clipLayer.layer()->getRoutingLevel() >= from_route_layer_ && 
        clipLayer.layer()->getRoutingLevel() <= to_route_layer_) ) {
    return;
  }

  auto layerDir = clipLayer.layer()->getDirection();

  // left/right
  if( mode == 0 || mode == 1 ) {
    if( layerDir != odb::dbTechLayerDir::HORIZONTAL ) { 
      return;
    }
  }

  // down/up 
  if( mode == 2 || mode == 3 ) {
    if( layerDir != odb::dbTechLayerDir::VERTICAL ) { 
      return;
    }
  }

  for(auto& gridY : clipLayer.grid()) {
    for(auto& clPin : gridY) {
      ClipLayerPin* prevClPin = nullptr;
      ClipLayerPin* nextClPin = nullptr;

      string prevKey = "", nextKey = "";
      string modeStr = "";

      bool prevPinExist = false;
      bool nextPinExist = false;

      bool prevPinObs = false;
      bool nextPinObs = false;

      switch(mode) {
        // left
        case 0:
          modeStr = "GL";
          prevClPin = clipLayer.clipLayerPin(clPin.x()-1, clPin.y());
          nextClPin = clipLayer.clipLayerPin(clPin.x()+1, clPin.y()); 
          if( prevClPin ) {
            prevKey = prevClPin->getMrc() + "_" + clPin.getMrc(); 
            prevPinExist = !prevClPin->hasObs() && !clPin.hasObs();
            if( !prevPinExist ) {
              prevPinObs = prevClPin->hasObsRight() || clPin.hasObsLeft();
            }
          }
          if( nextClPin ) {
            nextKey = clPin.getMrc() + "_" + nextClPin->getMrc();
            nextPinExist = !clPin.hasObs() && !nextClPin->hasObs();
            if( !nextPinExist ) {
              nextPinObs = clPin.hasObsRight() || nextClPin->hasObsLeft();
            }
          }
          break;

        // right
        case 1:
          modeStr = "GR";
          prevClPin = clipLayer.clipLayerPin(clPin.x()+1, clPin.y());
          nextClPin = clipLayer.clipLayerPin(clPin.x()-1, clPin.y());

          if( prevClPin ) {
            prevKey = clPin.getMrc() + "_" + prevClPin->getMrc(); 
            prevPinExist = !clPin.hasObs() && !prevClPin->hasObs();
            if( !prevPinExist ) {
              prevPinObs = clPin.hasObsRight() || prevClPin->hasObsLeft();
            }
          }
          if( nextClPin ) {
            nextKey = nextClPin->getMrc() + "_" + clPin.getMrc();
            nextPinExist = !nextClPin->hasObs() && !clPin.hasObs();
            if( !nextPinExist ) {
              nextPinObs = nextClPin->hasObsRight() || clPin.hasObsLeft();
            }
          }
          break;

        // down
        case 2:
          modeStr = "GD";
          prevClPin = clipLayer.clipLayerPin(clPin.x(), clPin.y()-1);
          nextClPin = clipLayer.clipLayerPin(clPin.x(), clPin.y()+1); 
          if( prevClPin ) {
            prevKey = prevClPin->getMrc() + "_" + clPin.getMrc(); 
            prevPinExist = !prevClPin->hasObs() && !clPin.hasObs();
            if( !prevPinExist ) {
              prevPinObs = prevClPin->hasObsUp() || clPin.hasObsDown();
            }
          }
          if( nextClPin ) {
            nextKey = clPin.getMrc() + "_" + nextClPin->getMrc();
            nextPinExist = !clPin.hasObs() && !nextClPin->hasObs();
            if( !nextPinExist ) {
              nextPinObs = clPin.hasObsUp() || nextClPin->hasObsDown();
            }
          }
          break;

        // up
        case 3:
          modeStr = "GU";
          prevClPin = clipLayer.clipLayerPin(clPin.x(), clPin.y()+1);
          nextClPin = clipLayer.clipLayerPin(clPin.x(), clPin.y()-1);

          if( prevClPin ) {
            prevKey = clPin.getMrc() + "_" + prevClPin->getMrc(); 
            prevPinExist = !clPin.hasObs() && !prevClPin->hasObs();
            if( !prevPinExist ) {
              prevPinObs = clPin.hasObsUp() || prevClPin->hasObsDown();
            }
          }
          if( nextClPin ) {
            nextKey = nextClPin->getMrc() + "_" + clPin.getMrc();
            nextPinExist = !nextClPin->hasObs() && !clPin.hasObs();
            if( !nextPinExist ) {
              nextPinObs = nextClPin->hasObsUp() || clPin.hasObsDown();
            }
          }
          break;

        default:
          break;
      }

      if( prevClPin && prevPinExist ) {
        out << "(assert ((_ at-most 1) " << modeStr << "_V_" << clPin.getMrc() 
          << " M_" << prevKey << "))" << endl;
      }
      else if( prevClPin && !prevPinExist ) {
      // means M_{prevKey} is always true -- occupied
      // geometry variable cannot be true -> must be false
        if( prevPinObs ) {
          out << "(assert (= " << modeStr << "_V_" << clPin.getMrc() << " false))" << endl;
        }
      }

      if( nextClPin && nextPinExist ) {  
          out << "(assert (ite (= " << modeStr << "_V_" << clPin.getMrc() 
            << " true) (= M_" << nextKey << " true) (= 1 1)))" << endl;

        // nextKey formulation
        if( prevClPin && prevPinExist ) {
          out << "(assert (ite (= (or " << modeStr << "_V_" << clPin.getMrc()
            << " M_" << prevKey << ") false) (= M_" 
            << nextKey << " false) (= 1 1)))" << endl;
        }
        else if( prevClPin && !prevPinExist ) {
          // not occupied -> opened
          if( !prevPinObs ) {
            out << "(assert (ite (= " << modeStr << "_V_" << clPin.getMrc()
              << " false) (= M_" 
              << nextKey << " false) (= 1 1)))" << endl;
          }
          // else means M_{prevKey} is true -> ite condition () cannot be true 
          // -> ignore printing
        }
      }

      // not exists, 
      // nextKey of M_{fromLoc}_{toLoc} is false 
      // -> ignore ite (if-true-else)
      
      // special cases - GEOM must be true in this case
      if( prevClPin && nextClPin 
          && !prevPinExist && !nextPinExist 
          && !prevPinObs && nextPinObs ) {
        // escape corner cases -- this is EXT PIN is placed, so ignore geom var 
        if( !( (clPin.hasObsLeft() && clPin.hasObsRight()) || 
              (clPin.hasObsDown() && clPin.hasObsUp()) ) ) {
          out << "(assert (= " << modeStr << "_V_" << clPin.getMrc() << " true))" << endl;
        }
      }

      if( prevClPin && nextClPin
          && prevPinExist && !nextPinExist
          && nextPinObs ) {
        out << "(assert ((_ at-least 1) " << modeStr << "_V_" << clPin.getMrc() 
          << " M_" << prevKey << "))" << endl;
      }
    }
  }
}


void 
Clip::writeSMT2SloConstraintsPattern(std::ofstream& out,
        std::set<std::string>& allSegmentsSet,
        eco::ClipLayer* clipLayer, 
        int* patterns, int x, int y, int z) {

  odb::Rect idxRect = clipLayer->gridCoreIdx();
  int cIdx = clipLayer->layer()->getRoutingLevel();

  auto layerDir = clipLayer->layer()->getDirection();
  if( layerDir == odb::dbTechLayerDir::HORIZONTAL ) {
    if( idxRect.xMax() - idxRect.xMin() < z ||
        idxRect.yMax() - idxRect.yMin() < y) {
      logger->warn(ECO, 3572, 
          "SLO cannot be considered because window is too small");
      return;
    } 
  }
  else if( layerDir == odb::dbTechLayerDir::VERTICAL ) {
    if( idxRect.xMax() - idxRect.xMin() < y ||
        idxRect.yMax() - idxRect.yMin() < z) {
      logger->warn(ECO, 3573, 
          "SLO cannot be considered because window is too small");
      return;
    }
  }


  // for each pattern
  for(int i=0; i<x; i++) {
    bool isFailed = false;

    out << ";SLO pattern " << i << endl;

    if( layerDir == odb::dbTechLayerDir::HORIZONTAL ) {
      for(int k=idxRect.yMin(); k <= idxRect.yMax() - y; k++) {
        for(int j=idxRect.xMin(); j <= idxRect.xMax() - z; j++) {
          // accumStr: metal segment
          std::vector<std::string> accumStr;
          // onOff: store pattern as 1D array
          std::vector<int> onOff;

          // check if pattern is okay
          for(int l=j; l < j+z; l++) {
            for(int m=k; m < k+y; m++) {

              // patternIndex i, 
              // y Idx m-k : 0~5
              // x Idx l-j : 0~4
              std::string mrc = "m" + to_string(cIdx) 
                + "r" + to_string(m)
                + "c" + to_string(l) 
                + "_m" + to_string(cIdx)
                + "r" + to_string(m)
                + "c" + to_string(l+1);
              // the mrc doesn't exist -> skip for this pattern
              if( allSegmentsSet.find(mrc) == allSegmentsSet.end()) {
                isFailed = true;
                break;
              }

              onOff.push_back(patterns[ i*(y*z) + (m-k)*z + (l-j) ]);
              accumStr.push_back(mrc); 
            }

            if( isFailed ) { 
              break;
            }
          } 

          // means all segments are alive
          if( isFailed == false && onOff.size() >= 1 ) {
            out << "(assert (= (and ";
            for(int p=0; p<onOff.size(); p++) {
              if( onOff[p] == 1 ) {
                out << "M_" << accumStr[p] << " ";
              }
              else {
                out << "( not M_" << accumStr[p] << ") ";
              }
            }
            out << ") false))" << endl;
          }
        }
      }
    }
    else if( layerDir == odb::dbTechLayerDir::VERTICAL ) {
      for(int k=idxRect.yMin(); k <= idxRect.yMax() - z; k++) {
        for(int j=idxRect.xMin(); j <= idxRect.xMax() - y; j++) {
          // accumStr: metal segment
          std::vector<std::string> accumStr;
          // onOff: store pattern as 1D array
          std::vector<int> onOff;

          // check if pattern is okay
          for(int l=j; l < j+y; l++) {
            for(int m=k; m < k+z; m++) {

              // patternIndex i, 
              // y Idx m-k : 0~5
              // x Idx l-j : 0~4
              std::string mrc = "m" + to_string(cIdx) 
                + "r" + to_string(m)
                + "c" + to_string(l) 
                + "_m" + to_string(cIdx)
                + "r" + to_string(m+1)
                + "c" + to_string(l);
              // the mrc doesn't exist -> skip for this pattern
              if( allSegmentsSet.find(mrc) == allSegmentsSet.end()) {
                isFailed = true;
                break;
              }

              onOff.push_back(patterns[ i*(y*z) + (l-j)*z + (m-k) ]);
              accumStr.push_back(mrc); 
            }

            if( isFailed ) { 
              break;
            }
          } 

          // means all segments are alive
          if( isFailed == false && onOff.size() >= 1 ) {
            out << "(assert (= (and ";
            for(int p=0; p<onOff.size(); p++) {
              if( onOff[p] == 1 ) {
                out << "M_" << accumStr[p] << " ";
              }
              else {
                out << "( not M_" << accumStr[p] << ") ";
              }
            }
            out << ") false))" << endl;
          }
        }
      }
    }
  }
}

void
Clip::writeSMT2SloConstraints(std::ofstream& out,
        std::set<std::string>& allSegmentsSet,
        eco::ClipLayer* clipLayer) {

  int pattern1[] = {
    1, 1, 1, 1, 
    1, 1, 0, 0, 
    1, 1, 1, 0, 
    1, 0, 0, 1, 
    1, 1, 1, 1, 
    // x flip
    1, 1, 1, 1, 
    0, 0, 1, 1, 
    0, 1, 1, 1, 
    1, 0, 0, 1, 
    1, 1, 1, 1, 
    // y flip
    1, 1, 1, 1, 
    1, 0, 0, 1, 
    1, 1, 1, 0, 
    1, 1, 0, 0, 
    1, 1, 1, 1, 
    // xy flip
    1, 1, 1, 1, 
    1, 0, 0, 1, 
    0, 1, 1, 1, 
    0, 0, 1, 1, 
    1, 1, 1, 1};


  writeSMT2SloConstraintsPattern(out, allSegmentsSet, clipLayer, (int*)pattern1, 4, 5, 4);

  int pattern2[] = {
    // original pattern
    1, 1, 1, 1, 
    1, 0, 0, 1, 
    1, 1, 1, 1, 
    1, 0, 0, 1, 
    1, 1, 1, 1};

  writeSMT2SloConstraintsPattern(out, allSegmentsSet, clipLayer, (int*)pattern2, 1, 5, 4);
}

void
Clip::writeSMT2Helper(std::string fileName) {
  std::ofstream myFile;
  myFile.open(fileName);

  myFile << die_.xMin() << " " << die_.yMin() << " " 
    << die_.xMax() << " " << die_.yMax() << endl;
  
  myFile << core_.xMin() << " " << core_.yMin() << " " 
    << core_.xMax() << " " << core_.yMax() << endl;

  // route grid dump
  myFile << from_route_layer_ << " " << to_route_layer_ << " " << clip_layer_stor_.size() << endl;
  for(auto& clipLayer : clip_layers_) {
    myFile << 
      clipLayer->layer()->getRoutingLevel() << " " 
      << clipLayer->layer()->getConstName() << " " 
      << clipLayer->gridX()[0] << " "
      << clipLayer->gridY()[0] << " " 
      << clipLayer->gridX()[1] - clipLayer->gridX()[0] << " "
      << clipLayer->gridY()[1] - clipLayer->gridY()[0] << " " 
      << clipLayer->gridX().size() << " "
      << clipLayer->gridY().size() << endl;
  } 

  // site dump
  // retrieve row/site
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();
  myFile << site->getWidth() << " " << site->getHeight() << endl;

  // dp grid dump
  if( dp_grid_.size() > 0) {
    myFile << dp_grid_.size() << " " <<  dp_grid_[0].size() << " " 
      << dp_grid_[0][0].dbuLx() << " " << dp_grid_[0][0].dbuLy() << endl;

    // all orient
    for(auto& row: dp_grid_) {
      myFile << row[0].printOrient() << " ";
    }
    myFile << endl;
  }
  else {
    myFile << "0 0 0 0" << endl; 
    myFile << "NONE" << endl;
  }

  myFile << rnets_.size() << endl;
  for(auto& rNet: rnets_) {
    if(!rNet.isFeasible(from_route_layer_, to_route_layer_)) {
      continue;
    }

    myFile << "NET " << rNet.getName() << " " << rNet.getCommodityCnt() << endl;
    for(int i=0; i<rNet.getCommodityCnt(); i++) {
      myFile << "  " << i 
        << " " << rNet.commodityOutPin(i)->getVertName() 
        << " " << rNet.commodityInPin(i)->getVertName() << endl;
    }
  }

  myFile.close();
}


Clip::EcoNode::EcoNode()
  : x(0), y(0),
  layer(nullptr), iterm(nullptr) {};

Clip::EcoNode::EcoNode(int inX, int inY, odb::dbTechLayer* inLayer) 
  : x(inX), y(inY), layer(inLayer), iterm(nullptr) {};

Clip::EcoNode::EcoNode(int inX, int inY, odb::dbTechLayer* inLayer, odb::dbITerm* inITerm) 
  : x(inX), y(inY), layer(inLayer), iterm(inITerm) {};

Clip::EcoNode::EcoNode(ClipLayerPin* clPin)
  : x(clPin->dbuX()), y(clPin->dbuY()), 
  layer(clPin->layer()), iterm(clPin->iTerm()) {};

std::tuple<int, int, int> 
Clip::EcoNode::getKey() {
  return std::tuple<int, int, int>(x, y, layer->getRoutingLevel());
}

Clip::EcoEdge::EcoEdge()
  : type(0), 
  techVia(nullptr), 
  via(nullptr), 
  coreMark(eco::OverlapMark::NONE),
  dieMark(eco::OverlapMark::NONE) {}

Clip::EcoEdge::EcoEdge(
    ClipLayerPin* prevClPin, 
    ClipLayerPin* nextClPin,
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& viaMap)
  : type(0),
  techVia(nullptr),
  via(nullptr),
  coreMark(eco::OverlapMark::NONE),
  dieMark(eco::OverlapMark::NONE) {
  
  odb::dbTechLayer* prevLayer = prevClPin->layer(), 
  *nextLayer = nextClPin->layer();

  // TECH VIA
  if( prevLayer != nextLayer ) {
    ClipLayerPin* lowerLayerPin = 
      (prevLayer->getRoutingLevel() < nextLayer->getRoutingLevel())?
      prevClPin : nextClPin;

    // check from lower layer
    auto vPtr = viaMap.find(lowerLayerPin->layer());
    techVia = vPtr->second;
    type = 1; 
  }
  // SEGMENT
  else {
    type = 0;
  }
}

Clip::EcoPath::EcoPath()
  : node(nullptr), edge(nullptr), child(-1) {};

Clip::EcoPath::EcoPath(EcoNode* inputNode)
  : node(inputNode), edge(nullptr), child(-1) {};

Clip::EcoPath::EcoPath(EcoEdge* inputEdge)
  : node(nullptr), edge(inputEdge), child(-1) {};

static std::string
getDebugString(eco::Clip::EcoNode& node) {
  std::stringstream ss;
  ss << "NODE " << node.x << " " << node.y 
    << " " << node.layer->getConstName(); 
  if (node.iterm) {
    ss << " " << node.iterm->getInst()->getConstName()
      << "/" << node.iterm->getMTerm()->getConstName();
  } 
  return ss.str();
}

static std::string
getDebugString(eco::Clip::EcoEdge& edge) {
  std::stringstream ss;
  ss << "EDGE type:" << edge.type;
  if( edge.techVia ) {
    ss << " " << edge.techVia->getConstName();
  }
  if( edge.via ) {
    ss << " " << edge.via->getConstName();
  }
  ss << " cmark: " << getString(edge.coreMark) 
    << " dmark: " << getString(edge.dieMark);
  return ss.str();
}

static std::string
getDebugString(eco::Clip::EcoPath& path) {
  std::stringstream ss;
  if( path.node ) {
    ss << getDebugString(*(path.node));
  }
  else if( path.edge ) {
    ss << getDebugString(*(path.edge));
  }
  if( path.child != -1 ) {
    ss << " child: " << path.child;
  }
  if( path.clPins.size() != 0 ) {
    ss << " #clPins: " << path.clPins.size();
    for(auto& clPin : path.clPins) {
      ss << " " << clPin->getVertName() << " ("<< clPin << ") ";
    }
  }
  return ss.str();
}

static std::vector<eco::Clip::EcoPath> 
getPath(std::vector<eco::Clip::EcoPath>& flatPath) {
  std::vector<eco::Clip::EcoPath> retPath;

  for(int i=0; i<flatPath.size(); i++) {
    eco::Clip::EcoPath* path = &flatPath[i];
    retPath.push_back(*path);
    debugPrint(logger, ECO, "core_eco", 5, 
        "    getPathDebug: {} {}", i, getDebugString(*path));

    // jump to child
    if( path->child != -1 ) {
      debugPrint(logger, ECO, "core_eco", 5, 
          "    getPathDebug: child: {}", path->child);
      i = path->child;
    }
  }
  return retPath;
}

// retrieve all paths from given net

std::vector<std::vector<eco::Clip::EcoPath>>& 
Clip::getPath(odb::dbNet* inputNet) {
  int netIdx = 0;
  bool isFound = false;
  for(auto& net : nets_) {
    if( inputNet == net ) {
      isFound = true;
      break;
    }
    netIdx++;
  }

  if( isFound == false ) {
    logger->error(ECO, 1354, "cannot find net: {} from eco_paths_", 
        inputNet->getConstName());
  }

  return eco_paths_[netIdx];
}

static odb::dbWireGraph::Node*
getNode(
    odb::dbWireGraph& wireGraph, 
    eco::Clip::EcoNode* ecoNode, 
    std::map<tuple<int, int, int>, odb::dbWireGraph::Node*>& nodePushMap) {
  auto npPtr = nodePushMap.find(ecoNode->getKey());
  if( npPtr == nodePushMap.end() ){
    odb::dbWireGraph::Node* retNode 
      = wireGraph.createNode( ecoNode->x, ecoNode->y, ecoNode->layer );
    nodePushMap[ ecoNode->getKey() ] = retNode;
    return retNode;
  }
  else {
    return npPtr->second;
  }
}

static tuple<int, int, int, int, int, int> 
getKey(
    eco::Clip::EcoNode* from, 
    eco::Clip::EcoNode* to) {
  return std::tuple_cat( from->getKey(), std::move(to->getKey()));
}

void
Clip::modifyPathITEX(odb::dbITerm* beginITerm,
    ClipLayerPin* endClPin,
    std::vector<eco::Clip::EcoPath>& path,
    bool& isModified, 
    std::vector<RoutedPoint>& routedPath, 
    std::vector<eco::Clip::EcoNode>& newNodes,
    std::vector<eco::Clip::EcoEdge>& newEdges,
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap) {

  debugPrint(logger, ECO, "core_eco", 5, 
      "     case2: IT-EX - {}/{} {}", 
      beginITerm->getInst()->getConstName(),
      beginITerm->getMTerm()->getConstName(),
      endClPin->getVertName());

  // need to find the outgoing edges and
  // corresponding clip layer pin
  int endIdx = -1;
  for(int i=0; i<path.size(); i++) {
    eco::Clip::EcoPath* step = &path[i]; 
    if( step->edge && isGoOutside(step->edge->coreMark) ) {
      // check if endClPin is found
      for(auto& clPin : step->clPins) {
        if( endClPin->isSameClPin(clPin) ) {
          endIdx = i;
          break;
        }
      }
    }
    // means found go-out pin
    // no need to search further
    if( endIdx != -1 ) {
      break;
    }
  }

  // means found go-out pin and 
  // corresponding external pin path
  if( endIdx != -1 ) {
    isModified = true;

    debugPrint(logger, ECO, "core_eco", 5, 
        "     case2: found end idx {}", 
        endIdx);

    path = getModifiedPathITEX(
        routedPath, newNodes, newEdges, layerViaMap, path, endIdx);

    // debug all new path
    for(auto& step : path ) {
      debugPrint(logger, ECO, "core_eco", 5, 
          "      NEW PATH2: {}", getDebugString(step)); 
    }
  }
}

void 
Clip::modifyPathEXIT(odb::dbITerm* endITerm, 
    ClipLayerPin* beginClPin, 
    std::vector<eco::Clip::EcoPath>& path, 
    bool& isModified,
    std::vector<RoutedPoint>& routedPath, 
    std::vector<eco::Clip::EcoNode>& newNodes,
    std::vector<eco::Clip::EcoEdge>& newEdges,
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap) {

  debugPrint(logger, ECO, "core_eco", 5, 
      "     case3: EX-IT - {} {}/{}", 
      beginClPin->getVertName(),
      endITerm->getInst()->getConstName(),
      endITerm->getMTerm()->getConstName());

  int beginIdx = -1;
  for(int i=path.size()-1; i>= 0; i--) {
    eco::Clip::EcoPath* step = &path[i]; 
    if( step->edge && isGoInside(step->edge->coreMark) ) {
      // check if beginClPin is found
      for(auto& clPin : step->clPins) {
        if( beginClPin->isSameClPin(clPin) ) {
          beginIdx = i;
          break;
        }
      }
    }
    // means found go-in pin
    // no need to search further
    if( beginIdx != -1 ) {
      break;
    }
  }

  // means found go-in pin and 
  // corresponding external pin path
  if( beginIdx != -1 ) {
    isModified = true;

    debugPrint(logger, ECO, "core_eco", 5, 
        "     case3: found begin idx {}", 
        beginIdx);

    path = getModifiedPathEXIT(
        routedPath, newNodes, newEdges, layerViaMap, path, beginIdx);

    // debug all new path
    for(auto& step : path) {
      debugPrint(logger, ECO, "core_eco", 5, 
          "      NEW PATH3: {}", getDebugString(step)); 
    }
  }
}


bool
Clip::readBackSMT2(std::string fileName, 
    EcoOdbNet& odbNetInfo,
    std::set<odb::dbNet*>& rectRevertedOrderedNets) {
  odb::dbBlock* block = db_->getChip()->getBlock();
  std::ifstream myFile(fileName, std::ifstream::in);
  logger->info(ECO, 1333, "Read smt2 solution file: {}", fileName);

  if( !myFile ) {
    logger->warn(ECO, 2478, "Cannot open file: {}, skip", fileName);
    return false;
  }
  
  // for resetting RNet pointer because clip is saved into clips_
  updateRNetMap();

  std::string placement;
  int  placeCnt;
  myFile >> placement >> placeCnt;
  cout << "placement : " << placement << " cnt: " << placeCnt << endl;

  for(int i=0; i<placeCnt; i++) {
    std::string instName, orientStr;
    int newLx, newLy;
    myFile >> instName >> orientStr >> newLx >> newLy;

    cout << instName << " " << orientStr << " " << newLx << " " << newLy << endl;

    // update opendb
    odb::dbInst* inst = db_->getChip()->getBlock()->findInst(instName.c_str());
    cout << "orientStr: " << orientStr << endl;
    odb::dbOrientType orient = odb::dbOrientType(orientStr.c_str());
    inst->setOrient(orient);
    inst->setLocation(newLx, newLy); 
  }
  
  
  std::string netName, netStrCnt, steiner, stnStrCnt;
  while(myFile >> netName >> netStrCnt) {

    int netCnt = stoi(netStrCnt);
    RNet* rNet = getRNet(netName);

    // avoid duplicated push to the routedPoint
    // Sometimes, the result may have the same OUT-IN conns
    std::set<std::string> outInStrSet;

    // for each CIDX netCnt 
    for(int i=0; i<netCnt; i++) {
      std::string strCIdx, strRouteCnt;
      myFile >> strCIdx >> strRouteCnt;
      int routeCnt = stoi(strRouteCnt);
      std::vector<RoutedPoint> routedPoints;
        
      for(int j=0; j<routeCnt; j++) {
        std::string routePt;
        myFile >> routePt;
        routedPoints.push_back(RoutedPoint(routePt, block));
      }


      std::string key = "";
      if( routedPoints.size() >= 2 ) {
        key = routedPoints[0].string() + "_" + routedPoints[routedPoints.size()-1].string();
      }
      else {
        logger->error(ECO, 832, "routedPoints size is weird: {}", 
            routedPoints.size());
      }

      // update routed segments
      if( rNet && outInStrSet.find(key) == outInStrSet.end() ) {
        rNet->addRoutedPoints(routedPoints);
        outInStrSet.insert(key);
      }
    }

    myFile >> steiner >> stnStrCnt;
    int stnCnt = stoi(stnStrCnt); 
    for(int i=0; i<stnCnt; i++) {
      std::string vertex;
      myFile >> vertex;
      if( rNet ) {
        rNet->addRoutedSteinerPoint(RoutedPoint(vertex, block));
      }
    }
    if (rNet == nullptr) {
      logger->warn(ECO, 2758, "cannot find RNet: {}, skip", netName);
    }
  }
  myFile.close();

  cout << "parsing check" << endl;
  for(auto& rNet : rnets_) {
    int i=0;
    for(auto& segs : rNet.routedPaths()) {
      cout << rNet.getName() << " " << i++ << endl;
      for(auto& seg: segs ) {
        cout << "   " << seg.string() << endl;
      }
    }

    cout << " STEINER: " << rNet.steinerPoints().size() << endl;
    for(auto& sPt : rNet.steinerPoints()) { 
      cout << "    " << sPt.string() << endl;
    }
  }

  // retrieve default tech vias!
  // TODO get via lists from users.
  //
  // Otherwise, traverse eco_paths and
  // get the most used vias on each metal layer
  
  // dbTechVia -> usages
  std::map<odb::dbTechVia*, int> viaMap;
  for(auto& netPaths : eco_paths_) {
    for(auto& paths : netPaths) {
      for(auto& path : paths) {
        if( path.edge && path.edge->techVia ) {
          auto viaPtr = viaMap.find(path.edge->techVia);
          if( viaPtr == viaMap.end() ) {
            viaMap[path.edge->techVia] = 1;
          }
          else {
            viaMap[path.edge->techVia] += 1;
          }
        }
      }
    }
  }
  // map to vector conversion
  std::vector<std::pair<int, odb::dbTechVia*>> vias;
  for(auto& viaPair : viaMap) {
    vias.push_back(make_pair(viaPair.second, viaPair.first));
  }
  std::sort(vias.begin(), vias.end(), std::greater<> ());
  
  // prepare layerViaMap (bottom layer -> dbTechVia)
  std::map<odb::dbTechLayer*, odb::dbTechVia*> layerViaMap;
  for(auto& viaPair : vias) {
    debugPrint(logger, ECO, "core_eco", 5, "Via statistics: {} {}", 
        viaPair.first, viaPair.second->getConstName());

    auto lvPtr = layerViaMap.find(viaPair.second->getBottomLayer());
    // update layer via map only when first meet the tech via
    if( lvPtr == layerViaMap.end() ) {
      layerViaMap[viaPair.second->getBottomLayer()] = viaPair.second;
    }
  }

  // fill missing default via -- first vias from tech->getVias();
  odb::dbSet<odb::dbTechVia> dbVias = db_->getTech()->getVias();
  for(int i=from_route_layer_-1; i<=to_route_layer_+1; i++) {
    odb::dbTechLayer* layer = db_->getTech()->findRoutingLayer(i);
    if( layer ) {
      auto lvPtr = layerViaMap.find(layer);
      // update layer via map only when first meet the tech via
      if( lvPtr == layerViaMap.end() ) {
       
        odb::dbTechVia* curVia = nullptr;
        for(odb::dbTechVia* via : dbVias){
          if( via->getBottomLayer() == layer ) {
            curVia = via;
            break; 
          }
        } 

        if( curVia == nullptr ) {
          logger->error(ECO, 4317, "cannot find via on layer: {}", 
              layer->getConstName());
        }
        layerViaMap[layer] = curVia;
      }
    }
  }
  
  // print the default vias
  for(int i=from_route_layer_-1; i<=to_route_layer_+1; i++) {
    odb::dbTechLayer* layer = db_->getTech()->findRoutingLayer(i);
    auto lvPtr = layerViaMap.find(layer);
    logger->info(ECO, 875, "Re-Route Default Vias: {}-{} : {}", 
        lvPtr->second->getBottomLayer()->getConstName(), 
        lvPtr->second->getTopLayer()->getConstName(), 
        lvPtr->second->getConstName());
  }

  // worst case
  int cnt = 0;
  for(auto& netPaths : eco_paths_) {
    for(auto& paths : netPaths) {
      cnt += paths.size();
    }
  }

  // to preserve ptrs in new* vectors;
  new_nodes_.reserve(cnt*2);
  new_edges_.reserve(cnt*2);

  // back when read back is failed to read back the whole data
  std::vector<std::vector<std::vector<EcoPath>>> ecoPathsBackup = eco_paths_;
  bool isFailed = false;

  for(auto& rNet : rnets_) {
    // retrieve path
    vector<vector<EcoPath>>& paths = getPath(rNet.net());
    
    // result paths
    vector<vector<EcoPath>> resultPaths = paths;


    for(auto& routedPath : rNet.routedPaths()) {
      RoutedPoint& beginStep = routedPath[0];
      RoutedPoint& endStep = routedPath[routedPath.size()-1];

      odb::dbITerm* beginITerm = beginStep.iTerm(), 
        *endITerm = endStep.iTerm();

      ClipLayerPin* beginClPin = nullptr,
        *endClPin = nullptr;
      
      bool isWarnHappen = false;

      // retrieve beginClPin / endClPin
      if( beginStep.isExternal() ) {
        odb::dbTechLayer* metal 
          = db_->getTech()->findRoutingLayer(beginStep.m());
        eco::ClipLayer* cLayer = dbToEb(metal);
        beginClPin  
          = cLayer->clipLayerPin( beginStep.c(), beginStep.r(), 
              ClipLayerPin::ClipLayerPinType::EXTPIN);

        if( beginClPin == nullptr ) {
          if( rNet.isSingleCommNet() == false ) {
            logger->error(ECO, 8381, 
                "cannot find begin external clip layer pin: ep_m{}r{}c{}", 
                beginStep.m(), beginStep.r(), beginStep.c());
          }
          else {
            logger->warn(ECO, 8382, 
                "cannot find begin external clip layer pin: ep_m{}r{}c{} - skip", 
                beginStep.m(), beginStep.r(), beginStep.c());
            isWarnHappen = true;
            break;
          }
        }
      }
      else if( beginITerm ) {
        RoutedPoint& beginNextStep = routedPath[1];

        odb::dbTechLayer* metal 
          = db_->getTech()->findRoutingLayer(beginNextStep.m());
        eco::ClipLayer* cLayer = dbToEb(metal);
        beginClPin  
          = cLayer->clipLayerPin( beginNextStep.c(), beginNextStep.r(), 
              ClipLayerPin::ClipLayerPinType::ITERM);
        cout << "ITERM: " << beginITerm->getInst()->getConstName() << " " << beginITerm->getMTerm()->getConstName() << endl;
        cout << beginNextStep.c() << " " << beginNextStep.r() << " " << beginClPin  << endl;
      }

      if( endStep.isExternal() ) {
        odb::dbTechLayer* metal 
          = db_->getTech()->findRoutingLayer(endStep.m());
        eco::ClipLayer* cLayer = dbToEb(metal);
        endClPin 
          = cLayer->clipLayerPin( endStep.c(), endStep.r(),
              ClipLayerPin::ClipLayerPinType::EXTPIN);

        if( endClPin == nullptr ) {
          if( rNet.isSingleCommNet() == false ) {
            logger->error(ECO, 8383, 
                "cannot find end external clip layer pin: ep_m{}r{}c{}", 
                endStep.m(), endStep.r(), endStep.c());
          }
          else {
            logger->warn(ECO, 8384, 
                "cannot find end external clip layer pin: ep_m{}r{}c{} - skip", 
                endStep.m(), endStep.r(), endStep.c());
            isWarnHappen = true;
            break;
          }
        }
      }
      // fill in the ITERM endClPin
      else if( endITerm ) {
        RoutedPoint& endPrevStep = routedPath[routedPath.size()-2];
        odb::dbTechLayer* metal 
          = db_->getTech()->findRoutingLayer(endPrevStep.m());
        eco::ClipLayer* cLayer = dbToEb(metal);
        endClPin 
          = cLayer->clipLayerPin( endPrevStep.c(), endPrevStep.r(), 
              ClipLayerPin::ClipLayerPinType::ITERM);
      }

      // skip for the other procedure for single comm net - no need to run
      if( isWarnHappen ) {
        continue;
      }


      // flag variable if routedPoints are applied or not
      bool isModified = false;

      // each source->sink path
      for(auto& path : resultPaths) {

        // filter paths (begin iterm)
        if( beginITerm 
            && path[0].node->iterm != beginITerm ) {
          continue;
        }
        // filter paths (end iterm)
        if( endITerm 
            && path[path.size()-1].node->iterm != endITerm ) {
          continue;
        }

        // if both pointers are alive, we found PATH!
        if( beginITerm && endITerm ) {
          isModified = true;
          debugPrint(logger, ECO, "core_eco", 5, 
              "     case1: IT-IT - {}/{} {}/{}",
              beginITerm->getInst()->getConstName(),
              beginITerm->getMTerm()->getConstName(),
              endITerm->getInst()->getConstName(),
              endITerm->getMTerm()->getConstName()); 
          // no need to traverse the path ptrs.
          // follow all route from routePoint
          
          // replace
          path = getModifiedPathITIT(
              routedPath, new_nodes_, new_edges_, layerViaMap);

          // debug all new path
          for(auto& step : path) {
            debugPrint(logger, ECO, "core_eco", 5, 
                "      NEW PATH1: {}", getDebugString(step)); 
          }
        }
        // IT -> go out case
        else if( beginITerm && endITerm == nullptr ) {
          modifyPathITEX(beginITerm, endClPin, path, isModified,
              routedPath, new_nodes_, new_edges_, layerViaMap);
        }
        // coming from outside -> IT case
        else if( beginITerm == nullptr && endITerm ) {
          modifyPathEXIT(endITerm, beginClPin, path, isModified,
              routedPath, new_nodes_, new_edges_, layerViaMap);
        }
        // external -> external case.
        // this case is a bit non-trivial.
        else {
          debugPrint(logger, ECO, "core_eco", 5, 
              "     case4: EX-EX - {} ({:p}) {} ({:p})", 
              beginClPin->getVertName(),
              fmt::ptr(beginClPin),
              endClPin->getVertName(),
              fmt::ptr(endClPin));

          int beginIdx = -1, endIdx = -1;
          for(int i=0; i<path.size(); i++) {
            EcoPath* step = &path[i]; 

            debugPrint(logger, ECO, "core_eco", 5, 
                "      case4 pathInfo: {} {}", 
                i, getDebugString(*step));
            if( step->edge ) {
              // both in/out case (LEFTIN && RIGHTOUT)
              if( step->clPins.size() == 2 
                  && isGoInside(step->edge->coreMark) 
                  && isGoOutside(step->edge->coreMark) ) {
                // note that clPins[0] is always output 
                // clPins[1] is always input
                // if found the case
                if( beginClPin->isSameClPin(step->clPins[0])
                    && endClPin->isSameClPin(step->clPins[1]) ) {
                  beginIdx = endIdx = i;
                  break;
                }
              }
              // beginClPin
              else if( step->clPins.size() == 1 
                  && isGoInside(step->edge->coreMark) 
                  && beginClPin->isSameClPin(step->clPins[0]) ) {
                beginIdx = i;
              }
              else if( step->clPins.size() == 1 
                  && isGoOutside(step->edge->coreMark)
                  && endClPin->isSameClPin(step->clPins[0]) ) {
                endIdx = i;
              }
            }
            // found both clPin ptrs!! 
            if( beginIdx != -1 && endIdx != -1 ) {
              break;
            }
          }

          // found both clPin ptrs!! 
          if( beginIdx != -1 && endIdx != -1 ) {
            isModified = true;
            debugPrint(logger, ECO, "core_eco", 5, 
                "     case4: found begin - end indices: {} - {}", 
                beginIdx, endIdx);

            path = getModifiedPathEXEX(
                routedPath, new_nodes_, new_edges_, layerViaMap, 
                path, beginIdx, endIdx);

            // debug all new path
            for(auto& step : path) {
              debugPrint(logger, ECO, "core_eco", 5, 
                "      NEW PATH4: {}", getDebugString(step)); 
            }
          }
        } // end case 4
      } // end each path

      // must not happen this error..
      if( isModified == false || rNet.isSpecialHandling() ) {
        // some ptrs are dead
        if( !beginClPin || !endClPin ) {
          // this could happen 
          // when IT-EX / EX-IT was swapped
          // and 
          // connected pin locs are changed because placement was changed
          // (flipped, moved, etc)
          
          logger->warn(ECO, 3249, "{} ({:p}) - {} ({:p}). Found Empty pointer. Try special handling",
            beginStep.string(), 
            fmt::ptr(beginClPin),
            endStep.string(),
            fmt::ptr(endClPin));
  
          if( (beginITerm && endITerm == nullptr) || (beginITerm == nullptr && endITerm) ) {
            // TRY INVERTED PATH 
            // (EX-IT) -> (IT-EX)
            // (IT-EX) -> (EX-IT)
            for(auto& path : resultPaths) {
              // filter paths (begin iterm)
              if( beginITerm 
                  && path[path.size()-1].node->iterm != beginITerm ) {
                continue;
              }
              // filter paths (end iterm)
              if( endITerm 
                  && path[0].node->iterm != endITerm ) {
                continue;
              }

              // original: IT-EX
              if( beginITerm && endITerm == nullptr ) {
                // TRY EX-IT
                modifyPathEXIT(beginITerm, endClPin, path, isModified,
                    routedPath, new_nodes_, new_edges_, layerViaMap);
              }
              // original: EX-IT
              else if( beginITerm == nullptr && endITerm ) {
                // TRY IT-EX
                modifyPathITEX(endITerm, beginClPin, path, isModified,
                    routedPath, new_nodes_, new_edges_, layerViaMap);
              }
            }
          }

          if( !isModified ) {
            logger->warn(ECO, 3250, "Cannot find feasible solution. Skip this clip");
            isFailed = true;
            break;
          }
        }
        // all ptrs are alive
        else {
          logger->warn(ECO, 3247, "Cannot find corresponding path! {} - {}. Try EXEX flow manually - case5",
            beginClPin->getVertName(), 
            endClPin->getVertName());

          // each source->sink path
          for(auto& path : resultPaths) {
            debugPrint(logger, ECO, "core_eco", 5, 
                "     case5: SPECIAL - {} {}",
                beginClPin->getVertName(),
                endClPin->getVertName());

            int beginIdx = -1, endIdx = -1;
            for(int i=0; i<path.size(); i++) {
              EcoPath* step = &path[i]; 

              debugPrint(logger, ECO, "core_eco", 5, 
                  "      case5 pathInfo: {} {}", 
                  i, getDebugString(*step));
              if( step->edge ) {
                // both in/out case (LEFTIN && RIGHTOUT)
                if( step->clPins.size() == 2 
                    && isGoInside(step->edge->coreMark) 
                    && isGoOutside(step->edge->coreMark) ) {
                  // note that clPins[0] is always output 
                  // clPins[1] is always input
                  // if found the case
                  if( beginClPin->isSameClPin(step->clPins[0])
                      && endClPin->isSameClPin(step->clPins[1]) ) {
                    beginIdx = endIdx = i;
                    break;
                  }
                }
                // beginClPin
                else if( step->clPins.size() == 1 
                    && isGoInside(step->edge->coreMark) 
                    && beginClPin->isSameClPin(step->clPins[0]) ) {
                  beginIdx = i;
                }
                else if( step->clPins.size() == 1 
                    && isGoOutside(step->edge->coreMark)
                    && endClPin->isSameClPin(step->clPins[0]) ) {
                  endIdx = i;
                }
              }
              // found both clPin ptrs!! 
              if( beginIdx != -1 && endIdx != -1 ) {
                break;
              }
            }

            // found both clPin ptrs!! 
            if( beginIdx != -1 && endIdx != -1 ) {
              debugPrint(logger, ECO, "core_eco", 5, 
                  "     case5: found begin - end indices: {} - {}!", 
                  beginIdx, endIdx);

              if( beginIdx + 2 == endIdx ) {
                // do nothing
                isModified = true;
                // debug all new path
                for(auto& step : path) {
                  debugPrint(logger, ECO, "core_eco", 5, 
                    "      NEW PATH5: {}", getDebugString(step)); 
                }
              }
              // ITEX case
              else if( beginITerm && endITerm == nullptr ) {
                isModified = true;

                path = getModifiedPathMiddleITEX(
                  routedPath, new_nodes_, new_edges_, layerViaMap, 
                  path, beginIdx, endIdx);

                // debug all new path
                for(auto& step : path) {
                  debugPrint(logger, ECO, "core_eco", 5, 
                    "      DEBUG NEW PATH5: {}", getDebugString(step)); 
                }
              }
              else if( beginITerm == nullptr && endITerm ) {
                // calling case4- EXEX is enough to stitch this path
                path = getModifiedPathEXEX(
                    routedPath, new_nodes_, new_edges_, layerViaMap, 
                    path, beginIdx, endIdx);

                // debug all new path
                for(auto& step : path) {
                  debugPrint(logger, ECO, "core_eco", 5, 
                    "      DEBUG NEW PATH5: {}", getDebugString(step)); 
                }
              }
            }
          } // end resultPaths for loop

          if( isModified == false ) {
            logger->warn(ECO, 3248, 
                "Cannot find corresponding path! {} - {}. CoRe-ECO was failed to read the current clip data.",
                beginClPin->getVertName(), 
                endClPin->getVertName());
            isFailed = true;
            break;
          }
        } // end else - all ptrs are alive
      } // end special isModified handling if clause
    } // end each routed path

    if( isFailed ) {
      break;
    }
  
    // replace with resultPaths
    paths = resultPaths;
  } // end rNets

  // revert back
  if( isFailed ) {
    eco_paths_ = ecoPathsBackup;
    return false;
  }


  // eco_paths_ net index
  for(int i=0; i<eco_paths_.size(); i++) {
    // each net has each wire graph
    odb::dbWireGraph wireGraph;
    odb::dbWire* curWire = 
      odb::dbWire::create( db_->getChip()->getBlock(), false );


    //cout << "BEFORE dump " << nets_[i]->getConstName() << endl;
    //odb::dumpDecoder4Net(nets_[i]); 
    //cout << "BEFORE dump Decoder finished" << endl;
    //nets_[i]->printWire(); 
    //cout << "BEFORE dump print wire finished" << endl;

    // erase previous wires
    odb::dbWire* prevWire = nets_[i]->getWire();
    //cout << "prevWire detach" << endl;
    odb::dbWire::destroy(prevWire);
    //cout << "prevWire destroy" << endl;

    // node push map helper
    std::map<tuple<int, int, int>, odb::dbWireGraph::Node*> nodePushMap;
    std::set<tuple<int, int, int, int, int, int>> edgePushSet;

    int pathCnt = 0;
    for(auto& paths : eco_paths_[i]) {
      // get merged paths
      // no need HERE
      // std::vector<EcoPath> newPaths = getMergedPaths(paths, stnPoints);

      for(int j=0; j<paths.size(); j++) {  
        debugPrint(logger, ECO, "core_eco", 5, " final path {}({}), p: {} s: {} - {}", 
           nets_[i]->getConstName(), i, pathCnt, j, getDebugString(paths[j]));

        EcoNode* curNode = paths[j].node;
        EcoEdge* curEdge = paths[j].edge;
        // for the first node
        if( j == 0 && curNode ) {
          odb::dbWireGraph::Node* odbNode = getNode(wireGraph, curNode, nodePushMap);
          if(curNode->iterm) {
            odbNode->setObject(curNode->iterm);
          } 
        }
        else if( curEdge ) {
          EcoNode* prevNode = paths[j-1].node;
          EcoNode* nextNode = paths[j+1].node;

          odb::dbWireGraph::Node* prevWireNode 
            = getNode(wireGraph, prevNode, nodePushMap);

          odb::dbWireGraph::Node* nextWireNode 
            = getNode(wireGraph, nextNode, nodePushMap);

          if( prevNode->iterm) {
            prevWireNode->setObject(prevNode->iterm);
          }

          if( nextNode->iterm) {
            nextWireNode->setObject(nextNode->iterm);
          }

          auto epPtr = edgePushSet.find(getKey(prevNode, nextNode));
          // update epPtr only when edge never pushed before!!!
          if( epPtr == edgePushSet.end() ) {
            edgePushSet.insert(getKey(prevNode, nextNode));
            odb::dbWireGraph::Segment* seg = nullptr;
            odb::dbWireGraph::TechVia* tVia = nullptr;
            cout << "trying: [EcoNode*] " << prevNode << " " <<  nextNode << endl;
            cout << "trying: [WireGraph::Node*] " << prevWireNode << " " << nextWireNode << endl;

            tuple<int, int, int, int, int, int> getTuple = getKey(prevNode, nextNode);
            cout << "tuple : " << std::get<0>(getTuple)
              << " " << std::get<1>(getTuple)
              << " " << std::get<2>(getTuple)
              << " " << std::get<3>(getTuple)
              << " " << std::get<4>(getTuple)
              << " " << std::get<5>(getTuple) << endl;

            cout << " PREV: "  << prevWireNode->layer()->getConstName() 
              << " " << prevWireNode->object() << endl;
            cout << " NEXT: "  << nextWireNode->layer()->getConstName()
              << " " << nextWireNode->object() << endl;

            switch( curEdge->type ) {
              case odb::dbWireGraph::Edge::SEGMENT:
                seg = wireGraph.createSegment(
                    prevWireNode, nextWireNode, 
                    odb::dbWireType::Value::ROUTED);
                cout << "SEG: " << seg << endl;
                break;
              case odb::dbWireGraph::Edge::TECH_VIA:
                tVia = wireGraph.createTechVia(
                    prevWireNode, nextWireNode,
                    curEdge->techVia, 
                    odb::dbWireType::Value::ROUTED);
                cout << "TVIA: " << tVia << endl;
                break;
              case odb::dbWireGraph::Edge::VIA:
                wireGraph.createVia(
                    prevWireNode, nextWireNode, 
                    curEdge->via,
                    odb::dbWireType::Value::ROUTED);
                break;
              // 10/13/2023
              // the following had errors when INVS parse the results...
              // Looks like SHORT exists only inside OpenDB, not DEF
              //
              case odb::dbWireGraph::Edge::SHORT:
                // wireGraph.createShort(
                //     prevWireNode, nextWireNode,
                //     odb::dbWireType::Value::ROUTED);
                cout << "SHORT, but skipped" << endl;
                break;
              case odb::dbWireGraph::Edge::VWIRE:
                wireGraph.createVWire(
                    prevWireNode, nextWireNode,
                    odb::dbWireType::Value::ROUTED);
                break;
              default:
                break;
            }
            cout << "NEW: " << prevWireNode << " " << nextWireNode << endl;
          }
        } 
      }

      pathCnt++;
    }

    wireGraph.encode(curWire);
    curWire->attach(nets_[i]);

    // cout << "BEFORE RECT dump " << nets_[i]->getConstName() << endl;
    // odb::dumpDecoder4Net(nets_[i]); 
    // cout << "BEFORE RECT dump Decoder finished" << endl;

    // attach RECT into curWire object
    odb::dbWire* newWire 
      = getNewWireForRects(curWire, odbNetInfo);  
    rectRevertedOrderedNets.insert( nets_[i] );

    odb::dbWire::destroy(curWire);
    newWire->attach(nets_[i]);
    
    // dump test
    // cout << "AFTER RECT dump " << nets_[i]->getConstName() << endl;
    // odb::dumpDecoder4Net(nets_[i]); 
    // cout << "AFTER RECT dump Decoder finished" << endl;
    // nets_[i]->printWire(); 
    // cout << "AFTER RECT print wire finished" << endl;

  }

  return true;
}

static void
updateErasePoints(std::vector<tuple<int, int, int>>& pointSeq, 
    std::set<tuple<int, int, int>>& erasePoints) {
  if( pointSeq.size() <= 2 ) {
    pointSeq.clear();
    pointSeq.shrink_to_fit();
    return;
  }

  for(size_t i=1; i<pointSeq.size()-1; i++) {
    tuple<int, int,int > prev = pointSeq[i-1];
    tuple<int, int,int > curr = pointSeq[i];
    tuple<int, int,int > next = pointSeq[i+1];

    if( ((std::get<1>(prev) == std::get<1>(curr) &&
        std::get<1>(curr) == std::get<1>(next))) ||
        ((std::get<2>(prev) == std::get<2>(curr) &&
        std::get<2>(curr) == std::get<2>(next)))) {
      erasePoints.insert(curr);
    }
  }

  pointSeq.clear();
  pointSeq.shrink_to_fit();
}

static odb::dbWire* 
getNewWireForRects(odb::dbWire* prevWire, EcoOdbNet& odbNetInfo) {

  if( prevWire == nullptr) {
    return nullptr;
  }

  using odb::dbWireDecoder;
  using odb::dbWireEncoder;
  using odb::dbTechLayer; 
  using odb::dbWireType;
  using odb::dbTechLayerRule;
  using odb::dbNet;
  using odb::dbWire;
  using odb::dbTechVia;

  dbWireDecoder decoder;
  dbWireDecoder::OpCode opcode;
  int x, y, ext;

  decoder.begin(prevWire);

  dbTechLayer* layer = nullptr;
  dbWireType wtype;
  dbTechLayerRule* lyr_rule = nullptr;

  // x,y,ext -> jctId retrieve
  std::vector<tuple<int,int,int,int>> extPointStor;

  // x,y -> jctId retrieve
  std::vector<tuple<int,int,int>> pointStor;

  odb::dbBlock* block = prevWire->getNet()->getBlock();

  // create new wire
  odb::dbWire* curWire = 
    odb::dbWire::create( block, false);

  dbWireEncoder encoder;
  encoder.begin(curWire);

  odb::dbNet* curNet = prevWire->getNet();

  // decoder Junction ID -> encoder Junction ID
  // int -> int map

  std::map<int, int> decoder2Encoder;

  while (1) {
    opcode = decoder.next();
    if (opcode == dbWireDecoder::END_DECODE) {
      break;
    }

    switch (opcode) {
      case dbWireDecoder::PATH: {
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        if (decoder.peek() == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
          encoder.newPath(layer, wtype, lyr_rule);
        } 
        else {
          encoder.newPath(layer, wtype);
        }
        break;
      }

      case dbWireDecoder::JUNCTION: {
        uint jct = decoder.getJunctionValue();
        layer = decoder.getLayer();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
          opcode = decoder.peek();
        }
        if (opcode == dbWireDecoder::POINT_EXT) {
          opcode = decoder.next();
          decoder.getPoint(x, y, ext);

          auto dePtr = decoder2Encoder.find(jct);
          if( dePtr == decoder2Encoder.end() ) {
            logger->error(utl::ECO, 
                9732, 
                "Cannot find junction id in e2d map: {}", jct);
          }

          int encoderJunctionId = dePtr->second;
          if( lyr_rule ) {
            encoder.newPathExt( encoderJunctionId, ext, wtype, lyr_rule);
          }
          else {
            encoder.newPathExt( encoderJunctionId, ext, wtype);
          }
        } 
        else if (opcode == dbWireDecoder::POINT) {
          opcode = decoder.next();
          decoder.getPoint(x, y);

          auto dePtr = decoder2Encoder.find(jct);
          if( dePtr == decoder2Encoder.end() ) {
            logger->error(utl::ECO, 
                9733, 
                "Cannot find junction id in e2d map: {}", jct);
          }

          int encoderJunctionId = dePtr->second;
          if( lyr_rule ) {
            encoder.newPath( encoderJunctionId, wtype, lyr_rule);
          }
          else {
            encoder.newPath( encoderJunctionId, wtype );
          }
        } 
        break;
      }

      case dbWireDecoder::SHORT: {
        uint jct = decoder.getJunctionValue();
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
        }

        auto dePtr = decoder2Encoder.find(jct);
        if( dePtr == decoder2Encoder.end() ) {
          logger->error(utl::ECO, 
              9734, 
              "Cannot find junction id in e2d map: {}", jct);
        }

        int encoderJunctionId = dePtr->second;
        if (lyr_rule) {
          encoder.newPathShort(encoderJunctionId, layer, wtype, lyr_rule);
        }
        else {
          encoder.newPathShort(encoderJunctionId, layer, wtype);
        }
        break;
      }

      case dbWireDecoder::VWIRE: {
        uint jct = decoder.getJunctionValue();
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
        }

        auto dePtr = decoder2Encoder.find(jct);
        if( dePtr == decoder2Encoder.end() ) {
          logger->error(utl::ECO, 
              9735, 
              "Cannot find junction id in e2d map: {}", jct);
        }

        int encoderJunctionId = dePtr->second;
        if( lyr_rule ) {
          encoder.newPathVirtualWire( encoderJunctionId, layer, wtype, lyr_rule );
        }
        else {
          encoder.newPathVirtualWire( encoderJunctionId, layer, wtype );
        }

        break;
      }

      case dbWireDecoder::POINT: {
        decoder.getPoint(x, y);
        int encoderJunctionId = encoder.addPoint(x,y);
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;

        if( rInfo.prevOpcode == opcode 
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {

          debugPrint(logger, ECO, "core_eco", 5, "  Found Rect (PONT) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
          encoder.addRect(rect.xMin(), rect.yMin(), rect.xMax(), rect.yMax());
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal point {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }
        break;
      }

      case dbWireDecoder::POINT_EXT: {
        decoder.getPoint(x, y, ext);

        int encoderJunctionId = encoder.addPoint(x,y, ext);
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        break;
      }

      case dbWireDecoder::TECH_VIA: {
        layer = decoder.getLayer();
        int encoderJunctionId = encoder.addTechVia(decoder.getTechVia());
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;

        if( (rInfo.prevOpcode == dbWireDecoder::TECH_VIA ||
              rInfo.prevOpcode == dbWireDecoder::POINT )
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {

          debugPrint(logger, ECO, "core_eco", 5, " Found Rect (TVIA) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          encoder.addRect(rect.xMin(), rect.yMin(), rect.xMax(), rect.yMax());
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal teVia {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }
        break;
      }

      case dbWireDecoder::VIA: {
        layer = decoder.getLayer();
        int encoderJunctionId = encoder.addVia(decoder.getVia());
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;

        if( (rInfo.prevOpcode == dbWireDecoder::VIA ||
              rInfo.prevOpcode == dbWireDecoder::POINT )
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {
          debugPrint(logger, ECO, "core_eco", 5, " Found Rect  (VIA) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          encoder.addRect(rect.xMin(), rect.yMin(), rect.xMax(), rect.yMax());
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal   Via {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }

        break;
      }

      // I don't expect this will happen......
      case dbWireDecoder::RECT: {
                                  
        int deltaX1;
        int deltaY1;
        int deltaX2;
        int deltaY2;
        decoder.getRect(deltaX1, deltaY1, deltaX2, deltaY2);
        break;
      }

      case dbWireDecoder::ITERM: {
        encoder.addITerm(decoder.getITerm());
        break;
      }

      case dbWireDecoder::BTERM: {
        encoder.addBTerm(decoder.getBTerm());
        break;
      }

      case dbWireDecoder::RULE: {
        // nothing dbWireEncoder can do...
        break;
      }

      case dbWireDecoder::END_DECODE:
        break;

      default: {
        break;
      }
    }  // switch opcode
  }    // while


  // finish encoding
  encoder.end();

  return curWire;
}

static odb::dbWire* 
getNewWireForRectsAndRemoveOverlap(
    odb::dbWire* prevWire, EcoOdbNet& odbNetInfo,
    int minLayer, int maxLayer) {

  if( prevWire == nullptr) {
    return nullptr;
  }

  using odb::dbWireDecoder;
  using odb::dbWireEncoder;
  using odb::dbTechLayer; 
  using odb::dbWireType;
  using odb::dbTechLayerRule;
  using odb::dbNet;
  using odb::dbWire;
  using odb::dbTechVia;

  dbWireDecoder decoder;
  dbWireDecoder::OpCode opcode;
  int x, y, ext;

  dbTechLayer* layer = nullptr;
  dbWireType wtype;
  dbTechLayerRule* lyr_rule = nullptr;

  odb::dbNet* curNet = prevWire->getNet();

  decoder.begin(prevWire);

  // save pointer sequence
  // first iter is needed to avoid wire rect overlaps
  std::vector<tuple<int, int,int>> pointSeq;
  std::set<tuple<int, int,int>> erasePoints, notErasePoints, finalErasePoints;
  bool shouldKeepNextPoint = false;


  while (1) {
    opcode = decoder.next();
    if (opcode == dbWireDecoder::END_DECODE) {
      break;
    }

    switch (opcode) {
      case dbWireDecoder::PATH: {
        updateErasePoints(pointSeq, erasePoints);
        shouldKeepNextPoint = false;
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        if (decoder.peek() == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
        } 
        break;
      }

      case dbWireDecoder::JUNCTION: {
        updateErasePoints(pointSeq, erasePoints);
        shouldKeepNextPoint = false;
        layer = decoder.getLayer();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
          opcode = decoder.peek();
        }
        if (opcode == dbWireDecoder::POINT_EXT) {
          opcode = decoder.next();
          decoder.getPoint(x, y, ext);
        } 
        else if (opcode == dbWireDecoder::POINT) {
          opcode = decoder.next();
          decoder.getPoint(x, y);
          int layerNum = layer->getRoutingLevel();
          tuple<int,int,int> coordi = tuple<int,int,int>(layerNum,x,y);

          if( layerNum >= minLayer && layerNum <= maxLayer ) {
            pointSeq.push_back(coordi);
          }
          notErasePoints.insert(coordi);
        } 
        break;
      }

      case dbWireDecoder::SHORT: {
        updateErasePoints(pointSeq, erasePoints);
        shouldKeepNextPoint = true;
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
        }

        debugPrint(logger, ECO, "core_eco", 5, "Point removal (SHRT): {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel());
        break;
      }

      case dbWireDecoder::VWIRE: {
        updateErasePoints(pointSeq, erasePoints);
        shouldKeepNextPoint = true;
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
        }
        break;
      }

      case dbWireDecoder::POINT: {
        decoder.getPoint(x, y);

        int layerNum = layer->getRoutingLevel();
        if( layerNum >= minLayer && layerNum <= maxLayer ) {
          pointSeq.push_back(tuple<int,int, int>(layerNum, x,y));
        }

        if( shouldKeepNextPoint ) {
          notErasePoints.insert(tuple<int,int,int>(layerNum,x,y));
          shouldKeepNextPoint = false;
        }

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;

        if( rInfo.prevOpcode == opcode 
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {

          debugPrint(logger, ECO, "core_eco", 5, "  Found Rect (PONT) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          notErasePoints.insert(tuple<int,int,int>(layerNum,x,y));
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal point {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }
        break;
      }

      case dbWireDecoder::POINT_EXT: {
        shouldKeepNextPoint = false;
        decoder.getPoint(x, y, ext);
        break;
      }

      case dbWireDecoder::TECH_VIA: {
        notErasePoints.insert(tuple<int, int, int>(layer->getRoutingLevel(), x, y));
        shouldKeepNextPoint = false;
        layer = decoder.getLayer();
        
        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;

        if( (rInfo.prevOpcode == dbWireDecoder::TECH_VIA ||
              rInfo.prevOpcode == dbWireDecoder::POINT )
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {

          debugPrint(logger, ECO, "core_eco", 5, "  Found Rect (TVIA) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          notErasePoints.insert(tuple<int,int,int>(layer->getRoutingLevel(),x,y));
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal teVia {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }

        break;
      }

      case dbWireDecoder::VIA: {
        notErasePoints.insert(tuple<int, int, int>(layer->getRoutingLevel(), x, y));
        shouldKeepNextPoint = false;
        layer = decoder.getLayer();

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;

        if( (rInfo.prevOpcode == dbWireDecoder::VIA ||
              rInfo.prevOpcode == dbWireDecoder::POINT )
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {

          debugPrint(logger, ECO, "core_eco", 5, "  Found Rect  (VIA) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          notErasePoints.insert(tuple<int,int,int>(layer->getRoutingLevel(),x,y));
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal   Via {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }

        break;
      }

      // I don't expect this will happen......
      case dbWireDecoder::RECT: {
        int deltaX1;
        int deltaY1;
        int deltaX2;
        int deltaY2;
        decoder.getRect(deltaX1, deltaY1, deltaX2, deltaY2);
        break;
      }

      case dbWireDecoder::ITERM: {
        break;
      }

      case dbWireDecoder::BTERM: {
        break;
      }

      case dbWireDecoder::RULE: {
        // nothing dbWireEncoder can do...
        break;
      }

      case dbWireDecoder::END_DECODE:
        break;

      default: {
        break;
      }
    }  // switch opcode
  }    // while

  for(auto& point : erasePoints) {
    debugPrint(logger, ECO, "core_eco", 5, 
        " point removal - erasePoints: {} {} {}", 
      std::get<0>(point),
      std::get<1>(point),
      std::get<2>(point));

    if( notErasePoints.find(point) == notErasePoints.end() ) {
      finalErasePoints.insert(point);
    }
  }

  for(auto& point : notErasePoints) {
    debugPrint(logger, ECO, "core_eco", 5, 
        " point removal - notErasePoints: {} {} {}", 
      std::get<0>(point),
      std::get<1>(point),
      std::get<2>(point));
  }

  for(auto& point : finalErasePoints) {
    debugPrint(logger, ECO, "core_eco", 5, 
        " point removal - finalErasePoints: {} {} {}", 
      std::get<0>(point),
      std::get<1>(point),
      std::get<2>(point));
  }


  // x,y,ext -> jctId retrieve
  std::vector<tuple<int,int,int,int>> extPointStor;

  // x,y -> jctId retrieve
  std::vector<tuple<int,int,int>> pointStor;

  odb::dbBlock* block = prevWire->getNet()->getBlock();

  // create new wire
  odb::dbWire* curWire = 
    odb::dbWire::create( block, false);

  dbWireEncoder encoder;
  encoder.begin(curWire);


  // decoder Junction ID -> encoder Junction ID
  // int -> int map

  std::map<int, int> decoder2Encoder;

  // re-traverse the wire to attach rects
  decoder.begin(prevWire);
  layer = nullptr;
  lyr_rule = nullptr;

  while (1) {
    opcode = decoder.next();
    if (opcode == dbWireDecoder::END_DECODE) {
      break;
    }

    switch (opcode) {
      case dbWireDecoder::PATH: {
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        if (decoder.peek() == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
          encoder.newPath(layer, wtype, lyr_rule);
        } 
        else {
          encoder.newPath(layer, wtype);
        }
        break;
      }

      case dbWireDecoder::JUNCTION: {
        uint jct = decoder.getJunctionValue();
        layer = decoder.getLayer();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
          opcode = decoder.peek();
        }
        if (opcode == dbWireDecoder::POINT_EXT) {
          opcode = decoder.next();
          decoder.getPoint(x, y, ext);

          auto dePtr = decoder2Encoder.find(jct);
          if( dePtr == decoder2Encoder.end() ) {
            logger->error(utl::ECO, 
                9832, 
                "Cannot find junction id in e2d map: {}", jct);
          }

          int encoderJunctionId = dePtr->second;
          if( lyr_rule ) {
            encoder.newPathExt( encoderJunctionId, ext, wtype, lyr_rule);
          }
          else {
            encoder.newPathExt( encoderJunctionId, ext, wtype);
          }
        } 
        else if (opcode == dbWireDecoder::POINT) {
          opcode = decoder.next();
          decoder.getPoint(x, y);

          auto dePtr = decoder2Encoder.find(jct);
          if( dePtr == decoder2Encoder.end() ) {
            logger->error(utl::ECO, 
                9833, 
                "Cannot find junction id in e2d map: {}", jct);
          }

          int encoderJunctionId = dePtr->second;
          if( lyr_rule ) {
            encoder.newPath( encoderJunctionId, wtype, lyr_rule);
          }
          else {
            encoder.newPath( encoderJunctionId, wtype );
          }

          debugPrint(logger, ECO, "core_eco", 5, "  Missing junc pont? (PONT) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        } 
        break;
      }

      case dbWireDecoder::SHORT: {
        uint jct = decoder.getJunctionValue();
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
        }

        auto dePtr = decoder2Encoder.find(jct);
        if( dePtr == decoder2Encoder.end() ) {
          logger->error(utl::ECO, 
              9834, 
              "Cannot find junction id in e2d map: {}", jct);
        }

        int encoderJunctionId = dePtr->second;
        if (lyr_rule) {
          encoder.newPathShort(encoderJunctionId, layer, wtype, lyr_rule);
        }
        else {
          encoder.newPathShort(encoderJunctionId, layer, wtype);
        }
        break;
      }

      case dbWireDecoder::VWIRE: {
        uint jct = decoder.getJunctionValue();
        layer = decoder.getLayer();
        wtype = decoder.getWireType();
        lyr_rule = NULL;
        opcode = decoder.peek();
        if (opcode == dbWireDecoder::RULE) {
          opcode = decoder.next();
          lyr_rule = decoder.getRule();
        }

        auto dePtr = decoder2Encoder.find(jct);
        if( dePtr == decoder2Encoder.end() ) {
          logger->error(utl::ECO, 
              9835, 
              "Cannot find junction id in e2d map: {}", jct);
        }

        int encoderJunctionId = dePtr->second;
        if( lyr_rule ) {
          encoder.newPathVirtualWire( encoderJunctionId, layer, wtype, lyr_rule );
        }
        else {
          encoder.newPathVirtualWire( encoderJunctionId, layer, wtype );
        }

        break;
      }

      case dbWireDecoder::POINT: {
        decoder.getPoint(x, y);

        std::tuple<int, int, int> searchCoordi(layer->getRoutingLevel(), x, y);
        // if point must be erased
        if( finalErasePoints.find( searchCoordi ) != finalErasePoints.end() ) {
          // AVOID adding the current point 
          debugPrint(logger, ECO, "core_eco", 5, " erasePoint: {} {} {} {} (dec junction id: {})",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y,
            decoder.getJunctionId());
          break;
        }

        int encoderJunctionId = encoder.addPoint(x,y);
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;

        if( rInfo.prevOpcode == opcode 
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {
          debugPrint(logger, ECO, "core_eco", 5, "  Found Rect (PONT) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          encoder.addRect(rect.xMin(), rect.yMin(), rect.xMax(), rect.yMax());
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal point {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }
        break;
      }

      case dbWireDecoder::POINT_EXT: {
        decoder.getPoint(x, y, ext);

        int encoderJunctionId = encoder.addPoint(x,y, ext);
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        break;
      }

      case dbWireDecoder::TECH_VIA: {
        layer = decoder.getLayer();
        int encoderJunctionId = encoder.addTechVia(decoder.getTechVia());
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;


        if( (rInfo.prevOpcode == dbWireDecoder::TECH_VIA ||
              rInfo.prevOpcode == dbWireDecoder::POINT )
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {

          debugPrint(logger, ECO, "core_eco", 5, "  Found Rect (TVIA) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          encoder.addRect(rect.xMin(), rect.yMin(), rect.xMax(), rect.yMax());
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal teVia {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }

        break;
      }

      case dbWireDecoder::VIA: {
        layer = decoder.getLayer();
        int encoderJunctionId = encoder.addVia(decoder.getVia());
        decoder2Encoder[decoder.getJunctionId()] = encoderJunctionId;

        // add rect if applicable
        RectInfo rInfo = odbNetInfo.getRect(curNet, layer->getRoutingLevel(), x, y);
        odb::Rect rect = rInfo.rect;


        if( (rInfo.prevOpcode == dbWireDecoder::VIA ||
              rInfo.prevOpcode == dbWireDecoder::POINT )
            && !(rect.xMin() == -1 && rect.yMin() == -1 && rect.xMax() == -1 && rect.yMax() == -1) ) {

          debugPrint(logger, ECO, "core_eco", 5, "  Found Rect  (VIA) {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);

          encoder.addRect(rect.xMin(), rect.yMin(), rect.xMax(), rect.yMax());
        }
        else {
          debugPrint(logger, ECO, "core_eco", 5, "  Normal   Via {}: - {} {} {}",
            curNet->getConstName(),
            layer->getRoutingLevel(),
            x,
            y);
        }

        break;
      }

      // I don't expect this will happen......
      case dbWireDecoder::RECT: {
                                  
        int deltaX1;
        int deltaY1;
        int deltaX2;
        int deltaY2;
        decoder.getRect(deltaX1, deltaY1, deltaX2, deltaY2);
        break;
      }

      case dbWireDecoder::ITERM: {
        encoder.addITerm(decoder.getITerm());
        break;
      }

      case dbWireDecoder::BTERM: {
        encoder.addBTerm(decoder.getBTerm());
        break;
      }

      case dbWireDecoder::RULE: {
        // nothing dbWireEncoder can do...
        break;
      }

      case dbWireDecoder::END_DECODE:
        break;

      default: {
        break;
      }
    }  // switch opcode
  }    // while


  // finish encoding
  encoder.end();

  return curWire;
}


// -1: empty
// 0: place-able
// 1: cannot place (Fixed)
//
void
Clip::outStreamDpGrid(std::ofstream& out) { 
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbRow> rows = block->getRows();

  // retrieve row/site
  odb::dbRow* row = *(rows.begin());
  odb::dbSite* site = row->getSite();

  out << "; DP site box size" << endl;
  out << "; " << site_lower_x_ << " " << site_lower_y_ << " " 
    << site_upper_x_ << " " <<  site_upper_y_ << endl;
  out << endl;

  out << "; DP grid type (E: empty, F: fixed)" << endl;
  for(auto& y: dp_grid_) {
    out << "; ";
    for(auto& dpSite: y) {
      out << dpSite.printType() << " ";
    }
    out << endl;
  }
  out << endl;

  out << "; DP grid row orient" << endl;
  for(auto& y: dp_grid_) {
    out << "; ";
    for(auto& dpSite: y) {
      out << dpSite.printOrient() << " ";
    }
    out << endl;
  }
  
  out << endl;

  out << "; DP grid place pairs" << endl;
  for(auto& row : dp_grid_place_pairs_) {
    int rowIdx = &row - &dp_grid_place_pairs_[0];
    for(auto& pair: row) {
      out << "; row:" << rowIdx << " : " << pair.first << " " << pair.second << endl;
    }
  }
  out << endl;
  out << "; fixed insts detail" << endl;
  out << "; (name dbuLx dbuLy dbuUx dbuUy gridLx gridLy gridUx gridUy orient)" << endl;
  for(auto& inst: fixed_insts_) {
    // get grid (lx, ly, ux uy)
    odb::dbBox* iBox = inst->getBBox();
    
    int gLx = (iBox->xMin() - site_lower_x_) / static_cast<int>(site->getWidth());
    int gLy = (iBox->yMin() - site_lower_y_) / static_cast<int>(site->getHeight());
    int gUx = (iBox->xMax() - site_lower_x_) / static_cast<int>(site->getWidth());
    int gUy = (iBox->yMax() - site_lower_y_) / static_cast<int>(site->getHeight());
    
    out << "; " << inst->getConstName() << ": " 
      << iBox->xMin() << " " << iBox->yMin() << " " 
      << iBox->xMax() << " " << iBox->yMax() << " ";

    out << gLx << " " << gLy << " " << gUx << " " << gUy << " ";
    out << getOrientStr(inst->getOrient()) << endl;
  }
  out << endl;

  out << "; movable insts detail" << endl;
  out << "; (name dbuDx dbuDy gridDx gridDy)" << endl;
  for(auto& inst : movable_insts_) {
    // get grid (lx, ly, ux uy)
    odb::dbBox* iBox = inst->getBBox();

    int gDx = (iBox->getDX()) / static_cast<int>(site->getWidth());
    int gDy = (iBox->getDY()) / static_cast<int>(site->getHeight());

    out << "; " << inst->getConstName() << ": " 
      << iBox->getDX() << " " << iBox->getDY() << " " ;
    out << gDx << " " << gDy << endl; 
  }
  out << endl;

  out << "; cutted netlist (RNet)" << endl;
  for(auto& rNet : rnets_) {
    out << "; net: " << rNet.getName() << endl;
    for(auto& oPin : rNet.oPins()) {
      out << ";  O "; 
      if( oPin->type() == ClipLayerPin::ClipLayerPinType::EXTPIN ) {
        out << " EX " << oPin->x() << " " << oPin->y();
      }
      else if( oPin->type() == ClipLayerPin::ClipLayerPinType::ITERM ) {
        out << " IT " << oPin->x() << " " << oPin->y();
        out << " " << oPin->iTerm()->getInst()->getConstName() << "/" << oPin->iTerm()->getMTerm()->getConstName();
      }
      else if( oPin->type() == ClipLayerPin::ClipLayerPinType::BTERM ) {
        out << " BT";
      }
      else if( oPin->type() == ClipLayerPin::ClipLayerPinType::OBS ) {
        out << " OB";
      }
      else if( oPin->type() == ClipLayerPin::ClipLayerPinType::EMPTY) {
        out << " EM";
      }

      out << " " << oPin->layer()->getConstName() << " ";

      if( oPin->dir() == ClipLayerPin::Direction::UP) {
        out << " UP";
      }
      else if( oPin->dir() == ClipLayerPin::Direction::DOWN) {
        out << " DO";
      }
      else if( oPin->dir() == ClipLayerPin::Direction::SAME) {
        out << " SA";
      }
      
      if( oPin->isFixed() ) {
        out << " FIXED";
      }
      out << endl;
    }
    for(auto& iPin : rNet.iPins()) {
      out << ";  I "; 
      if( iPin->type() == ClipLayerPin::ClipLayerPinType::EXTPIN ) {
        out << " EX " << iPin->x() << " " << iPin->y();
      }
      else if( iPin->type() == ClipLayerPin::ClipLayerPinType::ITERM ) {
        out << " IT " << iPin->x() << " " << iPin->y();
        out << " " << iPin->iTerm()->getInst()->getConstName() << "/" << iPin->iTerm()->getMTerm()->getConstName();
      }
      else if( iPin->type() == ClipLayerPin::ClipLayerPinType::BTERM ) {
        out << " BT";
      }
      else if( iPin->type() == ClipLayerPin::ClipLayerPinType::OBS ) {
        out << " OB";
      }
      else if( iPin->type() == ClipLayerPin::ClipLayerPinType::EMPTY ) {
        out << " EM";
      }
      
      out << " " << iPin->layer()->getConstName() << " ";

      if( iPin->dir() == ClipLayerPin::Direction::UP) {
        out << " UP";
      }
      else if( iPin->dir() == ClipLayerPin::Direction::DOWN) {
        out << " DO";
      }
      else if( iPin->dir() == ClipLayerPin::Direction::SAME) {
        out << " SA";
      }
      
      if( iPin->isFixed() ) {
        out << " FIXED";
      }
      out << endl;
    }
  }
}

Graphics::Graphics(utl::Logger* logger,
    Clip* clip)
  : log_(logger), clip_(clip), net_idx_(-1), wire_idx_(-1) 
{
  gui::Gui::get()->registerRenderer(this);
}
 
void 
Graphics::plot(bool pause) {
  if( pause ) {
    auto gui = gui::Gui::get();
    
    for(int i=0; i<clip_->nets().size(); i++) {
      net_idx_ = i;

      for(int j=0; j<clip_->ecoPaths()[net_idx_].size(); j++) {
        gui->clearFocusNets();
        gui->addFocusNet(clip_->nets()[i]);

        gui->zoomTo(clip_->die());
        wire_idx_ = j;
        gui->pause();
      }
    }
  }
}

void
Graphics::highlightClip(Clip* clip) {
  clip_ = clip;
}

void 
Graphics::drawObjects(gui::Painter& painter) {
  if( clip_ ) {
    odb::Rect die = clip_->die();
    odb::Rect core = clip_->core();

    // die boundary
    painter.setPen(gui::Painter::red, true);
    gui::Painter::Color color = gui::Painter::dark_green;
    color.a = 10;
    painter.setBrush(color);
    painter.drawRect({die.xMin(), die.yMin(), die.xMax(), die.yMax()});

    // core boundary
    painter.setPen(gui::Painter::yellow, true);
    color = gui::Painter::dark_blue;
    color.a = 10;
    painter.setBrush(color);
    painter.drawRect({core.xMin(), core.yMin(), core.xMax(), core.yMax()});

    if( net_idx_ != -1 ) {
      log_->report("  Net {}: {}", net_idx_, 
          clip_->nets()[net_idx_]->getConstName());

      // std::vector<std::vector<WireInfo>> wireInfos = clip_->wireInfos()[net_idx_];
      // std::vector<std::vector<int>> wireMarks = clip_->wireMarks()[net_idx_];

      // log_->report("    WireIdx: {}", wire_idx_); 

      // for(int i=0; i<wireInfos[wire_idx_].size(); i++) {
      //   gui::Painter::Anchor anchor = gui::Painter::Anchor::RIGHT_CENTER;
      //   odb::Rect rect = wireInfos[wire_idx_][i].rect;
      //   painter.drawString(
      //       rect.xMin(), rect.yMin(), anchor,
      //       std::to_string(i));

      //   int wireMark = wireMarks[wire_idx_][i];

      //   log_->report("    Idx: {}, Mark: {}, ERASE/OBS: {}, HasEXTPIN: {}, ZINPUT: {}, ZOUTPUT: {}", 
      //       i, 
      //       getString(wireInfos[wire_idx_][i].mark), 
      //       wireMark & 1,
      //       (wireMark >> 1) & 1,
      //       (wireMark >> 2) & 1,
      //       (wireMark >> 3) & 1); 
      // }
    }
  }
}


PinCoordi::PinCoordi(int lx, int ly, int ux, int uy):
  lx_(lx), ly_(ly), ux_(ux), uy_(uy),
  pin_(nullptr), db_tech_layer_(nullptr) {};

PinCoordi::~PinCoordi() {
  lx_ = ly_ = ux_ = uy_ = 0;
  pin_ = nullptr;
  db_tech_layer_ = nullptr;
}

bool
PinCoordi::isLineSegment() const {
  return (lx_ == ux_) || (ly_ == uy_);
}

bool
PinCoordi::isHorizontal() const {
  return (ly_ == uy_);
}

bool
PinCoordi::isVertical() const {
  return (lx_ == ux_);
}

int
PinCoordi::length() const {
  if( !isLineSegment() ) {
    return -1;
  }

  if( isVertical() ) {
    return uy_ - ly_;
  }
  else if( isHorizontal() ) {
    return ux_ - lx_;
  }

  return -1;
}

void
PinCoordi::setPin(eco::Pin* pin) {
  pin_ = pin;
}

void
PinCoordi::setDbTechLayer(odb::dbTechLayer* layer) {
  db_tech_layer_ = layer;
}


Pin::Pin(odb::dbMTerm* mTerm)
  : db_mterm_(mTerm) {}

Pin::~Pin() {
  reset();
}

void
Pin::reset() {
  pin_coordis_.clear();
  pin_coordis_.shrink_to_fit();
  db_mterm_ = nullptr;
}

void
Pin::addPinCoordi(eco::PinCoordi& coordi) {
  pin_coordis_.push_back(coordi);
}

Master::Master(odb::dbMaster* master)
  : master_(master) {}

Master::~Master() {
  reset();
}

void
Master::reset() {
  master_ = nullptr;
  pins_.clear();
  pins_.shrink_to_fit();
}

int
Master::width() {
  return master_->getWidth();
}

int
Master::height() {
  return master_->getHeight();
}

void
Master::addPin(eco::Pin& pin) {
  pins_.push_back(pin);
}

SMT2ClipLayerPin::SMT2ClipLayerPin (RNet* rnet, int cIdx, ClipLayerPin* clPin)
: rnet_(rnet), 
  commodity_index_(cIdx), 
  clip_layer_pin_(clPin) {}

SMT2Segment::SMT2Segment(int metalLayer, 
    std::string inputStr)
  : str_(inputStr) {

  switch(metalLayer) {
    case 1: 
      order_ = M1AP;
      break;
    case 2:
      order_ = M2;
      break;
    case 3:
      order_ = M3;
      break;
    case 4:
      order_ = M4;
      break;
    default:
      break;
  } 
}

SMT2Segment::SMT2Segment(int lowerLayer, 
    int upperLayer, 
    std::string inputStr) 
  : str_(inputStr) {

  if( lowerLayer +1 != upperLayer ) {
    logger->error(ECO, 4823, "given VIA layer is wrong {} -> {} (expected lowerLayer + 1 == upperLayer)", lowerLayer, upperLayer);
  }

  switch(lowerLayer) {
    case 1: 
      order_ = VIA12;
      break;
    case 2:
      order_ = VIA23; 
      break;
    case 3:
      order_ = VIA34;
      break;
    default:
      break;
  }
}

CoordiInfo::CoordiInfo (int layer, int y, int x, Clip* clip)
  : layer_(layer),
  y_(y), 
  x_(x),
  clip_(clip) {}

void
CoordiInfo::setClip(Clip* clip) {
  clip_ = clip;
}

void
CoordiInfo::addSMT2ClipLayerPin(SMT2ClipLayerPin smtClPin) {
  smt2_pins_.push_back(smtClPin);
}

std::vector<SMT2ClipLayerPin*>
CoordiInfo::getSMT2ClipLayerPins(RNet* rNet, int cIdx) {
  std::vector<SMT2ClipLayerPin*> ret;

  SMT2ClipLayerPin tmp(rNet, cIdx, nullptr);
  for(auto& smt2Pin : smt2_pins_) {
    if( smt2Pin.getKey() == tmp.getKey() ) {
      ret.push_back(&smt2Pin);
    }
  }
  return ret;
}

std::vector<SMT2ClipLayerPin*>
CoordiInfo::getSMT2ClipLayerPins(RNet* rNet) {
  std::vector<SMT2ClipLayerPin*> ret;
  for(auto& smt2Pin : smt2_pins_) {
    if( smt2Pin.rNet() == rNet ) {
      ret.push_back(&smt2Pin);
    }
  }
  return ret;
}

std::map<ClipLayerPin*, std::vector<SMT2ClipLayerPin*>>
CoordiInfo::getSMT2ClipLayerPinsMap(RNet* rNet) {
  std::map<ClipLayerPin*, std::vector<SMT2ClipLayerPin*>> ret;
  for(auto& smt2Pin : getSMT2ClipLayerPins(rNet)) {
    auto ptr = ret.find(smt2Pin->clipLayerPin());
    if( ptr == ret.end() ) {
      std::vector<SMT2ClipLayerPin*> tmp;
      tmp.push_back(smt2Pin);

      ret[smt2Pin->clipLayerPin()] =tmp; 
    }
    else {
      ret[smt2Pin->clipLayerPin()].push_back(smt2Pin);
    }
  }
  return ret;
}

std::vector<SMT2Segment>
CoordiInfo::getAllStringsCIdx(RNet* rNet, int cIdx) {
  std::vector<SMT2Segment> ret1, ret2;
  ret1 = getSMT2StringsCIdx(rNet, cIdx);
  ret2 = getAdjStrings();

  ret1.insert( ret1.end(), ret2.begin(), ret2.end() );
  return ret1;
}

std::vector<SMT2Segment>
CoordiInfo::getAllStrings(RNet* rNet, bool isViaEnclosure) {
  std::vector<SMT2Segment> ret1, ret2;
  ret1 = getSMT2Strings(rNet, isViaEnclosure);
  ret2 = getAdjStrings();
  
  ret1.insert( ret1.end(), ret2.begin(), ret2.end() );
  return ret1;
}

std::vector<SMT2Segment>
CoordiInfo::getSMT2StringsCIdx(RNet* rNet, int cIdx) {
  std::vector<SMT2Segment> ret;

  // return all smt2 pin
  for(auto& smt2Pin : getSMT2ClipLayerPins(rNet, cIdx)) {
    ClipLayerPin* clPin = smt2Pin->clipLayerPin();

    // if CoordiInfo is the same as layer_
    if( layer_ == clPin->layer()->getRoutingLevel() ) {
      ret.push_back( 
          SMT2Segment(
            layer_, 
          "m" + to_string(layer_)
          + "r" + to_string(y_)
          + "c" + to_string(x_) 
          + "_" 
          + clPin->getVertName()));

      // VIA12 handling
      if( layer_ == itermLayerNum ) {
        std::pair<int, int> pair = clip_->getNextLayerRowCol(
            clip_->dbToEb(clPin->layer()), y_, x_);

        if( pair.first != -1 && pair.second != -1 ) {
          ret.push_back(
              SMT2Segment(
                layer_, layer_+1, 
              "m" + to_string(layer_)
              + "r" + to_string(y_)
              + "c" + to_string(x_) 
              + "_" 
              + "m" + to_string(layer_+1)
              + "r" + to_string(pair.first)
              + "c" + to_string(pair.second)));
        }
      }
    }
    // retrieve M1-M2 VIA connection
    else if ( layer_ == itermLayerNum + 1 &&
        clPin->layer()->getRoutingLevel() == itermLayerNum ) {

      std::pair<int, int> pair = clip_->getPrevLayerRowCol(
          clip_->dbToEb(layer_), y_, x_);

      if( pair.first != -1 && pair.second != -1) {
        ret.push_back(
            SMT2Segment(
              layer_-1, layer_, 
            "m" + to_string(layer_-1)
            + "r" + to_string(pair.first)
            + "c" + to_string(pair.second) 
            + "_" 
            + "m" + to_string(layer_)
            + "r" + to_string(y_)
            + "c" + to_string(x_)));
      }
    }
  }
  return ret;
}

std::vector<SMT2Segment>
CoordiInfo::getSMT2Strings(RNet* rNet, bool isViaEnclosure) {
  std::vector<SMT2Segment> ret;

  // return all smt2 pin
  for(auto& smt2Pin : getSMT2ClipLayerPins(rNet)) {
    ClipLayerPin* clPin = smt2Pin->clipLayerPin();

    // if CoordiInfo is the same as layer_
    if( layer_ == clPin->layer()->getRoutingLevel() ) {
      // for the VIA enclosure, the EXTPIN should not be returned
      // if it has DOWN  or UP directions!
      if( !isViaEnclosure 
          || (isViaEnclosure && clPin->dir() == ClipLayerPin::Direction::SAME) ) { 
        ret.push_back( 
            SMT2Segment(
              layer_, 
              "m" + to_string(layer_)
              + "r" + to_string(y_)
              + "c" + to_string(x_) 
              + "_" 
              + clPin->getVertName()));
      }
      // cout << "!!!!! CLPIN TEST: type: " << clPin->type() << " dir:" << clPin->dir() << " vertName:" << clPin->getVertName() << endl;

      // VIA12 handling
      if( layer_ == itermLayerNum ) {
        std::pair<int, int> pair = clip_->getNextLayerRowCol(
            clip_->dbToEb(clPin->layer()), y_, x_);

        if( pair.first != -1 && pair.second != -1 ) {
          ret.push_back(
              SMT2Segment(
                layer_, layer_+1, 
              "m" + to_string(layer_)
              + "r" + to_string(y_)
              + "c" + to_string(x_) 
              + "_" 
              + "m" + to_string(layer_+1)
              + "r" + to_string(pair.first)
              + "c" + to_string(pair.second)));
        }
      }
    }
    // retrieve M1-M2 VIA connection
    else if ( layer_ == itermLayerNum + 1 &&
        clPin->layer()->getRoutingLevel() == itermLayerNum ) {
      
      std::pair<int, int> pair = clip_->getPrevLayerRowCol(
          clip_->dbToEb(layer_), y_, x_);

      if( pair.first != -1 && pair.second != -1) {
        ret.push_back(
            SMT2Segment(
              layer_-1, layer_, 
            "m" + to_string(layer_-1)
            + "r" + to_string(pair.first)
            + "c" + to_string(pair.second) 
            + "_" 
            + "m" + to_string(layer_)
            + "r" + to_string(y_)
            + "c" + to_string(x_)));
      }
    }
  }
  return ret;
}

std::vector<SMT2Segment>
CoordiInfo::getAdjStrings() {
  std::vector<SMT2Segment> ret;

  for(auto& adjSeg : all_adj_segments_) {
    std::string commStr 
      = "m" + to_string(adjSeg.first->layer()->getRoutingLevel())
         + "r" + to_string(adjSeg.first->y())
         + "c" + to_string(adjSeg.first->x())
         + "_"
         + "m" + to_string(adjSeg.second->layer()->getRoutingLevel())
         + "r" + to_string(adjSeg.second->y())
         + "c" + to_string(adjSeg.second->x());

    if( adjSeg.first->layer() == adjSeg.second->layer() ) {  
      ret.push_back(
        SMT2Segment(
          adjSeg.first->layer()->getRoutingLevel(), 
          commStr));
    }
    else {
      ret.push_back(
        SMT2Segment(
          adjSeg.first->layer()->getRoutingLevel(), 
          adjSeg.second->layer()->getRoutingLevel(),
          commStr));
    }
  }
  return ret;
}

static void
ParseDbBox(odb::dbBox* box, eco::Pin& curPin, utl::Logger* log_) {

  odb::dbTechLayer* layer = box->getTechLayer();
  if( layer->getType() != odb::dbTechLayerType::ROUTING ) {
    return;
  }

  auto layerDir = layer->getDirection();
  auto pitch = layer->getPitch();
  auto metalWidth = pitch/2;

  debugPrint(log_, ECO, "core_eco", 3, "  ParseDbBox : box info {} {} {} {}, layer: {}, pitch: {}, width: {}",
        box->xMin(), box->yMin(),
        box->xMax(), box->yMax(),
        layer->getConstName(),
        pitch, metalWidth);

  int lx = 0, ly = 0, ux = 0, uy = 0;
  if( layerDir == odb::dbTechLayerDir::VERTICAL) {
    int cx = (box->xMin() + box->xMax())/2 - metalWidth;
    lx = ux = cx/pitch;
    ly = (box->yMin() - metalWidth + metalWidth) / pitch;
    uy = (box->yMax() - metalWidth - metalWidth) / pitch;

    int lyMod = (box->yMin() - metalWidth + metalWidth) % pitch;
    int uyMod = (box->yMin() - metalWidth + metalWidth) % pitch;
    if( lyMod != 0 || uyMod != 0 ) {
      log_->warn(ECO, 101, "Unsupported LEF");
    }
  }
  else if( layerDir == odb::dbTechLayerDir::HORIZONTAL) {
    int cy = (box->yMin() + box->yMax())/2 - metalWidth;
    ly = uy = cy/pitch;
    lx = (box->xMin() - metalWidth + metalWidth/2) / pitch;
    ux = (box->xMax() - metalWidth - metalWidth/2) / pitch;

    int lxMod = (box->xMin() - metalWidth + metalWidth/2) % pitch;
    int uxMod = (box->xMax() - metalWidth - metalWidth/2) % pitch;

    if( lxMod != 0 || uxMod != 0 ) {
      log_->warn(ECO, 102, "Unsupported LEF");
    }
  }
  else {
    log_->error(ECO, 100, "No preferred direction on layer %s",
        layer->getConstName());
  }

  debugPrint(log_, ECO, "core_eco", 3, "  ParseDbBox : grid coordi {} {} {} {}",
      lx, ly, ux, uy);

  eco::PinCoordi curCoordi(lx, ly, ux, uy);
  curCoordi.setDbTechLayer(layer);
  curPin.addPinCoordi(curCoordi);
}


EcoBaseVars::EcoBaseVars()
  : swBoxSolveRangeX(0), swBoxSolveRangeY(0),
  swBoxObsRangeX(0), swBoxObsRangeY(0),
  fromMetalBlockLayer(0), toMetalBlockLayer(0),
  localStepX(0), localStepY(0),
  localCntX(0), localCntY(0),
  slackThreshold(0), 
  writeZ3FileMode(false), 
  readZ3FileMode(false),
  guiMode(false) {}

void
EcoBaseVars::reset() {
  swBoxSolveRangeX = swBoxSolveRangeY = 0;
  swBoxObsRangeX = swBoxObsRangeY = 0;
  fromMetalBlockLayer = toMetalBlockLayer = 0;
  localStepX = localStepY = 0;
  localCntX = localCntY = 0;
  slackThreshold = 0.0f;
  writeZ3FileMode = false;
  readZ3FileMode = false;
  guiMode = false;
}

EcoOdbNet::EcoOdbNet()
: is_init_(false) {}

void
EcoOdbNet::init(odb::dbSet <odb::dbNet> nets) {
  is_init_ = true;

  int totalRectCnt = 0;
  int totalNetCnt = 0; 
  for(odb::dbNet* net : nets) {
    odb::dbWire* curWire = net->getWire();
    if( curWire ) {
      nets_map_[net] = rects_.size(); 

      std::vector<RectInfo> netRects;
      std::map<tuple<int, int, int>, int> netRectsMap;

      odb::dbWireDecoder decoder;
      decoder.begin(curWire);

      odb::dbTechLayer* layer = nullptr;

      int x1 = -1, y1 = -1, x3 = -1, y3 = -1;
      int tmpExt = -1;

      // current point coordinate (important)
      int cx = -1, cy = -1;

      odb::dbWireDecoder::OpCode prevCode 
        = odb::dbWireDecoder::END_DECODE;

      while( 1 ) {
        odb::dbWireDecoder::OpCode opcode = decoder.next();
        if( opcode == odb::dbWireDecoder::END_DECODE ) {
          break;
        }

        switch(opcode) {
          case odb::dbWireDecoder::PATH:
            prevCode = opcode;
            layer = decoder.getLayer();

            if (decoder.peek() == odb::dbWireDecoder::RULE) {
              prevCode = opcode;
              opcode = decoder.next();
            }
            break;

          case odb::dbWireDecoder::JUNCTION: {
            prevCode = opcode;
            opcode = decoder.peek();
            if (opcode == odb::dbWireDecoder::RULE) {
              opcode = decoder.next();
              opcode = decoder.peek();
            }
            if (opcode == odb::dbWireDecoder::POINT_EXT) {
              prevCode = opcode;
              opcode = decoder.next();
              decoder.getPoint(cx, cy, tmpExt);
              break;
            }
            else if (opcode == odb::dbWireDecoder::POINT) {
              prevCode = opcode;
              opcode = decoder.next();
              decoder.getPoint(cx, cy);
            }
          }
          case odb::dbWireDecoder::SHORT:
          case odb::dbWireDecoder::VWIRE:
            prevCode = opcode;
            layer = decoder.getLayer();
            break;
          case odb::dbWireDecoder::RECT:
            decoder.getRect(x1,y1,x3,y3);
            // push rect to data structure
            if( layer != nullptr && cx != -1 && cy != -1) {
              netRectsMap[ 
                tuple<int, int, int>(cx, cy, 
                    layer->getRoutingLevel()) ] 
                = netRects.size(); 
              netRects.push_back(RectInfo(prevCode, odb::Rect(x1, y1, x3, y3)));
            }
            prevCode = opcode;
            break;

          case odb::dbWireDecoder::POINT:
            prevCode = opcode;
            decoder.getPoint(cx, cy);
            break;
          case odb::dbWireDecoder::POINT_EXT:
            prevCode = opcode;
            decoder.getPoint(cx, cy, tmpExt);
            break;

          case odb::dbWireDecoder::TECH_VIA:
          case odb::dbWireDecoder::VIA:
            layer = decoder.getLayer();
          case odb::dbWireDecoder::ITERM:
          case odb::dbWireDecoder::BTERM:
            prevCode = opcode;
            break;

          default:
            break;
        }
      }

      totalRectCnt += netRects.size();
      if( netRects.size() >= 1 ) {
        totalNetCnt += 1;
      }
      rects_.push_back(netRects);
      rects_map_.push_back(netRectsMap);
    }
  }

  logger->info(ECO, 905, "Found total rects in wires: {}, rect-related nets: {}", 
      totalRectCnt, totalNetCnt);

  for(odb::dbNet* net : nets) {
    auto nmPtr = nets_map_.find(net);
    if( nmPtr == nets_map_.end() ) {
      continue;
    } 
    else {
      for(auto& val : rects_map_[nmPtr->second]) {
        tuple<int, int, int> t = val.first;
        using std::get;

        RectInfo& rectInfo = rects_[nmPtr->second][val.second];

        debugPrint(logger, ECO, "core_eco", 5, 
            "RectInitInfo: {}: xym: {} {} {} - {} - ({} {}) ({} {})", 
            net->getConstName(), 
            get<0>(t), get<1>(t), get<2>(t),
            rectInfo.prevOpcode,
            rectInfo.rect.xMin(), 
            rectInfo.rect.yMin(),
            rectInfo.rect.xMax(), 
            rectInfo.rect.yMax());
      }
    }
  }
}

RectInfo
EcoOdbNet::getRect(odb::dbNet* net, int layer, int x, int y) {
  auto nmPtr = nets_map_.find(net);
  if( nmPtr == nets_map_.end() ) {
    return RectInfo(odb::dbWireDecoder::END_DECODE, odb::Rect(-1,-1,-1,-1));
  }
  else {
    int curNetIdx = nmPtr->second;
    std::vector<RectInfo>& curRects = rects_[curNetIdx];
    std::map<tuple<int, int, int>, int>& curRectMap = rects_map_[curNetIdx];

    auto rmPtr = curRectMap.find(tuple<int, int, int>(x,y,layer));
    if( rmPtr == curRectMap.end() ){
      return RectInfo(odb::dbWireDecoder::END_DECODE, odb::Rect(-1,-1,-1,-1));
    }
    else {
      return curRects[rmPtr->second];
    }
  }
}


EcoBase::EcoBase(
    EcoBaseVars& ebVars, 
    odb::dbDatabase* odb,
    sta::dbSta* dbSta, 
    utl::Logger* log,
    int verbose)
  : db_(odb), sta_(dbSta), log_(log),
  iterm_rtree_(nullptr), 
  odb_rect_nets_info_(EcoOdbNet()),
  verbose_(verbose) {
  eb_vars_ = ebVars;
  logger = log;
}

EcoBase::~EcoBase() {
  reset();
}

void
EcoBase::run() {
  odb::dbBlock* block = db_->getChip()->getBlock();
  
  if( iterm_rtree_ == nullptr || wire_rtree_ == nullptr ) {
    initRTree();
  }

  log_->info(ECO, 3, "#DRV Candidates: {}", drc_coordis_.size());  

  // to avoid overlaps between each clip
  std::vector<odb::Rect> allRects;

  // save all Rect before making all clips
  odb_rect_nets_info_.init(block->getNets());

  for(auto& coordi: drc_coordis_) {
    debugPrint(log_, ECO, "core_eco", 3, "run: drc_coordis_ {} {}", 
        coordi.first, coordi.second);

    int cntOffsetX = (eb_vars_.localCntX-1)/2;
    int cntOffsetY = (eb_vars_.localCntY-1)/2;

    for(int i=-cntOffsetX; i<=cntOffsetX; i++) {
      for(int j=-cntOffsetY; j<=cntOffsetY; j++) {
        int offsetX = i * eb_vars_.localStepX;
        int offsetY = j * eb_vars_.localStepY;

        eco::Clip clip = extractClip(
            coordi.first + offsetX, 
            coordi.second + offsetY, 
            allRects);

        if( clip.isFailed() )  {
          log_->info(ECO, 8412, 
              "CoRe-ECO was failed to generate an feasible SMT2. " 
              "CoRe-ECO will skip for generating a clip at {} {}",
              coordi.first + offsetX, 
              coordi.second + offsetY);
        }
        // for the successful clip
        else {
          // update allRect
          allRects.push_back(clip.die());
          int curDrvIdx = &coordi - &drc_coordis_[0];

          std::string key = to_string(curDrvIdx) 
            + "_" + to_string(i + cntOffsetX)
            + "_" + to_string(j + cntOffsetY);

          if( eb_vars_.writeZ3FileMode ) {
            clip.writeSMT2(eb_vars_.writeZ3DirName 
                + "/drv_" + key
                + ".smt2");
            clip.writeSMT2Helper(eb_vars_.writeZ3DirName 
                + "/drv_" + key
                + ".smt2.helper");
          }

          if( eb_vars_.readZ3FileMode ) {
            // the odbNetInfo is required for "RECT"
            clip.readBackSMT2(eb_vars_.readZ3DirName 
                + "/drv_" + key 
                + ".smt2.route", 
                odb_rect_nets_info_,
                rect_reverted_ordered_nets_ );
            // for sanity check
            // if( readResult ) {
            //   ord::OpenRoad* curOrd = ord::OpenRoad::openRoad();
            //   curOrd->writeDb(string("aes_routed_" + key + ".db").c_str());
            // }
          } 
        }
      }
    }
    //break;
  }

  runPostProcessRectNets();
  log_->info(ECO, 4, "Program finished");
}

bool
EcoBase::runGenSMT2(std::string fileName, int cx, int cy, int origLx, int origLy, int origUx, int origUy) {
  if( iterm_rtree_ == nullptr || wire_rtree_ == nullptr ) {
    initRTree();
  }

  if( odb_rect_nets_info_.isInit() == false ) {
    odb::dbBlock* block = db_->getChip()->getBlock();
    odb_rect_nets_info_.init(block->getNets());
  }

  incremental_clip_.reset();
  incremental_clip_ = extractClip(cx, cy, passed_rects_, origLx, origLy, origUx, origUy);
  if( incremental_clip_.isFailed() ) {
    log_->info(ECO, 8413, 
        "CoRe-ECO was failed to generate an feasible SMT2. " 
        "CoRe-ECO will skip for generating a clip at {} {}",
        cx, cy);
    return false;
  }

  incremental_clip_.writeSMT2(fileName);
  incremental_clip_.writeSMT2Helper(fileName+".helper");
  return true;
}

bool
EcoBase::runReadSMT2(std::string fileName, int lx, int ly, int ux, int uy) {
  if( iterm_rtree_ == nullptr || wire_rtree_ == nullptr ) {
    initRTree();
  }

  if( odb_rect_nets_info_.isInit() == false ) {
    odb::dbBlock* block = db_->getChip()->getBlock();
    odb_rect_nets_info_.init(block->getNets());
  }
  
  cout << "given: " << lx << " " << ly << " " << ux << " " <<  uy << endl;

  // note that given rect is coreRect
  odb::Rect searchRect(lx, ly, ux, uy);
  if( incremental_clip_.core() == searchRect ) {
    return incremental_clip_.readBackSMT2(fileName, 
        odb_rect_nets_info_, rect_reverted_ordered_nets_);
  }
  else {
    logger->warn(ECO, 1105, "Cannot find the generated smt2: {}", fileName);
  }
  return false;
}

// needed for incremental rect save to avoid the overlapped regions
void
EcoBase::runSaveCurRect() {
   passed_rects_.push_back(incremental_clip_.die());
}

void
EcoBase::runPostProcessRectNets() {
  for(auto& orderedNet : ordered_nets_ ) {
    log_->info(ECO, 593, "post-process attach nets: {}", orderedNet->getConstName());
    // dumpDecoder4Net(orderedNet);
    odb::dbWire* newWire 
      = getNewWireForRectsAndRemoveOverlap(orderedNet->getWire(), 
          odb_rect_nets_info_,
          eb_vars_.fromMetalRouteLayer,
          eb_vars_.toMetalRouteLayer);

    if( newWire ) {
      odb::dbWire::destroy(orderedNet->getWire());
      newWire->attach(orderedNet);
    }
  }
}

void
EcoBase::reset() {
  db_ = nullptr;
  sta_ = nullptr;
  log_ = nullptr;

  eb_vars_.reset();
  verbose_ = 0;

  ITermRTree* iTermRTree = (ITermRTree*) iterm_rtree_;
  WireRTree* wireRTree = (WireRTree*) wire_rtree_;

  delete iTermRTree;
  delete wireRTree;

  iterm_rtree_ = nullptr;
  wire_rtree_ = nullptr;

  // DRC coordinates should not be erased
  // drc_coordis_.clear(); 
  // drc_coordis_.shrink_to_fit();

  clips_.clear();
  clips_.shrink_to_fit();
  ordered_nets_.clear();
  rect_reverted_ordered_nets_.clear();
}

void
EcoBase::initRTree() {
  odb::dbBlock* block = db_->getChip()->getBlock();

  // allocate new tree
  iterm_rtree_ = (void*) (new ITermRTree); 
  ITermRTree* iTermRTree = (ITermRTree*) iterm_rtree_;

  for( odb::dbInst* inst : block->getInsts()) {
    for(odb::dbITerm* iTerm : inst->getITerms()) {
      odb::Rect rect = iTerm->getBBox();
      box b (point(rect.xMin(), rect.yMin()),
          point(rect.xMax(), rect.yMax()));
      iTermRTree->insert( make_pair(b, iTerm) );
    }
  }

  wire_rtree_ = (void*) (new WireRTree);
  WireRTree* wireRTree = (WireRTree*) wire_rtree_;
  for( odb::dbNet* net : block->getNets() ) {
    odb::dbWire* wire = net->getWire();
    if( wire ) {
      odb::dbWireGraph graph;
      graph.decode(wire);

      for(odb::dbWireGraph::edge_iterator 
          edgeIter = graph.begin_edges();
          edgeIter != graph.end_edges();
          edgeIter ++) {
        odb::dbWireGraph::Edge* edge = *edgeIter;

        int x1=-1, y1=-1, x3=-1, y3=-1;
        edge->source()->xy(x1,y1);
        edge->target()->xy(x3,y3);

        int xMin = std::min(x1,x3);
        int yMin = std::min(y1,y3);
        int xMax = std::max(x1,x3);
        int yMax = std::max(y1,y3);

        // only for a segment - 
        if( edge->type() 
            == odb::dbWireGraph::Edge::Type::SEGMENT ) {
          int layer = edge->source()->layer()->getRoutingLevel();
          
          // within metal route layer
          if( eb_vars_.fromMetalRouteLayer <= layer &&
              layer <= eb_vars_.toMetalRouteLayer ) {
            //segment s (point(xMin, yMin), 
            //    point(xMax, yMax));
            box b (point(xMin, yMin), 
                point(xMax, yMax));
            wireRTree->insert( make_pair(b, net ));
          }          
        } 
      }
    } 
  }

  log_->info(ECO, 1, "Initializing RTree is done");  
}

static bool
isInstFixed(odb::dbInst* inst) {
  switch( inst->getPlacementStatus() ) {
    case odb::dbPlacementStatus::NONE:
    case odb::dbPlacementStatus::UNPLACED:
    case odb::dbPlacementStatus::SUGGESTED:
    case odb::dbPlacementStatus::PLACED:
      return false;
      break;
    case odb::dbPlacementStatus::LOCKED:
    case odb::dbPlacementStatus::FIRM:
    case odb::dbPlacementStatus::COVER:
      return true;
      break;
  }
  return false;
}

static int
getSnapCoordi(int x, int dx, int minX, int maxX) {
  // projection x between minX <= x <= maxX
  x = std::max(x, minX);
  x = std::min(x, maxX);

  return static_cast<int>(round(1.0 * x / dx)) * dx;
}

eco::Clip
EcoBase::extractClip(int cx, int cy, 
    std::vector<odb::Rect>& allRects, 
    int origLx, int origLy,
    int origUx, int origUy) {
  odb::dbBlock* block = db_->getChip()->getBlock();
  ITermRTree* iTermRTree = (ITermRTree*) iterm_rtree_;
  WireRTree* wireRTree = (WireRTree*) wire_rtree_;

  // retrieve boxes
  odb::Rect coreBox = block->getCoreArea();

  int boxMarginX = eb_vars_.swBoxObsRangeX/2 - eb_vars_.swBoxSolveRangeX/2;
  int boxMarginY = eb_vars_.swBoxObsRangeY/2 - eb_vars_.swBoxSolveRangeY/2;
  log_->info(ECO, 3254, "given obsRange: {} {} - solveRange: {} {}", 
      eb_vars_.swBoxObsRangeX, eb_vars_.swBoxObsRangeY, 
      eb_vars_.swBoxSolveRangeX, eb_vars_.swBoxSolveRangeY);

  // error handling when OBS size < SOLVE size
  if( boxMarginX <= 0 || boxMarginY <= 0 ) {
    if( db_->getTech()->getRoutingLayerCount() < 3 ) {
      log_->error(ECO, 130, 
        "Box sizes (obs/solve) are wrong. "
        "The CoRe-ECO tried to set up default parameter, but "
        "failed because #routing layers < 3");
    }
    else {
      odb::dbTechLayer* m1 = db_->getTech()->findRoutingLayer(1);
      odb::dbTechLayer* m2 = db_->getTech()->findRoutingLayer(2);
      
      boxMarginX = 
        (m1->getDirection() 
         == odb::dbTechLayerDir::VERTICAL)?  
        m1->getPitchX() * 2 : m2->getPitchX() * 2;
      boxMarginY = 
        (m2->getDirection()
         == odb::dbTechLayerDir::HORIZONTAL)?
        m2->getPitchY() * 2 : m1->getPitchY() * 2;
     
      log_->warn(ECO, 131,
        "Box margin sizes (obs/solve) are reset because "
        "obs size was smaller than solve size: {} {}", 
        boxMarginX, boxMarginY);
    }
  }
  log_->info(ECO, 3253, "boxMarginX: {}, boxMarginY: {}", boxMarginX, boxMarginY);

  // current (lx,ly,ux,uy) solve box size
  int curSolveLx = cx - eb_vars_.swBoxSolveRangeX/2;
  int curSolveLy = cy - eb_vars_.swBoxSolveRangeY/2;
  int curSolveUx = cx + eb_vars_.swBoxSolveRangeX/2;
  int curSolveUy = cy + eb_vars_.swBoxSolveRangeY/2;

  odb::dbTechLayer* bottomRouteLayer = db_->getTech()->findRoutingLayer(eb_vars_.fromMetalRouteLayer);
  int unitX = bottomRouteLayer->getPitchY();
  int unitY = bottomRouteLayer->getPitchX();

  // snapping SW solve grids toward site box inside core area
  curSolveLx = getSnapCoordi(curSolveLx - coreBox.xMin(), 
      unitX,
      0,
      coreBox.dx()) + coreBox.xMin();
  curSolveLy = getSnapCoordi(curSolveLy - coreBox.yMin(), 
      unitY,
      0,
      coreBox.dy()) + coreBox.yMin();
  curSolveUx = getSnapCoordi(curSolveUx - coreBox.xMin(), 
      unitX,
      0,
      coreBox.dx()) + coreBox.xMin();
  curSolveUy = getSnapCoordi(curSolveUy - coreBox.yMin(), 
      unitY,
      0,
      coreBox.dy()) + coreBox.yMin();

  // escape the outlier cases
  if( curSolveLx == curSolveUx || curSolveLy == curSolveUy ) {
    eco::Clip retClip;
    retClip.setIsFailed(true);
    logger->warn(ECO, 1100, "No feasible solution box found.");
    return retClip;
  }

  odb::Rect obsRect(curSolveLx - boxMarginX, 
      curSolveLy - boxMarginY, 
      curSolveUx + boxMarginX, 
      curSolveUy + boxMarginY);

  // overlap happen, skip
  for(auto& prevRect : allRects) {
    if( prevRect.overlaps(obsRect) ) {
      eco::Clip retClip;
      retClip.setIsFailed(true);
      logger->warn(ECO, 1101, "Current box ({} {}) - ({} {}) was considered before: ({} {}) - ({} {})",
        obsRect.xMin(), obsRect.yMin(), obsRect.xMax(), obsRect.yMax(), 
        prevRect.xMin(), prevRect.yMin(), prevRect.xMax(), prevRect.yMax());
      return retClip;
    }
  }

  odb::Rect solveRect(curSolveLx, curSolveLy, 
      curSolveUx, curSolveUy);

  if( origLx != -1 && origLy != -1 && origUx != -1 && origUy != -1 ) {
    if( !( curSolveLx <= origLx && origUx <= curSolveUx &&
        curSolveLy <= origLy && origUy <= curSolveUy) ) {
      eco::Clip retClip;
      retClip.setIsFailed(true);
      logger->warn(ECO, 1102, "Current solve box ({} {}) - ({} {}) was outside of orig box({} {}) - ({} {})",
          curSolveLx, curSolveLy, curSolveUx, curSolveUy,
          origLx, origLy, origUx, origUy);
      return retClip;
    }
  }

  box queryBox( 
      point(obsRect.xMin(), obsRect.yMin()), 
      point(obsRect.xMax(), obsRect.yMax()));

  log_->info(ECO, 10, "Query OBSBox ({} {}) - ({} {}) to RTree", 
      obsRect.xMin(), obsRect.yMin(),
      obsRect.xMax(), obsRect.yMax());

  log_->info(ECO, 11, "SolveBox ({} {}) - ({} {})", 
      solveRect.xMin(), solveRect.yMin(),
      solveRect.xMax(), solveRect.yMax());

  // send region query to RTree
  vector<valueITerm> foundITerms;
  iTermRTree->query(bgi::intersects(queryBox),
      std::back_inserter(foundITerms));

  // All insts inside ObsRange
  set<odb::dbInst*> insts;

  // movable == within SolveRange rect
  set<odb::dbInst*> movableInsts;

  // fixed == overlapped with SolveRange rect,
  // and
  // outside of SolveRange, but intersects with ObsRange
  set<odb::dbInst*> fixedInsts;
  
  // nets within ObsRange
  set<odb::dbNet*> nets;

  for(valueITerm& val : foundITerms) {
    odb::dbITerm* iTerm = val.second;
    insts.insert(iTerm->getInst());
    odb::dbBox* bBox = iTerm->getInst()->getBBox();

    if( isWithInBox( bBox, solveRect) ) {
      if( isInstFixed(iTerm->getInst()) ) {
        fixedInsts.insert(iTerm->getInst());
      }
      else {
        movableInsts.insert(iTerm->getInst());
      }
    } 
    else {
      fixedInsts.insert(iTerm->getInst());
    }

    nets.insert(iTerm->getNet()); 
  }

  vector<valueWire> foundNets;
  wireRTree->query(bgi::intersects(queryBox),
      std::back_inserter(foundNets));
  
  // add missing nets using wire RTree
  for(valueWire& val : foundNets) {
    odb::dbNet* net = val.second;
    nets.insert(net);
  }
  

  // for gui debugging
  std::vector<std::vector<std::vector<eco::Clip::EcoPath>>> totalEcoPaths;

  // stor ptrs as vector  
  std::vector<odb::dbNet*> netList;
  std::vector<odb::dbInst*> instList;
  std::vector<odb::dbInst*> movableInstList;
  std::vector<odb::dbInst*> fixedInstList;

  // map to vector conversion
  for(auto& fixedInst : fixedInsts) {
    fixedInstList.push_back(fixedInst);
    instList.push_back(fixedInst);
  }

  // map to vector conversion
  for(auto& movableInst : movableInsts) {
    movableInstList.push_back(movableInst);
    instList.push_back(movableInst);
  }

  // map to vector conversion - net List
  for(auto& net : nets) {
    if( ordered_nets_.find(net) == ordered_nets_.end() ) {
      ordered_nets_.insert(net);

      //cout << "BEFORE ORDER WIRE" << endl;
      //dumpDecoder4Net(net);

      // order wires here
      odb::orderWires(log_, net);

      //cout << "AFTER ORDER WIRE" <<  endl;
      //dumpDecoder4Net(net);
    }

    // skip for NON WIRE existing netlist
    if( net->getWire() ) {  
      netList.push_back(net);
    }
  }

  // define clips
  eco::Clip retClip(db_,
      eb_vars_,
      obsRect, solveRect,
      instList,
      movableInstList,
      fixedInstList, 
      netList);

  if( retClip.isFailed() ) {
    logger->warn(ECO, 1103, "Current solve box ({} {}) - ({} {}), (orig box: {} {} {} {}) failed to generate clips",
      curSolveLx, curSolveLy, curSolveUx, curSolveUy, 
      origLx, origLy, origUx, origUy);
    return retClip;
  }
  
  log_->info(ECO, 12, "TotalInsts: {}, MovableInsts: {}, FixedInsts: {}, RelatedNets: {}", 
      retClip.insts().size(),
      retClip.movableInsts().size(), 
      retClip.fixedInsts().size(), 
      retClip.nets().size());

  std::vector<eco::Clip::EcoNode>& nodes = retClip.nodes();
  std::vector<eco::Clip::EcoEdge>& edges = retClip.edges();

  for(odb::dbNet* net: netList) {
    odb::dbWire* wire = net->getWire();

    debugPrint(log_, ECO, "core_eco", 3, "Net: {}", 
        net->getConstName());

    // takes input for Opins' (x,y) coordinates,
    // and generate output as rnets_'s index.
    std::unordered_map<std::pair<int, int>, int, PairHash> rNetMap;

    // 1D flatten path with parent idx. Needed to build the 2D vector.
    std::vector<eco::Clip::EcoPath> flatPath;

    // 2D paths. 
    // list of 
    //   (source -> sink1), 
    //   (source -> sink2), 
    //   (source -> sink3),
    //  ...
    std::vector<std::vector<eco::Clip::EcoPath>> netPaths;

    // get node feature -> retrieve flatPath's idx
    std::map<tuple<int, int, int>, int> nodesMap;

    odb::dbWireGraph graph;
    graph.decode(wire);

    for(odb::dbWireGraph::edge_iterator 
        edgeIter = graph.begin_edges();
        edgeIter != graph.end_edges();
        edgeIter ++) {
      odb::dbWireGraph::Edge* edge = *edgeIter;

      int x1=-1, y1=-1, x3=-1, y3=-1;
      edge->source()->xy(x1,y1);
      edge->target()->xy(x3,y3);

      odb::dbObject* sourceObj = edge->source()->object();
      odb::dbObject* targetObj = edge->target()->object();

      debugPrint(logger, ECO, "core_eco", 5,
          "  Current edgeType: {}, wireType: {}", 
          edge->type(),
          edge->wireType());

      debugPrint(logger, ECO, "core_eco", 5,
          "  fromNode: {} {} {} -> toNode: {} {} {}", 
          x1,y1,
          edge->source()->layer()->getConstName(),
          x3,y3,
          edge->target()->layer()->getConstName());

      eco::Clip::EcoNode eNode(x1, y1, edge->source()->layer());

      // update iterm if exists
      if( sourceObj ) {
        if( sourceObj->getObjName() == string("dbITerm") ) {
          odb::dbITerm* iterm 
            = odb::dbITerm::getITerm(block, sourceObj->getId());

          debugPrint(logger, ECO, "core_eco", 5, 
              "  fid: {} source Node: {} {}/{}",
              flatPath.size(), 
              sourceObj->getObjName(),
              iterm->getInst()->getConstName(), 
              iterm->getMTerm()->getConstName() );

          if( flatPath.size() != 0 ) {
            debugPrint(logger, ECO, "core_eco", 5, 
            "  source Node meets special case - reset flatPath: {}",
            flatPath.size());
            flatPath.clear();
            nodesMap.clear();
          }

          // update eNode's pointer
          eNode.iterm = iterm;
        }
      }

      auto nodePtr = nodesMap.find(eNode.getKey());

      // found already pushed Node
      // --> junction element found - see dbWireCode::decode
      if( nodePtr != nodesMap.end() ) {
        // update child idx
        flatPath[nodePtr->second].child = flatPath.size(); 
      }
      else {
        // update idx for the future purpose
        nodesMap[eNode.getKey()] = flatPath.size();
      }

      nodes.push_back(eNode);
      flatPath.push_back(eco::Clip::EcoPath(&nodes[nodes.size()-1]));

      eco::Clip::EcoEdge eEdge;
      eEdge.type = edge->type();

      // SEGMENT
      if( edge->type() == 0 ) {
        debugPrint(logger, ECO, "core_eco", 5, 
            "  EdgeType: Segment");
      }
      // TECH_VIA
      else if( edge->type() == 1 ) {
        odb::dbWireGraph::TechVia* techVia = reinterpret_cast<odb::dbWireGraph::TechVia*>(edge);
        debugPrint(logger, ECO, "core_eco", 5, 
            "  EdgeType: TechVia: {}", 
            techVia->via()->getConstName()); 
        eEdge.techVia = techVia->via();
      }
      // VIA 
      else if (edge->type() == 2) {
        odb::dbWireGraph::Via* via = reinterpret_cast<odb::dbWireGraph::Via*>(edge);
        debugPrint(logger, ECO, "core_eco", 5, 
            "  EdgeType: Via: {}", 
            via->via()->getConstName()); 
        eEdge.via = via->via();
      }
      else if (edge->type() == 3) {
        // nothing to retrieve
        // odb::dbWireGraph::Short* wShort = reinterpret_cast<odb::dbWireGraph::Short*>(edge);
        debugPrint(logger, ECO, "core_eco", 5, 
            "  EdgeType: Short");
      }
      else if (edge->type() == 4) {
        // nothing to retrieve
        // odb::dbWireGraph::VWire* vWire= reinterpret_cast<odb::dbWireGraph::VWire*>(edge);
        debugPrint(logger, ECO, "core_eco", 5, 
            "  EdgeType: VWire");
      }

      edges.push_back(eEdge);
      flatPath.push_back(eco::Clip::EcoPath(&edges[edges.size()-1]));


      if( targetObj && targetObj->getObjName() == string("dbITerm") ) {
        odb::dbITerm* iterm 
          = odb::dbITerm::getITerm(block, targetObj->getId());

        debugPrint(logger, ECO, "core_eco", 5, 
            "  fid: {} target Node: {} {}/{}",
            flatPath.size(), 
            targetObj->getObjName(),
            iterm->getInst()->getConstName(), 
            iterm->getMTerm()->getConstName() );

        nodes.push_back( eco::Clip::EcoNode(x3, y3, edge->target()->layer(), iterm ));
        flatPath.push_back(eco::Clip::EcoPath(&nodes[nodes.size()-1]));

        netPaths.push_back(getPath(flatPath));
      }
    } // end edgeIter

    for(auto& curNetPath : netPaths) {
      // update RNet -- create pin and RNet incrementally 
      retClip.updateRNet(net, rNetMap, curNetPath, eb_vars_);
      // failure happened.
      if( retClip.isFailed() ) {
        return retClip;
      }
    }

    totalEcoPaths.push_back(netPaths);
  }

  retClip.setEcoPaths(totalEcoPaths);
  retClip.postProcessRNets();
  retClip.printRNet(); 

  if( eb_vars_.guiMode ) {
    if( !graphics_ ) {
      graphics_ = std::make_unique<Graphics>(log_, &retClip);
    }

    graphics_->highlightClip(&retClip);
    graphics_->plot(true); 
  }

  return retClip;
}

void
EcoBase::setDrcCoordis(vector<pair<int,int>>& drcCoordis) {
  drc_coordis_ = drcCoordis;
}

void
EcoBase::setEbVars(EcoBaseVars ebVars) {
  eb_vars_ = ebVars;
}

void
EcoBase::writeRects(std::string fileName) {
  logger->info(ECO, 821, "Write Rect: {}", fileName);
  std::ofstream out;
  out.open(fileName);
  if(!out) {
    logger->error(ECO, 721, "Cannot open {}", fileName);
  }

  for(auto& rect : passed_rects_ ){
    out << rect.xMin() << " " << rect.yMin() << " " << rect.xMax() << " " << rect.yMax() << endl;
  }
  out.close();
}

void
EcoBase::readRects(std::string fileName) {
  logger->info(ECO, 822, "Read Rect: {}", fileName);
  passed_rects_.clear();
  passed_rects_.shrink_to_fit();

  std::ifstream inFile(fileName, std::ifstream::in);
  if( !inFile ) {
    logger->error(ECO, 722, "Cannot open {}", fileName);
  }

  int xMin, yMin, xMax, yMax;
  while(inFile>>xMin>>yMin>>xMax>>yMax) {
    passed_rects_.push_back(odb::Rect(xMin, yMin, xMax, yMax));
  }

  logger->info(ECO, 823, "Read Rect: parsed {} rects", passed_rects_.size());
  inFile.close();
}

static OverlapMark 
getWireOverlapInfo(odb::Rect& wireRect, odb::Rect& solveRect) {
  // vertical handling
  if( wireRect.xMin() == wireRect.xMax() ) {
    // outside of rect; e.g. leftmost or rightmost
    if( wireRect.xMin() < solveRect.xMin() ||
        wireRect.xMin() > solveRect.xMax() ) {
      return OverlapMark::OUTSIDE;
    }
    // from here, 
    // wireRect.x is WITHIN solveRect's X 
    int y1 = std::min(wireRect.yMin(), wireRect.yMax());
    int y3 = std::max(wireRect.yMin(), wireRect.yMax());
    
    // outside of rect; e.g., downmost or upmost
    if( y1 > solveRect.yMax() ||
        y3 < solveRect.yMin() ) {
      return OverlapMark::OUTSIDE;
    }

    // inside of rect; 
    if( y3 <= solveRect.yMax() &&
        y1 >= solveRect.yMin() ) {
      return OverlapMark::INSIDE;
    }
    
    // neither of outside/inside cases -- intersect happen
    // down to up case
    if( wireRect.yMin() < wireRect.yMax() ) {
      // go through
      if( wireRect.yMin() <= solveRect.yMin() &&
          wireRect.yMax() >= solveRect.yMax() ) {
        return OverlapMark::DOWNINUPOUT;
      }

      // hits up boundary
      if( wireRect.yMin() <= solveRect.yMax() &&
          solveRect.yMax() < wireRect.yMax() ) {
        return OverlapMark::UPOUT;
      }
      // hits down boundary
      else if( wireRect.yMin() < solveRect.yMin() &&
          solveRect.yMin() <= wireRect.yMax() ) {
        return OverlapMark::DOWNIN;
      }
      return OverlapMark::NONE;
    }
    // up to down case  
    else if( wireRect.yMin() > wireRect.yMax() ) {
      // go through
      if( wireRect.yMax() <= solveRect.yMin() &&
          wireRect.yMin() >= solveRect.yMax() ) {
        return OverlapMark::UPINDOWNOUT;
      }

      // hits up boundary
      if( wireRect.yMax() <= solveRect.yMax() &&
         solveRect.yMax() < wireRect.yMin() ) {
        return OverlapMark::UPIN;
      } 
      // hits down boundary
      else if( wireRect.yMax() < solveRect.yMin() &&
          solveRect.yMin() <= wireRect.yMin() ) {
        return OverlapMark::DOWNOUT;
      }
      return OverlapMark::NONE;
    }
  }

  // horizontal handling
  if( wireRect.yMin() == wireRect.yMax() ) {
    // outside of rect; e.g. leftmost or rightmost
    if( wireRect.yMin() < solveRect.yMin() ||
        wireRect.yMin() > solveRect.yMax() ) {
      return OverlapMark::OUTSIDE;
    }
    // from here, 
    // wireRect.y is WITHIN solveRect's Y 
    
    int x1 = std::min(wireRect.xMin(), wireRect.xMax());
    int x3 = std::max(wireRect.xMin(), wireRect.xMax());
    // outside of rect; e.g., downmost or upmost
    if( x1 > solveRect.xMax() ||
        x3 < solveRect.xMin() ) {
      return OverlapMark::OUTSIDE;
    }

    // inside of rect; 
    if( x3 <= solveRect.xMax() &&
        x1 >= solveRect.xMin() ) {
      return OverlapMark::INSIDE;
    }
    
    // neither of outside/inside cases -- intersect happen
    // left to right case
    if( wireRect.xMin() < wireRect.xMax() ) {
      // go through
      if( wireRect.xMin() <= solveRect.xMin() &&
          wireRect.xMax() >= solveRect.xMax() ) {
        return OverlapMark::LEFTINRIGHTOUT;
      }

      // hits right boundary
      if( wireRect.xMin() <= solveRect.xMax() &&
          solveRect.xMax() < wireRect.xMax() ) {
        return OverlapMark::RIGHTOUT;
      }
      // hits left boundary
      else if( wireRect.xMin() < solveRect.xMin() &&
          solveRect.xMin() <= wireRect.xMax() ) {
        return OverlapMark::LEFTIN;
      }
      return OverlapMark::NONE;
    }
    // right to left case  
    else if( wireRect.xMin() > wireRect.xMax() ) {
      // go through
      if( wireRect.xMax() <= solveRect.xMin() &&
          wireRect.xMin() >= solveRect.xMax() ) {
        return OverlapMark::RIGHTINLEFTOUT;
      }

      // hits right boundary
      if( wireRect.xMax() <= solveRect.xMax() &&
         solveRect.xMax() < wireRect.xMin() ) {
        return OverlapMark::RIGHTIN;
      } 
      // hits left boundary
      else if( wireRect.xMax() < solveRect.xMin() &&
          solveRect.xMin() <= wireRect.xMin() ) {
        return OverlapMark::LEFTOUT;
      }
      return OverlapMark::NONE;
    }
  }

  // not possible
  return OverlapMark::NONE;
} 

static std::string
getString(eco::OverlapMark mark) {
  switch (mark) {
    case eco::OverlapMark::LEFTIN:
      return "LEFT IN";
    case eco::OverlapMark::LEFTOUT:
      return "LEFT OUT";
    case eco::OverlapMark::RIGHTIN:
      return "RIGHT IN";
    case eco::OverlapMark::RIGHTOUT:
      return "RIGHT OUT";
    case eco::OverlapMark::UPIN:
      return "UP IN";
    case eco::OverlapMark::UPOUT:
      return "UP OUT";
    case eco::OverlapMark::DOWNIN:
      return "DOWN IN";
    case eco::OverlapMark::DOWNOUT:
      return "DOWN OUT";
    case eco::OverlapMark::DOWNINUPOUT:
      return "DOWN IN UP OUT";
    case eco::OverlapMark::UPINDOWNOUT:
      return "UP IN DOWN OUT";
    case eco::OverlapMark::LEFTINRIGHTOUT:
      return "LEFT IN RIGHT OUT";
    case eco::OverlapMark::RIGHTINLEFTOUT:
      return "RIGHT IN LEFT OUT";
    case eco::OverlapMark::ZLOWERIN:
      return "ZAXIS LOWER IN";
    case eco::OverlapMark::ZLOWEROUT:
      return "ZAXIS LOWER OUT";
    case eco::OverlapMark::ZUPPERIN:
      return "ZAXIS UPPER IN";
    case eco::OverlapMark::ZUPPEROUT:
      return "ZAXIS UPPER OUT";
    case eco::OverlapMark::INSIDE:
      return "INSIDE";
    case eco::OverlapMark::OUTSIDE:
      return "OUTSIDE";
    case eco::OverlapMark::NONE:
    default:
      return "NONE";
  }
  return "NONE";
}

static std::string
getString(eco::ClipLayerPin::Direction dir) { 
  switch (dir) {
    case eco::ClipLayerPin::Direction::DOWN:
      return "DOWN";
    case eco::ClipLayerPin::Direction::UP:
      return "UP";
    case eco::ClipLayerPin::Direction::SAME:
      return "SAME";
    default: 
      return "NONE";
  }
  return "NONE";
}

static std::string
getString(eco::ClipLayerPin::IO io) { 
  switch (io) {
    case eco::ClipLayerPin::IO::INPUT:
      return "INPUT";
    case eco::ClipLayerPin::IO::OUTPUT:
      return "OUTPUT";
    case eco::ClipLayerPin::IO::IONONE:
      return "IONONE";
    default:
      return "NONE";
  }
  return "NONE";
}

static bool
isWithInBox(odb::dbBox* box, 
    odb::Rect& rect) {
  odb::Rect boxRect = box->getBox();
  return isWithInBox( boxRect, rect );
}

static bool
isWithInBox(odb::Rect& box,
    odb::Rect& bigBox) {
  if( box.xMin() >= bigBox.xMin() &&
     box.yMin() >= bigBox.yMin() &&
     box.xMax() <= bigBox.xMax() &&
     box.yMax() <= bigBox.yMax() ) {
    return true;
  }
  else {
    return false;
  }
}

static bool
isGoInside(eco::OverlapMark mark) {
  switch(mark) {
    // four boundaries
    case eco::OverlapMark::LEFTIN:
    case eco::OverlapMark::RIGHTIN:
    case eco::OverlapMark::UPIN:
    case eco::OverlapMark::DOWNIN:

    // z axis coming
    case eco::OverlapMark::ZLOWERIN:
    case eco::OverlapMark::ZUPPERIN:
    
    // botth go/out cases
    case eco::OverlapMark::DOWNINUPOUT:
    case eco::OverlapMark::UPINDOWNOUT:
    case eco::OverlapMark::LEFTINRIGHTOUT:
    case eco::OverlapMark::RIGHTINLEFTOUT:
      return true;
      break;

    default:
      break;
  }
  return false;
}

static bool
isGoOutside(eco::OverlapMark mark) {
  switch(mark) {
    // four boundaries
    case eco::OverlapMark::LEFTOUT:
    case eco::OverlapMark::RIGHTOUT:
    case eco::OverlapMark::UPOUT:
    case eco::OverlapMark::DOWNOUT:

    // z axis coming
    case eco::OverlapMark::ZLOWEROUT:
    case eco::OverlapMark::ZUPPEROUT:
    
    // botth go/out cases
    case eco::OverlapMark::DOWNINUPOUT:
    case eco::OverlapMark::UPINDOWNOUT:
    case eco::OverlapMark::LEFTINRIGHTOUT:
    case eco::OverlapMark::RIGHTINLEFTOUT:
      return true;
      break;

    default:
      break;
  }
  return false;
}

static bool
is3DIntersect(eco::OverlapMark mark) {
  switch(mark) {
    case eco::OverlapMark::ZLOWERIN:
    case eco::OverlapMark::ZUPPERIN:
    case eco::OverlapMark::ZLOWEROUT:
    case eco::OverlapMark::ZUPPEROUT:
      return true;
      break;
    default:
      break;
  }
  return false;
}

static bool
is2DIntersect(eco::OverlapMark mark) {
  switch(mark) {
    case eco::OverlapMark::LEFTOUT:
    case eco::OverlapMark::RIGHTOUT:
    case eco::OverlapMark::UPOUT:
    case eco::OverlapMark::DOWNOUT:

    case eco::OverlapMark::LEFTIN:
    case eco::OverlapMark::RIGHTIN:
    case eco::OverlapMark::UPIN:
    case eco::OverlapMark::DOWNIN:

    // botth go/out cases
    case eco::OverlapMark::DOWNINUPOUT:
    case eco::OverlapMark::UPINDOWNOUT:
    case eco::OverlapMark::LEFTINRIGHTOUT:
    case eco::OverlapMark::RIGHTINLEFTOUT:
      return true;
      break;
    default:
      break;
  }
  return false;
}

static bool
isOutside(eco::OverlapMark mark) {
  switch(mark) {
    case eco::OverlapMark::OUTSIDE:
      return true;
      break;
    default:
      break;
  }
  return false;
}

static bool
isInside(eco::OverlapMark mark) {
  switch(mark) {
    case eco::OverlapMark::INSIDE:
      return true;
      break;
    default:
      break;
  }
  return false;
}

std::vector<eco::Clip::EcoPath>
Clip::getModifiedPathITIT(std::vector<RoutedPoint>& routedPath,
    std::vector<eco::Clip::EcoNode>& newNodes,
    std::vector<eco::Clip::EcoEdge>& newEdges, 
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap) {

  std::vector<eco::Clip::EcoPath> resultPath;

  // edge traversal
  for(int i=1; i<routedPath.size()-2; i++) {
    RoutedPoint* prevPnt = &routedPath[i];
    RoutedPoint* nextPnt = &routedPath[i+1];

    // for prevClPin 
    odb::dbTechLayer* prevDbLayer = nullptr;
    ClipLayer* prevLayer = nullptr;
    ClipLayerPin* prevClPin = nullptr;

    // for nextClPin
    odb::dbTechLayer* nextDbLayer = nullptr;
    ClipLayer* nextLayer = nullptr;
    ClipLayerPin* nextClPin = nullptr;

    // update prev NODE
    // means ITerm at begin
    if( i == 1 ) {
      // dbuX/Y are the same -- VIA12 segment!
      // fill in nextClPins ptrs 
      nextDbLayer
        = db_->getTech()->findRoutingLayer(nextPnt->m());
      nextLayer = dbToEb(nextDbLayer);
      nextClPin = nextLayer->clipLayerPin(nextPnt->c(), nextPnt->r());

      RoutedPoint* beginPnt = &routedPath[0];

      EcoNode iTermNode(nextClPin->dbuX(), 
          nextClPin->dbuY(), 
          iterm_layer_->layer(), 
          beginPnt->iTerm());

      newNodes.push_back(iTermNode);
    }
    else {
      // fill in prevClPins ptrs 
      prevDbLayer 
        = db_->getTech()->findRoutingLayer(prevPnt->m());

      prevLayer = dbToEb(prevDbLayer);
      prevClPin = prevLayer->clipLayerPin(prevPnt->c(), prevPnt->r());

      newNodes.push_back(EcoNode(prevClPin));
    }

    resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));

    // means NOT ITerm at end 
    if( i != routedPath.size() -3 ) {

      // fill in nextClPins ptrs 
      nextDbLayer
        = db_->getTech()->findRoutingLayer(nextPnt->m());
      nextLayer = dbToEb(nextDbLayer);
      nextClPin = nextLayer->clipLayerPin(nextPnt->c(), nextPnt->r());
    }

    // update EDGE (between prev <-> next)
    // means iterm / TECH VIA 12 edge segment at begin/end point
    if( prevClPin == nullptr || nextClPin == nullptr ) {
      EcoEdge tmpEdge;
      tmpEdge.type = 1;
      auto lvPtr = layerViaMap.find(iterm_layer_->layer());
      if( lvPtr == layerViaMap.end() ) {
        logger->error(ECO, 12374, "cannot find via in layer: {}", iterm_layer_->layer()->getConstName());
      }
      tmpEdge.techVia = lvPtr->second;
      newEdges.push_back(tmpEdge);
    }
    // normal cases
    else {
      newEdges.push_back(EcoEdge(prevClPin, nextClPin, layerViaMap));
    }

    resultPath.push_back(EcoPath(&newEdges[newEdges.size()-1]));

    // handle end point
    if( i == routedPath.size()-3 ) {
      RoutedPoint* endPnt = &routedPath[routedPath.size()-1];

      EcoNode iTermNode(prevClPin->dbuX(), 
          prevClPin->dbuY(), 
          iterm_layer_->layer(), 
          endPnt->iTerm());

      newNodes.push_back(iTermNode);
      resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));
    }
  }

  return resultPath;
}

std::vector<eco::Clip::EcoPath>
Clip::getModifiedPathITEX(std::vector<RoutedPoint>& routedPath,
    std::vector<eco::Clip::EcoNode>& newNodes,
    std::vector<eco::Clip::EcoEdge>& newEdges, 
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
    std::vector<EcoPath>& path,
    int endIdx) {
  std::vector<eco::Clip::EcoPath> resultPath;

  // edge traversal
  for(int i=1; i<routedPath.size()-2; i++) {
    RoutedPoint* prevPnt = &routedPath[i];
    RoutedPoint* nextPnt = &routedPath[i+1];
    
    // for prevClPin 
    odb::dbTechLayer* prevDbLayer = nullptr;
    ClipLayer* prevLayer = nullptr;
    ClipLayerPin* prevClPin = nullptr;

    // for nextClPin
    odb::dbTechLayer* nextDbLayer = nullptr;
    ClipLayer* nextLayer = nullptr;
    ClipLayerPin* nextClPin = nullptr;

    // update prev NODE
    // means ITerm at begin
    if( i == 1 ) {
      // dbuX/Y are the same -- VIA12 segment!
      // fill in nextClPins ptrs 
      nextDbLayer
        = db_->getTech()->findRoutingLayer(nextPnt->m());
      nextLayer = dbToEb(nextDbLayer);
      nextClPin = nextLayer->clipLayerPin(nextPnt->c(), nextPnt->r());

      RoutedPoint* beginPnt = &routedPath[0];

      EcoNode iTermNode(nextClPin->dbuX(), 
          nextClPin->dbuY(), 
          iterm_layer_->layer(), 
          beginPnt->iTerm());

      newNodes.push_back(iTermNode);
    }
    else {
      // fill in prevClPins ptrs 
      prevDbLayer 
        = db_->getTech()->findRoutingLayer(prevPnt->m());

      prevLayer = dbToEb(prevDbLayer);
      prevClPin = prevLayer->clipLayerPin(prevPnt->c(), prevPnt->r());

      newNodes.push_back(EcoNode(prevClPin));
    }

    resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));

    // fill in nextClPins ptrs 
    nextDbLayer
      = db_->getTech()->findRoutingLayer(nextPnt->m());
    nextLayer = dbToEb(nextDbLayer);
    nextClPin = nextLayer->clipLayerPin(nextPnt->c(), nextPnt->r());

    // update EDGE (between prev <-> next)
    // means iterm / TECH VIA 12 edge segment at begin point
    if( prevClPin == nullptr ) {
      EcoEdge tmpEdge;
      tmpEdge.type = 1;
      auto lvPtr = layerViaMap.find(iterm_layer_->layer());
      if( lvPtr == layerViaMap.end() ) {
        logger->error(ECO, 12375, "cannot find via in layer: {}", iterm_layer_->layer()->getConstName());
      }
      tmpEdge.techVia = lvPtr->second;
      newEdges.push_back(tmpEdge);
    }
    // normal cases
    else {
      newEdges.push_back(EcoEdge(prevClPin, nextClPin, layerViaMap));
    }
    
    resultPath.push_back(EcoPath(&newEdges[newEdges.size()-1]));

    // handle end point
    if( i == routedPath.size()-3 ) {
      newNodes.push_back(EcoNode(nextClPin));
      resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));
    }
  }

  // now reached to end.
  // concatenate the EXTPIN
  debugPrint(logger, ECO, "core_eco", 5, "     case2: current stitching path: {}",
      getDebugString(path[endIdx]));

  // path modification handling
  int beginIdx = endIdx;
  eco::OverlapMark mark = path[endIdx].edge->coreMark;

  std::tuple<int, int, int> t1, t2;

  switch(mark) {
    case OverlapMark::LEFTOUT:
    case OverlapMark::RIGHTOUT:
    case OverlapMark::UPOUT:
    case OverlapMark::DOWNOUT:
      beginIdx = endIdx;
      break;
    case OverlapMark::ZLOWEROUT:
    case OverlapMark::ZUPPEROUT:
      cout << "tuple: " << endl;
      t1 = path[endIdx+1].node->getKey();
      t2 = resultPath[resultPath.size()-1].node->getKey();
      cout << std::get<0>(t1) << " " << std::get<1>(t1) << " " << std::get<2>(t1) << endl;
      cout << std::get<0>(t2) << " " << std::get<1>(t2) << " " << std::get<2>(t2) << endl;
      cout << "same? : " << (t1 == t2) << endl;
      // rearPathBeginIdx = endIdx;
      // outlier handling:
      if( t1 == t2 ) {
        beginIdx = endIdx+2;
      }
      else {
        beginIdx = endIdx;
      }
      break;

    default:
      break;
  }

  // add remaining part
  for(int i=beginIdx; i<path.size(); i++) {
    // same as before
    resultPath.push_back(path[i]);
  }

  return resultPath;
}

std::vector<eco::Clip::EcoPath>
Clip::getModifiedPathEXIT(std::vector<RoutedPoint>& routedPath,
    std::vector<eco::Clip::EcoNode>& newNodes,
    std::vector<eco::Clip::EcoEdge>& newEdges, 
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
    std::vector<EcoPath>& path,
    int beginIdx) {
  std::vector<eco::Clip::EcoPath> resultPath;

  // path modification handling
  // -- it turned out that EX-IT case can jush push up to beginIdx! (differ from IT-EX case)
  //
  // push all prev route up to begin idx
  for(int i=0; i<=beginIdx; i++) {
    resultPath.push_back(path[i]);
  } 

  debugPrint(logger, ECO, "core_eco", 5, "     case3: current stitching path: {}",
      getDebugString(path[beginIdx]));

  // now reached to end.
  // concatenate the EXTPIN
  // edge traversal
  for(int i=1; i<routedPath.size()-2; i++) {
    RoutedPoint* prevPnt = &routedPath[i];
    RoutedPoint* nextPnt = &routedPath[i+1];
    
    // for prevClPin 
    odb::dbTechLayer* prevDbLayer = nullptr;
    ClipLayer* prevLayer = nullptr;
    ClipLayerPin* prevClPin = nullptr;

    // for nextClPin
    odb::dbTechLayer* nextDbLayer = nullptr;
    ClipLayer* nextLayer = nullptr;
    ClipLayerPin* nextClPin = nullptr;

    // update prev NODE
    // means EXTPIN at begin
    // fill in prevClPins ptrs 
    prevDbLayer 
      = db_->getTech()->findRoutingLayer(prevPnt->m());

    prevLayer = dbToEb(prevDbLayer);
    prevClPin = prevLayer->clipLayerPin(prevPnt->c(), prevPnt->r());

    newNodes.push_back(EcoNode(prevClPin));

    resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));


    // means NOT ITerm at end 
    if( i != routedPath.size() -3 ) {
      // fill in nextClPins ptrs 
      nextDbLayer
        = db_->getTech()->findRoutingLayer(nextPnt->m());
      nextLayer = dbToEb(nextDbLayer);
      nextClPin = nextLayer->clipLayerPin(nextPnt->c(), nextPnt->r());
    }

    // update EDGE (between prev <-> next)
    // means iterm / TECH VIA 12 edge segment at end point
    if( nextClPin == nullptr ) {
      EcoEdge tmpEdge;
      tmpEdge.type = 1;
      auto lvPtr = layerViaMap.find(iterm_layer_->layer());
      if( lvPtr == layerViaMap.end() ) {
        logger->error(ECO, 12373, "cannot find via in layer: {}", iterm_layer_->layer()->getConstName());
      }
      tmpEdge.techVia = lvPtr->second;
      newEdges.push_back(tmpEdge);
    }
    // normal cases
    else if (prevClPin != nullptr) {
      newEdges.push_back(EcoEdge(prevClPin, nextClPin, layerViaMap));
    }
    
    resultPath.push_back(EcoPath(&newEdges[newEdges.size()-1]));

    // handle end point
    if( i == routedPath.size()-3 ) {
      RoutedPoint* endPnt = &routedPath[routedPath.size()-1];

      EcoNode iTermNode(prevClPin->dbuX(), 
          prevClPin->dbuY(), 
          iterm_layer_->layer(), 
          endPnt->iTerm());

      newNodes.push_back(iTermNode);
      resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));
    }
  }

  return resultPath;
}

std::vector<eco::Clip::EcoPath>
Clip::getModifiedPathEXEX(std::vector<RoutedPoint>& routedPath,
    std::vector<eco::Clip::EcoNode>& newNodes,
    std::vector<eco::Clip::EcoEdge>& newEdges, 
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
    std::vector<EcoPath>& path,
    int beginIdx, int endIdx) {

  std::vector<eco::Clip::EcoPath> resultPath;

  // path modification handling
  // -- it turned out that EX-IT case can jush push up to beginIdx! (differ from IT-EX case)
  //
  // push all prev route up to begin idx
  for(int i=0; i<=beginIdx; i++) {
    resultPath.push_back(path[i]);
  } 

  debugPrint(logger, ECO, "core_eco", 5, "     case4: current stitching path (begin): {}",
      getDebugString(path[beginIdx]));

  debugPrint(logger, ECO, "core_eco", 5, "     case4: current stitching path (end): {}",
      getDebugString(path[endIdx]));

  // now reached to end.
  // concatenate the EXTPIN
  // edge traversal
  for(int i=1; i<routedPath.size()-2; i++) {
    RoutedPoint* prevPnt = &routedPath[i];
    RoutedPoint* nextPnt = &routedPath[i+1];
    
    // for prevClPin 
    odb::dbTechLayer* prevDbLayer = nullptr;
    ClipLayer* prevLayer = nullptr;
    ClipLayerPin* prevClPin = nullptr;

    // for nextClPin
    odb::dbTechLayer* nextDbLayer = nullptr;
    ClipLayer* nextLayer = nullptr;
    ClipLayerPin* nextClPin = nullptr;

    // update prev NODE
    // means EXTPIN at begin
    // fill in prevClPins ptrs 
    prevDbLayer 
      = db_->getTech()->findRoutingLayer(prevPnt->m());

    prevLayer = dbToEb(prevDbLayer);
    prevClPin = prevLayer->clipLayerPin(prevPnt->c(), prevPnt->r());

    newNodes.push_back(EcoNode(prevClPin));

    resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));

    // fill in nextClPins ptrs 
    nextDbLayer
      = db_->getTech()->findRoutingLayer(nextPnt->m());
    nextLayer = dbToEb(nextDbLayer);
    nextClPin = nextLayer->clipLayerPin(nextPnt->c(), nextPnt->r());

    newEdges.push_back(EcoEdge(prevClPin, nextClPin, layerViaMap));
    resultPath.push_back(EcoPath(&newEdges[newEdges.size()-1]));

    // handle end point
    if( i == routedPath.size()-3 ) {
      newNodes.push_back(EcoNode(nextClPin));
      resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));
    }
  }


  // path modification handling
  int rearPathBeginIdx = endIdx;
  eco::OverlapMark mark = path[endIdx].edge->coreMark;

  std::tuple<int, int, int> t1, t2;

  switch(mark) {
    case OverlapMark::LEFTOUT:
    case OverlapMark::RIGHTOUT:
    case OverlapMark::UPOUT:
    case OverlapMark::DOWNOUT:
      rearPathBeginIdx = endIdx;
      break;
    case OverlapMark::ZLOWEROUT:
    case OverlapMark::ZUPPEROUT:
      cout << "tuple: " << endl;
      t1 = path[endIdx+1].node->getKey();
      t2 = resultPath[resultPath.size()-1].node->getKey();
      cout << std::get<0>(t1) << " " << std::get<1>(t1) << " " << std::get<2>(t1) << endl;
      cout << std::get<0>(t2) << " " << std::get<1>(t2) << " " << std::get<2>(t2) << endl;
      cout << "same? : " << (t1 == t2) << endl;
      // rearPathBeginIdx = endIdx;
      // outlier handling:
      if( t1 == t2 ) {
        rearPathBeginIdx = endIdx+2;
      }
      else {
        rearPathBeginIdx = endIdx;
      }
      break;

    default:
      break;
  }

  cout << "current pushed to resultPath" << endl;
  for(int i=0; i<resultPath.size(); i++) {
    cout << i << " " << getDebugString(resultPath[i]) << endl;
  }

  cout << "current rearPathBeginIdx: " << rearPathBeginIdx << endl;
  cout << "WHOLE path from path" << endl;
  for(int i=0; i<path.size(); i++) {
    cout << i << " " << getDebugString(path[i]) << endl;
  }

  // add remaining part
  for(int i=rearPathBeginIdx; i<path.size(); i++) {
    // same as before
    resultPath.push_back(path[i]);
  }

  return resultPath;
}


// The DELTA between case4 EXEX and this function is "starting idx is 2"
std::vector<eco::Clip::EcoPath>
Clip::getModifiedPathMiddleITEX(std::vector<RoutedPoint>& routedPath,
    std::vector<eco::Clip::EcoNode>& newNodes,
    std::vector<eco::Clip::EcoEdge>& newEdges, 
    std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
    std::vector<EcoPath>& path,
    int beginIdx, int endIdx) {

  std::vector<eco::Clip::EcoPath> resultPath;

  // path modification handling
  // -- it turned out that EX-IT case can jush push up to beginIdx! (differ from IT-EX case)
  //
  // push all prev route up to begin idx
  for(int i=0; i<=beginIdx; i++) {
    resultPath.push_back(path[i]);
  } 

  debugPrint(logger, ECO, "core_eco", 5, "     case5: current stitching path (begin): {}",
      getDebugString(path[beginIdx]));

  debugPrint(logger, ECO, "core_eco", 5, "     case5: current stitching path (end): {}",
      getDebugString(path[endIdx]));

  // now reached to end.
  // concatenate the EXTPIN
  // edge traversal
  //
  // Starting POINT is 2!! (DELTA)
  for(int i=2; i<routedPath.size()-2; i++) {
    RoutedPoint* prevPnt = &routedPath[i];
    RoutedPoint* nextPnt = &routedPath[i+1];
    
    // for prevClPin 
    odb::dbTechLayer* prevDbLayer = nullptr;
    ClipLayer* prevLayer = nullptr;
    ClipLayerPin* prevClPin = nullptr;

    // for nextClPin
    odb::dbTechLayer* nextDbLayer = nullptr;
    ClipLayer* nextLayer = nullptr;
    ClipLayerPin* nextClPin = nullptr;

    // update prev NODE
    // means EXTPIN at begin
    // fill in prevClPins ptrs 
    prevDbLayer 
      = db_->getTech()->findRoutingLayer(prevPnt->m());

    prevLayer = dbToEb(prevDbLayer);
    prevClPin = prevLayer->clipLayerPin(prevPnt->c(), prevPnt->r());

    newNodes.push_back(EcoNode(prevClPin));

    resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));

    // fill in nextClPins ptrs 
    nextDbLayer
      = db_->getTech()->findRoutingLayer(nextPnt->m());
    nextLayer = dbToEb(nextDbLayer);
    nextClPin = nextLayer->clipLayerPin(nextPnt->c(), nextPnt->r());

    newEdges.push_back(EcoEdge(prevClPin, nextClPin, layerViaMap));
    resultPath.push_back(EcoPath(&newEdges[newEdges.size()-1]));

    // handle end point
    if( i == routedPath.size()-3 ) {
      newNodes.push_back(EcoNode(nextClPin));
      resultPath.push_back(EcoPath(&newNodes[newNodes.size()-1]));
    }
  }


  // path modification handling
  int rearPathBeginIdx = endIdx;
  eco::OverlapMark mark = path[endIdx].edge->coreMark;

  std::tuple<int, int, int> t1, t2;

  switch(mark) {
    case OverlapMark::LEFTOUT:
    case OverlapMark::RIGHTOUT:
    case OverlapMark::UPOUT:
    case OverlapMark::DOWNOUT:
      rearPathBeginIdx = endIdx;
      break;
    case OverlapMark::ZLOWEROUT:
    case OverlapMark::ZUPPEROUT:
      t1 = path[endIdx+1].node->getKey();
      t2 = resultPath[resultPath.size()-1].node->getKey();
      // rearPathBeginIdx = endIdx;
      // outlier handling:
      if( t1 == t2 ) {
        rearPathBeginIdx = endIdx+2;
      }
      else {
        rearPathBeginIdx = endIdx;
      }
      break;

    default:
      break;
  }

  cout << "current pushed to resultPath" << endl;
  for(int i=0; i<resultPath.size(); i++) {
    cout << i << " " << getDebugString(resultPath[i]) << endl;
  }

  cout << "current rearPathBeginIdx: " << rearPathBeginIdx << endl;
  cout << "WHOLE path from path" << endl;
  for(int i=0; i<path.size(); i++) {
    cout << i << " " << getDebugString(path[i]) << endl;
  }

  // add remaining part
  for(int i=rearPathBeginIdx; i<path.size(); i++) {
    // same as before
    resultPath.push_back(path[i]);
  }

  return resultPath;
}

}
