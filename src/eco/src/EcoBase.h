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

#ifndef __ECO_BASE_HEADER__
#define __ECO_BASE_HEADER__

#include <memory>
#include <vector>
#include "odb/geom.h"
#include "odb/dbTypes.h"
#include "gui/gui.h"

#include "odb/dbWireCodec.h"

namespace odb {
  class dbDatabase;
  class dbITerm;
  class dbBTerm;
  class dbMTerm;
  class dbTechLayer;
  class dbNet;
}

namespace sta {
  class dbSta;
}

namespace utl {
  class Logger;
}

namespace eco {

class ClipLayerPin;
struct PairHash;

class RoutedPoint
{
  public:
    int m() const;
    int r() const;
    int c() const; 
    bool isExternal() const;
    odb::dbITerm* iTerm() const;
    RoutedPoint(std::string input, odb::dbBlock* block);
    std::string string() const;

  private:
    int m_, r_, c_; 
    bool is_external_;
    odb::dbITerm* iterm_;
};

inline int
RoutedPoint::m() const {
  return m_;
}

inline int
RoutedPoint::r() const {
  return r_;
}

inline int
RoutedPoint::c() const {
  return c_;
}

inline odb::dbITerm*
RoutedPoint::iTerm() const {
  return iterm_;
}

inline bool
RoutedPoint::isExternal() const {
  return is_external_;
}

// redefine Net
class RNet 
{ 
  public:
    RNet(odb::dbNet* net);
    odb::dbNet* net() const;
    std::vector<ClipLayerPin*>& pins();
    std::vector<ClipLayerPin*>& iPins();
    std::vector<ClipLayerPin*>& oPins();

    std::string getName() const;

    // same net idx (for the same odb::Net*)
    void setIdx(int netIdx);

    // nets idx (for nets_ vector)
    void setNetsIdx(int netsIdx);
    int netsIdx() const; 

    void addPin(ClipLayerPin* clPin);
    bool isIPinExist(eco::ClipLayerPin* clPin) const;
    void print();

    void clearPins();

    // commodity index build
    void buildCommodityIndices();
  
    int getCommodityCnt() const;
    ClipLayerPin* commodityOutPin(int idx);
    ClipLayerPin* commodityInPin(int idx);

    void addRoutedPoints(std::vector<RoutedPoint> routedPoints);
    std::vector<std::vector<RoutedPoint>>& routedPaths();

    // update steiner points
    void addRoutedSteinerPoint(RoutedPoint point);
    std::vector<RoutedPoint>& steinerPoints();

    void setSingleCommNet(bool mode);
    bool isSingleCommNet() const { return is_single_comm_net_; };
    bool isFeasible(int from, int to);
    void setSpecialHandling(bool mode) { need_special_handling_ = mode; }
    bool isSpecialHandling() const { return need_special_handling_; } 

  private:
    odb::dbNet* net_;
    int net_idx_, nets_idx_;
    std::vector<eco::ClipLayerPin*> pins_, ipins_, opins_;
    std::set<eco::ClipLayerPin*> ipins_map_;

    // commodity-index -> outpin / inpin
    std::vector<ClipLayerPin*> c_opins_;
    std::vector<ClipLayerPin*> c_ipins_;

    // commodity-index -> vector<string> strings
    std::vector<std::vector<RoutedPoint>> routed_paths_;
    std::vector<RoutedPoint> steiner_points_;

    bool is_single_comm_net_;
    bool need_special_handling_;
};

inline int
RNet::netsIdx() const {
  return nets_idx_;
}

inline odb::dbNet*
RNet::net() const {
  return net_;
}

inline std::vector<eco::ClipLayerPin*>&
RNet::pins() {
  return pins_;
}

inline std::vector<eco::ClipLayerPin*>&
RNet::iPins() {
  return ipins_;
}

inline std::vector<eco::ClipLayerPin*>&
RNet::oPins() {
  return opins_;
}

inline std::vector<std::vector<RoutedPoint>>& 
RNet::routedPaths() {
  return routed_paths_;
}

inline std::vector<RoutedPoint>& 
RNet::steinerPoints() {
  return steiner_points_;
}

class ClipLayerPin 
{
  public:
    // EXT: cut by boundary
    // nonEXT: within boundary
    // OBS: obstacle (already consumed)
    // EMPTY: empty
    enum ClipLayerPinType {
      EXTPIN,
      ITERM,
      BTERM,
      OBS, 
      EMPTY
    };

    enum IO {
      INPUT, 
      OUTPUT,
      IONONE
    };

    enum Direction {
      DOWN,
      SAME,
      UP
    };

    ClipLayerPin(ClipLayerPinType type, 
        int x, int y, 
        int dbuX, int dbuY,
        odb::dbTechLayer* layer);

    IO io() const;
    ClipLayerPinType type() const;
    Direction dir() const;
  
    // for EXTITERM, ITERM
    odb::dbITerm* iTerm() const;

    // for EXTBTERM, BTERM
    odb::dbBTerm* bTerm() const;

    // layer info
    odb::dbTechLayer* layer() const;

    void setIo(IO io);
    void setIo(odb::dbIoType io);
    void setType(ClipLayerPinType type);
    void setITerm(odb::dbITerm* iTerm);
    void setBTerm(odb::dbBTerm* bTerm);
    void setDbLayer(odb::dbTechLayer* layer);
    void setIsFixed(bool isFixed);
    void setDir(Direction dir);

    void setIsObsArea(bool isObsArea);
    void setHasObsLeft(bool hasObs);
    void setHasObsRight(bool hasObs);
    void setHasObsUp(bool hasObs);
    void setHasObsDown(bool hasObs);
    void setHasObsZUp(bool hasObs);
    void setHasObsZDown(bool hasObs);

    // grid coordinate
    int x() const; 
    int y() const;
    
    // dbu coordinate
    int dbuX() const; 
    int dbuY() const;

    // key for hash map
    int getKey() const;

    // for OBS
    odb::dbInst* inst() const;

    bool isFixed() const;

    // for OBS awareness
    bool isObsArea() const;

    bool hasObs() const;
    bool hasObsLeft() const { return (has_obs_left_ == 1); }
    bool hasObsRight() const { return (has_obs_right_ == 1); }
    bool hasObsUp() const { return (has_obs_up_ == 1); }
    bool hasObsDown() const { return (has_obs_down_ == 1); }
    bool hasObsZUp() const { return (has_obs_z_up_ == 1); } 
    bool hasObsZDown() const { return (has_obs_z_down_ == 1); }
    bool isSameClPin(const ClipLayerPin* clPin);

    std::string getObsString() const;

    std::string getVertName() const;
    std::string getMrc() const;

    std::vector<std::pair<int, int>>& accessPointPairs();

  private:
    ClipLayerPinType type_;
    IO io_;
    odb::dbITerm* iterm_;
    odb::dbBTerm* bterm_;
    odb::dbTechLayer* layer_;
    ClipLayerPin::Direction dir_;
    int x_, y_;
    int dbu_x_, dbu_y_;
    unsigned char is_fixed_:1;
    unsigned char is_obs_area_:1;
    unsigned char has_obs_left_:1;
    unsigned char has_obs_right_:1;
    unsigned char has_obs_up_:1;
    unsigned char has_obs_down_:1;
    unsigned char has_obs_z_up_:1;
    unsigned char has_obs_z_down_:1;

    // accesspoint pairs
    std::vector<std::pair<int, int>> ap_pairs_;
};

inline ClipLayerPin::ClipLayerPinType
ClipLayerPin::type() const {
  return type_;
}

inline ClipLayerPin::IO
ClipLayerPin::io() const {
  return io_;
}

inline ClipLayerPin::Direction
ClipLayerPin::dir() const {
  return dir_;
}

inline int 
ClipLayerPin::x() const {
  return x_;
}

inline int 
ClipLayerPin::y() const {
  return y_;
}

inline int 
ClipLayerPin::dbuX() const {
  return dbu_x_;
}

inline int 
ClipLayerPin::dbuY() const {
  return dbu_y_;
}

inline odb::dbTechLayer*
ClipLayerPin::layer() const {
  return layer_; 
}

inline bool
ClipLayerPin::isFixed() const {
  return (is_fixed_ == 1);
}

inline bool
ClipLayerPin::isObsArea() const {
  return (is_obs_area_ == 1);
}

inline odb::dbITerm* 
ClipLayerPin::iTerm() const {
  return iterm_;
}

inline std::vector<std::pair<int, int>>&
ClipLayerPin::accessPointPairs() {
  return ap_pairs_;
}

class ClipLayer 
{
  public:
    ClipLayer(
        odb::dbDatabase* db,
        odb::Rect die,
        odb::Rect core,
        odb::dbTechLayer* layer, 
        odb::dbTechLayer* nextLayer);

    // will have grids on layer's pitch and nextlayer's pitch
    odb::dbTechLayer* layer() const;
    odb::dbTechLayer* nextLayer() const;

    odb::Rect gridCoreIdx() const;

    std::vector<std::vector<ClipLayerPin>>& grid();

    std::vector<int>& gridX();
    std::vector<int>& gridY();

    // idx query
    ClipLayerPin* clipLayerPin(int x, int y, 
        ClipLayerPin::ClipLayerPinType type 
        = ClipLayerPin::ClipLayerPinType::EMPTY);

    // dbu query
    ClipLayerPin* clipLayerPinDbu(int x, int y, bool allowNearest = true);

    std::vector<ClipLayerPin>& backupClipLayerPinStor();
    std::vector<ClipLayerPin*>& backupClipLayerPins();

    void addBackupClipLayerPin(ClipLayerPin clPin);

    // retrieve as 1D vector
    std::vector<ClipLayerPin*> allClipLayerPins();

    bool isFailed() { return is_failed_; };

  private:
    odb::dbDatabase* db_;
    odb::dbTechLayer *layer_, *next_layer_;
    odb::Rect die_, core_, grid_core_idx_;
    std::vector<int> grid_x_, grid_y_;

    // coordinate to index in grid_x_, grid_y_.
    std::map<int, int> idx_x_map_, idx_y_map_;
    std::vector<std::vector<ClipLayerPin>> grid_;

    // used for corner cases 
    // where required pins vs actual pins are on the same point
    // --> need to create and save pins to backup_clpins_;
    //
    // This usually not happened, so reserve this vector as "100"
    // and use pointer
    std::vector<ClipLayerPin> backup_clpins_stor_;
    std::vector<ClipLayerPin*> backup_clpins_;

    void init();
    bool is_failed_;
};

inline odb::dbTechLayer* 
ClipLayer::layer() const {
  return layer_;
}

inline odb::dbTechLayer*
ClipLayer::nextLayer() const {
  return next_layer_;
}

inline odb::Rect
ClipLayer::gridCoreIdx() const {
  return grid_core_idx_;
}

inline std::vector<std::vector<ClipLayerPin>>& 
ClipLayer::grid() {
  return grid_;
}

inline std::vector<int>& 
ClipLayer::gridX() {
  return grid_x_;
}

inline std::vector<int>& 
ClipLayer::gridY() {
  return grid_y_;
}

inline std::vector<ClipLayerPin>& 
ClipLayer::backupClipLayerPinStor() {
  return backup_clpins_stor_;
}

inline std::vector<ClipLayerPin*>& 
ClipLayer::backupClipLayerPins() {
  return backup_clpins_;
}

enum OverlapMark {
  LEFTIN, LEFTOUT, 
  RIGHTIN, RIGHTOUT, 
  UPIN, UPOUT, 
  DOWNIN, DOWNOUT, 
  UPINDOWNOUT,
  DOWNINUPOUT,
  LEFTINRIGHTOUT,
  RIGHTINLEFTOUT,
  ZLOWERIN, ZLOWEROUT, 
  ZUPPERIN, ZUPPEROUT,
  INSIDE, OUTSIDE, NONE
};

class DpSite 
{
  public: 
    DpSite(odb::dbInst* inst, 
        bool isFixed,
        odb::dbOrientType orient,  
        int gLx, int gLy,
        int dbuLx, int dbuLy);
    bool isFixed() const;
    bool isEmpty() const;
    bool isPlace() const;

    int gLx() const;
    int gLy() const;

    // real coordinate
    int dbuLx() const;
    int dbuLy() const;

    void setInst(odb::dbInst* inst);
    void setFixed(bool isFixed);
    odb::dbOrientType orient() const;

    std::string printType() const;
    std::string printOrient() const;

  private:
    // odbInst ptr
    odb::dbInst* inst_;

    // mark if this site has fixed inst
    bool is_fixed_;

    // row direction
    odb::dbOrientType orient_;

    // grid (lx, ly)
    // its obvious that gux_ = glx_+1, guy_ = gly_+1
    int glx_, gly_;

    // its obvious that 
    // dbuux_ = dbulx_ + site->getWidth()
    // dbuuy_ = dbuly_ + site->getHeight()
    int dbulx_, dbuly_;
};

inline bool
DpSite::isFixed() const {
  return is_fixed_;
}

inline bool
DpSite::isEmpty() const {
  return (inst_) == nullptr;
}

inline int
DpSite::gLx() const {
  return glx_;
}

inline int
DpSite::gLy() const {
  return gly_;
}

inline int 
DpSite::dbuLx() const {
  return dbulx_;
}

inline int
DpSite::dbuLy() const {
  return dbuly_;
}

inline odb::dbOrientType
DpSite::orient() const {
  return orient_;
}

class Pin;

class PinCoordi
{
  public:
    PinCoordi(int lx, int ly, int ux, int uy);
    ~PinCoordi();

    // stores grid coordinates
    int lx() const;
    int ly() const;
    int ux() const;
    int uy() const;

    bool isLineSegment() const;
    bool isHorizontal() const;
    bool isVertical() const;
    int length() const;

    void setPin(eco::Pin* pin);
    void setDbTechLayer(odb::dbTechLayer* layer);
    eco::Pin* pin() const;
    odb::dbTechLayer* dbTechLayer() const;

  private:
    int lx_, ly_, ux_, uy_;
    eco::Pin* pin_;
    odb::dbTechLayer* db_tech_layer_;
};

inline int
PinCoordi::lx() const {
  return lx_;
}

inline int
PinCoordi::ly() const {
  return ly_;
}

inline int
PinCoordi::ux() const {
  return ux_;
}

inline int
PinCoordi::uy() const {
  return uy_;
}

inline eco::Pin*
PinCoordi::pin() const {
  return pin_;
}

inline odb::dbTechLayer*
PinCoordi::dbTechLayer() const {
  return db_tech_layer_;
}

class Pin
{
  public:
    Pin(odb::dbMTerm* mTerm);
    ~Pin();

    // grid idx
    // ideal: single element
    std::vector<eco::PinCoordi> & pinCoordis();
    void addPinCoordi(eco::PinCoordi& coordi);

    bool isObs() const;
    odb::dbMTerm* mTerm() const;

  private:
    std::vector<eco::PinCoordi> pin_coordis_;
    odb::dbMTerm* db_mterm_;

    void reset();
};

inline std::vector<eco::PinCoordi> &
Pin::pinCoordis() {
  return pin_coordis_;
}

inline bool
Pin::isObs() const {
  return (db_mterm_ == nullptr);
}

inline odb::dbMTerm*
Pin::mTerm() const {
  return db_mterm_;
}

class Master
{
  public:
    Master(odb::dbMaster* master);
    ~Master();

    int width();
    int height();

    void addPin(eco::Pin& pin);
    std::vector<eco::Pin> & pins();
    odb::dbMaster* dbMaster() const;

  private:
    odb::dbMaster* master_;
    std::vector<eco::Pin> pins_;

    void reset();
};

inline std::vector<eco::Pin> &
Master::pins() {
  return pins_;
}

inline odb::dbMaster*
Master::dbMaster() const {
  return master_;
}

class SMT2ClipLayerPin
{
  public:
    SMT2ClipLayerPin(RNet* rnet, int cIdx, ClipLayerPin* clPin);
    RNet* rNet() const;
    int commodityIndex() const;
    int64_t getKey() const;
    ClipLayerPin* clipLayerPin() const;

  private:
    RNet* rnet_;
    int commodity_index_;
    ClipLayerPin* clip_layer_pin_;
};

inline RNet* 
SMT2ClipLayerPin::rNet() const {
  return rnet_;
}

inline int
SMT2ClipLayerPin::commodityIndex() const {
  return commodity_index_;
}

inline ClipLayerPin*
SMT2ClipLayerPin::clipLayerPin() const {
  return clip_layer_pin_;
}

inline int64_t
SMT2ClipLayerPin::getKey() const {
  return static_cast<int64_t>(rnet_->netsIdx()) << 32 
    | static_cast<int64_t>(commodity_index_);
}

class SMT2Segment {
  public:  
    enum SegmentOrder {
      M1AP,
      VIA12,
      M2,
      VIA23,
      M3,
      VIA34,
      M4,
      VIA45,
      M5,
      VIA56,
      M6,
      VIA67,
      M7,
      VIA78,
      M8,
      VIA89,
      M9,
      VIA910,
      M10
    };

    static const int maxSegmentOrder = 19;

    // for metal layer
    SMT2Segment(int metalLayer, std::string inputStr);

    // for via layer 
    SMT2Segment(int lowerLayer, int upperLayer, std::string inputStr);

    SegmentOrder order() const {return order_;}
    int intOrder() const { return static_cast<int>(order_); }
    std::string str() const {return str_;}

  private:
    SegmentOrder order_;
    std::string str_;
}; 

class Clip;
class CoordiInfo
{
  public: 
    CoordiInfo(int layer, int y, int x, Clip* clip);
    void setClip(Clip* clip);
    int layer() const;
    int y() const;
    int x() const;
    int32_t getKey() const;

    void addSMT2ClipLayerPin(SMT2ClipLayerPin smtClPin);
    std::vector<SMT2ClipLayerPin>& smt2Pins();

    // retrieve SMT2ClPin based on RNet and cIdx (filtering)
    std::vector<SMT2ClipLayerPin*>
      getSMT2ClipLayerPins(RNet* rNet, int cIdx);

    // retrieve SMT2ClPin based on RNet (all cIdx) (filtering)
    std::vector<SMT2ClipLayerPin*>
      getSMT2ClipLayerPins(RNet* rNet);

    // retrieve SMT2ClPin based on RNet (all cIdx) (filtering),
    // but group by "ClPin" for SMT2 writing
    std::map<ClipLayerPin*, std::vector<SMT2ClipLayerPin*>>
      getSMT2ClipLayerPinsMap(RNet* rNet);

    // note that allAdjSegments are vaild for all rNet and cIdx!
    std::vector<std::pair<ClipLayerPin*, ClipLayerPin*>>&
      allAdjSegments();

    // return all string segments with given rNet and cIdx variable
    // uses filtered smt2Pins by (rNet, cIdx)
    // + allAdjSegments with (rNet, cIdx) string
    //
    // returns {fromLoc}_{toLoc} | {loc}_{endPoints} | {loc}_{accessPoints}
    std::vector<SMT2Segment>
      getAllStringsCIdx(RNet* rNet, int cIdx);

    std::vector<SMT2Segment>
      getAllStrings(RNet* rNet, bool isViaEnclosure = false);

    std::vector<SMT2Segment>
      getSMT2StringsCIdx(RNet* rNet, int cIdx);

    std::vector<SMT2Segment>
      getSMT2Strings(RNet* rNet, bool isViaEnclosure = false);

    std::vector<SMT2Segment>
      getAdjStrings();

  private:
    int layer_;
    int y_, x_;
    Clip* clip_;
    std::vector<SMT2ClipLayerPin> smt2_pins_;
    std::vector<std::pair<ClipLayerPin*, ClipLayerPin*>> 
      all_adj_segments_;
};

inline int
CoordiInfo::layer() const {
  return layer_;
}

inline int
CoordiInfo::y() const {
  return y_;
}

inline int
CoordiInfo::x() const {
  return x_;
}

inline int32_t
CoordiInfo::getKey() const {
  return static_cast<int32_t>(layer_) << 16 
    | static_cast<int32_t>(y_) << 8 
    | static_cast<int32_t>(x_);
}

inline std::vector<SMT2ClipLayerPin>& 
CoordiInfo::smt2Pins() {
  return smt2_pins_;
}

inline std::vector<std::pair<ClipLayerPin*, ClipLayerPin*>>&
CoordiInfo::allAdjSegments() {
  return all_adj_segments_;
}

class EcoBaseVars;
class EcoOdbNet;
class Clip
{
  public:
    // internal node/edge to update dbWireGraph
    struct EcoNode {
      int x, y;
      odb::dbTechLayer* layer;
      odb::dbITerm* iterm;
      std::tuple<int, int, int> getKey();
      EcoNode();
      EcoNode(int x, int y, odb::dbTechLayer* layer);
      EcoNode(int x, int y, odb::dbTechLayer* layer, 
          odb::dbITerm* iterm);
      EcoNode(ClipLayerPin* clPin);
    };

    struct EcoEdge {
      int type;
      odb::dbTechVia* techVia;
      odb::dbVia* via;
      eco::OverlapMark coreMark;
      eco::OverlapMark dieMark;
      EcoEdge();
      EcoEdge(ClipLayerPin* prevClPin,
          ClipLayerPin* nextClPin,
          std::map<odb::dbTechLayer*, odb::dbTechVia*>& viaMap);
    };

    struct EcoPath {
      EcoNode* node;
      EcoEdge* edge;
      int child;
      std::vector<eco::ClipLayerPin*> clPins;
      EcoPath();
      EcoPath(EcoNode* node);
      EcoPath(EcoEdge* edge);
    };

    Clip();
    Clip(odb::dbDatabase* db,
        EcoBaseVars ebVars,
        odb::Rect die, odb::Rect core,
        std::vector<odb::dbInst*>& instList,
        std::vector<odb::dbInst*>& movableInstList,
        std::vector<odb::dbInst*>& fixedInstList,
        std::vector<odb::dbNet*>& netList);

    void reset();


    std::vector<eco::ClipLayer*> clipLayers();

    odb::Rect die() const;
    odb::Rect core() const;
    bool isFailed() const;
    void setIsFailed(bool isFailed);

    // 1D: nets index
    // -> 2D list of endpoint path (source -> sink) 
    //
    // e.g., 
    // list1: source -> seg1-> via1 -> seg2 -> sink1
    // list2: source -> ... -> sink2
    // list3: source -> ... -> ... -> sink3
    // ...
    std::vector<std::vector<std::vector<EcoPath>>>& ecoPaths();

    std::vector<odb::dbNet*>& nets();
    std::vector<odb::dbInst*>& insts();
    std::vector<odb::dbInst*>& movableInsts();
    std::vector<odb::dbInst*>& fixedInsts();

    // for gui debugging
    void setEcoPaths(std::vector<std::vector<std::vector<EcoPath>>>& ecoPaths);
    void printRNet();
    void writeSMT2(std::string fileName);
    void writeSMT2Helper(std::string fileName);
    bool readBackSMT2(std::string fileName, EcoOdbNet& odbNetInfo,
        std::set<odb::dbNet*>& rectRevertedOrderedNets);

    // case 2 wrap func
    void modifyPathITEX(odb::dbITerm* beginTerm, 
        ClipLayerPin* endClPin, 
        std::vector<EcoPath>& path, 
        bool& isModified,
        std::vector<RoutedPoint>& routedPath, 
        std::vector<eco::Clip::EcoNode>& newNodes,
        std::vector<eco::Clip::EcoEdge>& newEdges,
        std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap);

    // case 3 warp func
    void modifyPathEXIT(odb::dbITerm* endITerm, 
        ClipLayerPin* beginClPin, 
        std::vector<EcoPath>& path, 
        bool& isModified,
        std::vector<RoutedPoint>& routedPath, 
        std::vector<eco::Clip::EcoNode>& newNodes,
        std::vector<eco::Clip::EcoEdge>& newEdges,
        std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap);


    // incremental RNet update
    void updateRNet(odb::dbNet* net, 
        std::unordered_map<std::pair<int, int>, int, PairHash>& rNetMap,
        std::vector<EcoPath>& ecoNetPaths, 
        EcoBaseVars& ebVars);

    // update RNet's indices to receive string correctly
    void postProcessRNets();
    eco::ClipLayer* dbToEb(odb::dbTechLayer* layer) const;
    eco::ClipLayer* dbToEb(int layerNum) const;

    // SMT2 writing helper function
    std::pair<int, int> getPrevLayerRowCol(ClipLayer* curClipLayer, 
        int r, int c);
    std::pair<int, int> getNextLayerRowCol(ClipLayer* curClipLayer, 
        int r, int c);

    // cliplayer tool (prev <-> next)
    ClipLayer* getPrevClipLayer(ClipLayer* curClipLayer);
    ClipLayer* getNextClipLayer(ClipLayer* curClipLayer);

    // cliplayer tool (prev <-> next) with r/c (y/x) coordinates
    ClipLayerPin* getPrevLayerClPin(ClipLayer* curClipLayer, int r, int c);
    ClipLayerPin* getNextLayerClPin(ClipLayer* curClipLayer, int r, int c);

    RNet* getRNet(std::string name) const;

    std::vector<eco::Clip::EcoNode> & nodes();
    std::vector<eco::Clip::EcoEdge> & edges();

  private:
    odb::Rect die_, core_;
    odb::dbDatabase* db_;
    int from_route_layer_, to_route_layer_;
    int site_lower_x_, site_upper_x_, site_lower_y_, site_upper_y_;
    bool is_failed_;

    std::vector<eco::ClipLayer> clip_layer_stor_;
    std::vector<eco::ClipLayer*> clip_layers_;
    std::map<odb::dbTechLayer*, eco::ClipLayer*> clip_layer_map_;

    std::vector<eco::Clip::EcoNode> nodes_, new_nodes_;
    std::vector<eco::Clip::EcoEdge> edges_, new_edges_;

    // instance term awareness
    eco::ClipLayer* iterm_layer_;

    // init function
    void initClipLayers(EcoBaseVars ebVars);
    void initDpGrid(); 
    void initMasters(); 

    // routed wire graph path info
    std::vector<std::vector<std::vector<EcoPath>>> eco_paths_;
    std::vector<std::vector<eco::Clip::EcoPath>>& getPath(odb::dbNet* net);

    // netlist info
    std::vector<odb::dbInst*> insts_;
    std::vector<odb::dbInst*> movable_insts_;
    std::vector<odb::dbInst*> fixed_insts_;
    std::set<odb::dbInst*> fixed_insts_map_;
    bool isFixedInsts(odb::dbInst* inst);

    std::vector<odb::dbNet*> nets_;
    std::vector<eco::RNet> rnets_;

    // netName -> RNet* retrieving map
    std::map<std::string, eco::RNet*> rnet_map_;

    // dp site grid info: 2D dp grid, master map (pin locs) etc.
    std::vector<std::vector<DpSite>> dp_grid_;

    // saves fragemented row
    // first: row idx
    // second: vector of <startIdx, endIdx> on each row.
    std::vector<std::vector<std::pair<int,int>>> dp_grid_place_pairs_; 

    std::vector<odb::dbMaster*> db_masters_;
    std::vector<eco::Master> eco_master_stor_;
    std::vector<eco::Master*> eco_masters_;

    eco::Master* dbToEb(odb::dbMaster* master) const;
    std::map<odb::dbMaster*, eco::Master*> db_to_eco_master_map_; 

    eco::Pin* dbToEb(odb::dbMTerm* mterm) const;
    std::map<odb::dbMTerm*, eco::Pin*> db_to_eco_pin_map_;

    // postprocess private functions
    void removeDuplicatedPins();
    void updateRNetNames();
    void updateRNetSingleComm();
    void updateRNetIsFixed();
    void updateRNetDirectionLayer(); 
    void updateRNetClPinAPs();
    void buildCommodityIndices();
    void updateRNetMap();
    void updateObsClPin();
    
    // it enables to retrieve possible SMT2 clPins based on coordinates
    void updateCoordiInfo();

    // AccessPoints / EndPoints
    void updateCoordiInfoAddCoordiAPEP(
        ClipLayerPin* clPin,
        RNet& rNet, int cIdx, 
        std::set<int32_t>& allKeys,
        std::map<int32_t, int32_t>& tmpMap); 

    // for adjacent purpose Coordi creation
    void updateCoordiInfoAddCoordiAdj(
        ClipLayerPin* clPin1, ClipLayerPin* clPin2, 
        std::set<int32_t>& allKeys,
        std::map<int32_t, int32_t>& tmpMap);


    // for movable instance
    std::set<int> getAccessPoints(ClipLayerPin* clPin, 
        int y, int x, int flip);

    std::set<int> getAccessPoints(ClipLayerPin* clPin, 
        int y, int x, bool isFlipX, bool isFlipY);

    // for fixed instance
    std::set<int> getFixedInstAccessPoints(ClipLayerPin* clPin);

    // need to retrieve (l,m,n) -> all possible M1APs, external pins for SMT2 formulation
    std::vector<CoordiInfo> coordi_info_stor_;

    // coordi info key to coordiInfo ptr
    std::map<int, CoordiInfo*> coordi_info_map_;

    // SMT2 writing helper function: takes coordinate and return CoordiInfo object 
    CoordiInfo* getCoordiInfo(int layer, int y, int x);

    // for 2D grid debug purpose.
    void outStreamDpGrid(std::ofstream& out);

    void writeSMT2FlowCapControl(std::ofstream& out, 
        RNet& rNet, int cIdx, ClipLayerPin* clPin);

    void writeSMT2CommFlowConserv(std::ofstream& out,
        RNet& rNet, int cIdx, std::vector<SMT2Segment>& strings);

    void writeSMT2VertexExclusive(std::ofstream& out,
        RNet& rNet, CoordiInfo& coordi, 
        std::vector<SMT2Segment>& strings);

    void writeSMT2EdgeAssignments(std::ofstream& out,
        std::string& commStr, std::vector<std::string>& indivStrs);

    void writeSMT2GeomConsts(std::ofstream& out,
        int mode,
        ClipLayer& clipLayer);

    void writeSMT2SloConstraints(std::ofstream& out,
        std::set<std::string>& allSegmentsSet,
        eco::ClipLayer* clipLayer);

    void writeSMT2SloConstraintsPattern(std::ofstream& out,
        std::set<std::string>& allSegmentsSet,
        eco::ClipLayer* clipLayer, 
        int* patterns, int x, int y, int z);


    // ITERM - ITERM path
    std::vector<EcoPath>
      getModifiedPathITIT(std::vector<RoutedPoint>& routedPath,
          std::vector<eco::Clip::EcoNode>& newNodes,
          std::vector<eco::Clip::EcoEdge>& newEdges, 
          std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap);

    // ITERM - EXTPIN path
    std::vector<EcoPath>
      getModifiedPathITEX(std::vector<RoutedPoint>& routedPath,
          std::vector<eco::Clip::EcoNode>& newNodes,
          std::vector<eco::Clip::EcoEdge>& newEdges, 
          std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
          std::vector<eco::Clip::EcoPath>& path,
          int endIdx);

    // EXTPIN - ITERM path
    std::vector<EcoPath>
      getModifiedPathEXIT(std::vector<RoutedPoint>& routedPath,
          std::vector<eco::Clip::EcoNode>& newNodes,
          std::vector<eco::Clip::EcoEdge>& newEdges, 
          std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
          std::vector<eco::Clip::EcoPath>& path,
          int beginIdx);

    // EXTPIN - EXTPIN path
    std::vector<EcoPath>
      getModifiedPathEXEX(std::vector<RoutedPoint>& routedPath,
          std::vector<eco::Clip::EcoNode>& newNodes,
          std::vector<eco::Clip::EcoEdge>& newEdges, 
          std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
          std::vector<eco::Clip::EcoPath>& path,
          int beginIdx, int endIdx);

    // IT - EXTPIN path (special)
    std::vector<EcoPath>
      getModifiedPathMiddleITEX(std::vector<RoutedPoint>& routedPath,
          std::vector<eco::Clip::EcoNode>& newNodes,
          std::vector<eco::Clip::EcoEdge>& newEdges, 
          std::map<odb::dbTechLayer*, odb::dbTechVia*>& layerViaMap, 
          std::vector<eco::Clip::EcoPath>& path,
          int beginIdx, int endIdx);

    void updateEdgeMark(EcoBaseVars& ebVars, 
        EcoNode& prevNode, EcoNode& nextNode, 
        eco::OverlapMark& mark,
        odb::Rect rect);

    void updateCoordiInfoClipPtr();

    void getInstRoutingPinBlockageSegs(odb::dbInst* inst, 
        int gLx, int gLy,
        bool isFlipX, bool isFlipY,
        std::set<std::string>& returnSets);

    void updateInstRoutingPinBlockage(odb::dbInst* inst, 
        int gLx, int gLy,
        bool isFlipX, bool isFlipY);

};

inline std::vector<eco::ClipLayer*>
Clip::clipLayers() {
  return clip_layers_;
}

inline odb::Rect
Clip::die() const {
  return die_;
}

inline odb::Rect
Clip::core() const {
  return core_;
}

inline bool
Clip::isFailed() const {
  return is_failed_;
}

inline std::vector<std::vector<std::vector<eco::Clip::EcoPath>>>&
Clip::ecoPaths() {
  return eco_paths_;
}

inline std::vector<odb::dbNet*>&
Clip::nets() {
  return nets_;
}

inline std::vector<odb::dbInst*>&
Clip::insts() {
  return insts_;
}

inline std::vector<odb::dbInst*>&
Clip::fixedInsts() {
  return fixed_insts_;
}

inline std::vector<odb::dbInst*>&
Clip::movableInsts() {
  return movable_insts_;
}

inline std::vector<eco::Clip::EcoNode>&
Clip::nodes() {
  return nodes_;
}

inline std::vector<eco::Clip::EcoEdge>&
Clip::edges() {
  return edges_;
}

class Graphics : public gui::Renderer
{
  public:
    Graphics(utl::Logger* logger, Clip* clip);
    void plot(bool pause = true);
    void highlightClip(Clip* clip);
    virtual void drawObjects(gui::Painter& painter) override;

    static bool guiActive();

  private:
    utl::Logger* log_;
    Clip* clip_;
    int net_idx_;
    int wire_idx_;
};


class EcoBaseVars {
  public:
    EcoBaseVars();
    void reset();

    int swBoxSolveRangeX, swBoxSolveRangeY;
    int swBoxObsRangeX, swBoxObsRangeY;
    int fromMetalBlockLayer;
    int toMetalBlockLayer;
    int fromMetalRouteLayer;
    int toMetalRouteLayer;
    int localStepX, localStepY;
    int localCntX, localCntY;
    int slackThreshold;
    bool writeZ3FileMode;
    bool readZ3FileMode;
    bool guiMode;
    std::string writeZ3DirName;
    std::string readZ3DirName;
};

/////////////////////////////////////////////////////////////
// For the RECT stitching flow
//
// 1. TVIA -> POINT -> RECT 
//  a. POINT -> RECT
//  b. TVIA -> RECT (if point doesn't exist in new ordered wire)
// 
// 2. VIA -> POINT -> RECT 
//  a. POINT -> RECT
//  b. VIA -> RECT (if point doesn't exist in new ordered wire)
// 
// 3. TVIA -> RECT
//  a. TVIA -> RECT
// 
// 4. VIA -> RECT
//  a. VIA -> RECT
//
// --> HAVE TO SAVE PREVIOUS OPCODE
class RectInfo {
  public:
    odb::dbWireDecoder::OpCode prevOpcode; 
    odb::Rect rect;

    RectInfo(odb::dbWireDecoder::OpCode opcode, 
        odb::Rect inpRect) {prevOpcode = opcode; rect = inpRect;};
};

// EcoOdbNet is for retrieving the "RECT" segment 
// odb's orderWire has a bug - RECT elements are dropped.
// 
class EcoOdbNet
{
  public:
    EcoOdbNet();
    void init(odb::dbSet <odb::dbNet> nets);
    bool isInit() { return is_init_; };

    // input net, layer, x, y
    // output: odb::Rect object
    RectInfo getRect(odb::dbNet* net, int layer, int x, int y);

  private:
    bool is_init_;
    // 1D: net
    // 2D: rects_ index.
    // std::map<odb::dbNet*, int> nets_map_;
    std::map<odb::dbNet*, int> nets_map_;

    // 1D: net
    // 2D: rects idx
    std::vector<std::vector<RectInfo>> rects_;

    // 1D: net
    // 2D: key - tuple
    //     1: layerNum
    //     2: x
    //     3: y
    // 3D: rects idx 
    std::vector<std::map<std::tuple<int, int, int>, int>> rects_map_;
};

class EcoBase 
{
  public:
    EcoBase(
        EcoBaseVars& ebVars,
        odb::dbDatabase* odb, 
        sta::dbSta* dbSta, 
        utl::Logger* log, 
        int verbose=0);
    ~EcoBase();

    void setDrcCoordis(std::vector<std::pair<int, int>>& drcCoordis);
    void setEbVars(EcoBaseVars ebVars);
    void run();
    // incremental mode func
    bool runGenSMT2(std::string fileName, int cx, int cy, int origLx, int origLy, int origUx, int origUy);
    bool runReadSMT2(std::string fileName, int lx, int ly, int ux, int uy);
    void runSaveCurRect();
    void runPostProcessRectNets();

    void writeRects(std::string fileName);
    void readRects(std::string fileName);

    std::vector<eco::Clip>& clips();

  private:
    odb::dbDatabase* db_;
    sta::dbSta* sta_;
    utl::Logger* log_;
    std::unique_ptr<Graphics> graphics_;

    void *iterm_rtree_;
    void *wire_rtree_;

    EcoBaseVars eb_vars_;
    std::vector<std::pair<int, int>> drc_coordis_;

    void reset();
    void initRTree();

    // cx, cy: center inates of clip region 
    // the lx, ly,  ux, uy will be calculated later.
    //
    // allRects obj is required to avoid conflicts/overlaps
    eco::Clip extractClip(int x, int y, std::vector<odb::Rect>& allRects,
        int origLx = -1, int origLy = -1, int origUx = -1, int origUy = -1);

    std::vector<eco::Clip> clips_;
    // for incremental mode.
    eco::Clip incremental_clip_;

    // ordered_nets handling
    std::set<odb::dbNet*> ordered_nets_;

    // rect property reverted net handling
    EcoOdbNet odb_rect_nets_info_;
    std::set<odb::dbNet*> rect_reverted_ordered_nets_;

    // store of successful_rects_
    std::vector<odb::Rect> passed_rects_;

    int verbose_;
};

inline std::vector<eco::Clip>& 
EcoBase::clips() {
  return clips_;
}

struct PairHash {
  std::size_t operator()(const std::pair<int, int> &k) const {
    return k.first + k.second;
  }
};


}
#endif
