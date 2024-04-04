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

#ifndef __CORE_ECO_HEADER__
#define __CORE_ECO_HEADER__

#include <memory>
#include <vector>

namespace odb {
  class dbDatabase;
  class dbInst;
}
namespace sta {
  class dbSta;
}

namespace utl {
  class Logger;
}

namespace eco {

class EcoBase;
class CoreEco 
{
  public:
    CoreEco();
    ~CoreEco();

    void init(odb::dbDatabase* odb, sta::dbSta* dbSta, 
        utl::Logger* log, int verbose=0);
    void reset();

    // load default vars if doesn't exist
    void loadVars();

    // switch box range setup:  ObsRange >= SolveRange
    // ObsRange includes SolveRange. See CoReECO paper.
    void setSwitchBoxSolveRange(int width, int height);
    void setSwitchBoxObsRange(int width, int height);

    // routing layer range setup 
    void setMetalBlockRange(int fromLayer, int toLayer);
    void setMetalRouteRange(int fromLayer, int toLayer);

    // parse innovus drv reports
    // TODO: change this function to use DRC engine from TR
    void parseDrcReport(const char* file);
    void addDrcCoordi(int x, int y);

    // based on given drc coordinates, 
    // create localCntX * localCntY sub windows and solve SMTs
    void setSwitchBoxLocalStep(int localStepX, int localStepY);
    void setSwitchBoxLocalCnt(int localCntX, int localCntY);


    void setVerboseLevel(int verbose);

    // if timing slack < threshold, fix all instances.
    // if you're ok to change timing results, set threshold=0%
    // default: top 10%
    void setTimingFixThreshold(float slackThreshold);

    void runCoreEco();
    void setWriteZ3FileMode(bool mode);
    void setReadZ3FileMode(bool mode);
    void setWriteZ3FileDir(std::string dirName);
    void setReadZ3FileDir(std::string dirName);

    bool runGenSMT2CoreEco(std::string fileName, int cx, int cy, int orig_lx, int orig_ly, int orig_ux, int orig_uy);
    bool runReadSMT2CoreEco(std::string fileName, int lx, int ly, int ux, int uy); 
    void runSaveCurRect();
    void runPostProcess();

    void setLocalStep(int x, int y);
    void setLocalCnt(int x, int y);

    void setGuiMode(bool mode);

    void writeRects(std::string fileName);
    void readRects(std::string fileName);

  private:
    odb::dbDatabase* db_;
    sta::dbSta* sta_;
    utl::Logger* log_;

    std::shared_ptr<EcoBase> eb_;

    std::vector<std::pair<int, int>> drc_coordis_;

    int sw_box_solve_range_x_, sw_box_solve_range_y_;
    int sw_box_obs_range_x_, sw_box_obs_range_y_;

    int from_metal_block_layer_, 
        to_metal_block_layer_;
    int from_metal_route_layer_, 
        to_metal_route_layer_;

    int local_step_x_, local_step_y_,
        local_cnt_x_, local_cnt_y_;

    float slack_threshold_;
    bool write_z3_file_mode_;
    bool read_z3_file_mode_;
    bool gui_mode_;
    int verbose_;

    std::string write_z3_dir_name_;
    std::string read_z3_dir_name_;

    std::string rect_file_;
};
}

#endif
