/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <mutex>

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_legged_robot/gait/ModeSequenceTemplate.h"

namespace ocs2 {
namespace legged_robot {

class GaitSchedule {
 public:
  GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime);

  /**
   * Sets the mode schedule.
   *
   * @param [in] modeSchedule: The mode schedule to be used.
   */
  void setModeSchedule(const ModeSchedule& modeSchedule) {
    std::lock_guard<std::mutex> lock(mutex_);
    modeSchedule_ = modeSchedule;
  }

  /**
   * Gets the mode schedule.
   *
   * @param [in] lowerBoundTime: The smallest time for which the ModeSchedule should be defined.
   * @param [in] upperBoundTime: The greatest time for which the ModeSchedule should be defined.
   */
  ModeSchedule getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime);

  /**
   * megaDog-specific: a pure read-only snapshot of the currently stored mode schedule, safe to call
   * from any thread against the SAME instance getModeSchedule()/setModeSchedule()/
   * insertModeSequenceTemplate() are being called on from other threads (see mutex_'s doc comment).
   * Unlike getModeSchedule(scalar_t, scalar_t), this never re-tiles or otherwise mutates
   * modeSchedule_/modeSequenceTemplate_ - it just copies out the current state under the lock. Use
   * this for a cheap, frequent "what mode is active at time t" point query (e.g. every real-time
   * control tick); use the mutating overload only for the periodic gait-schedule-window refresh.
   */
  ModeSchedule getCurrentModeSchedule() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return modeSchedule_;
  }

  /**
   * Used to insert a new user defined logic in the given time period.
   *
   * @param [in] startTime: The initial time from which the new mode sequence template should start.
   * @param [in] finalTime: The final time until when the new mode sequence needs to be defined.
   * @param [in] allowPhaseTransitionStance: megaDog-specific (default true, matching upstream's
   * always-bridge behavior). MegadogWbcRuntime calls this not just on an actual gait change but also
   * periodically (~3*timeHorizon seconds) just to keep the schedule's tiled window from running out,
   * even when the gait name hasn't changed. Always inserting phaseTransitionStanceTime_'s forced STANCE
   * bridge on those keepalive refreshes splices a stance phase into an in-progress swing roughly every
   * 3s with no relation to the gait's own cadence - felt as a periodic, repeating "loss of power"/stumble
   * during otherwise-normal trot. Pass false on a keepalive refresh (same gait name) so only a genuine
   * gait change gets the bridge.
   */
  void insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime,
                                  bool allowPhaseTransitionStance = true);

 private:
  // megaDog-specific: unlike upstream's single-threaded usage, MegadogWbcRuntime
  // calls into this class from two different OS threads (the MPC worker thread's
  // advanceMpc()->modifyReferences()->getModeSchedule(), and the real-time
  // control thread's update()->setGaitTemplateIfNeeded()->insertModeSequenceTemplate())
  // against the SAME shared GaitSchedule instance - confirmed via a multi-agent
  // investigation to be a genuine, unguarded data race on modeSchedule_/
  // modeSequenceTemplate_'s plain std::vectors, capable of producing a torn/
  // inconsistent read (mismatched eventTimes/modeSequence sizes, or a
  // mid-assignment modeSequenceTemplate_) that manifests as a sudden,
  // catastrophic single-tick divergence - this is what megaDog's own
  // ~15-25s-into-any-gait instability traced back to. This mutex guards every
  // public entry point below so the two threads can never observe or produce
  // a torn intermediate state.
  //
  // A SIBLING race was found later, same session: MegadogWbcRuntime::update()
  // (control thread) used to call interface_->getReferenceManagerPtr()->
  // getModeSchedule() for its own "what mode is active right now" point
  // queries - a completely SEPARATE ModeSchedule (ocs2::ReferenceManager's
  // own BufferedValue<ModeSchedule> modeSchedule_), not this class's. That
  // buffer's active value is documented (BufferedValue.h) as unsafe to
  // read via get() concurrently with updateFromBuffer() - and
  // ReferenceManager::preSolverRun() calls updateFromBuffer() on it from
  // inside advanceMpc(), i.e. from the MPC worker thread, while update()'s
  // get() ran on the control thread: the exact same cross-thread pattern
  // this mutex was built to prevent, just on a different object. Symptom:
  // an intermittent "inconsistent mode schedule: eventTimes=N
  // modeSequence=N+2" warning (torn read of the two vectors mid
  // move-assignment) immediately followed by a torque spike and a full
  // tumble - reproduced in sim during an unrelated WBC-tuning experiment.
  // Fixed by adding getCurrentModeSchedule() above and switching
  // MegadogWbcRuntime's two per-tick queries to read through THIS
  // instance's mutex instead of ReferenceManager's unguarded buffer -
  // SwitchedModelReferenceManager::setModeSchedule() already keeps both
  // copies in sync, so this instance always has the same data, just safely.
  mutable std::mutex mutex_;

 private:
  /**
   * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
   *
   * @param [in] startTime: The initial time from which the mode schedule should be appended with the template.
   * @param [in] finalTime: The final time to which the mode schedule should be appended with the template.
   */
  void tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime);

 private:
  ModeSchedule modeSchedule_;
  ModeSequenceTemplate modeSequenceTemplate_;
  scalar_t phaseTransitionStanceTime_;
};

}  // namespace legged_robot
}  // namespace ocs2
