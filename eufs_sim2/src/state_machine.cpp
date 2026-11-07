#include "eufs_sim2/state_machine.hpp"
#include <state_lib/state.hpp>

namespace eufs::sim2::state_machine {
  using std::chrono_literals::operator""s;

bool StateMachine::UpdateStatus(const Time& time, const sensors::WheelSpeeds::Vector& ws) {
  switch (as_state_) {
    case ASState::OFF:
      if (ami_state_ != AMIState::NOT_SELECTED) {
        as_state_ = ASState::READY;
        timer_R2D_ = time;
        return true;
      }
      break;

    case ASState::READY:
      if (ebs_active_) {
        as_state_ = ASState::EMERGENCY_BRAKE;
        timer_EBS_ = time;
        return true;
      }

      if (time::HasTimeElapsed(time, timer_R2D_, 5s)) {
        if (go_) {
          as_state_ = ASState::DRIVING;
          return true;
        }
      }
      break;

    case ASState::DRIVING:
      if (ebs_active_) {
        as_state_ = ASState::EMERGENCY_BRAKE;
        timer_EBS_ = time;
        return true;
      }
      if (mission_finished_) {
        if (CheckWheelSpeeds(ws)) {
          as_state_ = ASState::FINISHED;
          return true;
        } else {  // Trigger EBS
          as_state_ = ASState::EMERGENCY_BRAKE;
          timer_EBS_ = time;
          return true;
        }
      }
      break;

    case ASState::EMERGENCY_BRAKE:
      if (time::HasTimeElapsed(time, timer_EBS_, 15s)) {
        return ResetState();
      }
      break;

    case ASState::FINISHED:
      break;
  }
  return false;
}

bool StateMachine::CheckWheelSpeeds(const sensors::WheelSpeeds::Vector& ws) const {
  if (ws(sensors::WheelSpeedsMember::_rl) <= 10 &&
      ws(sensors::WheelSpeedsMember::_rr) <= 10 &&
      ws(sensors::WheelSpeedsMember::_fl) <= 10 &&
      ws(sensors::WheelSpeedsMember::_fr) <= 10) {
    return true;
  }
  return false;
}

bool StateMachine::SetMission(const AMIState& mission) {
  if (mission == AMIState::NOT_SELECTED) return false;     // Invalid mission
  if (ami_state_ != AMIState::NOT_SELECTED) return false;  // Mission already selected

  ami_state_ = mission;  // AMI state will only be selected once
  return true;
}

bool StateMachine::ResetState() {
  as_state_ = ASState::OFF;
  ami_state_ = AMIState::NOT_SELECTED;
  mission_finished_ = false;
  ebs_active_ = false;
  go_ = false;
  return true;
}

bool StateMachine::TriggerEBS() {
  if (as_state_ == ASState::READY || as_state_ == ASState::DRIVING) {
    ebs_active_ = true;
    return true;
  }
  return false;
}

bool StateMachine::FinishMission() {
  if (as_state_ == ASState::DRIVING) {
    mission_finished_ = true;
    return true;
  }
  return false;
}

bool StateMachine::StartDriving() {
  if (as_state_ == ASState::READY) {
    go_ = true;
    return true;
  }
  return false;
}

bool StateMachine::IsMissionFinished() const { return mission_finished_; }

ASState StateMachine::GetASState() const { return as_state_; }

AMIState StateMachine::GetAMIState() const { return ami_state_; }

bool StateMachine::CanStartDriving() const { return go_; }

bool StateMachine::CanDrive() const {
  return as_state_ == ASState::DRIVING || ami_state_ == AMIState::MANUAL_DRIVING;
}

std::string StateMachine::GetEnumString(AMIState amiState) {
  switch (amiState) {
    case(AMIState::NOT_SELECTED): return "NOT_SELECTED";
    case(AMIState::ACCELERATION): return "ACCELERATION";
    case(AMIState::SKIDPAD): return "SKIDPAD";
    case(AMIState::AUTOCROSS): return "AUTOCROSS";
    case(AMIState::TRACK_DRIVE): return "TRACK_DRIVE";
    case(AMIState::DDT_AUTONOMOUS_DEMO): return "DDT_AUTONOMOUS_DEMO";
    case(AMIState::ADS_INSPECTION): return "ADS_INSPECTION";
    case(AMIState::ADS_EBS_TEST): return "ADS_EBS_TEST";
    case(AMIState::DDT_INSPECTION_A): return "DDT_INSPECTION_A";
    case(AMIState::DDT_INSPECTION_B): return "DDT_INSPECTION_B";
    case(AMIState::MANUAL_DRIVING): return "MANUAL_DRIVING";
    default: return "Invalid AMIState";
  }
}

std::string StateMachine::GetEnumString(ASState asState) {
  switch (asState) {
    case(state_machine::ASState::OFF): return "OFF";
    case(state_machine::ASState::READY): return "READY";
    case(state_machine::ASState::DRIVING): return "DRIVING";
    case(state_machine::ASState::EMERGENCY_BRAKE): return "EMERGENCY_BRAKE";
    case(state_machine::ASState::FINISHED): return "FINISHED";
    default: return "Invalid ASState";
  }
}

}  // namespace eufs::sim2::state_machine
