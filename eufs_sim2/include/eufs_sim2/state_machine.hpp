#ifndef INCLUDE_EUFS_SIM2_STATE_MACHINE_HPP_
#define INCLUDE_EUFS_SIM2_STATE_MACHINE_HPP_
#include <chrono>  // NOLINT
#include <string>

#include "eufs_sim2/sensors/wheel_speeds.hpp"
#include "time/time.hpp"

namespace eufs::sim2::state_machine {

enum class ASState { OFF, READY, DRIVING, FINISHED, EMERGENCY_BRAKE };

enum class AMIState {
  NOT_SELECTED,
  ACCELERATION,
  SKIDPAD,
  AUTOCROSS,
  TRACK_DRIVE,
  MANUAL_DRIVING,
  // Exclusive to ADS
  ADS_EBS_TEST,
  ADS_INSPECTION,
  // Exclusive to DDT
  DDT_INSPECTION_A,
  DDT_INSPECTION_B,
  DDT_AUTONOMOUS_DEMO
};

class StateMachine {
  using Time = eufs::sim2::time::Time;

 public:
  /**
   * @brief Attempts to transition to the next state of the StateMachine.
   * @param time Simulation time object.
   * @param ws Wheel speed object.
   * @return True if successful, false otherwise.
   */
  bool UpdateStatus(const Time& time, const sensors::WheelSpeeds::Vector& ws);

  /**
   * @brief Sets the mission for the vehicle to perform.
   * @param mission Mission to set.
   * @return True if successful, false otherwise.
   */
  bool SetMission(const AMIState& mission);

  /**
   * @brief Resets the state of the StateMachine.
   * @return True if successful, false otherwise.
   */
  bool ResetState();

  /**
   * @brief Triggers the Emergency Brake System (EBS)
   * @return True if EBS triggered successfully, false otherwise.
   */
  bool TriggerEBS();

  /**
   * @brief Returns the current AS state of the StateMachine.
   * @return Current AS state.
   */
  ASState GetASState() const;

  /**
   * @brief Returns the current AMI state of the StateMachine.
   * @return Current AMI state.
   */
  AMIState GetAMIState() const;

  /**
   * @brief Checks if the car is in a state that allows movement.
   * @return True if the car is in state allowed to move, false otherwise.
   */
  bool CanDrive() const;

  /**
   * @brief Completes the current mission; allowing a future transition to AS FINISHED state.
   * @return True if the state is successfully finished, false otherwise.
   */
  bool FinishMission();

  /**
   * @brief Changes state from Ready to Driving
   * @return True if the (previous) state is Ready
  */
  bool StartDriving();

  /**
   * @brief Checks if StartDriving has been called
   * @return True if StartDriving has been called, false otherwise
  */
  bool CanStartDriving() const;

  /**
   * @brief Checks if the current mission is marked as finished.
   * @return True if the mission is marked as complete, false otherwise.
   */
  bool IsMissionFinished() const;

  /**
   * @brief Overloadable function(s) that returns a string for a given enum
   * @param enum in question
   * @return
   */
  static std::string GetEnumString(AMIState amiState);
  static std::string GetEnumString(ASState asState);

 private:
  /**
   * @brief Check the feasability of transitioning DRIVING -> FINISHED.
   * @return True if wheel speeds are below 10 rpm, false otherwise.
   */
  bool CheckWheelSpeeds(const sensors::WheelSpeeds::Vector& ws) const;

 private:
  ASState as_state_ = ASState::OFF;
  AMIState ami_state_ = AMIState::NOT_SELECTED;
  bool mission_finished_ = false;
  bool ebs_active_ = false;
  bool go_ = false;
  eufs::sim2::time::Time timer_R2D_;
  eufs::sim2::time::Time timer_EBS_;
};

}  // namespace eufs::sim2::state_machine

#endif  // INCLUDE_EUFS_SIM2_STATE_MACHINE_HPP_
