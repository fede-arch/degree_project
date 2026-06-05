/*
 * Navigation plugin for Tethys AUV
 * LOS guidance + PID controller - 3D
 * Two-Phase Approach:
 *   Phase 1: YAW ALIGNMENT - spin in place until facing target (zero thrust)
 *   Phase 2: ADVANCE - full -31 thrust forward until arrival/braking zone
 */

#include "NavigationPlugin.hh"
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <cmath>

namespace tethys
{

class NavigationPrivateData
{
  public: gz::math::Vector3d target {0.0, 0.0, 0.0};
  public: bool hasTarget {false};
  public: bool arrived {false};
  
  // Navigation thresholds
  public: double arrivalThreshold {3.0};
  public: double brakingDistance {20.0};
  public: double yawAlignmentThreshold {0.3};  // rad (~17°)

  // Actuator limits
  public: double maxThrust {31.0};
  public: double maxFin {0.261799};           // ~15°
  public: double minThrustMagnitude {3.0};    // Minimum to keep fins effective

  // Thrust control
  public: double cruiseThrust {31.0};  // Full thrust when aligned and advancing

  // Fin smoothing (low-pass filter)
  public: double finAlpha {0.2};
  public: double prevVertFin {0.0};
  public: double prevHorizFin {0.0};

  // PID Yaw controller (for rotation in place)
  public: double kpYaw {2.0};      // Increased for faster yaw response
  public: double kiYaw {0.02};
  public: double kdYaw {0.4};
  public: double yawIntegral {0.0};
  public: double prevYawError {0.0};

  // PID Pitch controller
  public: double kpPitch {1.5};
  public: double kiPitch {0.01};
  public: double kdPitch {0.3};
  public: double pitchIntegral {0.0};
  public: double prevPitchError {0.0};

  public: double integralMax {0.5};

  // Logging
  public: double logTimer {0.0};
  public: double logInterval {1.0};

  // Gazebo components
  public: gz::sim::Entity linkEntity;
  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher thrustPub;
  public: gz::transport::Node::Publisher verticalFinPub;
  public: gz::transport::Node::Publisher horizontalFinPub;
  public: gz::transport::Node::Publisher statusPub;
  public: std::mutex mtx;

  // Helper: normalize angle to [-π, π]
  public: static double normalizeAngle(double angle)
  {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Helper: publish zero commands (stop)
  public: void publishStop()
  {
    gz::msgs::Double msg;
    msg.set_data(0.0);
    this->thrustPub.Publish(msg);
    this->verticalFinPub.Publish(msg);
    this->horizontalFinPub.Publish(msg);
  }

  // Callback: receive new target from topic
  public: void OnTarget(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->target = gz::msgs::Convert(_msg);
    this->hasTarget = true;
    this->arrived = false;
    
    // Reset PID states
    this->yawIntegral = 0.0;
    this->prevYawError = 0.0;
    this->pitchIntegral = 0.0;
    this->prevPitchError = 0.0;
    this->prevVertFin = 0.0;
    this->prevHorizFin = 0.0;
    
    std::cout << "[NAV] New target: ("
              << this->target.X() << ", "
              << this->target.Y() << ", "
              << this->target.Z() << ")" << std::endl;
  }
};

NavigationPlugin::NavigationPlugin()
  : dataPtr(std::make_unique<NavigationPrivateData>())
{
}

void NavigationPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/)
{
  auto model = gz::sim::Model(_entity);

  // Load configuration from SDF
  if (_sdf->HasElement("arrival_threshold"))
    this->dataPtr->arrivalThreshold = _sdf->Get<double>("arrival_threshold");
  if (_sdf->HasElement("braking_distance"))
    this->dataPtr->brakingDistance = _sdf->Get<double>("braking_distance");
  if (_sdf->HasElement("yaw_alignment_threshold"))
    this->dataPtr->yawAlignmentThreshold = _sdf->Get<double>("yaw_alignment_threshold");

  if (_sdf->HasElement("cruise_thrust"))
    this->dataPtr->cruiseThrust = _sdf->Get<double>("cruise_thrust");
  if (_sdf->HasElement("max_thrust"))
    this->dataPtr->maxThrust = _sdf->Get<double>("max_thrust");
  if (_sdf->HasElement("min_thrust_magnitude"))
    this->dataPtr->minThrustMagnitude = _sdf->Get<double>("min_thrust_magnitude");

  if (_sdf->HasElement("kp_yaw"))
    this->dataPtr->kpYaw = _sdf->Get<double>("kp_yaw");
  if (_sdf->HasElement("ki_yaw"))
    this->dataPtr->kiYaw = _sdf->Get<double>("ki_yaw");
  if (_sdf->HasElement("kd_yaw"))
    this->dataPtr->kdYaw = _sdf->Get<double>("kd_yaw");

  if (_sdf->HasElement("kp_pitch"))
    this->dataPtr->kpPitch = _sdf->Get<double>("kp_pitch");
  if (_sdf->HasElement("ki_pitch"))
    this->dataPtr->kiPitch = _sdf->Get<double>("ki_pitch");
  if (_sdf->HasElement("kd_pitch"))
    this->dataPtr->kdPitch = _sdf->Get<double>("kd_pitch");

  if (_sdf->HasElement("max_fin"))
    this->dataPtr->maxFin = _sdf->Get<double>("max_fin");
  if (_sdf->HasElement("fin_alpha"))
    this->dataPtr->finAlpha = _sdf->Get<double>("fin_alpha");

  // Load initial target if provided
  if (_sdf->HasElement("initial_target_x") &&
      _sdf->HasElement("initial_target_y") &&
      _sdf->HasElement("initial_target_z"))
  {
    this->dataPtr->target.X(_sdf->Get<double>("initial_target_x"));
    this->dataPtr->target.Y(_sdf->Get<double>("initial_target_y"));
    this->dataPtr->target.Z(_sdf->Get<double>("initial_target_z"));
    this->dataPtr->hasTarget = true;
    std::cout << "[NAV] Initial target loaded: ("
              << this->dataPtr->target.X() << ", "
              << this->dataPtr->target.Y() << ", "
              << this->dataPtr->target.Z() << ")" << std::endl;
  }

  // Get link entity (usually "base_link")
  auto linkName = _sdf->Get<std::string>("link_name", "base_link").first;
  this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);

  if (gz::sim::kNullEntity == this->dataPtr->linkEntity)
  {
    gzerr << "[NavigationPlugin] Failed to find link: " << linkName << "\n";
    return;
  }

  // Ensure WorldPose component exists
  if (!_ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->linkEntity))
    _ecm.CreateComponent(this->dataPtr->linkEntity, gz::sim::components::WorldPose());

  // Get namespace (model name by default)
  auto ns = _sdf->Get<std::string>("namespace", model.Name(_ecm)).first;

  // Advertise control topics
  this->dataPtr->thrustPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
    "/model/" + ns + "/joint/propeller_joint/cmd_thrust");
  this->dataPtr->verticalFinPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
    "/model/" + ns + "/joint/vertical_fins_joint/0/cmd_pos");
  this->dataPtr->horizontalFinPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
    "/model/" + ns + "/joint/horizontal_fins_joint/0/cmd_pos");
  this->dataPtr->statusPub = this->dataPtr->node.Advertise<gz::msgs::StringMsg>(
    "/model/" + ns + "/navigation/status");

  // Subscribe to target commands
  this->dataPtr->node.Subscribe(
    "/model/" + ns + "/navigation/target",
    &NavigationPrivateData::OnTarget,
    this->dataPtr.get());

  std::cout << "[NAV] Initialized. Two-phase navigation active:" << std::endl;
  std::cout << "      Phase 1: Yaw alignment (spin in place)" << std::endl;
  std::cout << "      Phase 2: Full thrust advance + pitch control" << std::endl;
}

void NavigationPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

  // If arrived at target, maintain stop
  if (this->dataPtr->arrived)
  {
    this->dataPtr->publishStop();
    return;
  }

  if (!this->dataPtr->hasTarget)
    return;

  // Get current pose
  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto pose = baseLink.WorldPose(_ecm);
  if (!pose)
    return;

  auto pos = pose->Pos();
  auto rot = pose->Rot();

  // Compute error vector to target
  double dx = this->dataPtr->target.X() - pos.X();
  double dy = this->dataPtr->target.Y() - pos.Y();
  double dz = this->dataPtr->target.Z() - pos.Z();

  double horizDist = std::sqrt(dx*dx + dy*dy);
  double distance3D = std::sqrt(dx*dx + dy*dy + dz*dz);

  // ─── PHASE 1: Check Arrival ───
  if (distance3D < this->dataPtr->arrivalThreshold)
  {
    std::cout << "[NAV] ARRIVED! Distance: " << distance3D << " m" << std::endl;
    this->dataPtr->hasTarget = false;
    this->dataPtr->arrived = true;

    this->dataPtr->publishStop();

    gz::msgs::StringMsg statusMsg;
    statusMsg.set_data("ARRIVED");
    this->dataPtr->statusPub.Publish(statusMsg);
    return;
  }

  // Time step
  double dt = (double)_info.dt.count() / 1e9;
  if (dt <= 0.0) dt = 0.001;

  // ─── LOS Guidance: Yaw ───
  double desiredYaw = std::atan2(dy, dx);
  double currentYaw = rot.Yaw();
  double yawError = NavigationPrivateData::normalizeAngle(desiredYaw - currentYaw);

  // ─── LOS Guidance: Pitch ───
  double desiredPitch = std::atan2(dz, horizDist);
  double currentPitch = rot.Pitch();
  double pitchError = NavigationPrivateData::normalizeAngle(desiredPitch - currentPitch);

  // ─── YAW PID Controller ───
  this->dataPtr->yawIntegral += yawError * dt;
  this->dataPtr->yawIntegral = std::clamp(
    this->dataPtr->yawIntegral,
    -this->dataPtr->integralMax,
    this->dataPtr->integralMax);
  
  double yawDerivative = (yawError - this->dataPtr->prevYawError) / dt;
  this->dataPtr->prevYawError = yawError;

  double yawPID = this->dataPtr->kpYaw * yawError
                + this->dataPtr->kiYaw * this->dataPtr->yawIntegral
                + this->dataPtr->kdYaw * yawDerivative;

  double vertFin = std::clamp(-yawPID, -this->dataPtr->maxFin, this->dataPtr->maxFin);

  // ─── PITCH PID Controller ───
  this->dataPtr->pitchIntegral += pitchError * dt;
  this->dataPtr->pitchIntegral = std::clamp(
    this->dataPtr->pitchIntegral,
    -this->dataPtr->integralMax,
    this->dataPtr->integralMax);
  
  double pitchDerivative = (pitchError - this->dataPtr->prevPitchError) / dt;
  this->dataPtr->prevPitchError = pitchError;

  double pitchPID = this->dataPtr->kpPitch * pitchError
                  + this->dataPtr->kiPitch * this->dataPtr->pitchIntegral
                  + this->dataPtr->kdPitch * pitchDerivative;

  double horizFin = std::clamp(-pitchPID, -this->dataPtr->maxFin, this->dataPtr->maxFin);

  // ─── Low-pass Filter for Fins ───
  vertFin = this->dataPtr->finAlpha * vertFin
          + (1.0 - this->dataPtr->finAlpha) * this->dataPtr->prevVertFin;
  horizFin = this->dataPtr->finAlpha * horizFin
           + (1.0 - this->dataPtr->finAlpha) * this->dataPtr->prevHorizFin;
  this->dataPtr->prevVertFin = vertFin;
  this->dataPtr->prevHorizFin = horizFin;

  // ─── Thrust Control: Two-Phase Strategy with Pitch Compensation ───
  // Phase 1: YAW ALIGNMENT (full thrust + steering fins)
  // Phase 2: ADVANCE (full thrust until braking zone)
  // PITCH COMPENSATION: boost thrust when climbing/descending to overcome gravity
  
  double thrustCommand = 0.0;
  std::string thrustPhase = "ALIGN";
  double absYawError = std::abs(yawError);
  
  // Pitch compensation factor: |sin(pitchError)| tells us how much vertical component we need
  // sin(pitchError) = 0 when level, ±1 when full pitch up/down
  // Boost factor: 1.0 when level, up to ~1.5 when pitching hard
  double pitchCompensation = 1.0 + 0.3 * std::abs(std::sin(pitchError));
  pitchCompensation = std::min(pitchCompensation, 1.3);
  
  if (absYawError <= this->dataPtr->yawAlignmentThreshold || 
    absYawError >= (M_PI - this->dataPtr->yawAlignmentThreshold))
  {
    // ─── PHASE 2: ALIGNED → ADVANCE with full thrust ───
    thrustPhase = "ADVANCE";
    
    // Braking factor: smooth deceleration in final zone
    double brakeFactor = std::min(distance3D / this->dataPtr->brakingDistance, 1.0);
    
    // Full cruise thrust * brake factor * pitch compensation
    // Negative because Tethys goes forward with negative thrust
    thrustCommand = -this->dataPtr->cruiseThrust * brakeFactor * pitchCompensation;
  }
  else
  {
    // ─── PHASE 1: NOT ALIGNED → FULL THRUST + STEERING ───
    // Use full thrust for good descent/ascent control
    // Fins will steer the AUV toward target while moving forward at full speed
    
    thrustCommand = -this->dataPtr->cruiseThrust * pitchCompensation;
  }

  // ─── Logging (every second) ───
  this->dataPtr->logTimer += dt;
  if (this->dataPtr->logTimer >= this->dataPtr->logInterval)
  {
    this->dataPtr->logTimer = 0.0;
    std::cout << "[NAV] "
      << "pos=(" << pos.X() << ", " << pos.Y() << ", " << pos.Z() << ") | "
      << "dist=" << distance3D << " m | "
      << "yaw_err=" << (yawError * 180.0 / M_PI) << "° | "
      << "pitch_err=" << (pitchError * 180.0 / M_PI) << "° | "
      << "phase=" << thrustPhase << " | "
      << "thrust=" << thrustCommand
      << std::endl;
  }

  // ─── Publish Commands ───
  gz::msgs::Double thrustMsg;
  thrustMsg.set_data(thrustCommand);
  this->dataPtr->thrustPub.Publish(thrustMsg);

  gz::msgs::Double vertMsg;
  vertMsg.set_data(vertFin);
  this->dataPtr->verticalFinPub.Publish(vertMsg);

  gz::msgs::Double horizMsg;
  horizMsg.set_data(horizFin);
  this->dataPtr->horizontalFinPub.Publish(horizMsg);

  gz::msgs::StringMsg statusMsg;
  statusMsg.set_data(thrustPhase + " | dist=" + std::to_string(distance3D) +
    " | yaw_err=" + std::to_string(yawError * 180.0 / M_PI) + "°");
  this->dataPtr->statusPub.Publish(statusMsg);
}

} // namespace tethys

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)