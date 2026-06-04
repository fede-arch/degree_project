/*
 * Navigation plugin for Tethys AUV
 * LOS guidance + PID controller - 3D
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
  public: double arrivalThreshold {1.5};
  public: double maxThrust {31.0};
  public: double maxFin {0.261799};
  public: double cruiseThrust {15.0};

  // PID yaw
  public: double kpYaw {1.5};
  public: double kiYaw {0.01};
  public: double kdYaw {0.3};
  public: double yawIntegral {0.0};
  public: double prevYawError {0.0};

  // PID pitch
  public: double kpPitch {1.5};
  public: double kiPitch {0.01};
  public: double kdPitch {0.3};
  public: double pitchIntegral {0.0};
  public: double prevPitchError {0.0};

  public: double integralMax {0.5};

  public: gz::sim::Entity linkEntity;
  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher thrustPub;
  public: gz::transport::Node::Publisher verticalFinPub;
  public: gz::transport::Node::Publisher horizontalFinPub;
  public: gz::transport::Node::Publisher statusPub;
  public: std::mutex mtx;

  public: void OnTarget(const gz::msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->target = gz::msgs::Convert(_msg);
    this->hasTarget = true;
    this->yawIntegral = 0.0;
    this->prevYawError = 0.0;
    this->pitchIntegral = 0.0;
    this->prevPitchError = 0.0;
    gzmsg << "[NavigationPlugin] New target: ("
          << this->target.X() << ", "
          << this->target.Y() << ", "
          << this->target.Z() << ")\n";
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

  if (_sdf->HasElement("arrival_threshold"))
    this->dataPtr->arrivalThreshold = _sdf->Get<double>("arrival_threshold");
  if (_sdf->HasElement("cruise_thrust"))
    this->dataPtr->cruiseThrust = _sdf->Get<double>("cruise_thrust");
  if (_sdf->HasElement("max_thrust"))
    this->dataPtr->maxThrust = _sdf->Get<double>("max_thrust");
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
  // Load initial target from SDF if present
    if (_sdf->HasElement("initial_target_x") &&
        _sdf->HasElement("initial_target_y") &&
        _sdf->HasElement("initial_target_z"))
    {
    this->dataPtr->target.X(_sdf->Get<double>("initial_target_x"));
    this->dataPtr->target.Y(_sdf->Get<double>("initial_target_y"));
    this->dataPtr->target.Z(_sdf->Get<double>("initial_target_z"));
    this->dataPtr->hasTarget = true;
    gzmsg << "[NavigationPlugin] Initial target from SDF: ("
            << this->dataPtr->target.X() << ", "
            << this->dataPtr->target.Y() << ", "
            << this->dataPtr->target.Z() << ")\n";
    }  

  auto linkName = _sdf->Get<std::string>("link_name", "base_link").first;
  this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);

  if (gz::sim::kNullEntity == this->dataPtr->linkEntity)
  {
    gzerr << "[NavigationPlugin] Failed to find link: " << linkName << "\n";
    return;
  }

  if (!_ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->linkEntity))
    _ecm.CreateComponent(this->dataPtr->linkEntity, gz::sim::components::WorldPose());

  auto ns = _sdf->Get<std::string>("namespace", model.Name(_ecm)).first;

  this->dataPtr->thrustPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
    "/model/" + ns + "/joint/propeller_joint/cmd_thrust");
  this->dataPtr->verticalFinPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
    "/model/" + ns + "/joint/vertical_fins_joint/0/cmd_pos");
  this->dataPtr->horizontalFinPub = this->dataPtr->node.Advertise<gz::msgs::Double>(
    "/model/" + ns + "/joint/horizontal_fins_joint/0/cmd_pos");
  this->dataPtr->statusPub = this->dataPtr->node.Advertise<gz::msgs::StringMsg>(
    "/model/" + ns + "/navigation/status");

  this->dataPtr->node.Subscribe(
    "/model/" + ns + "/navigation/target",
    &NavigationPrivateData::OnTarget,
    this->dataPtr.get());

  gzmsg << "[NavigationPlugin] Ready. LOS+PID 3D controller active.\n";
}

void NavigationPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);

  if (!this->dataPtr->hasTarget)
    return;

  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto pose = baseLink.WorldPose(_ecm);
  if (!pose)
    return;

  auto pos = pose->Pos();
  auto rot = pose->Rot();

  double dx = this->dataPtr->target.X() - pos.X();
  double dy = this->dataPtr->target.Y() - pos.Y();
  double dz = this->dataPtr->target.Z() - pos.Z();

  double horizDist = std::sqrt(dx*dx + dy*dy);
  double distance3D = std::sqrt(dx*dx + dy*dy + dz*dz);

  // Check arrival
  if (distance3D < this->dataPtr->arrivalThreshold)
  {
    gzmsg << "[NavigationPlugin] Target reached!\n";
    this->dataPtr->hasTarget = false;

    gz::msgs::Double stopMsg;
    stopMsg.set_data(0.0);
    this->dataPtr->thrustPub.Publish(stopMsg);
    this->dataPtr->verticalFinPub.Publish(stopMsg);
    this->dataPtr->horizontalFinPub.Publish(stopMsg);

    gz::msgs::StringMsg statusMsg;
    statusMsg.set_data("ARRIVED");
    this->dataPtr->statusPub.Publish(statusMsg);
    return;
  }

  double dt = (double)_info.dt.count() / 1e9;
  if (dt <= 0.0) dt = 0.001;

  // ── LOS Yaw ──
  double desiredYaw = std::atan2(dy, dx);
  double currentYaw = rot.Yaw();
  double yawError = desiredYaw - currentYaw;
  while (yawError >  M_PI) yawError -= 2.0 * M_PI;
  while (yawError < -M_PI) yawError += 2.0 * M_PI;

  // PID Yaw
  this->dataPtr->yawIntegral += yawError * dt;
  this->dataPtr->yawIntegral = std::max(-this->dataPtr->integralMax,
    std::min(this->dataPtr->integralMax, this->dataPtr->yawIntegral));
  double yawDerivative = (yawError - this->dataPtr->prevYawError) / dt;
  this->dataPtr->prevYawError = yawError;

  double yawPID = this->dataPtr->kpYaw * yawError
                + this->dataPtr->kiYaw * this->dataPtr->yawIntegral
                + this->dataPtr->kdYaw * yawDerivative;

  double vertFin = std::max(-this->dataPtr->maxFin,
    std::min(this->dataPtr->maxFin, -yawPID));

  // ── LOS Pitch ──
  double desiredPitch = std::atan2(-dz, horizDist);
  double currentPitch = rot.Pitch();
  double pitchError = desiredPitch - currentPitch;
  while (pitchError >  M_PI) pitchError -= 2.0 * M_PI;
  while (pitchError < -M_PI) pitchError += 2.0 * M_PI;

  // PID Pitch
  this->dataPtr->pitchIntegral += pitchError * dt;
  this->dataPtr->pitchIntegral = std::max(-this->dataPtr->integralMax,
    std::min(this->dataPtr->integralMax, this->dataPtr->pitchIntegral));
  double pitchDerivative = (pitchError - this->dataPtr->prevPitchError) / dt;
  this->dataPtr->prevPitchError = pitchError;

  double pitchPID = this->dataPtr->kpPitch * pitchError
                  + this->dataPtr->kiPitch * this->dataPtr->pitchIntegral
                  + this->dataPtr->kdPitch * pitchDerivative;

  double horizFin = std::max(-this->dataPtr->maxFin,
    std::min(this->dataPtr->maxFin, pitchPID));

  // ── Thrust with brake zone ──
  double brakeDist = 8.0;
  double thrust = -std::min(
    this->dataPtr->cruiseThrust * std::min(distance3D / brakeDist, 1.0),
    this->dataPtr->maxThrust);

  // ── Publish ──
  gz::msgs::Double thrustMsg;
  thrustMsg.set_data(thrust);
  this->dataPtr->thrustPub.Publish(thrustMsg);

  gz::msgs::Double vertMsg;
  vertMsg.set_data(vertFin);
  this->dataPtr->verticalFinPub.Publish(vertMsg);

  gz::msgs::Double horizMsg;
  horizMsg.set_data(horizFin);
  this->dataPtr->horizontalFinPub.Publish(horizMsg);

  gz::msgs::StringMsg statusMsg;
  statusMsg.set_data("NAVIGATING 3D dist=" + std::to_string(distance3D)
    + " yaw_err=" + std::to_string(yawError)
    + " pitch_err=" + std::to_string(pitchError));
  this->dataPtr->statusPub.Publish(statusMsg);
}

} // namespace tethys

GZ_ADD_PLUGIN(
  tethys::NavigationPlugin,
  gz::sim::System,
  tethys::NavigationPlugin::ISystemConfigure,
  tethys::NavigationPlugin::ISystemPreUpdate)
