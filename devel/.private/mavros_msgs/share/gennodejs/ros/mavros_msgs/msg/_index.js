
"use strict";

let TimesyncStatus = require('./TimesyncStatus.js');
let HilControls = require('./HilControls.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let HilGPS = require('./HilGPS.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let Vibration = require('./Vibration.js');
let RCIn = require('./RCIn.js');
let Param = require('./Param.js');
let Waypoint = require('./Waypoint.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let PositionTarget = require('./PositionTarget.js');
let RadioStatus = require('./RadioStatus.js');
let FileEntry = require('./FileEntry.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let HomePosition = require('./HomePosition.js');
let WaypointList = require('./WaypointList.js');
let State = require('./State.js');
let LogData = require('./LogData.js');
let ParamValue = require('./ParamValue.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let Mavlink = require('./Mavlink.js');
let MountControl = require('./MountControl.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let RTCM = require('./RTCM.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let RCOut = require('./RCOut.js');
let WaypointReached = require('./WaypointReached.js');
let Trajectory = require('./Trajectory.js');
let DebugValue = require('./DebugValue.js');
let BatteryStatus = require('./BatteryStatus.js');
let LandingTarget = require('./LandingTarget.js');
let CommandCode = require('./CommandCode.js');
let LogEntry = require('./LogEntry.js');
let HilSensor = require('./HilSensor.js');
let ManualControl = require('./ManualControl.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let Altitude = require('./Altitude.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let VehicleInfo = require('./VehicleInfo.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let ActuatorControl = require('./ActuatorControl.js');
let Thrust = require('./Thrust.js');
let ExtendedState = require('./ExtendedState.js');
let StatusText = require('./StatusText.js');
let VFR_HUD = require('./VFR_HUD.js');

module.exports = {
  TimesyncStatus: TimesyncStatus,
  HilControls: HilControls,
  ADSBVehicle: ADSBVehicle,
  HilGPS: HilGPS,
  HilActuatorControls: HilActuatorControls,
  Vibration: Vibration,
  RCIn: RCIn,
  Param: Param,
  Waypoint: Waypoint,
  OverrideRCIn: OverrideRCIn,
  PositionTarget: PositionTarget,
  RadioStatus: RadioStatus,
  FileEntry: FileEntry,
  CompanionProcessStatus: CompanionProcessStatus,
  WheelOdomStamped: WheelOdomStamped,
  HomePosition: HomePosition,
  WaypointList: WaypointList,
  State: State,
  LogData: LogData,
  ParamValue: ParamValue,
  GlobalPositionTarget: GlobalPositionTarget,
  Mavlink: Mavlink,
  MountControl: MountControl,
  AttitudeTarget: AttitudeTarget,
  RTCM: RTCM,
  HilStateQuaternion: HilStateQuaternion,
  RCOut: RCOut,
  WaypointReached: WaypointReached,
  Trajectory: Trajectory,
  DebugValue: DebugValue,
  BatteryStatus: BatteryStatus,
  LandingTarget: LandingTarget,
  CommandCode: CommandCode,
  LogEntry: LogEntry,
  HilSensor: HilSensor,
  ManualControl: ManualControl,
  OpticalFlowRad: OpticalFlowRad,
  Altitude: Altitude,
  OnboardComputerStatus: OnboardComputerStatus,
  VehicleInfo: VehicleInfo,
  EstimatorStatus: EstimatorStatus,
  CamIMUStamp: CamIMUStamp,
  ActuatorControl: ActuatorControl,
  Thrust: Thrust,
  ExtendedState: ExtendedState,
  StatusText: StatusText,
  VFR_HUD: VFR_HUD,
};
