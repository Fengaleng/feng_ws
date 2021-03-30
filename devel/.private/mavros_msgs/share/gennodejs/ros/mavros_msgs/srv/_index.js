
"use strict";

let FileOpen = require('./FileOpen.js')
let ParamSet = require('./ParamSet.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let StreamRate = require('./StreamRate.js')
let WaypointClear = require('./WaypointClear.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileMakeDir = require('./FileMakeDir.js')
let FileList = require('./FileList.js')
let WaypointPush = require('./WaypointPush.js')
let LogRequestList = require('./LogRequestList.js')
let FileClose = require('./FileClose.js')
let CommandTOL = require('./CommandTOL.js')
let LogRequestData = require('./LogRequestData.js')
let FileRead = require('./FileRead.js')
let CommandHome = require('./CommandHome.js')
let FileTruncate = require('./FileTruncate.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let MessageInterval = require('./MessageInterval.js')
let CommandBool = require('./CommandBool.js')
let ParamPush = require('./ParamPush.js')
let CommandInt = require('./CommandInt.js')
let CommandLong = require('./CommandLong.js')
let FileRemove = require('./FileRemove.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let SetMavFrame = require('./SetMavFrame.js')
let ParamPull = require('./ParamPull.js')
let FileChecksum = require('./FileChecksum.js')
let MountConfigure = require('./MountConfigure.js')
let FileWrite = require('./FileWrite.js')
let ParamGet = require('./ParamGet.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let WaypointPull = require('./WaypointPull.js')
let SetMode = require('./SetMode.js')
let FileRename = require('./FileRename.js')

module.exports = {
  FileOpen: FileOpen,
  ParamSet: ParamSet,
  CommandTriggerInterval: CommandTriggerInterval,
  StreamRate: StreamRate,
  WaypointClear: WaypointClear,
  LogRequestEnd: LogRequestEnd,
  FileMakeDir: FileMakeDir,
  FileList: FileList,
  WaypointPush: WaypointPush,
  LogRequestList: LogRequestList,
  FileClose: FileClose,
  CommandTOL: CommandTOL,
  LogRequestData: LogRequestData,
  FileRead: FileRead,
  CommandHome: CommandHome,
  FileTruncate: FileTruncate,
  VehicleInfoGet: VehicleInfoGet,
  MessageInterval: MessageInterval,
  CommandBool: CommandBool,
  ParamPush: ParamPush,
  CommandInt: CommandInt,
  CommandLong: CommandLong,
  FileRemove: FileRemove,
  WaypointSetCurrent: WaypointSetCurrent,
  FileRemoveDir: FileRemoveDir,
  CommandVtolTransition: CommandVtolTransition,
  SetMavFrame: SetMavFrame,
  ParamPull: ParamPull,
  FileChecksum: FileChecksum,
  MountConfigure: MountConfigure,
  FileWrite: FileWrite,
  ParamGet: ParamGet,
  CommandTriggerControl: CommandTriggerControl,
  WaypointPull: WaypointPull,
  SetMode: SetMode,
  FileRename: FileRename,
};
