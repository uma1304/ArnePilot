using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xbfa7e645486440c7;

# dp
struct DragonConf {
  dpThermalStarted @0 :Bool;
  dpThermalOverheat @1 :Bool;
  dpAtl @2 :Bool;
  dpAtlOpLong @3 :Bool;
  dpDashcamd @4 :Bool;
  dpAutoShutdown @5 :Bool;
  dpAthenad @6 :Bool;
  dpUploader @7 :Bool;
  dpLateralMode @8 :UInt8;
  dpSignalOffDelay @9 :Float32;
  dpLcMinMph @10 :UInt8;
  dpLcAutoMinMph @11 :UInt8;
  dpLcAutoDelay @12 :Float32;
  dpLaneLessModeCtrl @13 :Bool;
  dpLaneLessMode @14 :UInt8;
  dpAllowGas @15 :Bool;
  dpAccelProfileCtrl @16 :Bool;
  dpAccelProfile @17 :UInt8;
  dpGearCheck @18 :Bool;
  dpSpeedCheck @19 :Bool;
  dpUiDisplayMode @20 :UInt8;
  dpUiSpeed @21 :Bool;
  dpUiEvent @22 :Bool;
  dpUiMaxSpeed @23 :Bool;
  dpUiFace @24 :Bool;
  dpUiLane @25 :Bool;
  dpUiLead @26 :Bool;
  dpUiSide @27 :Bool;
  dpUiTop @28 :Bool;
  dpUiBlinker @29 :Bool;
  dpUiBrightness @30 :UInt8;
  dpUiVolume @31 :Int8;
  dpToyotaLdw @32 :Bool;
  dpToyotaSng @33 :Bool;
  dpToyotaCruiseOverride @34 :Bool;
  dpToyotaCruiseOverrideVego @35 :Bool;
  dpToyotaCruiseOverrideAt @36 :Float32;
  dpToyotaCruiseOverrideSpeed @37 :Float32;
  dpIpAddr @38 :Text;
  dpCameraOffset @39 :Int8;
  dpPathOffset @40 :Int8;
  dpLocale @41 :Text;
  dpSrLearner @42 :Bool;
  dpSrCustom @43 :Float32;
  dpMapd @44 :Bool;
}