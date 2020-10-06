#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <poll.h>
#include <sys/mman.h>
#include <string>
#include <sstream>
#include <sys/resource.h>
#include <czmq.h>
#include "json11.hpp"
#include <fstream>
#include "common/util.h"
#include "common/swaglog.h"
#include "common/visionimg.h"
#include "common/utilpp.h"
#include "ui.hpp"
#include "cereal/gen/cpp/arne182.capnp.h"
#include "dashcam.h"

#include "paint.hpp"

std::map<std::string, int> DF_TO_IDX = {{"close", 0}, {"normal", 1}, {"far", 2}, {"auto", 3}};

static void ui_set_brightness(UIState *s, int brightness) {
  static int last_brightness = -1;
  if (last_brightness != brightness && (s->awake || brightness == 0)) {
    if (set_brightness(brightness)) {
      last_brightness = brightness;
    }
  }
}

int event_processing_enabled = -1;
static void enable_event_processing(bool yes) {
  if (event_processing_enabled != 1 && yes) {
    system("service call window 18 i32 1");  // enable event processing
    event_processing_enabled = 1;
  } else if (event_processing_enabled != 0 && !yes) {
    system("service call window 18 i32 0");  // disable event processing
    event_processing_enabled = 0;
  }
}

static void set_awake(UIState *s, bool awake) {
#ifdef QCOM
  if (awake) {
    // 30 second timeout
    s->awake_timeout = 30*UI_FREQ;
  }
  if (s->awake != awake) {
    s->awake = awake;

    // TODO: replace command_awake and command_sleep with direct calls to android
    if (awake) {
      LOGW("awake normal");
      framebuffer_set_power(s->fb, HWC_POWER_MODE_NORMAL);
      enable_event_processing(true);
    } else {
      LOGW("awake off");
      ui_set_brightness(s, 0);
      framebuffer_set_power(s->fb, HWC_POWER_MODE_OFF);
      enable_event_processing(false);
    }
  }
#else
  // computer UI doesn't sleep
  s->awake = true;
#endif
}

static void update_offroad_layout_state(UIState *s) {
#ifdef QCOM
  static int timeout = 0;
  static bool prev_collapsed = false;
  static cereal::UiLayoutState::App prev_app = cereal::UiLayoutState::App::NONE;
  if (timeout > 0) {
    timeout--;
  }
  if (prev_collapsed != s->scene.uilayout_sidebarcollapsed || prev_app != s->active_app || timeout == 0) {
    capnp::MallocMessageBuilder msg;
    auto event = msg.initRoot<cereal::Event>();
    event.setLogMonoTime(nanos_since_boot());
    auto layout = event.initUiLayoutState();
    layout.setActiveApp(s->active_app);
    layout.setSidebarCollapsed(s->scene.uilayout_sidebarcollapsed);
    s->pm->send("offroadLayout", msg);
    LOGD("setting active app to %d with sidebar %d", (int)s->active_app, s->scene.uilayout_sidebarcollapsed);
    prev_collapsed = s->scene.uilayout_sidebarcollapsed;
    prev_app = s->active_app;
    timeout = 2 * UI_FREQ;
  }
#endif
}

// e2e model button.
static void send_ml(UIState *s, bool enabled) {
  capnp::MallocMessageBuilder msg;
  auto EventArne182 = msg.initRoot<cereal::EventArne182>();
  EventArne182.setLogMonoTime(nanos_since_boot());
  auto mlStatus = EventArne182.initModelLongButton();
  mlStatus.setEnabled(enabled);
  s->pm->send("modelLongButton", msg);
}

static bool handle_ml_touch(UIState *s, int touch_x, int touch_y) {
  //mlButton manager
  if ((s->awake && s->vision_connected && s->status != STATUS_STOPPED)) {
    int padding = 40;
    int btn_w = 400;
    int btn_h = 138;
    int xs[2] = {1920 / 2 + 75 - btn_w / 2, 1920 / 2 + 75 + btn_w / 2};
    int y_top = 915 - btn_h / 2;
    if (xs[0] <= touch_x + padding && touch_x - padding <= xs[1] && y_top - padding <= touch_y) {
      s->scene.mlButtonEnabled = !s->scene.mlButtonEnabled;
      send_ml(s, s->scene.mlButtonEnabled);
      return true;
    }
  }
    return false;
}

//dfButton manager
static void send_df(UIState *s, int status) {
  capnp::MallocMessageBuilder msg;
  auto EventArne182 = msg.initRoot<cereal::EventArne182>();
  EventArne182.setLogMonoTime(nanos_since_boot());
  auto dfStatus = EventArne182.initDynamicFollowButton();
  dfStatus.setStatus(status);
  s->pm->send("dynamicFollowButton", msg);
}

static bool handle_df_touch(UIState *s, int touch_x, int touch_y) {
  if (s->awake && s->vision_connected && s->status != STATUS_STOPPED) {
    int padding = 40;
    int btn_x_1 = 1660 - 200 - 175;
    int btn_x_2 = 1660 - 50 - 175;
    if ((btn_x_1 - padding <= touch_x) && (touch_x <= btn_x_2 + padding) && (855 - padding <= touch_y)) {
      s->scene.uilayout_sidebarcollapsed = true;  // collapse sidebar when tapping df button
      s->scene.dfButtonStatus++;
      if (s->scene.dfButtonStatus > 3) {
        s->scene.dfButtonStatus = 0;
      }
      send_df(s, s->scene.dfButtonStatus);
      return true;
    }
  }
  return false;
}

static void handle_sidebar_touch(UIState *s, int touch_x, int touch_y) {
  if (!s->scene.uilayout_sidebarcollapsed && touch_x <= sbr_w) {
    if (touch_x >= settings_btn_x && touch_x < (settings_btn_x + settings_btn_w)
      && touch_y >= settings_btn_y && touch_y < (settings_btn_y + settings_btn_h)) {
      s->active_app = cereal::UiLayoutState::App::SETTINGS;
    }
    else if (touch_x >= home_btn_x && touch_x < (home_btn_x + home_btn_w)
      && touch_y >= home_btn_y && touch_y < (home_btn_y + home_btn_h)) {
      if (s->started) {
        s->active_app = cereal::UiLayoutState::App::NONE;
        s->scene.uilayout_sidebarcollapsed = true;
      } else {
        s->active_app = cereal::UiLayoutState::App::HOME;
      }
    }
  }
}

static void handle_vision_touch(UIState *s, int touch_x, int touch_y) {
  if (s->started && (touch_x >= s->scene.ui_viz_rx - bdr_s)
    && (s->active_app != cereal::UiLayoutState::App::SETTINGS)) {
    if (!s->scene.frontview) {
      s->scene.uilayout_sidebarcollapsed = !s->scene.uilayout_sidebarcollapsed;
    } else {
      write_db_value("IsDriverViewEnabled", "0", 1);
    }
  }
}

volatile sig_atomic_t do_exit = 0;
static void set_do_exit(int sig) {
  do_exit = 1;
}

template <class T>
static int read_param(T* param, const char *param_name, bool persistent_param = false){
  T param_orig = *param;
  char *value;
  size_t sz;

  int result = read_db_value(param_name, &value, &sz, persistent_param);
  if (result == 0){
    std::string s = std::string(value, sz); // value is not null terminated
    free(value);

    // Parse result
    std::istringstream iss(s);
    iss >> *param;

    // Restore original value if parsing failed
    if (iss.fail()) {
      *param = param_orig;
      result = -1;
    }
  }
  return result;
}

template <class T>
static int read_param_timeout(T* param, const char* param_name, int* timeout, bool persistent_param = false) {
  int result = -1;
  if (*timeout > 0){
    (*timeout)--;
  } else {
    *timeout = 2 * UI_FREQ; // 0.5Hz
    result = read_param(param, param_name, persistent_param);
  }
  return result;
}

extern volatile sig_atomic_t do_exit;

int write_param_float(float param, const char* param_name, bool persistent_param) {
  char s[16];
  int size = snprintf(s, sizeof(s), "%f", param);
  return write_db_value(param_name, s, size < sizeof(s) ? size : sizeof(s), persistent_param);
}

void ui_init(UIState *s) {
  s->sm = new SubMaster({"model", "controlsState", "uiLayoutState", "liveCalibration", "radarState", "thermal",
                         "health", "carParams", "ubloxGnss", "driverState", "dMonitoringState", "sensorEvents", "carState", "gpsLocationExternal", "liveMpc"
#ifdef SHOW_SPEEDLIMIT
                                    , "liveMapData"
#endif
  });
  s->pm = new PubMaster({"offroadLayout", "dynamicFollowButton", "modelLongButton"});

  s->started = false;
  s->status = STATUS_OFFROAD;
  s->scene.satelliteCount = -1;
  read_param(&s->is_metric, "IsMetric");

  s->fb = framebuffer_init("ui", 0, true, &s->fb_w, &s->fb_h);
  assert(s->fb);

  s->livempc_or_radarstate_changed = false;

  set_awake(s, true);

  ui_nvg_init(s);
}

static void ui_init_vision(UIState *s, const VisionStreamBufs back_bufs,
                           int num_back_fds, const int *back_fds,
                           const VisionStreamBufs front_bufs, int num_front_fds,
                           const int *front_fds) {
  const VisionUIInfo ui_info = back_bufs.buf_info.ui_info;

  assert(num_back_fds == UI_BUF_COUNT);
  assert(num_front_fds == UI_BUF_COUNT);

  vipc_bufs_load(s->bufs, &back_bufs, num_back_fds, back_fds);
  vipc_bufs_load(s->front_bufs, &front_bufs, num_front_fds, front_fds);

  s->cur_vision_idx = -1;
  s->cur_vision_front_idx = -1;

  s->scene.frontview = getenv("FRONTVIEW") != NULL;
  s->scene.fullview = getenv("FULLVIEW") != NULL;
  s->scene.transformed_width = ui_info.transformed_width;
  s->scene.transformed_height = ui_info.transformed_height;
  s->scene.front_box_x = ui_info.front_box_x;
  s->scene.front_box_y = ui_info.front_box_y;
  s->scene.front_box_width = ui_info.front_box_width;
  s->scene.front_box_height = ui_info.front_box_height;
  s->scene.world_objects_visible = false;  // Invisible until we receive a calibration message.
  s->scene.gps_planner_active = false;

  // Dynamic Follow
  // from shane's todo: run opparams first (in main()?) to ensure json values exist https://github.com/ShaneSmiskol/openpilot/commit/90ded3f5689daf4f3622973dfd4c63ba369573fd#diff-f7a9e034709fadfe2b5065cde2766598
 std::ifstream op_params_file("/data/op_params.json");
 std::string op_params_content((std::istreambuf_iterator<char>(op_params_file)),
                               (std::istreambuf_iterator<char>()));

 std::string err;
 auto json = json11::Json::parse(op_params_content, err);
 if (!json.is_null() && err.empty()) {
   printf("successfully parsed opParams json\n");
   s->scene.dfButtonStatus = DF_TO_IDX[json["dynamic_follow"].string_value()];
//    printf("dfButtonStatus: %d\n", s->scene.dfButtonStatus);
 } else {  // error parsing json
   printf("ERROR PARSING OPPARAMS JSON!\n");
   s->scene.dfButtonStatus = 0;
 }
  s->scene.mlButtonEnabled = false;

  s->rgb_width = back_bufs.width;
  s->rgb_height = back_bufs.height;
  s->rgb_stride = back_bufs.stride;
  s->rgb_buf_len = back_bufs.buf_len;

  s->rgb_front_width = front_bufs.width;
  s->rgb_front_height = front_bufs.height;
  s->rgb_front_stride = front_bufs.stride;
  s->rgb_front_buf_len = front_bufs.buf_len;

  s->rgb_transform = (mat4){{
    2.0f/s->rgb_width, 0.0f, 0.0f, -1.0f,
    0.0f, 2.0f/s->rgb_height, 0.0f, -1.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f,
  }};

  read_param(&s->speed_lim_off, "SpeedLimitOffset");
  read_param(&s->is_metric, "IsMetric");
  read_param(&s->longitudinal_control, "LongitudinalControl");
  read_param(&s->limit_set_speed, "LimitSetSpeed");

  // Set offsets so params don't get read at the same time
  s->longitudinal_control_timeout = UI_FREQ / 3;
  s->is_metric_timeout = UI_FREQ / 2;
  s->limit_set_speed_timeout = UI_FREQ;
  s->dev_bbui_timeout = UI_FREQ / 4;
}

    VisionImg img = {
      .fd = s->stream.bufs[i].fd,
      .format = VISIONIMG_FORMAT_RGB24,
      .width = s->stream.bufs_info.width,
      .height = s->stream.bufs_info.height,
      .stride = s->stream.bufs_info.stride,
      .bpp = 3,
      .size = s->stream.bufs_info.buf_len,
    };
#ifndef QCOM
    s->priv_hnds[i] = s->stream.bufs[i].addr;
#endif
    s->frame_texs[i] = visionimg_to_gl(&img, &s->khr[i], &s->priv_hnds[i]);

    glBindTexture(GL_TEXTURE_2D, s->frame_texs[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // BGR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
  }
  assert(glGetError() == GL_NO_ERROR);
}

void ui_update_vision(UIState *s) {

  if (!s->vision_connected && s->started) {
    const VisionStreamType type = s->scene.frontview ? VISION_STREAM_RGB_FRONT : VISION_STREAM_RGB_BACK;
    int err = visionstream_init(&s->stream, type, true, nullptr);
    if (err == 0) {
      ui_init_vision(s);
      s->vision_connected = true;
    }
  }

  if (s->vision_connected) {
    if (!s->started) goto destroy;

    // poll for a new frame
    struct pollfd fds[1] = {{
      .fd = s->stream.ipc_fd,
      .events = POLLOUT,
    }};
    int ret = poll(fds, 1, 100);
    if (ret > 0) {
      if (!visionstream_get(&s->stream, nullptr)) goto destroy;
    }
  }

  return;

destroy:
  visionstream_destroy(&s->stream);
  s->vision_connected = false;
}

static inline void fill_path_points(const cereal::ModelData::PathData::Reader &path, float *points) {
  const capnp::List<float>::Reader &poly = path.getPoly();
  for (int i = 0; i < path.getValidLen(); i++) {
    points[i] = poly[0] * (i * i * i) + poly[1] * (i * i) + poly[2] * i + poly[3];
  }
}

void update_sockets(UIState *s) {

  UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  if (sm.update(0) == 0){
    return;
  }

  if (s->started && sm.updated("controlsState")) {
    auto event = sm["controlsState"];
    auto data = event.getControlsState();
    auto datad = data.getLateralControlState();
    auto pdata = datad.getPidState();
    auto qdata = datad.getLqrState();
    auto rdata = datad.getIndiState();
    scene.controls_state = event.getControlsState();

    // TODO: the alert stuff shouldn't be handled here
    auto alert_sound = scene.controls_state.getAlertSound();
    if (scene.alert_type.compare(scene.controls_state.getAlertType()) != 0) {
      if (alert_sound == AudibleAlert::NONE) {
        s->sound->stop();
      } else {
        s->sound->play(alert_sound);
      }
    }
    scene.alert_text1 = scene.controls_state.getAlertText1();
    scene.alert_text2 = scene.controls_state.getAlertText2();
    scene.alert_size = scene.controls_state.getAlertSize();
    scene.alert_type = scene.controls_state.getAlertType();
    auto alertStatus = scene.controls_state.getAlertStatus();
    if (alertStatus == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      s->status = STATUS_WARNING;
    } else if (alertStatus == cereal::ControlsState::AlertStatus::CRITICAL) {
      s->status = STATUS_ALERT;
    } else{
      s->status = scene.controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
    }

    float alert_blinkingrate = scene.controls_state.getAlertBlinkingRate();
    if (alert_blinkingrate > 0.) {
      if (s->alert_blinked) {
        if (s->alert_blinking_alpha > 0.0 && s->alert_blinking_alpha < 1.0) {
          s->alert_blinking_alpha += (0.05*alert_blinkingrate);
        } else {
          s->alert_blinked = false;
        }
      } else {
        if (s->alert_blinking_alpha > 0.25) {
          s->alert_blinking_alpha -= (0.05*alert_blinkingrate);
        } else {
          s->alert_blinking_alpha += 0.25;
          s->alert_blinked = true;
        }
      }
    }
    scene.steerOverride = data.getSteerOverride();
    float q = qdata.getOutput();
    float r = rdata.getOutput();
    float p = pdata.getOutput();
    scene.output_scale = q + p + r;
    scene.angleSteers = data.getAngleSteers();
    scene.angleSteersDes = data.getAngleSteersDes();
  }
  if (sm.updated("radarState")) {
    auto data = sm["radarState"].getRadarState();
    scene.lead_data[0] = data.getLeadOne();
    scene.lead_status = data.getLeadOne().getStatus();
    scene.lead_d_rel = data.getLeadOne().getDRel();
    //scene.lead_y_rel = scene.lead_data[0].getYRel();
    scene.lead_v_rel = data.getLeadOne().getVRel();
    scene.lead_data[1] = data.getLeadTwo();
    //scene.lead_status2 = scene.lead_data[1].getStatus;
    //scene.lead_d_rel2 = scene.lead_data[1].getDRel;
    //scene.lead_y_rel2 = scene.lead_data[1].getYRel();
    //scene.lead_v_rel2 = scene.lead_data[1].getVRel();
  }
  if (sm.updated("liveCalibration")) {
    scene.world_objects_visible = true;
    auto extrinsicl = sm["liveCalibration"].getLiveCalibration().getExtrinsicMatrix();
    for (int i = 0; i < 3 * 4; i++) {
      scene.extrinsic_matrix.v[i] = extrinsicl[i];
    }
  }
  if (sm.updated("model")) {
    scene.model = sm["model"].getModel();
    fill_path_points(scene.model.getPath(), scene.path_points);
    fill_path_points(scene.model.getLeftLane(), scene.left_lane_points);
    fill_path_points(scene.model.getRightLane(), scene.right_lane_points);
  }
  if (sm.updated("liveMpc")) {
     auto data = sm["liveMpc"].getLiveMpc();
     auto x_list = data.getX();
     auto y_list = data.getY();
     for (int i = 0; i < 50; i++){
       scene.mpc_x[i] = x_list[i];
       scene.mpc_y[i] = y_list[i];
     }
     s->livempc_or_radarstate_changed = true;
  }
  if (sm.updated("uiLayoutState")) {
    auto data = sm["uiLayoutState"].getUiLayoutState();
    s->active_app = data.getActiveApp();
    scene.uilayout_sidebarcollapsed = data.getSidebarCollapsed();
  }
  if (sm.updated("gpsLocationExternal")) {
    scene.gpsAccuracy = sm["gpsLocationExternal"].getGpsLocationExternal().getAccuracy();
    if (scene.gpsAccuracy > 100)
    {
      scene.gpsAccuracy = 99.99;
    }
    else if (scene.gpsAccuracy == 0)
    {
      scene.gpsAccuracy = 99.8;
    }
  }

#ifdef SHOW_SPEEDLIMIT
  if (sm.updated("liveMapData")) {
    scene.map_valid = sm["liveMapData"].getLiveMapData().getMapValid();
    scene.speedlimit = sm["liveMapData"].getLiveMapData().getSpeedLimit();
    scene.speedlimit_valid = sm["liveMapData"].getLiveMapData().getSpeedLimitValid();
    scene.speedlimitahead_valid = sm["liveMapData"].getLiveMapData().getSpeedLimitAheadValid();
    scene.speedlimitaheaddistance = sm["liveMapData"].getLiveMapData().getSpeedLimitAheadDistance();
  }
#endif
  if (sm.updated("thermal")) {
    scene.thermal = sm["thermal"].getThermal();
    scene.pa0 = scene.thermal.getPa0();
    scene.freeSpace = scene.thermal.getFreeSpace();
    scene.ipAddr = scene.thermal.getIpAddr();
  }
  if (sm.updated("ubloxGnss")) {
    auto data = sm["ubloxGnss"].getUbloxGnss();
    if (data.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
      scene.satelliteCount = data.getMeasurementReport().getNumMeas();
    }
  }
  if (sm.updated("health")) {
    auto health = sm["health"].getHealth();
    scene.hwType = health.getHwType();
    s->ignition = health.getIgnitionLine() || health.getIgnitionCan();
  } else if ((s->sm->frame - s->sm->rcv_frame("health")) > 5*UI_FREQ) {
    scene.hwType = cereal::HealthData::HwType::UNKNOWN;
  }
  if (sm.updated("carParams")) {
    s->longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
  }
  if (sm.updated("driverState")) {
    scene.driver_state = sm["driverState"].getDriverState();
  }
  if (sm.updated("dMonitoringState")) {
    scene.dmonitoring_state = sm["dMonitoringState"].getDMonitoringState();
    scene.is_rhd = scene.dmonitoring_state.getIsRHD();
    scene.frontview = scene.dmonitoring_state.getIsPreview();
    scene.frontview = scene.dmonitoring_state.getIsPreview();
  } else if ((sm.frame - sm.rcv_frame("dMonitoringState")) > UI_FREQ/2) {
    scene.frontview = false;
  }

  #ifdef QCOM2 // TODO: use this for QCOM too
    if (sm.updated("sensorEvents")) {
      for (auto sensor : sm["sensorEvents"].getSensorEvents()) {
        if (sensor.which() == cereal::SensorEventData::LIGHT) {
          s->light_sensor = sensor.getLight();
        }
  //dev ui
  //snprintf(scene.ipAddr, sizeof(s->scene.ipAddr), "%s", data.ipAddr.str);
  if (sm.updated("carState")) {
    auto data = sm["carState"].getCarState();
    if(scene.leftBlinker!=data.getLeftBlinker() || scene.rightBlinker!=data.getRightBlinker()){
      scene.blinker_blinkingrate = 100;
    }
    scene.brakeLights = data.getBrakeLights();
    scene.leftBlinker = data.getLeftBlinker();
    scene.rightBlinker = data.getRightBlinker();
    scene.leftblindspot = data.getLeftBlindspot();
    scene.rightblindspot = data.getRightBlindspot();
    scene.gear = data.getGearShifter();
  }

  s->started = scene.thermal.getStarted() || s->preview_started;
  // Handle onroad/offroad transition
  if (!s->started) {
    if (s->status != STATUS_STOPPED) {
      update_status(s, STATUS_STOPPED);
      s->vision_seen = false;
      s->controls_seen = false;
      s->active_app = cereal::UiLayoutState::App::HOME;

      #ifndef QCOM
      // disconnect from visionipc on PC
      close(s->ipc_fd);
      s->ipc_fd = -1;
      #endif
    }
  }
#endif

  s->started = scene.thermal.getStarted() || scene.frontview;
}

void ui_update(UIState *s) {

  update_sockets(s);
  ui_update_vision(s);

  // Handle onroad/offroad transition
  if (!s->started && s->status != STATUS_OFFROAD) {
    s->status = STATUS_OFFROAD;
    s->active_app = cereal::UiLayoutState::App::HOME;
    s->scene.uilayout_sidebarcollapsed = false;
  } else if (s->started && s->status == STATUS_OFFROAD) {
    s->status = STATUS_DISENGAGED;
    s->started_frame = s->sm->frame;

    s->active_app = cereal::UiLayoutState::App::NONE;
    s->scene.uilayout_sidebarcollapsed = true;
    s->alert_blinked = false;
    s->alert_blinking_alpha = 1.0;
    s->scene.alert_size = cereal::ControlsState::AlertSize::NONE;
  }

  // Handle controls timeout
  if (s->started && !s->scene.frontview && ((s->sm)->frame - s->started_frame) > 5*UI_FREQ) {
    if ((s->sm)->rcv_frame("controlsState") < s->started_frame) {
      // car is started, but controlsState hasn't been seen at all
      s->scene.alert_text1 = "openpilot Unavailable";
      s->scene.alert_text2 = "Waiting for controls to start";
      s->scene.alert_size = cereal::ControlsState::AlertSize::MID;
    } else if (((s->sm)->frame - (s->sm)->rcv_frame("controlsState")) > 5*UI_FREQ) {
      // car is started, but controls is lagging or died
      if (s->scene.alert_text2 != "Controls Unresponsive") {
        s->sound->play(AudibleAlert::CHIME_WARNING_REPEAT);
        LOGE("Controls unresponsive");
      }

      s->scene.alert_text1 = "TAKE CONTROL IMMEDIATELY";
      s->scene.alert_text2 = "Controls Unresponsive";
      s->scene.alert_size = cereal::ControlsState::AlertSize::FULL;
      s->status = STATUS_ALERT;
    }
  }
  sensors_close(device);
  return NULL;

fail:
  LOGE("LIGHT SENSOR IS MISSING");
  s->light_sensor = 255;
  return NULL;
}

#endif

int main(int argc, char* argv[]) {
  int err;
  setpriority(PRIO_PROCESS, 0, -14);

  zsys_handler_set(NULL);
  signal(SIGINT, (sighandler_t)set_do_exit);

  UIState uistate = {};
  UIState *s = &uistate;
  ui_init(s);

  enable_event_processing(true);

  pthread_t connect_thread_handle;
  err = pthread_create(&connect_thread_handle, NULL,
                       vision_connect_thread, s);
  assert(err == 0);

#ifdef QCOM
  pthread_t light_sensor_thread_handle;
  err = pthread_create(&light_sensor_thread_handle, NULL,
                       light_sensor_thread, s);
  assert(err == 0);
#endif

  TouchState touch = {0};
  touch_init(&touch);
  s->touch_fd = touch.fd;

  // light sensor scaling params
  const bool LEON = util::read_file("/proc/cmdline").find("letv") != std::string::npos;

  float brightness_b, brightness_m;
  int result = read_param(&brightness_b, "BRIGHTNESS_B", true);
  result += read_param(&brightness_m, "BRIGHTNESS_M", true);

  if(result != 0){
    brightness_b = LEON ? 10.0 : 5.0;
    brightness_m = LEON ? 2.6 : 1.3;
    write_param_float(brightness_b, "BRIGHTNESS_B", true);
    write_param_float(brightness_m, "BRIGHTNESS_M", true);
  }

  float smooth_brightness = brightness_b;

  const int MIN_VOLUME = LEON ? 12 : 9;
  const int MAX_VOLUME = LEON ? 15 : 12;
  assert(s->sound.init(MIN_VOLUME));

  int draws = 0;

  while (!do_exit) {
    bool should_swap = false;
    if (!s->started) {
      // Delay a while to avoid 9% cpu usage while car is not started and user is keeping touching on the screen.
      // Don't hold the lock while sleeping, so that vision_connect_thread have chances to get the lock.
      usleep(30 * 1000);
    }
    pthread_mutex_lock(&s->lock);
    double u1 = millis_since_boot();

    // light sensor is only exposed on EONs
    float clipped_brightness = (s->light_sensor*brightness_m) + brightness_b;
    if (clipped_brightness > 512) clipped_brightness = 512;
    smooth_brightness = clipped_brightness * 0.01 + smooth_brightness * 0.99;
    if (smooth_brightness > 255) smooth_brightness = 255;
    ui_set_brightness(s, (int)smooth_brightness);

    // resize vision for collapsing sidebar
    const bool hasSidebar = !s->scene.uilayout_sidebarcollapsed;
    s->scene.ui_viz_rx = hasSidebar ? box_x : (box_x - sbr_w + (bdr_s * 2));
    s->scene.ui_viz_rw = hasSidebar ? box_w : (box_w + sbr_w - (bdr_s * 2));
    s->scene.ui_viz_ro = hasSidebar ? -(sbr_w - 6 * bdr_s) : 0;

    // poll for touch events
    int touch_x = -1, touch_y = -1;
    int touched = touch_poll(&touch, &touch_x, &touch_y, 0);
    if (touched == 1) {
      set_awake(s, true);
      handle_sidebar_touch(s, touch_x, touch_y);

      if (!handle_df_touch(s, touch_x, touch_y) && !handle_df_touch(s, touch_x, touch_y) && !handle_ml_touch(s, touch_x, touch_y)) {  // disables sidebar from popping out when tapping df or ls button
        handle_vision_touch(s, touch_x, touch_y);
      } else {
      s->scene.uilayout_sidebarcollapsed = true;  // collapse sidebar when tapping any SA button
      }
    }

    if (!s->started) {
      // always process events offroad
      check_messages(s);

      if (s->started) {
        s->controls_timeout = 5 * UI_FREQ;
      }
    } else {
      // blank screen on reverse gear
      if (s->scene.gear == cereal::CarState::GearShifter::REVERSE) {
        set_awake(s, false);
      } else {
        set_awake(s, true);
      }
      // Car started, fetch a new rgb image from ipc
      if (s->vision_connected){
        ui_update(s);
      }

      check_messages(s);

      // Visiond process is just stopped, force a redraw to make screen blank again.
      if (!s->started) {
        s->scene.uilayout_sidebarcollapsed = false;
        ui_draw(s);
        glFinish();
        should_swap = true;
      }
    }

    // manage wakefulness
    if (s->awake_timeout > 0) {
      s->awake_timeout--;
    } else {
      set_awake(s, false);
    }

  // Read params
  if ((s->sm)->frame % (5*UI_FREQ) == 0) {
    read_param(&s->is_metric, "IsMetric");
  } else if ((s->sm)->frame % (6*UI_FREQ) == 0) {
    int param_read = read_param(&s->last_athena_ping, "LastAthenaPingTime");
    if (param_read != 0) { // Failed to read param
      s->scene.athenaStatus = NET_DISCONNECTED;
    } else if (nanos_since_boot() - s->last_athena_ping < 70e9) {
      s->scene.athenaStatus = NET_CONNECTED;
    } else {
      s->scene.athenaStatus = NET_ERROR;
      s->scene.hwType = cereal::HealthData::HwType::UNKNOWN;
    }

    // Don't waste resources on drawing in case screen is off
    if (s->awake) {
      dashcam(s, touch_x, touch_y);
      ui_draw(s);
      glFinish();
      should_swap = true;
    }

    s->sound.setVolume(fmin(MAX_VOLUME, MIN_VOLUME + s->scene.controls_state.getVEgo() / 5)); // up one notch every 5 m/s

    if (s->controls_timeout > 0) {
      s->controls_timeout--;
    } else if (s->started) {
      if (!s->controls_seen) {
        // car is started, but controlsState hasn't been seen at all
        s->scene.alert_text1 = "openpilot Unavailable";
        s->scene.alert_text2 = "Waiting for controls to start";
        s->scene.alert_size = cereal::ControlsState::AlertSize::MID;
      } else {
        // car is started, but controls is lagging or died
        LOGE("Controls unresponsive");

        if (s->scene.alert_text2 != "Controls Unresponsive") {
          s->sound.play(AudibleAlert::CHIME_WARNING_REPEAT);
        }

        s->scene.alert_text1 = "TAKE CONTROL IMMEDIATELY";
        s->scene.alert_text2 = "Controls Unresponsive";
        s->scene.alert_size = cereal::ControlsState::AlertSize::FULL;
        update_status(s, STATUS_ALERT);
      }
      ui_draw_vision_alert(s, s->scene.alert_size, s->status, s->scene.alert_text1.c_str(), s->scene.alert_text2.c_str());
    }

    read_param_timeout(&s->is_metric, "IsMetric", &s->is_metric_timeout);
    read_param_timeout(&s->longitudinal_control, "LongitudinalControl", &s->longitudinal_control_timeout);
    read_param_timeout(&s->limit_set_speed, "LimitSetSpeed", &s->limit_set_speed_timeout);
    read_param_timeout(&s->speed_lim_off, "SpeedLimitOffset", &s->limit_set_speed_timeout);
    read_param_timeout(&s->dev_bbui, "DevBBUI", &s->dev_bbui_timeout);
    int param_read = read_param_timeout(&s->last_athena_ping, "LastAthenaPingTime", &s->last_athena_ping_timeout);
    if (param_read != -1) { // Param was updated this loop
      if (param_read != 0) { // Failed to read param
        s->scene.athenaStatus = NET_DISCONNECTED;
      } else if (nanos_since_boot() - s->last_athena_ping < 70e9) {
        s->scene.athenaStatus = NET_CONNECTED;
      } else {
        s->scene.athenaStatus = NET_ERROR;
      }
    }
    update_offroad_layout_state(s);

    pthread_mutex_unlock(&s->lock);

    // the bg thread needs to be scheduled, so the main thread needs time without the lock
    // safe to do this outside the lock?
    if (should_swap) {
      double u2 = millis_since_boot();
      if (u2-u1 > 66) {
        // warn on sub 15fps
        LOGW("slow frame(%d) time: %.2f", draws, u2-u1);
      }
      draws++;
      framebuffer_swap(s->fb);
    }
  }
}
