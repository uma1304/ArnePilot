#pragma clang diagnostic ignored "-Wexceptions"
#pragma clang diagnostic ignored "-Wunused"
#include "traffic.h"

//#include <sched.h>

using namespace std;

volatile sig_atomic_t do_exit = 0;

const std::vector<std::string> modelLabels = {"SLOW", "GREEN", "NONE"};
const int numLabels = modelLabels.size();
const double modelRate = 1 / 4.;  // 3 Hz
const bool debug_mode = true;

const int original_shape[3] = {874, 1164, 3};   // global constants
//const int original_size = 874 * 1164 * 3;
//const int cropped_shape[3] = {665, 814, 3};
const int cropped_size = 665 * 814 * 3;

const int horizontal_crop = 175;
const int top_crop = 0;
const int hood_crop = 209;
const double msToSec = 1 / 1000.;  // multiply
const double secToUs = 1e+6;


//int set_realtime_priority(int level) {
//#ifdef __linux__
//
//  long tid = syscall(SYS_gettid);

//  // should match python using chrt
//  struct sched_param sa;
//  memset(&sa, 0, sizeof(sa));
//  sa.sched_priority = level;
//  return sched_setscheduler(tid, SCHED_FIFO, &sa);
//#else
//  return -1;
//#endif
//}

void sendPrediction(float *output, PubMaster &pm) {
  MessageBuilder msg;
  auto traffic_lights = msg.initEvent().initTrafficModelRaw();

  kj::ArrayPtr<const float> output_vs(&output[0], numLabels);
  traffic_lights.setPrediction(output_vs);
  pm.send("trafficModelRaw", msg);
}

void sleepFor(double sec) {
  usleep(sec * secToUs);
}

double rateKeeper(double loopTime, double lastLoop) {
  double toSleep;
  if (lastLoop < 0){  // don't sleep if last loop lagged
    lastLoop = std::max(lastLoop, -modelRate);  // this should ensure we don't keep adding negative time to lastLoop if a frame lags pretty badly
                          // negative time being time to subtract from sleep time
    // std::cout << "Last frame lagged by " << -lastLoop << " seconds. Sleeping for " << modelRate - (loopTime * msToSec) + lastLoop << " seconds" << std::endl;
    toSleep = modelRate - (loopTime * msToSec) + lastLoop;  // keep time as close as possible to our rate, this reduces the time slept this iter
  } else {
    toSleep = modelRate - (loopTime * msToSec);
  }
  if (toSleep > 0){  // don't sleep for negative time, in case loop takes too long one iteration
    sleepFor(toSleep);
  } else {
    std::cout << "trafficd lagging by " << -(toSleep / msToSec) << " ms." << std::endl;
  }
  return toSleep;
}

void set_do_exit(int sig) {
  std::cout << "trafficd - received signal: " << sig << std::endl;
  std::cout << "trafficd - shutting down!" << std::endl;
  do_exit = 1;
}

uint8_t clamp(int16_t value) {
  return value<0 ? 0 : (value>255 ? 255 : value);
}

static void getFlatArray(const VIPCBuf* buf, float flatImageArray[]) {
  // returns RGB if returnBGR is false
  const size_t width = original_shape[1];
  const size_t height = original_shape[0];

  uint8_t *y = (uint8_t*)buf->addr;
  uint8_t *u = y + (width * height);
  uint8_t *v = u + (width / 2) * (height / 2);

  int b, g, r;
  int idx = 0;
  for (int y_cord = top_crop; y_cord < (original_shape[0] - hood_crop); y_cord++) {
    for (int x_cord = horizontal_crop; x_cord < (original_shape[1] - horizontal_crop); x_cord++) {
      int yy = y[(y_cord * width) + x_cord];
      int uu = u[((y_cord / 2) * (width / 2)) + (x_cord / 2)];
      int vv = v[((y_cord / 2) * (width / 2)) + (x_cord / 2)];

      r = 1.164 * (yy - 16) + 1.596 * (vv - 128);
      g = 1.164 * (yy - 16) - 0.813 * (vv - 128) - 0.391 * (uu - 128);
      b = 1.164 * (yy - 16) + 2.018 * (uu - 128);

      flatImageArray[idx] = clamp(b) / 255.0;
      idx++;
      flatImageArray[idx] = clamp(g) / 255.0;
      idx++;
      flatImageArray[idx] = clamp(r) / 255.0;
      idx++;
    }
  }
}


//void run_trafficd(SubMaster sm, VisionStream *stream) {
//  double loopStart;
//  double lastLoop = 0;
//  float* flatImageArray = new float[cropped_size];
//
//  while (!do_exit || !active) {
//    double loopStart = millis_since_boot();
//    sm.update(0);
//    active = sm["trafficModelControl"].getTrafficModelControl().getActive();
//
//    VIPCBuf* buf;
//    VIPCBufExtra extra;
//    buf = visionstream_get(&stream, &extra);
//    if (buf == NULL) {
//      printf("trafficd: visionstream get failed\n");
//      break;
//    }
//
//    getFlatArray(buf, flatImageArray);  // writes float vector to flatImageArray
//    model->execute(flatImageArray, cropped_size, true);  // true uses special logic for trafficd
//
//    sendPrediction(output, pm);
//
//    lastLoop = rateKeeper(millis_since_boot() - loopStart, lastLoop);
//
//    if (debug_mode) {
//      int maxIdx = 0;
//      for (int i = 1; i < 3; i++) if (output[i] > output[maxIdx]) maxIdx = i;
//      printf("Model prediction: %s (%f)\n", modelLabels[maxIdx].c_str(), 100.0 * output[maxIdx]);
//      std::cout << "Current frequency: " << 1 / ((millis_since_boot() - loopStart) * msToSec) << " Hz" << std::endl;
//    }
//  }
//  free(flatImageArray);
//}


int main(){
  signal(SIGINT, (sighandler_t)set_do_exit);
  signal(SIGTERM, (sighandler_t)set_do_exit);

  PubMaster pm({"trafficModelRaw"});
  
  int err;
  float *output = (float*)calloc(numLabels, sizeof(float));
  RunModel *model = new DefaultRunModel("../../models/traffic_model.dlc", output, numLabels, USE_GPU_RUNTIME);

  VisionStream stream;
  while (!do_exit){  // keep traffic running in case we can't get a frame (mimicking modeld)
//    printf("running trafficd\n");
    VisionStreamBufs buf_info;
    err = visionstream_init(&stream, VISION_STREAM_YUV, true, &buf_info);
    if (err) {
      printf("trafficd: visionstream fail\n");
      usleep(500000);
      continue;
    }

    double loopStart;
    double lastLoop = 0;
    float* flatImageArray = new float[cropped_size];
    while (!do_exit) {
      loopStart = millis_since_boot();

      VIPCBuf* buf;
      VIPCBufExtra extra;
      buf = visionstream_get(&stream, &extra);
      if (buf == NULL) {
        printf("trafficd: visionstream get failed\n");
        break;
      }

      getFlatArray(buf, flatImageArray);  // writes float vector to flatImageArray
      model->execute(flatImageArray, cropped_size, true);  // true uses special logic for trafficd

      sendPrediction(output, pm);

      lastLoop = rateKeeper(millis_since_boot() - loopStart, lastLoop);

      if (debug_mode) {
        int maxIdx = 0;
        for (int i = 1; i < 3; i++) if (output[i] > output[maxIdx]) maxIdx = i;
        printf("Model prediction: %s (%f)\n", modelLabels[maxIdx].c_str(), 100.0 * output[maxIdx]);
        std::cout << "Current frequency: " << 1 / ((millis_since_boot() - loopStart) * msToSec) << " Hz" << std::endl;
      }
    }
    free(flatImageArray);
    visionstream_destroy(&stream);
  }
  free(output);
  delete model;
  std::cout << "trafficd is dead" << std::endl;
  return 0;

//  while (!do_exit) {
//    VisionStreamBufs buf_info;
//    err = visionstream_init(&stream, VISION_STREAM_YUV, true, &buf_info);
//    if (err) {
//      printf("trafficd: visionstream fail\n");
//      usleep(500000);
//      continue;
//    }
//
//    while (!do_exit) {
//      if (active) {
//        printf("running trafficd\n");
//        run_trafficd(sm, stream);
//      } else {
//        sm.update(0);
//        active = sm["trafficModelControl"].getTrafficModelControl().getActive();
//        sleepFor(1.0);
//      }
//    }
//  }
//  free(output);
//  delete model;
//  visionstream_destroy(&stream);
//  std::cout << "trafficd is dead" << std::endl;
//  return 0;
//}
}
