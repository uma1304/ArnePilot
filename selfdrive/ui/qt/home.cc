#include "selfdrive/ui/qt/home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <iomanip>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/prime.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(this, &HomeWindow::update, sidebar, &Sidebar::updateState);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome();
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  QObject::connect(this, &HomeWindow::update, onroad, &OnroadWindow::updateStateSignal);
  QObject::connect(this, &HomeWindow::offroadTransitionSignal, onroad, &OnroadWindow::offroadTransitionSignal);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::offroadTransition(bool offroad) {
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
  emit offroadTransitionSignal(offroad);
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}

void issue_debug_snapshot(SubMaster &sm) {
  auto longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
  auto live_map_data = sm["liveMapData"].getLiveMapData();
  auto car_state = sm["carState"].getCarState();

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream param_name_os;
  param_name_os << std::put_time(&tm, "%Y-%m-%d--%H-%M-%S");

  std::ostringstream os;
  os.setf(std::ios_base::fixed);
  os.precision(2);
  os << "Datetime: " << param_name_os.str() << ", vEgo: " << car_state.getVEgo() * 3.6 << "\n\n";
  os.precision(6);
  os << "Location: (" << live_map_data.getLastGpsLatitude() << ", " << live_map_data.getLastGpsLongitude()  << ")\n";
  os.precision(2);
  os << "Bearing: " << live_map_data.getLastGpsBearingDeg() << "; ";
  os << "GPSSpeed: " << live_map_data.getLastGpsSpeed() * 3.6 << "\n\n";
  os.precision(1);
  os << "Speed Limit: " << live_map_data.getSpeedLimit() * 3.6 << ", ";
  os << "Valid: " << live_map_data.getSpeedLimitValid() << "\n";
  os << "Speed Limit Ahead: " << live_map_data.getSpeedLimitAhead() * 3.6 << ", ";
  os << "Valid: " << live_map_data.getSpeedLimitAheadValid() << ", ";
  os << "Distance: " << live_map_data.getSpeedLimitAheadDistance() << "\n";
  os << "Turn Speed Limit: " << live_map_data.getTurnSpeedLimit() * 3.6 << ", ";
  os << "Valid: " << live_map_data.getTurnSpeedLimitValid() << ", ";
  os << "End Distance: " << live_map_data.getTurnSpeedLimitEndDistance() << ", ";
  os << "Sign: " << live_map_data.getTurnSpeedLimitSign() << "\n\n";

  const auto turn_speeds = live_map_data.getTurnSpeedLimitsAhead();
  os << "Turn Speed Limits Ahead:\n";
  os << "VALUE\tDIST\tSIGN\n";

  if (turn_speeds.size() == 0) {
    os << "-\t-\t-" << "\n\n";
  } else {
    const auto distances = live_map_data.getTurnSpeedLimitsAheadDistances();
    const auto signs = live_map_data.getTurnSpeedLimitsAheadSigns();
    for(int i = 0; i < turn_speeds.size(); i++) {
      os << turn_speeds[i] * 3.6 << "\t" << distances[i] << "\t" << signs[i] << "\n";
    }
    os << "\n";
  }

  os << "SPEED LIMIT CONTROLLER:\n";
  os << "sl: " << longitudinal_plan.getSpeedLimit() * 3.6  << ", ";
  os << "state: " << int(longitudinal_plan.getSpeedLimitControlState()) << ", ";
  os << "isMap: " << longitudinal_plan.getIsMapSpeedLimit() << "\n\n";

  os << "TURN SPEED CONTROLLER:\n";
  os << "speed: " << longitudinal_plan.getTurnSpeed() * 3.6 << ", ";
  os << "state: " << int(longitudinal_plan.getTurnSpeedControlState()) << "\n\n";

  os << "VISION TURN CONTROLLER:\n";
  os << "speed: " << longitudinal_plan.getVisionTurnSpeed() * 3.6 << ", ";
  os << "state: " << int(longitudinal_plan.getVisionTurnControllerState()); 

  Params().put(param_name_os.str().c_str(), os.str().c_str(), os.str().length());
  QUIState::ui_state.scene.display_debug_alert_frame = sm.frame;
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Toggle speed limit control enabled
  SubMaster &sm = *(QUIState::ui_state.sm);
  auto longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
  
  Rect speed_limit_touch_rect = QUIState::ui_state.scene.speed_limit_sign_touch_rect;
  Rect debug_tap_rect = {rect().center().x() - 200, rect().center().y() - 200, 400, 400};
  if (sidebar->isVisible()) {
    speed_limit_touch_rect.x += sidebar->width();
    debug_tap_rect.x += sidebar->width();
    debug_tap_rect.w -= sidebar->width();
  }

  if (onroad->isVisible() && longitudinal_plan.getSpeedLimit() > 0.0 &&
      speed_limit_touch_rect.ptInRect(e->x(), e->y())) {
    // If touching the speed limit sign area when visible
    QUIState::ui_state.scene.last_speed_limit_sign_tap = seconds_since_boot();
    QUIState::ui_state.scene.speed_limit_control_enabled = !QUIState::ui_state.scene.speed_limit_control_enabled;
    Params().putBool("SpeedLimitControl", QUIState::ui_state.scene.speed_limit_control_enabled);
  }

  else if (QUIState::ui_state.scene.debug_snapshot_enabled &&
           onroad->isVisible() && debug_tap_rect.ptInRect(e->x(), e->y())) {
    issue_debug_snapshot(*(QUIState::ui_state.sm));
  }

  // Handle sidebar collapsing
  else if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 45);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(15, 15, 15, 0);
  header_layout->setSpacing(16);

  date = new QLabel();
  header_layout->addWidget(date, 1, Qt::AlignHCenter | Qt::AlignLeft);

  update_notif = new QPushButton("UPDATE");
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  header_layout->addWidget(new QLabel(getBrandVersion()), 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget* statsAndSetupWidget = new QWidget(this);
  QHBoxLayout* statsAndSetup = new QHBoxLayout(statsAndSetupWidget);
  statsAndSetup->setMargin(0);
  statsAndSetup->setSpacing(30);
  statsAndSetup->addWidget(new DriveStats, 1);
  statsAndSetup->addWidget(new SetupWidget);

  center_layout->addWidget(statsAndSetupWidget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
     color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QDateTime::currentDateTime().toString("dddd, MMMM d"));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + " ALERT" + (alerts > 1 ? "S" : ""));
  }
}
