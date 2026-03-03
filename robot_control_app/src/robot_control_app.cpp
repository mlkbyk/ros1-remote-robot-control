#include <QApplication>
#include <QWidget>
#include <QTabWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QDateTime>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_robot_msgs/ExecuteMotionAction.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <QLineEdit>
#include <QProcess>


using Client = actionlib::SimpleActionClient<my_robot_msgs::ExecuteMotionAction>;

// ---------- CONFIG (change topics here) ----------
static const std::string ACTION_NAME = "/execute_motion";
static const std::string RGB_TOPIC   = "/camera/color/image_raw";
static const std::string DEPTH_TOPIC = "/camera/depth/image_rect_raw";
// ------------------------------------------------

static QImage matToQImageBGR(const cv::Mat& bgr) {
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}

static QImage matToQImageGray8(const cv::Mat& gray8) {
  return QImage(gray8.data, gray8.cols, gray8.rows, gray8.step, QImage::Format_Grayscale8).copy();
}

class RobotControlApp : public QWidget {
  Q_OBJECT

public:
  RobotControlApp(QWidget* parent = nullptr)
      : QWidget(parent),
        it_(nh_),
        ac_(ACTION_NAME, true) {

    setWindowTitle("Robot Control App (ROS1 Action + RGBD)");
    resize(980, 560);

    // --- UI Root ---
    auto* root = new QVBoxLayout(this);

    // Top status bar
    auto* top = new QHBoxLayout();
    lblConn_ = new QLabel("ROS: not connected", this);
    lblConn_->setStyleSheet("font-weight:600;");
    top->addWidget(lblConn_);

    top->addStretch(1);

    btnConnect_ = new QPushButton("Connect", this);
    btnDisconnect_ = new QPushButton("Disconnect", this);
    btnDisconnect_->setEnabled(false);
    top->addWidget(btnConnect_);
    top->addWidget(btnDisconnect_);
    root->addLayout(top);

    tabs_ = new QTabWidget(this);
    root->addWidget(tabs_, 1);

    // Tabs
    tabs_->addTab(buildMotionTab(), "Motion");
    tabs_->addTab(buildPerceptionTab(), "Perception");
    connect(btnRecordBag_, &QPushButton::clicked, this, [this](){
  if (!connected_) {
    appendLog("⚠️ Not connected. Start roscore + motion_server, then Connect.");
    return;
  }

  if (!bagRecording_) {
    QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString bagPath = "/tmp/session_" + ts + ".bag";

    QStringList args;
    args << "record" << "-O" << bagPath
         << "/odom" << "/tf" << "/scan" << "/imu" << "/limo_status";

    bagProc_ = new QProcess(this);
    bagProc_->setProgram("rosbag");
    bagProc_->setArguments(args);

    bagProc_->start();
    if (!bagProc_->waitForStarted(1500)) {
      appendLog("❌ Failed to start rosbag.");
      bagProc_->deleteLater();
      bagProc_ = nullptr;
      return;
    }

    bagRecording_ = true;
    btnRecordBag_->setText("Stop Bag");
    appendLog("⏺ Recording bag: " + bagPath);
    appendLog("Topics: /odom /tf /scan /imu /limo_status");
  } else {
    if (bagProc_) {
      bagProc_->terminate();
      if (!bagProc_->waitForFinished(1500)) {
        bagProc_->kill();
        bagProc_->waitForFinished(1500);
      }
      bagProc_->deleteLater();
      bagProc_ = nullptr;
    }

    bagRecording_ = false;
    btnRecordBag_->setText("Record Bag");
    appendLog("⏹ Bag recording stopped.");
  }
});

    // ---- Perception buttons ----
connect(btnApplyTopics_, &QPushButton::clicked, this, [this](){
  currentRgbTopic_ = editRgbTopic_->text().trimmed();
  currentDepthTopic_ = editDepthTopic_->text().trimmed();

  if (!connected_) {
    appendLog("⚠️ Not connected. Topics saved. Press Connect.");
    return;
  }

  rgbSub_.shutdown();
  depthSub_.shutdown();

  rgbSub_ = it_.subscribe(currentRgbTopic_.toStdString(), 1, &RobotControlApp::rgbCb, this);
  depthSub_ = it_.subscribe(currentDepthTopic_.toStdString(), 1, &RobotControlApp::depthCb, this);

  appendLog("✅ Applied RGB topic: " + currentRgbTopic_);
  appendLog("✅ Applied Depth topic: " + currentDepthTopic_);
});

connect(btnSaveFrame_, &QPushButton::clicked, this, [this](){
  if (lastRgbBgr_.empty() || lastDepthRaw_.empty()) {
    appendLog("⚠️ No frames yet to save.");
    return;
  }

  QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
  QString rgbPath = "/tmp/rgb_" + ts + ".png";
  QString depthPath = "/tmp/depth_" + ts + ".png";

  try {
    cv::imwrite(rgbPath.toStdString(), lastRgbBgr_);

    // depth: normalize for viewing
    cv::Mat depth8;
    if (lastDepthRaw_.type() == CV_16UC1) {
      cv::Mat d32; lastDepthRaw_.convertTo(d32, CV_32F);
      cv::normalize(d32, depth8, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else if (lastDepthRaw_.type() == CV_32FC1) {
      cv::normalize(lastDepthRaw_, depth8, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else {
      cv::Mat d32; lastDepthRaw_.convertTo(d32, CV_32F);
      cv::normalize(d32, depth8, 0, 255, cv::NORM_MINMAX, CV_8U);
    }

    cv::imwrite(depthPath.toStdString(), depth8);

    appendLog("✅ Saved RGB: " + rgbPath);
    appendLog("✅ Saved Depth(normalized): " + depthPath);
  } catch (...) {
    appendLog("❌ Save failed.");
  }
});      



    // Log panel
    log_ = new QPlainTextEdit(this);
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(500);
    root->addWidget(log_, 1);

    // Connections
    connect(btnConnect_, &QPushButton::clicked, this, &RobotControlApp::onConnect);
    connect(btnDisconnect_, &QPushButton::clicked, this, &RobotControlApp::onDisconnect);

    // ROS spinner
    spinner_ = std::make_unique<ros::AsyncSpinner>(2);
    spinner_->start();

    // UI timer (for fps/status refresh)
    uiTimer_ = new QTimer(this);
    connect(uiTimer_, &QTimer::timeout, this, &RobotControlApp::tickUI);
    uiTimer_->start(200);

    appendLog("App started. Press Connect.");
  }

private slots:
  void onConnect() {
    appendLog("Connecting to action server: " + QString::fromStdString(ACTION_NAME));

    // Wait for action server (non-blocking style: short wait)
    if (!ac_.waitForServer(ros::Duration(1.5))) {
      setConn(false, "Action server not found (start roscore + motion_server)");
      appendLog("❌ Could not find action server. Make sure roscore and motion_server are running.");
      return;
    }

    setConn(true, "Connected ✅");

    // Subscribe images (if topics exist, they will start coming)
    rgbSub_ = it_.subscribe(currentRgbTopic_.toStdString(), 1, &RobotControlApp::rgbCb, this);
    depthSub_ = it_.subscribe(currentDepthTopic_.toStdString(), 1, &RobotControlApp::depthCb, this);

    appendLog("Subscribed RGB: " + currentRgbTopic_);
    appendLog("Subscribed DEPTH: " + currentDepthTopic_);
  }

  void onDisconnect() {
    // Stop motion for safety
    sendGoal(0, 0.0, 0.0);

    rgbSub_.shutdown();
    depthSub_.shutdown();

    setConn(false, "Disconnected");
    appendLog("Disconnected. Subscriptions shutdown.");
  }

  void onStop() { sendGoal(0, 0.0, 0.0); }

  void onSendVel() {
    double lin = sliderLin_->value() / 100.0;   // -1..1
    double ang = sliderAng_->value() / 100.0;
    sendGoal(3, lin, ang);
  }

  void onTurnPlus90() { sendGoal(1, 90.0, 0.0); }
  void onTurnMinus90() { sendGoal(1, -90.0, 0.0); }
  void onDriveMeters() { sendGoal(2, spinMeters_->value(), 0.0); }

private:
  QWidget* buildMotionTab() {
    auto* w = new QWidget(this);
    auto* v = new QVBoxLayout(w);

    // Quick buttons
    auto* gbQuick = new QGroupBox("Quick Commands", w);
    auto* q = new QHBoxLayout(gbQuick);

    btnStop_ = new QPushButton("STOP", gbQuick);
    btnStop_->setStyleSheet("font-weight:700; padding:10px;");
    btnTurnP_ = new QPushButton("Turn +90°", gbQuick);
    btnTurnM_ = new QPushButton("Turn -90°", gbQuick);

    q->addWidget(btnStop_);
    q->addWidget(btnTurnP_);
    q->addWidget(btnTurnM_);
    v->addWidget(gbQuick);

    // Drive meters
    auto* gbDrive = new QGroupBox("Drive", w);
    auto* d = new QHBoxLayout(gbDrive);
    spinMeters_ = new QDoubleSpinBox(gbDrive);
    spinMeters_->setRange(0.1, 10.0);
    spinMeters_->setSingleStep(0.1);
    spinMeters_->setValue(1.0);
    btnDrive_ = new QPushButton("Drive (m)", gbDrive);
    d->addWidget(new QLabel("Meters:", gbDrive));
    d->addWidget(spinMeters_);
    d->addWidget(btnDrive_);
    v->addWidget(gbDrive);

    // Velocity sliders
    auto* gbVel = new QGroupBox("VEL Mode (publishes via Action, ~2s burst)", w);
    auto* vv = new QVBoxLayout(gbVel);

    auto* row1 = new QHBoxLayout();
    sliderLin_ = new QSlider(Qt::Horizontal, gbVel);
    sliderLin_->setRange(-100, 100);
    sliderLin_->setValue(0);
    lblLinVal_ = new QLabel("0.00", gbVel);
    row1->addWidget(new QLabel("Linear:", gbVel));
    row1->addWidget(sliderLin_, 1);
    row1->addWidget(lblLinVal_);
    vv->addLayout(row1);

    auto* row2 = new QHBoxLayout();
    sliderAng_ = new QSlider(Qt::Horizontal, gbVel);
    sliderAng_->setRange(-100, 100);
    sliderAng_->setValue(0);
    lblAngVal_ = new QLabel("0.00", gbVel);
    row2->addWidget(new QLabel("Angular:", gbVel));
    row2->addWidget(sliderAng_, 1);
    row2->addWidget(lblAngVal_);
    vv->addLayout(row2);

    btnVel_ = new QPushButton("Send VEL", gbVel);
    vv->addWidget(btnVel_);

    v->addWidget(gbVel);
    v->addStretch(1);

    // UI signals
    connect(btnStop_, &QPushButton::clicked, this, &RobotControlApp::onStop);
    connect(btnTurnP_, &QPushButton::clicked, this, &RobotControlApp::onTurnPlus90);
    connect(btnTurnM_, &QPushButton::clicked, this, &RobotControlApp::onTurnMinus90);
    connect(btnDrive_, &QPushButton::clicked, this, &RobotControlApp::onDriveMeters);
    connect(btnVel_, &QPushButton::clicked, this, &RobotControlApp::onSendVel);

    connect(sliderLin_, &QSlider::valueChanged, this, [this](int v){
      lblLinVal_->setText(QString::number(v/100.0, 'f', 2));
    });
    connect(sliderAng_, &QSlider::valueChanged, this, [this](int v){
      lblAngVal_->setText(QString::number(v/100.0, 'f', 2));
    });

    return w;
  }

QWidget* buildPerceptionTab() {
  auto* w = new QWidget(this);
  auto* v = new QVBoxLayout(w);

  // ---- Topic settings bar (top) ----
  auto* settings = new QGroupBox("Topic Settings", w);
  auto* s = new QHBoxLayout(settings);

  editRgbTopic_ = new QLineEdit(currentRgbTopic_, settings);
  editDepthTopic_ = new QLineEdit(currentDepthTopic_, settings);
  btnApplyTopics_ = new QPushButton("Apply", settings);
  btnSaveFrame_ = new QPushButton("Save Frame", settings);
  btnRecordBag_ = new QPushButton("Record Bag", settings);

  s->addWidget(new QLabel("RGB:", settings));
  s->addWidget(editRgbTopic_, 1);
  s->addWidget(new QLabel("Depth:", settings));
  s->addWidget(editDepthTopic_, 1);
  s->addWidget(btnApplyTopics_);
  s->addWidget(btnSaveFrame_);
  s->addWidget(btnRecordBag_);

  v->addWidget(settings);

  // ---- Views (bottom) ----
  auto* h = new QHBoxLayout();

  // RGB view
  auto* gbRgb = new QGroupBox("RGB", w);
  auto* vr = new QVBoxLayout(gbRgb);
  lblRgb_ = new QLabel("No RGB frames yet", gbRgb);
  lblRgb_->setMinimumSize(440, 330);
  lblRgb_->setStyleSheet("background:#222; color:#ddd; padding:8px;");
  vr->addWidget(lblRgb_, 1);

  lblRgbFps_ = new QLabel("RGB FPS: 0", gbRgb);
  vr->addWidget(lblRgbFps_);
  h->addWidget(gbRgb, 1);

  // Depth view
  auto* gbDepth = new QGroupBox("Depth (normalized)", w);
  auto* vd = new QVBoxLayout(gbDepth);
  lblDepth_ = new QLabel("No Depth frames yet", gbDepth);
  lblDepth_->setMinimumSize(440, 330);
  lblDepth_->setStyleSheet("background:#222; color:#ddd; padding:8px;");
  vd->addWidget(lblDepth_, 1);

  lblDepthFps_ = new QLabel("Depth FPS: 0", gbDepth);
  vd->addWidget(lblDepthFps_);
  h->addWidget(gbDepth, 1);

  v->addLayout(h, 1);

  return w;
}

  void setConn(bool ok, const QString& msg) {
    if (ok) {
      lblConn_->setText("ROS: " + msg);
      btnConnect_->setEnabled(false);
      btnDisconnect_->setEnabled(true);
    } else {
      lblConn_->setText("ROS: " + msg);
      btnConnect_->setEnabled(true);
      btnDisconnect_->setEnabled(false);
    }
    connected_ = ok;
  }

void appendLog(const QString& s) {
  QString line = QDateTime::currentDateTime().toString("HH:mm:ss") + "  " + s;
  QMetaObject::invokeMethod(log_, [this, line](){
    log_->appendPlainText(line);
  }, Qt::QueuedConnection);
}

  void sendGoal(uint8_t mode, double v1, double v2) {
    if (!connected_) {
      appendLog("⚠️ Not connected. Press Connect first.");
      return;
    }

    my_robot_msgs::ExecuteMotionGoal goal;
    goal.mode = mode;
    goal.v1 = (float)v1;
    goal.v2 = (float)v2;

    appendLog(QString("Goal → mode=%1 v1=%2 v2=%3")
                .arg(mode).arg(v1,0,'f',2).arg(v2,0,'f',2));

    ac_.sendGoal(goal,
      [this](const actionlib::SimpleClientGoalState& st,
             const my_robot_msgs::ExecuteMotionResultConstPtr& res){
        QString m = QString("Result ← %1 | ok=%2 | %3")
          .arg(QString::fromStdString(st.toString()))
          .arg(res ? (res->ok ? "true" : "false") : "false")
          .arg(res ? QString::fromStdString(res->message) : "no result");
        appendLog(m);
      },
      [this](){ appendLog("Active…"); },
      [this](const my_robot_msgs::ExecuteMotionFeedbackConstPtr& fb){
        if (!fb) return;
        appendLog(QString("Feedback ← %1 (%2)")
                    .arg(QString::fromStdString(fb->state))
                    .arg(fb->progress, 0, 'f', 2));
      }
    );
  }

  void rgbCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      auto cvp = cv_bridge::toCvShare(msg, "bgr8");
      lastRgbBgr_ = cvp->image.clone();
      lastRgbTs_ = ros::Time::now();
      rgbCount_++;

      QImage q = matToQImageBGR(cvp->image);
      QPixmap px = QPixmap::fromImage(q).scaled(lblRgb_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

      // UI update must be in UI thread; Qt queued update works because QLabel is thread-safe-ish for queued events.
      QMetaObject::invokeMethod(lblRgb_, [this, px](){ lblRgb_->setPixmap(px); }, Qt::QueuedConnection);
    } catch (...) {
      // ignore
    }
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      auto cvp = cv_bridge::toCvShare(msg);
      lastDepthTs_ = ros::Time::now();
      depthCount_++;

      // normalize to 8-bit for viewing
      cv::Mat depth = cvp->image;
      lastDepthRaw_ = depth.clone();
      cv::Mat depth8;

      if (depth.type() == CV_16UC1) {
        cv::Mat depth32f;
        depth.convertTo(depth32f, CV_32F);
        cv::normalize(depth32f, depth8, 0, 255, cv::NORM_MINMAX, CV_8U);
      } else if (depth.type() == CV_32FC1) {
        cv::normalize(depth, depth8, 0, 255, cv::NORM_MINMAX, CV_8U);
      } else {
        // unknown: try convert
        cv::Mat tmp;
        depth.convertTo(tmp, CV_32F);
        cv::normalize(tmp, depth8, 0, 255, cv::NORM_MINMAX, CV_8U);
      }

      QImage q = matToQImageGray8(depth8);
      QPixmap px = QPixmap::fromImage(q).scaled(lblDepth_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
      QMetaObject::invokeMethod(lblDepth_, [this, px](){ lblDepth_->setPixmap(px); }, Qt::QueuedConnection);
    } catch (...) {
      // ignore
    }
  }

  void tickUI() {
    // fps counters (simple)
    static int lastRgbCount = 0;
    static int lastDepthCount = 0;
    static ros::Time last = ros::Time::now();

    ros::Time now = ros::Time::now();
    double dt = (now - last).toSec();
    if (dt >= 1.0) {
      int rgbDiff = rgbCount_ - lastRgbCount;
      int dDiff = depthCount_ - lastDepthCount;
      lblRgbFps_->setText(QString("RGB FPS: %1").arg(rgbDiff));
      lblDepthFps_->setText(QString("Depth FPS: %1").arg(dDiff));
      lastRgbCount = rgbCount_;
      lastDepthCount = depthCount_;
      last = now;
    }
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber rgbSub_;
  image_transport::Subscriber depthSub_;

  Client ac_;
  bool connected_ = false;

  std::unique_ptr<ros::AsyncSpinner> spinner_;
  QTimer* uiTimer_ = nullptr;

  // UI widgets
  QTabWidget* tabs_ = nullptr;
  QLabel* lblConn_ = nullptr;
  QPushButton* btnConnect_ = nullptr;
  QPushButton* btnDisconnect_ = nullptr;


  QPushButton* btnStop_ = nullptr;
  QPushButton* btnVel_ = nullptr;
  QPushButton* btnTurnP_ = nullptr;
  QPushButton* btnTurnM_ = nullptr;
  QPushButton* btnDrive_ = nullptr;

  QSlider* sliderLin_ = nullptr;
  QSlider* sliderAng_ = nullptr;
  QLabel* lblLinVal_ = nullptr;
  QLabel* lblAngVal_ = nullptr;
  QDoubleSpinBox* spinMeters_ = nullptr;

  QLabel* lblRgb_ = nullptr;
  QLabel* lblDepth_ = nullptr;
  QLabel* lblRgbFps_ = nullptr;
  QLabel* lblDepthFps_ = nullptr;

  QPlainTextEdit* log_ = nullptr;

  // Rosbag recording
  QPushButton* btnRecordBag_ = nullptr;
  QProcess* bagProc_ = nullptr;
  bool bagRecording_ = false;
  
    // Topic settings UI
  QLineEdit* editRgbTopic_ = nullptr;
  QLineEdit* editDepthTopic_ = nullptr;
  QPushButton* btnApplyTopics_ = nullptr;
  QPushButton* btnSaveFrame_ = nullptr;

  // Current topics (editable from UI)
  QString currentRgbTopic_ = QString::fromStdString(RGB_TOPIC);
  QString currentDepthTopic_ = QString::fromStdString(DEPTH_TOPIC);

  // Last frames (for Save Frame)
  cv::Mat lastRgbBgr_;
  cv::Mat lastDepthRaw_;
  

  // stats
  ros::Time lastRgbTs_;
  ros::Time lastDepthTs_;
  int rgbCount_ = 0;
  int depthCount_ = 0;
};

#include "robot_control_app.moc"

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_control_app");
  QApplication a(argc, argv);
  RobotControlApp w;
  w.show();
  return a.exec();
}
