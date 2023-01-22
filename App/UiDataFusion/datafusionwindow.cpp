//
// Created by rebeater on 11/23/21.
//

// You may need to build the project (run Qt uic code generator) to get "ui_DataFusionWindow.h" resolved

#include "DataFusionThread.h"

#include <QFileDialog>
#include <QDesktopServices>
#include <QTime>
#include <QFontDatabase>
#include "datafusionwindow.h"
#include "ui_DataFusionWindow.h"

#include "glog/logging.h"

DataFusionWindow::DataFusionWindow(QMainWindow *parent) :
	QMainWindow(parent), ui(new Ui::DataFusionWindow) {
  google::InitGoogleLogging(".log/");
  google::LogToStderr();
  ui->setupUi(this);
  SetFont();
  InitialUi();
}

DataFusionWindow::~DataFusionWindow() {
  delete ui;
}
void DataFusionWindow::on_btnStart_clicked() {
  LOG(INFO) << "Start Progress";
  ui_set_ok = true;
  bool ok = false;
  config_.start_time = ui->editStartTime->text().toFloat(&ok);
  OnConvertResult(ui->editStartTime->text(), ok);
  config_.stop_time = ui->editStopTime->text().toFloat(&ok);
  OnConvertResult(ui->editStopTime->text(), ok);
  config_.output_config.file_path = ui->editOutputPath->text().toStdString();
  config_.imu_config.file_path = ui->editImuPath->text().toStdString();
  config_.imu_config.d_rate = ui->editImuRate->text().toInt(&ok);
  OnConvertResult(ui->editImuRate->text(), ok);
  config_.imu_config.format = (IMUFileFormat)ui->cbxImuFormat->currentIndex();
  config_.imu_config.parameter_path = ui->editImuParameterPath->text().toStdString();

  config_.gnss_config.file_path = ui->editGnssPath->text().toStdString();
  config_.gnss_config.format = (GnssFileFormat)ui->cbxGnssFormat->currentIndex();
  config_.gnss_config.level_arm[0] = ui->editGnssLevelArmX->text().toFloat(&ok);
  OnConvertResult(ui->editGnssLevelArmX->text(), ok);
  config_.gnss_config.level_arm[1] = ui->editGnssLevelArmY->text().toFloat(&ok);
  OnConvertResult(ui->editGnssLevelArmY->text(), ok);
  config_.gnss_config.level_arm[2] = ui->editGnssLevelArmZ->text().toFloat(&ok);
  OnConvertResult(ui->editGnssLevelArmZ->text(), ok);
  config_.gnss_config.double_antenna_enable = ui->cbxDoubleAntennaEnable->isChecked();
  if (config_.gnss_config.double_antenna_enable) {
	config_.gnss_config.pitch_of_antenna = ui->editDoubleAntennaPitch->text().toFloat(&ok);
	OnConvertResult(ui->editDoubleAntennaPitch->text(), ok);
	config_.gnss_config.yaw_of_antenna = ui->editDoubleAntennaYaw->text().toFloat(&ok);
	OnConvertResult(ui->editDoubleAntennaYaw->text(), ok);
  }

  config_.odometer_config.enable = ui->cbxOdometerEnable->isChecked();
  if (config_.odometer_config.enable) {
	config_.odometer_config.file_path = ui->editOdometerPath->text().toStdString();
	config_.odometer_config.odometer_std = ui->editOdometerStd->text().toFloat(&ok);
	OnConvertResult(ui->editOdometerStd->text(), ok);
	config_.odometer_config.scale_factor = ui->editOdometerScaleFactor->text().toFloat(&ok);
	config_.odometer_config.scale_factor_std = ui->editOdometerScaleFactorStd->text().toFloat(&ok);
  }
  config_.odometer_config.nhc_enable = ui->cbxNhcEnable->isChecked();
  if (config_.odometer_config.nhc_enable) {
	config_.odometer_config.nhc_std[0] = ui->editNhcStd1->text().toFloat(&ok);
	OnConvertResult(ui->editNhcStd1->text(), ok);
	config_.odometer_config.nhc_std[1] = ui->editNhcStd2->text().toFloat(&ok);
	OnConvertResult(ui->editNhcStd2->text(), ok);
  }

  if (config_.odometer_config.enable or config_.odometer_config.nhc_enable) {
	/* */
	config_.odometer_config.angle_bv[0] = ui->editAngleBvX->text().toFloat(&ok);
	OnConvertResult(ui->editAngleBvX->text(), ok);
	config_.odometer_config.angle_bv[1] = ui->editAngleBvY->text().toFloat(&ok);
	OnConvertResult(ui->editAngleBvY->text(), ok);
	config_.odometer_config.angle_bv[2] = ui->editAngleBvZ->text().toFloat(&ok);
	OnConvertResult(ui->editAngleBvZ->text(), ok);
	config_.odometer_config.wheel_level_arm[0] = ui->editOdometerLevelArmX->text().toFloat(&ok);
	OnConvertResult(ui->editOdometerLevelArmX->text(), ok);
	config_.odometer_config.wheel_level_arm[1] = ui->editOdometerLevelArmY->text().toFloat(&ok);
	OnConvertResult(ui->editOdometerLevelArmY->text(), ok);
	config_.odometer_config.wheel_level_arm[2] = ui->editOdometerLevelArmZ->text().toFloat(&ok);
	OnConvertResult(ui->editOdometerLevelArmZ->text(), ok);
	config_.odometer_config.scale_factor = ui->editOdometerScaleFactor->text().toFloat(&ok);
	config_.odometer_config.scale_factor_std = ui->editOdometerScaleFactorStd->text().toFloat(&ok);
  }
  config_.zupt_config.zupt_enable = ui->cbxZuptEnable->isChecked();
  if (config_.zupt_config.zupt_enable) {
	config_.zupt_config.zupt_std = ui->editZuptStd->text().toFloat(&ok);
	OnConvertResult(ui->editZuptStd->text(), ok);
  }
  config_.zupt_config.zupta_enable = ui->cbxZuptaEnable->isChecked();
  if (config_.zupt_config.zupta_enable) {
	config_.zupt_config.zupta_std = ui->editZuptaStd->text().toFloat(&ok);
	OnConvertResult(ui->editZuptaStd->text(), ok);
  }
  config_.align_config.mode = (RgioeAlignMode)ui->cbxAlignMode->currentIndex();
  if (config_.align_config.mode == RgioeAlignMode::ALIGN_MOVING) {
	config_.align_config.vel_threshold_for_moving = ui->editVelThresholdInMoving->text().toFloat(&ok);
	OnConvertResult(ui->editVelThresholdInMoving->text(), ok);
  }
  if (config_.align_config.mode == RgioeAlignMode::ALIGN_USE_GIVEN) {
	/*Do nothing*/
  }
  config_.outage_config.enable = ui->cbxOutageEnable->isChecked();
  if (config_.outage_config.enable) {
	config_.outage_config.start = ui->editOutageStartTime->text().toFloat(&ok);
	OnConvertResult(ui->editOutageStartTime->text(), ok);
	config_.outage_config.stop = ui->editOutageStopTime->text().toFloat(&ok);
	OnConvertResult(ui->editOutageStopTime->text(), ok);
	config_.outage_config.step = ui->editStepTime->text().toFloat(&ok);
	OnConvertResult(ui->editStepTime->text(), ok);
	config_.outage_config.outage = ui->editOutageTime->text().toFloat(&ok);
	OnConvertResult(ui->editOutageTime->text(), ok);
  }
  if (!ui_set_ok) {
	LOG(ERROR) << "部分参数格式不正确";
	ShowLog("部分参数格式不正确");
	return;
  }
  std::string error_message;
  ok = config_.LoadImuPara(error_message);
  if (!ok) {
	ShowLog("IMU参数文件格式不正确：" + QString::fromStdString(error_message));
	LOG(ERROR) << error_message;
	return;
  } else {
	LOG(INFO) << "Start Process...";
	fusion_thread = new DataFusionThread(config_, this);
	connect(fusion_thread, SIGNAL(SendStatus(int)), this, SLOT(on_ProcessThreadFeedBack(int)));
	connect(fusion_thread, SIGNAL(SendLog(QString)), this, SLOT(on_ProcessThreadLogFeedBack(QString)));
	fusion_thread->start();
  }
}
void DataFusionWindow::ShowLog(const QString &s) {
  auto time = QTime::currentTime();
  auto strtime = time.toString("hh:mm:ss.zzz");
  ui->listLog->addItem(strtime + ": " + s);
}
void DataFusionWindow::OnConvertResult(const QString &s, bool ok) {
  if (ok) return;
  ui_set_ok = false;
  ui->listLog->addItem("非法输入:" + s);
}
void DataFusionWindow::on_btnBrowseImuFile_clicked() {
  QString filename = QFileDialog::getOpenFileName(this, tr("open IMU file"), "", tr("Imu files(*)"));
  LOG(INFO) << "Choose path:" << filename.toStdString();
  if (filename.size() > 0) ui->editImuPath->setText(filename);
}
void DataFusionWindow::on_btnBrowseImuParameterFile_clicked() {
  QString filename = QFileDialog::getOpenFileName(this, tr("open IMU parameter file"), "", tr("YAML files(*.yml)"));
  LOG(INFO) << "Choose path:" << filename.toStdString();
  if (filename.size() > 0) ui->editImuParameterPath->setText(filename);
}
void DataFusionWindow::on_btnEditImuParameterFile_clicked() {
  QString filename = ui->editImuParameterPath->text();
  if (filename.size() == 0) return;
  QFile file(filename);
  if (!file.exists()) {
	LOG(ERROR) << "No such file or directory: " + filename.toStdString();
	ShowLog("No such file or directory: " + filename);
	return;
  }
  file.close();
  QDesktopServices::openUrl(QUrl(filename));
}
void DataFusionWindow::on_btnBrowseGnss_clicked() {
  QString filename = QFileDialog::getOpenFileName(this, tr("open GNSS file"), "", tr("GNSS files(*)"));
  LOG(INFO) << "Choose path:" << filename.toStdString();
  if (filename.size() > 0) ui->editGnssPath->setText(filename);
}
void DataFusionWindow::on_cbxDoubleAntennaEnable_stateChanged(int x) {
  LOG(INFO) << "Double Antenna Enable changed to " << x;
  if (ui->cbxDoubleAntennaEnable->isChecked()) {
	ui->editDoubleAntennaPitch->setEnabled(true);
	ui->editDoubleAntennaYaw->setEnabled(true);
  } else {
	ui->editDoubleAntennaPitch->setEnabled(false);
	ui->editDoubleAntennaYaw->setEnabled(false);
  }
}

void DataFusionWindow::InitialUi() {
  /*TODO setup ui by config_*/
  ui->editImuPath->setText(QString::fromStdString(config_.imu_config.file_path));
  ui->cbxImuFormat->setCurrentIndex((IMUFileFormat)config_.imu_config.format);
  ui->cbxImuFrame->setCurrentIndex((IMUFrame)config_.imu_config.frame);
  ui->editImuRate->setText(QString::number(config_.imu_config.d_rate));
  ui->editImuParameterPath->setText(QString::fromStdString(config_.imu_config.parameter_path));

  ui->editGnssPath->setText(QString::fromStdString(config_.gnss_config.file_path));
  ui->cbxGnssFormat->setCurrentIndex((GnssFileFormat)config_.gnss_config.format);
  ui->editGnssLevelArmX->setText(QString::number(config_.gnss_config.level_arm[0]));
  ui->editGnssLevelArmY->setText(QString::number(config_.gnss_config.level_arm[1]));
  ui->editGnssLevelArmZ->setText(QString::number(config_.gnss_config.level_arm[2]));
  ui->cbxDoubleAntennaEnable->setCheckState(config_.gnss_config.double_antenna_enable ? Qt::Checked : Qt::Unchecked);
  ui->editDoubleAntennaPitch->setText(QString::number(config_.gnss_config.pitch_of_antenna));
  ui->editDoubleAntennaYaw->setText(QString::number(config_.gnss_config.yaw_of_antenna));

  ui->cbxOdometerEnable->setCheckState(config_.odometer_config.enable ? Qt::Checked : Qt::Unchecked);
  ui->editOdometerPath->setText(QString::fromStdString(config_.odometer_config.file_path));
  ui->editOdometerStd->setText(QString::number(config_.odometer_config.odometer_std));
  ui->editOdometerScaleFactor->setText(QString::number(config_.odometer_config.scale_factor));
  ui->editOdometerScaleFactorStd->setText(QString::number(config_.odometer_config.scale_factor_std));
  ui->cbxNhcEnable->setCheckState(config_.odometer_config.nhc_enable ? Qt::Checked : Qt::Unchecked);
  ui->editNhcStd1->setText(QString::number(config_.odometer_config.nhc_std[0]));
  ui->editNhcStd2->setText(QString::number(config_.odometer_config.nhc_std[1]));
  ui->editAngleBvX->setText(QString::number(config_.odometer_config.angle_bv[0]));
  ui->editAngleBvY->setText(QString::number(config_.odometer_config.angle_bv[1]));
  ui->editAngleBvZ->setText(QString::number(config_.odometer_config.angle_bv[2]));
  ui->editOdometerLevelArmX->setText(QString::number(config_.odometer_config.wheel_level_arm[0]));
  ui->editOdometerLevelArmY->setText(QString::number(config_.odometer_config.wheel_level_arm[1]));
  ui->editOdometerLevelArmZ->setText(QString::number(config_.odometer_config.wheel_level_arm[2]));

  ui->cbxZuptEnable->setCheckState(config_.zupt_config.zupt_enable ? Qt::Checked : Qt::Unchecked);
  ui->editZuptStd->setText(QString::number(config_.zupt_config.zupt_std));
  ui->editZuptaStd->setText(QString::number(config_.zupt_config.zupta_std));
  ui->cbxZuptaEnable->setCheckState(config_.zupt_config.zupta_enable ? Qt::Checked : Qt::Unchecked);

  ui->cbxAlignMode->setCurrentIndex((RgioeAlignMode)config_.align_config.mode);
  ui->editVelThresholdInMoving->setText(QString::number(config_.align_config.vel_threshold_for_moving));

  ui->cbxOutageEnable->setCheckState(config_.outage_config.enable ? Qt::Checked : Qt::Unchecked);
  ui->editOutageStartTime->setText(QString::number(config_.outage_config.start));
  ui->editOutageStopTime->setText(QString::number(config_.outage_config.stop));
  ui->editStepTime->setText(QString::number(config_.outage_config.step));
  ui->editOutageTime->setText(QString::number(config_.outage_config.outage));

  ui->editStartTime->setText(QString::number(config_.start_time));
  ui->editStopTime->setText(QString::number(config_.stop_time));
  ui->editOutputPath->setText(QString::fromStdString(config_.output_config.file_path));

  ui->cbxDoubleAntennaEnable->stateChanged(0);
  ui->cbxOdometerEnable->stateChanged(0);
  ui->cbxNhcEnable->stateChanged(0);
  ui->cbxZuptEnable->stateChanged(0);
  ui->cbxZuptaEnable->stateChanged(0);
  ui->cbxAlignMode->currentIndexChanged(0);
  ui->cbxOutageEnable->stateChanged(0);
}

void DataFusionWindow::on_cbxOdometerEnable_stateChanged(int x) {
  LOG(INFO) << "Odometer Enable changed to " << x;
  if (ui->cbxOdometerEnable->isChecked()) {
	ui->editOdometerPath->setEnabled(true);
	ui->btnBrowseOdometerPath->setEnabled(true);
	ui->editOdometerStd->setEnabled(true);
	ui->editOdometerScaleFactor->setEnabled(true);
	ui->editOdometerScaleFactorStd->setEnabled(true);
  } else {
	ui->editOdometerPath->setEnabled(false);
	ui->btnBrowseOdometerPath->setEnabled(false);
	ui->editOdometerStd->setEnabled(false);
	ui->editOdometerScaleFactor->setEnabled(false);
	ui->editOdometerScaleFactorStd->setEnabled(false);
  }
  SetWheelInfoEnable();
}
void DataFusionWindow::SetWheelInfoEnable() {
  bool enable_wheel = false;
  if (ui->cbxOdometerEnable->isChecked() or ui->cbxNhcEnable->isChecked()) {
	enable_wheel = true;
  } else {
	enable_wheel = false;
  }
  ui->editAngleBvX->setEnabled(enable_wheel);
  ui->editAngleBvY->setEnabled(enable_wheel);
  ui->editAngleBvZ->setEnabled(enable_wheel);
  ui->editOdometerLevelArmX->setEnabled(enable_wheel);
  ui->editOdometerLevelArmY->setEnabled(enable_wheel);
  ui->editOdometerLevelArmZ->setEnabled(enable_wheel);
}
void DataFusionWindow::on_cbxNhcEnable_stateChanged(int x) {
  LOG(INFO) << "NHC Enable changed to " << x;
  if (ui->cbxNhcEnable->isChecked()) {
	ui->editNhcStd1->setEnabled(true);
	ui->editNhcStd2->setEnabled(true);
  } else {
	ui->editNhcStd1->setEnabled(false);
	ui->editNhcStd2->setEnabled(false);
  }
  SetWheelInfoEnable();
}
void DataFusionWindow::on_cbxZuptEnable_stateChanged(int x) {
  if (ui->cbxZuptEnable->isChecked())
	ui->editZuptStd->setEnabled(true);
  else
	ui->editZuptStd->setEnabled(false);
}
void DataFusionWindow::on_cbxZuptaEnable_stateChanged(int x) {
  if (ui->cbxZuptaEnable->isChecked())
	ui->editZuptaStd->setEnabled(true);
  else
	ui->editZuptaStd->setEnabled(false);
}
void DataFusionWindow::on_btnOpenZuptDetector_clicked() {
  LOG(INFO) << "Open ZUPT detector";
}
void DataFusionWindow::on_cbxAlignMode_currentIndexChanged(int x) {
  switch ((RgioeAlignMode)x) {
	case ALIGN_MOVING:ui->editVelThresholdInMoving->setEnabled(true);
	  ui->btnEditInitialState->setEnabled(false);
	  break;
	case ALIGN_STATIONARY:ui->editVelThresholdInMoving->setEnabled(false);
	  ui->btnEditInitialState->setEnabled(false);
	  break;
	case ALIGN_USE_GIVEN:ui->editVelThresholdInMoving->setEnabled(false);
	  ui->btnEditInitialState->setEnabled(true);
	  break;
  }
}
void DataFusionWindow::on_cbxOutageEnable_stateChanged(int x) {
  bool outage_state = ui->cbxOutageEnable->isChecked();
  ui->editOutageStartTime->setEnabled(outage_state);
  ui->editOutageStopTime->setEnabled(outage_state);
  ui->editOutageTime->setEnabled(outage_state);
  ui->editStepTime->setEnabled(outage_state);
}
void DataFusionWindow::on_btnClearLog_clicked() {
  ui->listLog->clear();
}
void DataFusionWindow::on_actOpenConfigFile_triggered() {
  LOG(INFO) << "Open YAML file";
  QString filename = QFileDialog::getOpenFileName(this, tr("open IMU parameter file"), "", tr("YAML files(*.yml)"));
  LOG(INFO) << "Choose path:" << filename.toStdString();
  config_.LoadFrom(filename.toStdString());
  InitialUi();
}
void DataFusionWindow::on_actSaveConfigFile_triggered() {
  LOG(INFO) << "Save YAML file";
  QString filename = QFileDialog::getSaveFileName(this, tr("open IMU parameter file"), "", tr("YAML files(*.yml)"));
  LOG(INFO) << "Choose path:" << filename.toStdString();
  config_.SaveTo(filename.toStdString());
}
void DataFusionWindow::on_ProcessThreadFeedBack(int x) {
  ui->progressBar->setValue(x);
}
void DataFusionWindow::on_ProcessThreadLogFeedBack(const QString &s) {
  ShowLog(s);
}
void DataFusionWindow::SetFont() {
  int fontId = QFontDatabase::addApplicationFont("./fonts/msyh.ttc");
  QStringList fontIDs = QFontDatabase::applicationFontFamilies(fontId);
  if (! fontIDs.isEmpty()) {
	QFont font(fontIDs.first());
	QApplication::setFont(font);
  }
  else {
	ShowLog("Failed to load font.");
  }
}

