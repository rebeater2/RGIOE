//
// Created by rebeater on 11/23/21.
//

#ifndef UIDATAFUSION__DATAFUSIONWINDOW_H_
#define UIDATAFUSION__DATAFUSIONWINDOW_H_
#include "Config.h"
#include "DataFusionThread.h"
#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class DataFusionWindow; }
QT_END_NAMESPACE

class DataFusionWindow : public QMainWindow {
 Q_OBJECT

 public:
  explicit DataFusionWindow(QMainWindow *parent = nullptr);
  ~DataFusionWindow() override;

 private:
  DataFusionThread *fusion_thread;
  Ui::DataFusionWindow *ui;
  Config config_;
  bool ui_set_ok{true};
  void ShowLog(const QString &s);
  void OnConvertResult(const QString &s, bool ok);
  void InitialUi();
  void SetWheelInfoEnable();
 public slots:
  void on_btnStart_clicked();
  void on_btnBrowseImuFile_clicked();
  void on_btnBrowseImuParameterFile_clicked();
  void on_btnEditImuParameterFile_clicked();
  void on_btnBrowseGnss_clicked();
  void on_cbxDoubleAntennaEnable_stateChanged(int x);
  void on_cbxOdometerEnable_stateChanged(int x);
  void on_cbxNhcEnable_stateChanged(int x);
  void on_cbxZuptEnable_stateChanged(int x);
  void on_cbxZuptaEnable_stateChanged(int x);
  void on_btnOpenZuptDetector_clicked();
  void on_cbxAlignMode_currentIndexChanged(int x);
  void on_cbxOutageEnable_stateChanged(int x);
  void on_btnClearLog_clicked();
  void on_actOpenConfigFile_triggered();
  void on_actSaveConfigFile_triggered();
  void on_ProcessThreadFeedBack(int x);
  void on_ProcessThreadLogFeedBack(const QString &s);
};

#endif //UIDATAFUSION__DATAFUSIONWINDOW_H_
