//
// Created by linfe on 2022/5/9.
//

#ifndef RGIOE_APP_UIDATAFUSION_NAVPLOT_H_
#define RGIOE_APP_UIDATAFUSION_NAVPLOT_H_

#include "FileIO.h"
#include "RgioeDataType.h"
#include <QMainWindow>
#include <qwt_plot_curve.h>

QT_BEGIN_NAMESPACE
namespace Ui { class NavPlot; }
QT_END_NAMESPACE

enum NavPlotErrorCode {
  NavPlotOK = 0,
  NavPlotFileNotFound,
  NavPlotFileBadFormat,
  NavPlotFileNotSupported,
};

class NavPlot : public QMainWindow {
 Q_OBJECT

 public:
  explicit NavPlot(QMainWindow *parent = nullptr);
  ~NavPlot() override;

  NavPlotErrorCode LoadFile(const std::string &file_path,NavFileFormat fmt = NavFileFormat::NavAscii);
  void LoadAndPlotFigure(const std::string &file_path);
 protected:
  void dragEnterEvent(QDragEnterEvent *event) override;
  void dropEvent(QDropEvent *event) override;

 public slots:
  void on_actionTrack_triggered();
  void on_actionAcceBias_triggered();
  void on_actionGyroBias_triggered();
  void on_actionAcce_Scale_Factor_triggered();
  void on_actionGyro_Scale_Factor_triggered();

 private:
  template<NavFileFormat>
  NavPlotErrorCode LoadFile_(const std::string &file_path);
  void ShowTrack();
  void ShowAcceBias();
  void ShowAcceScaleFactor();
  void ShowGyroBias();
  void ShowGyroScaleFactor();
  void SetFont();
  void ClearFigure();
  void InitialFigure();
  void ShowScaleFactor();/*懒得写*/
  void ShowFigure();

 private:

  Ui::NavPlot *ui;
  std::list<NavOutput> navdata;
  /*后缀和格式映射*/
  std::map<string, NavFileFormat> extension_map{
	  {"nav", NavFileFormat::NavAscii},
	  {"rst", NavFileFormat::NavAscii},
	  {"navmat", NavFileFormat::NavDoubleMatrix},
	  {"bin", NavFileFormat::NavBinary},
	  {"navbin", NavFileFormat::NavBinary}
  };
  QColor colors[3] = {
	  {0xff, 0, 0},
	  {1, 0xff, 2},
	  {0, 0, 0xff},
  };
  QwtPlotCurve * track_curve= nullptr;
  QwtPlotCurve * bias_curves[3] = {nullptr, nullptr, nullptr};
  QwtPlotCurve * track_curves= nullptr;
};
#endif //RGIOE_APP_UIDATAFUSION_NAVPLOT_H_
