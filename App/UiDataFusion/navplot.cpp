//
// Created by linfe on 2022/5/9.
//

// You may need to build the project (run Qt uic code generator) to get "ui_navplot.h" resolved

#include <glog/logging.h>
#include <fmt/format.h>
#include "navplot.h"
#include "ui_navplot.h"

#include <QEnterEvent>
#include <QMimeData>
#include <set>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_legend.h>
#include <QFontDatabase>

NavPlot::NavPlot(QMainWindow *parent) :
	QMainWindow(parent), ui(new Ui::NavPlot) {
  ui->setupUi(this);
  setAcceptDrops(true);
  SetFont();
  InitialFigure();
}

NavPlot::~NavPlot() {
  ClearFigure();
  delete ui;
}
template<>
NavPlotErrorCode NavPlot::LoadFile_<NavBinary>(const std::string &file_path) {
  ifstream ifs;
  NavOutput nav;
  ifs.open(file_path, std::ios::binary);
  if (!ifs.good()) return NavPlotFileNotFound;
  ifs.seekg(0, std::ios::end);
  size_t size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  LOG(INFO) << fmt::format("Loading... [file size = {:.4f} MB] \n", (double)size / 1024.0 / 1024.0);
  navdata.clear();
//  navdata.reserve(size / (sizeof(nav) + 1));
  while (!ifs.eof()) {
	ifs.read(reinterpret_cast<char *>(&nav), sizeof(nav));
	navdata.push_back(nav);
  }
  LOG(INFO) << fmt::format("Time range form:{:.5f} to {:.5f}", navdata.front().gpst, navdata.back().gpst);
  return NavPlotOK;
}
template<>
NavPlotErrorCode NavPlot::LoadFile_<NavAscii>(const std::string &file_path) {
  ifstream ifs(file_path);
  if (!ifs.good()) return NavPlotFileNotFound;
  while (!ifs.eof()) {
	string str;
	NavOutput nav;
	std::getline(ifs, str);
	std::stringstream ss(str);
	ss >> nav.week >> nav.gpst >> nav.lat >> nav.lon >> nav.height >> nav.vn[0] >> nav.vn[1] >> nav.vn[2] >> nav.atti[0]
	   >> nav.atti[1] >> nav.atti[2] >> nav.gb[0] >> nav.gb[1] >> nav.gb[2] >> nav.ab[0] >> nav.ab[1] >> nav.ab[2];
	navdata.push_back(nav);
  }
  return NavPlotOK;
}
template<>
NavPlotErrorCode NavPlot::LoadFile_<NavDoubleMatrix>(const std::string &file_path) {
  /*TODO*/
  return NavPlotFileNotSupported;
}
NavPlotErrorCode NavPlot::LoadFile(const std::string &file_path, NavFileFormat fmt) {
  switch (fmt) {
	case NavFileFormat::NavBinary: return LoadFile_<NavFileFormat::NavBinary>(file_path);
	case NavFileFormat::NavAscii:return LoadFile_<NavFileFormat::NavAscii>(file_path);
	case NavFileFormat::NavDoubleMatrix:return LoadFile_<NavFileFormat::NavDoubleMatrix>(file_path);
	default: return NavPlotFileBadFormat;
  }
}
void NavPlot::dragEnterEvent(QDragEnterEvent *event) {
  LOG(INFO) << __FUNCTION__ << event->mimeData()->text().toStdString();
  event->acceptProposedAction();
}
void NavPlot::dropEvent(QDropEvent *event) {
  event->acceptProposedAction();
}
void NavPlot::LoadAndPlotFigure(const std::string &file_path) {
  LOG(INFO) << "Load " << file_path;
  size_t index = file_path.find_last_of('.');
  if (index < 0) return;
  string extension = file_path.substr(index + 1);

  if (extension_map.find(extension) != extension_map.end()) {
	LOG(INFO) << fmt::format("The extension is {}", extension);
	auto err = LoadFile(file_path, extension_map[extension]);
	if (err== NavPlotOK)
	  ShowTrack();
	else{
	  LOG(ERROR)<<fmt::format("{} load failed {:d}",file_path,err);
	}
  } else {
	LOG(ERROR) << fmt::format("Unsupported format {}", extension);//"Unsupported extension";
	std::string temperate;
	auto f1 = [&temperate](const std::pair<std::string, NavFileFormat> &s) {
	  temperate = temperate + "," + s.first;
	};
	std::for_each(extension_map.begin(), extension_map.end(), f1);
	LOG(INFO) << "Supported extensions are:" << temperate;
	return;
  }
//  LoadFile(file_path);
}
void NavPlot::ShowTrack() {
  if (navdata.empty()) {
	LOG(ERROR) << "navigation data is empty";
	return;
  }
  ClearFigure();
  QPolygonF p;
  for (auto &d: navdata) {
	p << QPointF(d.lon, d.lat);
  }
  if (track_curve == nullptr)
	track_curve = new QwtPlotCurve("track");   //设置曲线
  track_curve->setSamples(p);
  track_curve->setPen(QColor{0, 0, 0xff}, 2, Qt::SolidLine);    //设置画笔(颜色,粗细,线条样式)
  track_curve->attach(ui->qwtPlot);   //把曲线附加到qwtPlot上
  track_curve->setCurveAttribute(QwtPlotCurve::Fitted, true);   //曲线光滑
  /*添加图例*/
  QwtLegend *legend = new QwtLegend();
  legend->setDefaultItemMode(QwtLegendData::Checkable);//图例可被点击
  ui->qwtPlot->insertLegend(legend, QwtPlot::RightLegend);  //右侧显示图例
  ui->qwtPlot->setAxisAutoScale(QwtPlot::xBottom, true);
  ui->qwtPlot->setAxisAutoScale(QwtPlot::yLeft, true);
  ui->qwtPlot->setAxisTitle(QwtPlot::xBottom, "East");
  ui->qwtPlot->setAxisTitle(QwtPlot::yLeft, "North");
  /*ui界面显示曲线*/
  ui->qwtPlot->replot();
  ui->qwtPlot->setAutoReplot(true);   //自动更新
//  delete curve;
}
void NavPlot::ShowAcceBias() {
  if (navdata.empty()) {
	LOG(ERROR) << "Navigation data is empty";
	return;
  }
  ClearFigure();
  QPolygonF point[3];
  QString titles[3] = {
	  "bias x", "bias y", "bias z",
  };
  for (int i = 0; i < 3; i++) {
	for (auto &d: navdata) {
	  point[i] << QPointF(d.gpst, d.ab[i]);
	}
	if (bias_curves[i] == nullptr)
	  bias_curves[i] = new QwtPlotCurve(titles[i]);   //设置曲线
	bias_curves[i]->setSamples(point[i]);
	bias_curves[i]->setPen(colors[i], 2, Qt::SolidLine);    //设置画笔(颜色,粗细,线条样式)
	bias_curves[i]->attach(ui->qwtPlot);   //把曲线附加到qwtPlot
	bias_curves[i]->setCurveAttribute(QwtPlotCurve::Fitted, true);   //曲线光滑
  }
  ui->qwtPlot->axisAutoScale(QwtPlot::xBottom);
  ui->qwtPlot->axisAutoScale(QwtPlot::yLeft);
  ui->qwtPlot->setAxisScale(QwtPlot::xBottom, navdata.front().gpst, navdata.back().gpst);
  ui->qwtPlot->setAxisTitle(QwtPlot::xBottom, "GPS Time/s");
  ui->qwtPlot->setAxisTitle(QwtPlot::yLeft, "Accelerator Bias/mGal");
  ui->qwtPlot->replot();
  ui->qwtPlot->setAutoReplot(true);   //自动更新
}
void NavPlot::SetFont() {
  int fontId = QFontDatabase::addApplicationFont("./fonts/msyh.ttc");
  QStringList fontIDs = QFontDatabase::applicationFontFamilies(fontId);
  if (!fontIDs.isEmpty()) {
	QFont font(fontIDs.first());
	QApplication::setFont(font);
  } else {
	LOG(ERROR) << "font file is missing";
  }
}
void NavPlot::ShowGyroBias() {
  if (navdata.empty()) {
	LOG(ERROR) << "Navigation data is empty";
	return;
  }
  ClearFigure();
  QPolygonF point[3];
  QString titles[3] = {
	  "gyroscope bias x", "gyroscope bias y", "gyroscope bias z",
  };
  for (int i = 0; i < 3; i++) {
	for (auto &d: navdata) {
	  point[i] << QPointF(d.gpst, d.gb[i]);
	}
	if (bias_curves[i] == nullptr)
	  bias_curves[i] = new QwtPlotCurve(titles[i]);   //设置曲线
	bias_curves[i]->setSamples(point[i]);
	bias_curves[i]->setPen(colors[i], 2, Qt::SolidLine);    //设置画笔(颜色,粗细,线条样式)
	bias_curves[i]->attach(ui->qwtPlot);   //把曲线附加到qwtPlot
	bias_curves[i]->setCurveAttribute(QwtPlotCurve::Fitted, true);   //曲线光滑
  }
  ui->qwtPlot->axisAutoScale(QwtPlot::xBottom);
  ui->qwtPlot->axisAutoScale(QwtPlot::yLeft);
  ui->qwtPlot->setAxisScale(QwtPlot::xBottom, navdata.front().gpst, navdata.back().gpst);
  ui->qwtPlot->setAxisTitle(QwtPlot::xBottom, "GPS Time/s");
  ui->qwtPlot->setAxisTitle(QwtPlot::yLeft, "Gyroscope Bias/(Deg/h)");
  ui->qwtPlot->replot();
  ui->qwtPlot->setAutoReplot(true);   //自动更新
}
void NavPlot::on_actionTrack_triggered() {
  ShowTrack();
}
void NavPlot::ClearFigure() {
//  for(auto &p:point){
//	p.clear();
//  }
  delete track_curve;
  track_curve = nullptr;
  for (auto &p: bias_curves) {
	delete p;
	p = nullptr;
  };
}
void NavPlot::on_actionAcceBias_triggered() {
  ShowAcceBias();
}
void NavPlot::on_actionGyroBias_triggered() {
  ShowGyroBias();
}
void NavPlot::InitialFigure() {
  /*添加网格*/
  auto *grid = new QwtPlotGrid();
  grid->enableX(true);//设置网格线
  grid->enableY(true);
  grid->setMajorPen(Qt::black, 1, Qt::DashLine);
  grid->attach(ui->qwtPlot);
}
void NavPlot::on_actionAcce_Scale_Factor_triggered() {
  ShowAcceScaleFactor();
}
void NavPlot::on_actionGyro_Scale_Factor_triggered() {
  ShowGyroScaleFactor();
}
void NavPlot::ShowAcceScaleFactor() {
  if (navdata.empty()) {
	LOG(ERROR) << "Navigation data is empty";
	return;
  }
  ClearFigure();
  QPolygonF point[3];
  QString titles[3] = {
	  "Accelerator scale factor x", "Accelerator scale factor y", "Accelerator scale factor z",
  };
  for (int i = 0; i < 3; i++) {
	for (auto &d: navdata) {
	  point[i] << QPointF(d.gpst, d.as[i]);
	}
	if (bias_curves[i] == nullptr)
	  bias_curves[i] = new QwtPlotCurve(titles[i]);   //设置曲线
	bias_curves[i]->setSamples(point[i]);
	bias_curves[i]->setPen(colors[i], 2, Qt::SolidLine);    //设置画笔(颜色,粗细,线条样式)
	bias_curves[i]->attach(ui->qwtPlot);   //把曲线附加到qwtPlot
	bias_curves[i]->setCurveAttribute(QwtPlotCurve::Fitted, true);   //曲线光滑
  }
  ui->qwtPlot->axisAutoScale(QwtPlot::xBottom);
  ui->qwtPlot->axisAutoScale(QwtPlot::yLeft);
  ui->qwtPlot->setAxisScale(QwtPlot::xBottom, navdata.front().gpst, navdata.back().gpst);
  ui->qwtPlot->setAxisTitle(QwtPlot::xBottom, "GPS Time/s");
  ui->qwtPlot->setAxisTitle(QwtPlot::yLeft, "Accelerator Scale Factor/ppm");
  ui->qwtPlot->replot();
  ui->qwtPlot->setAutoReplot(true);   //自动更新
}
void NavPlot::ShowGyroScaleFactor() {
  if (navdata.empty()) {
	LOG(ERROR) << "Navigation data is empty";
	return;
  }
  ClearFigure();
  QPolygonF point[3];
  QString titles[3] = {
	  "gyroscope scale factor x", "gyroscope scale factor y", "gyroscope scale factor z",
  };
  for (int i = 0; i < 3; i++) {
	for (auto &d: navdata) {
	  point[i] << QPointF(d.gpst, d.gs[i]);
	}
	if (bias_curves[i] == nullptr)
	  bias_curves[i] = new QwtPlotCurve(titles[i]);   //设置曲线
	bias_curves[i]->setSamples(point[i]);
	bias_curves[i]->setPen(colors[i], 2, Qt::SolidLine);    //设置画笔(颜色,粗细,线条样式)
	bias_curves[i]->attach(ui->qwtPlot);   //把曲线附加到qwtPlot
	bias_curves[i]->setCurveAttribute(QwtPlotCurve::Fitted, true);   //曲线光滑
  }
  ui->qwtPlot->axisAutoScale(QwtPlot::xBottom);
  ui->qwtPlot->axisAutoScale(QwtPlot::yLeft);
  ui->qwtPlot->setAxisScale(QwtPlot::xBottom, navdata.front().gpst, navdata.back().gpst);
  ui->qwtPlot->setAxisTitle(QwtPlot::xBottom, "GPS Time/s");
  ui->qwtPlot->setAxisTitle(QwtPlot::yLeft, "Gyroscope Scale Factor/ppm");
  ui->qwtPlot->replot();
  ui->qwtPlot->setAutoReplot(true);   //自动更新
}

