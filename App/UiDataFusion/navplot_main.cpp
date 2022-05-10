/**
* @file navplot_main.cpp in RGIOE
* @author linfe
* @comment
* Create on 2022/5/9 22:33
* @version 1.0
**/
#include <QApplication>
#include "navplot.h"
int main(int argc,char *argv[]) {
  QApplication app(argc, argv);
  Dialog w;
  w.show();
  return app.exec();
}