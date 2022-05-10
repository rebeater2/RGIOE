//
// Created by linfe on 2022/5/9.
//

// You may need to build the project (run Qt uic code generator) to get "ui_navplot.h" resolved

#include "navplot.h"
#include "ui_navplot.h"

Dialog::Dialog(QDialog *parent) :
	QDialog(parent), ui(new Ui::Dialog) {
  ui->setupUi(this);
}

Dialog::~Dialog() {
  delete ui;
}
