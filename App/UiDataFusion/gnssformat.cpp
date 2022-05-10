//
// Created by linfe on 2022/5/9.
//

// You may need to build the project (run Qt uic code generator) to get "ui_gnssformat.h" resolved

#include "gnssformat.h"
#include "ui_gnssformat.h"

gnssformat::gnssformat(QWidget *parent) :
	QDialog(parent), ui(new Ui::gnssformat) {
  ui->setupUi(this);
}

gnssformat::~gnssformat() {
  delete ui;
}
