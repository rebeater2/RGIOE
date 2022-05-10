//
// Created by linfe on 2022/5/9.
//

#ifndef RGIOE_APP_UIDATAFUSION_NAVPLOT_H_
#define RGIOE_APP_UIDATAFUSION_NAVPLOT_H_

#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui { class Dialog; }
QT_END_NAMESPACE

class Dialog : public QDialog {
 Q_OBJECT

 public:
  explicit Dialog(QDialog *parent = nullptr);
  ~Dialog() override;

 private:
  Ui::Dialog *ui;
};

#endif //RGIOE_APP_UIDATAFUSION_NAVPLOT_H_
