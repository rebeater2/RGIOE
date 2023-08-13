//
// Created by linfe on 2022/5/9.
//

#ifndef RGIOE_APP_UIDATAFUSION_GNSSFORMAT_H_
#define RGIOE_APP_UIDATAFUSION_GNSSFORMAT_H_

#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui { class gnssformat; }
QT_END_NAMESPACE

class gnssformat : public QDialog {
Q_OBJECT

public:
    explicit gnssformat(QWidget *parent = nullptr);

    ~gnssformat() override;

private:
    Ui::gnssformat *ui;
};

#endif //RGIOE_APP_UIDATAFUSION_GNSSFORMAT_H_
