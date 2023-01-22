/**
* @file DataFusionThread.h in UiDataFusion
* @author rebeater
* @comment
* Create on 11/27/21 11:28 PM
* @version 1.0
**/

#ifndef UIDATAFUSION__DATAFUSIONTHREAD_H_
#define UIDATAFUSION__DATAFUSIONTHREAD_H_
#include <QThread>
#include "Config.h"
class DataFusionThread: public QThread{
  Q_OBJECT
 private:
  Config config;
 public:
  explicit DataFusionThread(Config cfg,QObject *parent = 0);
  void run() override;
  bool Stop();
  signals:
  void SendStatus(int per);
  void SendLog(QString s);
};
#endif //UIDATAFUSION__DATAFUSIONTHREAD_H_
