#include <QApplication>
#include "datafusionwindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    DataFusionWindow w;
    w.show();
    return app.exec();
}
