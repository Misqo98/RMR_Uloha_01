#include "mainwindow.h"
#include <QApplication>
#include "mmsystem.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<DataSender>();
    MainWindow w;
    w.show();

    return a.exec();
}
