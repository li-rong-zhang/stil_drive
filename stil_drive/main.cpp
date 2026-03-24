#include "stil_drive.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    stil_drive window;
    window.show();
    return app.exec();
}
