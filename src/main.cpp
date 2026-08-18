#include <QApplication>

#include "ui/mainwindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}

// #include <iostream>
// int main(){
//     std::cout << "Hello, World!" << std::endl;
//     return 0;

// }