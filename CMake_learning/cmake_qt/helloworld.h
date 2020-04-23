#ifndef HELLOWORLD_H
#define HELLOWORLD_H

#include <QMainWindow>

namespace Ui {
class HelloWorld;
}

class HelloWorld : public QMainWindow
{
    Q_OBJECT

public:
    explicit HelloWorld(QWidget *parent = 0);
    ~HelloWorld();

private:
    Ui::HelloWorld *ui;
};

#endif // HELLOWORLD_H
