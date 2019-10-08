#ifndef HELLOWORLD_H
#define HELLOWORLD_H

#include <memory>

#include <QMainWindow>
#include <QGraphicsScene>

namespace Ui {
class HelloWorld;
}

class HelloWorld : public QMainWindow
{
    Q_OBJECT

public:
    explicit HelloWorld(QWidget *parent = 0);
    ~HelloWorld();

private slots:
    void onShowImage();
    void onShowLabelImage();

private:
    Ui::HelloWorld *ui;

    std::unique_ptr<QGraphicsScene> scene_;
};

#endif // HELLOWORLD_H
