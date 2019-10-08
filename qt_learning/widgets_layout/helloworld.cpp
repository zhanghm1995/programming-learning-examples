#include <QPixmap>
#include <QPainter>
#include "helloworld.h"
#include "build/ui_helloworld.h"

HelloWorld::HelloWorld(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::HelloWorld)
{
    ui->setupUi(this);

    connect(ui->btn_show_gfx_img, SIGNAL(released()), this,
            SLOT(onShowImage()));
    connect(ui->btn_show_label_img, SIGNAL(released()), this,
            SLOT(onShowLabelImage()));

    ui->gfx_image->setViewportUpdateMode(
            QGraphicsView::BoundingRectViewportUpdate);
    ui->gfx_image->setCacheMode(QGraphicsView::CacheBackground);
    ui->gfx_image->setRenderHints(QPainter::Antialiasing |
                                  QPainter::SmoothPixmapTransform);

}

HelloWorld::~HelloWorld()
{
    delete ui;
}

void HelloWorld::onShowImage()
{
    scene_.reset(new QGraphicsScene);
    QPixmap* pix=new QPixmap("1.jpg");
    scene_->addPixmap(*pix);
    ui->gfx_image->setScene(scene_.get());
//    ui->gfx_image->show();
//    ui->gfx_image->resize(pix->width(), pix->height());
    ui->gfx_image->fitInView(scene_->itemsBoundingRect(),Qt::KeepAspectRatio);
}

void HelloWorld::onShowLabelImage()
{
    QPixmap* pix=new QPixmap("1.jpg");
    ui->lbl_image->setPixmap(*pix);
    ui->lbl_image->resize(ui->lbl_image->pixmap()->size());
//      ui->lbl_image->setPixmap(pix->scaled(ui->lbl_image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}