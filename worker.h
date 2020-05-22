#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include "CImg.h"
using namespace cimg_library;

class Worker : public QThread
{
    Q_OBJECT
public:
    Worker(const CImg<double> *map=nullptr);
    ~Worker() override;
    QMutex mutex;
private:
    void run() override;
    const CImg<double> *image;
signals:

public slots:
};

#endif // WORKER_H
