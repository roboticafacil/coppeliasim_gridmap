#include "Worker.h"
#include <stdio.h>
#include <iostream>
#include <QThread>

Worker::Worker(const CImg<double> *map) : QThread(), image(map)
{
}

Worker::~Worker()
{
    if (isRunning())
    {
        requestInterruption();
        wait();
    }
}

void Worker::run()
{
    mutex.lock();
    CImgDisplay window(image->width(),image->height(),"Map");
    mutex.unlock();
    std::cout << "Window created" << std::endl;
    while (!window.is_closed() && !window.is_keyESC()&& !window.is_keyQ())
    {
        mutex.lock();
        if (window.is_resized())
            window.resize().display(*image);
        else
            window.display(*image);
        mutex.unlock();
        if (QThread::currentThread()->isInterruptionRequested())
        {
            std::cout << "Window destroyed" << std::endl;
            return;
        }
        cimg::wait(50);
    }
}
