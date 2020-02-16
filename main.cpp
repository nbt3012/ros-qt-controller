#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "RobotThread.h"


int main(int argc, char** argv) {
    QGuiApplication app(argc, argv);

    RobotThread m_RobotThread(argc, argv);
    m_RobotThread.init();

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.rootContext()->setContextProperty("RosController", &m_RobotThread);
    engine.load(url);

	return app.exec();
}

