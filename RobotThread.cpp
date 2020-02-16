#include "RobotThread.h"
#include <QDebug>

RobotThread::RobotThread(int argc, char** pArgv, const char * topic)
    :	m_Init_argc(argc),
      m_pInit_argv(pArgv),
      m_topic(topic),
      m_topic_send("default_topic"),
      m_isTopicNameChanged (true),
      m_speedX(0),
      m_speedY(0),
      m_speedZ(0)
{/** Constructor for the robot thread **/
    m_Mutex_Send = new QMutex();
}

RobotThread::~RobotThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if
    m_Mutex_Send->unlock();
    delete m_Mutex_Send;
    m_pThread->wait();
}//end destructor

bool RobotThread::init()
{
    m_pThread = new QThread();
    this->moveToThread(m_pThread);

    connect(m_pThread, &QThread::started, this, &RobotThread::run);
    ros::init(m_Init_argc, m_pInit_argv, "gui_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    m_nh = new ros::NodeHandle();
    //    sim_velocity  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pose_listener = m_nh->subscribe(m_topic, 10, &RobotThread::poseCallback, this);

    m_pThread->start();
    return true;
}//set up the thread

void RobotThread::poseCallback(const nav_msgs::Odometry & msg)
{
    QMutex * pMutex = new QMutex();

    pMutex->lock();
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;
    pMutex->unlock();

    delete pMutex;
    Q_EMIT newPose(m_xPos, m_yPos, m_aPos);
}//callback method to update the robot's position.

void RobotThread::setSpeed(int x, int y, int z)
{
    m_Mutex_Send->lock();
    m_speedX = x;
    m_speedY = y;
    m_speedZ = z;
    m_Mutex_Send->unlock();
}

void RobotThread::setTopicName(QString name)
{
    m_Mutex_Send->lock();
    if (name != "" && name != m_topic_send)
    {
        m_topic_send = name;
        m_isTopicNameChanged = true;
    }
    m_Mutex_Send->unlock();
}

void RobotThread::run()
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        geometry_msgs::Twist cmd_msg;
        QString newTopic;
        m_Mutex_Send->lock();
        cmd_msg.linear.x = m_speedX;
        cmd_msg.linear.y = m_speedY;
        cmd_msg.linear.z = m_speedZ;
        if (m_isTopicNameChanged)
        {
            newTopic = m_topic_send;
            m_isTopicNameChanged = false;
        }
        m_Mutex_Send->unlock();
        if (newTopic != "")
            sim_velocity  = m_nh->advertise<geometry_msgs::Twist>(newTopic.toStdString().c_str(), 100);
        sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();

    }//do ros things.
}

void RobotThread::sendMsg(QString topicName, int x, int y, int z)
{
    qDebug() << "Topic name: " << topicName << " | x: " << x << " | y: " << y << " | z: " << z;
    setTopicName(topicName);
    setSpeed(x, y, z);
}

double RobotThread::getXPos(){ return m_xPos; }
double RobotThread::getYPos(){ return m_yPos; }
double RobotThread::getAPos(){ return m_aPos; }

