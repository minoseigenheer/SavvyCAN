#include "serialbusconnection.h"

#include "canconmanager.h"

#include <QCanBus>
#include <QCanBusFrame>
#include <QDateTime>
#include <QDebug>
#include <QThread>


/***********************************/
/****    class definition       ****/
/***********************************/

SerialBusConnection::SerialBusConnection(QString portName, QString driverName, int pBusSpeed, int pDataRate, bool pCanFd, bool pListenOnly, bool pActive) :
    CANConnection(portName, driverName, CANCon::SERIALBUS,0 ,pBusSpeed, pCanFd, pDataRate ,1, 4000, true, pListenOnly, pActive),
    mTimer(this) /*NB: set connection as parent of timer to manage it from working thread */
{
}


SerialBusConnection::~SerialBusConnection()
{
    stop();
}


void SerialBusConnection::piStarted()
{
    QString errorString;
    mDev_p = QCanBus::instance()->createDevice(getDriver(), getPort(), &errorString);
    if (!mDev_p) {
        disconnectDevice();
        qDebug() << "piStarted: createDevice failed:" << errorString;
        return;
    }

    connect(mDev_p, &QCanBusDevice::errorOccurred,  this, &SerialBusConnection::errorReceived);
    connect(mDev_p, &QCanBusDevice::framesWritten,  this, &SerialBusConnection::framesWritten);
    connect(mDev_p, &QCanBusDevice::framesReceived, this, &SerialBusConnection::framesReceived);
    connect(mDev_p, &QCanBusDevice::stateChanged,   this, &SerialBusConnection::deviceStateChanged);

    connect(&mTimer, SIGNAL(timeout()), this, SLOT(testConnection()));
    mTimer.setInterval(1000);
    mTimer.setSingleShot(false);
    mTimer.start();

    mBusData[0].mConfigured = true;

    if (mBusData[0].mBus.isActive() && mBusData[0].mBus.getSpeed() > 0) {
        applyBusConfig(mBusData[0].mBus);
        mDev_p->connectDevice();
    }
}


void SerialBusConnection::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void SerialBusConnection::piStop() {
    qDebug() << "piStop()";
    mTimer.stop();
    disconnectDevice();
}


bool SerialBusConnection::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    return getBusConfig(pBusIdx, pBus);
}


void SerialBusConnection::piSetBusSettings(int pBusIdx, CANBus bus)
{
    if (pBusIdx != 0 || !mDev_p) return;
    disconnectDevice();
    setStatus(CANCon::NOT_CONNECTED);
    setBusConfig(0, bus);
    if (!bus.isActive() || mDev_p->state() != QCanBusDevice::UnconnectedState) return;
    applyBusConfig(bus);
    mDev_p->connectDevice();
}


bool SerialBusConnection::piSendFrame(const CommFrame& pFrame)
{
    /* sanity checks */
    if(0 != pFrame.getBus() /*|| pFrame.len>8*/)
        return false;
    if (!mDev_p) return false;

    //the CommFrame class is based upon QCanBusFrame so most everything lines up but they are not the same
    QCanBusFrame frame;
    frame.setFrameId(pFrame.frameId());
    frame.setPayload(pFrame.payload());
    frame.setExtendedFrameFormat(pFrame.hasExtendedFrameFormat());
    frame.setBitrateSwitch(pFrame.hasBitrateSwitch());
    frame.setFlexibleDataRateFormat(pFrame.hasFlexibleDataRateFormat());

    return mDev_p->writeFrame(frame);
}


/***********************************/
/****   private methods         ****/
/***********************************/


/* disconnect device */
void SerialBusConnection::disconnectDevice() {
    if(mDev_p && mDev_p->state() == QCanBusDevice::ConnectedState){
        mDev_p->disconnectDevice();
    }
}

void SerialBusConnection::applyBusConfig(const CANBus &bus)
{
    quint32 cfg = bus.isListenOnly() ? EN_SILENT_MODE : 0;
    mDev_p->setConfigurationParameter(QCanBusDevice::BitRateKey,      bus.getSpeed());
    mDev_p->setConfigurationParameter(QCanBusDevice::CanFdKey,        bus.isCanFD());
    if (bus.isCanFD() && bus.getDataRate() > 0)
        mDev_p->setConfigurationParameter(QCanBusDevice::DataBitRateKey, bus.getDataRate());
    mDev_p->setConfigurationParameter(QCanBusDevice::UserKey, cfg);
}

/* Immediately update status when the device state changes */
void SerialBusConnection::deviceStateChanged(QCanBusDevice::CanBusDeviceState state)
{
    CANConStatus stats;
    if (state == QCanBusDevice::ConnectedState) {
        setStatus(CANCon::CONNECTED);
        mHadFrameSinceConnect = false; // reset; will be set true when first frame arrives
        mReconnectAttempts = 0;
        stats.conStatus = getStatus();
        stats.numHardwareBuses = mNumBuses;
        emit status(stats);
    } else if (state == QCanBusDevice::UnconnectedState) {
        setStatus(CANCon::NOT_CONNECTED);
        mNoFrameSeconds = 0;
        mHadFrameSinceConnect = false;
        // mReconnectAttempts intentionally NOT reset here — it must accumulate
        // across failed connectDevice() calls so recreateDevice() eventually fires.
        stats.conStatus = getStatus();
        stats.numHardwareBuses = mNumBuses;
        emit status(stats);
    }
}

void SerialBusConnection::errorReceived(QCanBusDevice::CanBusError error)
{
    switch (error) {
        case QCanBusDevice::ConnectionError:
            /* Device may have been physically unplugged — force status update in case
               stateChanged does not fire (e.g. PCAN USB on some platforms). */
            qWarning() << "CAN connection error:" << mDev_p->errorString();
            if (getStatus() == CANCon::CONNECTED) {
                setStatus(CANCon::NOT_CONNECTED);
                CANConStatus stats;
                stats.conStatus = getStatus();
                stats.numHardwareBuses = mNumBuses;
                emit status(stats);
            }
            break;
        case QCanBusDevice::ReadError:
        case QCanBusDevice::WriteError:
        case QCanBusDevice::ConfigurationError:
        case QCanBusDevice::UnknownError:
            qWarning() << mDev_p->errorString();
            break;
        default:
            break;
    }
}

void SerialBusConnection::framesWritten(qint64 count)
{
    Q_UNUSED(count);
    //qDebug() << "Number of frames written:" << count;
}

void SerialBusConnection::framesReceived()
{
    uint64_t timeBasis = CANConManager::getInstance()->getTimeBasis();

    /* sanity checks */
    if(!mDev_p)
        return;

    /* read frame */
    while(true)
    {
        const QCanBusFrame recFrame = mDev_p->readFrame();

        /* exit case */
        if(!recFrame.isValid())
            break;

        /* A valid frame confirms the connection is alive — reset the idle counter and
         * mark that we have seen at least one frame on this connection.
         * (Inside the loop so that spurious framesReceived() signals with no valid data
         * do not prevent the 0-fps USB-unplug probe.) */
        mNoFrameSeconds = 0;
        mHadFrameSinceConnect = true;

        /* drop frame if capture is suspended */
        if(isCapSuspended())
            continue;

        /* check frame */
        //if (recFrame.payload().length() <= 8) {
        if (true) {
            CommFrame* frame_p = getQueue().get();
            if(frame_p) {
                frame_p->setPayload(recFrame.payload());
                frame_p->setBus(0);
                // All frames arriving via framesReceived() are genuinely received frames.
                // Do not use QCanBusFrame::hasLocalEcho() to determine direction: some backends
                // (e.g. PeakCAN/PCAN on certain platforms) report local echo unreliably,
                // causing all received frames to appear as Tx.
                frame_p->setReceived(true);

                if (recFrame.frameType() == recFrame.ErrorFrame)
                {
                    frame_p->setExtendedFrameFormat(recFrame.hasExtendedFrameFormat());
                    frame_p->setFrameId(recFrame.frameId() + 0x20000000ull);
                }
                else
                {
                    frame_p->setExtendedFrameFormat(recFrame.hasExtendedFrameFormat());
                    frame_p->setFrameId(recFrame.frameId());
                }
                //the timestamp structs are identical in theory but the compiler does not know or respect that. Best to regenerate
                CommFrame::TimeStamp stamp;
                stamp.fromMicroSeconds(recFrame.timeStamp().seconds() * 1000000ull + recFrame.timeStamp().microSeconds());
                frame_p->setTimeStamp(stamp);

                //Ditto for error but not frametype. That one is different

                frame_p->setFrameTypeFromQCan(recFrame.frameType());

                //use conversion to and from integer to patch over the difference in structures (there is no practical difference)
                CommFrame::FrameErrors errs;
                errs.fromInt(recFrame.error().toInt());
                frame_p->setError(errs);

                if (useSystemTime) {
                    frame_p->setTimeStamp(CommFrame::TimeStamp::fromMicroSeconds(QDateTime::currentMSecsSinceEpoch() * 1000ul));
                }
                else {
                    uint64_t hwTimestamp = static_cast<uint64_t>(recFrame.timeStamp().seconds()) * 1000000ULL
                                          + static_cast<uint64_t>(recFrame.timeStamp().microSeconds());
                    uint64_t relTimestamp;
                    if (hwTimestamp >= timeBasis) {
                        // Epoch-based hw timestamp (e.g. SocketCAN): subtract session basis.
                        relTimestamp = hwTimestamp - timeBasis;
                    } else {
                        // Uptime-based hw timestamp (e.g. PCAN): the hw clock cannot be
                        // synchronised to the elapsed timer without a fixed anchor bias,
                        // so use the elapsed timer directly — the same source as Tx timestamps.
                        // This guarantees Tx and Rx share one consistent clock with no drift.
                        relTimestamp = CANConManager::getInstance()->getElapsedUs();
                    }
                    frame_p->setTimeStamp(CommFrame::TimeStamp(0, relTimestamp));
                }

                checkTargettedFrame(*frame_p);

                /* enqueue frame */
                getQueue().queue();
            }
            else
                qDebug() << "can't get a frame, ERROR";
        }
    }
}

/*
enum CanBusDeviceState {
    UnconnectedState,
    ConnectingState,
    ConnectedState,
    ClosingState
};
*/

void SerialBusConnection::testConnection()
{
    if (!mDev_p) return;
    const auto devState = mDev_p->state();
    if (devState == QCanBusDevice::ConnectingState || devState == QCanBusDevice::ClosingState) return;

    const CANCon::status s = getStatus();
    CANConStatus stats;
    stats.numHardwareBuses = mNumBuses;

    if (s == CANCon::CONNECTED) {
        if (devState == QCanBusDevice::UnconnectedState) {
            // Fallback: backend went Unconnected without emitting stateChanged
            setStatus(CANCon::NOT_CONNECTED);
            mNoFrameSeconds = 0;
            stats.conStatus = CANCon::NOT_CONNECTED;
            emit status(stats);
        } else if (mHadFrameSinceConnect && ++mNoFrameSeconds >= 5) {
            // 5 s of no frames after being active — force disconnect so NOT_CONNECTED retry kicks in
            mNoFrameSeconds = 0;
            mDev_p->disconnectDevice();
        }
    } else {
        if (devState == QCanBusDevice::ConnectedState) {
            // Fallback: backend reconnected silently
            setStatus(CANCon::CONNECTED);
            mNoFrameSeconds = 0;
            mReconnectAttempts = 0;
            stats.conStatus = CANCon::CONNECTED;
            emit status(stats);
        } else if (devState == QCanBusDevice::UnconnectedState) {
            CANBus bus;
            if (getBusConfig(0, bus) && bus.isActive() && bus.getSpeed() > 0) {
                if ((++mReconnectAttempts % 10) == 0) { recreateDevice(); return; }
                applyBusConfig(bus);
                mDev_p->connectDevice();
            }
        }
    }
}

void SerialBusConnection::recreateDevice()
{
    if (mDev_p) {
        disconnect(mDev_p, nullptr, this, nullptr);
        if (mDev_p->state() != QCanBusDevice::UnconnectedState)
            mDev_p->disconnectDevice();
        delete mDev_p; mDev_p = nullptr;
    }

    QString err;
    mDev_p = QCanBus::instance()->createDevice(getDriver(), getPort(), &err);
    if (!mDev_p) { qDebug() << "recreateDevice: createDevice failed:" << err; return; }

    connect(mDev_p, &QCanBusDevice::errorOccurred,  this, &SerialBusConnection::errorReceived);
    connect(mDev_p, &QCanBusDevice::framesWritten,  this, &SerialBusConnection::framesWritten);
    connect(mDev_p, &QCanBusDevice::framesReceived, this, &SerialBusConnection::framesReceived);
    connect(mDev_p, &QCanBusDevice::stateChanged,   this, &SerialBusConnection::deviceStateChanged);

    CANBus bus;
    if (!getBusConfig(0, bus) || !bus.isActive() || bus.getSpeed() <= 0) return;
    applyBusConfig(bus);
    mDev_p->connectDevice();
}

// Both probeConnectionState (called on window open) and autoReconnect (called from main-thread
// timer) perform the same operation: disconnect if needed, apply stored config, reconnect.
void SerialBusConnection::probeConnectionState() { autoReconnect(); }

void SerialBusConnection::autoReconnect()
{
    if (!mDev_p) { recreateDevice(); return; }
    const auto devState = mDev_p->state();
    if (devState == QCanBusDevice::ConnectingState || devState == QCanBusDevice::ClosingState) return;
    CANBus bus;
    if (!getBusConfig(0, bus) || bus.getSpeed() <= 0 || !bus.isActive()) return;
    if (devState == QCanBusDevice::ConnectedState) mDev_p->disconnectDevice();
    applyBusConfig(bus);
    if (!mDev_p->connectDevice() && (++mReconnectAttempts % 10) == 0)
        recreateDevice();
}
