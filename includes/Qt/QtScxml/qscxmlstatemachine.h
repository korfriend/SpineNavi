// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR LGPL-3.0-only OR GPL-2.0-only OR GPL-3.0-only

#ifndef QSCXMLSTATEMACHINE_H
#define QSCXMLSTATEMACHINE_H

#include <QtScxml/qscxmldatamodel.h>
#include <QtScxml/qscxmlerror.h>
#include <QtScxml/qscxmlevent.h>
#include <QtScxml/qscxmlcompiler.h>
#include <QtScxml/qscxmlinvokableservice.h>

#include <QtCore/qlist.h>
#include <QtCore/qpointer.h>
#include <QtCore/qstring.h>
#include <QtCore/qurl.h>
#include <QtCore/qvariant.h>

#include <functional>

Q_MOC_INCLUDE(qscxmltabledata.h)

QT_BEGIN_NAMESPACE
class QIODevice;
class QXmlStreamWriter;
class QTextStream;
class QScxmlTableData;

class QScxmlStateMachinePrivate;
class Q_SCXML_EXPORT QScxmlStateMachine: public QObject
{
    Q_DECLARE_PRIVATE(QScxmlStateMachine)
    Q_OBJECT
    Q_PROPERTY(bool running READ isRunning WRITE setRunning NOTIFY runningChanged)
    Q_PROPERTY(bool initialized READ isInitialized
               NOTIFY initializedChanged BINDABLE bindableInitialized)
    Q_PROPERTY(QScxmlDataModel *dataModel READ dataModel WRITE setDataModel
               NOTIFY dataModelChanged BINDABLE bindableDataModel)
    Q_PROPERTY(QVariantMap initialValues READ initialValues WRITE setInitialValues
               NOTIFY initialValuesChanged BINDABLE bindableInitialValues)
    Q_PROPERTY(QList<QScxmlInvokableService*> invokedServices READ invokedServices
               NOTIFY invokedServicesChanged BINDABLE bindableInvokedServices)
    Q_PROPERTY(QString sessionId READ sessionId CONSTANT)
    Q_PROPERTY(QString name READ name CONSTANT)
    Q_PROPERTY(bool invoked READ isInvoked CONSTANT)
    Q_PROPERTY(QList<QScxmlError> parseErrors READ parseErrors CONSTANT)
    Q_PROPERTY(QScxmlCompiler::Loader *loader READ loader WRITE setLoader
               NOTIFY loaderChanged BINDABLE bindableLoader)
    Q_PROPERTY(QScxmlTableData *tableData READ tableData WRITE setTableData
               NOTIFY tableDataChanged BINDABLE bindableTableData)

protected:
    explicit QScxmlStateMachine(const QMetaObject *metaObject, QObject *parent = nullptr);
    QScxmlStateMachine(QScxmlStateMachinePrivate &dd, QObject *parent = nullptr);

public:
    static QScxmlStateMachine *fromFile(const QString &fileName);
    static QScxmlStateMachine *fromData(QIODevice *data, const QString &fileName = QString());
    QList<QScxmlError> parseErrors() const;

    QString sessionId() const;

    bool isInvoked() const;
    bool isInitialized() const;
    QBindable<bool> bindableInitialized() const;

    void setDataModel(QScxmlDataModel *model);
    QScxmlDataModel *dataModel() const;
    QBindable<QScxmlDataModel*> bindableDataModel();

    void setLoader(QScxmlCompiler::Loader *loader);
    QScxmlCompiler::Loader *loader() const;
    QBindable<QScxmlCompiler::Loader*> bindableLoader();

    bool isRunning() const;
    void setRunning(bool running);

    QVariantMap initialValues();
    void setInitialValues(const QVariantMap &initialValues);
    QBindable<QVariantMap> bindableInitialValues();

    QString name() const;
    Q_INVOKABLE QStringList stateNames(bool compress = true) const;
    Q_INVOKABLE QStringList activeStateNames(bool compress = true) const;
    Q_INVOKABLE bool isActive(const QString &scxmlStateName) const;

    QMetaObject::Connection connectToState(const QString &scxmlStateName,
                                           const QObject *receiver, const char *method,
                                           Qt::ConnectionType type = Qt::AutoConnection);

    // connect state to a QObject slot
    template <typename PointerToMemberFunction>
    inline QMetaObject::Connection connectToState(
            const QString &scxmlStateName,
            const typename QtPrivate::FunctionPointer<PointerToMemberFunction>::Object *receiver,
            PointerToMemberFunction method,
            Qt::ConnectionType type = Qt::AutoConnection)
    {
        typedef QtPrivate::FunctionPointer<PointerToMemberFunction> SlotType;
        return connectToStateImpl(
                    scxmlStateName, receiver, nullptr,
                    new QtPrivate::QSlotObject<PointerToMemberFunction,
                    typename SlotType::Arguments, void>(method),
                    type);
    }

    // connect state to a functor or function pointer (without context)
    template <typename Functor>
    inline typename std::enable_if<
            !QtPrivate::FunctionPointer<Functor>::IsPointerToMemberFunction &&
            !std::is_same<const char*, Functor>::value, QMetaObject::Connection>::type
    connectToState(const QString &scxmlStateName, Functor functor,
                   Qt::ConnectionType type = Qt::AutoConnection)
    {
        // Use this as context
        return connectToState(scxmlStateName, this, functor, type);
    }

    // connectToState to a functor or function pointer (with context)
    template <typename Functor>
    inline typename std::enable_if<
            !QtPrivate::FunctionPointer<Functor>::IsPointerToMemberFunction &&
            !std::is_same<const char*, Functor>::value, QMetaObject::Connection>::type
    connectToState(const QString &scxmlStateName, const QObject *context, Functor functor,
                   Qt::ConnectionType type = Qt::AutoConnection)
    {
        QtPrivate::QSlotObjectBase *slotObj = new QtPrivate::QFunctorSlotObject<Functor, 1,
                QtPrivate::List<bool>, void>(functor);
        return connectToStateImpl(scxmlStateName, context, reinterpret_cast<void **>(&functor),
                                  slotObj, type);
    }

    //! [onentry]
    static std::function<void(bool)> onEntry(const QObject *receiver, const char *method)
    {
        const QPointer<QObject> receiverPointer(const_cast<QObject *>(receiver));
        return [receiverPointer, method](bool isEnteringState) {
            if (isEnteringState && !receiverPointer.isNull())
                QMetaObject::invokeMethod(const_cast<QObject *>(receiverPointer.data()), method);
        };
    }

    //! [onexit]
    static std::function<void(bool)> onExit(const QObject *receiver, const char *method)
    {
        const QPointer<QObject> receiverPointer(const_cast<QObject *>(receiver));
        return [receiverPointer, method](bool isEnteringState) {
            if (!isEnteringState && !receiverPointer.isNull())
                QMetaObject::invokeMethod(receiverPointer.data(), method);
        };
    }

    //! [onentry-functor]
    template<typename Functor>
    static std::function<void(bool)> onEntry(Functor functor)
    {
        return [functor](bool isEnteringState) {
            if (isEnteringState)
                functor();
        };
    }

    //! [onexit-functor]
    template<typename Functor>
    static std::function<void(bool)> onExit(Functor functor)
    {
        return [functor](bool isEnteringState) {
            if (!isEnteringState)
                functor();
        };
    }

    //! [onentry-template]
    template<typename PointerToMemberFunction>
    static std::function<void(bool)> onEntry(
            const typename QtPrivate::FunctionPointer<PointerToMemberFunction>::Object *receiver,
            PointerToMemberFunction method)
    {
        typedef typename QtPrivate::FunctionPointer<PointerToMemberFunction>::Object Object;
        const QPointer<Object> receiverPointer(const_cast<Object *>(receiver));
        return [receiverPointer, method](bool isEnteringState) {
            if (isEnteringState && !receiverPointer.isNull())
                (receiverPointer->*method)();
        };
    }

    //! [onexit-template]
    template<typename PointerToMemberFunction>
    static std::function<void(bool)> onExit(
            const typename QtPrivate::FunctionPointer<PointerToMemberFunction>::Object *receiver,
            PointerToMemberFunction method)
    {
        typedef typename QtPrivate::FunctionPointer<PointerToMemberFunction>::Object Object;
        const QPointer<Object> receiverPointer(const_cast<Object *>(receiver));
        return [receiverPointer, method](bool isEnteringState) {
            if (!isEnteringState && !receiverPointer.isNull())
                (receiverPointer->*method)();
        };
    }

    QMetaObject::Connection connectToEvent(const QString &scxmlEventSpec,
                                           const QObject *receiver, const char *method,
                                           Qt::ConnectionType type = Qt::AutoConnection);

    // connect state to a QObject slot
    template <typename PointerToMemberFunction>
    inline QMetaObject::Connection connectToEvent(
            const QString &scxmlEventSpec,
            const typename QtPrivate::FunctionPointer<PointerToMemberFunction>::Object *receiver,
            PointerToMemberFunction method,
            Qt::ConnectionType type = Qt::AutoConnection)
    {
        typedef QtPrivate::FunctionPointer<PointerToMemberFunction> SlotType;
        return connectToEventImpl(
                    scxmlEventSpec, receiver, nullptr,
                    new QtPrivate::QSlotObject<PointerToMemberFunction,
                    typename SlotType::Arguments, void>(method),
                    type);
    }

    // connect state to a functor or function pointer (without context)
    template <typename Functor>
    inline typename std::enable_if<
            !QtPrivate::FunctionPointer<Functor>::IsPointerToMemberFunction &&
            !std::is_same<const char*, Functor>::value, QMetaObject::Connection>::type
    connectToEvent(const QString &scxmlEventSpec, Functor functor,
                   Qt::ConnectionType type = Qt::AutoConnection)
    {
        // Use this as context
        return connectToEvent(scxmlEventSpec, this, functor, type);
    }

    // connectToEvent to a functor or function pointer (with context)
    template <typename Functor>
    inline typename std::enable_if<
            !QtPrivate::FunctionPointer<Functor>::IsPointerToMemberFunction &&
            !std::is_same<const char*, Functor>::value, QMetaObject::Connection>::type
    connectToEvent(const QString &scxmlEventSpec, const QObject *context, Functor functor,
                   Qt::ConnectionType type = Qt::AutoConnection)
    {
        QtPrivate::QSlotObjectBase *slotObj = new QtPrivate::QFunctorSlotObject<Functor, 1,
                QtPrivate::List<QScxmlEvent>, void>(functor);
        return connectToEventImpl(scxmlEventSpec, context, reinterpret_cast<void **>(&functor),
                                  slotObj, type);
    }

    Q_INVOKABLE void submitEvent(QScxmlEvent *event);
    Q_INVOKABLE void submitEvent(const QString &eventName);
    Q_INVOKABLE void submitEvent(const QString &eventName, const QVariant &data);
    Q_INVOKABLE void cancelDelayedEvent(const QString &sendId);

    Q_INVOKABLE bool isDispatchableTarget(const QString &target) const;

    QList<QScxmlInvokableService *> invokedServices() const;
    QBindable<QList<QScxmlInvokableService*>> bindableInvokedServices();

    QScxmlTableData *tableData() const;
    void setTableData(QScxmlTableData *tableData);
    QBindable<QScxmlTableData*> bindableTableData();

Q_SIGNALS:
    void runningChanged(bool running);
    void invokedServicesChanged(const QList<QScxmlInvokableService *> &invokedServices);
    void log(const QString &label, const QString &msg);
    void reachedStableState();
    void finished();
    void dataModelChanged(QScxmlDataModel *model);
    void initialValuesChanged(const QVariantMap &initialValues);
    void initializedChanged(bool initialized);
    void loaderChanged(QScxmlCompiler::Loader *loader);
    void tableDataChanged(QScxmlTableData *tableData);

public Q_SLOTS:
    void start();
    void stop();
    bool init();

protected: // methods for friends:
    friend class QScxmlDataModel;
    friend class QScxmlEventBuilder;
    friend class QScxmlInvokableServicePrivate;
    friend class QScxmlExecutionEngine;

    // The methods below are used by the compiled state machines.
    bool isActive(int stateIndex) const;

private:
    QMetaObject::Connection connectToStateImpl(const QString &scxmlStateName,
                                               const QObject *receiver, void **slot,
                                               QtPrivate::QSlotObjectBase *slotObj,
                                               Qt::ConnectionType type = Qt::AutoConnection);
    QMetaObject::Connection connectToEventImpl(const QString &scxmlEventSpec,
                                               const QObject *receiver, void **slot,
                                               QtPrivate::QSlotObjectBase *slotObj,
                                               Qt::ConnectionType type = Qt::AutoConnection);
};

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QScxmlStateMachine *)
Q_DECLARE_METATYPE(QList<QScxmlInvokableService *>)

#endif // QSCXMLSTATEMACHINE_H
