// Copyright (C) 2022 The Qt Company Ltd.
// Copyright (C) 2019 Alexey Edelev <semlanik@gmail.com>
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only

#ifndef QABSTRACTGRPCCHANNEL_H
#define QABSTRACTGRPCCHANNEL_H

#include <QtCore/QString>
#include <QtCore/QThread>
#include <QtCore/qbytearray.h>
#include <QtGrpc/qgrpccredentials.h>
#include <QtGrpc/qgrpcstatus.h>
#include <QtGrpc/qtgrpcglobal.h>

#include <memory>

QT_BEGIN_NAMESPACE

class QAbstractGrpcClient;
class QAbstractProtobufSerializer;
struct QAbstractGrpcChannelPrivate;
class QGrpcStream;
class QGrpcCallReply;

class Q_GRPC_EXPORT QAbstractGrpcChannel
{
public:
    virtual QGrpcStatus call(QLatin1StringView method, QLatin1StringView service,
                             QByteArrayView args, QByteArray &ret) = 0;
    virtual std::shared_ptr<QGrpcCallReply> call(QAbstractGrpcClient *client,
                                                 QLatin1StringView method,
                                                 QLatin1StringView service,
                                                 QByteArrayView args) = 0;
    virtual void startStream(QGrpcStream *stream, QLatin1StringView service) = 0;
    virtual std::shared_ptr<QAbstractProtobufSerializer> serializer() const = 0;

protected:
    friend class QAbstractGrpcClient;
    QAbstractGrpcChannel();
    virtual ~QAbstractGrpcChannel();

private:
    Q_DISABLE_COPY(QAbstractGrpcChannel)
    std::unique_ptr<QAbstractGrpcChannelPrivate> dPtr;
};

QT_END_NAMESPACE

#endif // QABSTRACTGRPCCHANNEL_H
