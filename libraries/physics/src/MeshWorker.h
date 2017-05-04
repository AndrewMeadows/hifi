//
//  MeshWorker.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2016.06.16
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_MeshWorker_h
#define hifi_MeshWorker_h

#include <vector>

#include <QtCore/QObject>
#include <QtCore/QRunnable>

#include <gpu/Resource.h>
#include <model/Geometry.h>


class MeshWorker : public QObject, public QRunnable {
    Q_OBJECT
public:
    class Data {
    public:
        Data(model::MeshPointer& input) : _inputMesh(input) {}
        model::MeshPointer _inputMesh;
        model::MeshPointer _outputMesh;
    };

    MeshWorker(Data& data) : _data(data) {}

    void stop() { _stopped = true; }

signals:
    void onSuccess(model::MeshPointer output);
    void onProgress(float progressValue);
    void onError(int error, QString str);

protected:
    Data _data;
    bool _stopped { false };
};

#endif //hifi_MeshWorker_h
