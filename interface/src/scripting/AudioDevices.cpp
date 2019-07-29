//
//  AudioDevices.cpp
//  interface/src/scripting
//
//  Created by Zach Pomerantz on 28/5/2017.
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "AudioDevices.h"

#include <map>
#include <algorithm>

#include <shared/QtHelpers.h>
#include <plugins/DisplayPlugin.h>

#include "Application.h"
#include "AudioClient.h"
#include "Audio.h"
#include "UserActivityLogger.h"

using namespace scripting;

static Setting::Handle<QString> desktopInputDeviceSetting { QStringList { Audio::AUDIO, Audio::DESKTOP, "INPUT" }};
static Setting::Handle<QString> desktopOutputDeviceSetting { QStringList { Audio::AUDIO, Audio::DESKTOP, "OUTPUT" }};
static Setting::Handle<QString> hmdInputDeviceSetting { QStringList { Audio::AUDIO, Audio::HMD, "INPUT" }};
static Setting::Handle<QString> hmdOutputDeviceSetting { QStringList { Audio::AUDIO, Audio::HMD, "OUTPUT" }};

Setting::Handle<QString>& getSetting(bool contextIsHMD, QAudio::Mode mode) {
    if (mode == QAudio::AudioInput) {
        return contextIsHMD ? hmdInputDeviceSetting : desktopInputDeviceSetting;
    } else { // if (mode == QAudio::AudioOutput)
        return contextIsHMD ? hmdOutputDeviceSetting : desktopOutputDeviceSetting;
    }
}

enum AudioDeviceRole {
    DeviceNameRole = Qt::UserRole,
    SelectedDesktopRole,
    SelectedHMDRole,
    PeakRole,
    InfoRole
};

QHash<int, QByteArray> AudioDeviceList::_roles {
    { DeviceNameRole, "devicename" },
    { SelectedDesktopRole, "selectedDesktop" },
    { SelectedHMDRole, "selectedHMD" },
    { PeakRole, "peak" },
    { InfoRole, "info" }
};

static QString getTargetDevice(bool hmd, QAudio::Mode mode) {
    QString deviceName;
    auto& setting = getSetting(hmd, mode);
    if (setting.isSet()) {
        deviceName = setting.get();
    } else if (hmd) {
        if (mode == QAudio::AudioInput) {
            deviceName = qApp->getActiveDisplayPlugin()->getPreferredAudioInDevice();
        } else { // if (_mode == QAudio::AudioOutput)
            deviceName = qApp->getActiveDisplayPlugin()->getPreferredAudioOutDevice();
        }
    }
    return deviceName;
}

Qt::ItemFlags AudioDeviceList::_flags { Qt::ItemIsSelectable | Qt::ItemIsEnabled };

AudioDeviceList::AudioDeviceList(QAudio::Mode mode) : _mode(mode) {
    QString modeString = (_mode == QAudio::AudioInput) ? "input" : "output";
    auto& desktopSetting = getSetting(false, mode);
    if (desktopSetting.isSet()) {
        qDebug() << "Device name in settings for desktop" << modeString << "is" << desktopSetting.get();
    } else {
        qDebug() << "Device name in settings for desktop" << modeString << "is not set";
    }

    auto& hmdSetting = getSetting(true, mode);
    if (hmdSetting.isSet()) {
        qDebug() << "Device name in settings for HMD" << modeString << "is" << hmdSetting.get();
    } else {
        qDebug() << "Device name in settings for HMD" << modeString << " is not set";
    }
}

AudioDeviceList::~AudioDeviceList() {
}

QVariant AudioDeviceList::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() >= rowCount()) {
        return QVariant();
    }

    if (role == DeviceNameRole) {
        return _devices.at(index.row())->display;
    } else if (role == SelectedDesktopRole) {
        return _devices.at(index.row())->selectedDesktop;
    } else if (role == SelectedHMDRole) {
        return _devices.at(index.row())->selectedHMD;
    } else if (role == InfoRole) {
        return QVariant::fromValue<QAudioDeviceInfo>(_devices.at(index.row())->info);
    } else {
        return QVariant();
    }
}

QVariant AudioInputDeviceList::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() >= rowCount()) {
        return QVariant();
    }

    if (role == PeakRole) {
        return std::static_pointer_cast<AudioInputDevice>(_devices.at(index.row()))->peak;
    } else {
        return AudioDeviceList::data(index, role);
    }
}

void AudioDeviceList::resetDevice(bool contextIsHMD) {
    auto client = DependencyManager::get<AudioClient>().data();
    QString deviceName = getTargetDevice(contextIsHMD, _mode);
    // FIXME can't use blocking connections here, so we can't determine whether the switch succeeded or not
    // We need to have the AudioClient emit signals on switch success / failure
    QMetaObject::invokeMethod(client, "switchAudioDevice", 
        Q_ARG(QAudio::Mode, _mode), Q_ARG(QString, deviceName));

#if 0
    bool switchResult = false;
    QMetaObject::invokeMethod(client, "switchAudioDevice", Qt::BlockingQueuedConnection,
        Q_RETURN_ARG(bool, switchResult),
        Q_ARG(QAudio::Mode, _mode), Q_ARG(QString, deviceName));

    // try to set to the default device for this mode
    if (!switchResult) {
        if (contextIsHMD) {
            QString deviceName;
            if (_mode == QAudio::AudioInput) {
                deviceName = qApp->getActiveDisplayPlugin()->getPreferredAudioInDevice();
            } else { // if (_mode == QAudio::AudioOutput)
                deviceName = qApp->getActiveDisplayPlugin()->getPreferredAudioOutDevice();
            }
            if (!deviceName.isNull()) {
                QMetaObject::invokeMethod(client, "switchAudioDevice", Q_ARG(QAudio::Mode, _mode), Q_ARG(QString, deviceName));
            }
        } else {
            // use the system default
            QMetaObject::invokeMethod(client, "switchAudioDevice", Q_ARG(QAudio::Mode, _mode));
        }
    }
#endif
}

void AudioDeviceList::onDeviceChanged(const QAudioDeviceInfo& newDeviceInfo, bool isHMD) {
    QAudioDeviceInfo& currentDeviceInfo = isHMD ? _hmdDevice : _desktopDevice;
    currentDeviceInfo = newDeviceInfo;

    // walk _devices and update "isSelected" on its various entries
    for (auto i = 0; i < _devices.size(); ++i) {
        std::shared_ptr<AudioDevice> device = _devices[i];
        bool& isSelected = isHMD ? device->selectedHMD : device->selectedDesktop;
        if (isSelected && device->info != currentDeviceInfo) {
            isSelected = false;
        } else if (device->info == currentDeviceInfo) {
            isSelected = true;
        }
    }

    emit dataChanged(createIndex(0, 0), createIndex(rowCount() - 1, 0));
}

// Function returns 'strings similarity' as a number. The lesser number - the more similar strings are. Absolutely equal strings should return 0.
// Optimized version kindly provided by Ken
int levenshteinDistance(const QString& s1, const QString& s2) {
    const int m = s1.size();
    const int n = s2.size();

    if (m == 0) {
        return n;
    }
    if (n == 0) {
        return m;
    }

    auto cost = (int*)alloca((n + 1) * sizeof(int));

    for (int j = 0; j <= n; j++) {
        cost[j] = j;
    }

    for (int i = 0; i < m; i++) {

        int prev = i;
        cost[0] = i + 1;

        for (int j = 0; j < n; j++) {

            int temp = cost[j + 1];
            cost[j + 1] = (s1[i] == s2[j]) ? prev : std::min(cost[j], std::min(temp, prev)) + 1;
            prev = temp;
        }
    }
    return cost[n];
}

/*
std::shared_ptr<scripting::AudioDevice> getSimilarDevice(const QString& deviceName, const QList<std::shared_ptr<scripting::AudioDevice>>& devices) {

    int minDistance = INT_MAX;
    int minDistanceIndex = 0;

    for (auto i = 0; i < devices.length(); ++i) {
        auto distance = levenshteinDistance(deviceName, devices[i]->info.deviceName());
        if (distance < minDistance) {
            minDistance = distance;
            minDistanceIndex = i;
        }
    }

    return devices[minDistanceIndex];
}
*/

int32_t getSimilarDeviceIndex(const QString& deviceName, const QList<std::shared_ptr<scripting::AudioDevice>>& devices) {
    int minDistance = INT_MAX;
    int minDistanceIndex = 0;
    for (int32_t i = 0; i < devices.length(); ++i) {
        auto distance = levenshteinDistance(deviceName, devices[i]->info.deviceName());
        if (distance < minDistance) {
            minDistance = distance;
            minDistanceIndex = i;
        }
    }
    return minDistanceIndex;
}

AudioDevice::AudioDevice(const QAudioDeviceInfo& deviceInfo) {
    info = deviceInfo;
    display = deviceInfo.deviceName()
        .replace("High Definition", "HD")
        .remove("Device")
        .replace(" )", ")");
}

void AudioDeviceList::computePreferredDeviceName(bool isHMD, QString& deviceName) const {
    auto& setting = getSetting(isHMD, _mode);
    if (setting.isSet()) {
        deviceName = setting.get();
    } else if (isHMD) {
        if (_mode == QAudio::AudioInput) {
            deviceName = qApp->getActiveDisplayPlugin()->getPreferredAudioInDevice();
        } else { // _mode == QAudio::AudioOutput
            deviceName = qApp->getActiveDisplayPlugin()->getPreferredAudioOutDevice();
        }
    }
}

int32_t AudioDeviceList::findBestDeviceIndex(bool isHMD, QString deviceName) const {
    computePreferredDeviceName(isHMD, deviceName);
    return getSimilarDeviceIndex(deviceName, _devices);
}

void AudioDeviceList::onDevicesChanged(const QList<QAudioDeviceInfo>& deviceInfos, bool isHMD) {
    // This is called when the list of available devices has changed.
    // If a used device has been removed/disabled --> fall back to preferred settings if possible.
    // Or maybe a new device has been added/enabled --> switch to it.

    beginResetModel();

    // these are the previous names
    QString previousDesktopDeviceName(_desktopDevice.deviceName());
    QString previousHMDDeviceName(_hmdDevice.deviceName());

    auto& setting = getSetting(isHMD, _mode);
    QString preferredDeviceName = setting.get();

    int32_t desktopDeviceIndex = -1;
    int32_t hmdDeviceIndex = -1;

    // hunt for new devices as we build a fresh _devices list...
    QList<std::shared_ptr<AudioDevice>> devices;
    int32_t preferredDeviceIndex = -1;
    int32_t freshDeviceIndex = -1;
    foreach(const QAudioDeviceInfo& deviceInfo, deviceInfos) {
        AudioDevice device(deviceInfo);

        // if device not in old list then it was recently plugged in or enabled
        // and we assume the user intended to switch to it
        QString deviceName = deviceInfo.deviceName();
        bool isNewDevice = true;
        for (auto& oldDevice : _devices) {
            if (oldDevice->info.deviceName() == deviceName) {
                isNewDevice = false;
                break;
            }
        }
        if (isNewDevice) {
            // a new device has been discovered --> assume we want to switch to it
            desktopDeviceIndex = devices.size();
            hmdDeviceIndex = desktopDeviceIndex;
        }
        if (deviceName == preferredDeviceName) {
            preferredDeviceIndex = devices.size();
        }
        devices.push_back(newDevice(device));
    }
    _devices.swap(devices);

    if (_devices.empty()) {
        _desktopDevice = QAudioDeviceInfo();
        _hmdDevice = QAudioDeviceInfo();
    } else {
        // desktop
        if (desktopDeviceIndex == -1) {
            desktopDeviceIndex = findBestDeviceIndex(false, previousDesktopDeviceName);
        } else {
            // a new device has been found and we will switch to it unless...
            // the device for this context has been saved to settings
            // AND that preferred device is available
            if (preferredDeviceIndex != -1) {
                desktopDeviceIndex = preferredDeviceIndex;
            }
        }
        _desktopDevice = _devices[desktopDeviceIndex]->info;
        _devices[desktopDeviceIndex]->selectedDesktop = true;

        // similarly for HMD
        if (hmdDeviceIndex == -1) {
            hmdDeviceIndex = findBestDeviceIndex(true, previousHMDDeviceName);
        } else {
            if (preferredDeviceIndex != -1) {
                hmdDeviceIndex = preferredDeviceIndex;
            }
        }
        _hmdDevice = _devices[hmdDeviceIndex]->info;
        _devices[hmdDeviceIndex]->selectedHMD = true;
    }

    bool somethingChanged = false;
    if (_desktopDevice.deviceName() != previousDesktopDeviceName) {
        somethingChanged = true;
        if (!isHMD) {
            auto client = DependencyManager::get<AudioClient>().data();
            QMetaObject::invokeMethod(client, "switchAudioDevice",
                Q_ARG(QAudio::Mode, _mode),
                Q_ARG(QString, _desktopDevice.deviceName()));
        }
    }
    if (_hmdDevice.deviceName() != previousHMDDeviceName) {
        somethingChanged = true;
        if (isHMD) {
            auto client = DependencyManager::get<AudioClient>().data();
            QMetaObject::invokeMethod(client, "switchAudioDevice",
                Q_ARG(QAudio::Mode, _mode),
                Q_ARG(QString, _hmdDevice.deviceName()));
        }
    }
    if (somethingChanged) {
        emit dataChanged(createIndex(0, 0), createIndex(rowCount() - 1, 0));
    }

    endResetModel();
}

void AudioDeviceList::initDevices(const QList<QAudioDeviceInfo>& deviceInfos, bool isHMD) {
    // while we build a fresh _devices list...
    // find the current devices and cache whether it is "selected" or not
    bool foundDesktopDevice = false;
    bool foundHMDDevice = false;

    QString desktopName = _desktopDevice.deviceName();
    QString hmdName = _hmdDevice.deviceName();

    QList<std::shared_ptr<AudioDevice>> newDeviceList;
    foreach(const QAudioDeviceInfo& deviceInfo, deviceInfos) {
        AudioDevice device(deviceInfo);
        QString deviceName = deviceInfo.deviceName();
        if (!foundDesktopDevice && deviceName == desktopName) {
            foundDesktopDevice = true;
            device.selectedDesktop = true;
        }
        if (!foundHMDDevice && deviceName == hmdName) {
            foundHMDDevice = true;
            device.selectedHMD = true;
        }
        newDeviceList.push_back(newDevice(device));
    }
    _devices.swap(newDeviceList);

    // now that _devices is initialized we call onDevicesChanged()
    // which will redo most of this work but will also try to switch to preferred devices
    onDevicesChanged(deviceInfos, isHMD);

    emit dataChanged(createIndex(0, 0), createIndex(rowCount() - 1, 0));
}

void AudioDeviceList::setPreferredDevice(const QAudioDeviceInfo device, bool isHMD) {
    // save to settings
    auto& setting = getSetting(isHMD, _mode);
    setting.set(device.deviceName());
}

bool AudioInputDeviceList::peakValuesAvailable() {
    std::call_once(_peakFlag, [&] {
        _peakValuesAvailable = DependencyManager::get<AudioClient>()->peakValuesAvailable();
    });
    return _peakValuesAvailable;
}

void AudioInputDeviceList::setPeakValuesEnabled(bool enable) {
    if (peakValuesAvailable() && (enable != _peakValuesEnabled)) {
        DependencyManager::get<AudioClient>()->enablePeakValues(enable);
        _peakValuesEnabled = enable;
        emit peakValuesEnabledChanged(_peakValuesEnabled);
    }
}

void AudioInputDeviceList::onPeakValueListChanged(const QList<float>& peakValueList) {
    assert(_mode == QAudio::AudioInput);

    if (peakValueList.length() != rowCount()) {
        qWarning() << "AudioDeviceList" << __FUNCTION__ << "length mismatch";
    }

    for (auto i = 0; i < rowCount(); ++i) {
        std::static_pointer_cast<AudioInputDevice>(_devices[i])->peak = peakValueList[i];
    }

    emit dataChanged(createIndex(0, 0), createIndex(rowCount() - 1, 0), { PeakRole });
}

AudioDevices::AudioDevices(bool& contextIsHMD) : _contextIsHMD(contextIsHMD) {
    auto client = DependencyManager::get<AudioClient>().data();

    connect(client, &AudioClient::deviceChanged, this, &AudioDevices::onDeviceChanged, Qt::QueuedConnection);
    connect(client, &AudioClient::devicesChanged, this, &AudioDevices::onDevicesChanged, Qt::QueuedConnection);
    connect(client, &AudioClient::peakValueListChanged, &_inputs, &AudioInputDeviceList::onPeakValueListChanged, Qt::QueuedConnection);

    // slam the AudioDeviceLists' current devices to the default
    _inputs._desktopDevice = client->getActiveAudioDevice(QAudio::AudioInput);
    _inputs._hmdDevice = _inputs._desktopDevice;
    _outputs._desktopDevice = client->getActiveAudioDevice(QAudio::AudioOutput);
    _outputs._hmdDevice = _outputs._desktopDevice;

    // populate the device lists with the current state
    const QList<QAudioDeviceInfo>& inputDevices = client->getAudioDevices(QAudio::AudioInput);
    _inputs.initDevices(inputDevices, _contextIsHMD);
    const QList<QAudioDeviceInfo>& outputDevices = client->getAudioDevices(QAudio::AudioOutput);
    _outputs.initDevices(outputDevices, _contextIsHMD);
}

AudioDevices::~AudioDevices() {}

void AudioDevices::onContextChanged(const QString& context) {
    _inputs.resetDevice(_contextIsHMD);
    _outputs.resetDevice(_contextIsHMD);
}

void AudioDevices::onDeviceSelected(QAudio::Mode mode, const QAudioDeviceInfo& device,
                                    const QAudioDeviceInfo& previousDevice, bool isHMD) {
    QString deviceName = device.isNull() ? QString() : device.deviceName();
    auto& setting = getSetting(isHMD, mode);
    auto wasDefault = setting.get().isNull();

    // only store the selected device to settings if it matches the targetDevice
    QString targetDeviceName = getTargetDevice(isHMD, mode);
    if (targetDeviceName == deviceName) {
        setting.set(deviceName);
    }

    // log the selected device
    if (!device.isNull()) {
        QJsonObject data;

        const QString MODE = "audio_mode";
        const QString INPUT = "INPUT";
        const QString OUTPUT = "OUTPUT"; data[MODE] = mode == QAudio::AudioInput ? INPUT : OUTPUT;

        const QString CONTEXT = "display_mode";
        data[CONTEXT] = _contextIsHMD ? Audio::HMD : Audio::DESKTOP;

        const QString DISPLAY = "display_device";
        data[DISPLAY] = qApp->getActiveDisplayPlugin()->getName();

        const QString DEVICE = "device";
        const QString PREVIOUS_DEVICE = "previous_device";
        const QString WAS_DEFAULT = "was_default";
        data[DEVICE] = deviceName;
        data[PREVIOUS_DEVICE] = previousDevice.deviceName();
        data[WAS_DEFAULT] = wasDefault;

        UserActivityLogger::getInstance().logAction("selected_audio_device", data);
    }
}

void AudioDevices::onDeviceChanged(QAudio::Mode mode, const QAudioDeviceInfo& device) {
    if (mode == QAudio::AudioInput) {
        if (_requestedInputDevice == device) {
            onDeviceSelected(QAudio::AudioInput, device,
                             _contextIsHMD ? _inputs._hmdDevice : _inputs._desktopDevice,
                             _contextIsHMD);
            _requestedInputDevice = QAudioDeviceInfo();
        }
        _inputs.onDeviceChanged(device, _contextIsHMD);
    } else { // if (mode == QAudio::AudioOutput)
        if (_requestedOutputDevice == device) {
            onDeviceSelected(QAudio::AudioOutput, device,
                             _contextIsHMD ? _outputs._hmdDevice : _outputs._desktopDevice,
                             _contextIsHMD);
            _requestedOutputDevice = QAudioDeviceInfo();
        }
        _outputs.onDeviceChanged(device, _contextIsHMD);
    }
}

void AudioDevices::onDevicesChanged(QAudio::Mode mode, const QList<QAudioDeviceInfo>& devices) {

    //set devices for both contexts
    if (mode == QAudio::AudioInput) {
        _inputs.onDevicesChanged(devices, _contextIsHMD);
    } else { // if (mode == QAudio::AudioOutput)
        _outputs.onDevicesChanged(devices, _contextIsHMD);
    }
}

void AudioDevices::chooseDevice(QAudio::Mode mode, bool isHMD, const QAudioDeviceInfo& device) {
    if (mode == QAudio::AudioInput) {
        _inputs.setPreferredDevice(device, isHMD);
    } else {
        _outputs.setPreferredDevice(device, isHMD);
    }
    if (_contextIsHMD == isHMD) {
        // inside context we switchAudioDevice
        auto client = DependencyManager::get<AudioClient>().data();
        QMetaObject::invokeMethod(client, "switchAudioDevice",
                                  Q_ARG(QAudio::Mode, mode),
                                  Q_ARG(const QAudioDeviceInfo&, device));
    }
}
