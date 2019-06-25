//
//  PacketHeaders.h
//  libraries/networking/src
//
//  Created by Stephen Birarda on 4/8/13.
//  Copyright 2013 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_PacketHeaders_h
#define hifi_PacketHeaders_h

#pragma once

#include <cstdint>
#include <map>

#include <QtCore/QCryptographicHash>
#include <QtCore/QObject>
#include <QtCore/QSet>
#include <QtCore/QUuid>

// The enums are inside this PacketTypeEnum for run-time conversion of enum value to string via
// Q_ENUMS, without requiring a macro that is called for each enum value.
class PacketTypeEnum {
    Q_GADGET
    Q_ENUMS(Value)
public:
    // If adding a new packet packetType, you can replace one marked usable or add at the end.
    // This enum must hold 256 or fewer packet types (so the value is <= 255) since it is statically typed as a uint8_t
    enum class Value : uint8_t {
        Unknown,
        StunResponse,
        DomainList,
        Ping,
        PingReply,
        KillAvatar,
        AvatarData,
        InjectAudio,
        MixedAudio,
        MicrophoneAudioNoEcho,
        MicrophoneAudioWithEcho,
        BulkAvatarData,
        SilentAudioFrame,
        DomainListRequest,
        RequestAssignment,
        CreateAssignment,
        DomainConnectionDenied,
        MuteEnvironment,
        AudioStreamStats,
        DomainServerPathQuery,
        DomainServerPathResponse,
        DomainServerAddedNode,
        ICEServerPeerInformation,
        ICEServerQuery,
        OctreeStats,
        SetAvatarTraits,
        InjectorGainSet,
        AssignmentClientStatus,
        NoisyMute,
        AvatarIdentity,
        NodeIgnoreRequest,
        DomainConnectRequest,
        DomainServerRequireDTLS,
        NodeJsonStats,
        OctreeDataNack,
        StopNode,
        AudioEnvironment,
        EntityEditNack,
        ICEServerHeartbeat,
        ICEPing,
        ICEPingReply,
        EntityData,
        EntityQuery,
        EntityAdd,
        EntityErase,
        EntityEdit,
        DomainServerConnectionToken,
        DomainSettingsRequest,
        DomainSettings,
        AssetGet,
        AssetGetReply,
        AssetUpload,
        AssetUploadReply,
        AssetGetInfo,
        AssetGetInfoReply,
        DomainDisconnectRequest,
        DomainServerRemovedNode,
        MessagesData,
        MessagesSubscribe,
        MessagesUnsubscribe,
        ICEServerHeartbeatDenied,
        AssetMappingOperation,
        AssetMappingOperationReply,
        ICEServerHeartbeatACK,
        NegotiateAudioFormat,
        SelectedAudioFormat,
        MoreEntityShapes,
        NodeKickRequest,
        NodeMuteRequest,
        RadiusIgnoreRequest,
        UsernameFromIDRequest,
        UsernameFromIDReply,
        AvatarQuery,
        RequestsDomainListData,
        PerAvatarGainSet,
        EntityScriptGetStatus,
        EntityScriptGetStatusReply,
        ReloadEntityServerScript,
        EntityPhysics,
        EntityServerScriptLog,
        AdjustAvatarSorting,
        OctreeFileReplacement,
        CollisionEventChanges,
        ReplicatedMicrophoneAudioNoEcho,
        ReplicatedMicrophoneAudioWithEcho,
        ReplicatedInjectAudio,
        ReplicatedSilentAudioFrame,
        ReplicatedAvatarIdentity,
        ReplicatedKillAvatar,
        ReplicatedBulkAvatarData,
        DomainContentReplacementFromUrl,
        ChallengeOwnership,
        EntityScriptCallMethod,
        ChallengeOwnershipRequest,
        ChallengeOwnershipReply,
        OctreeDataFileRequest,
        OctreeDataFileReply,
        OctreeDataPersist,
        EntityClone,
        EntityQueryInitialResultsComplete,
        BulkAvatarTraits,
        AudioSoloRequest,
        BulkAvatarTraitsAck,
        StopInjector,
        NUM_PACKET_TYPE
    };

    const static QHash<PacketTypeEnum::Value, PacketTypeEnum::Value> getReplicatedPacketMapping() {
        const static QHash<PacketTypeEnum::Value, PacketTypeEnum::Value> REPLICATED_PACKET_MAPPING {
            { PacketTypeEnum::Value::MicrophoneAudioNoEcho, PacketTypeEnum::Value::ReplicatedMicrophoneAudioNoEcho },
            { PacketTypeEnum::Value::MicrophoneAudioWithEcho, PacketTypeEnum::Value::ReplicatedMicrophoneAudioWithEcho },
            { PacketTypeEnum::Value::InjectAudio, PacketTypeEnum::Value::ReplicatedInjectAudio },
            { PacketTypeEnum::Value::SilentAudioFrame, PacketTypeEnum::Value::ReplicatedSilentAudioFrame },
            { PacketTypeEnum::Value::AvatarIdentity, PacketTypeEnum::Value::ReplicatedAvatarIdentity },
            { PacketTypeEnum::Value::KillAvatar, PacketTypeEnum::Value::ReplicatedKillAvatar },
            { PacketTypeEnum::Value::BulkAvatarData, PacketTypeEnum::Value::ReplicatedBulkAvatarData }
        };
        return REPLICATED_PACKET_MAPPING;
    }

    static bool isNonVerifiedPacketType(PacketTypeEnum::Value type) {
        switch (type) {
            case PacketTypeEnum::Value::NodeJsonStats:
            case PacketTypeEnum::Value::EntityQuery:
            case PacketTypeEnum::Value::OctreeDataNack:
            case PacketTypeEnum::Value::EntityEditNack:
            case PacketTypeEnum::Value::DomainListRequest:
            case PacketTypeEnum::Value::StopNode:
            case PacketTypeEnum::Value::DomainDisconnectRequest:
            case PacketTypeEnum::Value::UsernameFromIDRequest:
            case PacketTypeEnum::Value::NodeKickRequest:
            case PacketTypeEnum::Value::NodeMuteRequest:
                return true;
            default:
                break;
        }
        return false;
    }

    static bool isNonSourcedPacketType(PacketTypeEnum::Value type) {
        switch (type) {
            case PacketTypeEnum::Value::StunResponse:
            case PacketTypeEnum::Value::CreateAssignment:
            case PacketTypeEnum::Value::RequestAssignment:
            case PacketTypeEnum::Value::DomainServerRequireDTLS:
            case PacketTypeEnum::Value::DomainConnectRequest:
            case PacketTypeEnum::Value::DomainList:
            case PacketTypeEnum::Value::DomainConnectionDenied:
            case PacketTypeEnum::Value::DomainServerPathQuery:
            case PacketTypeEnum::Value::DomainServerPathResponse:
            case PacketTypeEnum::Value::DomainServerAddedNode:
            case PacketTypeEnum::Value::DomainServerConnectionToken:
            case PacketTypeEnum::Value::DomainSettingsRequest:
            case PacketTypeEnum::Value::OctreeDataFileRequest:
            case PacketTypeEnum::Value::OctreeDataFileReply:
            case PacketTypeEnum::Value::OctreeDataPersist:
            case PacketTypeEnum::Value::DomainContentReplacementFromUrl:
            case PacketTypeEnum::Value::DomainSettings:
            case PacketTypeEnum::Value::ICEServerPeerInformation:
            case PacketTypeEnum::Value::ICEServerQuery:
            case PacketTypeEnum::Value::ICEServerHeartbeat:
            case PacketTypeEnum::Value::ICEServerHeartbeatACK:
            case PacketTypeEnum::Value::ICEPing:
            case PacketTypeEnum::Value::ICEPingReply:
            case PacketTypeEnum::Value::ICEServerHeartbeatDenied:
            case PacketTypeEnum::Value::AssignmentClientStatus:
            case PacketTypeEnum::Value::StopNode:
            case PacketTypeEnum::Value::DomainServerRemovedNode:
            case PacketTypeEnum::Value::UsernameFromIDReply:
            case PacketTypeEnum::Value::OctreeFileReplacement:
            case PacketTypeEnum::Value::ReplicatedMicrophoneAudioNoEcho:
            case PacketTypeEnum::Value::ReplicatedMicrophoneAudioWithEcho:
            case PacketTypeEnum::Value::ReplicatedInjectAudio:
            case PacketTypeEnum::Value::ReplicatedSilentAudioFrame:
            case PacketTypeEnum::Value::ReplicatedAvatarIdentity:
            case PacketTypeEnum::Value::ReplicatedKillAvatar:
            case PacketTypeEnum::Value::ReplicatedBulkAvatarData:
                return true;
            default:
                break;
        }
        return false;
    }

    static bool isDomainSourcedPacketType(PacketTypeEnum::Value type) {
        switch (type) {
            case PacketTypeEnum::Value::AssetMappingOperation:
            case PacketTypeEnum::Value::AssetGet:
            case PacketTypeEnum::Value::AssetUpload:
                return true;
            default:
                break;
        }
        return false;
    }

    static bool isDomainIgnoredVerificationPacketType(PacketTypeEnum::Value type) {
        switch (type) {
            case PacketTypeEnum::Value::AssetMappingOperationReply:
            case PacketTypeEnum::Value::AssetGetReply:
            case PacketTypeEnum::Value::AssetUploadReply:
                return true;
            default:
                break;
        }
        return false;
    }
};

using PacketType = PacketTypeEnum::Value;

const int NUM_BYTES_MD5_HASH = 16;

typedef char PacketVersion;

PacketVersion versionForPacketType(PacketType packetType);
QByteArray protocolVersionsSignature(); /// returns a unqiue signature for all the current protocols
QString protocolVersionsSignatureBase64();

#if (PR_BUILD || DEV_BUILD)
void sendWrongProtocolVersionsSignature(bool sendWrongVersion); /// for debugging version negotiation
#endif

uint qHash(const PacketType& key, uint seed);
QDebug operator<<(QDebug debug, const PacketType& type);

// Due to the different legacy behaviour, we need special processing for domains that were created before
// the zone inheritance modes were added.  These have version numbers up to 80
enum class EntityVersion : PacketVersion {
    StrokeColorProperty = 0,
    HasDynamicOwnershipTests,
    HazeEffect,
    StaticCertJsonVersionOne,
    OwnershipChallengeFix,
    ZoneLightInheritModes = 82,
    ZoneStageRemoved,
    SoftEntities,
    MaterialEntities,
    ShadowControl,
    MaterialData,
    CloneableData,
    CollisionMask16Bytes,
    YieldSimulationOwnership,
    ParticleEntityFix,
    ParticleSpin,
    BloomEffect,
    GrabProperties,
    ScriptGlmVectors,
    FixedLightSerialization,
    MaterialRepeat,
    EntityHostTypes,
    CleanupProperties,
    ImageEntities,
    GridEntities,
    MissingTextProperties,
    GrabTraits,
    MorePropertiesCleanup,
    FixPropertiesFromCleanup,
    UpdatedPolyLines,
    FixProtocolVersionBumpMismatch,
    MigrateOverlayRenderProperties,
    MissingWebEntityProperties,
    PulseProperties,
    RingGizmoEntities,
    AvatarPriorityZone,
    ShowKeyboardFocusHighlight,
    WebBillboardMode,
    ModelScale,
    ReOrderParentIDProperties,
    CertificateTypeProperty,
    DisableWebMedia,
    ParticleShapeType,
    ParticleShapeTypeDeadlockFix,
    PrivateUserData,

    // Add new versions above here
    NUM_PACKET_TYPE,
    LAST_PACKET_TYPE = NUM_PACKET_TYPE - 1
};

enum class EntityScriptCallMethodVersion : PacketVersion {
    ServerCallable = 18,
    ClientCallable = 19
};

enum class EntityQueryPacketVersion: PacketVersion {
    JSONFilter = 18,
    JSONFilterWithFamilyTree = 19,
    ConnectionIdentifier = 20,
    RemovedJurisdictions = 21,
    MultiFrustumQuery = 22,
    ConicalFrustums = 23
};

enum class AssetServerPacketVersion: PacketVersion {
    VegasCongestionControl = 19,
    RangeRequestSupport,
    RedirectedMappings,
    BakingTextureMeta
};

enum class AvatarMixerPacketVersion : PacketVersion {
    TranslationSupport = 17,
    SoftAttachmentSupport,
    AvatarEntities,
    AbsoluteSixByteRotations,
    SensorToWorldMat,
    HandControllerJoints,
    HasKillAvatarReason,
    SessionDisplayName,
    Unignore,
    ImmediateSessionDisplayNameUpdates,
    VariableAvatarData,
    AvatarAsChildFixes,
    StickAndBallDefaultAvatar,
    IdentityPacketsIncludeUpdateTime,
    AvatarIdentitySequenceId,
    MannequinDefaultAvatar,
    AvatarIdentitySequenceFront,
    IsReplicatedInAvatarIdentity,
    AvatarIdentityLookAtSnapping,
    UpdatedMannequinDefaultAvatar,
    AvatarJointDefaultPoseFlags,
    FBXReaderNodeReparenting,
    FixMannequinDefaultAvatarFeet,
    ProceduralFaceMovementFlagsAndBlendshapes,
    FarGrabJoints,
    MigrateSkeletonURLToTraits,
    MigrateAvatarEntitiesToTraits,
    FarGrabJointsRedux,
    JointTransScaled,
    GrabTraits,
    CollisionFlag,
    AvatarTraitsAck,
    FasterAvatarEntities,
    SendMaxTranslationDimension,
    FBXJointOrderChange,
    HandControllerSection,
    SendVerificationFailed
};

enum class DomainConnectRequestVersion : PacketVersion {
    NoHostname = 17,
    HasHostname,
    HasProtocolVersions,
    HasMACAddress,
    HasMachineFingerprint,
    AlwaysHasMachineFingerprint,
    HasTimestamp,
    HasReason,
    HasSystemInfo,
    HasCompressedSystemInfo
};

enum class DomainConnectionDeniedVersion : PacketVersion {
    ReasonMessageOnly = 17,
    IncludesReasonCode,
    IncludesExtraInfo
};

enum class DomainServerAddedNodeVersion : PacketVersion {
    PrePermissionsGrid = 17,
    PermissionsGrid
};

enum class DomainListVersion : PacketVersion {
    PrePermissionsGrid = 18,
    PermissionsGrid,
    GetUsernameFromUUIDSupport,
    GetMachineFingerprintFromUUIDSupport,
    AuthenticationOptional,
    HasTimestamp,
    HasConnectReason
};

enum class AudioVersion : PacketVersion {
    HasCompressedAudio = 17,
    CodecNameInAudioPackets,
    Exactly10msAudioPackets,
    TerminatingStreamStats,
    SpaceBubbleChanges,
    HasPersonalMute,
    HighDynamicRangeVolume,
    StopInjectors
};

enum class MessageDataVersion : PacketVersion {
    TextOrBinaryData = 18
};

enum class IcePingVersion : PacketVersion {
    SendICEPeerID = 18
};

enum class PingVersion : PacketVersion {
    IncludeConnectionID = 18
};

enum class AvatarQueryVersion : PacketVersion {
    SendMultipleFrustums = 21,
    ConicalFrustums = 22
};

#endif // hifi_PacketHeaders_h
