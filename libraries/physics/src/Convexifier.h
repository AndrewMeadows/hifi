//
//  Convexifier.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2016.06.16
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_Convexifier_h
#define hifi_Convexifier_h

#include <VHACD.h>

#include "MeshWorker.h"

using VhacdParameters = VHACD::IVHACD::Parameters;
// VhacdParameters data members are:
//  double         m_alpha;
//  double         m_beta;
//  double         m_gamma;
//  double         m_delta;
//  double         m_minVolumePerCH;
//  IUserCallback* m_callback;
//  IUserLogger*   m_logger;
//  unsigned int   m_resolution;
//  unsigned int   m_maxNumVerticesPerCH;
//  int            m_depth;
//  int            m_planeDownsampling;
//  int            m_convexhullDownsampling;
//  int            m_pca;
//  int            m_mode;
//  int            m_convexhullApproximation;
//  int            m_oclAcceleration;



class Convexifier : public MeshWorker {
    Q_OBJECT
public:
    Convexifier(MeshWorker::Data& data);
    void run() override;

    const VhacdParameters& getParams(VhacdParameters& params) const { return _params; }
    void setParams(const VhacdParameters& params);

protected:
	VhacdParameters _params;
};

#if 0
private:
    bool loadFBX(const QString filename, FBXGeometry& result);

    //void fattenMesh(const FBXMesh& mesh, const glm::mat4& gometryOffset, FBXMesh& result) const;

    bool computeVHACD(FBXGeometry& geometry,
                      VHACD::IVHACD::Parameters params,
                      FBXGeometry& result,
                      float minimumMeshSize, float maximumMeshSize);

    void getConvexResults(VHACD::IVHACD* convexifier, FBXMesh& resultMesh) const;

private:
    MeshDataPtr _meshData;
    bool _verbose { false };
};

class ProgressCallback : public VHACD::IVHACD::IUserCallback {
public:
    ProgressCallback(void);
    ~ProgressCallback();

    // Couldn't follow coding guideline here due to virtual function declared in IUserCallback
    void Update(const double overallProgress, const double stageProgress, const double operationProgress,
        const char * const stage, const char * const operation);
};

//AABox getAABoxForMeshPart(const FBXMeshPart &meshPart);

#endif

#endif //hifi_Convexifier_h
