// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Objects/Scene/ITMPlainVoxelArray.h"
#include "Objects/Scene/ITMSurfelTypes.h"
#include "Objects/Scene/ITMVoxelBlockHash.h"
#include "Objects/Scene/ITMVoxelTypes.h"

/** This chooses the information stored at each surfel. At the moment, valid
    options are ITMSurfel_grey and ITMSurfel_rgb.
*/
typedef ITMLib::ITMSurfel_rgb ITMSurfelT;

/** This chooses the information stored at each voxel. At the moment, valid
    options are ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb and ITMVoxel_f_rgb.
*/
typedef ITMVoxel_s ITMVoxel;

/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are ITMVoxelBlockHash and ITMPlainVoxelArray.
*/
typedef ITMLib::ITMVoxelBlockHash ITMVoxelIndex;
//typedef ITMLib::ITMPlainVoxelArray ITMVoxelIndex;

//////////////////////////////////////////////////////////////////////////
// Do not change below this point
//////////////////////////////////////////////////////////////////////////
#ifndef ITMFloatImage
#define ITMFloatImage ORUtils::Image<float>
#endif

#ifndef ITMFloat2Image
#define ITMFloat2Image ORUtils::Image<Vector2f>
#endif

#ifndef ITMFloat4Image
#define ITMFloat4Image ORUtils::Image<Vector4f>
#endif

#ifndef ITMShortImage
#define ITMShortImage ORUtils::Image<short>
#endif

#ifndef ITMShort3Image
#define ITMShort3Image ORUtils::Image<Vector3s>
#endif

#ifndef ITMShort4Image
#define ITMShort4Image ORUtils::Image<Vector4s>
#endif

#ifndef ITMUShortImage
#define ITMUShortImage ORUtils::Image<ushort>
#endif

#ifndef ITMUIntImage
#define ITMUIntImage ORUtils::Image<uint>
#endif

#ifndef ITMIntImage
#define ITMIntImage ORUtils::Image<int>
#endif

#ifndef ITMUCharImage
#define ITMUCharImage ORUtils::Image<uchar>
#endif

#ifndef ITMUChar4Image
#define ITMUChar4Image ORUtils::Image<Vector4u>
#endif

#ifndef ITMBoolImage
#define ITMBoolImage ORUtils::Image<bool>
#endif