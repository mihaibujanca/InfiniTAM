/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <timings.h>
#include <SLAMBenchAPI.h>
#include <Eigen/Core>


#include "ITAMConstants.h"
#include <io/SLAMFrame.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/CameraSensorFinder.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/IMUSensor.h>
#include <ITMMainEngine.h>
#include <ITMBasicEngine.h>
#include <ITMBasicSurfelEngine.h>
#include <ITMMultiEngine.h>
#include <MemoryBlock.h>
#include <ITMLibSettings.h>
#include <ITMLibDefines.h>
#include <Objects/Meshing/ITMMesh.h>
static ITMMainEngine* mainEngine ;

static Vector2i inputSize;
static Vector2i depthInputSize;


static ITMUChar4Image* rgbwrapper ;
static ITMShortImage * depthwrapper ;
static ITMIMUMeasurement * imuwrapper;
static ITMUChar4Image *rgb      ;
static ITMUChar4Image *depth    ;
static ITMUChar4Image *raycast  ;


static ITMLibSettings *internalSettings;

static ITMRGBDCalib calibration;


static slambench::io::DepthSensor *depth_sensor;
static slambench::io::CameraSensor *rgb_sensor;
static slambench::io::IMUSensor *imu_sensor;



static slambench::outputs::Output *pose_output;
static slambench::outputs::Output *pointcloud_output;

static slambench::outputs::Output *rgb_frame_output;
static slambench::outputs::Output *depth_frame_output;
static slambench::outputs::Output *render_frame_output;


static slambench::TimeStamp last_frame_timestamp;


// Algo parameters


bool
    useSwapping,
    useBilateralFilter,
    useApproximateRaycast,
    modelSensorNoise,
    skipPoints,
    stopIntegratingAtMaxW;

int maxW;

float
    depthTrackerICPThreshold,
    depthTrackerTerminationThreshold,
    voxelSize,
    viewFrustum_min,
    viewFrustum_max,
    mu;

ITMTrackerFactory::TrackerType trackerType;

ORUtils::DeviceType deviceType;

std::vector<TrackerIterationType>
    trackingRegime;



/**
 * Initialization.
 */

bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {


    slam_settings->addParameter(TypedParameter<ITMTrackerFactory::TrackerType>  ("tt", "trackerType", "COLOR, ICP, REN, IMU, or WICP",  &trackerType             , &default_trackerType            ));
    slam_settings->addParameter(TypedParameter<float>("dticp", "depthTrackerICPThreshold",     "depthTrackerICPThreshold",      &depthTrackerICPThreshold,         &default_depthTrackerICPThreshold        ));
    slam_settings->addParameter(TypedParameter<float>("dttt", "depthTrackerTerminationThreshold",     "depthTrackerTerminationThreshold",      &depthTrackerTerminationThreshold, &default_depthTrackerTerminationThreshold));
    slam_settings->addParameter(TypedParameter<float>("vs", "voxelSize",     "voxelSize",      &voxelSize,                        &default_voxelSize                       ));
    slam_settings->addParameter(TypedParameter<float>("vfmin", "viewFrustum_min",     "viewFrustum_min",      &viewFrustum_min,                  &default_viewFrustum_min                 ));
    slam_settings->addParameter(TypedParameter<float>("vfmax", "viewFrustum_max",     "viewFrustum_max",      &viewFrustum_max,                  &default_viewFrustum_max                 ));
    slam_settings->addParameter(TypedParameter<float>("mu", "mu",     "mu",      &mu,                               &default_mu                              ));
    slam_settings->addParameter(TypedParameter<typename std::vector<TrackerIterationType> >  ("tr", "trackingRegime", "trackingRegime",  &trackingRegime, &default_trackingRegime));
    slam_settings->addParameter(TypedParameter<int>  ("maxw", "maxW", "maxW",  &maxW             , &default_maxW            ));
    slam_settings->addParameter(TypedParameter<bool>("usesw", "useSwapping",        "useSwapping",        &useSwapping                 , &default_useSwapping               ));
    slam_settings->addParameter(TypedParameter<bool>("usebf", "useBilateralFilter",        "useBilateralFilter",        &useBilateralFilter          , &default_useBilateralFilter        ));
    slam_settings->addParameter(TypedParameter<bool>("usear", "useApproximateRaycast",        "useApproximateRaycast",        &useApproximateRaycast       , &default_useApproximateRaycast     ));
    slam_settings->addParameter(TypedParameter<bool>("msn", "modelSensorNoise",        "modelSensorNoise",        &modelSensorNoise            , &default_modelSensorNoise          ));
    slam_settings->addParameter(TypedParameter<bool>("sp", "skipPoints",        "skipPoints",        &skipPoints                  , &default_skipPoints                ));
    slam_settings->addParameter(TypedParameter<bool>("si", "stopIntegratingAtMaxW",        "stopIntegratingAtMaxW",        &stopIntegratingAtMaxW       , &default_stopIntegratingAtMaxW     ));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings)  {



    /**
     * Retrieve RGB and Depth sensors,
     *  - check input_size are the same
     *  - check camera are the same
     *  - get input_file
     */

	slambench::io::CameraSensorFinder sensor_finder;
	rgb_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "rgb"}});
	depth_sensor = (slambench::io::DepthSensor*)sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "depth"}});

	imu_sensor = (slambench::io::IMUSensor*)slam_settings->get_sensors().GetSensor(slambench::io::IMUSensor::kIMUType);


    if ((rgb_sensor == nullptr) || (depth_sensor == nullptr)) {
        std::cerr << "Invalid sensors found, RGB or Depth not found." << std::endl;
        return false;
    }

	if(rgb_sensor->FrameFormat != slambench::io::frameformat::Raster) {
		std::cerr << "RGB data is in wrong format" << std::endl;
		return false;
	}
	if(depth_sensor->FrameFormat != slambench::io::frameformat::Raster) {
		std::cerr << "Depth data is in wrong format" << std::endl;
		return false;
	}
	if(rgb_sensor->PixelFormat != slambench::io::pixelformat::RGB_III_888) {
		std::cerr << "RGB data is in wrong format pixel" << std::endl;
		return false;
	}
	if(depth_sensor->PixelFormat != slambench::io::pixelformat::D_I_16) {
		std::cerr << "Depth data is in wrong pixel format" << std::endl;
		return false;
	}

	if(rgb_sensor->Width != depth_sensor->Width || rgb_sensor->Height != depth_sensor->Height) {
		std::cerr << "Sensor size mismatch" << std::endl;
		return false;
	}



    sb_float4 depth_camera =  make_sb_float4(
    		depth_sensor->Intrinsics[0],
			depth_sensor->Intrinsics[1],
			depth_sensor->Intrinsics[2],
			depth_sensor->Intrinsics[3]);

     depth_camera.x = depth_camera.x * depth_sensor->Width;
     depth_camera.y = depth_camera.y * depth_sensor->Height;
     depth_camera.z = depth_camera.z * depth_sensor->Width;
     depth_camera.w = depth_camera.w * depth_sensor->Height;


     sb_float4 rgb_camera =  make_sb_float4(
                rgb_sensor->Intrinsics[0],
                rgb_sensor->Intrinsics[1],
                rgb_sensor->Intrinsics[2],
                rgb_sensor->Intrinsics[3]);

     rgb_camera.x = rgb_camera.x * rgb_sensor->Width;
     rgb_camera.y = rgb_camera.y * rgb_sensor->Height;
     rgb_camera.z = rgb_camera.z * rgb_sensor->Width;
     rgb_camera.w = rgb_camera.w * rgb_sensor->Height;


    // Teddy RGB camera settings :
    // 0.787907813,1.049802083,0.550714063,0.5670875

    // Teddy D camera settings :
    // 0.896421875,1.196654167,0.541360938,0.518814583


    internalSettings = new ITMLibSettings();

    internalSettings->useBilateralFilter = useBilateralFilter;
    internalSettings->useApproximateRaycast = useApproximateRaycast;
//    internalSettings->modelSensorNoise = modelSensorNoise;
//    internalSettings->trackerType = trackerType;
    internalSettings->skipPoints = skipPoints;

//    internalSettings->depthTrackerICPThreshold = depthTrackerICPThreshold;
//    internalSettings->depthTrackerTerminationThreshold = depthTrackerTerminationThreshold;
//    internalSettings->useSwapping = useSwapping;

//    internalSettings->noHierarchyLevels = trackingRegime.size();
//    delete internalSettings->trackingRegime;
//    internalSettings->trackingRegime = new TrackerIterationType[internalSettings->noHierarchyLevels];
//    for (int i = 0 ; i < noHierarchyLevels ; i++) {
//        internalSettings->trackingRegime[i] = trackingRegime[i];
//    }


    internalSettings->sceneParams.voxelSize = voxelSize;
    internalSettings->sceneParams.viewFrustum_min = viewFrustum_min;
    internalSettings->sceneParams.viewFrustum_max = viewFrustum_max;
    internalSettings->sceneParams.mu = mu;
    internalSettings->sceneParams.maxW = maxW;
    internalSettings->sceneParams.stopIntegratingAtMaxW = stopIntegratingAtMaxW;




    inputSize[0] = rgb_sensor->Width;
    inputSize[1] = rgb_sensor->Height;


    depthInputSize[0] = depth_sensor->Width;
    depthInputSize[1] = depth_sensor->Height;



    calibration.intrinsics_rgb.SetFrom(rgb_camera.x, rgb_camera.y,rgb_camera.z, rgb_camera.w, rgb_sensor->Width, rgb_sensor->Height);
    calibration.intrinsics_d.SetFrom(depth_camera.x, depth_camera.y,depth_camera.z, depth_camera.w, depth_sensor->Width,depth_sensor->Height);

    //calibration.intrinsics_d.SetFrom( 573.71,574.394,346.471,249.031,640,480);


    Matrix4f calib;

    // ICLNUIM
    //calib.m00 = 1.0f; calib.m10 = 0.0f; calib.m20 = 0.0f;   calib.m30 = 0.0f;
    //calib.m01 = 0.0f; calib.m11 = 1.0f; calib.m21 = 0.0f;   calib.m31 = 0.0f;
    //calib.m02 = 0.0f; calib.m12 = 0.0f; calib.m22 = 1.0f;   calib.m32 = 0.0f;
    //calib.m03 = 0.0f; calib.m13 = 0.0f; calib.m23 = 0.0f;   calib.m33 = 1.0f;


    // Teddy D trafo_rgb_to_depth calib :
    //calib.m00 =  0.9997490f; calib.m10 = 0.00518867f; calib.m20 =  0.0217975f;   calib.m30 =  0.2243070f;
    //calib.m01 = -0.0051649f; calib.m11 = 0.99998600f; calib.m21 = -0.0011465f;   calib.m31 = -0.5001670f;
    //calib.m02 = -0.0218031f; calib.m12 = 0.00103363f; calib.m22 =  0.9997620f;   calib.m32 =  0.0151706f;
    //calib.m03 =  0.0000000f; calib.m13 = 0.00000000f; calib.m23 =  0.0000000f;   calib.m33 =  1.0000000f;


//    if  (rgb_sensor->Pose !=  Eigen::Matrix4f::Identity()) {
//        std::cerr << "Invalid direction for rgb" << std::endl;
//        return false;
//    }

    // Teddy D trafo_rgb_to_depth calib :
    calib.m00 = depth_sensor->Pose(0,0); calib.m10 = depth_sensor->Pose(1,0); calib.m20 = depth_sensor->Pose(2,0); calib.m30 = depth_sensor->Pose(3,0);
    calib.m01 = depth_sensor->Pose(0,1); calib.m11 = depth_sensor->Pose(1,1); calib.m21 = depth_sensor->Pose(2,1); calib.m31 = depth_sensor->Pose(3,1);
    calib.m02 = depth_sensor->Pose(0,2); calib.m12 = depth_sensor->Pose(1,2); calib.m22 = depth_sensor->Pose(2,2); calib.m32 = depth_sensor->Pose(3,2);
    calib.m03 = depth_sensor->Pose(0,3); calib.m13 = depth_sensor->Pose(1,3); calib.m23 = depth_sensor->Pose(2,3); calib.m33 = depth_sensor->Pose(3,3);



    calibration.trafo_rgb_to_depth.SetFrom(calib);



    //std::stringstream src("affine 0.0002 0.0");
    //std::stringstream src("kinect 1135.09 0.0819141");

    ITMDisparityCalib::TrafoType type = ITMDisparityCalib::TRAFO_AFFINE;

    switch (depth_sensor->DisparityType) {
        case slambench::io::DepthSensor::kinect_disparity:
            type = ITMDisparityCalib::TRAFO_KINECT;
            break;
        case slambench::io::DepthSensor::affine_disparity:
            type = ITMDisparityCalib::TRAFO_AFFINE;
            break;
        default :
        	std::cerr << "Unknown disparity type" << std::endl;
            exit(1);
    }
    calibration.disparityCalib.SetFrom(depth_sensor->DisparityParams[0], depth_sensor->DisparityParams[1], type);

    rgbwrapper    = new ITMUChar4Image ( inputSize,true, false) ;
    depthwrapper  = new ITMShortImage  ( inputSize,true, false) ;
    imuwrapper    = new ITMIMUMeasurement ();

    switch (internalSettings->libMode)
    {
        case ITMLibSettings::LIBMODE_BASIC:
            mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, calibration, inputSize, depthInputSize);
            break;
        case ITMLibSettings::LIBMODE_BASIC_SURFELS:
            mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(internalSettings, calibration, inputSize, depthInputSize);
            break;
        case ITMLibSettings::LIBMODE_LOOPCLOSURE:
            mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, calibration, inputSize, depthInputSize);
            break;
        default:
            throw std::runtime_error("Unsupported library mode!");
            break;
    }

    rgb = new ITMUChar4Image(inputSize, true, false);
    depth = new ITMUChar4Image(inputSize, true, false);
    raycast = new ITMUChar4Image(inputSize, true, false);

    pose_output = new slambench::outputs::Output("Pose InfiniTAM", slambench::values::VT_POSE, true);
	slam_settings->GetOutputManager().RegisterOutput(pose_output);

    pointcloud_output = new slambench::outputs::Output("PointCloud InfiniTAM", slambench::values::VT_POINTCLOUD, true);
    pointcloud_output->SetKeepOnlyMostRecent(true);
	slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);

	rgb_frame_output = new slambench::outputs::Output("RGB Frame InfiniTAM", slambench::values::VT_FRAME);
	rgb_frame_output->SetKeepOnlyMostRecent(true);
	slam_settings->GetOutputManager().RegisterOutput(rgb_frame_output);

	depth_frame_output = new slambench::outputs::Output("Depth Frame", slambench::values::VT_FRAME);
	depth_frame_output->SetKeepOnlyMostRecent(true);
	slam_settings->GetOutputManager().RegisterOutput(depth_frame_output);

	render_frame_output = new slambench::outputs::Output("Rendered frame", slambench::values::VT_FRAME);
	render_frame_output->SetKeepOnlyMostRecent(true);
	slam_settings->GetOutputManager().RegisterOutput(render_frame_output);

    return true;

}

bool sb_clean_slam_system() {

    delete mainEngine;
    delete internalSettings;

    if (raycast) delete raycast;
    if (depth) delete depth;
    if (rgb) delete rgb;

    return true;
}

/*
 * Process frames.
 */

// TODO: this is ugly
bool depth_ready = false, rgb_ready = false, imu_ready = false;

bool sb_update_frame (SLAMBenchLibraryHelper * slam_settings, slambench::io::SLAMFrame* s) {
	assert(s != nullptr);
	
	char *target = nullptr;
	
	if(s->FrameSensor == depth_sensor) {
		target = (char*)depthwrapper->GetData(MEMORYDEVICE_CPU);
		depth_ready = true;
	} else if(s->FrameSensor == rgb_sensor) {
		target = (char*)rgbwrapper->GetData(MEMORYDEVICE_CPU);
		rgb_ready = true;
	} else if (s->FrameSensor == imu_sensor) {
		// TODO IMU is a Matrix4f but we only need 3f no translation
		imu_ready = false;
	}
	else {
		//std::cerr << "Unexpected sensor " << s->FrameSensor << ":" << s->FrameSensor->Description << std::endl;
	}

	if(target != nullptr) {
		memcpy(target, s->GetData(), s->GetSize());
		s->FreeData();
	}

	last_frame_timestamp = s->Timestamp;
	return depth_ready && rgb_ready;
}

bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)  {
	if(!depth_ready) return false;
	if(!rgb_ready) return false;

	if (imu_ready) {
		mainEngine->ProcessFrame(rgbwrapper, depthwrapper,imuwrapper);
    } else {
    	mainEngine->ProcessFrame(rgbwrapper, depthwrapper);
    }

#ifndef COMPILE_WITHOUT_CUDA
    ORcudaSafeCall(cudaThreadSynchronize());
#endif

    	depth_ready = false;
    	rgb_ready = false;
    	imu_ready = false;

    return true;

}





/*
 * Output part.
 *
 */



bool sb_get_pose (Eigen::Matrix4f *mat)  {
    mainEngine->GetTrackingState()->pose_d->GetM().getValues(&(*mat)(0,0));
    (*mat) = (Eigen::Matrix4f)(*mat).inverse();
    return true;
}

bool             sb_get_tracked  (bool* tracked)  {
    *tracked = true;
    return true;
}




bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
	(void)lib;

	if(pose_output->IsActive()) {
		// Get the current pose as an eigen matrix
		Eigen::Matrix4f matrix;
		sb_get_pose(&matrix);

		std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		pose_output->AddPoint(last_frame_timestamp, new slambench::values::PoseValue(matrix));
	}

	if(pointcloud_output->IsActive()) {
	    ITMMesh* mesh = new ITMMesh(MemoryDeviceType::MEMORYDEVICE_CPU); //mainEngine->UpdateMesh();
	    slambench::values::PointCloudValue *point_cloud = new slambench::values::PointCloudValue();



	    /// snippet from Mesh header to retrieve cloud even if in GPU
	    //*****************************************************************
        ORUtils::MemoryBlock<ITMMesh::Triangle> *cpu_triangles;
	    bool shoulDelete = false;

	    if (!mesh->triangles) {
	    	std::cout << "Error while retreiving point cloud data, null pointer." << std::endl;
	    } else {


	    	cpu_triangles = mesh->triangles;

#ifndef COMPILE_WITHOUT_CUDA

				cpu_triangles = new ORUtils::MemoryBlock<ITMMesh::Triangle>(mesh->noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(mesh->triangles, ORUtils::MemoryBlock<ITMMesh::Triangle>::CUDA_TO_CPU);
				shoulDelete = true;
#endif


			ITMMesh::Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			if (!triangleArray) {
				std::cout << "Error while retreiving point cloud data" << std::endl;

			} else {

				for (unsigned int idx = 0 ; idx < mesh->noTotalTriangles ; idx++ ) {
						  ITMMesh::Triangle triangle = triangleArray[idx];
						  point_cloud->AddPoint(slambench::values::Point3DF(triangle.p0.x,triangle.p0.y,triangle.p0.z));
						  point_cloud->AddPoint(slambench::values::Point3DF(triangle.p1.x,triangle.p1.y,triangle.p1.z));
						  point_cloud->AddPoint(slambench::values::Point3DF(triangle.p2.x,triangle.p2.y,triangle.p2.z));

				}

				if (shoulDelete) delete cpu_triangles;

			}
	    }
	    //*********************************



		// Take lock only after generating the map
		std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());

		pointcloud_output->AddPoint(last_frame_timestamp, point_cloud);

	}


	if(rgb_frame_output->IsActive()) {
		std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		rgb_frame_output->AddPoint(last_frame_timestamp, new slambench::values::FrameValue(inputSize.x, inputSize.y, slambench::io::pixelformat::RGB_III_888, rgbwrapper->GetData(MEMORYDEVICE_CPU)));
	}

	if(depth_frame_output->IsActive()) {
	    mainEngine->GetImage(depth, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
		std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		depth_frame_output->AddPoint(last_frame_timestamp, new slambench::values::FrameValue(inputSize.x, inputSize.y, slambench::io::pixelformat::RGBA_IIII_8888, depth));
	}


	if(render_frame_output->IsActive()) {
	    mainEngine->GetImage(raycast, ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
		std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		render_frame_output->AddPoint(last_frame_timestamp, new slambench::values::FrameValue(inputSize.x, inputSize.y, slambench::io::pixelformat::RGBA_IIII_8888, raycast));
	}

	return true;
}



