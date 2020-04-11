/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef DEFAULT_PARAMETERS_H_
#define DEFAULT_PARAMETERS_H_

#include <vector>
#include <sstream>


#include <timings.h>
#include <SLAMBenchAPI.h>
#include <Eigen/Core>
#include <ITMLibSettings.h>
#include <ITMTrackerFactory.h>
#include <ITMMainEngine.h>
#include <ImageTypes.h>
#include <ITMLibDefines.h>
using namespace ITMLib;
template<> inline void  TypedParameter<ITMTrackerFactory::TrackerType>::copyValue(ITMTrackerFactory::TrackerType* to,const ITMTrackerFactory::TrackerType* from) {*(ITMTrackerFactory::TrackerType*)to = *(ITMTrackerFactory::TrackerType*)from;};
template<> inline void  TypedParameter<ITMTrackerFactory::TrackerType >::setValue(const char* otarg)          {
    const std::string ss(otarg);
//FIXME: add the rest of the options from InfiniTAM
    if (ss == "COLOR") {
        (*((ITMTrackerFactory::TrackerType*)_ptr)) = ITMTrackerFactory::TRACKER_COLOR ;
    } else if (ss == "ICP") {
        (*((ITMTrackerFactory::TrackerType*)_ptr)) = ITMTrackerFactory::TRACKER_ICP ;
    }else if (ss == "IMU") {
        (*((ITMTrackerFactory::TrackerType*)_ptr)) = ITMTrackerFactory::TRACKER_IMU ;
    }else  {
       exit(1);
    }

}
template<> inline const std::string  TypedParameter< ITMTrackerFactory::TrackerType >::getValue(const void * ptr) {

    switch (*((ITMTrackerFactory::TrackerType*)ptr)) {
    //! Identifies a tracker based on colour image
    case ITMTrackerFactory::TRACKER_COLOR :
        return "COLOR";
        break;
    //! Identifies a tracker based on depth image
    case  ITMTrackerFactory::TRACKER_ICP :
        return "ICP";
        break;
    //! Identifies a tracker based on depth image (Ren et al, 2012)
    case  ITMTrackerFactory::TRACKER_IMU :
        return "IMU";
        break;
    }
    return "ERROR";
}


template<> inline void  TypedParameter<typename std::vector<TrackerIterationType>>::copyValue(std::vector<TrackerIterationType>* to,const std::vector<TrackerIterationType>* from) {
    to->clear();
    for (TrackerIterationType v : *from) {
        to->push_back(v);
    }
};

template<> inline void  TypedParameter< typename std::vector<TrackerIterationType> >::setValue(const char* otarg)          {
    std::vector<TrackerIterationType>* new_ptr = (std::vector<TrackerIterationType>*) _ptr ;
    new_ptr->clear();

    std::stringstream ss;
    ss.str(otarg);
    std::string item;
    while (std::getline(ss, item, ',')) {
        TrackerIterationType item_to_add = TRACKER_ITERATION_NONE;

        if (item == "R" )
            item_to_add = TRACKER_ITERATION_ROTATION;
        if (item == "T" )
            item_to_add = TRACKER_ITERATION_TRANSLATION;
        if (item == "B" )
            item_to_add = TRACKER_ITERATION_BOTH;


        new_ptr->push_back(item_to_add);
    }


};

template<> inline const std::string  TypedParameter< typename std::vector<TrackerIterationType> >::getValue(const void * ptr) {

    std::stringstream ss;
    bool first = true;
    for (TrackerIterationType item : *(std::vector<TrackerIterationType>*)ptr) {
        if (!first) {
            ss << ",";
        }
        if (item == TRACKER_ITERATION_ROTATION ) {
            ss << "R";
        } else if (item == TRACKER_ITERATION_TRANSLATION ) {
            ss << "T";
        }else if (item == TRACKER_ITERATION_BOTH ) {
            ss << "B";
        } else {
            ss << "N";
        }
        first = false;
    }
    return ss.str();
};



static const std::vector<TrackerIterationType>  default_trackingRegime = {
                                                                        TRACKER_ITERATION_BOTH,
                                                                        TRACKER_ITERATION_BOTH,
                                                                        TRACKER_ITERATION_ROTATION,
                                                                        TRACKER_ITERATION_ROTATION,
                                                                        TRACKER_ITERATION_ROTATION};

static const bool default_useSwapping = false;
static const bool default_useBilateralFilter = false;
static const bool default_useApproximateRaycast = false;
static const bool default_modelSensorNoise = false;
static const bool default_skipPoints = true;

static const int default_noICPRunTillLevel = 0;

static const float default_depthTrackerICPThreshold = 0.1f * 0.1f;
static const float default_depthTrackerTerminationThreshold = 1e-3f;


static const float default_mu = 0.02f;
static const int   default_maxW = 100;
static const float default_voxelSize = 0.005f;
static const float default_viewFrustum_min = 0.2f;
static const float default_viewFrustum_max = 3.0f;
static const bool default_stopIntegratingAtMaxW = false;

static const ITMTrackerFactory::TrackerType default_trackerType = ITMTrackerFactory::TRACKER_ICP;



#endif /* DEFAULT_PARAMETERS_H_ */
