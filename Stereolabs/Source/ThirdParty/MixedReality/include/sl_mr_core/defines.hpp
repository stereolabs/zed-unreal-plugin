//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

#ifndef __MR_CORE_DEFINES_H__
#define __MR_CORE_DEFINES_H__

#ifdef _WIN32 
	#ifdef  SLMRCORE_EXPORT  
		#define SLMRCORE_API __declspec(dllexport)   
	#else  
		#define SLMRCORE_API __declspec(dllimport)   
	#endif
#else
    #define SLMRCORE_API
#endif

#include <cstdint>
#include <sl/Camera.hpp>

namespace sl {
	namespace mr {

		/**
		 * \enum HMD_DEVICE_TYPE
		 * \ingroup Enumerations
		 * \brief Types of HMD supported
		 */
		enum class HMD_DEVICE_TYPE : uint8_t {
			HMD_DEVICE_TYPE_UNKNOWN,
			HMD_DEVICE_TYPE_OCULUS,
			HMD_DEVICE_TYPE_HTC
		};

		/**
		 * \struct noiseFactors
		 * \brief Factors of each channel for noise generation
		 */
		struct SLMRCORE_API noiseFactors {

			noiseFactors(sl::float2 r, sl::float2 g, sl::float2 b)
				:
				r(r),
				g(g),
				b(b)
			{}

			/** r channel */
			sl::float2 r;

			/** g channel */
			sl::float2 g;

			/** b channel */
			sl::float2 b;
		};


		/**
		* \struct trackingData
		* \brief Tracking data
		*/
		struct SLMRCORE_API trackingData {
			/** Path transform from tracking origin ((0, 0, 0) in world space relative to left eye) */
			sl::Transform zedPathTransform;

			/** Zed world space transform ((Location/rotation in world space) with anti drift in stereo) */
			sl::Transform zedWorldTransform;

			/**	Zed world transform without camera offset (Head location/rotation) */
			sl::Transform offsetZedWorldTransform;

			/** Tracking state */
			sl::POSITIONAL_TRACKING_STATE trackingState;
		};

        typedef unsigned long long latencyTime;

		/**
		* \struct keyPose
		* \brief A pair transform/time stamp
		*/
		struct SLMRCORE_API keyPose {
			keyPose()
			{}

            keyPose(sl::Transform transform, sl::Timestamp timeStamp)
				:
				transform(transform),
				timeStamp(timeStamp)
			{}

			/** The pose transform */
			sl::Transform transform;

			/** The pose time stamp */
            sl::Timestamp timeStamp;
		};


		/**
		* \struct keyPose
		* \brief A pair transform/time stamp
		*/
		struct SLMRCORE_API keyOrientation {
			keyOrientation()
			{}

            keyOrientation(sl::Orientation orientation, sl::Timestamp timeStamp)
				:
				orientation(orientation),
				timeStamp(timeStamp)
			{}

			/** The pose transform */
			sl::Orientation orientation;

			/** The pose time stamp */
            sl::Timestamp timeStamp;
		};
	}
}

#endif // __DEFINES_H__
