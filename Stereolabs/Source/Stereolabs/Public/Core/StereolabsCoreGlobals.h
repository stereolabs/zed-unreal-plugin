//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

#pragma once

#include "Stereolabs/Public/Core/StereolabsBaseTypes.h"

#include "Math/Matrix.h"
#include "Kismet/KismetMathLibrary.h"
#include "HeadMountedDisplay.h"

#include <sl_mr_core/defines.hpp>

/** Id of the grab thread */
extern STEREOLABS_API uint32 GSlGrabThreadId;

/** True if the grab thread has been initialized */
extern STEREOLABS_API bool GSlIsGrabThreadIdInitialized;

/** Eye half baseline of the Zed */
extern STEREOLABS_API float GSlEyeHalfBaseline;

/** Camera proxy instance */
extern STEREOLABS_API class USlCameraProxy* GSlCameraProxy;

/*
 * @return True if called from the grab thread
 */
FORCEINLINE bool IsInSlGrabThread()
{
	if (GSlIsGrabThreadIdInitialized)
	{
		const uint32 CurrentThreadId = FPlatformTLS::GetCurrentThreadId();
		return CurrentThreadId == GSlGrabThreadId;
	}

	return false;
}

/*
 * @param TextureQualityPreset The preset
 * @return The size of the texture from the quality setting
 */
FORCEINLINE FIntPoint GetSlTextureSizeFromPreset(int32 TextureQualityPreset)
{
	TextureQualityPreset = FMath::Clamp(TextureQualityPreset, 0, 2);

	switch (static_cast<ESlTextureQuality>(TextureQualityPreset))
	{
		case ESlTextureQuality::TQ_High:
			return FIntPoint(1280, 720);
		case ESlTextureQuality::TQ_Medium:
			return FIntPoint(640, 360);
		case ESlTextureQuality::TQ_Low:
			return FIntPoint(384, 192);
		default:
			return FIntPoint(1280, 720);
	}
}

/*
 * Convert ESlTextureFormat to EPixelFormat
 * @param TextureFormat The texture format
 * @return				The EPixelFormat
 */
FORCEINLINE EPixelFormat GetPixelFormatFromSlTextureFormat(ESlTextureFormat TextureFormat)
{
	switch (TextureFormat)
	{
		case ESlTextureFormat::TF_R32_FLOAT:
			return EPixelFormat::PF_R32_FLOAT;
		case ESlTextureFormat::TF_B8G8R8A8_UNORM:
			/*
			 * Replace by PF_R8G8B8A8_UINT if using other engine source code.
			 * Convert texture sample from RGRA to BGRA in material.
			 */
			return EPixelFormat::PF_B8G8R8A8_UNORM; 
		case ESlTextureFormat::TF_R8G8B8A8_UINT:
			return EPixelFormat::PF_R8G8B8A8_UINT;
		case ESlTextureFormat::TF_A32B32G32R32F:
			return EPixelFormat::PF_A32B32G32R32F;
		case ESlTextureFormat::TF_R8_UNORM:
			return EPixelFormat::PF_G8;
	}

	ensureMsgf(false, TEXT("Unhandled ESlTextureFormat entry %u"), (uint32)TextureFormat);
	return PF_Unknown;
}

/*
 * Conversion functions
 */
namespace sl
{
	namespace unreal
	{
		namespace arrays
		{
			/*
			 * Convert from c++/sl array to TArray
			 */
			template<typename Unreal, typename Sl>
			FORCEINLINE TArray<Unreal> ToUnrealType(const std::vector<Sl>& SlArray)
			{
				size_t Size = SlArray.size();
				TArray<Unreal> Array;
				Array.Reserve(Size);

				for (size_t Index = 0; Index < Size; ++Index)
				{
					Array.Add(static_cast<Unreal>(SlArray[Index]));
				}

				return Array;
			}

			/*
			 * Convert from TArray array to c++/sl
			 */
			template<typename Sl, typename Unreal>
			FORCEINLINE std::vector<Sl> ToSlType(const TArray<Unreal>& UnrealArray)
			{
				int32 Size = UnrealArray.Num();
				std::vector<Sl> Array;
				Array.reserve(Size);

				for (int32 Index = 0; Index < Size; ++Index)
				{
					Array.push_back(static_cast<Sl>(UnrealArray[Index]));
				}

				return Array;
			}
		}

 
		/*
		 * Convert from sl::TIME_REFERENCE to ESlTimeReference
		 */
		FORCEINLINE ESlTimeReference ToUnrealType(sl::TIME_REFERENCE SlType)
		{
			switch (SlType)
			{
				case sl::TIME_REFERENCE::IMAGE:
					return ESlTimeReference::TR_Image;
				case sl::TIME_REFERENCE::CURRENT:
					return ESlTimeReference::TR_Current;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::TIME_REFERENCE entry %u"), (uint32)SlType);

					return (ESlTimeReference)0;
				}
			}
		};

		/*
		 * Convert from sl::REFERENCE_FRAME to ESlReferenceFrame
		 */
		FORCEINLINE ESlReferenceFrame ToUnrealType(sl::REFERENCE_FRAME SlType)
		{
			switch (SlType)
			{
				case sl::REFERENCE_FRAME::CAMERA:
					return ESlReferenceFrame::RF_Camera;
				case sl::REFERENCE_FRAME::WORLD:
					return ESlReferenceFrame::RF_World;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::REFERENCE_FRAME entry %u"), (uint32)SlType);

					return (ESlReferenceFrame)0;
				}
			}
		};

		/*
		 * Convert from sl::TRACKING_STATE to EZEDTrackingState
		 */
		FORCEINLINE ESlTrackingState ToUnrealType(sl::POSITIONAL_TRACKING_STATE SlType)
		{
			switch (SlType)
			{
				case sl::POSITIONAL_TRACKING_STATE::SEARCHING:
					return ESlTrackingState::TS_TrackingSearch;
				case sl::POSITIONAL_TRACKING_STATE::OK:
					return ESlTrackingState::TS_TrackingOk;
				case sl::POSITIONAL_TRACKING_STATE::OFF:
					return ESlTrackingState::TS_TrackingOff;
				case sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
					return ESlTrackingState::TS_FpsTooLow;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::TRACKING_STATE entry %u"), (uint32)SlType);

					return (ESlTrackingState)0;
				}
			}
		};

		/*
		 * Convert from ESlResolution to sl::RESOLUTION
		 */
		FORCEINLINE ESlResolution ToUnrealType(sl::RESOLUTION SlType)
		{
			switch (SlType)
			{
				case sl::RESOLUTION::HD2K:
					return ESlResolution::R_HD2K;
				case sl::RESOLUTION::HD1080:
					return ESlResolution::R_HD1080;
				case sl::RESOLUTION::HD720:
					return ESlResolution::R_HD720;
				case sl::RESOLUTION::VGA:
					return ESlResolution::R_VGA;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::RESOLUTION entry %u"), (uint32)SlType);

					return (ESlResolution)0;
				}		
			}
		}

		/*
		 * Convert from sl::ERROR_CODE to ESlErrorCode
		 */
		FORCEINLINE ESlErrorCode ToUnrealType(sl::ERROR_CODE SlType)
		{
			switch (SlType)
			{
				case sl::ERROR_CODE::SUCCESS: /**< Standard code for successful behavior.*/
					return ESlErrorCode::EC_Success;
				case sl::ERROR_CODE::FAILURE: /**< Standard code for unsuccessful behavior.*/
					return ESlErrorCode::EC_Failure;
				case sl::ERROR_CODE::NO_GPU_COMPATIBLE: /**< No GPU found or CUDA capability of the device is not supported.*/
					return ESlErrorCode::EC_NoGpuCompatible;
				case sl::ERROR_CODE::NOT_ENOUGH_GPU_MEMORY: /**< Not enough GPU memory for this depth mode, try a different mode (such as PERFORMANCE), or increase the minimum depth value (see InitParameters::depth_minimum_distance).*/
					return ESlErrorCode::EC_NotEnoughGPUMemory;
				case sl::ERROR_CODE::CAMERA_NOT_DETECTED: /**< The ZED camera is not plugged or detected.*/
					return ESlErrorCode::EC_CameraNotDetected;
				case sl::ERROR_CODE::SENSORS_NOT_AVAILABLE: /**< a ZED-M or ZED2 camera is detected but the sensors (imu,barometer...) cannot be opened. Only for ZED-M or ZED2 devices*/
					return ESlErrorCode::EC_SensorNotAvailable;
				case sl::ERROR_CODE::INVALID_RESOLUTION: /**< In case of invalid resolution parameter, such as a upsize beyond the original image size in Camera::retrieveImage */
					return ESlErrorCode::EC_InvalidResolution;
				case sl::ERROR_CODE::LOW_USB_BANDWIDTH: /**< This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue).*/
					return ESlErrorCode::EC_LowUSBBandwidth;
				case sl::ERROR_CODE::CALIBRATION_FILE_NOT_AVAILABLE: /**< ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.*/
					return ESlErrorCode::EC_CalibrationFileNotAvailable;
				case sl::ERROR_CODE::INVALID_CALIBRATION_FILE: /**< ZED calibration file is not valid, try to download the factory one or recalibrate your camera using 'ZED Calibration'.*/
					return ESlErrorCode::EC_InvalidCalibrationFile;
				case sl::ERROR_CODE::INVALID_SVO_FILE: /**< The provided SVO file is not valid.*/
					return ESlErrorCode::EC_InvalidSVOFile;
				case sl::ERROR_CODE::SVO_RECORDING_ERROR: /**< An recorder related error occurred (not enough free storage, invalid file).*/
					return ESlErrorCode::EC_SVORecordingError;
				case sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION: /**< An SVO related error when NVIDIA based compression cannot be loaded.*/
					return ESlErrorCode::EC_SVOUnsupportedCompression;
				case sl::ERROR_CODE::END_OF_SVOFILE_REACHED: /**<SVO end of file has been reached, and no frame will be available until the SVO position is reset.*/
					return ESlErrorCode::EC_SVOEndOfSVOFile;
				case sl::ERROR_CODE::INVALID_COORDINATE_SYSTEM: /**< The requested coordinate system is not available.*/
					return ESlErrorCode::EC_InvalidCoordinateSystem;
				case sl::ERROR_CODE::INVALID_FIRMWARE: /**< The firmware of the ZED is out of date. Update to the latest version.*/
					return ESlErrorCode::EC_InvalidFirmware;
				case sl::ERROR_CODE::INVALID_FUNCTION_PARAMETERS: /**< An invalid parameter has been set for the function. */
					return ESlErrorCode::EC_InvalidFunctionParameters;
				case sl::ERROR_CODE::CUDA_ERROR: /**< In grab() only, a CUDA error has been detected in the process. Activate verbose in sl::Camera::open for more info.*/
					return ESlErrorCode::EC_CUDAError;
				case sl::ERROR_CODE::CAMERA_NOT_INITIALIZED: /**< In grab() only, ZED SDK is not initialized. Probably a missing call to sl::Camera::open.*/
					return ESlErrorCode::EC_CameraNotInitialized;
				case sl::ERROR_CODE::NVIDIA_DRIVER_OUT_OF_DATE: /**< Your NVIDIA driver is too old and not compatible with your current CUDA version. */
					return ESlErrorCode::EC_NVIDIADriverOutOfDate;
				case sl::ERROR_CODE::INVALID_FUNCTION_CALL: /**< The call of the function is not valid in the current context. Could be a missing call of sl::Camera::open. */
					return ESlErrorCode::EC_InvalidFunctionCall;
				case sl::ERROR_CODE::CORRUPTED_SDK_INSTALLATION: /**< The SDK wasn't able to load its dependencies or somes assets are missing, the installer should be launched. */
					return ESlErrorCode::EC_CorruptedSDKInstallation;
				case sl::ERROR_CODE::INCOMPATIBLE_SDK_VERSION: /**< The installed SDK is incompatible SDK used to compile the program. */
					return ESlErrorCode::EC_IncompatibleSDKVersion;
				case sl::ERROR_CODE::INVALID_AREA_FILE: /**< The given area file does not exist, check the path. */
					return ESlErrorCode::EC_InvalidAreaFile;
				case sl::ERROR_CODE::INCOMPATIBLE_AREA_FILE: /**< The area file does not contain enought data to be used or the sl::DEPTH_MODE used during the creation of the area file is different from the one currently set. */
					return ESlErrorCode::EC_IncompatibleAreaFile;
				case sl::ERROR_CODE::CAMERA_FAILED_TO_SETUP: /**< Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.*/
					return ESlErrorCode::EC_CameraFailedToSetup;
				case sl::ERROR_CODE::CAMERA_DETECTION_ISSUE: /**< Your ZED can not be opened, try replugging it to another USB port or flipping the USB-C connector.*/
					return ESlErrorCode::EC_CameraDetectionIssue;
				case sl::ERROR_CODE::CANNOT_START_CAMERA_STREAM: /**< Cannot start camera stream. Make sure your camera is not already used by another process or blocked by firewall or antivirus.*/
					return ESlErrorCode::EC_CameraFailedToStart;
				case sl::ERROR_CODE::NO_GPU_DETECTED: /**< No GPU found, CUDA is unable to list it. Can be a driver/reboot issue.*/
					return ESlErrorCode::EC_NoGpuDetected;
				case sl::ERROR_CODE::PLANE_NOT_FOUND: /**< Plane not found, either no plane is detected in the scene, at the location or corresponding to the floor, or the floor plane doesn't match the prior given*/
					return ESlErrorCode::EC_NoplaneFound;
				case sl::ERROR_CODE::MODULE_NOT_COMPATIBLE_WITH_CAMERA: /**< The Object detection module is only compatible with the ZED 2*/
					return ESlErrorCode::EC_ModuleNotCompatible;
				case sl::ERROR_CODE::MOTION_SENSORS_REQUIRED: /**< The module needs the sensors to be enabled (see InitParameters::disable_sensors) */
					return ESlErrorCode::EC_MotionSensorsRequired;
				case sl::ERROR_CODE::LAST:
					return ESlErrorCode::EC_None;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::ERROR_CODE entry %u"), (uint32)SlType);

					return (ESlErrorCode)0;
				}
			}

		}

		/*
		 * Convert from sl::AREA_EXPORT_STATE to ESlSpatialMemoryExportState
		 */
		FORCEINLINE ESlSpatialMemoryExportingState ToUnrealType(sl::AREA_EXPORTING_STATE SlType)
		{
			switch (SlType)
			{
				case sl::AREA_EXPORTING_STATE::SUCCESS:
					return ESlSpatialMemoryExportingState::SMES_Success;
				case sl::AREA_EXPORTING_STATE::RUNNING:
					return ESlSpatialMemoryExportingState::SMES_Running;
				case sl::AREA_EXPORTING_STATE::NOT_STARTED:
					return ESlSpatialMemoryExportingState::SMES_NotStarted;
				case sl::AREA_EXPORTING_STATE::FILE_EMPTY:
					return ESlSpatialMemoryExportingState::SMES_FileEmpty;
				case sl::AREA_EXPORTING_STATE::FILE_ERROR:
					return ESlSpatialMemoryExportingState::SMES_FileError;
				case sl::AREA_EXPORTING_STATE::SPATIAL_MEMORY_DISABLED:
					return ESlSpatialMemoryExportingState::SMES_SpatialMemoryDisabled;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::AREA_EXPORT_STATE entry %u"), (uint32)SlType);

					return (ESlSpatialMemoryExportingState)0;
				}
			}
		}

		/*
		 * Convert from sl::MAT_TYPE to ESlMatType
		 */
		FORCEINLINE ESlMatType ToUnrealType(sl::MAT_TYPE SlType)
		{
			switch (SlType)
			{
				case sl::MAT_TYPE::F32_C1:
					return ESlMatType::MT_32F_C1;
				case sl::MAT_TYPE::F32_C2:
					return ESlMatType::MT_32F_C2;
				case sl::MAT_TYPE::F32_C3:
					return ESlMatType::MT_32F_C3;
				case sl::MAT_TYPE::F32_C4:
					return ESlMatType::MT_32F_C4;
				case sl::MAT_TYPE::U8_C1:
					return ESlMatType::MT_8U_C1;
				case sl::MAT_TYPE::U8_C2:
					return ESlMatType::MT_8U_C2;
				case sl::MAT_TYPE::U8_C3:
					return ESlMatType::MT_8U_C3;
				case sl::MAT_TYPE::U8_C4:
					return ESlMatType::MT_8U_C4;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::MAT_TYPE entry %u"), (uint32)SlType);

					return (ESlMatType)0;
				}
						
			}
		}

		/*
		 * Convert from sl::MEM to ESlMemoryType
		 */
		FORCEINLINE ESlMemoryType ToUnrealType(sl::MEM SlType)
		{
			if (SlType == sl::MEM::CPU)
			{
				return ESlMemoryType::MT_CPU;
			}
			else if (SlType == sl::MEM::GPU)
			{
				return ESlMemoryType::MT_GPU;
			}
			else if (SlType == (sl::MEM::CPU | sl::MEM::GPU))
			{
				return (ESlMemoryType::MT_CPU | ESlMemoryType::MT_GPU);
			}

			ensureMsgf(false, TEXT("Unhandled sl::MEM entry %u"), (uint32)SlType);

			return (ESlMemoryType)0;
		}

		/*
		 * Convert from sl::MODEL to ESlModel
		 */
		FORCEINLINE ESlModel ToUnrealType(sl::MODEL SlType)
		{
			switch (SlType)
			{
				case sl::MODEL::ZED:
					return ESlModel::M_Zed;
				case sl::MODEL::ZED_M:
					return ESlModel::M_ZedM;
				case sl::MODEL::ZED2:
					return ESlModel::M_Zed2;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::MODEL entry %u"), (uint32)SlType);

					return (ESlModel)0;
				}
			}
		}

		/*
		 * Convert from sl::CAMERA_STATE to ESlCameraState
		 */
		FORCEINLINE ESlCameraState ToUnrealType(sl::CAMERA_STATE SlType)
		{
			switch (SlType)
			{
				case sl::CAMERA_STATE::AVAILABLE:
					return ESlCameraState::CS_Available;
				case sl::CAMERA_STATE::NOT_AVAILABLE:
					return ESlCameraState::CS_NotAvailable;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::CAMERA_STATE entry %u"), (uint32)SlType);

					return (ESlCameraState)0;
				}
			}
		}

		/*
	 	 * Convert from sl::SENSING_MODE to ESlSensingMode
	 	 */
		FORCEINLINE ESlSensingMode ToUnrealType(sl::SENSING_MODE SlType)
		{
			switch (SlType)
			{
				case sl::SENSING_MODE::FILL:
					return ESlSensingMode::SM_Fill;
				case sl::SENSING_MODE::STANDARD:
					return ESlSensingMode::SM_Standard;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled sl::SENSING_MODE entry %u"), (uint32)SlType);

					return (ESlSensingMode)0;
				}
			}
		}

		/*
		 * Convert from ESlTimeReference to sl::TIME_REFERENCE
		 */
		FORCEINLINE sl::TIME_REFERENCE ToSlType(ESlTimeReference UnrealType)
		{
			switch (UnrealType)
			{
				case ESlTimeReference::TR_Image:
					return sl::TIME_REFERENCE::IMAGE;
				case ESlTimeReference::TR_Current:
					return sl::TIME_REFERENCE::CURRENT;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlTimeReference entry %u"), (uint32)UnrealType);

					return (sl::TIME_REFERENCE)0;
				}
			}
		};

		/*
		 * Convert from ESlReferenceFrame to sl::REFERENCE_FRAME
		 */
		FORCEINLINE sl::REFERENCE_FRAME ToSlType(ESlReferenceFrame UnrealType)
		{
			switch (UnrealType)
			{
				case ESlReferenceFrame::RF_Camera:
					return sl::REFERENCE_FRAME::CAMERA;
				case ESlReferenceFrame::RF_World:
					return sl::REFERENCE_FRAME::WORLD;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlReferenceFrame entry %u"), (uint32)UnrealType);

					return (sl::REFERENCE_FRAME)0;
				}
			}
		};

		/*
		 * Convert from ESlTrackingState to sl::TRACKING_STATE
		 */
		FORCEINLINE sl::POSITIONAL_TRACKING_STATE ToSlType(ESlTrackingState UnrealType)
		{
			switch (UnrealType)
			{
				case ESlTrackingState::TS_TrackingSearch:
					return sl::POSITIONAL_TRACKING_STATE::SEARCHING;
				case ESlTrackingState::TS_TrackingOk:
					return sl::POSITIONAL_TRACKING_STATE::OK;
				case ESlTrackingState::TS_TrackingOff:
					return sl::POSITIONAL_TRACKING_STATE::OFF;
				case ESlTrackingState::TS_FpsTooLow:
					return sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlTrackingState entry %u"), (uint32)UnrealType);

					return (sl::POSITIONAL_TRACKING_STATE)0;
				}
			}
		}

		/*
		 * Convert from ESlResolution to sl::RESOLUTION
		 */
		FORCEINLINE sl::RESOLUTION ToSlType(ESlResolution UnrealType)
		{
			switch (UnrealType)
			{
				case ESlResolution::R_HD2K:
					return sl::RESOLUTION::HD2K;
				case ESlResolution::R_HD1080:
					return sl::RESOLUTION::HD1080;
				case ESlResolution::R_HD720:
					return sl::RESOLUTION::HD720;
				case ESlResolution::R_VGA:
					return sl::RESOLUTION::VGA;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlResolution entry %u"), (uint32)UnrealType);

					return (sl::RESOLUTION)0;
				}		
			}
		}

		/*
		 * Convert from ESlDepthMode to sl::DEPTH_MODE
		 */
		FORCEINLINE sl::DEPTH_MODE ToSlType(ESlDepthMode UnrealType)
		{
			switch (UnrealType)
			{
				case ESlDepthMode::DM_None:
					return sl::DEPTH_MODE::NONE;
				case ESlDepthMode::DM_Performance:
					return sl::DEPTH_MODE::PERFORMANCE;
				case ESlDepthMode::DM_Quality:
					return sl::DEPTH_MODE::QUALITY;
				case ESlDepthMode::DM_Ultra:
					return sl::DEPTH_MODE::ULTRA;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlDepthMode entry %u"), (uint32)UnrealType);

					return (sl::DEPTH_MODE)0;
				}		
			}
		}

		/*
		 * Convert from ESlMeasure to sl::MEASURE
		 */
		FORCEINLINE sl::MEASURE ToSlType(ESlMeasure UnrealType)
		{
			switch (UnrealType)
			{
				case ESlMeasure::M_Depth:
					return sl::MEASURE::DEPTH;
				case ESlMeasure::M_DepthRight:
					return sl::MEASURE::DEPTH_RIGHT;
				case ESlMeasure::M_Disparity:
					return sl::MEASURE::DISPARITY;
				case ESlMeasure::M_DisparityRight:
					return sl::MEASURE::DISPARITY_RIGHT;
				case ESlMeasure::M_Normals:
					return sl::MEASURE::NORMALS;
				case ESlMeasure::M_NormalsRight:
					return sl::MEASURE::NORMALS_RIGHT;
				case ESlMeasure::M_XYZ:
					return sl::MEASURE::XYZ;
				case ESlMeasure::M_XYZ_Right:
					return sl::MEASURE::XYZ_RIGHT;
				case ESlMeasure::M_XYZ_RGBA:
					return sl::MEASURE::XYZRGBA;
				case ESlMeasure::M_XYZ_RGBA_Right:
					return sl::MEASURE::XYZRGBA_RIGHT;
				case ESlMeasure::M_XYZ_BGRA:
					return sl::MEASURE::XYZBGRA;
				case ESlMeasure::M_XYZ_BGRA_Right:
					return sl::MEASURE::XYZBGRA_RIGHT;
				case ESlMeasure::M_XYZ_ARGB:
					return sl::MEASURE::XYZARGB;
				case ESlMeasure::M_XYZ_ARGB_Right:
					return sl::MEASURE::XYZARGB_RIGHT;
				case ESlMeasure::M_XYZ_ABGR:
					return sl::MEASURE::XYZABGR;
				case ESlMeasure::M_XYZ_ABGR_Right:
					return sl::MEASURE::XYZABGR_RIGHT;
				case ESlMeasure::M_Confidence:
					return sl::MEASURE::CONFIDENCE;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlMeasure entry %u"), (uint32)UnrealType);

					return (sl::MEASURE)0;
				}
			}
		}

		/*
		 * Convert from ESlView to sl::VIEW
		 */
		FORCEINLINE sl::VIEW ToSlType(ESlView UnrealType)
		{
			switch (UnrealType)
			{
				case ESlView::V_Left:
					return  sl::VIEW::LEFT;
				case ESlView::V_Right:
					return sl::VIEW::RIGHT;
				case ESlView::V_LeftUnrectified:
					return sl::VIEW::LEFT_UNRECTIFIED;
				case ESlView::V_RightUnrectified:
					return sl::VIEW::RIGHT_UNRECTIFIED;
				case ESlView::V_LeftUnrectifiedGray:
					return sl::VIEW::LEFT_UNRECTIFIED_GRAY;
				case ESlView::V_RightUnrectifiedGray:
					return sl::VIEW::RIGHT_UNRECTIFIED_GRAY;
				case ESlView::V_LeftGray:
					return sl::VIEW::LEFT_GRAY;
				case ESlView::V_RightGray:
					return sl::VIEW::RIGHT_GRAY;
				case ESlView::V_SideBySide:
					return sl::VIEW::SIDE_BY_SIDE;
				case ESlView::V_Depth:
					return sl::VIEW::DEPTH;
				case ESlView::V_DepthRight:
					return sl::VIEW::DEPTH_RIGHT;
				case ESlView::V_Confidence:
					return sl::VIEW::CONFIDENCE;
				case ESlView::V_Normals:
					return sl::VIEW::NORMALS;
				case ESlView::V_NormalsRight:
					return sl::VIEW::NORMALS_RIGHT;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlView entry %u"), (uint32)UnrealType);

					return (sl::VIEW)0;
				}
			}
		}
		
		/*
		 * Convert from EStereoRenderingDeviceType to sl::mr::HMD_DEVICE_TYPE
		 */
		FORCEINLINE sl::mr::HMD_DEVICE_TYPE ToSlType(FName UnrealType)
		{
			if (UnrealType == TEXT("SteamVR"))
			{
				return sl::mr::HMD_DEVICE_TYPE::HMD_DEVICE_TYPE_HTC;
			}
			else if (UnrealType == TEXT("OculusHMD"))
			{
				return sl::mr::HMD_DEVICE_TYPE::HMD_DEVICE_TYPE_OCULUS;
			}
			else
			{
				ensureMsgf(false, TEXT("Unhandled HMD device type FName entry"));
				return sl::mr::HMD_DEVICE_TYPE::HMD_DEVICE_TYPE_UNKNOWN;
			}
		}

		/*
		 * Convert from ESlErrorCode to sl::ERROR_CODE
		 */
		FORCEINLINE sl::ERROR_CODE ToSlType(ESlErrorCode UnrealType)
		{
			switch (UnrealType)
			{
				case ESlErrorCode::EC_Success: /**< Standard code for successful behavior.*/
					return sl::ERROR_CODE::SUCCESS;
				case ESlErrorCode::EC_Failure: /**< Standard code for unsuccessful behavior.*/
					return sl::ERROR_CODE::FAILURE;
				case ESlErrorCode::EC_NoGpuCompatible: /**< No GPU found or CUDA capability of the device is not supported.*/
					return sl::ERROR_CODE::NO_GPU_COMPATIBLE;
				case ESlErrorCode::EC_NotEnoughGPUMemory: /**< Not enough GPU memory for this depth mode, try a different mode (such as PERFORMANCE), or increase the minimum depth value (see InitParameters::depth_minimum_distance).*/
					return sl::ERROR_CODE::NOT_ENOUGH_GPU_MEMORY ;
				case ESlErrorCode::EC_CameraNotDetected: /**< The ZED camera is not plugged or detected.*/
					return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
				case ESlErrorCode::EC_SensorNotAvailable: /**< a ZED-M or ZED2 camera is detected but the sensors (imu,barometer...) cannot be opened. Only for ZED-M or ZED2 devices*/
					return sl::ERROR_CODE::SENSORS_NOT_AVAILABLE;
				case ESlErrorCode::EC_InvalidResolution: /**< In case of invalid resolution parameter, such as a upsize beyond the original image size in Camera::retrieveImage */
					return sl::ERROR_CODE::INVALID_RESOLUTION;
				case  ESlErrorCode::EC_LowUSBBandwidth: /**< This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue).*/
					return sl::ERROR_CODE::LOW_USB_BANDWIDTH;
				case ESlErrorCode::EC_CalibrationFileNotAvailable: /**< ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.*/
					return sl::ERROR_CODE::CALIBRATION_FILE_NOT_AVAILABLE;
				case ESlErrorCode::EC_InvalidCalibrationFile: /**< ZED calibration file is not valid, try to download the factory one or recalibrate your camera using 'ZED Calibration'.*/
					return sl::ERROR_CODE::INVALID_CALIBRATION_FILE;
				case ESlErrorCode::EC_InvalidSVOFile : /**< The provided SVO file is not valid.*/
					return sl::ERROR_CODE::INVALID_SVO_FILE;
				case ESlErrorCode::EC_SVORecordingError : /**< An recorder related error occurred (not enough free storage, invalid file).*/
					return sl::ERROR_CODE::SVO_RECORDING_ERROR;
				case ESlErrorCode::EC_SVOUnsupportedCompression : /**< An SVO related error when NVIDIA based compression cannot be loaded.*/
					return sl::ERROR_CODE::SVO_UNSUPPORTED_COMPRESSION;
				case ESlErrorCode::EC_SVOEndOfSVOFile: /**<SVO end of file has been reached, and no frame will be available until the SVO position is reset.*/
					return sl::ERROR_CODE::END_OF_SVOFILE_REACHED;
				case ESlErrorCode::EC_InvalidCoordinateSystem: /**< The requested coordinate system is not available.*/
					return sl::ERROR_CODE::INVALID_COORDINATE_SYSTEM ;
				case ESlErrorCode::EC_InvalidFirmware : /**< The firmware of the ZED is out of date. Update to the latest version.*/
					return sl::ERROR_CODE::INVALID_FIRMWARE;
				case  ESlErrorCode::EC_InvalidFunctionParameters: /**< An invalid parameter has been set for the function. */
					return sl::ERROR_CODE::INVALID_FUNCTION_PARAMETERS;
				case ESlErrorCode::EC_CUDAError: /**< In grab() only, a CUDA error has been detected in the process. Activate verbose in sl::Camera::open for more info.*/
					return sl::ERROR_CODE::CUDA_ERROR ;
				case ESlErrorCode::EC_CameraNotInitialized : /**< In grab() only, ZED SDK is not initialized. Probably a missing call to sl::Camera::open.*/
					return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
				case ESlErrorCode::EC_NVIDIADriverOutOfDate: /**< Your NVIDIA driver is too old and not compatible with your current CUDA version. */
					return sl::ERROR_CODE::NVIDIA_DRIVER_OUT_OF_DATE;
				case ESlErrorCode::EC_InvalidFunctionCall: /**< The call of the function is not valid in the current context. Could be a missing call of sl::Camera::open. */
					return sl::ERROR_CODE::INVALID_FUNCTION_CALL;
				case ESlErrorCode::EC_CorruptedSDKInstallation: /**< The SDK wasn't able to load its dependencies or somes assets are missing, the installer should be launched. */
					return sl::ERROR_CODE::CORRUPTED_SDK_INSTALLATION;
				case ESlErrorCode::EC_IncompatibleSDKVersion: /**< The installed SDK is incompatible SDK used to compile the program. */
					return sl::ERROR_CODE::INCOMPATIBLE_SDK_VERSION ;
				case ESlErrorCode::EC_InvalidAreaFile: /**< The given area file does not exist, check the path. */
					return sl::ERROR_CODE::INVALID_AREA_FILE;
				case ESlErrorCode::EC_IncompatibleAreaFile: /**< The area file does not contain enought data to be used or the sl::DEPTH_MODE used during the creation of the area file is different from the one currently set. */
					return sl::ERROR_CODE::INCOMPATIBLE_AREA_FILE;
				case ESlErrorCode::EC_CameraFailedToSetup: /**< Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.*/
					return sl::ERROR_CODE::CAMERA_FAILED_TO_SETUP;
				case ESlErrorCode::EC_CameraDetectionIssue : /**< Your ZED can not be opened, try replugging it to another USB port or flipping the USB-C connector.*/
					return sl::ERROR_CODE::CAMERA_DETECTION_ISSUE;
				case ESlErrorCode::EC_CameraFailedToStart : /**< Cannot start camera stream. Make sure your camera is not already used by another process or blocked by firewall or antivirus.*/
					return sl::ERROR_CODE::CANNOT_START_CAMERA_STREAM;
				case ESlErrorCode::EC_NoGpuDetected: /**< No GPU found, CUDA is unable to list it. Can be a driver/reboot issue.*/
					return  sl::ERROR_CODE::NO_GPU_DETECTED;
				case ESlErrorCode::EC_NoplaneFound : /**< Plane not found, either no plane is detected in the scene, at the location or corresponding to the floor, or the floor plane doesn't match the prior given*/
					return sl::ERROR_CODE::PLANE_NOT_FOUND;
				case ESlErrorCode::EC_ModuleNotCompatible : /**< The Object detection module is only compatible with the ZED 2*/
					return sl::ERROR_CODE::MODULE_NOT_COMPATIBLE_WITH_CAMERA;
				case  ESlErrorCode::EC_MotionSensorsRequired: /**< The module needs the sensors to be enabled (see InitParameters::disable_sensors) */
					return sl::ERROR_CODE::MOTION_SENSORS_REQUIRED;
				case ESlErrorCode::EC_None:
					return  sl::ERROR_CODE::LAST;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlErrorCode entry %u"), (uint32)UnrealType);

					return (sl::ERROR_CODE)0;
				}
			}
		}

		/*
		 * Convert from ESlSpatialMemoryExportState to sl::AREA_EXPORT_STATE
		 */
		FORCEINLINE sl::AREA_EXPORTING_STATE ToSlType(ESlSpatialMemoryExportingState UnrealType)
		{
			switch (UnrealType)
			{
				case ESlSpatialMemoryExportingState::SMES_Success:
					return sl::AREA_EXPORTING_STATE::SUCCESS;
				case ESlSpatialMemoryExportingState::SMES_Running:
					return sl::AREA_EXPORTING_STATE::RUNNING;
				case ESlSpatialMemoryExportingState::SMES_NotStarted:
					return sl::AREA_EXPORTING_STATE::NOT_STARTED;
				case ESlSpatialMemoryExportingState::SMES_FileEmpty:
					return sl::AREA_EXPORTING_STATE::FILE_EMPTY;
				case ESlSpatialMemoryExportingState::SMES_FileError:
					return sl::AREA_EXPORTING_STATE::FILE_ERROR;
				case ESlSpatialMemoryExportingState::SMES_SpatialMemoryDisabled:
					return sl::AREA_EXPORTING_STATE::SPATIAL_MEMORY_DISABLED;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlSpatialMemoryExportState entry %u"), (uint32)UnrealType);

					return (sl::AREA_EXPORTING_STATE)0;
				}
			}
		}

		/*
		 * Convert from ESlSVOCompressionMode to sl::SVO_COMPRESSION_MODE
		 */
		FORCEINLINE sl::SVO_COMPRESSION_MODE ToSlType(ESlSVOCompressionMode UnrealType)
		{
			switch (UnrealType)
			{
				case ESlSVOCompressionMode::SCM_Lossless:
					return sl::SVO_COMPRESSION_MODE::LOSSLESS;
				case ESlSVOCompressionMode::SCM_H264:
					return sl::SVO_COMPRESSION_MODE::H264;
				case ESlSVOCompressionMode::SCM_H265:
					return sl::SVO_COMPRESSION_MODE::H265;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlSVOCompressionMode entry %u"), (uint32)UnrealType);

					return (sl::SVO_COMPRESSION_MODE)0;
				}
			}
		}

		/*
		 * Convert from ESlMeshFileFormat to sl::MESH_FILE_FORMAT
		 */
		FORCEINLINE sl::MESH_FILE_FORMAT ToSlType(ESlMeshFileFormat UnrealType)
		{
			switch (UnrealType)
			{
				case ESlMeshFileFormat::MFF_OBJ:
					return sl::MESH_FILE_FORMAT::OBJ;
				case ESlMeshFileFormat::MFF_PLY:
					return sl::MESH_FILE_FORMAT::PLY;
				case ESlMeshFileFormat::MFF_PLY_BIN:
					return sl::MESH_FILE_FORMAT::PLY_BIN;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlMeshFileFormat entry %u"), (uint32)UnrealType);

					return (sl::MESH_FILE_FORMAT)0;
				}
			}
		}

		/*
		 * Convert from ESlMeshTextureFormat to sl::MESH_TEXTURE_FORMAT
	 	 */
		FORCEINLINE sl::MESH_TEXTURE_FORMAT ToSlType(ESlMeshTextureFormat UnrealType)
		{
			switch (UnrealType)
			{
				case ESlMeshTextureFormat::MTF_RGB:
					return sl::MESH_TEXTURE_FORMAT::RGB;
				case ESlMeshTextureFormat::MTF_RGBA:
					return sl::MESH_TEXTURE_FORMAT::RGBA;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlMeshTextureFormat entry %u"), (uint32)UnrealType);

					return (sl::MESH_TEXTURE_FORMAT)0;
				}
			}
		}

		/*
		 * Convert from ESlFilterIntensity to sl::MeshFilterParameters::FILTER
		 */
		FORCEINLINE sl::MeshFilterParameters::MESH_FILTER ToSlType(ESlFilterIntensity UnrealType)
		{
			switch (UnrealType)
			{
				case ESlFilterIntensity::FI_Low:
					return  sl::MeshFilterParameters::MESH_FILTER::LOW;
				case ESlFilterIntensity::FI_Medium:
					return  sl::MeshFilterParameters::MESH_FILTER::MEDIUM;
				case ESlFilterIntensity::FI_High:
					return  sl::MeshFilterParameters::MESH_FILTER::HIGH;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlFilterIntensity entry %u"), (uint32)UnrealType);

					return (sl::MeshFilterParameters::MESH_FILTER)0;
				}
			}
		}
		
		/*
		 * Convert from ESlUnit to sl::UNIT
		 */
		FORCEINLINE sl::UNIT ToSlType(ESlUnit UnrealType)
		{
			switch (UnrealType)
			{
				case ESlUnit::DU_Centimeter:
					return sl::UNIT::CENTIMETER;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlUnit entry %u"), (uint32)UnrealType);

					return (sl::UNIT)0;
				}
			}
		}

		/*
		 * Convert from ESlCoordinateSystem to sl::COORDINATE_SYSTEM
		 */
		FORCEINLINE sl::COORDINATE_SYSTEM ToSlType(ESlCoordinateSystem UnrealType)
		{
			switch (UnrealType)
			{
				case ESlCoordinateSystem::CS_Cartesian:
					return sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlCoordinateSystem entry %u"), (uint32)UnrealType);

					return (sl::COORDINATE_SYSTEM)0;
				}
			}
		}

		/*
		 * Convert from ESlCopyType to sl::COPY_TYPE
		 */
		FORCEINLINE sl::COPY_TYPE ToSlType(ESlCopyType UnrealType)
		{
			switch (UnrealType)
			{
				case ESlCopyType::CT_CPUToCPU:
					return sl::COPY_TYPE::CPU_CPU;
				case ESlCopyType::CT_CPUToGPU:
					return sl::COPY_TYPE::CPU_GPU;
				case ESlCopyType::CT_GPUToCPU:
					return sl::COPY_TYPE::GPU_CPU;
				case ESlCopyType::CT_GPUToGPU:
					return sl::COPY_TYPE::GPU_GPU;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlCopyType entry %u"), (uint32)UnrealType);

					return (sl::COPY_TYPE)0;
				}
			}
		}

		/*
		 * Convert from ESlMatType to sl::MAT_TYPE
		 */
		FORCEINLINE sl::MAT_TYPE ToSlType(ESlMatType UnrealType)
		{
			switch (UnrealType)
			{
				case ESlMatType::MT_32F_C1:
					return sl::MAT_TYPE::F32_C1;
				case ESlMatType::MT_32F_C2:
					return sl::MAT_TYPE::F32_C2;
				case ESlMatType::MT_32F_C3:
					return sl::MAT_TYPE::F32_C3;
				case  ESlMatType::MT_32F_C4:
					return sl::MAT_TYPE::F32_C4;
				case ESlMatType::MT_8U_C1:
					return sl::MAT_TYPE::U8_C1;
				case ESlMatType::MT_8U_C2:
					return sl::MAT_TYPE::U8_C2;
				case ESlMatType::MT_8U_C3:
					return sl::MAT_TYPE::U8_C3;
				case ESlMatType::MT_8U_C4:
					return sl::MAT_TYPE::U8_C4;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlMatType entry %u"), (uint32)UnrealType);

					return (sl::MAT_TYPE)0;
				}
			}
		}

		/*
		 * Convert from ESlModel to sl::MODEL
		 */
		FORCEINLINE sl::MODEL ToSlType(ESlModel UnrealType)
		{
			switch (UnrealType)
			{
				case ESlModel::M_Zed:
					return  sl::MODEL::ZED;
				case  ESlModel::M_ZedM:
					return sl::MODEL::ZED_M;
				case  ESlModel::M_Zed2:
					return sl::MODEL::ZED2;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlModel entry %u"), (uint32)UnrealType);

					return (sl::MODEL)0;
				}
			}
		}

		/*
		 * Convert from ESlCameraState to sl::CAMERA_STATE
		 */
		FORCEINLINE sl::CAMERA_STATE ToSlType(ESlCameraState UnrealType)
		{
			switch (UnrealType)
			{
				case ESlCameraState::CS_Available:
					return sl::CAMERA_STATE::AVAILABLE;
				case ESlCameraState::CS_NotAvailable:
					return sl::CAMERA_STATE::NOT_AVAILABLE;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlCameraState entry %u"), (uint32)UnrealType);

					return (sl::CAMERA_STATE)0;
				}
			}
		}

		/*
		 * Convert from ESlMemoryType to sl::MEM
		 */
		FORCEINLINE sl::MEM ToSlType(ESlMemoryType UnrealType)
		{
			if (UnrealType == ESlMemoryType::MT_CPU)
			{
				return sl::MEM::CPU;
			}
			else if(UnrealType == ESlMemoryType::MT_GPU)
			{
				return sl::MEM::GPU;
			}
			else if (UnrealType == (ESlMemoryType::MT_CPU | ESlMemoryType::MT_GPU))
			{
				return (sl::MEM::CPU | sl::MEM::GPU);
			}

			ensureMsgf(false, TEXT("Unhandled ESlMemoryType entry %u"), (uint32)UnrealType);

			return (sl::MEM)0;
		}

		/*
		 * Convert from ESlSpatialMappingRange to sl::SpatialMappingParameters::RANGE
		 */
		FORCEINLINE sl::SpatialMappingParameters::MAPPING_RANGE ToSlType(ESlSpatialMappingRange UnrealType)
		{
			switch (UnrealType)
			{
				case ESlSpatialMappingRange::SMR_Short:
					return sl::SpatialMappingParameters::MAPPING_RANGE::SHORT;
				case ESlSpatialMappingRange::SMR_Medium:
					return sl::SpatialMappingParameters::MAPPING_RANGE::MEDIUM;
				case ESlSpatialMappingRange::SMR_Long:
					return sl::SpatialMappingParameters::MAPPING_RANGE::LONG;
				case ESlSpatialMappingRange::SMR_Auto:
					return sl::SpatialMappingParameters::MAPPING_RANGE::AUTO;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlSpatialMappingRange entry %u"), (uint32)UnrealType);

					return (sl::SpatialMappingParameters::MAPPING_RANGE)0;
				}
			}
		}

		/*
		 * Convert from ESlSpatialMappingResolution to sl::SpatialMappingParameters::RESOLUTION
		 */
		FORCEINLINE sl::SpatialMappingParameters::MAPPING_RESOLUTION ToSlType(ESlSpatialMappingResolution UnrealType)
		{
			switch (UnrealType)
			{
				case ESlSpatialMappingResolution::SMR_Low:
					return sl::SpatialMappingParameters::MAPPING_RESOLUTION::LOW;
				case ESlSpatialMappingResolution::SMR_Medium:
					return sl::SpatialMappingParameters::MAPPING_RESOLUTION::MEDIUM;
				case ESlSpatialMappingResolution::SMR_High:
					return sl::SpatialMappingParameters::MAPPING_RESOLUTION::HIGH;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlSpatialMappingResolution entry %u"), (uint32)UnrealType);

					return (sl::SpatialMappingParameters::MAPPING_RESOLUTION)0;
				}
			}
		}

		/*
	 	 * Convert from ESlSensingMode to sl::SENSING_MODE
	 	 */
		FORCEINLINE sl::SENSING_MODE ToSlType(ESlSensingMode UnrealType)
		{
			switch (UnrealType)
			{
				case ESlSensingMode::SM_Fill:
					return sl::SENSING_MODE::FILL;
				case ESlSensingMode::SM_Standard:
					return sl::SENSING_MODE::STANDARD;
				default:
				{
					ensureMsgf(false, TEXT("Unhandled ESlSensingMode entry %u"), (uint32)UnrealType);

					return (sl::SENSING_MODE)0;
				}
			}
		}

		/*
		 * Convert from sl::Resolution to FIntPoint
		 */
		FORCEINLINE FIntPoint ToUnrealType(const sl::Resolution& SlVector)
		{
			return FIntPoint(SlVector.width, SlVector.height);

		}

		/*
		 * Convert from sl::uchar2 to FIntPoint
		 */
		FORCEINLINE FIntPoint ToUnrealType(const sl::uchar2& SlVector)
		{
			return FIntPoint(SlVector.x, SlVector.y);
		}

		/*
		 * Convert from sl::uchar3 to FIntVector
		 */
		FORCEINLINE FIntVector ToUnrealType(const sl::uchar3& SlVector)
		{
			return FIntVector(SlVector.x, SlVector.y, SlVector.z);
		}

		/*
		 * Convert from sl::uchar4 to FColor
		 */
		FORCEINLINE FColor ToUnrealType(const sl::uchar4& SlColor)
		{
			return FColor(SlColor.x, SlColor.y, SlColor.z, SlColor.w);
		}

		/*
	 	 * Convert from sl::float2 to FVector2D
		 */
		FORCEINLINE FVector2D ToUnrealType(const sl::float2& SlVector)
		{
			return FVector2D(SlVector.x, SlVector.y);
		}

		/*
		 * Convert from sl::float3 to FVector
		*/
		 FORCEINLINE FVector ToUnrealType(const sl::float3& SlVector)
		{
			return FVector(SlVector.x, SlVector.y, SlVector.z);
		}

		/*
		 * Convert from sl::float4 to FVector4
		 */
		FORCEINLINE FVector4 ToUnrealType(const sl::float4& SlVector)
		{
			return FVector4(SlVector.x, SlVector.y, SlVector.z, SlVector.w);
		}

		/*
		 * Convert from sl::Orientation to FQuat
		 */
		FORCEINLINE FQuat ToUnrealType(const sl::Orientation& SlQuat)
		{
			return FQuat(SlQuat.x, SlQuat.y, SlQuat.z, SlQuat.w);
		}

		/*
		 * Convert from sl::Matrix4f to FMatrix (row to column)
		 */
		FORCEINLINE FMatrix ToUnrealType(const sl::Matrix4f& SlMatrix)
		{
			FMatrix Matrix;

			// X plane
			Matrix.M[0][0] = SlMatrix.r00;
			Matrix.M[0][1] = SlMatrix.r10;
			Matrix.M[0][2] = SlMatrix.r20;
			Matrix.M[0][3] = SlMatrix.m30;

			// Y plane
			Matrix.M[1][0] = SlMatrix.r01;
			Matrix.M[1][1] = SlMatrix.r11;
			Matrix.M[1][2] = SlMatrix.r21;
			Matrix.M[1][3] = SlMatrix.m31;

			// Z plane
			Matrix.M[2][0] = SlMatrix.r02;
			Matrix.M[2][1] = SlMatrix.r12;
			Matrix.M[2][2] = SlMatrix.r22;
			Matrix.M[2][3] = SlMatrix.m32;

			// Origin
			Matrix.M[3][0] = SlMatrix.tx;
			Matrix.M[3][1] = SlMatrix.ty;
			Matrix.M[3][2] = SlMatrix.tz;
			Matrix.M[3][3] = SlMatrix.m33;

			return Matrix;
		}

		/*
		 * Convert from sl::Matrix3f to FRotationMatrix (row to column)
		 */
		FORCEINLINE FMatrix ToUnrealType(const sl::Matrix3f& SlMatrix)
		{
			FMatrix Matrix;

			// X plane
			Matrix.M[0][0] = SlMatrix.r00;
			Matrix.M[0][1] = SlMatrix.r10;
			Matrix.M[0][2] = SlMatrix.r20;

			// Y plane
			Matrix.M[1][0] = SlMatrix.r01;
			Matrix.M[1][1] = SlMatrix.r11;
			Matrix.M[1][2] = SlMatrix.r21;

			// Z plane
			Matrix.M[2][0] = SlMatrix.r02;
			Matrix.M[2][1] = SlMatrix.r12;
			Matrix.M[2][2] = SlMatrix.r22;

			return Matrix;
		}

		/*
		 * Convert from sl::Transform to FTransform
		 */
		FORCEINLINE FTransform ToUnrealType(const sl::Transform& SlTransform)
		{
			return FTransform(sl::unreal::ToUnrealType(static_cast<sl::Matrix4f>(SlTransform)));
		}

		/*
		 * Convert from sl::RecordingState to FSlRecordingState
		 */
		FORCEINLINE FSlRecordingState ToUnrealType(const sl::RecordingStatus& SlData)
		{
			FSlRecordingState RecordingState;

			RecordingState.CurrentCompressionTime = SlData.current_compression_time;
			RecordingState.CurrentCompressionRatio = SlData.current_compression_ratio;
			RecordingState.AverageCompressionTime = SlData.average_compression_time;
			RecordingState.AverageCompressionRatio = SlData.average_compression_ratio;
			RecordingState.Status = SlData.status;

			return RecordingState;
		}

		/*
		 * Convert from sl::DeviceProperties to FSlDeviceProperties
		 */
		FORCEINLINE FSlDeviceProperties ToUnrealType(const sl::DeviceProperties& SlData)
		{
			FSlDeviceProperties DeviceProperties;

			DeviceProperties.CameraModel = sl::unreal::ToUnrealType(SlData.camera_model);
			DeviceProperties.CameraState = sl::unreal::ToUnrealType(SlData.camera_state);
			DeviceProperties.ID = SlData.id;
			//DeviceProperties.Path = FString(SlData.path.c_str());
			DeviceProperties.SerialNumber = static_cast<int32>(SlData.serial_number);

			return DeviceProperties;
		}

		/*
		 * Convert from sl::CameraParameters to FSlCameraParameters
		 */
		FORCEINLINE FSlCameraParameters ToUnrealType(const sl::CameraParameters& SlData)
		{
			FSlCameraParameters CameraParameters;

			CameraParameters.HFOV = SlData.h_fov;
			CameraParameters.VFOV = SlData.v_fov;
			CameraParameters.HFocal = SlData.fx;
			CameraParameters.VFocal = SlData.fy;
			CameraParameters.OpticalCenterX = SlData.cx;
			CameraParameters.OpticalCenterY = SlData.cy;

			std::vector<double> Tmp;
			Tmp.assign(SlData.disto, SlData.disto + 5);
			CameraParameters.Disto = sl::unreal::arrays::ToUnrealType<float, double>(Tmp);

			CameraParameters.Resolution = sl::unreal::ToUnrealType(SlData.image_size);

			return CameraParameters;
		}

		/*
		 * Convert from sl::CalibrationParameters to FSlCalibrationParameters
		 */
		FORCEINLINE FSlCalibrationParameters ToUnrealType(const sl::CalibrationParameters& SlData)
		{
			FSlCalibrationParameters CalibrationParameters;

			CalibrationParameters.LeftCameraParameters = sl::unreal::ToUnrealType(SlData.left_cam);
			CalibrationParameters.RightCameraParameters = sl::unreal::ToUnrealType(SlData.right_cam);
			CalibrationParameters.Rotation = sl::unreal::ToUnrealType(SlData.R);
			CalibrationParameters.Translation = sl::unreal::ToUnrealType(SlData.T);

			return CalibrationParameters;
		}

		 
		/*
		 * Convert from sl::CameraInformation to FSlCameraInformation
		 */
		FORCEINLINE FSlCameraInformation ToUnrealType(const sl::CameraInformation& SlData)
		{
			FSlCameraInformation CameraInformation;

			CameraInformation.CalibrationParameters = sl::unreal::ToUnrealType(SlData.calibration_parameters);
			CameraInformation.CalibrationParametersRaw = sl::unreal::ToUnrealType(SlData.calibration_parameters_raw);
			CameraInformation.CameraModel = sl::unreal::ToUnrealType(SlData.camera_model);
			CameraInformation.CameraFirmwareVersion = static_cast<int32>(SlData.camera_firmware_version);
			CameraInformation.SensorsFirmwareVersion = static_cast<int32>(SlData.sensors_firmware_version);
			CameraInformation.SerialNumber = static_cast<int32>(SlData.serial_number);
			CameraInformation.HalfBaseline = SlData.calibration_parameters.T.x / 2.0f;

			return CameraInformation;
		}

		/*
		 * Convert from sl::Pose to FSlPose
		 */
		FORCEINLINE FSlPose ToUnrealType(const sl::Pose& SlData)
		{
			FSlPose Pose;

			Pose.Transform = sl::unreal::ToUnrealType(SlData.pose_data);
			Pose.Timestamp = SlData.timestamp.data_ns;
			Pose.Confidence = SlData.pose_confidence;
			Pose.bValid = SlData.valid;

			return Pose;
		}

		/*
		 * Convert from sl::IMUData to FSlIMUData
		 */
		FORCEINLINE FSlIMUData ToUnrealType(const sl::SensorsData& SlData)
		{
			FSlIMUData IMUData;

			IMUData.Transform = sl::unreal::ToUnrealType(SlData.imu.pose);
			IMUData.Timestamp = SlData.imu.timestamp;
			IMUData.bValid = SlData.imu.is_available;
			IMUData.OrientationCovariance = sl::unreal::ToUnrealType(SlData.imu.pose_covariance);
			IMUData.AngularVelocity = sl::unreal::ToUnrealType(SlData.imu.angular_velocity);
			IMUData.LinearAcceleration = sl::unreal::ToUnrealType(SlData.imu.linear_acceleration);
			IMUData.AngularVelocityConvariance = sl::unreal::ToUnrealType(SlData.imu.angular_velocity_covariance);
			IMUData.LinearAccelerationConvariance = sl::unreal::ToUnrealType(SlData.imu.linear_acceleration_covariance);

			return IMUData;
		}

		/*
		 * Convert from FMatrix to sl::Matrix4f (column to row)
		 */
		FORCEINLINE sl::Matrix4f ToSlType(const FMatrix& UnrealMatrix)
		{
			sl::Matrix4f Matrix;

			// X plane
			Matrix.r00 = UnrealMatrix.M[0][0];
			Matrix.r10 = UnrealMatrix.M[0][1];
			Matrix.r20 = UnrealMatrix.M[0][2];
			Matrix.m30 = UnrealMatrix.M[0][3];

			// Y plane
			Matrix.r01 = UnrealMatrix.M[1][0];
			Matrix.r11 = UnrealMatrix.M[1][1];
			Matrix.r21 = UnrealMatrix.M[1][2];
			Matrix.m31 = UnrealMatrix.M[1][3];

			// Z plane
			Matrix.r02 = UnrealMatrix.M[2][0];
			Matrix.r12 = UnrealMatrix.M[2][1];
			Matrix.r22 = UnrealMatrix.M[2][2];
			Matrix.m32 = UnrealMatrix.M[2][3];
			 
			// Origin
			Matrix.tx  = UnrealMatrix.M[3][0];
			Matrix.ty  = UnrealMatrix.M[3][1];
			Matrix.tz  = UnrealMatrix.M[3][2];
		    Matrix.m33 = UnrealMatrix.M[3][3];

			return Matrix;
		}

		/*
		 * Convert from FTransform to sl::Transform 
		 */
		FORCEINLINE sl::Transform ToSlType(const FTransform& UnrealTransform)
		{
			return static_cast<sl::Transform>(sl::unreal::ToSlType(UnrealTransform.ToMatrixWithScale()));
		}

		/*
		 * Convert from FIntPoint to sl::Resolution
		 */
		FORCEINLINE sl::Resolution ToSlType2(const FIntPoint& UnrealType)
		{
			return sl::Resolution(UnrealType.X, UnrealType.Y);
		}

		/*
		 * Convert from FIntPoint to sl::uchar2
		 */
		FORCEINLINE sl::uchar2 ToSlType(const FIntPoint& UnrealVector)
		{
			return sl::uchar2(FMath::Clamp(UnrealVector.X, 0, 255), FMath::Clamp(UnrealVector.Y, 0, 255));
		}

		/*
		 * Convert from FVector to sl::uchar3
		 */
		FORCEINLINE sl::uchar3 ToSlType(const FIntVector& UnrealVector)
		{
			return sl::uchar3(FMath::Clamp(UnrealVector.X, 0, 255), FMath::Clamp(UnrealVector.Y, 0, 255), FMath::Clamp(UnrealVector.Z, 0, 255));
		}

		/*
		 * Convert from FColor to sl::uchar4
		 */
		FORCEINLINE sl::uchar4 ToSlType(const FColor& UnrealColor)
		{
			return sl::uchar4(UnrealColor.R, UnrealColor.G, UnrealColor.B, UnrealColor.A);
		}

		/*
		 * Convert from FVector2D to sl::float2
		 */
		FORCEINLINE sl::float2 ToSlType(const FVector2D& UnrealVector)
		{
			return sl::float2(UnrealVector.X, UnrealVector.Y);
		}

		/*
		 * Convert from FVector to sl::float3
		 */
		FORCEINLINE sl::float3 ToSlType(const FVector& UnrealVector)
		{
			return sl::float3(UnrealVector.X, UnrealVector.Y, UnrealVector.Z);
		}

		/*
		 * Convert from FVector4 to sl::float4
		 */
		FORCEINLINE sl::float4 ToSlType(const FVector4& UnrealVector)
		{
			return sl::float4(UnrealVector.X, UnrealVector.Y, UnrealVector.Z, UnrealVector.W);
		}

		/*
		 * Convert from FSlTrackingParameters to sl::TrackingParameters
		 */
		FORCEINLINE sl::PositionalTrackingParameters ToSlType(const FSlPositionalTrackingParameters& UnrealData)
		{
			sl::PositionalTrackingParameters TrackingParameters;

			if (UnrealData.bLoadSpatialMemoryFile)
			{
				TrackingParameters.area_file_path = TCHAR_TO_UTF8(*UnrealData.SpatialMemoryFileLoadingPath);
			}
			TrackingParameters.enable_area_memory = UnrealData.bEnableAreaMemory;
			TrackingParameters.enable_pose_smoothing = UnrealData.bEnablePoseSmoothing;
			TrackingParameters.initial_world_transform = sl::unreal::ToSlType(FTransform(UnrealData.Rotation, UnrealData.Location));

			return TrackingParameters;
		}

		/*
		 * Covnert from FSlInitParameters to sl::InitParameters
		 */
		FORCEINLINE sl::InitParameters ToSlType(const FSlInitParameters& UnrealData)
		{
			sl::InitParameters InitParameters;

			InitParameters.camera_disable_self_calib = UnrealData.bDisableSelfCalibration;
			InitParameters.camera_fps = UnrealData.FPS;
			InitParameters.camera_image_flip = UnrealData.bVerticalFlipImage;
			InitParameters.camera_resolution = sl::unreal::ToSlType(UnrealData.Resolution);
			InitParameters.coordinate_system = sl::unreal::ToSlType(UnrealData.CoordinateSystem);
			InitParameters.coordinate_units = sl::unreal::ToSlType(UnrealData.Unit);
			InitParameters.depth_minimum_distance = UnrealData.DepthMinimumDistance;
			InitParameters.depth_mode = sl::unreal::ToSlType(UnrealData.DepthMode);
			InitParameters.enable_right_side_measure = UnrealData.bEnableRightSideMeasure;
			InitParameters.sdk_gpu_id = FMath::FloorToInt(UnrealData.GPUID);
			InitParameters.sdk_verbose = UnrealData.bVerbose;
			InitParameters.sdk_verbose_log_file = TCHAR_TO_UTF8(*UnrealData.VerboseFilePath);
			if (UnrealData.bUseSVO)
			{
				InitParameters.input.setFromSVOFile(TCHAR_TO_UTF8(*UnrealData.SVOFilePath));
				InitParameters.svo_real_time_mode = UnrealData.bRealTime;
			}

			InitParameters.depth_stabilization = UnrealData.bEnableDepthStabilization;

			return InitParameters;
		}

		/*
		 * Convert from FSlRuntimeParameters to sl::RuntimeParameters
		 */
		FORCEINLINE sl::RuntimeParameters ToSlType(const FSlRuntimeParameters& UnrealData)
		{
			sl::RuntimeParameters RuntimeParameters;
			
			RuntimeParameters.enable_depth = UnrealData.bEnableDepth;
			RuntimeParameters.confidence_threshold = UnrealData.ConfidenceThreshold;
			RuntimeParameters.measure3D_reference_frame = sl::unreal::ToSlType(UnrealData.ReferenceFrame);
			RuntimeParameters.sensing_mode = sl::unreal::ToSlType(UnrealData.SensingMode);
			
			return RuntimeParameters;
		}

		/*
		 * Convert from FSlSpatialMappingParameters to sl::SpatialMappingParameters
		 */
		FORCEINLINE sl::SpatialMappingParameters ToSlType(const FSlSpatialMappingParameters& UnrealData)
		{
			sl::SpatialMappingParameters SpatialMappingParameters;

			SpatialMappingParameters.max_memory_usage = UnrealData.MaxMemoryUsage;
			SpatialMappingParameters.save_texture = UnrealData.bSaveTexture;
			SpatialMappingParameters.set(sl::unreal::ToSlType(UnrealData.PresetRange));
			SpatialMappingParameters.set(sl::unreal::ToSlType(UnrealData.PresetResolution));
            SpatialMappingParameters.use_chunk_only = UnrealData.GetUseChunksOnly();
			SpatialMappingParameters.reverse_vertex_order = true;

			return SpatialMappingParameters;
		}

		/*
		 * Convert from FSlMeshFilterParameters to sl::MeshFilterParameters
		 */
		FORCEINLINE sl::MeshFilterParameters ToSlType(const FSlMeshFilterParameters& UnrealData)
		{
			return sl::MeshFilterParameters(sl::unreal::ToSlType(UnrealData.FilterIntensity));
		}

		/*
		* Convert from sl::CameraParameters to FSlCameraParameters
		*/
		FORCEINLINE sl::CameraParameters ToSlType(const FSlCameraParameters& UnrealData)
		{
			sl::CameraParameters CameraParameters;

			CameraParameters.h_fov = UnrealData.HFOV;
			CameraParameters.v_fov = UnrealData.VFOV;
			CameraParameters.fx = UnrealData.HFocal;
			CameraParameters.fy = UnrealData.VFocal;
			CameraParameters.cx = UnrealData.OpticalCenterX;
			CameraParameters.cy = UnrealData.OpticalCenterY;

			CameraParameters.disto[0] = UnrealData.Disto[0];
			CameraParameters.disto[1] = UnrealData.Disto[1];
			CameraParameters.disto[2] = UnrealData.Disto[2];
			CameraParameters.disto[3] = UnrealData.Disto[3];
			CameraParameters.disto[4] = UnrealData.Disto[4];

			CameraParameters.image_size = sl::unreal::ToSlType2(UnrealData.Resolution);

			return CameraParameters;
		}

		/*
		* Convert from sl::CalibrationParameters to FSlCalibrationParameters
		*/
		FORCEINLINE sl::CalibrationParameters ToSlType(const FSlCalibrationParameters& UnrealData)
		{
			sl::CalibrationParameters CalibrationParameters;

			CalibrationParameters.left_cam = sl::unreal::ToSlType(UnrealData.LeftCameraParameters);
			CalibrationParameters.left_cam = sl::unreal::ToSlType(UnrealData.RightCameraParameters);
			CalibrationParameters.R = sl::unreal::ToSlType(UnrealData.Rotation);
			CalibrationParameters.T = sl::unreal::ToSlType(UnrealData.Translation);

			return CalibrationParameters;
		}


		/*
		* Convert from sl::CameraInformation to FSlCameraInformation
		*/
		FORCEINLINE sl::CameraInformation ToSlType(const FSlCameraInformation& UnrealData)
		{
			sl::CameraInformation CameraInformation;

			CameraInformation.calibration_parameters = sl::unreal::ToSlType(UnrealData.CalibrationParameters);
			CameraInformation.calibration_parameters_raw = sl::unreal::ToSlType(UnrealData.CalibrationParametersRaw);
			CameraInformation.camera_model = sl::unreal::ToSlType(UnrealData.CameraModel);
			CameraInformation.camera_firmware_version = UnrealData.CameraFirmwareVersion;
			CameraInformation.sensors_firmware_version = UnrealData.SensorsFirmwareVersion;
			CameraInformation.serial_number = UnrealData.SerialNumber;

			return CameraInformation;
		}
	}
}