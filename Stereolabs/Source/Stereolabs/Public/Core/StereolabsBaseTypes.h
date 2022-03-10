//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

#pragma once

#include "AllowWindowsPlatformTypes.h"
#include <sl/Camera.hpp>
#include "HideWindowsPlatformTypes.h"

#include "StereolabsBaseTypes.generated.h"

/************************************************************************/
/*								 Enums									*/
/************************************************************************/

/*
 * List of possible camera state
 * see sl::CAMERA_STATE
 */
enum class ESlCameraState : uint8
{
	CS_Available			UMETA(DisplayName = "Available"),
	CS_NotAvailable			UMETA(DisplayName = "Not available")
};

/*
 * Formats supported by USlTexture
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlTextureFormat : uint8
{
	TF_R32_FLOAT			UMETA(DisplayName = "R32 Float (depth/disparity)"),
	TF_B8G8R8A8_UNORM		UMETA(DisplayName = "BGRA8 UNORM (Color)"),
	TF_R8G8B8A8_UINT		UMETA(DisplayName = "RGBA8 UINT (Point Cloud)"),
	TF_A32B32G32R32F		UMETA(DisplayName = "ABGR32 Float (normals)"),
	TF_R8_UNORM				UMETA(DisplayName = "R8 UNORM (Color grayscale)"),
	TF_Unkown				UMETA(Hidden, DisplayName = "Unknown")
};

/*
 * Texture quality presets for depth and normals
 * Low    = 192p
 * Medium = 480p
 * High   = 720p
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlTextureQuality : uint8
{
	TQ_Low					UMETA(DisplayName = "Low"),
	TQ_Medium				UMETA(DisplayName = "Medium"),
	TQ_High					UMETA(DisplayName = "High")
};

/*
 * Resolution presets for spatial mapping
 * See sl::SpatialMappingParameters::RESOLUTION
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlSpatialMappingResolution : uint8
{
	SMR_Low					UMETA(DisplayName = "Low resolution"),
	SMR_Medium				UMETA(DisplayName = "Medium resolution"),
	SMR_High				UMETA(DisplayName = "High resolution")
};

/*
 * Range presets for spatial mapping
 * See sl::SpatialMappingParameters::RANGE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlSpatialMappingRange : uint8
{
	SMR_Short				UMETA(DisplayName = "Short"),
	SMR_Medium				UMETA(DisplayName = "Medium"),
	SMR_Long				UMETA(DisplayName = "Long"),
	SMR_Auto				UMETA(DisplayName = "Auto")
};

/*
 * Mesh filtering presets for spatial mapping
 * See sl::MeshFilterParameters::FILTER
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlFilterIntensity : uint8
{
	FI_Low					UMETA(DisplayName = "Low"),
	FI_Medium				UMETA(DisplayName = "Medium"),
	FI_High					UMETA(DisplayName = "High")
};

/*
 * IDs of each "eye"
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlEye : uint8
{
	E_Left					UMETA(DisplayName = "Left"),
	E_Right					UMETA(DisplayName = "Right")
};

/*
 * Threading mode selected to perform grab
 * MultiThreaded  = Grab called asynchronously at max rate
 * SingleThreaded = Grab called in game thread at game FPS rate
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlThreadingMode : uint8
{
	TM_MultiThreaded		 UMETA(DisplayName = "MultiThreaded"),
	TM_SingleThreaded		 UMETA(DisplayName = "SingleThreaded"),
	TM_None					 UMETA(Hidden, DisplayName = "Unselected")
};

/*
 * SDK Memory types
 * see sl::MEM
 */
UENUM(BlueprintType, Meta = (Bitflags), Category = "Stereolabs|Enum")
enum class ESlMemoryType : uint8
{
	MT_CPU = 1 << 0			UMETA(DisplayName = "CPU"),
	MT_GPU = 1 << 1			UMETA(DisplayName = "GPU"),
};
ENUM_CLASS_FLAGS(ESlMemoryType)

/*
 * SDK Video resolutions
 * see sl::RESOLUTION
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlResolution : uint8
{
	R_HD2K			   		 UMETA(DisplayName = "HD 2K"),
	R_HD1080		   		 UMETA(DisplayName = "HD 1080p"),
	R_HD720		   			 UMETA(DisplayName = "HD 720p"),
	R_VGA			   		 UMETA(DisplayName = "VGA")
};

/*
 * SDK Depth modes
 * see sl::DEPTH_MODE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlDepthMode : uint8
{
	DM_None				     UMETA(DisplayName = "None"),
	DM_Performance			 UMETA(DisplayName = "Performance"),
	DM_Quality				 UMETA(DisplayName = "Quality"),
	DM_Ultra				 UMETA(DisplayName = "Ultra"),
	DM_Neural				 UMETA(DisplayName = "Neural")
};

/*
 * SDK Sensing modes
 * see sl::SENSING_MODE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlSensingMode : uint8
{
	SM_Fill					 UMETA(DisplayName = "Fill"),
	SM_Standard				 UMETA(DisplayName = "Standard")
};

/*
 * Units used for measures.
 * For performance concern, only centimeter natively used by UE is available
 * see sl::UNIT
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlUnit : uint8
{
	DU_Centimeter            UMETA(DisplayName = "Centimeter")
};

/*
 * Coordinate systems used for measures.
 * For performance concern, only Cartesian natively used by UE is available
 * See sl::COORDINATE_SYSTEM
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlCoordinateSystem : uint8
{
	CS_Cartesian   			 UMETA(DisplayName = "Cartesian left-handed")
};

/*
 * Input type
 * See sl::INPUT_TYPE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlInputType : uint8
{
	IT_USB   			 UMETA(DisplayName = "USB input mode"),
	IT_SVO   			 UMETA(DisplayName = "SVO input mode"),
	IT_STREAM   		 UMETA(DisplayName = "Stream input mode"),
};

/*
 * Camera flip mode
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlFlipMode : uint8
{
	FP_OFF   			 UMETA(DisplayName = "Default behaviour"),
	FP_ON   			 UMETA(DisplayName = "Images and camera sensors data are flipped"),
	FP_AUTO   			 UMETA(DisplayName = "Detect automatically")
};

/*
 * SDK Tracking states 
 * See sl::TRACKING_STATE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlTrackingState : uint8
{
	TS_TrackingSearch	     UMETA(DisplayName = "Searching a match"),
	TS_TrackingOk            UMETA(DisplayName = "Operates normally"),
	TS_FpsTooLow			 UMETA(DisplayName = "FPS too low"),
	TS_TrackingOff           UMETA(DisplayName = "Disabled")
};

/*
 * Rendering modes available in the plugin
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlRenderingMode : uint8
{
	RM_Mono				     UMETA(DisplayName = "Mono"),
	RM_Stereo			     UMETA(DisplayName = "Stereo"),
	RM_None					 UMETA(Hidden, DisplayName = "Unselected")
};

/*
 * SDK View types
 * See sl::VIEW
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlView : uint8
{
	V_Left					 UMETA(DisplayName = "Left"),
	V_Right					 UMETA(DisplayName = "Right"),
	V_LeftUnrectified		 UMETA(DisplayName = "Left unrectified"),
	V_RightUnrectified		 UMETA(DisplayName = "Right unrectified"),
	V_LeftUnrectifiedGray	 UMETA(DisplayName = "Left unrectified gray"),
	V_RightUnrectifiedGray	 UMETA(DisplayName = "Right unrectified gray"),
	V_LeftGray				 UMETA(DisplayName = "Left gray"),
	V_RightGray				 UMETA(DisplayName = "Right gray"),
	V_SideBySide			 UMETA(DisplayName = "Side by side"),
	V_Depth					 UMETA(DisplayName = "Depth"),
	V_DepthRight			 UMETA(DisplayName = "Depth right"),
	V_Confidence			 UMETA(DisplayName = "Confidence"),
	V_Normals				 UMETA(DisplayName = "Normals"),
	V_NormalsRight		     UMETA(DisplayName = "Normals right")
};

/*
 * SDK Measure types
 * See sl::MEASURE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlMeasure : uint8
{
	M_Depth					 UMETA(DisplayName = "Depth"),
	M_DepthRight			 UMETA(DisplayName = "Depth right"),
	M_Disparity				 UMETA(DisplayName = "Disparity"),
	M_DisparityRight		 UMETA(DisplayName = "Disparity right"),
	M_Normals				 UMETA(DisplayName = "Normals"),
	M_NormalsRight			 UMETA(DisplayName = "Normals right"),
	M_Confidence			 UMETA(DisplayName = "Confidence"),
	M_XYZ					 UMETA(DisplayName = "Point Cloud"),
	M_XYZ_Right				 UMETA(DisplayName = "Point Cloud right"),
	M_XYZ_RGBA				 UMETA(DisplayName = "Point Cloud RGBA"),
	M_XYZ_RGBA_Right		 UMETA(DisplayName = "Point Cloud RGBA right"),
	M_XYZ_BGRA				 UMETA(DisplayName = "Point Cloud BGRA"),
	M_XYZ_BGRA_Right		 UMETA(DisplayName = "Point Cloud BGRA right"),
	M_XYZ_ARGB				 UMETA(DisplayName = "Point Cloud ARGB"),
	M_XYZ_ARGB_Right		 UMETA(DisplayName = "Point Cloud ARGB right"),
	M_XYZ_ABGR				 UMETA(DisplayName = "Point Cloud ABGR"),
	M_XYZ_ABGR_Right		 UMETA(DisplayName = "Point Cloud ABGR right"),
	M_DEPTH_U16_MM		     UMETA(DisplayName = "Depth in millimeter"),
	M_DEPTH_U16_MM_RIGHT     UMETA(DisplayName = "Depth right in millimeter")
};

/*
 * Plugin texture types
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlTextureType : uint8
{
	TT_View					 UMETA(DisplayName = "View"),
	TT_Measure				 UMETA(DisplayName = "Measure")
};

/*
 * Retrieve valid or not
 * RR_DepthNotValid does not exist because depth values are clamped between min and max
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlRetrieveResult : uint8
{
	/** The location is valid and the retrieve returned a valid value */
	RR_RetrieveValid        UMETA(DisplayName = "Retrieve valid"), 
	/** The location is not valid and no retrieve */
	RR_LocationNotValid		UMETA(DisplayName = "Retrieve location not valid"), 
	/** The location is valid and the normal retrieved is not valid */
	RR_NormalNotValid		UMETA(DisplayName = "Normal not valid") 
};

/* 
 * SDK Error codes
 * see sl::ERROR_CODE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlErrorCode : uint8
{
 	EC_Success					   UMETA(DisplayName = "Success"),
	EC_Failure					   UMETA(DisplayName = "Failure"),
	EC_NoGpuCompatible			   UMETA(DisplayName = "No GPU compatible"),
	EC_NotEnoughGPUMemory		   UMETA(DisplayName = "Not enough GPU memory"),
	EC_CameraNotDetected		   UMETA(DisplayName = "Camera not detected"),
	EC_SensorNotAvailable		   UMETA(DisplayName = "Sensors not available"),
	EC_InvalidResolution		   UMETA(DisplayName = "Invalid resolution"),
	EC_LowUSBBandwidth			   UMETA(DisplayName = "Low USB bandwidth"),
	EC_CalibrationFileNotAvailable UMETA(DisplayName = "Calibration file not available"),
	EC_InvalidCalibrationFile	   UMETA(DisplayName = "Invalid calibration file"),
	EC_InvalidSVOFile			   UMETA(DisplayName = "Invalid SVO file"),
	EC_SVORecordingError		   UMETA(DisplayName = "SVO recording error"),
	EC_SVOUnsupportedCompression   UMETA(DisplayName = "SVO unsupported compression"),
	EC_SVOEndOfSVOFile             UMETA(DisplayName = "End of SVO file reached"),
	EC_InvalidCoordinateSystem	   UMETA(DisplayName = "Invalid coordinate system"),
	EC_InvalidFirmware			   UMETA(DisplayName = "Invalid firmware"),
	EC_InvalidFunctionParameters   UMETA(DisplayName = "Invalid function parameters"),
	EC_CUDAError				   UMETA(DisplayName = "CUDA error"),
	EC_CameraNotInitialized		   UMETA(DisplayName = "Camera not initialized"),
	EC_NVIDIADriverOutOfDate	   UMETA(DisplayName = "NVIDIA driver out of date"),
	EC_InvalidFunctionCall		   UMETA(DisplayName = "Invalid function call"),
	EC_CorruptedSDKInstallation	   UMETA(DisplayName = "Corrupted SDK installation"),
	EC_IncompatibleSDKVersion      UMETA(DisplayName = "Incompatible SDK version"),
	EC_InvalidAreaFile			   UMETA(DisplayName = "Invalid area file"),
	EC_IncompatibleAreaFile		   UMETA(DisplayName = "Incompatible area file"),
	EC_CameraFailedToSetup		   UMETA(DisplayName = "Camera failed to setup"),
	EC_CameraDetectionIssue		   UMETA(DisplayName = "Camera detection issue"),
	EC_CameraFailedToStart		   UMETA(DisplayName = "Camera failed to start stream"),
	EC_NoGpuDetected			   UMETA(DisplayName = "No GPU detected"),
	EC_NoplaneFound				   UMETA(DisplayName = "No plane(s) found"),
	EC_ModuleNotCompatible         UMETA(DisplayName = "Module not compatible with camera"),
	EC_MotionSensorsRequired       UMETA(DisplayName = "Motion sensors required"),
	// ERROR_CODE_LAST
	EC_None					  	   UMETA(DisplayName = "No error") 
};

/* 
 * SDK SVO compression modes
 * see sl::SVO_COMPRESSION_MODE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlSVOCompressionMode : uint8
{
	SCM_Lossless			UMETA(DisplayName = "Lossless"),
	SCM_H264				UMETA(DisplayName = "H264"),
	SCM_H265				UMETA(DisplayName = "H265"),
    SCM_H264_Lossless	    UMETA(DisplayName = "H264_Lossless"),
	SCM_H265_Lossless	    UMETA(DisplayName = "H265_Lossless")
};

/* 
 * SDK spatial memory export state
 * see sl::AREA_EXPORT_STATE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlSpatialMemoryExportingState : uint8
{
	SMES_Success			   UMETA(DisplayName = "Success"),
	SMES_Running			   UMETA(DisplayName = "Running"),
	SMES_NotStarted			   UMETA(DisplayName = "NotStarted"),
	SMES_FileEmpty			   UMETA(DisplayName = "FileEmpty"),
	SMES_FileError			   UMETA(DisplayName = "FileError"),
	SMES_SpatialMemoryDisabled UMETA(DisplayName = "SpatialMemoryDisabled")
};

/* 
 * SDK mesh file format
 * see sl::MESH_FILE_FORMAT
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlMeshFileFormat : uint8
{
	MFF_OBJ					UMETA(DisplayName = "Obj (.obj)"),
	MFF_PLY					UMETA(DisplayName = "Ply (.ply)"),
	MFF_PLY_BIN				UMETA(DisplayName = "Ply binary (.ply)")
};

/* 
 * SDK mesh texture format
 * see sl::MESH_TEXTURE_FORMAT
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlMeshTextureFormat : uint8
{
	MTF_RGB					UMETA(DisplayName = "RGB"),
	MTF_RGBA				UMETA(DisplayName = "RGBA")
};

/* 
 * SDK camera type
 * see sl::MODEL
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlModel : uint8
{
	M_Zed					UMETA(DisplayName = "ZED"),
	M_ZedM					UMETA(DisplayName = "ZED Mini"),
	M_Zed2					UMETA(DisplayName = "ZED 2"),
	M_Zed2i          		UMETA(DisplayName = "ZED 2i"),
	M_Unknown				UMETA(DisplayName = "Unknown")
};

/*
 * SDK reference frame
 * see sl::REFERENCE_FRAME
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlReferenceFrame : uint8
{
	/** The transform of FSlPose will contains the motion with reference to the world frame */
	RF_World				UMETA(DisplayName = "World"),
	/** The transform of FSlPose will contains the motion with reference to the previous camera frame */
	RF_Camera				UMETA(DisplayName = "Camera")
};

/*
 * SDK time reference
 * see sl::TIME_REFERENCE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlTimeReference : uint8
{
	TR_Image				UMETA(DisplayName = "Image"),
	TR_Current				UMETA(DisplayName = "Current")
};

/*
 * SDK mat type
 * see sl::MAT_TYPE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlMatType : uint8
{
	MT_32F_C1				UMETA(DisplayName = "Float 1 channel"),
	MT_32F_C2				UMETA(DisplayName = "Float 2 channels"),
	MT_32F_C3				UMETA(DisplayName = "Float 3 channels"),
	MT_32F_C4				UMETA(DisplayName = "Float 4 channels"),
	MT_8U_C1				UMETA(DisplayName = "Uint 1 channel"),
	MT_8U_C2				UMETA(DisplayName = "Uint 2 channels"),
	MT_8U_C3				UMETA(DisplayName = "Uint 3 channels"),
	MT_8U_C4				UMETA(DisplayName = "Uint 4 channels")
};

/*
 * SDK mat copy type
 * see sl::COPY_TYPE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlCopyType : uint8
{
	CT_CPUToCPU				UMETA(DisplayName = "CPU to CPU"),
	CT_CPUToGPU				UMETA(DisplayName = "CPU to GPU"),
	CT_GPUToGPU				UMETA(DisplayName = "GPU to GPU"),
	CT_GPUToCPU				UMETA(DisplayName = "GPU to CPU")
};

/*
 * SDK self calibration state
 * see sl::SELF_CALIBRATION_STATE
 */
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ESlSelfCalibrationState : uint8
{
	SCS_NotStarted			UMETA(DisplayName = "Not started"),
	SCS_Running				UMETA(DisplayName = "Running"),
	SCS_Failed				UMETA(DisplayName = "Failed"),
	SCS_Success				UMETA(DisplayName = "Success"),
};

/*
* Tracking type selection
* Allow to chose which tracking is selected for the pawn.
* ZED : Zed tracking only
* HMD : Hmd tracking only
* Mixte : Zed Imu rotations and Hmd translations
*/
UENUM(BlueprintType, Category = "Stereolabs|Enum")
enum class ETrackingType : uint8
{
	TrT_ZED			UMETA(DisplayName = "Zed"),
	TrT_HMD			UMETA(DisplayName = "Hmd"),
	TrT_Mixte		UMETA(DisplayName = "Mixte"),
};

/************************************************************************/
/*							    Structs				    				*/
/************************************************************************/

/*
 * SDK device properties
 * see sl::DeviceProperties
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlDeviceProperties
{
	GENERATED_BODY()

	FSlDeviceProperties()
		:
		CameraState(ESlCameraState::CS_NotAvailable),
		ID(0),
		Path(""),
		CameraModel(ESlModel::M_Unknown),
		SerialNumber(0)
	{
	}

	/** The camera system path */
	FString Path;

	/** The camera ID (Notice that only the camera with ID '0' can be used on Windows) */
	int32 ID;

	/** he camera serial number */
	int32 SerialNumber;

	/** The camera state */
	ESlCameraState CameraState;

	/** The camera model */
	ESlModel CameraModel;
};

/*
 * SDK camera parameters
 * see sl::CameraParameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlCameraParameters
{
	GENERATED_BODY()

	FSlCameraParameters()
		:
		Resolution(FIntPoint::ZeroValue),
		HFOV(0.0f),
		VFOV(0.0f),
		HFocal(0.0f),
		VFocal(0.0f),
		OpticalCenterX(0.0f),
		OpticalCenterY(0.0f)
	{
	}

	/** Distortion factor : [ k1, k2, p1, p2, k3 ]. Radial (k1,k2,k3) and Tangential (p1,p2) distortion. */
	UPROPERTY(BlueprintReadOnly)
	TArray<float> Disto;

	/** Resolution of images */
	UPROPERTY(BlueprintReadOnly)
	FIntPoint Resolution;

	/** Horizontal field of view */
	UPROPERTY(BlueprintReadOnly)
	float HFOV;

	/** Vertical field of view */
	UPROPERTY(BlueprintReadOnly)
	float VFOV;

	/** Horizontal focal */
	UPROPERTY(BlueprintReadOnly)
	float HFocal;

	/** Vertical focal */
	UPROPERTY(BlueprintReadOnly)
	float VFocal;

	/** Horizontal position of the optical center in pixels */
	UPROPERTY(BlueprintReadOnly)
	float OpticalCenterX;

	/** Vertical position of the optical center in pixels */
	UPROPERTY(BlueprintReadOnly)
	float OpticalCenterY;
};

/*
 * SDK calibration parameters
 * see Sl::CalibrationParameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlCalibrationParameters
{
	GENERATED_BODY()

	FSlCalibrationParameters()
		:
		Rotation(FVector::ZeroVector),
		Translation(FVector::ZeroVector)
	{
	}

    /** Intrinsic parameters of the left camera */
	UPROPERTY(BlueprintReadOnly)
	FSlCameraParameters LeftCameraParameters;

	/** Intrinsic parameters of the left camera */
	UPROPERTY(BlueprintReadOnly)
	FSlCameraParameters RightCameraParameters;

	/** Rotation (using Rodrigues' transformation) between the two sensors. Defined as 'tilt', 'convergence' and 'roll' */
	UPROPERTY(BlueprintReadOnly)
	FVector Rotation;

	/** Translation between the two sensors. T.x is the distance between the two cameras (baseline) in the sl::UNIT */
	UPROPERTY(BlueprintReadOnly)
	FVector Translation;
};

/*
 * SDK camera information
 * see Sl::CameraInformation
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlCameraInformation
{
	GENERATED_BODY()

	FSlCameraInformation()
		:
		HalfBaseline(0.0f),
		SerialNumber(0),
		CameraFirmwareVersion(0),
		SensorsFirmwareVersion(0),
		CameraModel(ESlModel::M_Unknown)
	{
	}

	/** Intrinsic and Extrinsic stereo parameters for rectified images (default) */
	UPROPERTY(BlueprintReadOnly)
	FSlCalibrationParameters CalibrationParameters; 

	/** Intrinsic and Extrinsic stereo parameters for original images (unrectified) */
	UPROPERTY(BlueprintReadOnly)
	FSlCalibrationParameters CalibrationParametersRaw;
	
	/** Half baseline in selected unit */
	UPROPERTY(BlueprintReadOnly)
	float HalfBaseline;

	/** camera dependent serial number */
	UPROPERTY(BlueprintReadOnly)
	int32 SerialNumber;

	/** current firmware version of the camera */
	UPROPERTY(BlueprintReadOnly)
	int32 CameraFirmwareVersion;

	/** current firmware version of the camera */
	UPROPERTY(BlueprintReadOnly)
	int32 SensorsFirmwareVersion;

	/** camera model (ZED or ZED-M) */
	UPROPERTY(BlueprintReadOnly)
	ESlModel CameraModel; 
};

/*
 * SDK mat
 * Only holding the mat
 * see sl::Mat
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlMat
{
	GENERATED_BODY()

	FSlMat()
	{
		Mat.name = TCHAR_TO_UTF8(*FName("Unnamed").ToString());
	}

	FSlMat(const sl::Mat& NewMat)
		:
		Mat(NewMat)
	{
		if (FName(Mat.name.c_str()).IsEqual(""))
		{
			Mat.name = TCHAR_TO_UTF8(*FName("Unnamed").ToString());
		}
	}

	FSlMat& operator=(const sl::Mat& NewMat)
	{
		Mat = NewMat;
		return *this;
	}

	/** The underlying sl::Mat */
	sl::Mat Mat;
};

/*
 * Spatial mapping data of a mesh
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlMeshData
{
	GENERATED_BODY()

	FSlMeshData()
		:
		Texture(nullptr)
	{
	}

	/*
	 * Clear all the data
	 */
	void Clear()
	{
		Vertices.Empty(0);
		Indices.Empty(0);
		Normals.Empty(0);
		UV0.Empty(0);

		if (Texture && Texture->IsValidLowLevel())
		{
			Texture->ConditionalBeginDestroy();
			Texture = nullptr;
		}
	}

	/** Vertices of the mesh */
	UPROPERTY(BlueprintReadOnly)
	TArray<FVector> Vertices;

	/** Indices of the mesh */
	UPROPERTY(BlueprintReadOnly)
	TArray<int32> Indices;

	/** Normals of the mesh */
	UPROPERTY(BlueprintReadOnly)
	TArray<FVector> Normals;

	/** UV0 of the mesh */
	UPROPERTY(BlueprintReadOnly)
	TArray<FVector2D> UV0;

	/** Texture of the mesh */
	UPROPERTY(BlueprintReadOnly)
	UTexture2D* Texture;
};

/*
 * SDK mesh filter parameters
 * see sl::MeshFilterParameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlMeshFilterParameters
{
	GENERATED_BODY()

	FSlMeshFilterParameters(ESlFilterIntensity FilterIntensity)
		:
		FilterIntensity(FilterIntensity)
	{
	}

	FSlMeshFilterParameters()
		:
		FilterIntensity(ESlFilterIntensity::FI_Low)
	{
	}

	/** Intensity of the mesh filtering */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlFilterIntensity FilterIntensity;
};


/*
 * SDK spatial mapping parameters
 * see sl::SpatialMappingParameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Types")
struct STEREOLABS_API FSlSpatialMappingParameters
{
	GENERATED_BODY()

	FSlSpatialMappingParameters();

	/*
	 * Set the resolution
	 * @param NewPresetResolution The new preset
	 */
	void SetResolution(ESlSpatialMappingResolution NewPresetResolution);

	/*
	 * Set the resolution
	 * @param NewResolution The new resolution
	 */
	FORCEINLINE void SetResolution(float NewResolution)
	{
		sl::SpatialMappingParameters Dummy;

		Resolution = FMath::Clamp(NewResolution, Dummy.allowed_resolution.first, Dummy.allowed_resolution.second);
	}

	/*
	 * Set the range
	 * @param NewPresetRange The new preset
	 */
	void SetMaxRange(ESlSpatialMappingRange NewPresetRange);

	/*
	 * Set the max range
	 * @param NewRange The new range
	 */
	FORCEINLINE void SetMaxRange(float NewRange)
	{
		sl::SpatialMappingParameters Dummy;

		MaxRange = FMath::Clamp(NewRange, Dummy.allowed_range.first, Dummy.allowed_range.second);
	}

	/** Depth integration max range. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ClampMin = "2.0", ClampMax = "20.0"))
	float MaxRange;

	/** Spatial mapping resolution. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ClampMin = "0.01", ClampMax = "0.2"))
	float Resolution;

	/** The maximum CPU memory (in mega bytes) allocated for the meshing process (will fit your configuration in any case). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int32 MaxMemoryUsage;

	/** Resolution preset */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlSpatialMappingResolution PresetResolution;

	/** Range preset */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlSpatialMappingRange PresetRange;

	/*
	 * Set to true if you want be able to apply texture to your mesh after its creation.
	 * This option will take more memory.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bSaveTexture;

	/** remove this function when chunks supported */
	bool GetUseChunksOnly() const
	{
		return bUseChunkOnly;
	}

private:
	/*  Set to false if you want keep consistency between the mesh and its inner chunks data.
	 *	Updating the Mesh is time consuming, consider using only Chunks data for better performance.
	 *  Chunks are not supported, forced to false.
	 */
	//UPROPERTY(EditAnywhere, BlueprintReadOnly)
	bool bUseChunkOnly = false;
};

/*
 * SDK recording state
 * see sl::RecordingState
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Types")
struct STEREOLABS_API FSlRecordingState
{
	GENERATED_BODY()

	FSlRecordingState()
		:
		CurrentCompressionTime(0.0f),
		CurrentCompressionRatio(0.0f),
		AverageCompressionTime(0.0f),
		AverageCompressionRatio(0.0f),
		Status(false)
	{
	}

	/** Compression time for the current frame in ms. */
	UPROPERTY(BlueprintReadOnly)
	float CurrentCompressionTime;

	/** Compression ratio (% of raw size) for the current frame. */
	UPROPERTY(BlueprintReadOnly)
	float CurrentCompressionRatio;

	/** Average compression time in ms since beginning of recording. */
	UPROPERTY(BlueprintReadOnly)
	float AverageCompressionTime; 

	/** Compression ratio (% of raw size) since beginning of recording. */
	UPROPERTY(BlueprintReadOnly)
	float AverageCompressionRatio;

	/** Status of current frame. May be true for success or false if frame could not be written in the SVO file. */
	UPROPERTY(BlueprintReadOnly)
	uint8 Status:1;
};

/*
 * SDK camera settings
 * see sl::CameraSettings
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlVideoSettings
{
	GENERATED_BODY()

	const TCHAR* Section = TEXT("Camera");

	FSlVideoSettings()
		:
		Brightness(4),
		Contrast(4),
		Hue(0),
		Saturation(4),
		Sharpness(3),
		WhiteBalance(4700),
		Gain(56),
		Exposure(100),
		bAutoWhiteBalance(true),
		bAutoGainAndExposure(true),
		bDefault(false)
	{
	}

	FORCEINLINE void Load(const FString& Path)
	{
		GConfig->GetInt(
			Section,
			TEXT("Brightness"),
			Brightness,
			*Path
			);

		GConfig->GetInt(
			Section,
			TEXT("Contrast"),
			Contrast,
			*Path
			);

		GConfig->GetInt(
			Section,
			TEXT("Hue"),
			Hue,
			*Path
			);

		GConfig->GetInt(
			Section,
			TEXT("Saturation"),
			Saturation,
			*Path
			);

		GConfig->GetInt(
			Section,
			TEXT("Sharpness"),
			Sharpness,
			*Path
		);

		GConfig->GetInt(
			Section,
			TEXT("WhiteBalance"),
			WhiteBalance,
			*Path
			);

		GConfig->GetInt(
			Section,
			TEXT("Gain"),
			Gain,
			*Path
			);

		GConfig->GetInt(
			Section,
			TEXT("Exposure"),
			Exposure,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bAutoWhiteBalance"),
			bAutoWhiteBalance,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bAutoGainAndExposure"),
			bAutoGainAndExposure,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bDefault"),
			bDefault,
			*Path
			);
	}

	FORCEINLINE void Save(const FString& Path) const
	{

		GConfig->SetInt(
			Section,
			TEXT("Brightness"),
			Brightness,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("Contrast"),
			Contrast,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("Hue"),
			Hue,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("Saturation"),
			Saturation,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("Sharpness"),
			Sharpness,
			*Path
		);

		GConfig->SetInt(
			Section,
			TEXT("WhiteBalance"),
			WhiteBalance,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("Gain"),
			Gain,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("Exposure"),
			Exposure,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bAutoWhiteBalance"),
			bAutoWhiteBalance,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bAutoGainAndExposure"),
			bAutoGainAndExposure,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bDefault"),
			bDefault,
			*Path
			);
	}

	/** Brightness, default = 4 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "8"))
	int32 Brightness;

	/** Contrast, default = 4 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "8"))
	int32 Contrast;

	/** Hue, default = 0 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "11"))
	int32 Hue;

	/** Saturation, default = 4 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "8"))
	int32 Saturation;

	/** Saturation, default = 3 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "8"))
	int32 Sharpness;
	
	/** WhiteBalance */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "2800", ClampMax = "6500"))
	int32 WhiteBalance;

	/** Gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "100"))
	int32 Gain;

	/** Exposure */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0", ClampMax = "100"))
	int32 Exposure;

	/** Automatic white balance, default = true */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bAutoWhiteBalance;

	/** Automatic gain and exposure, default = true */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bAutoGainAndExposure;

	/** True to reset to default */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bDefault;
};

/*
 * SDK runtime parameters
 * see sl::RuntimeParameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlRuntimeParameters
{
	GENERATED_BODY()

	const TCHAR* Section = TEXT("Runtime");

	FSlRuntimeParameters()
		:
		SensingMode(ESlSensingMode::SM_Fill),
		bEnableDepth(true),
		ConfidenceThreshold(100),
		ReferenceFrame(ESlReferenceFrame::RF_World),
		bRemoveSaturatedAreas(true)
	{
	}

	FORCEINLINE void Load(const FString& Path)
	{
		int32 ConfigSensingMode;
		GConfig->GetInt(
			Section,
			TEXT("SensingMode"),
			ConfigSensingMode,
			*Path
			);
		SensingMode = (ESlSensingMode)ConfigSensingMode;

		GConfig->GetBool(
			Section,
			TEXT("bEnableDepth"),
			bEnableDepth,
			*Path
			);

		GConfig->GetInt(
			Section,
			TEXT("ConfidenceThreshold"),
			ConfidenceThreshold,
			*Path
			);

		int32 ConfigReferenceFrame;
		GConfig->GetInt(
			Section,
			TEXT("ReferenceFrame"),
			ConfigReferenceFrame,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bRemoveSaturatedAreas"),
			bRemoveSaturatedAreas,
			*Path
		);
		ReferenceFrame = (ESlReferenceFrame)ConfigReferenceFrame;
	}

	FORCEINLINE void Save(const FString& Path) const
	{
		GConfig->SetInt(
			Section,
			TEXT("SensingMode"),
			static_cast<int32>(SensingMode),
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bEnableDepth"),
			bEnableDepth,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("ConfidenceThreshold"),
			ConfidenceThreshold,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("ReferenceFrame"),
			static_cast<int32>(ReferenceFrame),
			*Path
		);

		GConfig->SetBool(
			Section,
			TEXT("bRemoveSaturatedAreas"),
			bRemoveSaturatedAreas,
			*Path
		);
	}

	/** Sensing mode */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlSensingMode SensingMode;

	/** Enable depth (need to be true if tracking enabled) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bEnableDepth;

	/** Enable point cloud */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int ConfidenceThreshold;

	/** Reference frame */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlReferenceFrame ReferenceFrame;

	/** Remove saturated areas */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bRemoveSaturatedAreas;
};

/*
 * SDK timestamp
 * Only holding the timestamp
 * see sl::timeStamp
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlTimestamp
{
	GENERATED_BODY()

	FSlTimestamp()
		:
		timestamp((sl::Timestamp)0)
	{
	}

	FSlTimestamp(sl::Timestamp tstamp)
		:
		timestamp(tstamp)
	{
	}

	FSlTimestamp& operator=(sl::Timestamp NewTimestamp)
	{
		timestamp = NewTimestamp;
		return *this;
	}

	FString ToString() const
	{
		return FString::Printf(TEXT("%llu"), timestamp.data_ns);
	}

	/** Underlying timestamp */
	sl::Timestamp timestamp;
};

/*
 * SDK tracking parameters
 * see sl::TrackingParameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlPositionalTrackingParameters
{
	GENERATED_BODY()

	const TCHAR* Section = TEXT("Tracking");

	FSlPositionalTrackingParameters()
		:
		Rotation(FRotator::ZeroRotator),
		Location(FVector::ZeroVector),
		bEnableTracking(true),
		bEnableAreaMemory(false),
		bEnablePoseSmoothing(true),
		bLoadSpatialMemoryFile(false),
		SpatialMemoryFileLoadingPath(""),
		SpatialMemoryFileSavingPath(""),
		TrackingType(ETrackingType::TrT_ZED)
	{
	}

	FORCEINLINE void Load(const FString& Path)
	{
		GConfig->GetString(
			Section,
			TEXT("SpatialMemoryFileLoadingPath"),
			SpatialMemoryFileLoadingPath,
			*Path
			);

		GConfig->GetString(
			Section,
			TEXT("SpatialMemoryFileSavingPath"),
			SpatialMemoryFileSavingPath,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bEnableTracking"),
			bEnableTracking,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bEnableAreaMemory"),
			bEnableAreaMemory,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bEnablePoseSmoothing"),
			bEnablePoseSmoothing,
			*Path
		);

		GConfig->GetBool(
			Section,
			TEXT("bLoadSpatialMemoryFile"),
			bLoadSpatialMemoryFile,
			*Path
			);

		GConfig->GetVector(
			Section,
			TEXT("Location"),
			Location,
			*Path
			);

		GConfig->GetRotator(
			Section,
			TEXT("Rotation"),
			Rotation,
			*Path
			);

		int32 ConfigTrackingType;
		GConfig->GetInt(
			Section,
			TEXT("TrackingType"),
			ConfigTrackingType,
			*Path
		);
		TrackingType = (ETrackingType)ConfigTrackingType;
	}

	FORCEINLINE void Save(const FString& Path) const
	{
		GConfig->SetString(
			Section,
			TEXT("SpatialMemoryFileLoadingPath"),
			*SpatialMemoryFileLoadingPath,
			*Path
			);

		GConfig->SetString(
			Section,
			TEXT("SpatialMemoryFileSavingPath"),
			*SpatialMemoryFileSavingPath,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bEnableTracking"),
			bEnableTracking,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bEnableAreaMemory"),
			bEnableAreaMemory,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bEnablePoseSmoothing"),
			bEnablePoseSmoothing,
			*Path
		);

		GConfig->SetBool(
			Section,
			TEXT("bLoadSpatialMemoryFile"),
			bLoadSpatialMemoryFile,
			*Path
			);

		GConfig->SetVector(
			Section,
			TEXT("Location"),
			Location,
			*Path
			);

		GConfig->SetRotator(
			Section,
			TEXT("Rotation"),
			Rotation,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("TrackingType"),
			static_cast<int32>(TrackingType),
			*Path
		);
	}

	/*
	 * Initial position. 
	 * If using HMD tracking origin, this is the HMD location.
	 * If not using HMD tracking origin, this is an offset from origin added to the tracking.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector Location;

	/*
	 * Initial rotation.
	 * If using HMD tracking origin, this is the HMD rotation.
	 * If not using HMD tracking origin, this is an offset from origin added to the tracking.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FRotator Rotation;

	/** Enable positional tracking */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Tracking"))
	bool bEnableTracking;

	/** Enable area localization */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "Area Memory"))
	bool bEnableAreaMemory;

	/** Enable smooth pose */
	bool bEnablePoseSmoothing;

	/** Load area file when enabling tracking */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bLoadSpatialMemoryFile;

	/** Path to the area file */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString SpatialMemoryFileLoadingPath;

	/** Path to area file */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString SpatialMemoryFileSavingPath;

	/** Tracking type 
	* Allow to chose which tracking is selected for the pawn.
	* ZED : Zed tracking only
	* HMD : Hmd tracking only
	* Mixte : Zed Imu rotations and Hmd translations
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ETrackingType TrackingType;
};

/*
 * SDK pose
 * see Sl::Pose
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlPose
{
	GENERATED_BODY()

	FSlPose()
		:
		Transform(FTransform::Identity),
		Timestamp((sl::Timestamp)0),
		Confidence(0),
		bValid(false)
	{
	}

	/** Translation and rotation */
	FTransform Transform;

	/** Timestamp of the pose. This timestamp should be compared with the camera timestamp for synchronization. */
	FSlTimestamp Timestamp;

	/** 
	 * Confidence/Quality of the pose estimation for the target frame.
	 * A confidence metric of the tracking [0-100], 0 means that the tracking is lost, 100 means that the tracking can be fully trusted.
	 */
	int Confidence;

	/** Indicates if tracking is activated or not. You should check that first if something wrong. */
	bool bValid;
};

/*
 * SDK IMU data
 * see Sl::IMUData
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlIMUData : public FSlPose
{
	GENERATED_BODY()

	FSlIMUData()
		:
		OrientationCovariance(FMatrix::Identity),
		AngularVelocity(FVector::ZeroVector),
		LinearAcceleration(FVector::ZeroVector),
		AngularVelocityConvariance(FMatrix::Identity),
		LinearAccelerationConvariance(FMatrix::Identity),
		FSlPose()
	{
	}
	/** Covariance matrix for orientation (x,y,z axes) */
	FMatrix OrientationCovariance;

	/** Vector for angular velocity of the IMU, given in deg/s */
	FVector AngularVelocity;

	/** Vector for linear acceleration of the IMU, given in m/s^2 */
	FVector LinearAcceleration;

	/** Covariance matrix for the angular velocity */
	FMatrix AngularVelocityConvariance;

	/** Covariance matrix for the linear acceleration */
	FMatrix LinearAccelerationConvariance;
};

/*
 * SDK init parameters
 * see sl::InitParameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlInitParameters
{
	GENERATED_BODY()

	const TCHAR* Section = TEXT("Init");

	FSlInitParameters()
		:
		Resolution(ESlResolution::R_HD720),
		FPS(60),
		InputType(ESlInputType::IT_USB),
		StreamIP(""),
		StreamPort(30000),
		SVOFilePath(""),
		bRealTime(false),
		DepthMode(ESlDepthMode::DM_Performance),
		Unit(ESlUnit::DU_Centimeter),
		CoordinateSystem(ESlCoordinateSystem::CS_Cartesian),
		bVerbose(false),
		VerboseFilePath(""),
		GPUID(-1.0f),
		DepthMinimumDistance(10.0f),
		DepthMaximumDistance(2000.0f),
		bDisableSelfCalibration(false),
		VerticalFlipImage(ESlFlipMode::FP_AUTO),
		bEnableRightSideMeasure(false),
		bEnableDepthStabilization(true)
	{
	}

	FORCEINLINE void Load(const FString& Path)
	{
		int32 ConfigDepthMode;
		GConfig->GetInt(
			Section,
			TEXT("DepthMode"),
			ConfigDepthMode,
			*Path
			);
		DepthMode = (ESlDepthMode)ConfigDepthMode;

		/*int32 ConfigUnit;
		GConfig->GetInt(
			Section,
			TEXT("Unit"),
			ConfigUnit,
			*Path
			);
		Unit = (ESlUnit)ConfigUnit;

		int32 ConfigCoordinateSystem;
		GConfig->GetInt(
			Section,
			TEXT("CoordinateSystem"),
			ConfigCoordinateSystem,
			*Path
			);
		CoordinateSystem = (ESlCoordinateSystem)ConfigCoordinateSystem;*/

		GConfig->GetFloat(
			Section,
			TEXT("GPUID"),
			GPUID,
			*Path
			);

		int32 inputType;
		GConfig->GetInt(
			Section,
			TEXT("InputType"),
			inputType,
			*Path
		);
		InputType = (ESlInputType)inputType;

		GConfig->GetFloat(
			Section,
			TEXT("DepthMinimumDistance"),
			DepthMinimumDistance,
			*Path
			);

		GConfig->GetFloat(
			Section,
			TEXT("DepthMaximumDistance"),
			DepthMaximumDistance,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bRealTime"),
			bRealTime,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bVerbose"),
			bVerbose,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bDisableSelfCalibration"),
			bDisableSelfCalibration,
			*Path
			);

		int32 FlipMode;
		GConfig->GetInt(
			Section,
			TEXT("VerticalFlipImage"),
			FlipMode,
			*Path
		);
		VerticalFlipImage = (ESlFlipMode)FlipMode;

		int32 ConfigResolution;
		GConfig->GetInt(
			Section,
			TEXT("Resolution"),
			ConfigResolution,
			*Path
			);
		Resolution = (ESlResolution)ConfigResolution;

		GConfig->GetInt(
			Section,
			TEXT("FPS"),
			FPS,
			*Path
			);

		GConfig->GetString(
			Section,
			TEXT("SVOFilePath"),
			SVOFilePath,
			*Path
			);

		GConfig->GetString(
			Section,
			TEXT("StreamIP"),
			StreamIP,
			*Path
		);

		GConfig->GetInt(
			Section,
			TEXT("StreamPort"),
			StreamPort,
			*Path
		);

		GConfig->GetString(
			Section,
			TEXT("VerboseFilePath"),
			VerboseFilePath,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bEnableDepthStabilization"),
			bEnableDepthStabilization,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("bEnableRightSideMeasure"),
			bEnableRightSideMeasure,
			*Path
		);
	}

	FORCEINLINE void Save(const FString& Path) const
	{
		GConfig->SetInt(
			Section,
			TEXT("Resolution"),
			static_cast<int32>(Resolution),
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("FPS"),
			FPS,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("InputType"),
			static_cast<int32>(InputType),
			*Path
		);

		GConfig->SetBool(
			Section,
			TEXT("bRealTime"),
			bRealTime,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("DepthMode"),
			static_cast<int32>(DepthMode),
			*Path
			);

		/*GConfig->SetInt(
			Section,
			TEXT("Unit"),
			static_cast<int32>(Unit),
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("CoordinateSystem"),
			static_cast<int32>(CoordinateSystem),
			*Path
			);*/

		GConfig->SetFloat(
			Section,
			TEXT("GPUID"),
			GPUID,
			*Path
			);

		GConfig->SetFloat(
			Section,
			TEXT("DepthMinimumDistance"),
			DepthMinimumDistance,
			*Path
			);

		GConfig->SetFloat(
			Section,
			TEXT("DepthMaximumDistance"),
			DepthMaximumDistance,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bVerbose"),
			bVerbose,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bDisableSelfCalibration"),
			bDisableSelfCalibration,
			*Path
			);

		GConfig->SetInt(
			Section,
			TEXT("VerticalFlipImage"),
			static_cast<int32>(VerticalFlipImage),
			*Path
		);

		GConfig->SetString(
			Section,
			TEXT("StreamIP"),
			*StreamIP,
			*Path
		);

		GConfig->SetInt(
			Section,
			TEXT("StreamPort"),
			StreamPort,
			*Path
		);

		GConfig->SetString(
			Section,
			TEXT("SVOFilePath"),
			*SVOFilePath,
			*Path
			);

		GConfig->SetString(
			Section,
			TEXT("VerboseFilePath"),
			*VerboseFilePath,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bEnableDepthStabilization"),
			bEnableDepthStabilization,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("bEnableRightSideMeasure"),
			bEnableRightSideMeasure,
			*Path
		);
	}

	/** Input type used in the ZED SDK */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlInputType InputType;

	/** Path to an SVO file if inputType is set to SVO */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString SVOFilePath;

	/** IP of the sender camera if inputType is set to Stream */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString StreamIP;

	/** Port of the sender camera if inputType is set to Stream */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int StreamPort;

	/** Verbose file path */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString VerboseFilePath;

	/** Minimum distance for depth */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float DepthMinimumDistance;

	/** Maximum distance for depth */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float DepthMaximumDistance;

	/** Capture fps of the camera */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	int FPS;

	/** Device selected */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float GPUID;

	/** Resolution of the camera (720p if used with HMD) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlResolution Resolution;

	/** Disparity */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlDepthMode DepthMode;

	/** Unit */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlUnit Unit;

	/** Coordinate system */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlCoordinateSystem CoordinateSystem;

	/** If true, skip some SVO frame if computation time is too long */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bRealTime;

	/** Verbose ZED */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bVerbose;

	/** Disable self calibration */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bDisableSelfCalibration;

	/** Vertical flip */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlFlipMode VerticalFlipImage;

	/** Enable right side measure */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bEnableRightSideMeasure;

	/** Enable depth stabilizer (need to be false if tracking disabled) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool bEnableDepthStabilization;
};

/*
 * SVO parameters used for recording
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlSVOParameters
{
	GENERATED_BODY()

	const TCHAR* Section = TEXT("SVO");

	FSlSVOParameters()
		:
		RecordingFilePath(""),
		CompressionMode(ESlSVOCompressionMode::SCM_Lossless),
		bLoop(false)
	{
	}

	FORCEINLINE void Load(const FString& Path)
	{
		int32 ConfigCompressionMode;
		GConfig->GetInt(
			Section,
			TEXT("CompressionMode"),
			ConfigCompressionMode,
			*Path
			);
		CompressionMode = (ESlSVOCompressionMode)ConfigCompressionMode;

		GConfig->GetString(
			Section,
			TEXT("RecordingFilePath"),
			RecordingFilePath,
			*Path
		);

		bool bConfigLoop;
		GConfig->GetBool(
			Section,
			TEXT("bLoop"),
			bConfigLoop,
			*Path
		);
		bLoop = bConfigLoop;
	}

	FORCEINLINE void Save(const FString& Path) const
	{
		GConfig->SetInt(
			Section,
			TEXT("CompressionMode"),
			static_cast<int32>(CompressionMode),
			*Path
		);

		GConfig->SetString(
			Section,
			TEXT("RecordingFilePath"),
			*RecordingFilePath,
			*Path
		);

		GConfig->SetBool(
			Section,
			TEXT("bLoop"),
			bLoop,
			*Path
		);
	}

	/** Path to save the SVO file */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString RecordingFilePath;

	/** SVO compression mode */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlSVOCompressionMode CompressionMode;

	/** True to loop when SVO playback enabled */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	uint8 bLoop:1;
};

/*
 * Anti drift parameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlAntiDriftParameters
{
	GENERATED_BODY()
		
	const TCHAR* Section = TEXT("HMD");

	FSlAntiDriftParameters()
		:
		CalibrationTransform(FTransform(FQuat::Identity, FVector(13.5, -3.15, 0)))
	{
	}

	FORCEINLINE void Load(const FString& Path)
	{
		FVector HMDToZedVector;
		GConfig->GetVector(
			Section,
			TEXT("HMDToZedVector"),
			HMDToZedVector,
			*Path
			);

		FRotator HMDToZedRotator;
		GConfig->GetRotator(
			Section,
			TEXT("HMDToZedRotator"),
			HMDToZedRotator,
			*Path
			);
		CalibrationTransform = FTransform(HMDToZedRotator, HMDToZedVector);
	}

	FORCEINLINE void Save(const FString& Path) const
	{
		GConfig->SetVector(
			Section,
			TEXT("HMDToZedVector"),
			CalibrationTransform.GetLocation(),
			*Path
			);

		GConfig->SetRotator(
			Section,
			TEXT("HMDToZedRotator"),
			CalibrationTransform.Rotator(),
			*Path
			);
	}

	/** Zed to HMD offset */
	UPROPERTY(BlueprintReadWrite)
	FTransform CalibrationTransform;
};

/*
 * Rendering parameters
 */
USTRUCT(BlueprintType, Category = "Stereolabs|Struct")
struct STEREOLABS_API FSlRenderingParameters
{
	GENERATED_BODY()
		
	const TCHAR* Section = TEXT("Rendering");

	FSlRenderingParameters()
		:
		PerceptionDistance(100.0f),
		SRemapEnable(false),
		ThreadingMode(ESlThreadingMode::TM_MultiThreaded)
	{}

	FORCEINLINE void Load(const FString& Path)
	{
		GConfig->GetFloat(
			Section,
			TEXT("PerceptionDistance"),
			PerceptionDistance,
			*Path
			);

		GConfig->GetBool(
			Section,
			TEXT("SRemapEnable"),
			SRemapEnable,
			*Path
		);

		int32 ConfigThreadingMode;
		GConfig->GetInt(
			Section,
			TEXT("ThreadingMode"),
			ConfigThreadingMode,
			*Path
			);
		ThreadingMode = static_cast<ESlThreadingMode>(ConfigThreadingMode);
	}

	FORCEINLINE void Save(const FString& Path) const
	{
		GConfig->SetFloat(
			Section,
			TEXT("PerceptionDistance"),
			PerceptionDistance,
			*Path
			);

		GConfig->SetBool(
			Section,
			TEXT("SRemapEnable"),
			SRemapEnable,
			*Path
		);

		GConfig->SetInt(
			Section,
			TEXT("ThreadingMode"),
			static_cast<int32>(ThreadingMode),
			*Path
			);
	}

	/** Distance in cm at which real object perfectly match their real size, between 75 and 300. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "75", ClampMax = "3000"))
	float PerceptionDistance;

	/** ! Experimental ! : enable SRemap. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	bool SRemapEnable;

	/** Threading mode of the Grab. Multithreading is recommended for better performance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESlThreadingMode ThreadingMode;
};