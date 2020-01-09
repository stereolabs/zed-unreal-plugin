//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

#pragma once

#include "Stereolabs/Public/Core/StereolabsBaseTypes.h"

#include <sl/Camera.hpp>

#include "StereolabsSensorsData.generated.h"

UCLASS(Category = "Stereolabs|Core")
class STEREOLABS_API USlSensorsData : public UObject
{
	GENERATED_BODY()

public:
	USlSensorsData();
	 

public:
	/* Underlying sl::SensorsData struct*/
	sl::SensorsData sdata;
};