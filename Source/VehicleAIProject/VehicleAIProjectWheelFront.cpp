// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.

#include "VehicleAIProject.h"
#include "VehicleAIProjectWheelFront.h"
#include "Vehicles/TireType.h"

UVehicleAIProjectWheelFront::UVehicleAIProjectWheelFront()
{
	ShapeRadius = 18.f;
	ShapeWidth = 15.0f;
	bAffectedByHandbrake = false;
	SteerAngle = 40.f;

	// Setup suspension forces
	SuspensionForceOffset = -4.0f;
	SuspensionMaxRaise = 8.0f;
	SuspensionMaxDrop = 12.0f;
	SuspensionNaturalFrequency = 9.0f;
	SuspensionDampingRatio = 1.05f;

	// Find the tire object and set the data for it
	static ConstructorHelpers::FObjectFinder<UTireType> TireData(TEXT("/Game/VehicleAdv/Vehicle/WheelData/Vehicle_FrontTireType.Vehicle_FrontTireType"));
	TireType = TireData.Object;
}
