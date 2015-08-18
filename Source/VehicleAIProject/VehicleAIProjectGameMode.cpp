// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.

#include "VehicleAIProject.h"
#include "VehicleAIProjectGameMode.h"
#include "VehicleAIProjectPawn.h"
#include "VehicleAIProjectHud.h"

AVehicleAIProjectGameMode::AVehicleAIProjectGameMode()
{
	DefaultPawnClass = AVehicleAIProjectPawn::StaticClass();
	HUDClass = AVehicleAIProjectHud::StaticClass();
}
