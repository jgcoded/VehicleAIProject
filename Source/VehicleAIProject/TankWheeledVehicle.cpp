// Fill out your copyright notice in the Description page of Project Settings.

#include "VehicleAIProject.h"
#include "TankWheeledVehicle.h"
#include "TankVehicleMovementComponent.h"

ATankWheeledVehicle::ATankWheeledVehicle(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UTankVehicleMovementComponent>(VehicleMovementComponentName))
{
}
