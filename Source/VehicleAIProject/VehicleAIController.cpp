// Fill out your copyright notice in the Description page of Project Settings.

#include "VehicleAIProject.h"
#include "VehicleAIController.h"
#include "VehiclePathFollowingComponent.h"

AVehicleAIController::AVehicleAIController(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UVehiclePathFollowingComponent>(TEXT("PathFollowingComponent")))
{
}
