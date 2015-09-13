// Fill out your copyright notice in the Description page of Project Settings.

#include "VehicleAIProject.h"
#include "TankAIController.h"
#include "TankVehicleMovementComponent.h"
#include "TankWheeledVehicle.h"

void ATankAIController::OnMoveCompleted(FAIRequestID RequestID, EPathFollowingResult::Type Result)
{
    AAIController::OnMoveCompleted(RequestID, Result);
    GetPawn()->GetMovementComponent()->StopMovementImmediately();
}
