// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "AIController.h"
#include "TankAIController.generated.h"

/**
 *
 */
UCLASS()
class VEHICLEAIPROJECT_API ATankAIController : public AAIController
{
    GENERATED_BODY()

    virtual void OnMoveCompleted(FAIRequestID RequestID, EPathFollowingResult::Type Result) override;

};
