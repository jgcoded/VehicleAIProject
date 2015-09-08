// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/WheeledVehicle.h"
#include "TankWheeledVehicle.generated.h"

/**
 *
 */
UCLASS()
class VEHICLEAIPROJECT_API ATankWheeledVehicle : public AWheeledVehicle
{
    GENERATED_UCLASS_BODY()

    virtual void Tick(float DeltaSeconds) override;

};
