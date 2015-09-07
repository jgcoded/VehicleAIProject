// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Vehicles/WheeledVehicleMovementComponent.h"
#include "TankVehicleMovementComponent.generated.h"

/**
    From the NVidia PhysX api:
    With eSTANDARD the legal range of left and right thrust is (0,1).
    With eSPECIAL the legal range of left and right thrust is (-1,1).
*/
UENUM()
enum EDriveTankControlModel
{
    Standard = 0,
    Special
};

/**
 *
 */
UCLASS()
class VEHICLEAIPROJECT_API UTankVehicleMovementComponent : public UWheeledVehicleMovementComponent
{
    GENERATED_UCLASS_BODY()

    virtual void Serialize(FArchive& Ar) override;
    virtual void ComputeConstants() override;

    /** Engine */
    UPROPERTY(EditAnywhere, Category = "MechanicalSetup")
    FVehicleEngineData EngineSetup;

    /** Transmission data */
    UPROPERTY(EditAnywhere, Category = "MechanicalSetup")
    FVehicleTransmissionData TransmissionSetup;

    UPROPERTY(EditAnywhere, Category = "TankSetup")
    TEnumAsByte<EDriveTankControlModel> DriveTankControlModel;

    /* Rate at which left input throttle can rise and fall */
    UPROPERTY(EditAnywhere, Category = "VehicleInput", AdvancedDisplay)
    FVehicleInputRate LeftThrustInputRate;

    /* Rate at which right input throttle can rise and fall */
    UPROPERTY(EditAnywhere, Category = "VehicleInput", AdvancedDisplay)
    FVehicleInputRate RightThrustInputRate;

    /* Rate at which left input brake can rise and fall */
    UPROPERTY(EditAnywhere, Category = "VehicleInput", AdvancedDisplay)
    FVehicleInputRate LeftBrakeInputRate;

    /* Rate at which right input brake can rise and fall */
    UPROPERTY(EditAnywhere, Category = "VehicleInput", AdvancedDisplay)
    FVehicleInputRate RightBrakeInputRate;

    /**
        The motion of the caterpillar tracks could be rendered with
        a scrolling texture, safe in the knowledge that all wheels
        [on the same side] have the same speed. This is from the
        NVidia PhysX docs. Rad/sec.
    */
    UPROPERTY(BlueprintReadOnly, Transient, Category = "Track")
    float LeftWheelsSpeed;

    /**
        The motion of the caterpillar tracks could be rendered with
        a scrolling texture, safe in the knowledge that all wheels
        [on the same side] have the same speed. This is from the
        NVidia PhysX docs. Rad/sec.
    */
    UPROPERTY(BlueprintReadOnly, Transient, Category = "Track")
    float RightTrackSpeed;

    /* What the player has the left accelerator set to. */
    UPROPERTY(Transient)
    float RawLeftThrottleInput;

    /* What the player has the right accelerator set to. */
    UPROPERTY(Transient)
    float RawRightThrottleInput;

    /* What the player has the brake set to. Range: 0..1 */
    UPROPERTY(Transient) 
    float RawLeftBrakeInput;

    /* What the player has the brake set to. Range: 0..1 */
    UPROPERTY(Transient)
    float RawRightBrakeInput;

    /**
        \brief Apply a throttle input to the left and right set of wheels.
    
        Valid values for LeftThrottle and RightThrottle depend on the
        selected tank control model. \see EDriveTankControlModel.

        \param LeftThrottle The input throttle for the left wheels
        \param RightThrottle The input throttle for the right wheels
    */
    UFUNCTION(BlueprintCallable, Category = "TankVehicleMovement")
    void SetThrottleInput(float LeftThrottle, float RightThrottle);

    /*!
        \brief Apply a brake input to the left and right set of wheels.
    
        Valid values are from 0..1.

        \param LeftBrake Value from 0..1
        \param RightBrake Value from 0..1
    */
    UFUNCTION(BlueprintCallable, Category = "TankVehicleMovement")
    void SetBrakeInput(float LeftBrake, float RightBrake);

#if WITH_EDITOR
    virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

protected:

#if WITH_VEHICLE

    virtual void SetupVehicle() override;

    virtual void UpdateSimulation(float DeltaTime) override;

#endif

};
