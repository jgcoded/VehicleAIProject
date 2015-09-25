// Fill out your copyright notice in the Description page of Project Settings.

#include "VehicleAIProject.h"
#include "TankVehicleMovementComponent.h"
#include "PhysicsPublic.h"

#if WITH_VEHICLE
#include "PhysXIncludes.h"
#include "PxVehicleDriveTank.h"
#endif // WITH_VEHICLE


UTankVehicleMovementComponent::UTankVehicleMovementComponent(const FObjectInitializer& ObjectInitializer)
{
#if WITH_VEHICLE
    // The following values are from the NVidia Tank snippet code:
    LeftThrustInputRate.RiseRate = 2.5f;
    RightThrustInputRate.RiseRate = 2.5f;

    LeftThrustInputRate.FallRate = 5.0f;
    RightThrustInputRate.FallRate = 5.0f;

    LeftBrakeInputRate.RiseRate = 6.0f;
    RightBrakeInputRate.RiseRate = 6.0f;

    LeftBrakeInputRate.FallRate = 10.0f;
    RightBrakeInputRate.FallRate = 10.0f;

    RawLeftBrakeInput = 0;
    RawRightBrakeInput = 0;
    RawLeftThrottleInput = 0;
    RawRightThrottleInput = 0;

    MaximumTankAISpeed = 45.0f; // 45 Km/h
    MaximumTankAITurnRate = 10.0f; // 10 Deg/sec

    // Set up the engine to be more powerful but also more damped than the default engine.
    PxVehicleEngineData DefEngineData;
    DefEngineData.mPeakTorque *= 2.0f;
    DefEngineData.mDampingRateZeroThrottleClutchEngaged = 2.0f;
    DefEngineData.mDampingRateZeroThrottleClutchDisengaged = 0.5f;
    DefEngineData.mDampingRateFullThrottle = 0.5f;

    //  The following is a Copy-paste from vehiclemovementcomponent4w constructor:
    EngineSetup.MOI = DefEngineData.mMOI;
    EngineSetup.MaxRPM = OmegaToRPM(DefEngineData.mMaxOmega);
    EngineSetup.DampingRateFullThrottle = DefEngineData.mDampingRateFullThrottle;
    EngineSetup.DampingRateZeroThrottleClutchEngaged = DefEngineData.mDampingRateZeroThrottleClutchEngaged;
    EngineSetup.DampingRateZeroThrottleClutchDisengaged = DefEngineData.mDampingRateZeroThrottleClutchDisengaged;

    DriveTankControlModel = EDriveTankControlModel::Special;

    // Convert from PhysX curve to ours
    FRichCurve* TorqueCurveData = EngineSetup.TorqueCurve.GetRichCurve();
    for (PxU32 KeyIdx = 0; KeyIdx < DefEngineData.mTorqueCurve.getNbDataPairs(); KeyIdx++)
    {
        float Input = DefEngineData.mTorqueCurve.getX(KeyIdx) * EngineSetup.MaxRPM;
        float Output = DefEngineData.mTorqueCurve.getY(KeyIdx) * DefEngineData.mPeakTorque;
        TorqueCurveData->AddKey(Input, Output);
    }

    PxVehicleClutchData DefClutchData;
    TransmissionSetup.ClutchStrength = DefClutchData.mStrength;

    PxVehicleGearsData DefGearSetup;
    TransmissionSetup.GearSwitchTime = DefGearSetup.mSwitchTime;
    TransmissionSetup.ReverseGearRatio = DefGearSetup.mRatios[PxVehicleGearsData::eREVERSE];
    TransmissionSetup.FinalRatio = DefGearSetup.mFinalRatio;

    PxVehicleAutoBoxData DefAutoBoxSetup;
    TransmissionSetup.NeutralGearUpRatio = DefAutoBoxSetup.mUpRatios[PxVehicleGearsData::eNEUTRAL];
    TransmissionSetup.GearAutoBoxLatency = DefAutoBoxSetup.getLatency();
    TransmissionSetup.bUseGearAutoBox = true;

    for (uint32 i = PxVehicleGearsData::eFIRST; i < DefGearSetup.mNbRatios; i++)
    {
        FVehicleGearData GearData;
        GearData.DownRatio = DefAutoBoxSetup.mDownRatios[i];
        GearData.UpRatio = DefAutoBoxSetup.mUpRatios[i];
        GearData.Ratio = DefGearSetup.mRatios[i];
        TransmissionSetup.ForwardGears.Add(GearData);
    }

#endif // WITH_VEHICLE
}

void UTankVehicleMovementComponent::BackwardsConvertCm2ToM2(float& val, float defaultValue)
{
    if (val != defaultValue)
    {
        val = Cm2ToM2(val);
    }
}

float AngleDiff(const double angleSrc, const double angleDest, const bool radiansFlag)
{
    float diff = angleDest - angleSrc;

    if (radiansFlag)
    {
        while (diff > PI)
        {
            diff -= 2.0f*PI;
        }
        while (diff <= -PI)
        {
            diff += 2.0f*PI;
        }
    }
    else
    {
        while (diff > 180.0f)
        {
            diff -= 360.0f;
        }
        while (diff <= -180.0f)
        {
            diff += 360.0f;
        }
    }

    return diff;
}

void UTankVehicleMovementComponent::ApplyRequestedMove(float DeltaTime)
{

    if (!bHasRequestedVelocity)
        return;

    AActor* Owner = this->GetOwner();

    if (!Owner)
        return;

    SetBrakeInput(0.0f, 0.0f);

    // Update thrust

    float ForwardSpeedKmH = GetForwardSpeed() * 3600.0f / 100000.0f;	//convert from cm/s to km/h

    float SpeedError = MaximumTankAISpeed - ForwardSpeedKmH;

    Thrust += UPIDController::NextValue(VelocityController, SpeedError, DeltaTime);

    Thrust = FMath::Clamp(Thrust, 0.0f, VelocityController.UpperLimit);



    // Update heading

    float DesiredHeading = FRotationMatrix::MakeFromX(RequestedVelocity).Rotator().Yaw;
    float CurrentHeading = Owner->GetActorRotation().Yaw;

    float HeadingError = AngleDiff(CurrentHeading, DesiredHeading, false);

    if (fabs(HeadingError) > 45.0f)
        Thrust = 0.0f;

    Heading = UPIDController::NextValue(HeadingController, HeadingError, DeltaTime);

    Heading = FMath::Clamp(Heading, HeadingController.LowerLimit, HeadingController.UpperLimit);

    if (fabs(HeadingError) < 2.0f)
        Heading = 0.0f;

    // Do saturated mixing for left and right side
    // thrust values. This will provide us faster
    // forward speeds.

    float LeftThrottle = (Thrust + Heading) / 2.0f;
    float RightThrottle = (Thrust - Heading) / 2.0f;

    LeftThrottle /= (VelocityController.UpperLimit - VelocityController.LowerLimit) / 2.0f;
    RightThrottle /= (VelocityController.UpperLimit - VelocityController.LowerLimit) / 2.0f;

    LeftThrottle = FMath::Clamp(LeftThrottle, -1.0f, 1.0f);
    RightThrottle = FMath::Clamp(RightThrottle, -1.0f, 1.0f);

    //#ifdef NDEBUG
    //    if (GEngine)
    //    {
    //        DrawDebugLine(GetWorld(), Owner->GetActorLocation(), Owner->GetActorLocation() + RequestedVelocity, FColor::Red, false, -1.0f, 0, 10.0f);
    //        GEngine->AddOnScreenDebugMessage(0, 2.0f, FColor::Cyan, FString::Printf(TEXT("PID Thrust: %f"), Thrust));
    //        GEngine->AddOnScreenDebugMessage(1, 2.0f, FColor::Cyan, FString::Printf(TEXT("PID Heading Rate: %f"), Heading));
    //        GEngine->AddOnScreenDebugMessage(2, 2.0f, FColor::Cyan, FString::Printf(TEXT("Left Throttle: %f"), LeftThrottle));
    //        GEngine->AddOnScreenDebugMessage(3, 2.0f, FColor::Cyan, FString::Printf(TEXT("Right Throttle: %f"), RightThrottle));
    //        GEngine->AddOnScreenDebugMessage(4, 2.0f, FColor::Cyan, FString::Printf(TEXT("Current Velocity: %f Km/h"), ForwardSpeedKmH));
    //        GEngine->AddOnScreenDebugMessage(5, 2.0f, FColor::Cyan, FString::Printf(TEXT("Heading Error: %f degrees"), HeadingError));
    //        GEngine->AddOnScreenDebugMessage(6, 2.0f, FColor::Cyan, FString::Printf(TEXT("Speed Error: %f"), SpeedError));
    //        //GEngine->AddOnScreenDebugMessage(7, 2.0f, FColor::Cyan, FString::Printf(TEXT("brake: %f"), brake));
    //
    //    }
    //#endif

    SetThrottleInput((float)LeftThrottle, (float)RightThrottle);
}

void UTankVehicleMovementComponent::Serialize(FArchive& Ar)
{
    Super::Serialize(Ar);
    // copy-paste from WheeledVehicleMovementComponent4W

#if WITH_VEHICLE
    if (Ar.IsLoading() && Ar.UE4Ver() < VER_UE4_VEHICLES_UNIT_CHANGE)
    {
        PxVehicleEngineData DefEngineData;
        float DefaultRPM = OmegaToRPM(DefEngineData.mMaxOmega);

        //we need to convert from old units to new. This backwards compatable code fails in the rare case that they were using very strange values that are the new defaults in the correct units.
        EngineSetup.MaxRPM = EngineSetup.MaxRPM != DefaultRPM ? OmegaToRPM(EngineSetup.MaxRPM) : DefaultRPM;	//need to convert from rad/s to RPM
    }

    if (Ar.IsLoading() && Ar.UE4Ver() < VER_UE4_VEHICLES_UNIT_CHANGE2)
    {
        PxVehicleEngineData DefEngineData;
        PxVehicleClutchData DefClutchData;

        //we need to convert from old units to new. This backwards compatable code fails in the rare case that they were using very strange values that are the new defaults in the correct units.
        BackwardsConvertCm2ToM2(EngineSetup.DampingRateFullThrottle, DefEngineData.mDampingRateFullThrottle);
        BackwardsConvertCm2ToM2(EngineSetup.DampingRateZeroThrottleClutchDisengaged, DefEngineData.mDampingRateZeroThrottleClutchDisengaged);
        BackwardsConvertCm2ToM2(EngineSetup.DampingRateZeroThrottleClutchEngaged, DefEngineData.mDampingRateZeroThrottleClutchEngaged);
        BackwardsConvertCm2ToM2(EngineSetup.MOI, DefEngineData.mMOI);
        BackwardsConvertCm2ToM2(TransmissionSetup.ClutchStrength, DefClutchData.mStrength);
    }
#endif
}


void UTankVehicleMovementComponent::ComputeConstants()
{
    Super::ComputeConstants();
    MaxEngineRPM = EngineSetup.MaxRPM;
}


void UTankVehicleMovementComponent::SetThrottleInput(float LeftThrottle, float RightThrottle)
{
    if (DriveTankControlModel == EDriveTankControlModel::Standard)
    {
        RawLeftThrottleInput = FMath::Clamp(LeftThrottle, 0.0f, 1.0f);
        RawRightThrottleInput = FMath::Clamp(RightThrottle, 0.0f, 1.0f);
    }
    else
    {
        RawLeftThrottleInput = FMath::Clamp(LeftThrottle, -1.0f, 1.0f);
        RawRightThrottleInput = FMath::Clamp(RightThrottle, -1.0f, 1.0f);
    }
}

void UTankVehicleMovementComponent::SetBrakeInput(float LeftBrake, float RightBrake)
{
    RawLeftBrakeInput = FMath::Clamp(LeftBrake, 0.0f, 1.0f);
    RawRightBrakeInput = FMath::Clamp(RightBrake, 0.0f, 1.0f);
}

#if WITH_EDITOR
void UTankVehicleMovementComponent::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

    if (PropertyName == TEXT("DownRatio"))
    {
        for (int32 GearIdx = 0; GearIdx < TransmissionSetup.ForwardGears.Num(); ++GearIdx)
        {
            FVehicleGearData & GearData = TransmissionSetup.ForwardGears[GearIdx];
            GearData.DownRatio = FMath::Min(GearData.DownRatio, GearData.UpRatio);
        }
    }
    else if (PropertyName == TEXT("UpRatio"))
    {
        for (int32 GearIdx = 0; GearIdx < TransmissionSetup.ForwardGears.Num(); ++GearIdx)
        {
            FVehicleGearData & GearData = TransmissionSetup.ForwardGears[GearIdx];
            GearData.UpRatio = FMath::Max(GearData.DownRatio, GearData.UpRatio);
        }
    }
}
#endif


#if WITH_VEHICLE

void UTankVehicleMovementComponent::TickVehicle(float DeltaTime)
{
    Super::TickVehicle(DeltaTime);

    ApplyRequestedMove(DeltaTime);
}


// Helper to convert from Unreal Engine data to PhysX data
static void GetVehicleEngineSetup(const FVehicleEngineData& Setup, PxVehicleEngineData& PxSetup)
{
    PxSetup.mMOI = M2ToCm2(Setup.MOI);
    PxSetup.mMaxOmega = RPMToOmega(Setup.MaxRPM);
    PxSetup.mDampingRateFullThrottle = M2ToCm2(Setup.DampingRateFullThrottle);
    PxSetup.mDampingRateZeroThrottleClutchEngaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchEngaged);
    PxSetup.mDampingRateZeroThrottleClutchDisengaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchDisengaged);

    float PeakTorque = 0.f; // In Nm

    TArray<FRichCurveKey> TorqueKeys = Setup.TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
    for (int32 KeyIdx = 0; KeyIdx < TorqueKeys.Num(); KeyIdx++)
    {
        FRichCurveKey& Key = TorqueKeys[KeyIdx];
        PeakTorque = FMath::Max(PeakTorque, Key.Value);
    }

    PxSetup.mPeakTorque = M2ToCm2(PeakTorque);	// convert Nm to (kg cm^2/s^2)

    // Convert from our curve to PhysX
    PxSetup.mTorqueCurve.clear();
    int32 NumTorqueCurveKeys = FMath::Min<int32>(TorqueKeys.Num(), PxVehicleEngineData::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES);
    for (int32 KeyIdx = 0; KeyIdx < NumTorqueCurveKeys; KeyIdx++)
    {
        FRichCurveKey& Key = TorqueKeys[KeyIdx];
        PxSetup.mTorqueCurve.addPair(FMath::Clamp(Key.Time / Setup.MaxRPM, 0.f, 1.f), Key.Value / PeakTorque); // Normalize torque to 0-1 range
    }
}

// Helper to convert from Unreal Engine data to PhysX data
static void GetVehicleGearSetup(const FVehicleTransmissionData& Setup, PxVehicleGearsData& PxSetup)
{
    PxSetup.mSwitchTime = Setup.GearSwitchTime;
    PxSetup.mRatios[PxVehicleGearsData::eREVERSE] = Setup.ReverseGearRatio;
    for (int32 i = 0; i < Setup.ForwardGears.Num(); i++)
    {
        PxSetup.mRatios[i + PxVehicleGearsData::eFIRST] = Setup.ForwardGears[i].Ratio;
    }
    PxSetup.mFinalRatio = Setup.FinalRatio;
    PxSetup.mNbRatios = Setup.ForwardGears.Num() + PxVehicleGearsData::eFIRST;
}

// Helper to convert from Unreal Engine data to PhysX data
static void GetVehicleAutoBoxSetup(const FVehicleTransmissionData& Setup, PxVehicleAutoBoxData& PxSetup)
{
    for (int32 i = 0; i < Setup.ForwardGears.Num(); i++)
    {
        const FVehicleGearData& GearData = Setup.ForwardGears[i];
        PxSetup.mUpRatios[i] = GearData.UpRatio;
        PxSetup.mDownRatios[i] = GearData.DownRatio;
    }
    PxSetup.mUpRatios[PxVehicleGearsData::eNEUTRAL] = Setup.NeutralGearUpRatio;
    PxSetup.setLatency(Setup.GearAutoBoxLatency);
}


void SetupDriveHelper(const UTankVehicleMovementComponent* VehicleData, PxVehicleDriveSimData& DriveData)
{
    PxVehicleEngineData EngineSetup;
    GetVehicleEngineSetup(VehicleData->EngineSetup, EngineSetup);
    DriveData.setEngineData(EngineSetup);

    PxVehicleClutchData ClutchSetup;
    ClutchSetup.mStrength = M2ToCm2(VehicleData->TransmissionSetup.ClutchStrength);
    DriveData.setClutchData(ClutchSetup);

    PxVehicleGearsData GearSetup;
    GetVehicleGearSetup(VehicleData->TransmissionSetup, GearSetup);
    DriveData.setGearsData(GearSetup);

    PxVehicleAutoBoxData AutoBoxSetup;
    GetVehicleAutoBoxSetup(VehicleData->TransmissionSetup, AutoBoxSetup);
    DriveData.setAutoBoxData(AutoBoxSetup);
}

void UTankVehicleMovementComponent::SetupVehicle()
{
    if (!UpdatedPrimitive || WheelSetups.Num() == 0)
    {
        return;
    }

    for (int32 WheelIdx = 0; WheelIdx < WheelSetups.Num(); ++WheelIdx)
    {
        const FWheelSetup& WheelSetup = WheelSetups[WheelIdx];
        if (WheelSetup.BoneName == NAME_None)
        {
            return;
        }
    }

    // Setup the chassis and wheel shapes
    SetupVehicleShapes();

    // Setup mass properties
    SetupVehicleMass();

    // Setup the wheels
    PxVehicleWheelsSimData* PWheelsSimData = PxVehicleWheelsSimData::allocate(WheelSetups.Num());
    SetupWheels(PWheelsSimData);

    // Setup drive data
    PxVehicleDriveSimData DriveData;
    SetupDriveHelper(this, DriveData);

    // Set up the engine to be more powerful but also more damped than the default engine.
    PxVehicleEngineData engineData = DriveData.getEngineData();
    engineData.mPeakTorque *= 2.0f;
    engineData.mDampingRateZeroThrottleClutchEngaged = 2.0f;
    engineData.mDampingRateZeroThrottleClutchDisengaged = 0.5f;
    engineData.mDampingRateFullThrottle = 0.5f;
    DriveData.setEngineData(engineData);

    PxVehicleDriveTank* PVehicleDriveTank = PxVehicleDriveTank::allocate(WheelSetups.Num());
    check(PVehicleDriveTank);

    FBodyInstance* BodyInstance = UpdatedPrimitive->GetBodyInstance();
    BodyInstance->ExecuteOnPhysicsReadWrite([&]
    {
        PVehicleDriveTank->setup(GPhysXSDK,
            BodyInstance->GetPxRigidDynamic_AssumesLocked(),
            *PWheelsSimData, DriveData, WheelSetups.Num());

        PVehicleDriveTank->setToRestState();
        PVehicleDriveTank->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);

        PxVehicleDriveTankControlModel::Enum PxDriveTankControlModel =
            PxVehicleDriveTankControlModel::eSTANDARD;

        if (DriveTankControlModel == EDriveTankControlModel::Special)
            PxDriveTankControlModel = PxVehicleDriveTankControlModel::eSPECIAL;

        PVehicleDriveTank->setDriveModel(PxDriveTankControlModel);
        PWheelsSimData->free();
    });

    PWheelsSimData = nullptr;

    PVehicle = PVehicleDriveTank;
    PVehicleDrive = PVehicleDriveTank;

    SetUseAutoGears(TransmissionSetup.bUseGearAutoBox);
}

void UTankVehicleMovementComponent::UpdateSimulation(float DeltaTime)
{
    if (PVehicleDrive == nullptr)
        return;

    UpdatedPrimitive->GetBodyInstance()->ExecuteOnPhysicsReadWrite([&]
    {
        // #TODO explore the different inputs for tanks
        // only use analog inputs for now, use digital later
        PxVehicleDriveTankRawInputData RawInputData((PxVehicleDriveTankControlModel::Enum)DriveTankControlModel.GetValue());

        if (fabs(RawLeftThrottleInput) > DELTA || fabs(RawRightThrottleInput) > DELTA)
        {
            RawInputData.setAnalogAccel(1.0f);
        }
        else
        {
            RawInputData.setAnalogAccel(0.0f);
        }
        RawInputData.setAnalogLeftThrust(RawLeftThrottleInput);
        RawInputData.setAnalogRightThrust(RawRightThrottleInput);
        RawInputData.setAnalogLeftBrake(RawLeftBrakeInput);
        RawInputData.setAnalogRightBrake(RawRightBrakeInput);

        if (!PVehicleDrive->mDriveDynData.getUseAutoGears())
        {
            RawInputData.setGearUp(bRawGearUpInput);
            RawInputData.setGearDown(bRawGearDownInput);
        }

        PxVehiclePadSmoothingData PadSmoothingData =
        {
            {
                ThrottleInputRate.RiseRate,
                LeftBrakeInputRate.RiseRate,
                RightBrakeInputRate.RiseRate,
                LeftThrustInputRate.RiseRate,
                RightThrustInputRate.RiseRate
            },
            {
                ThrottleInputRate.FallRate,
                LeftBrakeInputRate.FallRate,
                RightBrakeInputRate.FallRate,
                LeftThrustInputRate.FallRate,
                RightThrustInputRate.FallRate
            }
        };

        PxVehicleDriveTank* PVehicleDriveTank = (PxVehicleDriveTank*)PVehicleDrive;
        PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs(PadSmoothingData, RawInputData, DeltaTime, *PVehicleDriveTank);

        // From the NVidia PhysX docs:https://developer.nvidia.com/sites/default/files/akamai/physx/Docs/Vehicles.html
        // Vehicle State Queries section.
        LeftWheelsSpeed = PVehicle->mWheelsDynData.getWheelRotationSpeed(0);
        RightWheelsSpeed = PVehicle->mWheelsDynData.getWheelRotationSpeed(1);

    });
}

void UTankVehicleMovementComponent::RequestDirectMove(const FVector& MoveVelocity, bool bForceMaxSpeed)
{
    Super::RequestDirectMove(MoveVelocity, bForceMaxSpeed);
    if (MoveVelocity.SizeSquared() < KINDA_SMALL_NUMBER)
    {
        SetThrottleInput(0.0f, 0.0f);
        SetBrakeInput(1.0f, 1.0f);
        return;
    }

    bHasRequestedVelocity = true;
    RequestedVelocity = MoveVelocity;
}

void UTankVehicleMovementComponent::StopActiveMovement()
{
    Super::StopActiveMovement();
    bHasRequestedVelocity = false;
    UPIDController::Clear(VelocityController);
    UPIDController::Clear(HeadingController);
    SetThrottleInput(0.0f, 0.0f);
    SetBrakeInput(1.0f, 1.0f);
}

void UTankVehicleMovementComponent::StopMovementImmediately()
{
    Super::StopMovementImmediately();
    bHasRequestedVelocity = false;
    UPIDController::Clear(VelocityController);
    UPIDController::Clear(HeadingController);
    SetThrottleInput(0.0f, 0.0f);
    SetBrakeInput(1.0f, 1.0f);
}

#endif
