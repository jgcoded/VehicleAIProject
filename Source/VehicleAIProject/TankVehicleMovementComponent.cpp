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
    // From the Nvidia Tank snippet code:

    /* {
        6.0f,	//rise rate eANALOG_INPUT_ACCEL=0,
        6.0f,	//rise rate eANALOG_INPUT_BRAKE,
        6.0f,	//rise rate eANALOG_INPUT_HANDBRAKE,
        2.5f,	//rise rate eANALOG_INPUT_STEER_LEFT,
        2.5f,	//rise rate eANALOG_INPUT_STEER_RIGHT,
        },
        {
        10.0f,	//fall rate eANALOG_INPUT_ACCEL=0
        10.0f,	//fall rate eANALOG_INPUT_BRAKE_LEFT
        10.0f,	//fall rate eANALOG_INPUT_BRAKE_RIGHT
        5.0f,	//fall rate eANALOG_INPUT_THRUST_LEFT
        5.0f	//fall rate eANALOG_INPUT_THRUST_RIGHT
        }
    */

    LeftThrustInputRate.RiseRate = 2.5f;
    RightThrustInputRate.RiseRate = 2.5f;

    LeftThrustInputRate.FallRate = 5.0f;
    RightThrustInputRate.FallRate = 5.0f;

    LeftBrakeInputRate.RiseRate = 6.0f;
    RightBrakeInputRate.RiseRate = 6.0f;

    LeftBrakeInputRate.FallRate = 10.0f;
    RightBrakeInputRate.FallRate = 10.0f;


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

// Copy-paste from WheeledVehicleMovementComponent4W

void BackwardsConvertCm2ToM2(float& val, float defaultValue)
{
    if (val != defaultValue)
    {
        val = Cm2ToM2(val);
    }
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

// Copy-paste from WheeledVehicleMovementComponent4W
float FVehicleEngineData::FindPeakTorque() const
{
    // Find max torque
    float PeakTorque = 0.f;
    TArray<FRichCurveKey> TorqueKeys = TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
    for (int32 KeyIdx = 0; KeyIdx < TorqueKeys.Num(); KeyIdx++)
    {
        FRichCurveKey& Key = TorqueKeys[KeyIdx];
        PeakTorque = FMath::Max(PeakTorque, Key.Value);
    }
    return PeakTorque;
}

// Helper to convert from Unreal Engine data to PhysX data
void GetVehicleEngineSetup(const FVehicleEngineData& Setup, PxVehicleEngineData& PxSetup)
{
    PxSetup.mMOI = M2ToCm2(Setup.MOI);
    PxSetup.mMaxOmega = RPMToOmega(Setup.MaxRPM);
    PxSetup.mDampingRateFullThrottle = M2ToCm2(Setup.DampingRateFullThrottle);
    PxSetup.mDampingRateZeroThrottleClutchEngaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchEngaged);
    PxSetup.mDampingRateZeroThrottleClutchDisengaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchDisengaged);

    float PeakTorque = Setup.FindPeakTorque(); // In Nm
    PxSetup.mPeakTorque = M2ToCm2(PeakTorque);	// convert Nm to (kg cm^2/s^2)

    // Convert from our curve to PhysX
    PxSetup.mTorqueCurve.clear();
    TArray<FRichCurveKey> TorqueKeys = Setup.TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
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
    if (!UpdatedPrimitive)
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
    PxVehicleWheelsSimData* PWheelsSimData = PxVehicleWheelsSimData::allocate(4);
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
        RawInputData.setAnalogAccel(1.0f);
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
        RightTrackSpeed = PVehicle->mWheelsDynData.getWheelRotationSpeed(1);

    });
}

#endif

