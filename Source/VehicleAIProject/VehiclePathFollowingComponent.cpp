// Fill out your copyright notice in the Description page of Project Settings.

#include "VehicleAIProject.h"
#include "VehiclePathFollowingComponent.h"
#include "GameFramework/WheeledVehicle.h"
#include "Vehicles/WheeledVehicleMovementComponent.h"

#define EPSILON 1e-9

UVehiclePathFollowingComponent::UVehiclePathFollowingComponent()
    : CurrentThrottle(0.0f)
    , CurrentSteering(0.0f)
    , InitialDistanceToDestination(-1.0f)
{
    VelocityController.P = 0.05f;
    VelocityController.I = 0.001f;
    VelocityController.D = 0.03f;
    VelocityController.LowerLimit = -1.0f;
    VelocityController.UpperLimit = 1.0f;

    HeadingController.P = 0.05f;
    HeadingController.I = 0.001f;
    HeadingController.D = 0.0f;
    HeadingController.LowerLimit = -1.0f;
    HeadingController.UpperLimit = 1.0f;
}

void UVehiclePathFollowingComponent::SetVelocityPID(float P, float I, float D, float LowerLimit, float UpperLimit)
{
    VelocityController.P = P;
    VelocityController.I = I;
    VelocityController.D = D;
    VelocityController.LowerLimit = LowerLimit;
    VelocityController.UpperLimit = UpperLimit;
}

void UVehiclePathFollowingComponent::SetRotationPID(float P, float I, float D, float LowerLimit, float UpperLimit)
{
    HeadingController.P = P;
    HeadingController.I = I;
    HeadingController.D = D;
    HeadingController.LowerLimit = LowerLimit;
    HeadingController.UpperLimit = UpperLimit;
}

void UVehiclePathFollowingComponent::SetMoveSegment(int32 SegmentStartIndex)
{
    UPathFollowingComponent::SetMoveSegment(SegmentStartIndex);

    AWheeledVehicle* Owner = Cast<AWheeledVehicle>(MovementComp->GetOwner());

    if (Owner)
    {
        InitialDistanceToDestination = (Owner->GetActorLocation() - GetCurrentTargetLocation()).Size();
    }
}

void UVehiclePathFollowingComponent::FollowPathSegment(float DeltaTime)
{
    if (MovementComp)
    {
        UWheeledVehicleMovementComponent* VehicleMoveComp =
            Cast<UWheeledVehicleMovementComponent>(MovementComp);

        AWheeledVehicle* Owner = Cast<AWheeledVehicle>(VehicleMoveComp->GetOwner());

        if (Owner && VehicleMoveComp)
        {

            FVector VehicleLocation = Owner->GetActorLocation();
            FVector Destination = GetCurrentTargetLocation();
            FVector DirectionToDestion = Destination - VehicleLocation;

            float DistanceToDestination = DirectionToDestion.Size();

            FRotator RotatorToDestination = FRotationMatrix::MakeFromX(DirectionToDestion.GetSafeNormal2D()).Rotator();
            float DeltaYaw = (RotatorToDestination - Owner->GetActorRotation()).Yaw;
            bool DestinationInFront = DeltaYaw - 70.0f <= EPSILON && DeltaYaw + 70.0f >= EPSILON;

            float DesiredSteering = FMath::GetMappedRangeValueClamped(FVector2D(-180.0f, 180.0f), FVector2D(-1.0f, 1.0f), DeltaYaw);

            if (!DestinationInFront && DistanceToDestination < 1e3) // reverse
                DesiredSteering = -DesiredSteering;

            VehicleMoveComp->SetHandbrakeInput(false);

            // The DesiredSteering includes the sensed steering already
            CurrentSteering += UPIDController::NextValue(HeadingController, DesiredSteering - CurrentSteering, DeltaTime);
            VehicleMoveComp->SetSteeringInput(CurrentSteering);

            float PercentDistanceLeft = DistanceToDestination / InitialDistanceToDestination;

            float DesiredThrottle = 0.0f;

            // #TODO Turn hardcoded values into variables
            if (DestinationInFront)
                DesiredThrottle = FMath::Clamp(PercentDistanceLeft, 0.35f, 0.8f);
            else
                DesiredThrottle = -0.5f; // default throttle when going in reverse

            /* #TODO Fix Math:
                Allow user to specify a maximum vehicle speed that the PID
                controller should approach.
            */
            CurrentThrottle += UPIDController::NextValue(VelocityController, DesiredThrottle - CurrentThrottle, DeltaTime);

            VehicleMoveComp->SetThrottleInput(CurrentThrottle);

#ifdef NDEBUG
            DrawDebugPoint(GetWorld(), Destination, 50.0f, FColor::Blue);
            DrawDebugLine(GetWorld(), VehicleLocation, VehicleLocation + DirectionToDestion, FColor::Yellow);

            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage(0, 1.0f, FColor::Cyan, FString::Printf(TEXT("Throttle %0.3f Steering: %0.3f"), CurrentThrottle, CurrentSteering));
                GEngine->AddOnScreenDebugMessage(1, 1.0f, FColor::Cyan, FString::Printf(TEXT("Distance to destination: %0.2f"), DistanceToDestination));
            }
#endif

        }
    }
}

void UVehiclePathFollowingComponent::UpdatePathSegment()
{
    UPathFollowingComponent::UpdatePathSegment();
#ifdef NDEBUG
    if (Path.IsValid())
    {
        for (int i = 0; i < Path->GetPathPoints().Num(); ++i)
        {
            DrawDebugPoint(GetWorld(), Path->GetPathPoints()[i].Location, 40.0f, FColor::Yellow, false, 0.3f);
        }
    }
#endif

    float DistanceLeft = (
        *Path->GetPathPointLocation(Path->GetPathPoints().Num() - 1)
        - MovementComp->GetActorLocation()
    ).Size();

    if (DistanceLeft < AcceptanceRadius)
    {
        OnSegmentFinished();
        OnPathFinished(EPathFollowingResult::Success);
    }
}

void UVehiclePathFollowingComponent::OnPathfindingQuery(FPathFindingQuery& Query)
{
    UPathFollowingComponent::OnPathfindingQuery(Query);
#ifdef NDEBUG
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(4, 3, FColor::Blue, TEXT("Custom Path finding queried"));
    }
#endif

    // #TODO Perform avoidance checks here. Alternative: try to work with RVOAvoidance or AIPerception.
}

void UVehiclePathFollowingComponent::OnPathFinished(EPathFollowingResult::Type Result)
{
#ifdef NDEBUG
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(0, 3.0f, FColor::Green, "Path Following Result: " + FString::FromInt(Result));
    }
#endif
    UWheeledVehicleMovementComponent* VehicleMoveComp =
        Cast<UWheeledVehicleMovementComponent>(MovementComp);

    InitialDistanceToDestination = -1.0f;

    VehicleMoveComp->SetThrottleInput(0.0f);
    VehicleMoveComp->SetSteeringInput(0.0f);
    VehicleMoveComp->SetHandbrakeInput(true);
    UPIDController::Clear(VelocityController);
    UPIDController::Clear(HeadingController);

    UPathFollowingComponent::OnPathFinished(Result);
}
