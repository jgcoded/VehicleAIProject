// Fill out your copyright notice in the Description page of Project Settings.

#include "VehicleAIProject.h"
#include "PIDController.h"

PIDController::PIDController()
    : P(1.0f)
    , I(1.0f)
    , D(1.0f)
    , LowerLimit(0.0f)
    , UpperLimit(10.0f)
    , Integral(0.0f)
    , PreviousError(0.0f)
{
}

PIDController::PIDController(float PTerm, float ITerm, float DTerm, float MinResult, float MaxResult)
    : P(PTerm)
    , I(ITerm)
    , D(DTerm)
    , LowerLimit(MinResult)
    , UpperLimit(MaxResult)
    , Integral(0.0f)
    , PreviousError(0.0f)
{
}

PIDController::~PIDController()
{
}

float PIDController::NextValue(float Error, float DeltaTime)
{
    float Result;

    Integral += Error * DeltaTime;

    float Derivative = (Error - PreviousError) / DeltaTime;
    PreviousError = Error;

    Result = Error      * P +
        Integral   * I +
        Derivative * D;

    return FMath::Clamp(Result, LowerLimit, UpperLimit);
}

void PIDController::Clear()
{
    Integral = 0.0f;
    PreviousError = 0.0f;
}
