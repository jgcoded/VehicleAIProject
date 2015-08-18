// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

/**
*
*/
class VEHICLEAIPROJECT_API PIDController
{
public:

    PIDController();

    PIDController(float PTerm, float ITerm, float DTerm, float MinResult, float MaxResult);
    ~PIDController();

    float NextValue(float Error, float DeltaTime);

    void Clear();

private:

    float P;
    float I;
    float D;

    float Integral;
    float PreviousError;

    float UpperLimit;
    float LowerLimit;

};
