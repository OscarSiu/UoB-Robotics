// Actor: measure & simulate data
// Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/CalyoPulse.h

// This sensor simulates an ultrasonic 3D sensor based on ray-casting
// with Rayleigh & Mie scattering, Doppler effect, and TOF modelling.

#pragma once

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/CalyoPulseData.h>
#include <compiler/enable-ue4-macros.h>

#include "CalyoPulse.generated.h"

// A raycast based 3D ultrasonic sensor
UCLASS()
// Declare class and inherit from ASensor
class CARLA_API ACalyoPulse : public ASensor
{
    GENERATED_BODY()

    using FCalyoPulseData = carla::sensor::data::CalyoPulseData;

public:
    
    static FActorDefinition GetSensorDefinition(); //Define sensor properties

    ACalyoPulse(const FObjectInitializer &ObjectInitializer); // Initialize sensor settings

    void Set(const FActorDescription &ActorDescription) override; // Configure sensor based on user-defined properties

    // Adjustable Parameters (Attributes)
    UFUNCTION(BlueprintCallable, Category = "Ultrasonic")
    void SetHorizontalFOV(float NewHorizontalFOV);

    UFUNCTION(BlueprintCallable, Category = "Ultrasonic")
    void SetVerticalFOV(float NewVerticalFOV);

    UFUNCTION(BlueprintCallable, Category = "Ultrasonic")
    void SetRange(float NewRange);

    UFUNCTION(BlueprintCallable, Category = "Ultrasonic")
    void SetPointsPerSecond(int NewPointsPerSecond);

    UFUNCTION(BlueprintCallable, Category = "Ultrasonic")
    void SetAtmosphereAttenuationRate(float NewAttenuationRate);
    

protected:

  void BeginPlay() override; // call when the sensor starts

  // virtual void PrePhysTick(float DeltaTime) override;
  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override; // Runs after the physics update to process data

  // Sensor parameters
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Detection")
  float Range;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Detection")
  float HorizontalFOV;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Detection")
  float VerticalFOV;

  UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Detection")
  int PointsPerSecond;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Detection")
  float AtmospAttenRate;


// Define Private Methods
private:

  void CalculateCurrentVelocity(const float DeltaTime); //Determine movement speed of radar

  void SendLineTraces(float DeltaTime); // Shoots ray traces to detect objects
  float CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& CalyoPulseLocation); //compute relative velocity of detected objects
  
  //CUSTOM FUNCTION
  float CalculateIntensity(const FHitResult& OutHit, const FVector& CalyoSensorLocation);
  



  FCalyoPulseData CalyoPulseData;

  FCollisionQueryParams TraceParams;

  FVector CurrentVelocity;

  /// Used to compute the velocity of the radar
  FVector PrevLocation;

  struct RayData {
    float Radius;
    float Angle;
    bool Hitted;
    float RelativeVelocity;
    FVector2D AzimuthAndElevation;
    float Distance;

    // 'OSCAR CUSTOMIZATION'
    float Intensity;
    // float AttenuationFactor;
  };

  
  
  // Declare output array
  std::vector<RayData> Rays;
  //std::vector<CalyoPulseData> Rays;
};