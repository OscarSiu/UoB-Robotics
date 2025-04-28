// Actor: measure & simulate data
// Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/CalyoPulse.cpp

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include "Carla/Sensor/CalyoPulse.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Location.h"
// #include "carla/ros2/ROS2.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Engine/CollisionProfile.h"

FActorDefinition ACalyoPulse::GetSensorDefinition()
{

  return UActorBlueprintFunctionLibrary::MakeCalyoPulseDefinition();
}

ACalyoPulse::ACalyoPulse(const FObjectInitializer &ObjectInitializer)
    : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  // SetSeed(Description.RandomSeed);

  TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;
}

void ACalyoPulse::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetCalyoPulse(ActorDescription, this);
}

void ACalyoPulse::SetHorizontalFOV(float NewHorizontalFOV)
{
  HorizontalFOV = NewHorizontalFOV;
}

void ACalyoPulse::SetVerticalFOV(float NewVerticalFOV)
{
  VerticalFOV = NewVerticalFOV;
}

void ACalyoPulse::SetRange(float NewRange)
{
  Range = NewRange;
}

void ACalyoPulse::SetPointsPerSecond(int NewPointsPerSecond)
{
  PointsPerSecond = NewPointsPerSecond;
  CalyoPulseData.SetResolution(PointsPerSecond);
}

// void ACalyoPulse::SetChannels(int NewChannels)
// {
//     Channels = NewChannels;
// }

void ACalyoPulse::SetAtmosphereAttenuationRate(float NewAttenuationRate)
{
  AtmospAttenRate = NewAttenuationRate;
}

void ACalyoPulse::BeginPlay()
{
  Super::BeginPlay();
  PrevLocation = GetActorLocation();
}

void ACalyoPulse::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACalyoPulse::PostPhysTick);
  CalculateCurrentVelocity(DeltaTime);

  CalyoPulseData.Reset();
  SendLineTraces(DeltaTime);

  auto DataStream = GetDataStream(*this);

  //   // ROS2
  //   #if defined(WITH_ROS2)
  //   auto ROS2 = carla::ros2::ROS2::GetInstance();
  //   if (ROS2->IsEnabled())
  //   {
  //     TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
  //     auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
  //     AActor* ParentActor = GetAttachParentActor();
  //     if (ParentActor)
  //     {
  //       FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
  //       ROS2->ProcessDataFromRadar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, RadarData, this);
  //     }
  //     else
  //     {
  //       ROS2->ProcessDataFromRadar(DataStream.GetSensorType(), StreamId, DataStream.GetSensorTransform(), RadarData, this);
  //     }
  //   }
  //   #endif

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    DataStream.SerializeAndSend(*this, CalyoPulseData, DataStream.PopBufferFromPool());
  }
}

void ACalyoPulse::CalculateCurrentVelocity(const float DeltaTime)
{
  const FVector CalyoPulseLocation = GetActorLocation();
  CurrentVelocity = (CalyoPulseLocation - PrevLocation) / DeltaTime;
  PrevLocation = CalyoPulseLocation;
}

void ACalyoPulse::SendLineTraces(float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACalyoPulse::SendLineTraces);
  constexpr float TO_METERS = 1e-2;
  // constexpr float SPEED_OF_SOUND = 343.0f; // Speed of sound in m/s (at 20°C)

  const FTransform &ActorTransform = GetActorTransform();
  const FRotator &TransformRotator = ActorTransform.Rotator();
  const FVector &CalyoPulseLocation = GetActorLocation();
  const FVector &ForwardVector = GetActorForwardVector();
  const FVector TransformXAxis = ActorTransform.GetUnitAxis(EAxis::X);
  const FVector TransformYAxis = ActorTransform.GetUnitAxis(EAxis::Y);
  const FVector TransformZAxis = ActorTransform.GetUnitAxis(EAxis::Z);

  // Maximum radar radius in horizontal and vertical direction
  const float MaxRx = FMath::Tan(FMath::DegreesToRadians(HorizontalFOV * 0.5f)) * Range;
  const float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f)) * Range;
  const int NumPoints = (int)(PointsPerSecond * DeltaTime);

  // Generate the parameters of the rays in a deterministic way
  Rays.clear();
  Rays.resize(NumPoints);
  for (int i = 0; i < Rays.size(); i++)
  {
    Rays[i].Radius = RandomEngine->GetUniformFloat();
    Rays[i].Angle = RandomEngine->GetUniformFloatInRange(0.0f, carla::geom::Math::Pi2<float>());
    Rays[i].Hitted = false;
  }

  FCriticalSection Mutex;
  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(NumPoints, [&](int32 idx)
                {
            TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);
            FHitResult OutHit(ForceInit);
            const float Radius = Rays[idx].Radius;
            const float Angle  = Rays[idx].Angle;

            float Sin, Cos;
            FMath::SinCos(&Sin, &Cos, Angle);

            const FVector EndLocation = CalyoPulseLocation + TransformRotator.RotateVector({
              Range,
              MaxRx * Radius * Cos,
              MaxRy * Radius * Sin
            });

            const bool Hitted = GetWorld()->LineTraceSingleByChannel(
                OutHit,
                CalyoPulseLocation,
                EndLocation,
                ECC_GameTraceChannel2,
                TraceParams,
                FCollisionResponseParams::DefaultResponseParam
            );

            const TWeakObjectPtr<AActor> HittedActor = OutHit.Actor;
            if (Hitted && HittedActor.Get()) {
                Rays[idx].Hitted = true;
                Rays[idx].RelativeVelocity = CalculateRelativeVelocity(OutHit, CalyoPulseLocation);
                
                Rays[idx].Intensity = CalculateIntensity(OutHit, CalyoPulseLocation);

                Rays[idx].AzimuthAndElevation = FMath::GetAzimuthAndElevation(
                    (EndLocation - CalyoPulseLocation).GetSafeNormal() * Range,
                    TransformXAxis,
                    TransformYAxis,
                    TransformZAxis
                );
                Rays[idx].Distance = OutHit.Distance * TO_METERS;

            } });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  for (auto &ray : Rays)
  {
    if (ray.Hitted)
    {
      CalyoPulseData.WriteDetection({
          ray.RelativeVelocity,
          ray.AzimuthAndElevation.X,
          ray.AzimuthAndElevation.Y,
          ray.Distance,
          ray.Intensity,
      });
    }
  }
}

// custom
float ACalyoPulse::CalculateRelativeVelocity(const FHitResult &OutHit, const FVector &CalyoPulseLocation)
{
  constexpr float TO_METERS = 1e-2;

  const TWeakObjectPtr<AActor> HittedActor = OutHit.Actor;
  const FVector TargetVelocity = HittedActor->GetVelocity();
  const FVector TargetLocation = OutHit.ImpactPoint;
  const FVector Direction = (TargetLocation - CalyoPulseLocation).GetSafeNormal();
  const FVector DeltaVelocity = (TargetVelocity - CurrentVelocity);
  const float V = TO_METERS * FVector::DotProduct(DeltaVelocity, Direction);

  return V;
}

float ACalyoPulse::CalculateIntensity(const FHitResult &OutHit, const FVector &CalyoPulseLocation)
{
  constexpr float TO_METERS = 1e-2;
  constexpr float US_ATTENUATION = 0.208f; // Attenuation rate in dB/m

  const FVector HitPointLocation = OutHit.ImpactPoint;
  const float Distance = (HitPointLocation - CalyoPulseLocation).Size();
  const float DistanceToImpact = Distance * TO_METERS;
  const float I_attenuated = FMath::Exp(-US_ATTENUATION * DistanceToImpact);
  // FMath::Clamp(Intensity, 0.0f, 1.0f);

  return I_attenuated;
}
