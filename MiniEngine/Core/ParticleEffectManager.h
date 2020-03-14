//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
// Developed by Minigraph
//
// Author(s):  Julia Careaga
//             James Stanard
//

#pragma once

#include "ParticleEffectProperties.h"
#include "ParticleEffect.h"
#include "CommandContext.h"
#include "Math/Random.h"

namespace Math
{
    class Camera;
}

namespace ParticleEffects
{
    void Initialize( uint32_t MaxDisplayWidth, uint32_t MaxDisplayHeight );
    void Shutdown();
    void ClearAll();
    typedef uint32_t EffectHandle;
    struct EffectInstance
    {
        EffectHandle indexToPool = -1; // effect type
        uint32_t instanceID = -1; // unique ID
        uint32_t indexToActive = -1; // active pool vector index (auto deletes in Updat(...), so maybe invalid); set to -1 when instance deletes
    };
    EffectHandle PreLoadEffectResources( ParticleEffectProperties& effectProperties );
    EffectInstance InstantiateEffect( EffectHandle effectHandle );
    EffectInstance InstantiateEffect( ParticleEffectProperties& effectProperties );
    void Update(ComputeContext& Context, float timeDelta );
    void Render(CommandContext& Context, const Camera& Camera, ColorBuffer& ColorTarget, DepthBuffer& DepthTarget, ColorBuffer& LinearDepth);
    void ResetEffect(EffectInstance& EffectID);
    float GetCurrentLife(EffectInstance& EffectID);

    // Accessors
    ParticleEffect* GetEffect(EffectHandle effectType);

    // Extended functionality
    void DeActivateEffect(EffectInstance& EffectID);
    uint32_t IsActive(EffectHandle poolEffectID);
    void ClearActive();

    extern BoolVar Enable;
    extern BoolVar PauseSim;
    extern BoolVar EnableTiledRendering;
    extern bool Reproducible; //If you want to repro set to true. When true, effect uses the same set of random numbers each run
    extern UINT ReproFrame;
} // namespace ParticleEffects
namespace
{
    extern std::vector<ParticleEffect*> ParticleEffectsActive;
}