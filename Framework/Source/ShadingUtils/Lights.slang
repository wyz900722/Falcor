/***************************************************************************
# Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************/
#ifndef _FALCOR_LIGHTS_SLANG_
#define _FALCOR_LIGHTS_SLANG_
#include "HostDeviceData.h"

struct LightSample
{
    float3 intensity;   // The light intensity at the hit point
    float3 dirW;        // The direction from the surface to the light source
    float3 posW;        // The world-space position of the light 
};

/** Evaluate a directional light source intensity/direction at a shading point
*/
LightSample evalDirectionalLight(in LightData light, in float3 hitPosW)
{
    LightSample ls;
    ls.intensity = light.intensity;
    ls.dirW = -normalize(light.dirW);
    float dist = length(hitPosW - light.posW);
    ls.posW = hitPosW - light.dirW * dist;
    return ls;
}

/** Evaluate a point light source intensity/direction at a shading point
*/
LightSample evalPointLight(in LightData light, in float3 hitPosW)
{
    LightSample ls;
    ls.posW = light.posW;
    ls.dirW = light.posW - hitPosW;
    // Avoid NaN
    float distance2 = dot(ls.dirW, ls.dirW);
    ls.dirW = (distance2 > 1e-5f) ? normalize(ls.dirW) : 0;

    // Calculate the falloff
    float falloff = 1 / (1 + distance2); // The 1 is to avoid infs when the light source is close to the shading point
    // Calculate the falloff for spot-lights
    float cosTheta = -dot(ls.dirW, light.dirW); // cos of angle of light orientation
    if(cosTheta < light.cosOpeningAngle)
    {
        falloff = 0;
    }
    else if(light.penumbraAngle > 0)
    {
        float deltaAngle = light.openingAngle - acos(cosTheta);
        falloff *= saturate((deltaAngle - light.penumbraAngle) / light.penumbraAngle);
    }
    ls.intensity = light.intensity * falloff;
    return ls;
}

/** Evaluate a area light source intensity/direction at a shading point
*/
LightSample evalAreaLight(in LightData light, in float3 hitPosW)
{
    LightSample ls;
    ls.posW = mul(float4(light.posW, 1.0), light.transMat).xyz;
    ls.dirW = light.posW - hitPosW;
    float distance2 = dot(ls.dirW, ls.dirW);
    ls.dirW = (distance2 > 1e-5f) ? normalize(ls.dirW) : 0;

    // Calculate the falloff
    float cosTheta = -dot(ls.dirW, light.dirW); // cos of angle of light orientation 
    float falloff = max(0.f, cosTheta) * light.surfaceArea;
    falloff *= 1 / (1 + distance2); // The 1 is to avoid infs when the light source is close to the shading point
    ls.intensity = falloff * light.intensity;
    return ls;
}

/** Evaluate a light source intensity/direction at a shading point
*/
LightSample evalLight(in LightData light, in float3 hitPosW)
{
    if(light.type == LightDirectional) return evalDirectionalLight(light, hitPosW);
    else if(light.type == LightPoint)  return evalPointLight(light, hitPosW);
    else return evalAreaLight(light, hitPosW);
};

#endif	// _FALCOR_LIGHTS_SLANG_
