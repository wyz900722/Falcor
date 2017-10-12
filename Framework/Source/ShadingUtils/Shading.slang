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
#ifndef _FALCOR_SHADING_SLANG_
#define _FALCOR_SHADING_SLANG_
#import "HostDeviceData.h"
__import DefaultVS;
__import Lights;
__exported __import ShaderCommon;

/** This struct desribes the geometric data for a specific hit point used for lighting calculations 
*/
struct HitPointData
{
    float3    posW;         ///< Shading hit position in world space
    float3    toEye;        ///< Direction to the eye at shading hit
    float3    normal;       ///< Shading normal at shading hit
    float3    tangent;      ///< Shading tangent at shading hit
    float3    bitangent;    ///< Shading bitangent at shading hit
    float2    uv;           ///< Texture mapping coordinates

    // Pre-loaded texture data
    float4 diffuse;
    float4 specular;
    float4 emissive;
    float4 occlusion;
    float4 reflection;
    float4 bakedLight;
    float4 height;
};

/** Load data from a texture
*/
float4 sampleTexture(Texture2D t, SamplerState s, float2 uv, float4 factor, uint mode)
{
    if(mode == ChannelTypeUnused) return 0;
    if(mode == ChannelTypeConst) return factor;
    // else mode == ChannelTypeTexture
    return t.Sample(s, uv);
}

/** Shading result struct
*/
struct ShadingResult
{
    float3 diffuseIllumination;
    float3 specularIllumination;
    float3 color;
};

/** Convert RGB to normal
*/
float3 RgbToNormal(float3 rgb)
{
    float3 n = rgb * 2 - 1;
    return normalize(n);
}

/** Convert RG to normal
*/
float3 RgToNormal(float2 rg)
{
    float3 n;
    n.xy = rg;
    n.z = dot(rg, rg); // z = r*r + g*g
    n.z = sqrt(1 - n.z);
    return n;
}

/** Apply normal map
*/
void applyNormalMap(in MaterialData m, inout HitPointData hitPt)
{
    uint mapType = EXTRACT_NORMAL_MAP_TYPE(m.flags);
    if(mapType == NormalMapUnused) return;

    float3 mapN = m.textures.normalMap.Sample(m.samplerState, hitPt.uv).rgb;
    switch(mapType)
    {
    case NormalMapRGB:
        mapN = RgbToNormal(mapN);
        break;
    case NormalMapRG:
        mapN = RgToNormal(mapN.rg);
        break;
    default:
        return;
    }

    // Apply the transformation. Everything is normalized already
    hitPt.normal = hitPt.tangent * mapN.x + hitPt.bitangent * mapN.y + hitPt.normal * mapN.z;
    hitPt.bitangent = normalize(hitPt.bitangent - hitPt.normal * dot(hitPt.bitangent, hitPt.normal));
    hitPt.tangent = normalize(cross(hitPt.bitangent, hitPt.normal));
}

/** Apply alpha-test
*/
void applyAlphaTest(MaterialData m, float alpha)
{
    if(EXTRACT_ALPHA_MODE(m.flags) == AlphaModeMask)
    {
        if(alpha < m.alphaThreshold) discard;
    }
}

/** Prepare the hit-point data
*/
HitPointData prepareHitPointData(in VertexOut v, in MaterialData m, in float3 camPosW)
{
    HitPointData hitPt;
    // Sample the diffuse texture and apply the alpha test
    hitPt.diffuse = sampleTexture(m.textures.diffuse, m.samplerState, v.texC, m.diffuse, EXTRACT_DIFFUSE_TYPE(m.flags));
    applyAlphaTest(m, hitPt.diffuse.a);

    hitPt.posW = v.posW;
    hitPt.uv = v.texC;
    hitPt.toEye     = normalize(camPosW - v.posW);
    hitPt.normal    = normalize(v.normalW);
    hitPt.bitangent = normalize(v.bitangentW - hitPt.normal * (dot(v.bitangentW, hitPt.normal)));
    hitPt.tangent   = normalize(cross(hitPt.bitangent, hitPt.normal));

    // Sample the rest of the texture
    hitPt.specular = sampleTexture(m.textures.specular, m.samplerState, v.texC, m.specular, EXTRACT_SPECULAR_TYPE(m.flags));
    hitPt.emissive = sampleTexture(m.textures.emissive, m.samplerState, v.texC, float4(m.emissive, 1), EXTRACT_EMISSIVE_TYPE(m.flags));

#define channel_type(extract) (extract(m.flags) ? ChannelTypeTexture : ChannelTypeUnused)
    hitPt.occlusion = sampleTexture(m.textures.occlusionMap, m.samplerState, v.texC, 1, channel_type(EXTRACT_OCCLUSION_MAP));
    hitPt.reflection = sampleTexture(m.textures.reflectionMap, m.samplerState, v.texC, 1, channel_type(EXTRACT_REFLECTION_MAP));
    hitPt.bakedLight = sampleTexture(m.textures.lightMap, m.samplerState, v.texC, 1, channel_type(EXTRACT_LIGHT_MAP));
    hitPt.height = sampleTexture(m.textures.heightMap, m.samplerState, v.texC, 1, channel_type(EXTRACT_HEIGHT_MAP));
    hitPt.height.xy = hitPt.height.xy * m.heightScaleOffset.x + m.heightScaleOffset.y;
#undef channel_type

    applyNormalMap(m, hitPt);

    return hitPt;
}

/** Shading routines
*/

float3 evalDiffuse(in HitPointData hitPt, in MaterialData mat, in LightSample ls)
{
    float ndotl = dot(ls.dirW, hitPt.normal);
    float3 albedo = mat.textures.diffuse.Sample(mat.samplerState, hitPt.uv).rgb;
    albedo *= (1 / 3.141591);
    albedo *= ndotl * ls.intensity;
    return albedo;
}

ShadingResult evalMaterial(in HitPointData hitPt, in MaterialData mat, in LightData light, in float shadowFactor)
{
    ShadingResult sr;
    sr.diffuseIllumination = 0;
    sr.specularIllumination = 0;
    LightSample ls = evalLight(light, hitPt.posW);

    sr.color = evalDiffuse(hitPt, mat, ls);

    return sr;
};

#endif	// _FALCOR_SHADING_SLANG_
