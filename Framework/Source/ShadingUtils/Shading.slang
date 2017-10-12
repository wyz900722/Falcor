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
#ifndef _FALCOR_SHADING_H_
#define _FALCOR_SHADING_H_
#import "HostDeviceData.h"
__import DefaultVS;

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
void applyAlphaTest(MaterialData m, float2 texC)
{
    if(EXTRACT_ALPHA_MODE(m.flags) == AlphaModeMask)
    {
        float alpha = m.textures.diffuse.Sample(m.samplerState, texC).a;
        if(alpha < m.alphaThreshold) discard;
    }
}

HitPointData prepareHitPointData(in VertexOut v, in MaterialData m, in float3 camPosW)
{
    // Apply alpha test first
    applyAlphaTest(m, v.texC);

    HitPointData hitPt;
    hitPt.posW = v.posW;
    hitPt.uv = v.texC;
    hitPt.toEye     = normalize(camPosW - v.posW);
    hitPt.normal    = normalize(v.normalW);
    hitPt.bitangent = normalize(v.bitangentW - hitPt.normal * (dot(v.bitangentW, hitPt.normal)));
    hitPt.tangent   = normalize(cross(hitPt.bitangent, hitPt.normal));

    applyNormalMap(m, hitPt);

    return hitPt;
}

#endif	// _FALCOR_SHADING_H_
