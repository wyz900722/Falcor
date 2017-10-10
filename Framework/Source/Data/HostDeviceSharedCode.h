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
#ifndef _HOST_DEVICE_SHARED_CODE_H
#define _HOST_DEVICE_SHARED_CODE_H

#include "HostDeviceSharedMacros.h"

#ifdef HOST_CODE

#include "glm/gtx/compatibility.hpp"

using glm::float2;
using glm::float3;
using glm::float4;

using glm::float2x2;
using glm::float3x3;
using glm::float4x4;

namespace Falcor {
/*******************************************************************
                    CPU declarations
*******************************************************************/
    class Sampler;
    class Texture;


#elif defined(CUDA_CODE)
/*******************************************************************
                    CUDA declarations
*******************************************************************/
typedef float float4x4_t[16];
typedef float mat3_t [12];
_fn float clamp(float t, float mn, float mx) { return fminf(mx, fmaxf(mn, t)); }
_fn float2 clamp(float2 t, float2 mn, float2 mx) { return fminf(mx, fmaxf(mn, t)); }
_fn float dot(const float2& a, const float2& b) { return a.x*b.x + a.y*b.y; }
_fn float dot(const float3& a, const float3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
_fn float dot(const float4& a, const float4& b) { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
_fn float3 cross(const float3& x, const float3& y) {
    return float3(
        x.y * y.z - y.y * x.z,
        x.z * y.x - y.z * x.x,
        x.x * y.y - y.x * x.y);
}
_fn float length(const float3& a) { return sqrt(dot(a, a)); }
_fn float length(const float2& a) { return sqrt(dot(a, a)); }
_fn float3 normalize(const float3& a) { return a / length(a); }
_fn float3 mix(const float3& a, const float3& b, const float w) { return a + w * (b - a); }
_fn float2 sqrt(const float2& a) { return float2(sqrt(a.x), sqrt(a.y)); }
// Texture access
_fn bool isSamplerBound(sampler2D sampler) { return sampler > 0; }
_fn float4 texture2D(sampler2D sampler, float2 uv) { return tex2D<float4>(sampler, uv.x, uv.y); }
_fn float4 textureLod(sampler2D sampler, float2 uv, float lod) { return tex2DLod<float4>(sampler, uv.x, uv.y, lod); }
_fn float4 textureGrad(sampler2D sampler, float2 uv, float2 dPdx, float2 dPdy) { return tex2DGrad<float4>(sampler, uv.x, uv.y, dPdx, dPdy); }
_fn float4 textureBias(sampler2D sampler, float2 uv, float bias) { return texture2D(sampler, uv); }
struct TexPtr
{
    int            ptr;
    uint        pad[7];
};
typedef TexPtr BufPtr;
#else
/*******************************************************************
                    HLSL declarations
*******************************************************************/
typedef uint uint32_t;
typedef int int32_t;
#endif


/*******************************************************************
Camera
*******************************************************************/
/**
This is a general host/device structure that describe a camera.
*/
struct CameraData
{
    float4x4 viewMat                DEFAULTS(float4x4());       ///< Camera view matrix.
    float4x4 projMat                DEFAULTS(float4x4());       ///< Camera projection matrix.
    float4x4 viewProjMat            DEFAULTS(float4x4());       ///< Camera view-projection matrix.
    float4x4 invViewProj            DEFAULTS(float4x4());       ///< Camera inverse view-projection matrix.
    float4x4 prevViewProjMat        DEFAULTS(float4x4());       ///< Camera view-projection matrix associated to previous frame.

    float3   position               DEFAULTS(float3(0, 0, 0));  ///< Camera world-space position.
    float    focalLength            DEFAULTS(21.0f);            ///< Camera focal length in mm. Default is 59 degree vertical, 90 horizontal FOV at 16:9 aspect ratio.
    float3   up                     DEFAULTS(float3(0, 1, 0));  ///< Camera world-space up vector.
    float    aspectRatio            DEFAULTS(1.f);              ///< Camera aspect ratio.
    float3   target                 DEFAULTS(float3(0, 0, -1)); ///< Camera target point in world-space.
    float    nearZ                  DEFAULTS(0.1f);             ///< Camera near plane.
    float3   cameraU                DEFAULTS(float3(0, 0, 1));  ///< Camera base vector U. normalized it indicates the left image plane vector. The length is dependent on the FOV. 
    float    farZ                   DEFAULTS(10000.0f);         ///< Camera far plane.
    float3   cameraV                DEFAULTS(float3(0, 1, 0));  ///< Camera base vector V. normalized it indicates the up image plane vector. The length is dependent on the FOV. 
    float    jitterX                DEFAULTS(0.0f);             ///< Eventual camera jitter in the x coordinate
    float3   cameraW                DEFAULTS(float3(1, 0, 0));  ///< Camera base vector U. normalized it indicates the forward direction. The length is the camera focal distance.
    float    jitterY                DEFAULTS(0.0f);             ///< Eventual camera jitter in the y coordinate

    float4x4 rightEyeViewMat;
    float4x4 rightEyeProjMat;
    float4x4 rightEyeViewProjMat;
    float4x4 rightEyePrevViewProjMat;
};

/*******************************************************************
                    Material
*******************************************************************/
// Shading model
#define ShadingModelSpecGloss 0

// Channel type
#define ChannelTypeUnused    0
#define ChannelTypeConst     1
#define ChannelTypeTexture   2

// Normal map type
#define NormalMapUnused     0
#define NormalMapRGB        1
#define NormalMapRG         2

// Alpha mode
#define AlphaModeOpaque      0
#define AlphaModeTransparent 1
#define AlphaModeMask        2

// NDF type
#define NdfGGX      0
#define NdfBeckmann 1

// Bit count
#define SHADING_MODEL_BITS   (3)
#define DIFFUSE_TYPE_BITS    (3)
#define SPECULAR_TYPE_BITS   (3)
#define EMISSIVE_TYPE_BITS   (3)
#define NORMAL_MAP_BITS      (2)
#define OCCLUSION_MAP_BITS   (1)
#define REFLECTION_MAP_BITS  (1)
#define LIGHT_MAP_BITS       (1)
#define HEIGHT_MAP_BITS      (1)
#define ALPHA_MODE_BITS      (2)
#define DOUBLE_SIDED_BITS    (1)
#define NDF_BITS             (3)

// Offsets
#define SHADING_MODEL_OFFSET (0)
#define DIFFUSE_TYPE_OFFSET  (SHADING_MODEL_OFFSET + SHADING_MODEL_BITS)
#define SPECULAR_TYPE_OFFSET (DIFFUSE_TYPE_OFFSET  + DIFFUSE_TYPE_BITS)
#define EMISSIVE_TYPE_OFFSET (SPECULAR_TYPE_OFFSET + SPECULAR_TYPE_BITS)
#define NORMAL_MAP_OFFSET    (EMISSIVE_TYPE_OFFSET + EMISSIVE_TYPE_BITS)
#define OCCLUSION_MAP_OFFSET (NORMAL_MAP_OFFSET    + NORMAL_MAP_BITS)
#define REFLECTION_MAP_OFFSET (OCCLUSION_MAP_OFFSET+ OCCLUSION_MAP_BITS)
#define LIGHT_MAP_OFFSET     (REFLECTION_MAP_OFFSET+ REFLECTION_MAP_BITS)
#define HEIGHT_MAP_OFFSET    (LIGHT_MAP_OFFSET     + LIGHT_MAP_BITS)
#define ALPHA_MODE_OFFSET    (HEIGHT_MAP_OFFSET    + HEIGHT_MAP_BITS)
#define DOUBLE_SIDED_OFFSET  (ALPHA_MODE_OFFSET    + ALPHA_MODE_BITS)
#define NDF_OFFSET           (DOUBLE_SIDED_OFFSET  + DOUBLE_SIDED_BITS)

// Extract bits
#define EXTRACT_BITS(bits, offset, value) ((value >> offset) & ((1 << bits) - 1))
#define EXTRACT_SHADING_MODEL(value)    EXTRACT_BITS(SHADING_MODEL_BITS,    SHADING_MODEL_OFFSET,   value)
#define EXTRACT_DIFFUSE_TYPE(value)     EXTRACT_BITS(DIFFUSE_TYPE_BITS,     DIFFUSE_TYPE_OFFSET,    value)
#define EXTRACT_SPECULAR_TYPE(value)    EXTRACT_BITS(SPECULAR_TYPE_BITS,    SPECULAR_TYPE_OFFSET,   value)
#define EXTRACT_EMISSIVE_TYPE(value)    EXTRACT_BITS(EMISSIVE_TYPE_BITS,    EMISSIVE_TYPE_OFFSET,   value)
#define EXTRACT_NORMAL_MAP_TYPE(value)  EXTRACT_BITS(NORMAL_MAP_BITS,       NORMAL_MAP_OFFSET,      value)
#define EXTRACT_OCCLUSION_MAP(value)    EXTRACT_BITS(OCCLUSION_MAP_BITS,    OCCLUSION_MAP_OFFSET,   value)
#define EXTRACT_REFLECTION_MAP (value)  EXTRACT_BITS(REFLECTION_MAP_BITS,   REFLECTION_MAP_OFFSET,  value)
#define EXTRACT_LIGHT_MAP(value)        EXTRACT_BITS(LIGHT_MAP_BITS,        LIGHT_MAP_OFFSET,       value)  
#define EXTRACT_HEIGHT_MAP(value)       EXTRACT_BITS(HEIGHT_MAP_BITS,       HEIGHT_MAP_OFFSET,      value)
#define EXTRACT_ALPHA_MODE(value)       EXTRACT_BITS(ALPHA_MODE_BITS,       ALPHA_MODE_OFFSET,      value)
#define EXTRACT_DOUBLE_SIDED(value)     EXTRACT_BITS(DOUBLE_SIDED_BITS,     DOUBLE_SIDED_OFFSET,    value)
#define EXTRACT_NDF_TYPE(value)         EXTRACT_BITS(NDF_BITS,              NDF_OFFSET,             value)

// Pack bits
#define PACK_BITS(bits, offset, flags, value) (((value & ((1 << bits) - 1)) << offset) | (flags & (~(((1 << bits) - 1) << offset))))
#define PACK_SHADING_MODEL(flags, value)    PACK_BITS(SHADING_MODEL_BITS,    SHADING_MODEL_OFFSET,   flags, value)
#define PACK_DIFFUSE_TYPE(flags, value)     PACK_BITS(DIFFUSE_TYPE_BITS,     DIFFUSE_TYPE_OFFSET,    flags, value)
#define PACK_SPECULAR_TYPE(flags, value)    PACK_BITS(SPECULAR_TYPE_BITS,    SPECULAR_TYPE_OFFSET,   flags, value)
#define PACK_EMISSIVE_TYPE(flags, value)    PACK_BITS(EMISSIVE_TYPE_BITS,    EMISSIVE_TYPE_OFFSET,   flags, value)
#define PACK_NORMAL_MAP_TYPE(flags, value)  PACK_BITS(NORMAL_MAP_BITS,       NORMAL_MAP_OFFSET,      flags, value)
#define PACK_OCCLUSION_MAP(flags, value)    PACK_BITS(OCCLUSION_MAP_BITS,    OCCLUSION_MAP_OFFSET,   flags, value)
#define PACK_REFLECTION_MAP(flags, value)   PACK_BITS(REFLECTION_MAP_BITS,   REFLECTION_MAP_OFFSET,  flags, value)
#define PACK_LIGHT_MAP(flags, value)        PACK_BITS(LIGHT_MAP_BITS,        LIGHT_MAP_OFFSET,       flags, value)
#define PACK_HEIGHT_MAP(flags, value)       PACK_BITS(HEIGHT_MAP_BITS,       HEIGHT_MAP_OFFSET,      flags, value)
#define PACK_ALPHA_MODE(flags, value)       PACK_BITS(ALPHA_MODE_BITS,       ALPHA_MODE_OFFSET,      flags, value)
#define PACK_DOUBLE_SIDED(flags, value)     PACK_BITS(DOUBLE_SIDED_BITS,     DOUBLE_SIDED_OFFSET,    flags, value)
#define PACK_NDF_TYPE(flags, value)         PACK_BITS(NDF_BITS,              NDF_OFFSET,             flags, value)

struct MaterialTextures
{
    Texture2D diffuse;          // RGB - diffuse color, A - transparency
    Texture2D specular;         // RGB - specular color, A - roughness
    Texture2D emissive;         // RGB - emissive color, A - unused
    Texture2D normalMap;        // 2 or 3 channel normal map, depending on the type
    Texture2D occlusionMap;     // Ambient occlusion map
    Texture2D reflectionMap;    // Reflection map
    Texture2D lightMap;         // Light map
    Texture2D heightMap;        // Height map. Not used by the default material system
};

struct MaterialData
{
    vec4 diffuse  DEFAULTS(vec4(1));
    vec4 specular DEFAULTS(vec4(1));
    vec3 emissive DEFAULTS(vec3(1));
    float padf    DEFAULTS(0);

    float alphaThreshold DEFAULTS(0.5f); // Used in case the alpha mode is mask
    float IoR DEFAULTS(1);               // Index of refraction
    uint32_t id;
    uint32_t flags DEFAULTS(0);

    vec2 heightScaleOffset  DEFAULTS(vec2(1, 0));
    vec2 pad                DEFAULTS(vec2(0));

    MaterialTextures textures;
};

/*******************************************************************
                    Lights
*******************************************************************/

/**
    This is a general host/device structure that describe a light source.
*/
struct LightData
{
    float3   worldPos           DEFAULTS(float3(0, 0, 0));  ///< World-space position of the center of a light source
    uint32_t type               DEFAULTS(LightPoint);       ///< Type of the light source (see above)
    float3   worldDir           DEFAULTS(float3(0, -1, 0)); ///< World-space orientation of the light source
    float    openingAngle       DEFAULTS(3.14159265f);      ///< For point (spot) light: Opening angle of a spot light cut-off, pi by default - full-sphere point light
    float3   intensity          DEFAULTS(float3(1, 1, 1));  ///< Emitted radiance of th light source
    float    cosOpeningAngle    DEFAULTS(-1.f);             ///< For point (spot) light: cos(openingAngle), -1 by default because openingAngle is pi by default
    float3   aabbMin            DEFAULTS(float3(1e20f));    ///< For area light: minimum corner of the AABB
    float    penumbraAngle      DEFAULTS(0.f);              ///< For point (spot) light: Opening angle of penumbra region in radians, usually does not exceed openingAngle. 0.f by default, meaning a spot light with hard cut-off
    float3   aabbMax            DEFAULTS(float3(-1e20f));   ///< For area light: maximum corner of the AABB
    float    surfaceArea        DEFAULTS(0.f);              ///< Surface area of the geometry mesh
	float3   tangent            DEFAULTS(float3());         ///< Tangent vector of the geometry mesh
	uint32_t numIndices         DEFAULTS(0);                ///< Number of triangle indices in a polygonal area light
	float3   bitangent          DEFAULTS(float3());         ///< BiTangent vector of the geometry mesh
	float    pad;
    float4x4 transMat           DEFAULTS(float4x4());       ///< Transformation matrix of the model instance for area lights

    // For area light
// 	BufPtr          indexPtr;                                     ///< Buffer id for indices
// 	BufPtr          vertexPtr;                                    ///< Buffer id for vertices
// 	BufPtr          texCoordPtr;                                  ///< Buffer id for texcoord
// 	BufPtr          meshCDFPtr;                                   ///< Pointer to probability distributions of triangle meshes

    /*TODO(tfoley) HACK: Slang can't hanlde this
    // Keep that last
    MaterialData    material;                                     ///< Emissive material of the geometry mesh
    */
};

/*******************************************************************
                    Shared material routines
*******************************************************************/


/** Converts specular power to roughness. Note there is no "the conversion".
    Reference: http://simonstechblog.blogspot.com/2011/12/microfacet-brdf.html
    \param shininess specular power of an obsolete Phong BSDF
*/
inline float _fn convertShininessToRoughness(const float shininess)
{
    return clamp(sqrt(2.0f / (shininess + 2.0f)), 0.f, 1.f);
}

inline float2 _fn convertShininessToRoughness(const float2 shininess)
{
    return clamp(sqrt(2.0f / (shininess + 2.0f)), 0.f, 1.f);
}

inline float _fn convertRoughnessToShininess(const float a)
{
    return 2.0f / clamp(a*a, 1e-8f, 1.f) - 2.0f;
}

inline float2 _fn convertRoughnessToShininess(const float2 a)
{
    return 2.0f / clamp(a*a, 1e-8f, 1.f) - 2.0f;
}

/*******************************************************************
Other helpful shared routines
*******************************************************************/


/** Returns a relative luminance of an input linear RGB color in the ITU-R BT.709 color space
\param RGBColor linear HDR RGB color in the ITU-R BT.709 color space
*/
inline float _fn luminance(const float3 rgb)
{
    return dot(rgb, float3(0.2126f, 0.7152f, 0.0722f));
}

/** Converts color from RGB to YCgCo space
\param RGBColor linear HDR RGB color
*/
inline float3 _fn RGBToYCgCo(const float3 rgb)
{
    const float Y = dot(rgb, float3(0.25f, 0.50f, 0.25f));
    const float Cg = dot(rgb, float3(-0.25f, 0.50f, -0.25f));
    const float Co = dot(rgb, float3(0.50f, 0.00f, -0.50f));

    return float3(Y, Cg, Co);
}

/** Converts color from YCgCo to RGB space
\param YCgCoColor linear HDR YCgCo color
*/
inline float3 _fn YCgCoToRGB(const float3 YCgCo)
{
    const float tmp = YCgCo.x - YCgCo.y;
    const float r = tmp + YCgCo.z;
    const float g = YCgCo.x + YCgCo.y;
    const float b = tmp - YCgCo.z;

    return float3(r, g, b);
}

/** Returns a YUV version of an input linear RGB color in the ITU-R BT.709 color space
\param RGBColor linear HDR RGB color in the ITU-R BT.709 color space
*/
inline float3 _fn RGBToYUV(const float3 rgb)
{
    float3 ret;

    ret.x = dot(rgb, float3(0.2126f, 0.7152f, 0.0722f));
    ret.y = dot(rgb, float3(-0.09991f, -0.33609f, 0.436f));
    ret.z = dot(rgb, float3(0.615f, -0.55861f, -0.05639f));

    return ret;
}

/** Returns a RGB version of an input linear YUV color in the ITU-R BT.709 color space
\param YUVColor linear HDR YUV color in the ITU-R BT.709 color space
*/
inline float3 _fn YUVToRGB(const float3 yuv)
{
    float3 ret;

    ret.x = dot(yuv, float3(1.0f, 0.0f, 1.28033f));
    ret.y = dot(yuv, float3(1.0f, -0.21482f, -0.38059f));
    ret.z = dot(yuv, float3(1.0f, 2.12798f, 0.0f));

    return ret;
}

/** Returns a linear-space RGB version of an input RGB channel value in the ITU-R BT.709 color space
\param sRGBColor sRGB input channel value
*/
inline float _fn SRGBToLinear(const float srgb)
{
    if (srgb <= 0.04045f)
    {
        return srgb * (1.0f / 12.92f);
    }
    else
    {
        return pow((srgb + 0.055f) * (1.0f / 1.055f), 2.4f);
    }
}

/** Returns a linear-space RGB version of an input RGB color in the ITU-R BT.709 color space
\param sRGBColor sRGB input color
*/
inline float3 _fn SRGBToLinear(const float3 srgb)
{
    return float3(
        SRGBToLinear(srgb.x),
        SRGBToLinear(srgb.y),
        SRGBToLinear(srgb.z));
}

/** Returns a sRGB version of an input linear RGB channel value in the ITU-R BT.709 color space
\param LinearColor linear input channel value
*/
inline float _fn LinearToSRGB(const float lin)
{
    if (lin <= 0.0031308f)
    {
        return lin * 12.92f;
    }
    else
    {
        return pow(lin, (1.0f / 2.4f)) * (1.055f) - 0.055f;
    }
}

/** Returns a sRGB version of an input linear RGB color in the ITU-R BT.709 color space
\param LinearColor linear input color
*/
inline float3 _fn LinearToSRGB(const float3 lin)
{
    return float3(
        LinearToSRGB(lin.x),
        LinearToSRGB(lin.y),
        LinearToSRGB(lin.z));
}


/** Returns Michelson contrast given minimum and maximum intensities of an image region
\param Imin minimum intensity of an image region
\param Imax maximum intensity of an image region
*/
inline float _fn computeMichelsonContrast(const float iMin, const float iMax)
{
    if (iMin == 0.0f && iMax == 0.0f) return 0.0f;
    else return (iMax - iMin) / (iMax + iMin);
}

struct DrawArguments
{
    uint vertexCountPerInstance;
    uint instanceCount;
    uint startVertexLocation;
    uint startInstanceLocation;
};

struct DrawIndexedArguments
{
    uint indexCountPerInstance;
    uint instanceCount;
    uint startIndexLocation;
    int baseVertexLocation;
    uint startInstanceLocation;
};

struct DispatchArguments
{
    uint threadGroupCountX;
    uint threadGroupCountY;
    uint threadGroupCountZ;
};

#ifdef HOST_CODE
#undef SamplerState
#undef Texture2D
} // namespace Falcor
#endif // HOST_CODE

#endif //_HOST_DEVICE_SHARED_CODE_H
