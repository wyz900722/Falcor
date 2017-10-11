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
#ifndef _HOST_DEVICE_SHARED_MACROS_H
#define _HOST_DEVICE_SHARED_MACROS_H

/*******************************************************************
                    Common structures & routines
*******************************************************************/


/*******************************************************************
                    Glue code for CPU/GPU compilation
*******************************************************************/

#if (defined(__STDC_HOSTED__) || defined(__cplusplus)) && !defined(__CUDACC__)    // we're in C-compliant compiler, probably host
#    define HOST_CODE 1
#elif defined(__CUDACC__)
#   define CUDA_CODE
#else
#   define HLSL_CODE
#define FALCOR_SHADER_CODE
#endif

#ifdef HLSL_CODE
//#extension GL_NV_shader_buffer_load : enable
#endif

#ifdef HOST_CODE

/*******************************************************************
                    CPU declarations
*******************************************************************/
#define loop_unroll
#define v2 vec2
#define v3 vec3
#define v4 vec4
#define _fn
#define DEFAULTS(x_) = ##x_
#define SamplerState std::shared_ptr<Sampler>
#define Texture2D std::shared_ptr<Texture>
#elif defined(CUDA_CODE)
/*******************************************************************
                    CUDA declarations
*******************************************************************/
// Modifiers
#define DEFAULTS(x_)
#define in
#define out &
#define _ref(__x) __x&
#define discard
#define sampler2D int
#define inline __forceinline
#define _fn inline __device__
// Types
#define int32_t int
#define uint unsigned int
#define uint32_t uint
// Vector math
#define vec2 float2
#define vec3 float3
#define vec4 float4
#ifndef mat4
#define mat4 mat4_t
#endif
#ifndef mat3
#define mat3 mat3_t
#endif
#define mul(mx, v) ((v) * (mx))
#define v2 make_float2
#define v3 make_float3
#define v4 make_float4
#else
/*******************************************************************
                    HLSL declarations
*******************************************************************/
#define loop_unroll [unroll]
#define _fn 
#define __device__ 
#define inline 
#define _ref(__x) inout __x
#define DEFAULTS(x_)
#endif

/*******************************************************************
                    Lights
*******************************************************************/

/**
    Types of light sources. Used in LightData structure.
*/
#define LightPoint           0    ///< Point light source, can be a spot light if its opening angle is < 2pi
#define LightDirectional     1    ///< Directional light source
#define LightArea            2    ///< Area light source, potentially with arbitrary geometry
//#define LightVolume        3    ///< Volumetric light source

#define MAX_LIGHT_SOURCES 16
#endif //_HOST_DEVICE_SHARED_MACROS_H
