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
#include "Framework.h"
#include "Material.h"
#include "API/ConstantBuffer.h"
#include "API/Texture.h"
#include "API/Buffer.h"
#include "Utils/os.h"
#include "Utils/Math/FalcorMath.h"
#include "MaterialSystem.h"
#include "API/ProgramVars.h"

namespace Falcor
{
    uint32_t Material::sMaterialCounter = 0;

    Material::Material(const std::string& name) : mName(name)
    {
        mData.id = sMaterialCounter;
        sMaterialCounter++;
    }

    Material::SharedPtr Material::create(const std::string& name)
    {
        Material* pMaterial = new Material(name);
        return SharedPtr(pMaterial);
    }

    Material::~Material() = default;

    void Material::resetGlobalIdCounter()
    {
        sMaterialCounter = 0;
    }

    void Material::setDiffuseTexture(Texture::SharedPtr& pDiffuse)
    {
        mData.textures.diffuse = pDiffuse;
        updateDiffuseType();
    }

    void Material::setSpecularTexture(Texture::SharedPtr pSpecular)
    {
        mData.textures.specular = pSpecular;
        updateSpecularType();
    }

    void Material::setEmissiveTexture(Texture::SharedPtr& pEmissive)
    {
        mData.textures.emissive = pEmissive;
        updateEmissiveType();
    }

    void Material::setDiffuseColor(const vec4& color)
    {
        mData.diffuse = color;
        updateDiffuseType();
    }

    void Material::setSpecularColor(const vec4& color)
    {
        mData.specular = color;
        updateSpecularType();
    }

    void Material::setEmissiveColor(const vec3& color)
    {
        mData.emissive = color;
        updateEmissiveType();
    }

    template<typename vec>
    static uint32_t getChannelMode(bool hasTexture, const vec& color)
    {
        if (hasTexture) return ChannelTypeTexture;
        if (luminance(color) == 0) return ChannelTypeUnused;
        return ChannelTypeConst;
    }

    void Material::updateDiffuseType()
    {
        mData.flags = PACK_DIFFUSE_TYPE(mData.flags, getChannelMode(mData.textures.diffuse != nullptr, mData.diffuse));
    }

    void Material::updateSpecularType()
    {
        mData.flags = PACK_SPECULAR_TYPE(mData.flags, getChannelMode(mData.textures.specular != nullptr, mData.specular));
    }

    void Material::updateEmissiveType()
    {
        mData.flags = PACK_EMISSIVE_TYPE(mData.flags, getChannelMode(mData.textures.emissive != nullptr, mData.emissive));
    }

    void Material::setNormalMap(Texture::SharedPtr pNormalMap)
    {
        mData.textures.normalMap = pNormalMap;
        uint32_t normalMode = NormalMapUnused;
        if (pNormalMap)
        {
            switch(getFormatChannelCount(pNormalMap->getFormat()))
            {
            case 2:
                normalMode = NormalMapRG;
                break;
            case 3:
                normalMode = NormalMapRGB;
                break;
            default:
                should_not_get_here();
                logWarning("Unsupported normal map format for material " + mName);
            }
        }
        mData.flags = PACK_NORMAL_MAP_TYPE(mData.flags, normalMode);
    }

    void Material::setOcclusionMap(Texture::SharedPtr pOcclusionMap)
    {
        mData.textures.occlusionMap = pOcclusionMap;
        mData.flags = PACK_OCCLUSION_MAP(mData.flags, pOcclusionMap ? 1 : 0);
    }

    void Material::setReflectionMap(Texture::SharedPtr pReflectionMap)
    {
        mData.textures.reflectionMap = pReflectionMap;
        mData.flags = PACK_REFLECTION_MAP(mData.flags, pReflectionMap ? 1 : 0);
    }

    void Material::setLightMap(Texture::SharedPtr pLightMap)
    {
        mData.textures.lightMap = pLightMap;
        mData.flags = PACK_LIGHT_MAP(mData.flags, pLightMap ? 1 : 0);
    }

    void Material::setHeightMap(Texture::SharedPtr pHeightMap)
    {
        mData.textures.heightMap = pHeightMap;
        mData.flags = PACK_HEIGHT_MAP(mData.flags, pHeightMap ? 1 : 0);
    }

    bool Material::operator==(const Material& other) const 
    {
#define compare_field(_a) if (mData._a != other.mData._a) return false
        compare_field(diffuse);
        compare_field(specular);
        compare_field(emissive);
        compare_field(alphaThreshold);
        compare_field(IoR);
        compare_field(flags);
        compare_field(heightScaleOffset);
#undef compare_field

#define compare_texture(_a) if (mData.textures._a != other.mData.textures._a) return false
        compare_texture(diffuse);
        compare_texture(specular);
        compare_texture(emissive);
        compare_texture(normalMap);
        compare_texture(occlusionMap);
        compare_texture(reflectionMap);
        compare_texture(lightMap);
        compare_texture(heightMap);
#undef compare_texture
        return true;
    }
}
