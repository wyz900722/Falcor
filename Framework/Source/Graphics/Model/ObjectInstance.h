/***************************************************************************
# Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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
#pragma once

#include "Graphics/Transform.h"
#include "Utils/AABB.h"
#include "Utils/Math/FalcorMath.h"

namespace Falcor
{
    class SceneRenderer;
    class Model;

    /** Handles transformations for Mesh and Model instances. Primary transform is stored in the "Base" transform. An additional "Movable"
        transform is applied after the Base transform can be set through the IMovableObject interface. This is currently used by paths.
    */
    template<typename ObjectType>
    class ObjectInstance : public Transform, public inherit_shared_from_this<Transform, ObjectInstance<ObjectType>>
    {
    public:
        using SharedPtr = std::shared_ptr<ObjectInstance<ObjectType>>;
        using SharedConstPtr = std::shared_ptr<const ObjectInstance<ObjectType>>;
        using inherit_shared_from_this<Transform, ObjectInstance<ObjectType>>::shared_from_this;

        /** Constructs an object instance with a transform
            \param[in] pObject Object to create an instance of
            \param[in] baseTransform Base transform matrix of the instance
            \param[in] name Name of the instance
            \return A new instance of the object if pObject
        */
        static SharedPtr create(const typename ObjectType::SharedPtr& pObject, const mat4& baseTransform, const std::string& name = "")
        {
            assert(pObject);
            return SharedPtr(new ObjectInstance<ObjectType>(pObject, baseTransform, name));
        }

        /** Constructs an object instance with a transform
            \param[in] pObject Object to create an instance of
            \param[in] translation Base translation of the instance
            \param[in] target Base look-at target of the instance
            \param[in] up Base up vector of the instance
            \param[in] scale Base scale of the instance
            \param[in] name Name of the instance
            \return A new instance of the object
        */
        static SharedPtr create(const typename ObjectType::SharedPtr& pObject, const vec3& translation, const vec3& target, const vec3& up, const vec3& scale, const std::string& name = "")
        {
             return SharedPtr(new ObjectInstance<ObjectType>(pObject, translation, target, up, scale, name));
        }

        /** Constructs an object instance with a transform
            \param[in] pObject Object to create an instance of
            \param[in] translation Base translation of the instance
            \param[in] yawPitchRoll Rotation of the instance in radians
            \param[in] scale Base scale of the instance
            \param[in] name Name of the instance
            \return A new instance of the object
        */
        static SharedPtr create(const typename ObjectType::SharedPtr& pObject, const vec3& translation, const vec3& yawPitchRoll, const vec3& scale, const std::string& name = "")
        {
            return SharedPtr(new ObjectInstance<ObjectType>(pObject, translation, yawPitchRoll, scale, name));
        }

        /** Gets object for which this is an instance of
            \return Object for this instance
        */
        const typename ObjectType::SharedPtr& getObject() const { return mpObject; };

        /** Sets visibility of this instance
            \param[in] visible Visibility of this instance
        */
        void setVisible(bool visible) { mVisible = visible; };

        /** Gets whether this instance is visible
            \return Whether this instance is visible
        */
        bool isVisible() const { return mVisible; };

        /** Gets instance name
            \return Instance name
        */
        const std::string& getName() const { return mName; }

        /** Sets instance name
            \param[in] name Instance name
        */
        void setName(const std::string& name) { mName = name; }

        /** Gets the bounding box
            \return Bounding box
        */
        const BoundingBox& getBoundingBox() const
        {
            updateTransformMatrix();
            return mBoundingBox;
        }

    private:

        virtual void onTransformMatrixUpdated() override
        {
            mBoundingBox = mpObject->getBoundingBox().transform(mFinalTransformMatrix);
        }

        ObjectInstance(const typename ObjectType::SharedPtr& pObject, const std::string& name)
            : mpObject(pObject), mName(name) { }

        ObjectInstance(const typename ObjectType::SharedPtr& pObject, const mat4& baseTransform, const std::string& name)
            : ObjectInstance(pObject, name)
        {
            // #TODO Decompose matrix
            mBase.matrix = baseTransform;
            mBase.matrixDirty = false;
        }

        ObjectInstance(const typename ObjectType::SharedPtr& pObject, const vec3& translation, const vec3& target, const vec3& up, const vec3& scale, const std::string& name = "")
            : Transform(translation, target, up, scale)
            , mpObject(pObject)
            , mName(name)
        {
        }

        ObjectInstance(const typename ObjectType::SharedPtr& pObject, const vec3& translation, const vec3& yawPitchRoll, const vec3& scale, const std::string& name = "")
            : Transform(translation, yawPitchRoll, scale)
            , mpObject(pObject)
            , mName(name)
        {
        }

        friend class Model;

        std::string mName;
        bool mVisible = true;

        typename ObjectType::SharedPtr mpObject;

        mutable BoundingBox mBoundingBox;
    };
}
