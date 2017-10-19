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
#include "glm/vec3.hpp"
#include "glm/mat4x4.hpp"

namespace Falcor
{
    /** Handles world-space transformations for scene objects.
        Contains two sets of transforms that can be applied.
            - Most functions will modify the 'base' transform
            - Calling move() will modify a separate 'movable' transform that is applied on top of the 'base' transform. This is currently used by paths.
    */
    class Transform : public std::enable_shared_from_this<Transform>
    {
    public:
        using SharedPtr = std::shared_ptr<Transform>;
        using SharedConstPtr = std::shared_ptr<const Transform>;

        /** Sets position/translation.
            \param[in] position World-space position
            \param[in] translateLookAt If true, additionally translates the look-at target to preserve rotation
        */
        void setPosition(const vec3& position, bool translateLookAt);

        /** Sets scale.
            \param[in] scaling XYZ scaling
        */
        void setScaling(const vec3& scaling);

        /** Sets rotation from angles.
            \param[in] yawPitchRoll Yaw-Pitch-Roll (YXZ) rotation in radians
        */
        void setRotation(const vec3& yawPitchRoll);

        /** Sets the up vector.
            \param[in] up Up vector
        */
        void setUpVector(const vec3& up);

        /** Sets the look-at target.
            \param[in] target Look-at target position
        */
        void setTarget(const vec3& target);

        /** Sets the look direction. Updates target to the new direction, but the same distance away.
        */
        void setForwardVector(const vec3& forward);

        /** Gets the position/translation.
            \return World space position
        */
        const vec3& getPosition() const { return mBase.position; }

        /** Gets rotation as 3 angles.
            \return Yaw-Pitch-Roll rotations in radians
        */
        vec3 getRotation() const;

        /** Gets scale.
            \return XYZ scale
        */
        const vec3& getScaling() const { return mBase.scale; }

        /** Gets the up vector of the orientation.
            \return Normalized up vector
        */
        const vec3& getUpVector() const { return mBase.up; }

        /** Gets the look-at target of the orientation.
            \return World space look-at target position
        */
        const vec3& getTarget() const { return mBase.target; }

        /** Gets the forward vector of the orientation
            \return Normalized forward vector
        */
        const vec3& getForwardVector() const { return mBase.forward; }

        /** Gets the final transform matrix containing all 'base' and 'movable' transforms applied.
            \return World transform matrix
        */
        const mat4& getTransformMatrix() const;

        /** Move the object by setting its 'movable' transformation.
        */
        virtual void move(const vec3& position, const vec3& target, const vec3& up);

    protected:
        Transform() = default;
        Transform(const vec3& position, const vec3& target, const vec3& up, const vec3& scale);
        Transform(const vec3& position, const vec3& yawPitchRoll, const vec3& scale);

        /** Updates dirty transforms.
            Combines position, up, target, and scale into a single matrix, then updates mFinalTransformMatrix from mBase and mMovable
        */
        void updateTransformMatrix();

        /** Called after updateTransformMatrix() finishes.
        */
        virtual void onTransformMatrixUpdated() {}

        struct TransformData
        {
            vec3 position;
            vec3 up = vec3(0.0f, 1.0f, 0.0f);
            vec3 target = vec3(0.0f, 0.0f, 1.0f);
            vec3 forward = vec3(0.0f, 0.0f, 1.0f);
            vec3 scale = vec3(1.0f);

            // Matrix containing the above transforms
            mat4 matrix;
            bool matrixDirty = true;
        };

        mutable TransformData mBase;
        mutable TransformData mMovable;
        mutable mat4 mFinalTransformMatrix;
    };
}