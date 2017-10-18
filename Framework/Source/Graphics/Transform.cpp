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

#include "Framework.h"
#include "Graphics/Transform.h"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "Utils/Math/FalcorMath.h"

namespace Falcor
{
    Transform::Transform(const vec3& translation, const vec3& target, const vec3& up, const vec3& scale)
    {
        mBase.translation = translation;
        mBase.target = target;
        mBase.up = up;
        mBase.scale = scale;
    }

    Transform::Transform(const vec3& translation, const vec3& yawPitchRoll, const vec3& scale)
    {
        mBase.translation = translation;
        setRotation(yawPitchRoll);
        mBase.scale = scale;
    }

    void Transform::setTranslation(const vec3& translation, bool translateLookAt)
    {
        if (translateLookAt)
        {
            vec3 toLookAt = mBase.target - mBase.translation;
            mBase.target = translation + toLookAt;
        }

        mBase.translation = translation;
        mBase.matrixDirty = true;
    }

    void Transform::setRotation(const vec3& yawPitchRoll)
    {
        // Construct matrix from angles and take upper 3x3
        const mat3 rotMtx(glm::yawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]));

        // Preserve distance of look-at target
        const float targetDist = length(mBase.target - mBase.translation);

        // Get look-at info
        mBase.up = rotMtx[1];
        mBase.target = mBase.translation + rotMtx[2] * targetDist; // position + forward

        mBase.matrixDirty = true;
    }

    void Transform::setScaling(const vec3& scaling)
    {
        mBase.scale = scaling;
        mBase.matrixDirty = true;
    }

    void Transform::setUpVector(const vec3& up)
    {
        mBase.up = normalize(up);
        mBase.matrixDirty = true;
    }

    void Transform::setTarget(const vec3& target)
    {
        mBase.target = target;
        mBase.matrixDirty = true;
    }

    vec3 Transform::getRotation() const
    {
        mat4 rotationMtx = createMatrixFromLookAt(mBase.translation, mBase.target, mBase.up);

        vec3 result;
        extractEulerAngleXYZ(rotationMtx, result[1], result[0], result[2]); // YawPitchRoll is YXZ
        return result;
    }

    const mat4& Transform::getTransformMatrix() const
    {
        // #TODO do we have to do this
        const_cast<Transform*>(this)->updateTransformMatrix();
        return mFinalTransformMatrix;
    }

    void Transform::move(const vec3& position, const vec3& target, const vec3& up)
    {
        mMovable.translation = position;
        mMovable.target = target;
        mMovable.up = up;
        mMovable.scale = vec3(1.0f);
        mMovable.matrixDirty = true;
    }

    mat4 calculateTransformMatrix(const vec3& translation, const vec3& target, const vec3& up, const vec3& scale)
    {
        mat4 translationMtx = mat4();
        translationMtx[3] = vec4(translation, 1);
        mat4 rotationMtx = createMatrixFromLookAt(translation, target, up);
        mat4 scalingMtx = glm::scale(mat4(), scale);

        return translationMtx * rotationMtx * scalingMtx;
    }

    mat4 calculateTransformMatrix(const vec3& translation, const vec3& yawPitchRoll, const vec3& scale)
    {
        mat4 translationMtx = mat4();
        translationMtx[3] = vec4(translation, 1);
        mat4 rotationMtx = glm::yawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
        mat4 scalingMtx = glm::scale(mat4(), scale);

        return translationMtx * rotationMtx * scalingMtx;
    }

    void Transform::updateTransformMatrix()
    {
        if (mBase.matrixDirty || mMovable.matrixDirty)
        {
            if (mBase.matrixDirty)
            {
                mBase.matrix = calculateTransformMatrix(mBase.translation, mBase.target, mBase.up, mBase.scale);
                mBase.matrixDirty = false;
            }

            if (mMovable.matrixDirty)
            {
                mMovable.matrix = calculateTransformMatrix(mMovable.translation, mMovable.target, mMovable.up, mMovable.scale);
                mMovable.matrixDirty = false;
            }

            mFinalTransformMatrix = mMovable.matrix * mBase.matrix;
        }
    }

}