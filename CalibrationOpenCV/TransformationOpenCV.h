// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#pragma once

#include <opencv2/core.hpp>

/**
* struct TransformationOpenCV
*
* This struct aims to work like the Transformation struct (cf. Transformation.h), but with OpenCV structures instead of classical
* C++ ones.
*/
struct TransformationOpenCV
{
    cv::Matx33d R;
    cv::Vec3d t;

    // Construct an identity transformation.
    TransformationOpenCV() : R(cv::Matx33d::eye()), t(0., 0., 0.) {}

    TransformationOpenCV(cv::Matx33d Rot, cv::Vec3d trans) : R(Rot), t(trans) {}

    TransformationOpenCV(const cv::Matx44d& H) : R(H.get_minor<3, 3>(0, 0)), t(H(0, 3), H(1, 3), H(2, 3)) {}

    // Create homogeneous matrix from this transformation
    cv::Matx44d to_homogeneous() const
    {
        return cv::Matx44d(
            // row 1
            R(0, 0),
            R(0, 1),
            R(0, 2),
            t(0),
            // row 2
            R(1, 0),
            R(1, 1),
            R(1, 2),
            t(1),
            // row 3
            R(2, 0),
            R(2, 1),
            R(2, 2),
            t(2),
            // row 4
            0,
            0,
            0,
            1);
    }

    // Construct a transformation equivalent to this transformation followed by the second transformation
    TransformationOpenCV compose_with(const TransformationOpenCV& second_transformation) const
    {
        // get this transform
        cv::Matx44d H_1 = to_homogeneous();
        // get the transform to be composed with this one
        cv::Matx44d H_2 = second_transformation.to_homogeneous();
        // get the combined transform
        cv::Matx44d H_3 = H_1 * H_2;
        return TransformationOpenCV(H_3);
    }
};