/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef TMC_BASE_PATH_FOLLOWER_PARAMETER_DEFAULT_VALUE_HPP_
#define TMC_BASE_PATH_FOLLOWER_PARAMETER_DEFAULT_VALUE_HPP_
/// Define default values for parameters
/// Be cautious when changing some default values as they are used across multiple classes
namespace tmc_base_path_follower {
/// Default value for the movement model name
constexpr const char* const kMoveModelNameDefault = "omni";
/// Default value for whether to perform route passage speed control
constexpr bool kUsePathTransitVelocityDefault = false;
// Distance from the goal to the line for goal area judgment
constexpr double kGoalAreaLengthDefault = 0.5;
// Distance from the goal to the line for goal judgment
constexpr double kGoalLineLengthDefault = 0.05;
// Position deviation threshold for goal judgment
constexpr double kGoalStopErrorLengthDefault = 0.03;
// Angle deviation threshold for goal judgment
constexpr double kGoalStopErrorAngleDefault = 0.03;
// Partial search for the nearest route point: search range from the previous nearest point [m]. If 0.0 is specified, always perform a full search
constexpr double kPartialSearchRangeDefault = 0.0;
// Partial search for the nearest route point: allowable error value [m] between the searched route point and the self-position. If the allowable value is exceeded, perform a full search
constexpr double kPartialSearchPermitErrorDefault = 0.5;
// Number of interpolation points between two points in route interpolation
constexpr int32_t kInterpolationNumberDefault = 10;
// Maximum translational speed
constexpr double kMaxLinearVelocityDefault = 0.6;
// Minimum translational speed
constexpr double kMinLinearVelocityDefault = 0.3;
// Maximum rotational speed
constexpr double kMaxAngularVelocityDefault = 0.8;
// Maximum translational acceleration
constexpr double kMaxLinearAccelerationDefault = 1.0;
// Maximum translational deceleration
constexpr double kMaxLinearDecelerationDefault = 1.0;
// Maximum rotational acceleration
constexpr double kMaxAngularAccelerationDefault = 1.0;
// Maximum rotational deceleration
constexpr double kMaxAngularDecelerationDefault = 1.0;
// Translational deceleration near the goal
constexpr double kGoalDecelerationDefault = 0.3;
// Translational speed margin during goal deceleration
constexpr double kVelocityMarginDefault = 0.05;
// Path length threshold (m) to change the upper body orientation
constexpr double kPathLengthThresholdDefault = 2.0;
// Translational speed P control gain parameter
constexpr double kLinearPGainDefault = 0.5;
// Rotational speed P control gain parameter
constexpr double kAngularPGainDefault = 0.5;
// Goal posture alignment gain parameter
constexpr double kGoalAngleGainDefault = 0.5;
// Translational speed α gain parameter
constexpr double kLinearAlphaGainDefault = 3.0;
// Translational speed β gain parameter
constexpr double kLinearBetaGainDefault = 1.5;
// Ratio of rotational speed to angular error
constexpr double kAngleErrorAngularVelocityRateDefault = 1.0;
// Angular error to start in-place rotation
constexpr double kSpinStartErrorAngleDefault = 0.8;
// Angular error to end in-place rotation
constexpr double kSpinEndErrorAngleDefault = 0.03;
// Maximum rotational speed for in-place rotation
constexpr double kSpinMaxAngularVelocityDefault = 2.0;
// Minimum rotational speed for in-place rotation
constexpr double kSpinMinAngularVelocityDefault = 0.05;
// Multiplier applied to rotational speed when calculating passage speed based on curvature
constexpr double kTransitVelocityAngularVelocityRatioDefault = 0.6;
}  // namespace tmc_base_path_follower
#endif  // TMC_BASE_PATH_FOLLOWER_PARAMETER_DEFAULT_VALUE_HPP_
