/****************************************************************************
 *
 *   Copyright (c) 2015 Mohammed Kabir. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sat_att_control_params.c
 *
 * Parameters defined by the satellite attitude control task
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Pitch rate proportional gain.
 *
 * This defines how much the elevator input will be commanded depending on the
 * current body angular rate error.
 *
 * @min 0.005
 * @max 1.0
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_PR_P, 0.005f);

/**
 * Pitch rate integrator gain.
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @min 0.0
 * @max 0.5
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_PR_I, 0.0f);

/**
 * Roll rate proportional Gain
 *
 * This defines how much the aileron input will be commanded depending on the
 * current body angular rate error.
 *
 * @min 0.005
 * @max 1.0
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_RR_P, 0.005f);

/**
 * Roll rate integrator Gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @min 0.0
 * @max 0.2
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_RR_I, 0.0f);

/**
 * Yaw rate proportional gain
 *
 * This defines how much the rudder input will be commanded depending on the
 * current body angular rate error.
 *
 * @min 0.005
 * @max 1.0
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_YR_P, 0.005f);

/**
 * Yaw rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @min 0.0
 * @max 50.0
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_YR_I, 0.0f);

/**
 * Roll time constant
 *
 * @unit seconds
 * @min 0.4
 * @max 1.0
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_R_TC, 0.4f);

/**
 * Pitch time constant
 *
 * @unit seconds
 * @min 0.4
 * @max 1.0
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_P_TC, 0.4f);

/**
 * Yaw time constant
 *
 * @unit seconds
 * @min 0.4
 * @max 1.0
 * @group Satellite Attitude Control
 */
PARAM_DEFINE_FLOAT(SAT_Y_TC, 0.4f);
