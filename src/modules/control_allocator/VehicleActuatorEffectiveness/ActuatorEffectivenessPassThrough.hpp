/****************************************************************************
 *
 *   Copyright (c) 2026.
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>

/**
 * "Pass-through" effectiveness: expose a subset of the 6D control vector directly as actuator outputs.
 *
 * Output convention (actuator_servos controls[0..3]):
 * - 0: Mx (roll torque)
 * - 1: My (pitch torque)
 * - 2: Mz (yaw torque)
 * - 3: Fz (thrust z)
 *
 * All values are in body frame (FRD) and normalized (same as vehicle_{torque,thrust}_setpoint).
 */
class ActuatorEffectivenessPassThrough : public ModuleParams, public ActuatorEffectiveness
{
public:
	explicit ActuatorEffectivenessPassThrough(ModuleParams *parent);
	~ActuatorEffectivenessPassThrough() override = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		// Prefer the simplest solution to keep pass-through behavior predictable.
		allocation_method_out[0] = AllocationMethod::PSEUDO_INVERSE;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		// No normalization: keep 1:1 mapping.
		normalize[0] = false;
	}

	const char *name() const override { return "UEPassThrough"; }
};

