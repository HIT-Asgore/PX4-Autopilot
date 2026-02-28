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

#include "ActuatorEffectivenessPassThrough.hpp"

using matrix::Vector3f;

ActuatorEffectivenessPassThrough::ActuatorEffectivenessPassThrough(ModuleParams *parent)
	: ModuleParams(parent)
{
}

bool ActuatorEffectivenessPassThrough::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// We expose a subset of the control vector directly via 4 "servo" actuators:
	// actuator_sp[0..2] == control_sp[0..2] (Mx, My, Mz)
	// actuator_sp[3]    == control_sp[5]   (Fz)
	//
	// Achieved by setting the corresponding columns in B.
	const Vector3f zero{0.f, 0.f, 0.f};

	// Torque (Mx, My, Mz)
	if (configuration.addActuator(ActuatorType::SERVOS, Vector3f{1.f, 0.f, 0.f}, zero) < 0) { return false; }
	if (configuration.addActuator(ActuatorType::SERVOS, Vector3f{0.f, 1.f, 0.f}, zero) < 0) { return false; }
	if (configuration.addActuator(ActuatorType::SERVOS, Vector3f{0.f, 0.f, 1.f}, zero) < 0) { return false; }

	// Thrust (Fz)
	if (configuration.addActuator(ActuatorType::SERVOS, zero, Vector3f{0.f, 0.f, 1.f}) < 0) { return false; }

	return true;
}

