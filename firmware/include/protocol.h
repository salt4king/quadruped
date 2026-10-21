#pragma once

// The link to the host.
//
// Takes a velocity command and a body pose -- intent, not angles -- and
// sends telemetry back. Roughly 24 bytes each way at 20-50 Hz.
//
// The deadman lives here: if nothing arrives for COMMAND_TIMEOUT_MS the
// robot stops. See docs/CONTROL.md.
