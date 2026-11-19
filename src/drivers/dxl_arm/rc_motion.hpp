#pragma once

#include <math.h>
#include <stdint.h>

namespace dxl_rc
{
enum class ScPosition { Unknown, Up, Center, Down };

static constexpr int32_t level_goal = 2000;
static constexpr int32_t raise_goal = 2800;
static constexpr int32_t wave_min = 1900;
static constexpr int32_t wave_max = 2500;
static constexpr float wave_period_s = 7.f;

class CenterGate
{
public:
	void reset()
	{
		_mode = ScPosition::Unknown;
		_center_ready = false;
	}

	ScPosition update(ScPosition raw, bool stable)
	{
		// Leaving either outer detent consumes the permission earned in center.
		if (raw != _mode && (_mode == ScPosition::Up || _mode == ScPosition::Down
				     || (_mode == ScPosition::Center && !_center_ready))) {
			reset();
		}

		if (stable && raw == ScPosition::Center && _mode == ScPosition::Unknown) {
			_mode = ScPosition::Center;

		} else if (stable && _center_ready && (raw == ScPosition::Up || raw == ScPosition::Down)) {
			_mode = raw;
			_center_ready = false;
		}

		return _mode;
	}

	void set_center_ready(bool settled) { _center_ready = _mode == ScPosition::Center && settled; }

private:
	ScPosition _mode{ScPosition::Unknown};
	bool _center_ready{false};
};

inline int32_t wave_goal(float elapsed_s)
{
	constexpr int32_t center = (wave_min + wave_max) / 2;
	constexpr int32_t amplitude = (wave_max - wave_min) / 2;
	constexpr float ramp_s = 0.7f;
	constexpr float omega = 6.283185307f / wave_period_s;
	const float start_phase = asinf(static_cast<float>(level_goal - center) / amplitude);
	// Start at 2000 going upward; increase phase speed from zero continuously.
	const float phase_time = elapsed_s < ramp_s ? elapsed_s * elapsed_s / (2.f * ramp_s) : elapsed_s - ramp_s / 2.f;
	return center + static_cast<int32_t>(roundf(amplitude * sinf(start_phase + omega * phase_time)));
}
} // namespace dxl_rc
