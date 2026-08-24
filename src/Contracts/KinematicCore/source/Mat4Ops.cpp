#include "Mat4Ops.h"

namespace kinematic_core
{
void mat4IdentityColumnMajor(double out[16])
{
	for (int i = 0; i < 16; ++i)
	{
		out[i] = 0.0;
	}
	out[0] = out[5] = out[10] = out[15] = 1.0;
}

void mat4CopyColumnMajor16(const double in[16], double out[16])
{
	for (int i = 0; i < 16; ++i)
	{
		out[i] = in[i];
	}
}

void mat4MulColumnMajor16(const double a[16], const double b[16], double out[16])
{
	double tmp[16];
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			tmp[c * 4 + r] = a[0 * 4 + r] * b[c * 4 + 0] + a[1 * 4 + r] * b[c * 4 + 1] + a[2 * 4 + r] * b[c * 4 + 2] +
							 a[3 * 4 + r] * b[c * 4 + 3];
		}
	}
	for (int i = 0; i < 16; ++i)
	{
		out[i] = tmp[i];
	}
}

} // namespace kinematic_core
