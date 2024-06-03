using UnityEngine;

public class MadgwickAHRS 
{
    private float beta; // Algorithm gain
    private Quaternion q; // Quaternion of sensor frame relative to auxiliary frame

    public MadgwickAHRS(float beta = 0.1f)
    {
        this.beta = beta;
        this.q = Quaternion.identity;
    }

    // Update method
    public void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltaT)
    {
        float q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;
        float norm;
        float hx, hy, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
        float pa, pb, pc;

        // Normalize accelerometer measurement
        norm = Mathf.Sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0f) return;
        norm = 1f / norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalize magnetometer measurement
        norm = Mathf.Sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0f) return;
        norm = 1f / norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        hx = 2f * mx * (0.5f - q3 * q3 - q4 * q4) + 2f * my * (q2 * q3 - q1 * q4) + 2f * mz * (q2 * q4 + q1 * q3);
        hy = 2f * mx * (q2 * q3 + q1 * q4) + 2f * my * (0.5f - q2 * q2 - q4 * q4) + 2f * mz * (q3 * q4 - q1 * q2);
        bx = Mathf.Sqrt((hx * hx) + (hy * hy));
        bz = 2f * mx * (q2 * q4 - q1 * q3) + 2f * my * (q3 * q4 + q1 * q2) + 2f * mz * (0.5f - q2 * q2 - q3 * q3);

        // Estimated direction of gravity and magnetic field
        vx = 2f * (q2 * q4 - q1 * q3);
        vy = 2f * (q1 * q2 + q3 * q4);
        vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;
        wx = 2f * bx * (0.5f - q3 * q3 - q4 * q4) + 2f * bz * (q2 * q4 - q1 * q3);
        wy = 2f * bx * (q2 * q3 - q1 * q4) + 2f * bz * (q1 * q2 + q3 * q4);
        wz = 2f * bx * (q1 * q3 + q2 * q4) + 2f * bz * (0.5f - q2 * q2 - q3 * q3);

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

        if (ex != 0f && ey != 0f && ez != 0f)
        {
            // Apply feedback terms
            gx += beta * ex;
            gy += beta * ey;
            gz += beta * ez;
        }

        // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 += (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltaT);
        q2 += (q1 * gx + pb * gz - pc * gy) * (0.5f * deltaT);
        q3 += (q1 * gy - pa * gz + pc * gx) * (0.5f * deltaT);
        q4 += (q1 * gz + pa * gy - pb * gx) * (0.5f * deltaT);

        // Normalize quaternion
        norm = Mathf.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1f / norm;
        q1 *= norm;
        q2 *= norm;
        q3 *= norm;
        q4 *= norm;

        q = new Quaternion(q2, q3, q4, q1);
    }

    public Quaternion GetQuaternion()
    {
        return q;
    }
}