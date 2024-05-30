using UnityEngine;

public class AttitudeCalculator : MonoBehaviour
{
    private IMUDataProcessor dataProcessor;
    public Quaternion quaternion = Quaternion.identity;
    public Vector3 eulerAngles;

    private Vector3 error = Vector3.zero;
    private Vector3 errorIntegral = Vector3.zero;
    private float magYawBias = 0f;

    public void Initialize()
    {
        dataProcessor = GetComponent<IMUDataProcessor>();
    }

    public void CalculateAttitude(Vector3 accelData, Vector3 gyroData, Vector3 magData, float deltaTime)
    {
        dataProcessor.ProcessData(out accelData, out gyroData, out magData);

        Vector3 accelCorrected = accelData * 1000.0f;
        Vector3 gyroCorrected = gyroData;
        Vector3 magCorrected = new Vector3(magData.y, magData.x, -magData.z);

        // 如果需要计算姿态
        if (!ShouldRun())
        {
            return;
        }

        // 电子罗盘处理
        if (UseMag())
        {
            HandleMagneticData(magCorrected, ref magYawBias);
        }

        // 加速度计处理
        Vector3 accTmp = Normalize(accelCorrected);
        CalculateError(accTmp, deltaTime, ref error, ref errorIntegral);

        // 修正陀螺仪数据
        Vector3 gyroCorrect = CorrectGyroData(gyroCorrected, deltaTime, error, errorIntegral, magYawBias);

        // 一阶龙格库塔更新四元数值
        quaternion = UpdateQuaternion(quaternion, gyroCorrect, deltaTime);

        // 四元数归一化
        quaternion = NormalizeQuaternion(quaternion);

        // 计算旋转矩阵
        float[,] rotateMatrix = CalculateRotateMatrix(quaternion);

        // 计算世界坐标系下的加速度和磁力计值
        Vector3 accWorld = TransformToWorld(rotateMatrix, accelCorrected);
        Vector3 magWorld = TransformToWorld(rotateMatrix, magCorrected);

        // 计算欧拉角
        eulerAngles = CalculateEulerAngles(rotateMatrix);
    }

    private bool ShouldRun()
    {
        // 判断是否需要计算姿态的逻辑
        return true;
    }

    private bool UseMag()
    {
        // 判断是否使用磁力计的逻辑
        return true;
    }

    private void HandleMagneticData(Vector3 magCorrected, ref float magYawBias)
    {
        float length = Mathf.Sqrt(magCorrected.x * magCorrected.x + magCorrected.y * magCorrected.y);
        if (magCorrected.x != 0 && magCorrected.y != 0 && magCorrected.z != 0 && length != 0)
        {
            float magYaw = Mathf.Atan2(magCorrected.y / length, magCorrected.x / length);
            if (IsRotationMatrixValid())
            {
                magYawBias = CorrectYawBias(magYaw, magYawBias);
            }
            else
            {
                magYawBias = 0;
            }
        }
    }

    private bool IsRotationMatrixValid()
    {
        // 验证旋转矩阵是否有效的逻辑
        return true;
    }

    private float CorrectYawBias(float magYaw, float magYawBias)
    {
        // 计算yaw偏差的逻辑
        float yaw = eulerAngles.z; // 假设yaw存储在eulerAngles的z分量中
        float correctedYawBias = CorrectKp * (yaw - magYaw);
        return Mathf.Abs(correctedYawBias) < 360 ? correctedYawBias : 0;
    }

    private const float CorrectKp = 1.0f; // 修正系数

    private Vector3 Normalize(Vector3 vector)
    {
        float length = vector.magnitude;
        return length != 0 ? vector / length : Vector3.zero;
    }

    private void CalculateError(Vector3 accTmp, float deltaTime, ref Vector3 error, ref Vector3 errorIntegral)
    {
        // 加速度计误差计算
        if (accTmp.magnitude > 0.8f && accTmp.magnitude < 1.2f)
        {
            Vector3 errorTemp = CrossProduct(accTmp, Vector3.forward); // 与z轴叉乘得到误差
            error += errorTemp * Mathf.PI * deltaTime;
        }
        else
        {
            error = Vector3.zero;
        }

        // 误差积分
        errorIntegral += error * deltaTime * ErrorKi;
        errorIntegral = Clamp(errorIntegral, -0.035f, 0.035f);
    }

    private const float ErrorKi = 0.1f; // 误差积分系数

    private Vector3 CrossProduct(Vector3 a, Vector3 b)
    {
        return new Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    private Vector3 Clamp(Vector3 vector, float min, float max)
    {
        return new Vector3(
            Mathf.Clamp(vector.x, min, max),
            Mathf.Clamp(vector.y, min, max),
            Mathf.Clamp(vector.z, min, max)
        );
    }

    private Vector3 CorrectGyroData(Vector3 gyroData, float deltaTime, Vector3 error, Vector3 errorIntegral, float magYawBias)
    {
        Vector3 gyroCorrect = new Vector3(
            (gyroData.x - RotateMatrix[0, 2] * magYawBias) * Mathf.Deg2Rad + (CorrectKp * error.x + errorIntegral.x),
            (gyroData.y - RotateMatrix[1, 2] * magYawBias) * Mathf.Deg2Rad + (CorrectKp * error.y + errorIntegral.y),
            (gyroData.z - RotateMatrix[2, 2] * magYawBias) * Mathf.Deg2Rad + (CorrectKp * error.z + errorIntegral.z)
        );

        return gyroCorrect;
    }

    private Quaternion UpdateQuaternion(Quaternion quaternion, Vector3 gyroCorrect, float deltaTime)
    {
        Quaternion deltaQuaternion = new Quaternion(
            gyroCorrect.x * deltaTime * 0.5f,
            gyroCorrect.y * deltaTime * 0.5f,
            gyroCorrect.z * deltaTime * 0.5f,
            0
        );

        quaternion = quaternion * deltaQuaternion;
        return quaternion;
    }

    private Quaternion NormalizeQuaternion(Quaternion q)
    {
        float length = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        return length != 0 ? new Quaternion(q.x / length, q.y / length, q.z / length, q.w / length) : q;
    }

    private float[,] CalculateRotateMatrix(Quaternion quaternion)
    {
        float[,] rotateMatrix = new float[3, 3];

        rotateMatrix[0, 0] = quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z;
        rotateMatrix[0, 1] = 2 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
        rotateMatrix[0, 2] = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);

        rotateMatrix[1, 0] = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
        rotateMatrix[1, 1] = quaternion.w * quaternion.w - quaternion.x * quaternion.x + quaternion.y * quaternion.y - quaternion.z * quaternion.z;
        rotateMatrix[1, 2] = 2 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x);

        rotateMatrix[2, 0] = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
        rotateMatrix[2, 1] = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
        rotateMatrix[2, 2] = quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z;

        return rotateMatrix;
    }

    private Vector3 TransformToWorld(float[,] rotateMatrix, Vector3 localData)
    {
        Vector3 worldData = new Vector3(
            rotateMatrix[0, 0] * localData.x + rotateMatrix[1, 0] * localData.y + rotateMatrix[2, 0] * localData.z,
            rotateMatrix[0, 1] * localData.x + rotateMatrix[1, 1] * localData.y + rotateMatrix[2, 1] * localData.z,
            rotateMatrix[0, 2] * localData.x + rotateMatrix[1, 2] * localData.y + rotateMatrix[2, 2] * localData.z
        );

        return worldData;
    }

    private Vector3 CalculateEulerAngles(float[,] rotateMatrix)
    {
        float roll = Mathf.Atan2(rotateMatrix[2, 2], rotateMatrix[1, 2]);
        float pitch = -Mathf.Asin(rotateMatrix[0, 2]);
        float yaw = Mathf.Atan2(rotateMatrix[0, 0], rotateMatrix[0, 1]);

        return new Vector3(roll, pitch, yaw);
    }

    private float[,] RotateMatrix = new float[3, 3]; // 旋转矩阵
}
