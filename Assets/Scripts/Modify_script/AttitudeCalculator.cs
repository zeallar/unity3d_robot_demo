using System.Collections.Generic;
using UnityEngine;

public class AttitudeCalculator : MonoBehaviour
{
    private bool run = true;                 // 开启计算标志
    private bool useMag = true;              // 使用地磁标志

    // 接口数据
    private Vector3 accData = Vector3.zero;
    private Vector3 gyroData = Vector3.zero;
    private Vector3 magData = Vector3.zero;

    // 参数
    private float errorKi = 1.25f;
    private float errorKp = 5.5f;
    private float correctKp = 0.4f;

    // 中间变量
    private Vector3 error = Vector3.zero;
    private Vector3 errorIntegral = Vector3.zero;
    private Quaternion quaternion = Quaternion.identity;

    // 数据
    private Matrix4x4 rotateMatrix = Matrix4x4.zero;
    private Vector3 magWorld = Vector3.zero;
    private Vector3 accWorld = Vector3.zero;
    private Vector3 magCorrect = Vector3.zero;
    private Vector3 accCorrect = Vector3.zero;
    private Vector3 gyroCorrect = Vector3.zero;
    public Vector3 eulerAngles = Vector3.zero;  // public to allow access from other scripts

    // 初始化
    public void InitAttitude()
    {
        run = true;
        useMag = true;

        accData = Vector3.zero;
        gyroData = Vector3.zero;
        magData = Vector3.zero;

        errorKi = 1.25f;
        errorKp = 5.5f;
        correctKp = 0.4f;

        error = Vector3.zero;
        errorIntegral = Vector3.zero;

        quaternion = Quaternion.identity;

        rotateMatrix = Matrix4x4.identity;
        magWorld = Vector3.zero;
        accWorld = Vector3.zero;
        magCorrect = Vector3.zero;
        accCorrect = Vector3.zero;
        gyroCorrect = Vector3.zero;

        eulerAngles = Vector3.zero;
    }

    public void CalculateAttitude(float cycle, Vector3 accInput, Vector3 gyroInput, Vector3 magInput)
    {
        if (!run) return;

        // 更新接口数据
        accData = accInput * 1000.0f;
        gyroData = gyroInput;
        magData = new Vector3(magInput.y, magInput.x, -magInput.z);

        // 磁力计处理
        if (useMag)
        {
            float length = Mathf.Sqrt(magCorrect.x * magCorrect.x + magCorrect.y * magCorrect.y);
            if (magCorrect != Vector3.zero && length != 0)
            {
                float magYaw = Mathf.Atan2(magCorrect.y / length, magCorrect.x / length);
                if (rotateMatrix[2, 2] > 0.0f)
                {
                    float magYawBias = correctKp * TranslateAngle(eulerAngles.z - magYaw);
                    magYawBias = Mathf.Clamp(magYawBias, -360, 360);
                }
            }
        }

        // 加速度计处理
        float accLength = accData.magnitude;
        if (Mathf.Abs(accData.x) < 1050.0f && Mathf.Abs(accData.y) < 1050.0f && Mathf.Abs(accData.z) < 1050.0f && 800.0f < accLength && accLength < 1200.0f)
        {
            Vector3 accNorm = accData / accLength;
            error.x = (accNorm.y * rotateMatrix[2, 2] - accNorm.z * rotateMatrix[1, 2]);
            error.y = (accNorm.z * rotateMatrix[0, 2] - accNorm.x * rotateMatrix[2, 2]);
            error.z = (accNorm.x * rotateMatrix[1, 2] - accNorm.y * rotateMatrix[0, 2]);

            error = Vector3.Lerp(error, error, 1.0f * 3.14f * cycle);

            errorIntegral += error * errorKi * cycle;
            errorIntegral = Vector3.ClampMagnitude(errorIntegral, 0.035f);
        }
        else
        {
            error = Vector3.zero;
        }

        // 开始修正陀螺仪值
        gyroCorrect.x = ((gyroData.x) - rotateMatrix[0, 2] * correctKp) * 0.01745329f + (errorKp * error.x + errorIntegral.x);
        gyroCorrect.y = ((gyroData.y) - rotateMatrix[1, 2] * correctKp) * 0.01745329f + (errorKp * error.y + errorIntegral.y);
        gyroCorrect.z = ((gyroData.z) - rotateMatrix[2, 2] * correctKp) * 0.01745329f + (errorKp * error.z + errorIntegral.z);

        // 一阶龙格库塔更新四元数值
        quaternion = Quaternion.Euler(gyroCorrect * cycle / 2.0f) * quaternion;
        quaternion.Normalize();

        // 计算旋转矩阵
        rotateMatrix = Matrix4x4.Rotate(quaternion);

        // 计算世界坐标系下的磁力计和加速度计值
        magWorld = rotateMatrix.MultiplyPoint3x4(magData);
        accWorld = rotateMatrix.MultiplyPoint3x4(accData);

        // 求解欧拉角
        eulerAngles.x = Mathf.Atan2(rotateMatrix[2, 2], rotateMatrix[1, 2]) * Mathf.Rad2Deg;
        eulerAngles.y = -Mathf.Asin(rotateMatrix[0, 2]) * Mathf.Rad2Deg;
        eulerAngles.z = Mathf.Atan2(rotateMatrix[0, 0], rotateMatrix[0, 1]) * Mathf.Rad2Deg;

        // 计算矫正后的加速度和磁场
        accCorrect.x = accWorld.x * Mathf.Cos(eulerAngles.z * Mathf.Deg2Rad) + accWorld.y * Mathf.Sin(eulerAngles.z * Mathf.Deg2Rad);
        accCorrect.y = -accWorld.x * Mathf.Sin(eulerAngles.z * Mathf.Deg2Rad) + accWorld.y * Mathf.Cos(eulerAngles.z * Mathf.Deg2Rad);
        accCorrect.z = accWorld.z;

        if (useMag)
        {
            Vector3 refV = new Vector3(rotateMatrix[0, 2], rotateMatrix[1, 2], rotateMatrix[2, 2]);
            Vector3 magTmp = new Vector3(magData.x, magData.y, magData.z);
            Simple3DTransform(refV, magTmp, out magCorrect);
        }
    }

    private float TranslateAngle(float angle)
    {
        return Mathf.DeltaAngle(angle, 0);
    }

    private void Simple3DTransform(Vector3 refVector, Vector3 input, out Vector3 output)
    {
        float hTmpX = Mathf.Sqrt(refVector.z * refVector.z + refVector.y * refVector.y);
        float hTmpY = Mathf.Sqrt(refVector.z * refVector.z + refVector.x * refVector.x);
        float pn = refVector.z < 0 ? -1 : 1;

        output = new Vector3
        {
            x = hTmpX * input.x - pn * refVector.x * input.z,
            y = pn * hTmpY * input.y - refVector.y * input.z,
            z = refVector.x * input.x + refVector.y * input.y + refVector.z * input.z
        };
    }
}
