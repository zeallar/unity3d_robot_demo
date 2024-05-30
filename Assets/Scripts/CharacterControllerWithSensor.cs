using UnityEngine;

public class CharacterControllerWithSensor : MonoBehaviour
{
    public float speed = 3; // 角色移动速度
    public float turnSpeed = 10;
    public float threshold = 0.3f; // 控制移动的加速度阈值
    private bool isGrounded;
    private bool canJump;
    public float jumpForce = 50f;

    private Vector3 acceleration;
    private Vector3 calibrationOffset;

    private Vector3 move;
    private float forwardAmount;
    private float turnAmount;

    private Animator anim;
    private Rigidbody rigid;
    // 使用简单的移动平均滤波
    private Vector3[] accelerationBuffer = new Vector3[10];
    private int bufferIndex = 0;

    private float previousAccelX; // 用于存储前一帧的加速度值
    private float previousAccelZ; // 用于存储前一帧的加速度值

    private readonly object updateLock = new object();//互斥锁
    // 用于陀螺仪数据的平滑处理
    private const int gyroBufferLength = 10;
    private Vector3[] gyroBuffer = new Vector3[gyroBufferLength];
    private int gyroBufferIndex = 0;
    //反向惯性力过滤
    private Vector3 lastGyroValue = Vector3.zero;
    private Vector3 currentGyroValue = Vector3.zero;
    private bool isRotating = false;
    private Vector3 ignoreNegative = Vector3.zero;
    void Start()
    {
        canJump = false;
        anim = GetComponent<Animator>();
        rigid = GetComponent<Rigidbody>();
        // 预处理：计算静止状态下的校准偏移量
        calibrationOffset = new Vector3(0,0, 0); // 使用静止状态数据的平均值
    }

    void Update()
    {
        lock (updateLock)
        {
            // 检查是否有新的加速度数据
            if (SensorData.Instance.hasNewAccelData)
            {
                // 从传感器获取加速度数据
                Vector3 rawAcceleration = GetSensorData();

                // 预处理：减去校准偏移量
                acceleration = rawAcceleration - calibrationOffset;

                // 滤波：移动平均滤波
                accelerationBuffer[bufferIndex] = acceleration;
                bufferIndex = (bufferIndex + 1) % accelerationBuffer.Length;
                if (bufferIndex >= 9) {
                    canJump = true;
                }

                Vector3 smoothedAcceleration = Vector3.zero;
                foreach (Vector3 acc in accelerationBuffer)
                {
                    smoothedAcceleration += acc;
                }
                smoothedAcceleration /= accelerationBuffer.Length;

                // 映射到角色控制
                Vector3 moveDirection = Vector3.zero;

                string accelLog = $"加速度X: {smoothedAcceleration.x}, Y: {smoothedAcceleration.y}, Z: {smoothedAcceleration.z}";
                //Logger.Info(accelLog);

                if (Mathf.Abs(smoothedAcceleration.x) > threshold)
                {
                    // 平滑处理
                    moveDirection.x = SmoothAccelerometerValue(smoothedAcceleration.x);
                    // 饱和限制
                    moveDirection.x = Mathf.Clamp(moveDirection.x, -1f, 1f);

                    accelLog = $"---------------------------------------移动X: {moveDirection.x}, Y: {moveDirection.y}, Z: {moveDirection.z}";
                    //Logger.Info(accelLog);
                }
                if (Mathf.Abs(smoothedAcceleration.z) > threshold)
                {
                    moveDirection.z = SmoothAccelerometerValue(smoothedAcceleration.z); // 对 accelZ 进行平滑处理
                    moveDirection.z = Mathf.Clamp(moveDirection.z, -1f, 1f); // 对 accelZ 进行饱和限制
                    accelLog = $"---------------------------------------移动z: {moveDirection.x}, Y: {moveDirection.y}, Z: {moveDirection.z}";
                    //Logger.Info(accelLog);
                }

                // 角色移动
                //transform.Translate(moveDirection * speed * Time.deltaTime);
                // 世界坐标系move向量
                move = new Vector3(moveDirection.z, 0, moveDirection.x);//x、z轴反转
                Vector3 localMove = transform.InverseTransformVector(move);
                forwardAmount = localMove.z;
                turnAmount = Mathf.Atan2(localMove.x, localMove.z);

                // 检测地面
                isGrounded = Physics.Raycast(transform.position + Vector3.up * 0.1f, Vector3.down, 0.2f);
                if (isGrounded && smoothedAcceleration.y >-8.5&& canJump&& moveDirection.x==0&& moveDirection.z==0)
                {
                    rigid.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
                    anim.SetTrigger("Jump");  // 触发跳跃动画
                }

                UpdateAnim();
                // 重置标志位
                SensorData.Instance.hasNewAccelData = false;
            }
        }
    }
    private void FixedUpdate()
    {

        rigid.velocity = new Vector3(forwardAmount * transform.forward.x * speed, rigid.velocity.y, forwardAmount * transform.forward.z * speed);
        //rigid.MoveRotation(rigid.rotation * Quaternion.Euler(0, turnAmount * turnSpeed, 0));//利用加速度计转向
        // 处理角色的旋转
        GyroControl();
    }
    void UpdateAnim()
    {
        anim.SetFloat("Speed", move.magnitude);
    }
    Vector3 GetSensorData()
    {
        // 从SensorData类中获取加速度数据
        SensorData.Instance.GetAccelData(out float accelX, out float accelY, out float accelZ);


        // 将获取的数据除以 100
        accelX /= 100f;
        accelY /= 100f;
        accelZ /= 100f;
        string accelLog = $"获取 饱前 传感器加速度X: {accelX}, Y: {accelY}, Z: {accelZ}";
        //Logger.Info(accelLog);
        return new Vector3(accelX, accelY, accelZ);
    }
    Vector3 GetGyroData()
    {
        SensorData.Instance.GetGyroData(out float gyroX, out float gyroY, out float gyroZ);

        return new Vector3(gyroX, gyroY, gyroZ);
    }
    void GyroControl() {
        string GyroLog;
        // 获取陀螺仪数据
        currentGyroValue = GetGyroData();

        // 检测是否开始旋转
        if (!isRotating)
        {
            if (Mathf.Abs(currentGyroValue.x) > 1f || Mathf.Abs(currentGyroValue.y) > 1f || Mathf.Abs(currentGyroValue.z) > 1f)
            {
                isRotating = true;
                ignoreNegative = Vector3.zero;
                GyroLog = $"旋转 陀螺仪XX: {currentGyroValue.x}, Y: {currentGyroValue.y}, Z: {currentGyroValue.z}";
                Logger.Info(GyroLog);
            }
        }
        else
        {
            // 检测旋转是否结束
            if (currentGyroValue.magnitude <= 0.01f && lastGyroValue.magnitude > currentGyroValue.magnitude)
            {
                // 标记需要忽略的轴的反向旋转
                ignoreNegative = new Vector3(
                    lastGyroValue.x > 0 ? 1 : 0,
                    lastGyroValue.y > 0 ? 1 : 0,
                    lastGyroValue.z > 0 ? 1 : 0
                );
                GyroLog = $"旋转 陀螺仪XX: {currentGyroValue.x}, Y: {currentGyroValue.y}, Z: {currentGyroValue.z}";
                Logger.Info(GyroLog);
                isRotating = false;
            }
        }

        // 处理旋转数据
        if (isRotating)
        {
            Vector3 effectiveRotation = new Vector3(
                ignoreNegative.x == 0 ? currentGyroValue.x : 0,
                ignoreNegative.y == 0 ? currentGyroValue.y : 0,
                ignoreNegative.z == 0 ? currentGyroValue.z : 0
            );
            RotateCharacter(effectiveRotation);
        }

        lastGyroValue = currentGyroValue; // 更新上一帧的值
    }
    void RotateCharacter(Vector3 gyroData)
    {
        // 将陀螺仪数据存入缓冲区
        gyroBuffer[gyroBufferIndex] = gyroData;
        gyroBufferIndex = (gyroBufferIndex + 1) % gyroBufferLength;

        // 计算缓冲区内陀螺仪数据的平均值
        Vector3 smoothedGyroData = Vector3.zero;
        foreach (Vector3 gyro in gyroBuffer)
        {
            smoothedGyroData += gyro;
        }
        smoothedGyroData /= gyroBufferLength;

        // 使用平滑后的陀螺仪数据进行旋转
        float rotationY = smoothedGyroData.z * turnSpeed * Time.deltaTime;
        Quaternion deltaRotation = Quaternion.Euler(0, rotationY, 0);
        rigid.MoveRotation(rigid.rotation * deltaRotation);
    }
    //平滑处理的技术
    float SmoothAccelerometerValue(float currentAccelX)
    {
        // 使用简单的平滑处理方法，例如线性插值
        float smoothedAccelX = Mathf.Lerp(previousAccelX, currentAccelX, 0.5f); // 这里的 0.5f 可以根据需要进行调整

        // 更新前一帧的加速度值
        previousAccelX = smoothedAccelX;

        return smoothedAccelX;
    }
}
