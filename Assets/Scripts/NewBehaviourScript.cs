using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewBehaviourScript : MonoBehaviour
{
    public float speed = 3;
    public float turnSpeed = 10;
    public float jumpForce = 50f;

    private Animator anim;
    private Rigidbody rigid;
    private Vector3 move;
    private bool isGrounded;
    private float forwardAmount;
    private float turnAmount;

    private RotUDPReceiver RotUDPReceiver;
    //private float originalY;  // 原始Y位置
    //public SerialPortController serialPortController;
    void Start()
    {
        anim = GetComponent<Animator>();
        rigid = GetComponent<Rigidbody>();
        //originalY = transform.position.y;  // 保存角色初始的Y位置

        // 获取串口控制器
        //serialPortController = FindObjectOfType<SerialPortController>();

        //获取UDP
        RotUDPReceiver = FindObjectOfType<RotUDPReceiver>();
        if (RotUDPReceiver == null)
        {
            Debug.LogError("RotUDPReceiver not found in the scene.");
        }
    }

    void Update()
    {

        if (false)
        {
            // 校验通过后才开始执行的逻辑
            Debug.Log("所有校验通过，开始执行主逻辑！");
            // 在这里添加你的主逻辑代码

            // 从共享数据中读取传感器数据
            SensorData.Instance.GetTemp(out float temp);
            SensorData.Instance.GetAccelData(out float accelX, out float accelY, out float accelZ);
            SensorData.Instance.GetGyroData(out float gyroX, out float gyroY, out float gyroZ);
            SensorData.Instance.GetMagData(out float magX, out float magY, out float magZ);

            // 根据传感器数据控制角色
            // 这里简单地将加速度映射到角色的移动

            // 世界坐标系move向量
            move = new Vector3(accelX, 0, accelZ);
            Vector3 localMove = transform.InverseTransformVector(move);
            forwardAmount = localMove.z;
            turnAmount = Mathf.Atan2(localMove.x, localMove.z);

            // 检测地面
            isGrounded = Physics.Raycast(transform.position + Vector3.up * 0.1f, Vector3.down, 0.2f);
            //Debug.Log("Update: isGrounded = " + isGrounded);

            if (isGrounded && accelY > 10)
            {
                rigid.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
                anim.SetTrigger("Jump");  // 触发跳跃动画
            }

            UpdateAnim();
        }
        else
        {
            Debug.Log("等待校验完成...");
        }
       
    }

    private void FixedUpdate()
    {

        rigid.velocity = new Vector3(forwardAmount * transform.forward.x * speed, rigid.velocity.y, forwardAmount * transform.forward.z * speed);
        rigid.MoveRotation(rigid.rotation * Quaternion.Euler(0, turnAmount * turnSpeed, 0));
    }

    void UpdateAnim()
    {
        anim.SetFloat("Speed", move.magnitude);
    }
}
