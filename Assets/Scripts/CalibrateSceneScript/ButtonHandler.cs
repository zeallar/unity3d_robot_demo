using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
public class ButtonHandler : MonoBehaviour
{
    public Text messageText;    // 引用Text组件
    public Button noButton;     // 引用NoButton组件
    public Button yesButton;    // 引用YesButton组件
    public Button continueButton;   // 引用continueButton组件
    public Text countdownText;  // 引用倒计时Text组件

    private List<string> instructions = new List<string>(); // 存储指令文本的列表
    private int indexText = 0;

    private SensorDataProcessor sensorDataProcessor;

    void Start()
    {
        // 添加指令文本到列表中
        instructions.Add("请保持静止");
        instructions.Add("请向前一步");
        instructions.Add("请后退一步");
        instructions.Add("请左转");
        instructions.Add("请右转");
        // 为YesButton添加点击事件监听器
        yesButton.onClick.AddListener(OnYesButtonClick);
        // 为continueButton添加点击事件监听器
        continueButton.onClick.AddListener(OnContinueButtonClick);

        // 为noButton添加点击事件监听器
        noButton.onClick.AddListener(OnNoButtonClick);

        sensorDataProcessor = FindObjectOfType<SensorDataProcessor>();
        if (sensorDataProcessor == null)
        {
            Debug.LogError("SensorDataProcessor not found in the scene.");
        }
    }
    // NoButton点击事件处理程序
    void OnNoButtonClick()
    {
        if (AllOffsetsStored())
        {
            SceneManager.LoadScene("ChaMoveExample");
        }
        else
        {
            messageText.text = "所有动作的偏移量必须在进行场景跳转前存储。";
        }
    }
    // YesButton点击事件处理程序
    void OnYesButtonClick()
    {
        // 启动协程
        StartCoroutine(WaitAndChangeText(instructions[indexText]));
    }
    // continueButton点击事件处理程序
    void OnContinueButtonClick()
    {
        // 更新索引值
        indexText++;

        // 如果索引值超过指令列表的长度，则重置为0
        if (indexText >= instructions.Count)
        {
            indexText = 0;
            // 加载场景

        }
        UDPReceiver.Instance.clearCacheData();
        // 启动协程显示下一个指令
        StartCoroutine(WaitAndChangeText(instructions[indexText]));
    }

    // 协程函数
    IEnumerator WaitAndChangeText(string newText)
    {
        // 隐藏NoButton和YesButton
        noButton.gameObject.SetActive(false);
        yesButton.gameObject.SetActive(false);
        continueButton.gameObject.SetActive(false);
        // 改变Text内容
        messageText.text = "准备";

        // 等待一秒
        yield return new WaitForSeconds(1f);

        // 改变Text文字大小
        messageText.fontSize = 24; // 设置为你需要的字体大小

        // 改变Text内容
        messageText.text = "开始";
        // 等待一秒
        yield return new WaitForSeconds(1f);

        // 改变Text内容
        messageText.text = newText;

        // 显示倒计时
        countdownText.gameObject.SetActive(true);

        // 开始接收UDP数据
        UDPReceiver.Instance.StartReceiving();
        // 倒计时从 3 开始
        for (int i = 3; i >= 0; i--)
        {
            // 更新倒计时文本
            countdownText.text = i.ToString();

            // 等待一秒
            yield return new WaitForSeconds(1f);
        }
        // 停止接收UDP数据
        UDPReceiver.Instance.StopReceiving();

        // 关闭倒计时文本
        countdownText.gameObject.SetActive(false);

        // 获取接收到的数据
        UDPReceiver.Instance.GetBufferedData(out List<Vector3> accData, out List<Vector3> gyroData, out List<Vector3> magData);

        // 处理接收到的数据
        sensorDataProcessor.ProcessData(accData, gyroData, magData, newText); // 传递当前动作名称

        // 显示继续按钮
        continueButton.gameObject.SetActive(true);
    }
    private bool AllOffsetsStored()
    {
        foreach (var action in new[] { "请保持静止", "请向前一步", "请后退一步", "请左转", "请右转" })
        {
            if (!sensorDataProcessor.HasActionOffset(action))
            {
                return false;
            }
        }
        return true;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
