using UnityEngine;
using UnityEngine.UI;

public class InputDebugUI : MonoBehaviour
{
    public ControlActions input;

    public Slider rollSlider;
    public Slider pitchSlider;
    public Slider yawSlider;
    public Slider throttleSlider;

    void Start()
    {
        rollSlider.minValue = -1f;
        rollSlider.maxValue = 1f;

        pitchSlider.minValue = -1f;
        pitchSlider.maxValue = 1f;

        yawSlider.minValue = -1f;
        yawSlider.maxValue = 1f;

        throttleSlider.minValue = 0f;
        throttleSlider.maxValue = 1f;
    }

    void Update()
    {
        rollSlider.value = input.Roll;
        pitchSlider.value = input.Pitch;
        yawSlider.value = input.Yaw;
        throttleSlider.value = input.Throttle;
    }

}
