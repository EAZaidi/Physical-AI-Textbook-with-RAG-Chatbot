using UnityEngine;

/// <summary>
/// Simple joint position controller for humanoid robot
/// Demonstrates Unity's ArticulationBody API for robot control
/// </summary>
public class SimpleJointController : MonoBehaviour
{
    [Header("Joint References")]
    [Tooltip("Assign ArticulationBody components for each joint")]
    public ArticulationBody leftHipJoint;
    public ArticulationBody rightHipJoint;
    public ArticulationBody leftKneeJoint;
    public ArticulationBody rightKneeJoint;

    [Header("Control Settings")]
    [Tooltip("Target position for joint in degrees")]
    public float targetAngle = 0f;

    [Tooltip("Joint stiffness (N⋅m/rad)")]
    public float stiffness = 10000f;

    [Tooltip("Joint damping (N⋅m⋅s/rad)")]
    public float damping = 1000f;

    [Tooltip("Maximum force to apply (N⋅m)")]
    public float maxForce = 500f;

    private bool isControlling = false;

    void Start()
    {
        // Validate joint references
        if (leftHipJoint == null || rightHipJoint == null ||
            leftKneeJoint == null || rightKneeJoint == null)
        {
            Debug.LogError("Not all joints assigned! Please assign all ArticulationBody references in Inspector.");
            enabled = false;
            return;
        }

        // Initialize joint drives
        InitializeJointDrive(leftHipJoint);
        InitializeJointDrive(rightHipJoint);
        InitializeJointDrive(leftKneeJoint);
        InitializeJointDrive(rightKneeJoint);

        Debug.Log("SimpleJointController initialized. Press SPACE to toggle control.");
    }

    void InitializeJointDrive(ArticulationBody joint)
    {
        // Get the existing drive configuration
        ArticulationDrive drive = joint.xDrive;

        // Configure PD controller parameters
        drive.stiffness = stiffness;
        drive.damping = damping;
        drive.forceLimit = maxForce;

        // Set initial target position
        drive.target = 0f; // Start at neutral position

        // Apply drive configuration back to joint
        joint.xDrive = drive;
    }

    void Update()
    {
        // Toggle control on/off with SPACE
        if (Input.GetKeyDown(KeyCode.Space))
        {
            isControlling = !isControlling;
            Debug.Log($"Joint control: {(isControlling ? "ON" : "OFF")}");
        }

        if (isControlling)
        {
            HandleKeyboardControl();
        }
    }

    void HandleKeyboardControl()
    {
        // Arrow keys control hip joints
        if (Input.GetKey(KeyCode.UpArrow))
        {
            SetJointTarget(leftHipJoint, targetAngle);
            SetJointTarget(rightHipJoint, targetAngle);
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            SetJointTarget(leftHipJoint, -targetAngle);
            SetJointTarget(rightHipJoint, -targetAngle);
        }

        // Left/Right arrows control individual legs
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            SetJointTarget(leftKneeJoint, targetAngle);
        }
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            SetJointTarget(rightKneeJoint, targetAngle);
        }

        // R key resets all joints to neutral
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetAllJoints();
        }
    }

    void SetJointTarget(ArticulationBody joint, float targetDegrees)
    {
        // Get current drive configuration
        ArticulationDrive drive = joint.xDrive;

        // Set target position (convert degrees to radians if needed)
        // Note: Unity ArticulationBody uses degrees for revolute joints
        drive.target = targetDegrees;

        // Apply updated drive back to joint
        joint.xDrive = drive;
    }

    void ResetAllJoints()
    {
        SetJointTarget(leftHipJoint, 0f);
        SetJointTarget(rightHipJoint, 0f);
        SetJointTarget(leftKneeJoint, 0f);
        SetJointTarget(rightKneeJoint, 0f);

        Debug.Log("All joints reset to neutral position.");
    }

    // GUI for debugging
    void OnGUI()
    {
        if (!isControlling) return;

        GUILayout.BeginArea(new Rect(10, 10, 300, 150));
        GUILayout.Label("Joint Controller (ACTIVE)");
        GUILayout.Label("↑/↓: Hip joints");
        GUILayout.Label("←/→: Knee joints");
        GUILayout.Label("R: Reset to neutral");
        GUILayout.Label("SPACE: Toggle control");
        GUILayout.Label($"Target Angle: {targetAngle}°");
        GUILayout.EndArea();
    }
}
