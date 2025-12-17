using UnityEngine;

/// <summary>
/// First-person camera controller for exploring Unity scenes
/// WASD for movement, mouse for looking around
/// </summary>
public class CameraController : MonoBehaviour
{
    [Header("Movement Settings")]
    [Tooltip("Camera movement speed in meters per second")]
    public float moveSpeed = 5.0f;

    [Tooltip("Sprint multiplier (hold Shift)")]
    public float sprintMultiplier = 2.0f;

    [Header("Look Settings")]
    [Tooltip("Mouse sensitivity for camera rotation")]
    public float lookSensitivity = 2.0f;

    [Tooltip("Maximum vertical look angle (degrees)")]
    public float maxLookAngle = 90.0f;

    private float rotationX = 0f;
    private float rotationY = 0f;

    void Start()
    {
        // Lock cursor to center of screen
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;

        // Initialize rotation from current camera orientation
        Vector3 currentRotation = transform.eulerAngles;
        rotationY = currentRotation.y;
        rotationX = currentRotation.x;
    }

    void Update()
    {
        HandleMovement();
        HandleLook();
        HandleCursorToggle();
    }

    void HandleMovement()
    {
        // Get input axes
        float horizontal = Input.GetAxis("Horizontal"); // A/D or Left/Right arrows
        float vertical = Input.GetAxis("Vertical");     // W/S or Up/Down arrows

        // Calculate movement direction (relative to camera orientation)
        Vector3 moveDirection = transform.right * horizontal + transform.forward * vertical;

        // Apply sprint modifier if Shift is held
        float currentSpeed = moveSpeed;
        if (Input.GetKey(KeyCode.LeftShift))
        {
            currentSpeed *= sprintMultiplier;
        }

        // Move camera
        transform.position += moveDirection * currentSpeed * Time.deltaTime;

        // Vertical movement (Q/E for up/down)
        if (Input.GetKey(KeyCode.Q))
        {
            transform.position += Vector3.down * currentSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.E))
        {
            transform.position += Vector3.up * currentSpeed * Time.deltaTime;
        }
    }

    void HandleLook()
    {
        // Get mouse input
        float mouseX = Input.GetAxis("Mouse X") * lookSensitivity;
        float mouseY = Input.GetAxis("Mouse Y") * lookSensitivity;

        // Update rotation
        rotationY += mouseX;
        rotationX -= mouseY;

        // Clamp vertical rotation to prevent camera flipping
        rotationX = Mathf.Clamp(rotationX, -maxLookAngle, maxLookAngle);

        // Apply rotation to camera
        transform.eulerAngles = new Vector3(rotationX, rotationY, 0f);
    }

    void HandleCursorToggle()
    {
        // Press Escape to unlock cursor
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }

        // Click to lock cursor again
        if (Input.GetMouseButtonDown(0) && Cursor.lockState == CursorLockMode.None)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
    }
}
