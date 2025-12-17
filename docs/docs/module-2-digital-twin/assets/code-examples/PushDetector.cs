using UnityEngine;

/// <summary>
/// Collision detection and physics interaction script
/// Demonstrates Unity's physics callbacks and force application
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class PushDetector : MonoBehaviour
{
    [Header("Push Settings")]
    [Tooltip("Force magnitude to apply when pushing (Newtons)")]
    public float pushForce = 100f;

    [Tooltip("Direction to apply push force")]
    public Vector3 pushDirection = Vector3.forward;

    [Header("Collision Feedback")]
    [Tooltip("Enable visual feedback on collision")]
    public bool visualFeedback = true;

    [Tooltip("Enable audio feedback on collision")]
    public bool audioFeedback = false;

    [Tooltip("Audio clip to play on collision")]
    public AudioClip collisionSound;

    private Rigidbody rb;
    private Renderer objectRenderer;
    private Color originalColor;
    private AudioSource audioSource;

    private int collisionCount = 0;
    private float lastCollisionTime = 0f;

    void Start()
    {
        // Get required components
        rb = GetComponent<Rigidbody>();

        if (rb == null)
        {
            Debug.LogError("PushDetector requires a Rigidbody component!");
            enabled = false;
            return;
        }

        // Get renderer for visual feedback
        objectRenderer = GetComponent<Renderer>();
        if (objectRenderer != null)
        {
            originalColor = objectRenderer.material.color;
        }

        // Setup audio source if audio feedback is enabled
        if (audioFeedback)
        {
            audioSource = gameObject.AddComponent<AudioSource>();
            audioSource.playOnAwake = false;
            audioSource.clip = collisionSound;
        }

        Debug.Log($"PushDetector initialized on {gameObject.name}. Press P to apply push force.");
    }

    void Update()
    {
        // Press P to apply push force
        if (Input.GetKeyDown(KeyCode.P))
        {
            ApplyPush();
        }

        // Display collision statistics
        if (Input.GetKeyDown(KeyCode.C))
        {
            Debug.Log($"Collision Statistics for {gameObject.name}:");
            Debug.Log($"  Total collisions: {collisionCount}");
            Debug.Log($"  Last collision: {Time.time - lastCollisionTime:F2}s ago");
        }
    }

    /// <summary>
    /// Apply push force to the rigidbody
    /// </summary>
    void ApplyPush()
    {
        Vector3 force = pushDirection.normalized * pushForce;
        rb.AddForce(force, ForceMode.Impulse);

        Debug.Log($"Applied push force: {force} N to {gameObject.name}");
    }

    /// <summary>
    /// Called when collision begins
    /// </summary>
    void OnCollisionEnter(Collision collision)
    {
        collisionCount++;
        lastCollisionTime = Time.time;

        // Calculate collision impulse magnitude
        float impulseMagnitude = collision.impulse.magnitude;

        Debug.Log($"Collision detected on {gameObject.name}:");
        Debug.Log($"  Collided with: {collision.gameObject.name}");
        Debug.Log($"  Impact force: {impulseMagnitude:F2} N⋅s");
        Debug.Log($"  Contact points: {collision.contactCount}");
        Debug.Log($"  Relative velocity: {collision.relativeVelocity.magnitude:F2} m/s");

        // Visual feedback
        if (visualFeedback && objectRenderer != null)
        {
            StartCoroutine(FlashColor(Color.red, 0.2f));
        }

        // Audio feedback
        if (audioFeedback && audioSource != null && collisionSound != null)
        {
            // Vary volume based on impact magnitude
            float volume = Mathf.Clamp01(impulseMagnitude / 10f);
            audioSource.PlayOneShot(collisionSound, volume);
        }
    }

    /// <summary>
    /// Called while collision is ongoing
    /// </summary>
    void OnCollisionStay(Collision collision)
    {
        // Log continuous contact (throttled to every 0.5s)
        if (Time.time - lastCollisionTime > 0.5f)
        {
            Debug.Log($"Continuous contact with {collision.gameObject.name}");
            lastCollisionTime = Time.time;
        }
    }

    /// <summary>
    /// Called when collision ends
    /// </summary>
    void OnCollisionExit(Collision collision)
    {
        Debug.Log($"Collision ended with {collision.gameObject.name}");
    }

    /// <summary>
    /// Flash object color for visual feedback
    /// </summary>
    System.Collections.IEnumerator FlashColor(Color flashColor, float duration)
    {
        if (objectRenderer == null) yield break;

        objectRenderer.material.color = flashColor;
        yield return new WaitForSeconds(duration);
        objectRenderer.material.color = originalColor;
    }

    /// <summary>
    /// Visualize push direction in Scene view
    /// </summary>
    void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Vector3 start = transform.position;
        Vector3 end = start + pushDirection.normalized * 2f;
        Gizmos.DrawLine(start, end);
        Gizmos.DrawSphere(end, 0.1f);
    }

    // GUI for debugging
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(Screen.width - 310, 10, 300, 120));
        GUILayout.Label($"Push Detector - {gameObject.name}");
        GUILayout.Label($"Collisions: {collisionCount}");
        GUILayout.Label($"Velocity: {rb.velocity.magnitude:F2} m/s");
        GUILayout.Label("P: Apply push force");
        GUILayout.Label("C: Show collision stats");
        GUILayout.EndArea();
    }
}
