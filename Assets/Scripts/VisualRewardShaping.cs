public static class VisualRewardShaping
{
    public const float CameraProgressScale = 0.08f;
    public const float AlignmentProgressScale = 0.08f;

    public static float CalculateProgress(
        float previousCameraError,
        float currentCameraError,
        float previousAlignment,
        float currentAlignment)
    {
        float cameraProgress = previousCameraError - currentCameraError;
        float alignmentProgress = currentAlignment - previousAlignment;
        return cameraProgress * CameraProgressScale +
               alignmentProgress * AlignmentProgressScale;
    }
}
