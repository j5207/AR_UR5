using UnityEngine;
using HoloToolkit.Unity.InputModule;

public class MoveCommand : MonoBehaviour//, IManipulationHandler
{
    public GameObject primaryCubeOne;
    public GameObject primaryCubeTwo;
    Vector3 originalPosition;
    private Vector3 manipulationOriginalPosition = Vector3.zero;
    // Called by GazeGestureManager when the user performs a Select gesture
    void OnSelect()
    {
        primaryCubeOne.SetActive(false);
        primaryCubeTwo.SetActive(false);

    }

    //void IManipulationHandler.OnManipulationStarted(ManipulationEventData eventData)
    //{
    //    InputManager.Instance.PushModalInputHandler(gameObject);

    //    manipulationOriginalPosition = transform.position;
    //}

    //void IManipulationHandler.OnManipulationUpdated(ManipulationEventData eventData)
    //{
    //    // 4.a: Make this transform's position be the manipulationOriginalPosition + eventData.CumulativeDelta
    //    transform.position = manipulationOriginalPosition + eventData.CumulativeDelta;
    //}

    //void IManipulationHandler.OnManipulationCompleted(ManipulationEventData eventData)
    //{
    //    InputManager.Instance.PopModalInputHandler();
    //}

    //void IManipulationHandler.OnManipulationCanceled(ManipulationEventData eventData)
    //{
    //    InputManager.Instance.PopModalInputHandler();
    //}
}