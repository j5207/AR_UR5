using UnityEngine;

public class MyCommandsCubeTwo : MonoBehaviour
{
    public GameObject primaryCubeTwoPick;
    public GameObject primaryCubeTwo;
    public GameObject WorldCenter;

    Vector3 originalPosition;

    // Use this for initialization
    void Start()
    {
        // Grab the original local position of the sphere when the app starts.

    }

    void OnPick()
    {
        primaryCubeTwoPick.SetActive(true);
        //Debug.Log(gameObject.name + "Picked");
        originalPosition = this.transform.localPosition;

    }

    void OnPlace()
    {
        Debug.Log("****************");
        Debug.Log("Position of Primary Cube Two (yellow) is: " + primaryCubeTwo.transform.position.x + ", " + primaryCubeTwo.transform.position.y + ", " + primaryCubeTwo.transform.position.z);
        Debug.Log("Position of Picked Cube Two (purple) is: " + primaryCubeTwoPick.transform.position.x + ", " + primaryCubeTwoPick.transform.position.y + ", " + primaryCubeTwoPick.transform.position.z);
        Debug.Log("Position of World Center (0,0,0) is: " + WorldCenter.transform.position.x + ", " + WorldCenter.transform.position.y + ", " + WorldCenter.transform.position.z);
        //Debug.Log("Placed");
    }

    void OnSelect()
    {
        //Debug.Log("Select Gesture");
    }
}