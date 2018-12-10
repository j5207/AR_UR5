using UnityEngine;
using RosSharp.RosBridgeClient;

public class MyCommandsCubeOne : MonoBehaviour
{
    public GameObject primaryCubeOnePick;
    public GameObject primaryCubeOne;
    public GameObject WorldCenter;
    Vector3 originalPosition;
    private RosSocket ros = null;
    int publication_id;
    float i = 1.0f;
    float j = 3.0f;

    // Use this for initialization
    void Start()
    {
        // Grab the original local position
        //originalPosition = this.transform.localPosition;
        ros = new RosSocket("ws://192.168.1.140:9090");
        publication_id = ros.Advertise("/CHOICE", "std_msgs/String");
        while (i > 0.0f)
        {
            //publication_id = ros.Advertise("/CHOICE", "std_msgs/String");
            i -= Time.deltaTime;
            //publishn();
        }
        //publishn();
    }

    void Update()
    {
        //publishn();
    }

    void OnPick()
    {
        //Debug.Log(originalPosition);
        primaryCubeOnePick.SetActive(true);

    }

    void OnPlace()
    {
        Debug.Log("*******************");
        Debug.Log("Position of Primary Cube One (yellow) is: " + primaryCubeOne.transform.position.x + ", " + primaryCubeOne.transform.position.y + ", " + primaryCubeOne.transform.position.z);
        Debug.Log("Position of Picked Cube One (purple) is: " + primaryCubeOnePick.transform.position.x + ", " + primaryCubeOnePick.transform.position.y + ", " + primaryCubeOnePick.transform.position.z);
        Debug.Log("Position of World Center (0,0,0) is: " + WorldCenter.transform.position.x + ", " + WorldCenter.transform.position.y + ", " + WorldCenter.transform.position.z);
        Debug.Log("Rotation of World Center (0,0,0) is: " + WorldCenter.transform.eulerAngles);
        Debug.Log("Rotation of Primary Cube One (yellow) is: " + primaryCubeOne.transform.eulerAngles);
        publishn();
    }

    void publishn()
    {
        //Debug.Log("Pub No");
        StandardString message = new StandardString();
        message.data = "Pick Position: " + primaryCubeOne.transform.position.x + ", " + primaryCubeOne.transform.position.y + ", " + primaryCubeOne.transform.position.z + "  |  " + "Pick Rotation: " + primaryCubeOne.transform.eulerAngles + "  |  " + "Place Position: " + primaryCubeOnePick.transform.position.x + ", " + primaryCubeOnePick.transform.position.y + ", " + primaryCubeOnePick.transform.position.z + "  |  " + "Place Position: " + primaryCubeOnePick.transform.eulerAngles + "  |  " + "World Center: " + WorldCenter.transform.position.x + ", " + WorldCenter.transform.position.y + ", " + WorldCenter.transform.position.z + "  |  " + "World Center is: " + WorldCenter.transform.eulerAngles;
        ros.Publish(publication_id, message);
    }
}