using UnityEngine;

public class WorldCursor : MonoBehaviour
{
    private MeshRenderer meshRenderer;
    public GameObject primaryCubeOne;
    public GameObject primaryCubeOneHlt;
    public GameObject primaryCubeTwo;
    public GameObject primaryCubeTwoHlt;

    // Use this for initialization
    void Start()
    {
        // Grab the mesh renderer that's on the same object as this script.
        meshRenderer = this.gameObject.GetComponentInChildren<MeshRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
        // Do a raycast into the world based on the user's
        // head position and orientation.
        var headPosition = Camera.main.transform.position;
        var gazeDirection = Camera.main.transform.forward;



        RaycastHit hitInfo;

        if (Physics.Raycast(headPosition, gazeDirection, out hitInfo))
        {

            // If the raycast hit a hologram...
            // Display the cursor mesh.
            meshRenderer.enabled = true;

            // Move thecursor to the point where the raycast hit.
            this.transform.position = hitInfo.point;

            // Rotate the cursor to hug the surface of the hologram.
            this.transform.rotation = Quaternion.FromToRotation(Vector3.up, hitInfo.normal);

            //print name of target hit
            if (hitInfo.collider.name == "primaryCubeOne")
            {
                //Debug.Log(hitInfo.collider.name);
                primaryCubeOne.SetActive(false);
                primaryCubeOneHlt.SetActive(true);
            }
            else if (hitInfo.collider.name == "primaryCubeTwo")
            {
                primaryCubeTwo.SetActive(false);
                primaryCubeTwoHlt.SetActive(true);
            }
//            Debug.Log(hitInfo.collider.name != null);

            //change color of hit hologram
//            if (hitI)
        }
        else
        {
            // If the raycast did not hit a hologram, hide the cursor mesh.
            meshRenderer.enabled = false;
            primaryCubeOne.SetActive(true);
            primaryCubeOneHlt.SetActive(false);
            primaryCubeTwo.SetActive(true);
            primaryCubeTwoHlt.SetActive(false);
        }
    }
}