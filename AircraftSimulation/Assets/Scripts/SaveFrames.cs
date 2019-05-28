using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SaveFrames : MonoBehaviour
{
    string folder = "frames";
    int frameCount;
    public bool saveFrames = false;

    // Start is called before the first frame update
    void Start()
    {
        if(saveFrames) {
            if(System.IO.Directory.Exists("Assets/DataCollection/" + folder))
            {
                System.IO.Directory.Delete("Assets/DataCollection/" + folder, true);
            }
            System.IO.Directory.CreateDirectory("Assets/DataCollection/" + folder);
            frameCount = 0;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(saveFrames) {
            string fileName = string.Format("{0:000000}", frameCount);
            ScreenCapture.CaptureScreenshot("Assets/DataCollection/" + folder + "/" + fileName + ".png");
            frameCount++;
        }
    }
}
