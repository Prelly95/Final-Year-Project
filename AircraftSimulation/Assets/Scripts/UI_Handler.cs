using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UI_Handler : MonoBehaviour
{
    
    void Update()
    {
        if (Input.GetKeyUp("n"))
        {
            step();
        }
    }

    public void step()
	{
        Time.timeScale = 1;
    }
}
