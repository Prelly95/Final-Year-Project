using UnityEngine;
using System;
using System.IO;
using System.Text;

public class RayCast : MonoBehaviour
{
	static float FOV = Mathf.PI / 2F;
	static int numberOfRays = 30;
	int count = 0;
	Vector3[,] dir = new Vector3[numberOfRays, numberOfRays];

	//Data Capture directory
	static DirectoryInfo currentDir = new DirectoryInfo(".");
	static string dataPath = currentDir.FullName + "/Assets/DataCollection/";

	// Data file names
	static string lidarFile = "lidar.txt";
	static string poseFile = "pose.txt";
	static string velocityFile = "velocity.txt";

	static string lidarFilePath = dataPath + lidarFile;
	static string poseFilePath = dataPath + poseFile;
	static string velocityFilePath = dataPath + velocityFile;
	//Data Streams
	public StreamWriter lidarStream;
	public StreamWriter poseStream;
	public StreamWriter velocityStream;

	public Rigidbody UAVrb;

    public bool debug = true;

	void Start()
	{
		if (!Directory.Exists(dataPath))
		{
			Directory.CreateDirectory(dataPath);
		}
		if (File.Exists(lidarFilePath))
		{
			File.Delete(lidarFilePath);
		}

		//Get rigid body for velocity
		Rigidbody UAVrb = GetComponent<Rigidbody>();

		// Create the data text files
		lidarStream = File.CreateText(lidarFilePath);
		poseStream = File.CreateText(poseFilePath);
		velocityStream = File.CreateText(velocityFilePath);

		for (int ii = 0; ii < numberOfRays; ii++)
		{
			for (int jj = 0; jj < numberOfRays; jj++)
			{
				float theta = jj * ((FOV) / (numberOfRays - 1)) - FOV / 2;
				float phi = ii * ((FOV) / (numberOfRays - 1)) + FOV / 2;

				dir[ii, jj].x = Mathf.Cos(theta) * Mathf.Cos(phi);
				dir[ii, jj].z = Mathf.Cos(theta) * Mathf.Sin(phi);
				dir[ii, jj].y = -Mathf.Sin(theta);
			}
		}
	}

	void FixedUpdate()
	{
		string lidarData;
		string tVelocityData;
		string aVelocityData;
        string poseData;

        Vector3 transVelocity;
        Vector3 rotVelocity;
        Vector3 pose;

        RaycastHit hit;

		for (int ii = 0; ii < numberOfRays; ii++)
		{
            lidarStream.Write("{0}", count);
            for (int jj = 0; jj < numberOfRays; jj++)
			{
				if (Physics.Raycast(transform.position, transform.rotation * dir[ii, jj], out hit, Mathf.Infinity))
				{
					// Write to the file:
                    if(debug) {
                        Debug.DrawRay(transform.position, transform.rotation * dir[ii, jj] * hit.distance, Color.green);
                    }
                    lidarData = string.Format("\t{0}", hit.distance);
				}
				else
				{
                    if(debug) {
                        Debug.DrawRay(transform.position, transform.rotation * dir[ii, jj] * 1000, Color.red);
                    }
                    lidarData = string.Format("\t{0}", 0);
				}
                lidarStream.Write(lidarData);
            }
            lidarStream.WriteLine("");
        }
        count++;

		transVelocity = transform.InverseTransformDirection(UAVrb.velocity);
		rotVelocity = UAVrb.angularVelocity;
        pose = UAVrb.transform.eulerAngles;

		tVelocityData = string.Format("{0}\t{1}\t{2}", transVelocity.x, transVelocity.y, transVelocity.z);
		aVelocityData = string.Format("{0}\t{1}\t{2}", rotVelocity.x, rotVelocity.y, rotVelocity.z);
        poseData = string.Format("{0}\t{1}\t{2}", pose.x, pose.y, pose.z);

        if(debug) {
            Debug.Log(aVelocityData);
        }

        //write the data to text files
        poseStream.WriteLine(Time.time + "\t" + poseData);
        velocityStream.WriteLine(Time.time + "\t" + tVelocityData + "\t" + aVelocityData);
    }

    void OnApplicationQuit()
	{
        if(debug) {
            Debug.Log("Application ending after " + Time.time + " seconds");
        }
		lidarStream.Close();
	}
}
