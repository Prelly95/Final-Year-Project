using UnityEngine;

public class GridGenerator : MonoBehaviour
{
	#region Settings

	public GameObject Obsticalprefab = null;

	public Vector3 GridSize = Vector3.zero;

	public float spacing = 3;

	#endregion

	#region MonoBehavior

	void Start()
	{
		GenerateGrid();
	}

	#endregion

	void GenerateGrid()
	{
		Vector3 pos = Vector3.zero;
		if (Obsticalprefab != null)
		{
			for (int ii = 0; ii < GridSize.x; ii++)
			{
				for (int jj = 0; jj < GridSize.y; jj++)
				{
					for (int kk = 0; kk < GridSize.z; kk++)
					{
						pos.x = ii * spacing + Random.Range(-spacing / 4, spacing / 4);
						pos.y = jj * spacing + Random.Range(-spacing / 4, spacing / 4);
						pos.z = kk * spacing + Random.Range(-spacing / 4, spacing / 4);
						Instantiate(Obsticalprefab, pos, Quaternion.identity);
					}
				}
			}
		}
	}
}
