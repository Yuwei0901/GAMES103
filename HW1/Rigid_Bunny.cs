using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	float g = -9.8f;							//for accleration of gravity

	Mesh mesh;
	Vector3[] vertices;

	private GameObject ball;

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Quaternion QuatAdd(Quaternion a, Quaternion b)
	{
		Quaternion tmp = new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
		return	tmp;
	}

	Matrix4x4 MatSub(Matrix4x4 A, Matrix4x4 B)
	{
		Matrix4x4 M = Matrix4x4.zero;
		for(int i = 0; i < 4; ++i)
			for(int j = 0; j < 4; ++j)
				M[i, j] = A[i, j] - B[i, j];
		return M;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{	
		Vector3 sumV = new Vector3();
		int	numV = 0;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		//Debug.Log(R);
		//Vector3 Rr_i, x_i;
		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 Rr_i = R * vertices[i];
			Vector3 x_i = transform.position + Rr_i;

			float phi_function = Vector3.Dot(x_i - P, N);
			if (phi_function < 0 )
			{
				sumV += vertices[i];
				numV++;
			}
		}
		if(numV == 0) return;
 		Vector3 aveV = sumV / numV; // Average collision vertice
		Vector3 v_i = v + Vector3.Cross(w, R*aveV);
		
		if(Vector3.Dot(v_i, N) > 0)return;

		Vector3 v_ni = Vector3.Dot(v_i, N) * N;
		Vector3 v_ti = v_i - v_ni;

		float u_t = 0.5f;
		float u_n = restitution;
		float a = Mathf.Max(1 - (u_t * (1 + u_n) * v_ni.sqrMagnitude) / v_ti.sqrMagnitude, 0);

		v_ni *= -u_n;
		v_ti *= a;
		Vector3 v_i_new = v_ni + v_ti;

		Matrix4x4 M = Matrix4x4.identity;
		for(int i = 0; i < 3; ++i)
		{
			M[i, i] = 1.0f / mass;
		}
		Debug.Log(M);
		Matrix4x4 Rri = Get_Cross_Matrix(R.MultiplyPoint(aveV));
		Matrix4x4 I = R * I_ref * R.transpose;
		Matrix4x4 K = MatSub(M, Rri * I.inverse * Rri);
		K[3,3] = 1;
		//Debug.Log(K);
		Vector3 j = K.inverse.MultiplyPoint(v_i_new - v_i);

		v += j / mass;
		w += I.inverse.MultiplyPoint(Rri.MultiplyPoint(j));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5.0f, 0, 0);
			w = new Vector3 (3.0f, 0, 0);
			launched=true;
		}

		if(!launched)
		{
			return;
		}


		//Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Artificial Stabilization
		if (v.sqrMagnitude < 0.07)
        {
            v = new Vector3(0, 0, 0);
            w = new Vector3(0, 0, 0);
            return;
        }

		//Update linear status
		v.y += dt * g;
		v *= linear_decay;
		transform.position += dt * v;

		//Update angular status
		Quaternion q = transform.rotation;
		w *= angular_decay;
		Quaternion temp_q = new Quaternion(0.5f * dt * w.x, 0.5f * dt * w.y, 0.5f * dt * w.z, 0);
		transform.rotation = Quaternion.Normalize(QuatAdd(q, temp_q * q));
	}	
}
