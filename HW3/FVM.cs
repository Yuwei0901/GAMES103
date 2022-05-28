using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

using System.Threading.Tasks;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	float g 			= -9.8f;

	int[] Tet;					//
	int   tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for(int i = 0 ; i < tet_number; ++i)
		{
			Matrix4x4 Dm = Build_Edge_Matrix(i);
			inv_Dm[i] = Dm.inverse;
		}

    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
		Vector3 X10 = X[Tet[tet * 4 + 1]] - X[Tet[tet * 4 + 0]];
		Vector3 X20 = X[Tet[tet * 4 + 2]] - X[Tet[tet * 4 + 0]];
		Vector3 X30 = X[Tet[tet * 4 + 3]] - X[Tet[tet * 4 + 0]];
		ret.SetColumn(0, X10);
		ret.SetColumn(1, X20);
		ret.SetColumn(2, X30);
		ret[3,3] = 1;
		return ret;
    }

    Matrix4x4 _MatSub(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 m = Matrix4x4.identity;
        m[0, 0] = a[0, 0] - b[0, 0]; m[0, 1] = a[0, 1] - b[0, 1]; m[0, 2] = a[0, 2] - b[0, 2];
        m[1, 0] = a[1, 0] - b[1, 0]; m[1, 1] = a[1, 1] - b[1, 1]; m[1, 2] = a[1, 2] - b[0, 2];
        m[2, 0] = a[2, 0] - b[2, 0]; m[2, 1] = a[2, 1] - b[2, 1]; m[2, 2] = a[2, 2] - b[2, 2];
        return m;
    }
    Matrix4x4 _MatAdd(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 m = Matrix4x4.identity;
        m[0, 0] = a[0, 0] + b[0, 0]; m[0, 1] = a[0, 1] + b[0, 1]; m[0, 2] = a[0, 2] + b[0, 2];
        m[1, 0] = a[1, 0] + b[1, 0]; m[1, 1] = a[1, 1] + b[1, 1]; m[1, 2] = a[1, 2] + b[0, 2];
        m[2, 0] = a[2, 0] + b[2, 0]; m[2, 1] = a[2, 1] + b[2, 1]; m[2, 2] = a[2, 2] + b[2, 2];
        return m;
    }
    Matrix4x4 _MatMulScaler(Matrix4x4 a, float s)
    {
        Matrix4x4 m = new Matrix4x4();
        m[0, 0] = a[0, 0] * s; m[0, 1] = a[0, 1] * s; m[0, 2] = a[0, 2] * s;
        m[1, 0] = a[1, 0] * s; m[1, 1] = a[1, 1] * s; m[1, 2] = a[1, 2] * s;
        m[2, 0] = a[2, 0] * s; m[2, 1] = a[2, 1] * s; m[2, 2] = a[2, 2] * s;
        return m;
    }
    float _MatTrace(Matrix4x4 m)
    {
        float tr = m[0, 0] + m[1, 1] + m[2, 2];
        return tr;
    }
    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i] = Vector3.zero;
			Force[i].y += mass * g;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
    		Matrix4x4 dm = Build_Edge_Matrix(tet);
			Matrix4x4 F = dm * inv_Dm[tet];
			F[3, 3] = 1;

    		//TODO: Green Strain
			Matrix4x4 G = _MatSub(F.transpose * F, Matrix4x4.identity);
			G[3,3] = 1;
			G = _MatMulScaler(G, 0.5f);

    		//TODO: Second PK Stress : S
			//S = 2*u*G + gama*Trace(G)*I
			Matrix4x4 S = new Matrix4x4();
			S = _MatAdd(_MatMulScaler(G , 2 * stiffness_1), _MatMulScaler(Matrix4x4.identity, _MatTrace(G) * stiffness_0));
			S[3, 3] = 1;

    		//First PK stess : P
			Matrix4x4 P = new Matrix4x4();
			P = F * S;
			P[3, 3] = 1;

			//TODO: Elastic Force
			Vector3 f1, f2, f3, f0;
			Matrix4x4 fs = new Matrix4x4();
			float tmp = -1 / (6 * inv_Dm[tet].determinant);
			fs = _MatMulScaler(P * 	inv_Dm[tet].transpose, tmp);
			f1 = fs.GetColumn(0);
			f2 = fs.GetColumn(1);
			f3 = fs.GetColumn(2);
			f0 = -f1 -f2 -f3;

			Force[Tet[tet * 4 + 0]] += f0;
			Force[Tet[tet * 4 + 1]] += f1;
			Force[Tet[tet * 4 + 2]] += f2;
			Force[Tet[tet * 4 + 3]] += f3;
    	}

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
			V[i] += dt * Force[i];
			V[i] *= damp;
		}
			//TODO: Laplacian Smoothing
		for(int i=0; i<number; i++)
		{
			V_sum[i] = Vector3.zero;
			V_num[i] = 0;
		}

		for(int tet=0; tet < tet_number; tet++)
		{
			Vector3 tet_V_sum = V[Tet[tet * 4 + 0]] + V[Tet[tet * 4 + 1]] + V[Tet[tet * 4 + 2]] + V[Tet[tet * 4 + 3]]; 
			V_sum[Tet[tet * 4 + 0]] += tet_V_sum; V_num[Tet[tet * 4 + 0]] += 4;
            V_sum[Tet[tet * 4 + 1]] += tet_V_sum; V_num[Tet[tet * 4 + 1]] += 4;
            V_sum[Tet[tet * 4 + 2]] += tet_V_sum; V_num[Tet[tet * 4 + 2]] += 4;
            V_sum[Tet[tet * 4 + 3]] += tet_V_sum; V_num[Tet[tet * 4 + 3]] += 4;
		}
		for(int i=0; i<number; i++)
		{
			V[i] = V_sum[i] / V_num[i];
		}
		for(int i=0; i<number; i++)
		{
			//V[i] = new Vector3(0, 0, 0);
			X[i] += dt * V[i];
			Debug.Log(X[1]);
			
		
			
    		//TODO: (Particle) collision with floor.
			float floor = -3.0f;
			Vector3 N = new Vector3(0, 1, 0);
			if(X[i].y < floor)
			{
				X[i].y = floor;
				if(Vector3.Dot(V[i], N) < 0)
				{
                    Vector3 Vn = Vector3.Dot(V[i], N) * N;
                    Vector3 Vt = V[i] - Vn;

                    float mu_N = 0.5f;
                    float a = 0.5f;
                    Vn *= -mu_N;
                    Vt *= a;
                    V[i] = Vn + Vt;
				}
			}
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
