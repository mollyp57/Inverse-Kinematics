using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

public class TargetTriangle : MonoBehaviour
{
    [Header("Joint System Setup")]
    public Transform Target;
    public Transform Tip;
    public GameObject[] Joints;
    public GameObject Root;
    public GameObject jointPiece;

    
    [Header("Editable IK Settings")]
    public float EPS;
    public int AmountJoints;

    
    private Vector3 tempLegPos;
    private float B;
    private float OB;
    private float BT;

    private Stopwatch timer;

    
    // Start is called before the first frame update
    void Start()
    {
        //Creates the chain of joints
        Joints = new GameObject[AmountJoints];
        MakeLeg();

        //Creates and starts a timer  
        timer = new Stopwatch();
        timer.Start();

    }

    //Makes the chain of joints for the kinematic to run on 
    void MakeLeg()
    {
        tempLegPos = Root.transform.position;
        GameObject tempLegParent = Root;
        Joints[0] = Root;
        for (int i = 1; i <= AmountJoints - 1; i++)
        {
            GameObject tempPiece = Instantiate(jointPiece, new Vector3(tempLegPos.x, (tempLegPos.y + 1f), tempLegPos.z), Root.transform.rotation);
            tempPiece.transform.parent = Joints[i - 1].transform;
            tempLegPos = tempPiece.transform.position;
            Joints[i] = tempPiece;
        }
        Tip.transform.parent = Joints[Joints.Length - 1].transform;
        Tip.transform.position = new Vector3((Joints[AmountJoints - 1].transform.position.x), (Joints[AmountJoints - 1].transform.position.y + 1f), Joints[AmountJoints - 1].transform.position.z);


    }

    // Update is called once per frame
    void Update()
    {
        //Checks to see if the tip is close enough to the target position 
        if (Vector3.Distance(Tip.position, Target.position) > EPS)
        {
            for (int i = Joints.Length - 1; i >= 0; i--)
            {
                Triangulation(i);
                B = 0f;
            }

        }
        else
        {
            //Stops the timer and prints the time 
            timer.Stop();
            print("Time elapsed: {0}" + timer.Elapsed);

        }

    }



    void Triangulation(int location)
    {
        //Calculates VT and VE to further work out the Angle0 And Rotational Axis VR
        Vector3 VT = Vector3.Normalize(Target.position - Joints[location].transform.position);
        Vector3 VE = Vector3.Normalize(Tip.position - Joints[location].transform.position);
        float Angle0 = Mathf.Acos(Vector3.Dot(VE, VT));
        Angle0 = Angle0 * Mathf.Rad2Deg;
        Vector3 CrossVal = Vector3.Cross(VE, VT);
        Vector3 VR = ((Vector3.Cross(VE, VT) / Vector3.Magnitude(CrossVal)));

        //Calculates A value
        Vector3 meshSize = Joints[location].GetComponent<MeshRenderer>().bounds.size;
        float A = meshSize.y;

        //Calculates B Value
        for (int i = 0; i <= location; i++)
        {
            B += Joints[i].GetComponent<MeshRenderer>().bounds.size.y;
        }
     
        //Calculates C value
        float C = Vector3.Distance(Joints[location].transform.position, Target.position);


        if (C > A + B)
        {
            Joints[location].transform.RotateAround(Joints[location].transform.position, VR, Angle0);
            return;
        }

        if (C < Mathf.Abs(A - B))
        {
            Joints[location].transform.RotateAround(Joints[location].transform.position, VR, -Angle0);
            return;
        }

        //Checks to see if a triangle has been formed
        if (Mathf.Pow(A, 2) + Mathf.Pow(B, 2) - Mathf.Pow(C, 2) > 0)
        {
            float tempTop = Mathf.Pow(A, 2) + Mathf.Pow(C, 2) - Mathf.Pow(B, 2);
            float tempBottom = 2 * A * C;
            OB = Mathf.Acos(tempTop / tempBottom);
            OB = OB * Mathf.Rad2Deg;


            tempTop = Mathf.Pow(A, 2) + Mathf.Pow(B, 2) - Mathf.Pow(C, 2);
            tempBottom = 2 * A * B;
            float OC = Mathf.Acos(tempTop / tempBottom);
            OC = OC * Mathf.Rad2Deg;
            BT = Mathf.PI - OC;
     

            if (BT > 180)
            {
                
                OB = OB - 10;

            }

            else
            {
                OB = OB + 10;
            }
           
            float BI = Angle0 - OB;

            Joints[location].transform.RotateAround(Joints[location].transform.position, VR, BI);
        }
    }
}
