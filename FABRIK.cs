using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

public class FABRIK : MonoBehaviour
{
    [Header("Joint System Setup")]
    public GameObject[] Joints;
    public GameObject Root;
    public GameObject jointPiece;
    public Transform Tip;
    public Transform Target;

    [Header("Editable IK Settings")]
    public int AmountJoints;
    public float EPS;



    private float LengthOfLigament;
    private Vector3 tempLegPos;
    private Stopwatch timer;
  

    // Start is called before the first frame update
    void Start()
    {
        //Creates the chain of joints
        Joints = new GameObject[AmountJoints];
        MakeLeg();

        //Calculates the mesh size of one joint along the Y Axis
        Vector3 meshSize = Joints[0].GetComponent<MeshRenderer>().bounds.size;
        LengthOfLigament = meshSize.y;

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

    //Preforms the backwards part of the algorithm 
    void Backwards()
    {
        //Saves the location of the root joint and moves the end effector to target position
        Vector3 Jointbase = Joints[0].transform.position;
        Joints[AmountJoints - 1].transform.position = Target.position - (Joints[AmountJoints - 1].transform.forward.normalized * LengthOfLigament);
      
        //Calulcates and moves joints to their new positions
        for (int i = AmountJoints - 2; i >= 0; i--)
        {
            Vector3 direction = Vector3.Normalize(Joints[i].transform.position - Joints[i + 1].transform.position); 
            Joints[i].transform.position = Joints[i + 1].transform.position + direction * LengthOfLigament; 
        }
        //Resets root joint to its orignal position
        Joints[0].transform.position = Jointbase;

    }

    //Preforms the forwards part of the algorithm
    void Forwards()
    {
       //Calculates the seconds joints position using the root joint
        Joints[1].transform.position = Joints[0].transform.position + Joints[0].transform.rotation * Vector3.forward * LengthOfLigament;

        //Calculates and moves joints to their new positions
        for (int i = 2; i < Joints.Length; i++)
        {
            Vector3 direction = Vector3.Normalize(Joints[i].transform.position - Joints[i - 1].transform.position);
            Joints[i - 1].transform.rotation = Quaternion.LookRotation(direction, Joints[i - 1].transform.parent.rotation * Vector3.up);
            Joints[i].transform.position = Joints[i - 1].transform.position + Joints[i - 1].transform.rotation * Vector3.forward * LengthOfLigament;
        }

        Vector3 dir = Vector3.zero;
        for (int i = 0; i < Joints.Length - 2; i++)
        {
            dir += (Joints[i].transform.position - Joints[AmountJoints - 1].transform.position);

        }

        //Ensures the end effector is looking at the target
        Joints[AmountJoints - 1].transform.LookAt(Target);

    }


    //Calls backwards first then forwards
    void SolveFABRIK()
    {
        Backwards();
        Forwards();
    }

    
    // Update is called once per frame
    void Update()
    {
        //Checks to see if the tip is close enough to the target position 
        if (Vector3.Distance(Tip.position, Target.position) > EPS)
        {
            SolveFABRIK();
        }
        else
        {
            //Stops the timer and prints the time 
            timer.Stop();
            print("Time elapsed for FABRIK: {0}" + timer.Elapsed);
        }

    }
}
