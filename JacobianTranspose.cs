using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System.Diagnostics;

public class JacobianTranspose : MonoBehaviour
{
    [Header("Joint System Setup")]
    public Transform[] Joints; // The joints of the kinematic chain
    public Transform Target; // The target position
    public Transform Tip;   //Tip of the chain
    public GameObject Root; //Root of the chain 
    public GameObject jointPiece;   //Additional pieces of joint 

    [Header("Rotation Axis")]
    public Vector3 AxisConstraint; //Add constraints to Jacobian Algorithm

   
    [Header("Editable IK Settings")]
    public float EPS; //Distance between target n end tip
    public int AmountJoints; //Amount of joints in the system

    private float[] Angles; // The current angle of the i-th joint
    private Vector4[] AnglesVector; // The vector containing the angles of all joints (array = 3 axes)
    private Vector4 TargetGoalVector; // Same as below only as a Vector4 to multiply with matrix
    private Vector3 TargetGoal; // The vector between the target position and the end-effector
    private Vector3[] VectorJoint; // The vector of each joint in the kinematic chain (array = 3 axes)
   

    private Stopwatch timer;
    private Vector3 tempLegPos;
   
    //All the matrix created for the Algorithm to run
    private Matrix<float> Jacobian;
    private Matrix<float> ColumnMatrix;
    private Matrix<float> AngleMatrix;
    private Matrix<float> JT;
    private Matrix<float> Identity;
    
    
    void Start()
    {
        //Creates the chain of joints
        Joints = new Transform[AmountJoints];
        MakeLeg();

        //Initialises and fills all the matrixes
        JT = Matrix<float>.Build.Dense(AmountJoints + 1, AmountJoints + 1);
        Identity = Matrix<float>.Build.DenseDiagonal(AmountJoints + 1, AmountJoints + 1, 1.0f * AmountJoints * 100);
        Jacobian = Matrix<float>.Build.Dense(AmountJoints + 1, AmountJoints + 1);
        ColumnMatrix = Matrix<float>.Build.Dense(AmountJoints + 1, AmountJoints + 1);
        AngleMatrix = Matrix<float>.Build.Dense(3, 3);
        VectorJoint = new Vector3[Joints.Length];
        Angles = new float[Joints.Length * 3]; //0:X i=0, 1:Y i=0, 2:Z i=0, 3:X i=1, 4:Y i=1 etc...

        //Creates and starts a timer                                       
        timer = new Stopwatch();
        timer.Start();
    }


    //Makes the chain of joints for the kinematic to run on 
    void MakeLeg()
    {

        tempLegPos = Root.transform.position;
        GameObject tempLegParent = Root;
        Joints[0] = Root.transform;
        for (int i = 1; i <= AmountJoints - 1; i++)
        {
            GameObject tempPiece = Instantiate(jointPiece, new Vector3(tempLegPos.x, (tempLegPos.y + 1f), tempLegPos.z), Root.transform.rotation);
            tempPiece.transform.parent = Joints[i - 1].transform;
            tempLegPos = tempPiece.transform.position;
            Joints[i] = tempPiece.transform;
        }
        Tip.transform.parent = Joints[Joints.Length - 1].transform;
        Tip.transform.position = new Vector3((Joints[AmountJoints - 1].transform.position.x), (Joints[AmountJoints - 1].transform.position.y + 1f), Joints[AmountJoints - 1].transform.position.z);

    }

    //Fills in the columnMatrix with the targets positon 
    void MakeColumnVector()
    {
        ColumnMatrix[0, 0] = TargetGoalVector.x;
        ColumnMatrix[0, 1] = TargetGoalVector.y;
        ColumnMatrix[0, 2] = TargetGoalVector.z;

    }

    // Update is called once per frame
    void Update()
    {
        //Checks to see if the tip is close enough to the target position 
        if (Vector3.Distance(Tip.position, Target.position) > EPS)
        {
            // X = 0, Y = 1, Z = 2
            if (AxisConstraint.x != 0)
            {
                // Creates the Jacobian Matrix
                JacobianMatrix(0);

                //Creates the matrix containing the target position
                MakeColumnVector();

                //Transposes the Jacobian Matrix
                Jacobian = JacobianTranpose();

                //Finds the angles at which the joint needs to move
                AngleMatrix = Jacobian * ColumnMatrix;

                //Moves the joint by the angles calculated on the correct axis
                ApplyInverseKinematics(0);
            }
            if (AxisConstraint.y != 0)
            {
                JacobianMatrix(1);
                MakeColumnVector();
                Jacobian = JacobianTranpose();
                AngleMatrix = Jacobian * ColumnMatrix;
                ApplyInverseKinematics(1);
            }

            if (AxisConstraint.z != 0)
            {
                JacobianMatrix(2);
                MakeColumnVector();
                Jacobian = JacobianTranpose();
                AngleMatrix = Jacobian * ColumnMatrix;
                ApplyInverseKinematics(2);
            }
        }
        else
        {
            //Stops the timer and prints the time 
            timer.Stop();
            print("Time elapsed: {0}" + timer.Elapsed);

        }


    }


    //Transposes the Jacobian Matrix
    Matrix<float> JacobianTranpose()
    {
        Identity.Clear();
        Identity = Matrix<float>.Build.DenseDiagonal(AmountJoints + 1, AmountJoints + 1, 1.0f * AmountJoints * 100);
        JT = Identity * Jacobian.Transpose();
        return JT;
    }





    //Creates the Jacobian Matrix for the desired axis 
    Matrix<float> JacobianMatrix(int Axis)
    {
        TargetGoal = Target.position - Tip.position;
        TargetGoalVector = new Vector4(TargetGoal.x, TargetGoal.y, TargetGoal.z, 1.0f);

        switch (Axis)
        {
            case 0: // X Axis
                for (int i = 0; i < Joints.Length - 1; i++)
                {
                    VectorJoint[i] = Vector3.Cross(Joints[i].right,
                        (Joints[Joints.Length - 1].position - Joints[i].position));
                }
                break;
            case 1: // Y Axis
                for (int i = 0; i < Joints.Length - 1; i++)
                {
                    VectorJoint[i] = Vector3.Cross(Joints[i].up,
                        (Joints[Joints.Length - 1].position - Joints[i].position));
                }
                break;
            case 2: // Z Axis
                for (int i = 0; i < Joints.Length - 1; i++)
                {
                    VectorJoint[i] = Vector3.Cross(Joints[i].forward,
                        (Joints[Joints.Length - 1].position - Joints[i].position));
                }
                break;
        }
        for (int i = 0; i < Joints.Length - 1; i++)
        {
            
            Jacobian[i, 0] = VectorJoint[i].x;
            Jacobian[i, 1] = VectorJoint[i].y;
            Jacobian[i, 2] = VectorJoint[i].z;
            Jacobian[i, 3] = 0.0f;

        }
        Jacobian[Joints.Length - 1, 0] = 0.0f;
        Jacobian[Joints.Length - 1, 1] = 0.0f;
        Jacobian[Joints.Length - 1, 2] = 0.0f;
        Jacobian[Joints.Length - 1, 3] = 1.0f;
        return Jacobian;
    }


    //Uses the Angles calulcated with the Jacobian Matrix and moves the joints
    void ApplyInverseKinematics(int Axis)
    {

        for (int i = 0; i < Joints.Length - 1; i++)
        {

            Angles[i] += AngleMatrix[0, Axis];
            Angles[i] %= 360;
            if (Mathf.Abs(Angles[i]) > 180.0f)
            {
                Angles[i] -= 360 * Mathf.Sign(Angles[i]);
            }

            switch (Axis)
            {
                case 0: //For the X Axis
                    Joints[i].localEulerAngles = new Vector3(Angles[i],
                                                                      Joints[i].localEulerAngles.y,
                                                                      Joints[i].localEulerAngles.z);
                    break;

                case 1: //For the Y Axis
                    Joints[i].localEulerAngles = new Vector3(Joints[i].localEulerAngles.x,
                                                                      Angles[i],
                                                                      Joints[i].localEulerAngles.z);
                    break;

                case 2: //For the Z Axis
                    Joints[i].localEulerAngles = new Vector3(Joints[i].localEulerAngles.x,
                                                                      Joints[i].localEulerAngles.y,
                                                                      Angles[i]);
                    break;
            }

        }

    }
}

