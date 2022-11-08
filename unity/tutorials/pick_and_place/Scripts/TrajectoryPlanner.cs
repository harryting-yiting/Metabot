using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;  // self-defined message in NiryoMoveit package
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

// ly change
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class TrajectoryPlanner : MonoBehaviour
{
    // 没有声明public就是类的私有成员
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    // Service name - “niryo_moveit”
    string m_RosServiceName = "niryo_moveit";

    // ly change
    [SerializeField]
    string m_RosSubscribeName = "joint_state"; // 话题名可能会写错

    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }
    // 这里对每个gameobject的对象都赋予多一个public的gameobject
    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    // read only，只读（相当于const）
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();

        //写在构造函数里的注册一个服务，<request的消息类型，response的消息类型>（service的名字）
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        // ly change
        // this code just register the subscriber rather than call the subscribe callback function
        // m_Ros.RegisterSubscriber<JointStateMsg>(m_RosSubscribeName, DisplayGazeboJointState);

        // ly change
        // this code is call the subscribe callback function at the beginning
        m_Ros.Subscribe<JointStateMsg>(m_RosSubscribeName, DisplayGazeboJointState);


        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        // 这个的赋值方式应该是unity自己的习惯，直接对 m_LeftGripper.xDrive.target是赋值不了的，因为是值拷贝
        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    /// this is a function, and return is NiryoMoveitJointsMsg MessageType
    // And this function is to give the jointstate of robot in unity, and publish this message for ROS

    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();

        //  -------------------这是在给request赋值，即给出joints信息、pick pose和place pose-------------------
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };
        // -------------------这是在给request赋值，即给出joints信息、pick pose和place pose-------------------

        // ROSConnection.GetOrCreateInstance().SendServiceMessage<SetSceneResponse>(serviceName, request, Callback) 函数原型
        // SendServiceMessage<srv文件名+Response>(注册的服务名，new MoverServiceRequest()，Callback)

        // m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);

        // ly change
        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, PlanningResultResponse);
    }

    // ly change
    // ----------------------------------
    // This Callback function, Tell the unity whether Planning Trajectory success or not, Give the Planning Result
    // True means planning successful, False means planning fail
    // ----------------------------------
    void PlanningResultResponse(MoverServiceResponse response)
    {
        if (response.planningResult == true)
        {
            Debug.Log("Trajectory returned.");
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }


    // ly change
    // describe callback
    void DisplayGazeboJointState(JointStateMsg jointstate)
    {
        StartCoroutine(SetJointValues(jointstate));
    }

    // response，当response从服务器server传回来的时候，进入回调函数
    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>

    // ly change
    IEnumerator SetJointValues(JointStateMsg jointstate)
    {
        for (int i = 0; i < jointstate.name.Length - 2; i++)
        {
            var joint1XDrive = m_JointArticulationBodies[i].xDrive;
            joint1XDrive.target = (float)(jointstate.position[i]) * Mathf.Rad2Deg;
            m_JointArticulationBodies[i].xDrive = joint1XDrive;
            Debug.Log(joint1XDrive.target);
        }
        yield return new WaitForSeconds(k_JointAssignmentWait);
    }

    // 回来看看这个函数 如何extract trajectory
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned —— 每次接收到轨迹规划，trajectory
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint —— 为每个关节赋值
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}
