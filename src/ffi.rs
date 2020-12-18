//! Foreign function interface for Bullet C API.
#![allow(non_camel_case_types, non_snake_case)]

use std::os::raw::{c_char, c_int, c_uchar};

#[repr(C)]
pub struct b3PhysicsClientHandle__ {
    _unused: c_int,
}
pub type b3PhysicsClientHandle = *mut b3PhysicsClientHandle__;

#[repr(C)]
pub struct b3SharedMemoryCommandHandle__ {
    _unused: c_int,
}
pub type b3SharedMemoryCommandHandle = *mut b3SharedMemoryCommandHandle__;

#[repr(C)]
pub struct b3SharedMemoryStatusHandle__ {
    _unused: c_int,
}
pub type b3SharedMemoryStatusHandle = *mut b3SharedMemoryStatusHandle__;

extern "C" {
    pub fn b3ConnectPhysicsDirect() -> b3PhysicsClientHandle;
    pub fn b3CreateInProcessPhysicsServerAndConnect(
        argc: c_int,
        argv: *mut *mut c_char,
    ) -> b3PhysicsClientHandle;
    pub fn b3CreateInProcessPhysicsServerAndConnectMainThread(
        argc: c_int,
        argv: *mut *mut c_char,
    ) -> b3PhysicsClientHandle;
    pub fn b3DisconnectSharedMemory(physClient: b3PhysicsClientHandle);

    pub fn b3InitConfigureOpenGLVisualizer(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3ConfigureOpenGLVisualizerSetVisualizationFlags(
        commandHandle: b3SharedMemoryCommandHandle,
        flag: c_int,
        enabled: c_int,
    );
    pub fn b3CanSubmitCommand(physClient: b3PhysicsClientHandle) -> c_int;
    pub fn b3SubmitClientCommandAndWaitStatus(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
    ) -> b3SharedMemoryStatusHandle;

    pub fn b3GetStatusType(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3GetStatusActualState(
        statusHandle: b3SharedMemoryStatusHandle,
        bodyUniqueId: *mut c_int,
        numDegreeOfFreedomQ: *mut c_int,
        numDegreeOfFreedomU: *mut c_int,
        rootLocalInertialFrame: *const *mut f64,
        actualStateQ: *const *mut f64,
        actualStateQdot: *const *mut f64,
        jointReactionForces: *const *mut f64,
    ) -> c_int;

    pub fn b3GetStatusBodyIndex(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3InitSyncBodyInfoCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3InitSyncUserDataCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitPhysicsParamCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3PhysicsParamSetGravity(
        commandHandle: b3SharedMemoryCommandHandle,
        gravx: f64,
        gravy: f64,
        gravz: f64,
    ) -> c_int;
    pub fn b3PhysicsParamSetTimeStep(
        commandHandle: b3SharedMemoryCommandHandle,
        time_step: f64,
    ) -> c_int;
    pub fn b3PhysicsParamSetRealTimeSimulation(
        commandHandle: b3SharedMemoryCommandHandle,
        enableRealTimeSimulation: c_int,
    ) -> c_int;
    pub fn b3InitStepSimulationCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitResetSimulationCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitResetSimulationCommand2(
        commandHandle: b3SharedMemoryCommandHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitResetSimulationSetFlags(
        commandHandle: b3SharedMemoryCommandHandle,
        flags: c_int,
    ) -> c_int;

    pub fn b3SetAdditionalSearchPath(
        physClient: b3PhysicsClientHandle,
        path: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3LoadUrdfCommandInit(
        physClient: b3PhysicsClientHandle,
        urdfFileName: *const c_char,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3LoadUrdfCommandSetFlags(
        commandHandle: b3SharedMemoryCommandHandle,
        flags: c_int,
    ) -> c_int;
    pub fn b3LoadUrdfCommandSetStartPosition(
        commandHandle: b3SharedMemoryCommandHandle,
        startPosX: f64,
        startPosY: f64,
        startPosZ: f64,
    ) -> c_int;
    pub fn b3LoadUrdfCommandSetStartOrientation(
        commandHandle: b3SharedMemoryCommandHandle,
        startOrnX: f64,
        startOrnY: f64,
        startOrnZ: f64,
        startOrnW: f64,
    ) -> c_int;
    pub fn b3LoadUrdfCommandSetUseFixedBase(
        commandHandle: b3SharedMemoryCommandHandle,
        useFixedBase: c_int,
    ) -> c_int;
    pub fn b3LoadUrdfCommandSetGlobalScaling(
        commandHandle: b3SharedMemoryCommandHandle,
        globalScaling: f64,
    ) -> c_int;

    pub fn b3RequestActualStateCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3RequestActualStateCommandComputeLinkVelocity(
        commandHandle: b3SharedMemoryCommandHandle,
        computeLinkVelocity: c_int,
    ) -> c_int;
    pub fn b3RequestActualStateCommandComputeForwardKinematics(
        commandHandle: b3SharedMemoryCommandHandle,
        computeForwardKinematics: c_int,
    ) -> c_int;
    pub fn b3GetJointState(
        physClient: b3PhysicsClientHandle,
        statusHandle: b3SharedMemoryStatusHandle,
        jointIndex: c_int,
        state: *mut b3JointSensorState,
    ) -> c_int;
    pub fn b3GetJointStateMultiDof(
        physClient: b3PhysicsClientHandle,
        statusHandle: b3SharedMemoryStatusHandle,
        jointIndex: c_int,
        state: *mut b3JointSensorState2,
    ) -> c_int;
    pub fn b3GetLinkState(
        physClient: b3PhysicsClientHandle,
        statusHandle: b3SharedMemoryStatusHandle,
        linkIndex: c_int,
        state: *mut b3LinkState,
    ) -> c_int;
    pub fn b3InitChangeDynamicsInfo(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3ChangeDynamicsInfoSetLinearDamping(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linear_damping: f64,
    ) -> c_int;
    pub fn b3ChangeDynamicsInfoSetAngularDamping(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linear_damping: f64,
    ) -> c_int;

    pub fn b3GetNumJoints(physClient: b3PhysicsClientHandle, bodyUniqueId: c_int) -> c_int;
    pub fn b3GetJointInfo(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        jointIndex: c_int,
        jointInfo: *mut b3JointInfo,
    );
    pub fn b3CreatePoseCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3CreatePoseCommandSetBasePosition(
        commandHandle: b3SharedMemoryCommandHandle,
        startPosX: f64,
        startPosY: f64,
        startPosZ: f64,
    ) -> c_int;

    pub fn b3CreatePoseCommandSetBaseOrientation(
        commandHandle: b3SharedMemoryCommandHandle,
        startOrnX: f64,
        startOrnY: f64,
        startOrnZ: f64,
        startOrnW: f64,
    ) -> c_int;
    pub fn b3CreatePoseCommandSetJointPosition(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        jointIndex: c_int,
        jointPosition: f64,
    ) -> c_int;
    pub fn b3CreatePoseCommandSetJointVelocity(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        jointIndex: c_int,
        jointVelocity: f64,
    ) -> c_int;

    pub fn b3ComputeDofCount(physClient: b3PhysicsClientHandle, bodyUniqueId: c_int) -> c_int;

    pub fn b3CalculateInverseDynamicsCommandInit2(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        jointPositionsQ: *const f64,
        dofCountQ: c_int,
        jointVelocitiesQdot: *const f64,
        jointAccelerations: *const f64,
        dofCountQdot: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3CalculateInverseDynamicsSetFlags(
        commandHandle: b3SharedMemoryCommandHandle,
        flags: c_int,
    );
    pub fn b3GetStatusInverseDynamicsJointForces(
        statusHandle: b3SharedMemoryStatusHandle,
        bodyUniqueId: *mut c_int,
        dofCount: *mut c_int,
        jointForces: *mut f64,
    ) -> c_int;

    pub fn b3CalculateMassMatrixCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        jointPositionsQ: *const f64,
        dofCountQ: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3CalculateMassMatrixSetFlags(commandHandle: b3SharedMemoryCommandHandle, flags: c_int);

    #[doc = "the mass matrix is stored in column-major layout of size dofCount*dofCount"]
    pub fn b3GetStatusMassMatrix(
        physClient: b3PhysicsClientHandle,
        statusHandle: b3SharedMemoryStatusHandle,
        dofCount: *mut c_int,
        massMatrix: *mut f64,
    ) -> c_int;

    pub fn b3CalculateInverseKinematicsCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3CalculateInverseKinematicsAddTargetPurePosition(
        commandHandle: b3SharedMemoryCommandHandle,
        endEffectorLinkIndex: c_int,
        targetPosition: *const f64,
    );
    pub fn b3CalculateInverseKinematicsAddTargetsPurePosition(
        commandHandle: b3SharedMemoryCommandHandle,
        numEndEffectorLinkIndices: c_int,
        endEffectorIndices: *const c_int,
        targetPositions: *const f64,
    );
    pub fn b3CalculateInverseKinematicsAddTargetPositionWithOrientation(
        commandHandle: b3SharedMemoryCommandHandle,
        endEffectorLinkIndex: c_int,
        targetPosition: *const f64,
        targetOrientation: *const f64,
    );
    pub fn b3CalculateInverseKinematicsPosWithNullSpaceVel(
        commandHandle: b3SharedMemoryCommandHandle,
        numDof: c_int,
        endEffectorLinkIndex: c_int,
        targetPosition: *const f64,
        lowerLimit: *const f64,
        upperLimit: *const f64,
        jointRange: *const f64,
        restPose: *const f64,
    );
    pub fn b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(
        commandHandle: b3SharedMemoryCommandHandle,
        numDof: c_int,
        endEffectorLinkIndex: c_int,
        targetPosition: *const f64,
        targetOrientation: *const f64,
        lowerLimit: *const f64,
        upperLimit: *const f64,
        jointRange: *const f64,
        restPose: *const f64,
    );
    pub fn b3CalculateInverseKinematicsSetJointDamping(
        commandHandle: b3SharedMemoryCommandHandle,
        numDof: c_int,
        jointDampingCoeff: *const f64,
    );
    pub fn b3CalculateInverseKinematicsSelectSolver(
        commandHandle: b3SharedMemoryCommandHandle,
        solver: c_int,
    );
    pub fn b3GetStatusInverseKinematicsJointPositions(
        commandHandle: b3SharedMemoryStatusHandle,
        bodyUniqueId: *mut c_int,
        dofCount: *mut c_int,
        jointPositions: *mut f64,
    ) -> c_int;

    pub fn b3CalculateInverseKinematicsSetCurrentPositions(
        commandHandle: b3SharedMemoryCommandHandle,
        numDof: c_int,
        currentJointPositions: *const f64,
    );
    pub fn b3CalculateInverseKinematicsSetMaxNumIterations(
        commandHandle: b3SharedMemoryCommandHandle,
        maxNumIterations: c_int,
    );
    pub fn b3CalculateInverseKinematicsSetResidualThreshold(
        commandHandle: b3SharedMemoryCommandHandle,
        residualThreshold: f64,
    );

    pub fn b3CalculateJacobianCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        localPosition: *const f64,
        jointPositionsQ: *const f64,
        jointVelocitiesQdot: *const f64,
        jointAccelerations: *const f64,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3GetStatusJacobian(
        statusHandle: b3SharedMemoryStatusHandle,
        dofCount: *mut c_int,
        linearJacobian: *mut f64,
        angularJacobian: *mut f64,
    ) -> c_int;
    pub fn b3JointControlCommandInit2(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        controlMode: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3JointControlSetDesiredPosition(
        commandHandle: b3SharedMemoryCommandHandle,
        qIndex: c_int,
        value: f64,
    ) -> c_int;
    pub fn b3JointControlSetKp(
        commandHandle: b3SharedMemoryCommandHandle,
        dofIndex: c_int,
        value: f64,
    ) -> c_int;
    pub fn b3JointControlSetKd(
        commandHandle: b3SharedMemoryCommandHandle,
        dofIndex: c_int,
        value: f64,
    ) -> c_int;
    pub fn b3JointControlSetMaximumVelocity(
        commandHandle: b3SharedMemoryCommandHandle,
        dofIndex: c_int,
        maximumVelocity: f64,
    ) -> c_int;

    pub fn b3JointControlSetDesiredVelocity(
        commandHandle: b3SharedMemoryCommandHandle,
        dofIndex: c_int,
        value: f64,
    ) -> c_int;
    pub fn b3JointControlSetMaximumForce(
        commandHandle: b3SharedMemoryCommandHandle,
        dofIndex: c_int,
        value: f64,
    ) -> c_int;
    pub fn b3JointControlSetDesiredForceTorque(
        commandHandle: b3SharedMemoryCommandHandle,
        dofIndex: c_int,
        value: f64,
    ) -> c_int;
    pub fn b3InitRequestCameraImage(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3RequestCameraImageSetPixelResolution(
        commandHandle: b3SharedMemoryCommandHandle,
        width: c_int,
        height: c_int,
    );
    pub fn b3RequestCameraImageSetCameraMatrices(
        commandHandle: b3SharedMemoryCommandHandle,
        viewMatrix: *const f32,
        projectionMatrix: *const f32,
    );

    pub fn b3GetCameraImageData(
        physClient: b3PhysicsClientHandle,
        imageData: *mut b3CameraImageData,
    );

    pub fn b3ComputeViewMatrixFromPositions(
        cameraPosition: *const f32,
        cameraTargetPosition: *const f32,
        cameraUp: *const f32,
        viewMatrix: *mut f32,
    );
    pub fn b3ComputeViewMatrixFromYawPitchRoll(
        cameraTargetPosition: *const f32,
        distance: f32,
        yaw: f32,
        pitch: f32,
        roll: f32,
        upAxis: c_int,
        viewMatrix: *mut f32,
    );
    pub fn b3ComputePositionFromViewMatrix(
        viewMatrix: *const f32,
        cameraPosition: *mut f32,
        cameraTargetPosition: *mut f32,
        cameraUp: *mut f32,
    );

    pub fn b3ComputeProjectionMatrix(
        left: f32,
        right: f32,
        bottom: f32,
        top: f32,
        nearVal: f32,
        farVal: f32,
        projectionMatrix: *mut f32,
    );
    pub fn b3ComputeProjectionMatrixFOV(
        fov: f32,
        aspect: f32,
        nearVal: f32,
        farVal: f32,
        projectionMatrix: *mut f32,
    );

    pub fn b3InitUserDebugAddParameter(
        physClient: b3PhysicsClientHandle,
        // this was once a normal char but i could only get pointers as u8 instead of i8
        // so now this parameter is a uchar. seems to work
        txt: *const c_uchar,
        rangeMin: f64,
        rangeMax: f64,
        startValue: f64,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetDebugItemUniqueId(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3InitUserDebugReadParameter(
        physClient: b3PhysicsClientHandle,
        debugItemUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetStatusDebugParameterValue(
        statusHandle: b3SharedMemoryStatusHandle,
        paramValue: *mut f64,
    ) -> c_int;
    #[doc = " Add/remove user-specific debug lines and debug text messages"]
    pub fn b3InitUserDebugDrawAddLine3D(
        physClient: b3PhysicsClientHandle,
        fromXYZ: *const f64,
        toXYZ: *const f64,
        colorRGB: *const f64,
        lineWidth: f64,
        lifeTime: f64,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3InitUserDebugDrawAddText3D(
        physClient: b3PhysicsClientHandle,
        // this was once a normal char but i could only get pointers as u8 instead of i8
        // so now this parameter is a uchar. seems to work
        txt: *const c_uchar,
        positionXYZ: *const f64,
        colorRGB: *const f64,
        textSize: f64,
        lifeTime: f64,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3UserDebugTextSetOptionFlags(
        commandHandle: b3SharedMemoryCommandHandle,
        optionFlags: c_int,
    );
    pub fn b3UserDebugTextSetOrientation(
        commandHandle: b3SharedMemoryCommandHandle,
        orientation: *const f64,
    );
    pub fn b3UserDebugItemSetReplaceItemUniqueId(
        commandHandle: b3SharedMemoryCommandHandle,
        replaceItem: c_int,
    );

    pub fn b3UserDebugItemSetParentObject(
        commandHandle: b3SharedMemoryCommandHandle,
        objectUniqueId: c_int,
        linkIndex: c_int,
    );
    pub fn b3InitUserDebugDrawRemove(
        physClient: b3PhysicsClientHandle,
        debugItemUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitUserDebugDrawRemoveAll(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    #[doc = " Apply external force at the body (or link) center of mass, in world space/Cartesian coordinates."]
    pub fn b3ApplyExternalForceCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3ApplyExternalForce(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkId: c_int,
        force: *const f64,
        position: *const f64,
        flag: c_int,
    );
    pub fn b3ApplyExternalTorque(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkId: c_int,
        torque: *const f64,
        flag: c_int,
    );

    pub fn b3RequestKeyboardEventsCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetKeyboardEventsData(
        physClient: b3PhysicsClientHandle,
        keyboardEventsData: *mut b3KeyboardEventsData,
    );

    pub fn b3RequestMouseEventsCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetMouseEventsData(
        physClient: b3PhysicsClientHandle,
        mouseEventsData: *mut b3MouseEventsData,
    );
    #[doc = "We are currently not reading the sensor information from the URDF file, and programmatically assign sensors."]
    #[doc = "This is rather inconsistent, to mix programmatical creation with loading from file."]
    pub fn b3CreateSensorCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3CreateSensorEnable6DofJointForceTorqueSensor(
        commandHandle: b3SharedMemoryCommandHandle,
        jointIndex: c_int,
        enable: c_int,
    ) -> c_int;
    pub fn b3InitDebugDrawingCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3SetDebugObjectColor(
        commandHandle: b3SharedMemoryCommandHandle,
        objectUniqueId: c_int,
        linkIndex: c_int,
        objectColorRGB: *const f64,
    );
    pub fn b3RemoveDebugObjectColor(
        commandHandle: b3SharedMemoryCommandHandle,
        objectUniqueId: c_int,
        linkIndex: c_int,
    );
}

#[repr(C)]
pub enum eURDF_Flags {
    URDF_USE_INERTIA_FROM_FILE = 2,
    URDF_USE_SELF_COLLISION = 8,
    URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,
    URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,
    URDF_RESERVED = 64,
    URDF_USE_IMPLICIT_CYLINDER = 128,
    URDF_GLOBAL_VELOCITIES_MB = 256,
    MJCF_COLORS_FROM_FILE = 512,
    URDF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,
    URDF_ENABLE_SLEEPING = 2048,
    URDF_INITIALIZE_SAT_FEATURES = 4096,
    URDF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,
    URDF_PARSE_SENSORS = 16384,
    URDF_USE_MATERIAL_COLORS_FROM_MTL = 32768,
    URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,
    URDF_MAINTAIN_LINK_ORDER = 131072,
    URDF_ENABLE_WAKEUP = 262144,
}

#[repr(C)]
pub enum EnumSharedMemoryServerStatus {
    CMD_SHARED_MEMORY_NOT_INITIALIZED = 0,
    CMD_WAITING_FOR_CLIENT_COMMAND,
    CMD_CLIENT_COMMAND_COMPLETED,
    CMD_UNKNOWN_COMMAND_FLUSHED,
    CMD_SDF_LOADING_COMPLETED,
    CMD_SDF_LOADING_FAILED,
    CMD_URDF_LOADING_COMPLETED,
    CMD_URDF_LOADING_FAILED,
    CMD_BULLET_LOADING_COMPLETED,
    CMD_BULLET_LOADING_FAILED,
    CMD_BULLET_SAVING_COMPLETED,
    CMD_BULLET_SAVING_FAILED,
    CMD_MJCF_LOADING_COMPLETED,
    CMD_MJCF_LOADING_FAILED,
    CMD_REQUEST_INTERNAL_DATA_COMPLETED,
    CMD_REQUEST_INTERNAL_DATA_FAILED,
    CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED,
    CMD_BULLET_DATA_STREAM_RECEIVED_FAILED,
    CMD_BOX_COLLISION_SHAPE_CREATION_COMPLETED,
    CMD_RIGID_BODY_CREATION_COMPLETED,
    CMD_SET_JOINT_FEEDBACK_COMPLETED,
    CMD_ACTUAL_STATE_UPDATE_COMPLETED,
    CMD_ACTUAL_STATE_UPDATE_FAILED,
    CMD_DEBUG_LINES_COMPLETED,
    CMD_DEBUG_LINES_OVERFLOW_FAILED,
    CMD_DESIRED_STATE_RECEIVED_COMPLETED,
    CMD_STEP_FORWARD_SIMULATION_COMPLETED,
    CMD_RESET_SIMULATION_COMPLETED,
    CMD_CAMERA_IMAGE_COMPLETED,
    CMD_CAMERA_IMAGE_FAILED,
    CMD_BODY_INFO_COMPLETED,
    CMD_BODY_INFO_FAILED,
    CMD_INVALID_STATUS,
    CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED,
    CMD_CALCULATED_INVERSE_DYNAMICS_FAILED,
    CMD_CALCULATED_JACOBIAN_COMPLETED,
    CMD_CALCULATED_JACOBIAN_FAILED,
    CMD_CALCULATED_MASS_MATRIX_COMPLETED,
    CMD_CALCULATED_MASS_MATRIX_FAILED,
    CMD_CONTACT_POINT_INFORMATION_COMPLETED,
    CMD_CONTACT_POINT_INFORMATION_FAILED,
    CMD_REQUEST_AABB_OVERLAP_COMPLETED,
    CMD_REQUEST_AABB_OVERLAP_FAILED,
    CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED,
    CMD_CALCULATE_INVERSE_KINEMATICS_FAILED,
    CMD_SAVE_WORLD_COMPLETED,
    CMD_SAVE_WORLD_FAILED,
    CMD_VISUAL_SHAPE_INFO_COMPLETED,
    CMD_VISUAL_SHAPE_INFO_FAILED,
    CMD_VISUAL_SHAPE_UPDATE_COMPLETED,
    CMD_VISUAL_SHAPE_UPDATE_FAILED,
    CMD_LOAD_TEXTURE_COMPLETED,
    CMD_LOAD_TEXTURE_FAILED,
    CMD_USER_DEBUG_DRAW_COMPLETED,
    CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED,
    CMD_USER_DEBUG_DRAW_FAILED,
    CMD_USER_CONSTRAINT_COMPLETED,
    CMD_USER_CONSTRAINT_INFO_COMPLETED,
    CMD_USER_CONSTRAINT_REQUEST_STATE_COMPLETED,
    CMD_REMOVE_USER_CONSTRAINT_COMPLETED,
    CMD_CHANGE_USER_CONSTRAINT_COMPLETED,
    CMD_REMOVE_USER_CONSTRAINT_FAILED,
    CMD_CHANGE_USER_CONSTRAINT_FAILED,
    CMD_USER_CONSTRAINT_FAILED,
    CMD_REQUEST_VR_EVENTS_DATA_COMPLETED,
    CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED,
    CMD_SYNC_BODY_INFO_COMPLETED,
    CMD_SYNC_BODY_INFO_FAILED,
    CMD_STATE_LOGGING_COMPLETED,
    CMD_STATE_LOGGING_START_COMPLETED,
    CMD_STATE_LOGGING_FAILED,
    CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED,
    CMD_REQUEST_KEYBOARD_EVENTS_DATA_FAILED,
    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED,
    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED,
    CMD_REMOVE_BODY_COMPLETED,
    CMD_REMOVE_BODY_FAILED,
    CMD_GET_DYNAMICS_INFO_COMPLETED,
    CMD_GET_DYNAMICS_INFO_FAILED,
    CMD_CREATE_COLLISION_SHAPE_FAILED,
    CMD_CREATE_COLLISION_SHAPE_COMPLETED,
    CMD_CREATE_VISUAL_SHAPE_FAILED,
    CMD_CREATE_VISUAL_SHAPE_COMPLETED,
    CMD_CREATE_MULTI_BODY_FAILED,
    CMD_CREATE_MULTI_BODY_COMPLETED,
    CMD_REQUEST_COLLISION_INFO_COMPLETED,
    CMD_REQUEST_COLLISION_INFO_FAILED,
    CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED,
    CMD_CHANGE_TEXTURE_COMMAND_FAILED,
    CMD_CUSTOM_COMMAND_COMPLETED,
    CMD_CUSTOM_COMMAND_FAILED,
    CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED,
    CMD_SAVE_STATE_FAILED,
    CMD_SAVE_STATE_COMPLETED,
    CMD_RESTORE_STATE_FAILED,
    CMD_RESTORE_STATE_COMPLETED,
    CMD_COLLISION_SHAPE_INFO_COMPLETED,
    CMD_COLLISION_SHAPE_INFO_FAILED,
    CMD_LOAD_SOFT_BODY_FAILED,
    CMD_LOAD_SOFT_BODY_COMPLETED,

    CMD_SYNC_USER_DATA_COMPLETED,
    CMD_SYNC_USER_DATA_FAILED,
    CMD_REQUEST_USER_DATA_COMPLETED,
    CMD_REQUEST_USER_DATA_FAILED,
    CMD_ADD_USER_DATA_COMPLETED,
    CMD_ADD_USER_DATA_FAILED,
    CMD_REMOVE_USER_DATA_COMPLETED,
    CMD_REMOVE_USER_DATA_FAILED,
    CMD_REMOVE_STATE_COMPLETED,
    CMD_REMOVE_STATE_FAILED,

    CMD_REQUEST_MESH_DATA_COMPLETED,
    CMD_REQUEST_MESH_DATA_FAILED,

    CMD_MAX_SERVER_COMMANDS,
}

#[repr(C)]
#[derive(Debug)]
pub struct b3JointInfo {
    pub m_link_name: [c_char; 1024],
    pub m_joint_name: [c_char; 1024],
    pub m_joint_type: i32,
    pub m_q_index: i32,
    pub m_u_index: i32,
    pub m_joint_index: i32,
    pub m_flags: i32,
    pub m_joint_damping: f64,
    pub m_joint_friction: f64,
    pub m_joint_upper_limit: f64,
    pub m_joint_lower_limit: f64,
    pub m_joint_max_force: f64,
    pub m_joint_max_velocity: f64,
    pub m_parent_frame: [f64; 7],
    pub m_child_frame: [f64; 7],
    pub m_joint_axis: [f64; 3],
    pub m_parent_index: i32,
    pub m_q_size: i32,
    pub m_u_size: i32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct b3JointSensorState {
    pub m_joint_position: f64,
    pub m_joint_velocity: f64,
    pub m_joint_force_torque: [f64; 6],
    pub m_joint_motor_torque: f64,
}

#[repr(C)]
#[derive(Debug, Default)]
pub struct b3JointSensorState2 {
    pub m_joint_position: [f64; 4],
    pub m_joint_velocity: [f64; 3],
    pub m_joint_reaction_force_torque: [f64; 6],
    pub m_joint_motor_torque_multi_dof: [f64; 3],
    pub m_q_dof_size: c_int,
    pub m_u_dof_size: c_int,
}

#[repr(C)]
#[derive(Debug, Default)]
pub struct b3LinkState {
    pub m_world_position: [f64; 3],
    pub m_world_orientation: [f64; 4],
    pub m_local_inertial_position: [f64; 3],
    pub m_local_inertial_orientation: [f64; 4],
    pub m_world_link_frame_position: [f64; 3],
    pub m_world_link_frame_orientation: [f64; 4],
    ///only valid when ACTUAL_STATE_COMPUTE_LINKVELOCITY is set (b3RequestActualStateCommandComputeLinkVelocity)
    pub m_world_linear_velocity: [f64; 3],
    ///only valid when ACTUAL_STATE_COMPUTE_LINKVELOCITY is set (b3RequestActualStateCommandComputeLinkVelocity)
    pub m_world_angular_velocity: [f64; 3],

    pub m_world_aabb_min: [f64; 3],
    pub m_world_aabb_max: [f64; 3],
}

#[repr(C)]
#[derive(Debug)]
pub struct b3CameraImageData {
    pub m_pixel_width: c_int,
    pub m_pixel_height: c_int,
    pub m_rgb_color_data: *mut [u8],
    pub m_depth_values: *mut [f32],
    pub m_segmentation_mask_values: *mut [c_int],
}

impl Default for b3CameraImageData {
    fn default() -> Self {
        b3CameraImageData {
            m_pixel_width: 0,
            m_pixel_height: 0,
            m_rgb_color_data: &mut [0 as u8],
            m_depth_values: &mut [0. as f32],
            m_segmentation_mask_values: &mut [0],
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3KeyboardEventsData {
    pub m_numKeyboardEvents: c_int,
    pub m_keyboardEvents: *mut b3KeyboardEvent,
}

impl Default for b3KeyboardEventsData {
    fn default() -> Self {
        b3KeyboardEventsData {
            m_numKeyboardEvents: 0,
            m_keyboardEvents: [].as_mut_ptr(),
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct b3KeyboardEvent {
    pub m_keyCode: c_int,
    pub m_keyState: c_int,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3MouseEventsData {
    pub m_numMouseEvents: c_int,
    pub m_mouseEvents: *mut b3MouseEvent,
}

impl Default for b3MouseEventsData {
    fn default() -> Self {
        b3MouseEventsData {
            m_numMouseEvents: 0,
            m_mouseEvents: [].as_mut_ptr(),
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3MouseEvent {
    pub m_eventType: c_int,
    pub m_mousePosX: f32,
    pub m_mousePosY: f32,
    pub m_buttonIndex: c_int,
    pub m_buttonState: c_int,
}
