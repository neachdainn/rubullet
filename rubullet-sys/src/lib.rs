//! Foreign function interface for Bullet C API.
#![allow(non_camel_case_types, non_snake_case, clippy::upper_case_acronyms)]
use std::os::raw::{c_char, c_int, c_uchar};
use std::ptr::NonNull;

#[repr(C)]
pub struct b3PhysicsClientHandle__ {
    _unused: c_int,
}
pub type b3PhysicsClientHandle = NonNull<b3PhysicsClientHandle__>;

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
pub const MAX_SDF_BODIES: u32 = 512;
extern "C" {
    pub fn b3ConnectPhysicsDirect() -> Option<b3PhysicsClientHandle>;
    pub fn b3CreateInProcessPhysicsServerAndConnect(
        argc: c_int,
        argv: *mut *mut c_char,
    ) -> Option<b3PhysicsClientHandle>;
    pub fn b3CreateInProcessPhysicsServerAndConnectMainThread(
        argc: c_int,
        argv: *mut *mut c_char,
    ) -> Option<b3PhysicsClientHandle>;
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
        rootLocalInertialFrame: *mut *const f64,
        actualStateQ: *mut *const f64,
        actualStateQdot: *mut *const f64,
        jointReactionForces: *mut *const f64,
    ) -> c_int;

    pub fn b3GetStatusBodyIndices(
        statusHandle: b3SharedMemoryStatusHandle,
        bodyIndicesOut: *mut c_int,
        bodyIndicesCapacity: c_int,
    ) -> c_int;

    pub fn b3GetStatusBodyIndex(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3RequestCollisionInfoCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetStatusAABB(
        statusHandle: b3SharedMemoryStatusHandle,
        linkIndex: c_int,
        aabbMin: *mut f64,
        aabbMax: *mut f64,
    ) -> c_int;

    pub fn b3InitSyncBodyInfoCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3InitSyncUserDataCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

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
    pub fn b3LoadUrdfCommandSetUseMultiBody(
        commandHandle: b3SharedMemoryCommandHandle,
        useMultiBody: c_int,
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
    pub fn b3GetDynamicsInfoCommandInit(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
    ) -> b3SharedMemoryCommandHandle;

    #[doc = "given a body unique id and link index, return the dynamics information. See b3DynamicsInfo in SharedMemoryPublic.h"]
    pub fn b3GetDynamicsInfo(
        statusHandle: b3SharedMemoryStatusHandle,
        info: *mut b3DynamicsInfo,
    ) -> c_int;
    pub fn b3InitChangeDynamicsInfo(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3ChangeDynamicsInfoSetMass(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        mass: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetLocalInertiaDiagonal(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        localInertiaDiagonal: *const f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetAnisotropicFriction(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        anisotropicFriction: *const f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetJointLimit(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        jointLowerLimit: f64,
        jointUpperLimit: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetJointLimitForce(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        jointLimitForce: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetDynamicType(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        dynamicType: c_int,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetLateralFriction(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        lateralFriction: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetSpinningFriction(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        friction: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetRollingFriction(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        friction: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetRestitution(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        restitution: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetLinearDamping(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linearDamping: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetAngularDamping(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        angularDamping: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetJointDamping(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        jointDamping: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetContactStiffnessAndDamping(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        contactStiffness: f64,
        contactDamping: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetFrictionAnchor(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        frictionAnchor: c_int,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetCcdSweptSphereRadius(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        ccdSweptSphereRadius: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetContactProcessingThreshold(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
        contactProcessingThreshold: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetActivationState(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        activationState: c_int,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetMaxJointVelocity(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        maxJointVelocity: f64,
    ) -> c_int;

    pub fn b3ChangeDynamicsInfoSetCollisionMargin(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueId: c_int,
        collisionMargin: f64,
    ) -> c_int;

    pub fn b3InitRemoveBodyCommand(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    #[doc = "return the total number of bodies in the simulation"]
    pub fn b3GetNumBodies(physClient: b3PhysicsClientHandle) -> c_int;

    #[doc = " return the body unique id, given the index in range [0 , b3GetNumBodies() )"]
    pub fn b3GetBodyUniqueId(physClient: b3PhysicsClientHandle, serialIndex: c_int) -> c_int;

    #[doc = "given a body unique id, return the body information. See b3BodyInfo in SharedMemoryPublic.h"]
    pub fn b3GetBodyInfo(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        info: *mut b3BodyInfo,
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
    pub fn b3CreatePoseCommandSetBaseLinearVelocity(
        commandHandle: b3SharedMemoryCommandHandle,
        linVel: *const f64,
    ) -> c_int;

    pub fn b3CreatePoseCommandSetBaseAngularVelocity(
        commandHandle: b3SharedMemoryCommandHandle,
        angVel: *const f64,
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
    pub fn b3SaveStateCommandInit(physClient: b3PhysicsClientHandle)
        -> b3SharedMemoryCommandHandle;

    pub fn b3InitRemoveStateCommand(
        physClient: b3PhysicsClientHandle,
        stateId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetStatusGetStateId(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3LoadStateCommandInit(physClient: b3PhysicsClientHandle)
        -> b3SharedMemoryCommandHandle;

    pub fn b3LoadStateSetStateId(
        commandHandle: b3SharedMemoryCommandHandle,
        stateId: c_int,
    ) -> c_int;

    pub fn b3LoadStateSetFileName(
        commandHandle: b3SharedMemoryCommandHandle,
        fileName: *const c_char,
    ) -> c_int;

    pub fn b3LoadBulletCommandInit(
        physClient: b3PhysicsClientHandle,
        fileName: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3SaveBulletCommandInit(
        physClient: b3PhysicsClientHandle,
        fileName: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3LoadMJCFCommandInit(
        physClient: b3PhysicsClientHandle,
        fileName: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3LoadMJCFCommandInit2(
        commandHandle: b3SharedMemoryCommandHandle,
        fileName: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3LoadMJCFCommandSetFlags(commandHandle: b3SharedMemoryCommandHandle, flags: c_int);

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
    pub fn b3CollisionFilterCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3SetCollisionFilterPair(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueIdA: c_int,
        bodyUniqueIdB: c_int,
        linkIndexA: c_int,
        linkIndexB: c_int,
        enableCollision: c_int,
    );

    pub fn b3SetCollisionFilterGroupMask(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueIdA: c_int,
        linkIndexA: c_int,
        collisionFilterGroup: c_int,
        collisionFilterMask: c_int,
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
        txt: *const c_char,
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

    pub fn b3ConfigureOpenGLVisualizerSetViewMatrix(
        commandHandle: b3SharedMemoryCommandHandle,
        cameraDistance: f32,
        cameraPitch: f32,
        cameraYaw: f32,
        cameraTargetPosition: *const f32,
    );

    pub fn b3InitRequestOpenGLVisualizerCameraCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetStatusOpenGLVisualizerCamera(
        statusHandle: b3SharedMemoryStatusHandle,
        camera: *mut b3OpenGLVisualizerCameraInfo,
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
        txt: *const c_char,
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

    pub fn b3CreateRaycastCommandInit(
        physClient: b3PhysicsClientHandle,
        rayFromWorldX: f64,
        rayFromWorldY: f64,
        rayFromWorldZ: f64,
        rayToWorldX: f64,
        rayToWorldY: f64,
        rayToWorldZ: f64,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3CreateRaycastBatchCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3RaycastBatchSetNumThreads(
        commandHandle: b3SharedMemoryCommandHandle,
        numThreads: c_int,
    );

    pub fn b3RaycastBatchAddRay(
        commandHandle: b3SharedMemoryCommandHandle,
        rayFromWorld: *const f64,
        rayToWorld: *const f64,
    );

    pub fn b3RaycastBatchAddRays(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        rayFromWorld: *const f64,
        rayToWorld: *const f64,
        numRays: c_int,
    );

    pub fn b3RaycastBatchSetParentObject(
        commandHandle: b3SharedMemoryCommandHandle,
        parentObjectUniqueId: c_int,
        parentLinkIndex: c_int,
    );

    pub fn b3RaycastBatchSetReportHitNumber(
        commandHandle: b3SharedMemoryCommandHandle,
        reportHitNumber: c_int,
    );

    pub fn b3RaycastBatchSetCollisionFilterMask(
        commandHandle: b3SharedMemoryCommandHandle,
        collisionFilterMask: c_int,
    );

    pub fn b3RaycastBatchSetFractionEpsilon(
        commandHandle: b3SharedMemoryCommandHandle,
        fractionEpsilon: f64,
    );

    pub fn b3GetRaycastInformation(
        physClient: b3PhysicsClientHandle,
        raycastInfo: *mut b3RaycastInformation,
    );

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
    pub fn b3StateLoggingCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3StateLoggingStart(
        commandHandle: b3SharedMemoryCommandHandle,
        loggingType: c_int,
        fileName: *const c_char,
    ) -> c_int;

    pub fn b3StateLoggingAddLoggingObjectUniqueId(
        commandHandle: b3SharedMemoryCommandHandle,
        objectUniqueId: c_int,
    ) -> c_int;

    pub fn b3StateLoggingSetMaxLogDof(
        commandHandle: b3SharedMemoryCommandHandle,
        maxLogDof: c_int,
    ) -> c_int;

    pub fn b3StateLoggingSetLinkIndexA(
        commandHandle: b3SharedMemoryCommandHandle,
        linkIndexA: c_int,
    ) -> c_int;

    pub fn b3StateLoggingSetLinkIndexB(
        commandHandle: b3SharedMemoryCommandHandle,
        linkIndexB: c_int,
    ) -> c_int;

    pub fn b3StateLoggingSetBodyAUniqueId(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyAUniqueId: c_int,
    ) -> c_int;

    pub fn b3StateLoggingSetBodyBUniqueId(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyBUniqueId: c_int,
    ) -> c_int;

    pub fn b3StateLoggingSetDeviceTypeFilter(
        commandHandle: b3SharedMemoryCommandHandle,
        deviceTypeFilter: c_int,
    ) -> c_int;

    pub fn b3StateLoggingSetLogFlags(
        commandHandle: b3SharedMemoryCommandHandle,
        logFlags: c_int,
    ) -> c_int;

    pub fn b3GetStatusLoggingUniqueId(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3StateLoggingStop(
        commandHandle: b3SharedMemoryCommandHandle,
        loggingUid: c_int,
    ) -> c_int;

    pub fn b3ProfileTimingCommandInit(
        physClient: b3PhysicsClientHandle,
        name: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3SetProfileTimingDuractionInMicroSeconds(
        commandHandle: b3SharedMemoryCommandHandle,
        duration: c_int,
    );

    pub fn b3SetProfileTimingType(commandHandle: b3SharedMemoryCommandHandle, type_: c_int);

    pub fn b3PushProfileTiming(physClient: b3PhysicsClientHandle, timingName: *const c_char);

    pub fn b3PopProfileTiming(physClient: b3PhysicsClientHandle);

    pub fn b3SetTimeOut(physClient: b3PhysicsClientHandle, timeOutInSeconds: f64);

    pub fn b3GetTimeOut(physClient: b3PhysicsClientHandle) -> f64;
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

    #[doc = "the creation of collision shapes and rigid bodies etc is likely going to change,"]
    #[doc = "but good to have a b3CreateBoxShapeCommandInit for now"]
    pub fn b3CreateCollisionShapeCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3CreateCollisionShapeAddSphere(
        commandHandle: b3SharedMemoryCommandHandle,
        radius: f64,
    ) -> c_int;
    pub fn b3CreateCollisionShapeAddBox(
        commandHandle: b3SharedMemoryCommandHandle,
        halfExtents: *const f64,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddCapsule(
        commandHandle: b3SharedMemoryCommandHandle,
        radius: f64,
        height: f64,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddCylinder(
        commandHandle: b3SharedMemoryCommandHandle,
        radius: f64,
        height: f64,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddHeightfield(
        commandHandle: b3SharedMemoryCommandHandle,
        fileName: *const c_char,
        meshScale: *const f64,
        textureScaling: f64,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddHeightfield2(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        meshScale: *const f64,
        textureScaling: f64,
        heightfieldData: *mut f32,
        numHeightfieldRows: c_int,
        numHeightfieldColumns: c_int,
        replaceHeightfieldIndex: c_int,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddPlane(
        commandHandle: b3SharedMemoryCommandHandle,
        planeNormal: *const f64,
        planeConstant: f64,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddMesh(
        commandHandle: b3SharedMemoryCommandHandle,
        fileName: *const c_char,
        meshScale: *const f64,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddConvexMesh(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        meshScale: *const f64,
        vertices: *const f64,
        numVertices: c_int,
    ) -> c_int;

    pub fn b3CreateCollisionShapeAddConcaveMesh(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        meshScale: *const f64,
        vertices: *const f64,
        numVertices: c_int,
        indices: *const c_int,
        numIndices: c_int,
    ) -> c_int;

    pub fn b3CreateCollisionSetFlag(
        commandHandle: b3SharedMemoryCommandHandle,
        shapeIndex: c_int,
        flags: c_int,
    );

    pub fn b3CreateCollisionShapeSetChildTransform(
        commandHandle: b3SharedMemoryCommandHandle,
        shapeIndex: c_int,
        childPosition: *const f64,
        childOrientation: *const f64,
    );
    pub fn b3GetStatusCollisionShapeUniqueId(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3InitRemoveCollisionShapeCommand(
        physClient: b3PhysicsClientHandle,
        collisionShapeId: c_int,
    ) -> b3SharedMemoryCommandHandle;
    pub fn b3CreateVisualShapeCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3CreateVisualShapeAddSphere(
        commandHandle: b3SharedMemoryCommandHandle,
        radius: f64,
    ) -> c_int;
    pub fn b3CreateVisualShapeAddBox(
        commandHandle: b3SharedMemoryCommandHandle,
        halfExtents: *const f64,
    ) -> c_int;

    pub fn b3CreateVisualShapeAddCapsule(
        commandHandle: b3SharedMemoryCommandHandle,
        radius: f64,
        height: f64,
    ) -> c_int;

    pub fn b3CreateVisualShapeAddCylinder(
        commandHandle: b3SharedMemoryCommandHandle,
        radius: f64,
        height: f64,
    ) -> c_int;

    pub fn b3CreateVisualShapeAddPlane(
        commandHandle: b3SharedMemoryCommandHandle,
        planeNormal: *const f64,
        planeConstant: f64,
    ) -> c_int;

    pub fn b3CreateVisualShapeAddMesh(
        commandHandle: b3SharedMemoryCommandHandle,
        fileName: *const c_char,
        meshScale: *const f64,
    ) -> c_int;

    pub fn b3CreateVisualShapeAddMesh2(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        meshScale: *const f64,
        vertices: *const f64,
        numVertices: c_int,
        indices: *const c_int,
        numIndices: c_int,
        normals: *const f64,
        numNormals: c_int,
        uvs: *const f64,
        numUVs: c_int,
    ) -> c_int;

    pub fn b3CreateVisualSetFlag(
        commandHandle: b3SharedMemoryCommandHandle,
        shapeIndex: c_int,
        flags: c_int,
    );

    pub fn b3CreateVisualShapeSetChildTransform(
        commandHandle: b3SharedMemoryCommandHandle,
        shapeIndex: c_int,
        childPosition: *const f64,
        childOrientation: *const f64,
    );

    pub fn b3CreateVisualShapeSetSpecularColor(
        commandHandle: b3SharedMemoryCommandHandle,
        shapeIndex: c_int,
        specularColor: *const f64,
    );

    pub fn b3CreateVisualShapeSetRGBAColor(
        commandHandle: b3SharedMemoryCommandHandle,
        shapeIndex: c_int,
        rgbaColor: *const f64,
    );

    pub fn b3GetStatusVisualShapeUniqueId(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3CreateMultiBodyCommandInit(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3CreateMultiBodyBase(
        commandHandle: b3SharedMemoryCommandHandle,
        mass: f64,
        collisionShapeUnique: c_int,
        visualShapeUniqueId: c_int,
        basePosition: *const f64,
        baseOrientation: *const f64,
        baseInertialFramePosition: *const f64,
        baseInertialFrameOrientation: *const f64,
    ) -> c_int;

    pub fn b3CreateMultiBodyLink(
        commandHandle: b3SharedMemoryCommandHandle,
        linkMass: f64,
        linkCollisionShapeIndex: f64,
        linkVisualShapeIndex: f64,
        linkPosition: *const f64,
        linkOrientation: *const f64,
        linkInertialFramePosition: *const f64,
        linkInertialFrameOrientation: *const f64,
        linkParentIndex: c_int,
        linkJointType: c_int,
        linkJointAxis: *const f64,
    ) -> c_int;

    pub fn b3CreateMultiBodySetBatchPositions(
        physClient: b3PhysicsClientHandle,
        commandHandle: b3SharedMemoryCommandHandle,
        batchPositions: *mut f64,
        numBatchObjects: c_int,
    ) -> c_int;

    pub fn b3CreateMultiBodyUseMaximalCoordinates(commandHandle: b3SharedMemoryCommandHandle);

    pub fn b3CreateMultiBodySetFlags(commandHandle: b3SharedMemoryCommandHandle, flags: c_int);

    #[doc = "request an contact point information"]
    pub fn b3InitRequestContactPointInformation(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3SetContactFilterBodyA(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueIdA: c_int,
    );

    pub fn b3SetContactFilterBodyB(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueIdB: c_int,
    );

    pub fn b3SetContactFilterLinkA(commandHandle: b3SharedMemoryCommandHandle, linkIndexA: c_int);

    pub fn b3SetContactFilterLinkB(commandHandle: b3SharedMemoryCommandHandle, linkIndexB: c_int);

    pub fn b3GetContactPointInformation(
        physClient: b3PhysicsClientHandle,
        contactPointData: *mut b3ContactInformation,
    );

    #[doc = "compute the closest points between two bodies"]
    pub fn b3InitClosestDistanceQuery(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3SetClosestDistanceFilterBodyA(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueIdA: c_int,
    );

    pub fn b3SetClosestDistanceFilterLinkA(
        commandHandle: b3SharedMemoryCommandHandle,
        linkIndexA: c_int,
    );

    pub fn b3SetClosestDistanceFilterBodyB(
        commandHandle: b3SharedMemoryCommandHandle,
        bodyUniqueIdB: c_int,
    );

    pub fn b3SetClosestDistanceFilterLinkB(
        commandHandle: b3SharedMemoryCommandHandle,
        linkIndexB: c_int,
    );

    pub fn b3SetClosestDistanceThreshold(commandHandle: b3SharedMemoryCommandHandle, distance: f64);

    pub fn b3SetClosestDistanceFilterCollisionShapeA(
        commandHandle: b3SharedMemoryCommandHandle,
        collisionShapeA: c_int,
    );

    pub fn b3SetClosestDistanceFilterCollisionShapeB(
        commandHandle: b3SharedMemoryCommandHandle,
        collisionShapeB: c_int,
    );

    pub fn b3SetClosestDistanceFilterCollisionShapePositionA(
        commandHandle: b3SharedMemoryCommandHandle,
        collisionShapePositionA: *const f64,
    );

    pub fn b3SetClosestDistanceFilterCollisionShapePositionB(
        commandHandle: b3SharedMemoryCommandHandle,
        collisionShapePositionB: *const f64,
    );

    pub fn b3SetClosestDistanceFilterCollisionShapeOrientationA(
        commandHandle: b3SharedMemoryCommandHandle,
        collisionShapeOrientationA: *const f64,
    );

    pub fn b3SetClosestDistanceFilterCollisionShapeOrientationB(
        commandHandle: b3SharedMemoryCommandHandle,
        collisionShapeOrientationB: *const f64,
    );

    pub fn b3GetClosestPointInformation(
        physClient: b3PhysicsClientHandle,
        contactPointInfo: *mut b3ContactInformation,
    );

    #[doc = "get all the bodies that touch a given axis aligned bounding box specified in world space (min and max coordinates)"]
    pub fn b3InitAABBOverlapQuery(
        physClient: b3PhysicsClientHandle,
        aabbMin: *const f64,
        aabbMax: *const f64,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetAABBOverlapResults(physClient: b3PhysicsClientHandle, data: *mut b3AABBOverlapData);
    pub fn b3InitRequestVisualShapeInformation(
        physClient: b3PhysicsClientHandle,
        bodyUniqueIdA: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetVisualShapeInformation(
        physClient: b3PhysicsClientHandle,
        visualShapeInfo: *mut b3VisualShapeInformation,
    );

    pub fn b3InitRequestCollisionShapeInformation(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        linkIndex: c_int,
    ) -> b3SharedMemoryCommandHandle;

    // pub fn b3GetCollisionShapeInformation(
    //     physClient: b3PhysicsClientHandle,
    //     collisionShapeInfo: *mut b3CollisionShapeInformation,
    // );

    pub fn b3InitLoadTexture(
        physClient: b3PhysicsClientHandle,
        filename: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetStatusTextureUniqueId(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    pub fn b3CreateChangeTextureCommandInit(
        physClient: b3PhysicsClientHandle,
        textureUniqueId: c_int,
        width: c_int,
        height: c_int,
        rgbPixels: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitUpdateVisualShape(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        jointIndex: c_int,
        shapeIndex: c_int,
        textureUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitUpdateVisualShape2(
        physClient: b3PhysicsClientHandle,
        bodyUniqueId: c_int,
        jointIndex: c_int,
        shapeIndex: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3UpdateVisualShapeTexture(
        commandHandle: b3SharedMemoryCommandHandle,
        textureUniqueId: c_int,
    );

    pub fn b3UpdateVisualShapeRGBAColor(
        commandHandle: b3SharedMemoryCommandHandle,
        rgbaColor: *const f64,
    );

    pub fn b3UpdateVisualShapeFlags(commandHandle: b3SharedMemoryCommandHandle, flags: c_int);

    pub fn b3UpdateVisualShapeSpecularColor(
        commandHandle: b3SharedMemoryCommandHandle,
        specularColor: *const f64,
    );

    pub fn b3InitPhysicsParamCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitPhysicsParamCommand2(
        commandHandle: b3SharedMemoryCommandHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3PhysicsParamSetGravity(
        commandHandle: b3SharedMemoryCommandHandle,
        gravx: f64,
        gravy: f64,
        gravz: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetTimeStep(
        commandHandle: b3SharedMemoryCommandHandle,
        timeStep: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetDefaultContactERP(
        commandHandle: b3SharedMemoryCommandHandle,
        defaultContactERP: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetDefaultNonContactERP(
        commandHandle: b3SharedMemoryCommandHandle,
        defaultNonContactERP: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetDefaultFrictionERP(
        commandHandle: b3SharedMemoryCommandHandle,
        frictionERP: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetDefaultGlobalCFM(
        commandHandle: b3SharedMemoryCommandHandle,
        defaultGlobalCFM: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetDefaultFrictionCFM(
        commandHandle: b3SharedMemoryCommandHandle,
        frictionCFM: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetNumSubSteps(
        commandHandle: b3SharedMemoryCommandHandle,
        numSubSteps: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetRealTimeSimulation(
        commandHandle: b3SharedMemoryCommandHandle,
        enableRealTimeSimulation: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetNumSolverIterations(
        commandHandle: b3SharedMemoryCommandHandle,
        numSolverIterations: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetNumNonContactInnerIterations(
        commandHandle: b3SharedMemoryCommandHandle,
        numMotorIterations: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetWarmStartingFactor(
        commandHandle: b3SharedMemoryCommandHandle,
        warmStartingFactor: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetArticulatedWarmStartingFactor(
        commandHandle: b3SharedMemoryCommandHandle,
        warmStartingFactor: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetCollisionFilterMode(
        commandHandle: b3SharedMemoryCommandHandle,
        filterMode: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetUseSplitImpulse(
        commandHandle: b3SharedMemoryCommandHandle,
        useSplitImpulse: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetSplitImpulsePenetrationThreshold(
        commandHandle: b3SharedMemoryCommandHandle,
        splitImpulsePenetrationThreshold: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetContactBreakingThreshold(
        commandHandle: b3SharedMemoryCommandHandle,
        contactBreakingThreshold: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetMaxNumCommandsPer1ms(
        commandHandle: b3SharedMemoryCommandHandle,
        maxNumCmdPer1ms: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetEnableFileCaching(
        commandHandle: b3SharedMemoryCommandHandle,
        enableFileCaching: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetRestitutionVelocityThreshold(
        commandHandle: b3SharedMemoryCommandHandle,
        restitutionVelocityThreshold: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetEnableConeFriction(
        commandHandle: b3SharedMemoryCommandHandle,
        enableConeFriction: c_int,
    ) -> c_int;

    pub fn b3PhysicsParameterSetDeterministicOverlappingPairs(
        commandHandle: b3SharedMemoryCommandHandle,
        deterministicOverlappingPairs: c_int,
    ) -> c_int;

    pub fn b3PhysicsParameterSetAllowedCcdPenetration(
        commandHandle: b3SharedMemoryCommandHandle,
        allowedCcdPenetration: f64,
    ) -> c_int;

    pub fn b3PhysicsParameterSetJointFeedbackMode(
        commandHandle: b3SharedMemoryCommandHandle,
        jointFeedbackMode: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetSolverResidualThreshold(
        commandHandle: b3SharedMemoryCommandHandle,
        solverResidualThreshold: f64,
    ) -> c_int;

    pub fn b3PhysicsParamSetContactSlop(
        commandHandle: b3SharedMemoryCommandHandle,
        contactSlop: f64,
    ) -> c_int;

    pub fn b3PhysicsParameterSetEnableSAT(
        commandHandle: b3SharedMemoryCommandHandle,
        enableSAT: c_int,
    ) -> c_int;

    pub fn b3PhysicsParameterSetConstraintSolverType(
        commandHandle: b3SharedMemoryCommandHandle,
        constraintSolverType: c_int,
    ) -> c_int;

    pub fn b3PhysicsParameterSetMinimumSolverIslandSize(
        commandHandle: b3SharedMemoryCommandHandle,
        minimumSolverIslandSize: c_int,
    ) -> c_int;

    pub fn b3PhysicsParamSetSolverAnalytics(
        commandHandle: b3SharedMemoryCommandHandle,
        reportSolverAnalytics: c_int,
    ) -> c_int;

    pub fn b3PhysicsParameterSetSparseSdfVoxelSize(
        commandHandle: b3SharedMemoryCommandHandle,
        sparseSdfVoxelSize: f64,
    ) -> c_int;

    pub fn b3InitRequestPhysicsParamCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetStatusPhysicsSimulationParameters(
        statusHandle: b3SharedMemoryStatusHandle,
        params: *mut b3PhysicsSimulationParameters,
    ) -> c_int;

    pub fn b3PhysicsParamSetInternalSimFlags(
        commandHandle: b3SharedMemoryCommandHandle,
        flags: c_int,
    ) -> c_int;

    pub fn b3LoadSdfCommandInit(
        physClient: b3PhysicsClientHandle,
        sdfFileName: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3LoadSdfCommandSetUseMultiBody(
        commandHandle: b3SharedMemoryCommandHandle,
        useMultiBody: c_int,
    ) -> c_int;

    pub fn b3LoadSdfCommandSetUseGlobalScaling(
        commandHandle: b3SharedMemoryCommandHandle,
        globalScaling: f64,
    ) -> c_int;

    pub fn b3SaveWorldCommandInit(
        physClient: b3PhysicsClientHandle,
        sdfFileName: *const c_char,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitCreateUserConstraintCommand(
        physClient: b3PhysicsClientHandle,
        parentBodyUniqueId: c_int,
        parentJointIndex: c_int,
        childBodyUniqueId: c_int,
        childJointIndex: c_int,
        info: *mut b3JointInfo,
    ) -> b3SharedMemoryCommandHandle;

    #[doc = "return a unique id for the user constraint, after successful creation, or -1 for an invalid constraint id"]
    pub fn b3GetStatusUserConstraintUniqueId(statusHandle: b3SharedMemoryStatusHandle) -> c_int;

    #[doc = "change parameters of an existing user constraint"]
    pub fn b3InitChangeUserConstraintCommand(
        physClient: b3PhysicsClientHandle,
        userConstraintUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3InitChangeUserConstraintSetPivotInB(
        commandHandle: b3SharedMemoryCommandHandle,
        jointChildPivot: *const f64,
    ) -> c_int;

    pub fn b3InitChangeUserConstraintSetFrameInB(
        commandHandle: b3SharedMemoryCommandHandle,
        jointChildFrameOrn: *const f64,
    ) -> c_int;

    pub fn b3InitChangeUserConstraintSetMaxForce(
        commandHandle: b3SharedMemoryCommandHandle,
        maxAppliedForce: f64,
    ) -> c_int;

    pub fn b3InitChangeUserConstraintSetGearRatio(
        commandHandle: b3SharedMemoryCommandHandle,
        gearRatio: f64,
    ) -> c_int;

    pub fn b3InitChangeUserConstraintSetGearAuxLink(
        commandHandle: b3SharedMemoryCommandHandle,
        gearAuxLink: c_int,
    ) -> c_int;

    pub fn b3InitChangeUserConstraintSetRelativePositionTarget(
        commandHandle: b3SharedMemoryCommandHandle,
        relativePositionTarget: f64,
    ) -> c_int;

    pub fn b3InitChangeUserConstraintSetERP(
        commandHandle: b3SharedMemoryCommandHandle,
        erp: f64,
    ) -> c_int;

    pub fn b3InitRemoveUserConstraintCommand(
        physClient: b3PhysicsClientHandle,
        userConstraintUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetNumUserConstraints(physClient: b3PhysicsClientHandle) -> c_int;

    pub fn b3InitGetUserConstraintStateCommand(
        physClient: b3PhysicsClientHandle,
        constraintUniqueId: c_int,
    ) -> b3SharedMemoryCommandHandle;

    pub fn b3GetStatusUserConstraintState(
        statusHandle: b3SharedMemoryStatusHandle,
        constraintState: *mut b3UserConstraintState,
    ) -> c_int;

    pub fn b3GetUserConstraintInfo(
        physClient: b3PhysicsClientHandle,
        constraintUniqueId: c_int,
        info: *mut b3UserConstraint,
    ) -> c_int;
    #[doc = " return the user constraint id, given the index in range [0 , b3GetNumUserConstraints() )"]
    pub fn b3GetUserConstraintId(physClient: b3PhysicsClientHandle, serialIndex: c_int) -> c_int;
}

#[repr(C)]
#[derive(Clone, Copy)]
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
    pub m_joint_lower_limit: f64,
    pub m_joint_upper_limit: f64,
    pub m_joint_max_force: f64,
    pub m_joint_max_velocity: f64,
    pub m_parent_frame: [f64; 7],
    pub m_child_frame: [f64; 7],
    pub m_joint_axis: [f64; 3],
    pub m_parent_index: i32,
    pub m_q_size: i32,
    pub m_u_size: i32,
}
impl Default for b3JointInfo {
    fn default() -> Self {
        b3JointInfo {
            m_link_name: [2; 1024],
            m_joint_name: [2; 1024],
            m_joint_type: 0,
            m_q_index: 0,
            m_u_index: 0,
            m_joint_index: 0,
            m_flags: 0,
            m_joint_damping: 0.0,
            m_joint_friction: 0.0,
            m_joint_lower_limit: 0.0,
            m_joint_upper_limit: 0.0,
            m_joint_max_force: 0.0,
            m_joint_max_velocity: 0.0,
            m_parent_frame: [0.; 7],
            m_child_frame: [0.; 7],
            m_joint_axis: [0.; 3],
            m_parent_index: 0,
            m_q_size: 0,
            m_u_size: 0,
        }
    }
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
#[derive(Copy, Clone)]
pub struct b3BodyInfo {
    pub m_baseName: [c_char; 1024usize],
    pub m_bodyName: [c_char; 1024usize],
}
#[repr(C)]
#[derive(Debug)]
pub struct b3CameraImageData {
    pub m_pixel_width: c_int,
    pub m_pixel_height: c_int,
    pub m_rgb_color_data: *const c_uchar,
    pub m_depth_values: *const f32,
    pub m_segmentation_mask_values: *const c_int,
}

impl Default for b3CameraImageData {
    fn default() -> Self {
        b3CameraImageData {
            m_pixel_width: 0,
            m_pixel_height: 0,
            m_rgb_color_data: &(0_u8),
            m_depth_values: &(0_f32),
            m_segmentation_mask_values: &0,
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

pub const B3_MAX_NUM_INDICES: usize = if cfg!(target_os = "macos") {
    32768
} else {
    524288
};
pub const B3_MAX_NUM_VERTICES: usize = if cfg!(target_os = "macos") {
    8192
} else {
    131072
};
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3VisualShapeInformation {
    pub m_numVisualShapes: c_int,
    pub m_visualShapeData: *mut b3VisualShapeData,
}
impl Default for b3VisualShapeInformation {
    fn default() -> Self {
        b3VisualShapeInformation {
            m_numVisualShapes: 0,
            m_visualShapeData: [].as_mut_ptr(),
        }
    }
}
impl Default for b3VisualShapeData {
    fn default() -> Self {
        b3VisualShapeData {
            m_objectUniqueId: 0,
            m_linkIndex: 0,
            m_visualGeometryType: 0,
            m_dimensions: [0.; 3],
            m_meshAssetFileName: [0; 1024],
            m_localVisualFrame: [0.; 7],
            m_rgbaColor: [0.; 4],
            m_tinyRendererTextureId: 0,
            m_textureUniqueId: 0,
            m_openglTextureId: 0,
        }
    }
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct b3VisualShapeData {
    pub m_objectUniqueId: c_int,
    pub m_linkIndex: c_int,
    pub m_visualGeometryType: c_int,
    pub m_dimensions: [f64; 3usize],
    pub m_meshAssetFileName: [c_char; 1024usize],
    pub m_localVisualFrame: [f64; 7usize],
    pub m_rgbaColor: [f64; 4usize],
    pub m_tinyRendererTextureId: c_int,
    pub m_textureUniqueId: c_int,
    pub m_openglTextureId: c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3UserConstraint {
    pub m_parentBodyIndex: c_int,
    pub m_parentJointIndex: c_int,
    pub m_childBodyIndex: c_int,
    pub m_childJointIndex: c_int,
    pub m_parentFrame: [f64; 7usize],
    pub m_childFrame: [f64; 7usize],
    pub m_jointAxis: [f64; 3usize],
    pub m_jointType: c_int,
    pub m_maxAppliedForce: f64,
    pub m_userConstraintUniqueId: c_int,
    pub m_gearRatio: f64,
    pub m_gearAuxLink: c_int,
    pub m_relativePositionTarget: f64,
    pub m_erp: f64,
}
impl Default for b3UserConstraint {
    fn default() -> Self {
        b3UserConstraint {
            m_parentBodyIndex: 0,
            m_parentJointIndex: 0,
            m_childBodyIndex: 0,
            m_childJointIndex: 0,
            m_parentFrame: [0.; 7],
            m_childFrame: [0.; 7],
            m_jointAxis: [0.; 3],
            m_jointType: 0,
            m_maxAppliedForce: 0.0,
            m_userConstraintUniqueId: 0,
            m_gearRatio: 0.0,
            m_gearAuxLink: 0,
            m_relativePositionTarget: 0.0,
            m_erp: 0.0,
        }
    }
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3UserConstraintState {
    pub m_appliedConstraintForces: [f64; 6usize],
    pub m_numDofs: c_int,
}
impl Default for b3UserConstraintState {
    fn default() -> Self {
        b3UserConstraintState {
            m_appliedConstraintForces: [0.; 6],
            m_numDofs: 0,
        }
    }
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3DynamicsInfo {
    pub m_mass: f64,
    pub m_localInertialDiagonal: [f64; 3usize],
    pub m_localInertialFrame: [f64; 7usize],
    pub m_lateralFrictionCoeff: f64,
    pub m_rollingFrictionCoeff: f64,
    pub m_spinningFrictionCoeff: f64,
    pub m_restitution: f64,
    pub m_contactStiffness: f64,
    pub m_contactDamping: f64,
    pub m_activationState: c_int,
    pub m_bodyType: c_int,
    pub m_angularDamping: f64,
    pub m_linearDamping: f64,
    pub m_ccdSweptSphereRadius: f64,
    pub m_contactProcessingThreshold: f64,
    pub m_frictionAnchor: c_int,
    pub m_collisionMargin: f64,
    pub m_dynamicType: c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3AABBOverlapData {
    pub m_numOverlappingObjects: c_int,
    pub m_overlappingObjects: *mut b3OverlappingObject,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3OverlappingObject {
    pub m_objectUniqueId: c_int,
    pub m_linkIndex: c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3ContactInformation {
    pub m_numContactPoints: c_int,
    pub m_contactPointData: *mut b3ContactPointData,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3ContactPointData {
    pub m_contactFlags: c_int,
    pub m_bodyUniqueIdA: c_int,
    pub m_bodyUniqueIdB: c_int,
    pub m_linkIndexA: c_int,
    pub m_linkIndexB: c_int,
    pub m_positionOnAInWS: [f64; 3usize],
    pub m_positionOnBInWS: [f64; 3usize],
    pub m_contactNormalOnBInWS: [f64; 3usize],
    pub m_contactDistance: f64,
    pub m_normalForce: f64,
    pub m_linearFrictionForce1: f64,
    pub m_linearFrictionForce2: f64,
    pub m_linearFrictionDirection1: [f64; 3usize],
    pub m_linearFrictionDirection2: [f64; 3usize],
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct b3PhysicsSimulationParameters {
    pub m_deltaTime: f64,
    pub m_simulationTimestamp: f64,
    pub m_gravityAcceleration: [f64; 3usize],
    pub m_numSimulationSubSteps: c_int,
    pub m_numSolverIterations: c_int,
    pub m_warmStartingFactor: f64,
    pub m_articulatedWarmStartingFactor: f64,
    pub m_useRealTimeSimulation: c_int,
    pub m_useSplitImpulse: c_int,
    pub m_splitImpulsePenetrationThreshold: f64,
    pub m_contactBreakingThreshold: f64,
    pub m_internalSimFlags: c_int,
    pub m_defaultContactERP: f64,
    pub m_collisionFilterMode: c_int,
    pub m_enableFileCaching: c_int,
    pub m_restitutionVelocityThreshold: f64,
    pub m_defaultNonContactERP: f64,
    pub m_frictionERP: f64,
    pub m_defaultGlobalCFM: f64,
    pub m_frictionCFM: f64,
    pub m_enableConeFriction: c_int,
    pub m_deterministicOverlappingPairs: c_int,
    pub m_allowedCcdPenetration: f64,
    pub m_jointFeedbackMode: c_int,
    pub m_solverResidualThreshold: f64,
    pub m_contactSlop: f64,
    pub m_enableSAT: c_int,
    pub m_constraintSolverType: c_int,
    pub m_minimumSolverIslandSize: c_int,
    pub m_reportSolverAnalytics: c_int,
    pub m_sparseSdfVoxelSize: f64,
    pub m_numNonContactInnerIterations: c_int,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct b3OpenGLVisualizerCameraInfo {
    pub m_width: c_int,
    pub m_height: c_int,
    pub m_viewMatrix: [f32; 16usize],
    pub m_projectionMatrix: [f32; 16usize],
    pub m_camUp: [f32; 3usize],
    pub m_camForward: [f32; 3usize],
    pub m_horizontal: [f32; 3usize],
    pub m_vertical: [f32; 3usize],
    pub m_yaw: f32,
    pub m_pitch: f32,
    pub m_dist: f32,
    pub m_target: [f32; 3usize],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct b3RaycastInformation {
    pub m_numRayHits: c_int,
    pub m_rayHits: *mut b3RayHitInfo,
}
impl Default for b3RaycastInformation {
    fn default() -> Self {
        b3RaycastInformation {
            m_numRayHits: 0,
            m_rayHits: [].as_mut_ptr(),
        }
    }
}
#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct b3RayHitInfo {
    pub m_hitFraction: f64,
    pub m_hitObjectUniqueId: c_int,
    pub m_hitObjectLinkIndex: c_int,
    pub m_hitPositionWorld: [f64; 3usize],
    pub m_hitNormalWorld: [f64; 3usize],
}
pub const MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING: usize = 16384;
