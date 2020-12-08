//! Foreign function interface for Bullet C API.
#![allow(non_camel_case_types, non_snake_case)]
use std::os::raw::{c_char, c_int};

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

    pub fn b3InitStepSimulationCommand(
        physClient: b3PhysicsClientHandle,
    ) -> b3SharedMemoryCommandHandle;

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
