// Auto-generated. Do not edit!

// (in-package agile_grasp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let Grasp = require('../msg/Grasp.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class FindGraspsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_cloud = null;
      this.approach_offset = null;
    }
    else {
      if (initObj.hasOwnProperty('object_cloud')) {
        this.object_cloud = initObj.object_cloud
      }
      else {
        this.object_cloud = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('approach_offset')) {
        this.approach_offset = initObj.approach_offset
      }
      else {
        this.approach_offset = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FindGraspsRequest
    // Serialize message field [object_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.object_cloud, buffer, bufferOffset);
    // Serialize message field [approach_offset]
    bufferOffset = _serializer.float64(obj.approach_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FindGraspsRequest
    let len;
    let data = new FindGraspsRequest(null);
    // Deserialize message field [object_cloud]
    data.object_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [approach_offset]
    data.approach_offset = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.object_cloud);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agile_grasp/FindGraspsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1a17d86bb5d28b9268d803adf5b9d85';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 object_cloud
    float64 approach_offset
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FindGraspsRequest(null);
    if (msg.object_cloud !== undefined) {
      resolved.object_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.object_cloud)
    }
    else {
      resolved.object_cloud = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.approach_offset !== undefined) {
      resolved.approach_offset = msg.approach_offset;
    }
    else {
      resolved.approach_offset = 0.0
    }

    return resolved;
    }
};

class FindGraspsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grasps = null;
      this.grasp_poses = null;
      this.approach_poses = null;
    }
    else {
      if (initObj.hasOwnProperty('grasps')) {
        this.grasps = initObj.grasps
      }
      else {
        this.grasps = [];
      }
      if (initObj.hasOwnProperty('grasp_poses')) {
        this.grasp_poses = initObj.grasp_poses
      }
      else {
        this.grasp_poses = [];
      }
      if (initObj.hasOwnProperty('approach_poses')) {
        this.approach_poses = initObj.approach_poses
      }
      else {
        this.approach_poses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FindGraspsResponse
    // Serialize message field [grasps]
    // Serialize the length for message field [grasps]
    bufferOffset = _serializer.uint32(obj.grasps.length, buffer, bufferOffset);
    obj.grasps.forEach((val) => {
      bufferOffset = Grasp.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [grasp_poses]
    // Serialize the length for message field [grasp_poses]
    bufferOffset = _serializer.uint32(obj.grasp_poses.length, buffer, bufferOffset);
    obj.grasp_poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [approach_poses]
    // Serialize the length for message field [approach_poses]
    bufferOffset = _serializer.uint32(obj.approach_poses.length, buffer, bufferOffset);
    obj.approach_poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FindGraspsResponse
    let len;
    let data = new FindGraspsResponse(null);
    // Deserialize message field [grasps]
    // Deserialize array length for message field [grasps]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.grasps = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.grasps[i] = Grasp.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [grasp_poses]
    // Deserialize array length for message field [grasp_poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.grasp_poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.grasp_poses[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [approach_poses]
    // Deserialize array length for message field [approach_poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.approach_poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.approach_poses[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 100 * object.grasps.length;
    object.grasp_poses.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    object.approach_poses.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agile_grasp/FindGraspsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f075248670dd30604cea28fd03da384';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    agile_grasp/Grasp[] grasps
    geometry_msgs/PoseStamped[] grasp_poses
    geometry_msgs/PoseStamped[] approach_poses
    
    
    
    ================================================================================
    MSG: agile_grasp/Grasp
    geometry_msgs/Vector3 center
    geometry_msgs/Vector3 axis
    geometry_msgs/Vector3 approach
    geometry_msgs/Vector3 surface_center
    std_msgs/Float32 width
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FindGraspsResponse(null);
    if (msg.grasps !== undefined) {
      resolved.grasps = new Array(msg.grasps.length);
      for (let i = 0; i < resolved.grasps.length; ++i) {
        resolved.grasps[i] = Grasp.Resolve(msg.grasps[i]);
      }
    }
    else {
      resolved.grasps = []
    }

    if (msg.grasp_poses !== undefined) {
      resolved.grasp_poses = new Array(msg.grasp_poses.length);
      for (let i = 0; i < resolved.grasp_poses.length; ++i) {
        resolved.grasp_poses[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.grasp_poses[i]);
      }
    }
    else {
      resolved.grasp_poses = []
    }

    if (msg.approach_poses !== undefined) {
      resolved.approach_poses = new Array(msg.approach_poses.length);
      for (let i = 0; i < resolved.approach_poses.length; ++i) {
        resolved.approach_poses[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.approach_poses[i]);
      }
    }
    else {
      resolved.approach_poses = []
    }

    return resolved;
    }
};

module.exports = {
  Request: FindGraspsRequest,
  Response: FindGraspsResponse,
  md5sum() { return '3e976888ec7075e7e22c71cd9a0d3700'; },
  datatype() { return 'agile_grasp/FindGrasps'; }
};
