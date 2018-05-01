// Auto-generated. Do not edit!

// (in-package agile_grasp.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Grasp {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.center = null;
      this.axis = null;
      this.approach = null;
      this.surface_center = null;
      this.width = null;
    }
    else {
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('axis')) {
        this.axis = initObj.axis
      }
      else {
        this.axis = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('approach')) {
        this.approach = initObj.approach
      }
      else {
        this.approach = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('surface_center')) {
        this.surface_center = initObj.surface_center
      }
      else {
        this.surface_center = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = new std_msgs.msg.Float32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Grasp
    // Serialize message field [center]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [axis]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.axis, buffer, bufferOffset);
    // Serialize message field [approach]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.approach, buffer, bufferOffset);
    // Serialize message field [surface_center]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.surface_center, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.width, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Grasp
    let len;
    let data = new Grasp(null);
    // Deserialize message field [center]
    data.center = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [axis]
    data.axis = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [approach]
    data.approach = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [surface_center]
    data.surface_center = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 100;
  }

  static datatype() {
    // Returns string type for a message object
    return 'agile_grasp/Grasp';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e812ccd1fa0a0ad5be105b582346ad98';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Grasp(null);
    if (msg.center !== undefined) {
      resolved.center = geometry_msgs.msg.Vector3.Resolve(msg.center)
    }
    else {
      resolved.center = new geometry_msgs.msg.Vector3()
    }

    if (msg.axis !== undefined) {
      resolved.axis = geometry_msgs.msg.Vector3.Resolve(msg.axis)
    }
    else {
      resolved.axis = new geometry_msgs.msg.Vector3()
    }

    if (msg.approach !== undefined) {
      resolved.approach = geometry_msgs.msg.Vector3.Resolve(msg.approach)
    }
    else {
      resolved.approach = new geometry_msgs.msg.Vector3()
    }

    if (msg.surface_center !== undefined) {
      resolved.surface_center = geometry_msgs.msg.Vector3.Resolve(msg.surface_center)
    }
    else {
      resolved.surface_center = new geometry_msgs.msg.Vector3()
    }

    if (msg.width !== undefined) {
      resolved.width = std_msgs.msg.Float32.Resolve(msg.width)
    }
    else {
      resolved.width = new std_msgs.msg.Float32()
    }

    return resolved;
    }
};

module.exports = Grasp;
