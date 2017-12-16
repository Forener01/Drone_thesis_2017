; Auto-generated. Do not edit!


(cl:in-package thesis_aurian-msg)


;//! \htmlinclude refdoors.msg.html

(cl:defclass <refdoors> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass refdoors (<refdoors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <refdoors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'refdoors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thesis_aurian-msg:<refdoors> is deprecated: use thesis_aurian-msg:refdoors instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <refdoors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_aurian-msg:data-val is deprecated.  Use thesis_aurian-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <refdoors>) ostream)
  "Serializes a message object of type '<refdoors>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <refdoors>) istream)
  "Deserializes a message object of type '<refdoors>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<refdoors>)))
  "Returns string type for a message object of type '<refdoors>"
  "thesis_aurian/refdoors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'refdoors)))
  "Returns string type for a message object of type 'refdoors"
  "thesis_aurian/refdoors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<refdoors>)))
  "Returns md5sum for a message object of type '<refdoors>"
  "f43a8e1b362b75baa741461b46adc7e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'refdoors)))
  "Returns md5sum for a message object of type 'refdoors"
  "f43a8e1b362b75baa741461b46adc7e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<refdoors>)))
  "Returns full string definition for message of type '<refdoors>"
  (cl:format cl:nil "# This contains all the reference doors and open spaces for the matching step.~%~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'refdoors)))
  "Returns full string definition for message of type 'refdoors"
  (cl:format cl:nil "# This contains all the reference doors and open spaces for the matching step.~%~%uint8[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <refdoors>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <refdoors>))
  "Converts a ROS message object to a list"
  (cl:list 'refdoors
    (cl:cons ':data (data msg))
))
