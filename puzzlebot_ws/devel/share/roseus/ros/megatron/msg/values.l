;; Auto-generated. Do not edit!


(when (boundp 'megatron::values)
  (if (not (find-package "MEGATRON"))
    (make-package "MEGATRON"))
  (shadow 'values (find-package "MEGATRON")))
(unless (find-package "MEGATRON::VALUES")
  (make-package "MEGATRON::VALUES"))

(in-package "ROS")
;;//! \htmlinclude values.msg.html


(defclass megatron::values
  :super ros::object
  :slots (_direction _velocity ))

(defmethod megatron::values
  (:init
   (&key
    ((:direction __direction) "")
    ((:velocity __velocity) "")
    )
   (send-super :init)
   (setq _direction (string __direction))
   (setq _velocity (string __velocity))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; string _direction
    4 (length _direction)
    ;; string _velocity
    4 (length _velocity)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _direction
       (write-long (length _direction) s) (princ _direction s)
     ;; string _velocity
       (write-long (length _velocity) s) (princ _velocity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _velocity
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _velocity (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get megatron::values :md5sum-) "4cf332b5242ea2dc6436208b418b4da2")
(setf (get megatron::values :datatype-) "megatron/values")
(setf (get megatron::values :definition-)
      "string direction
string velocity
")



(provide :megatron/values "4cf332b5242ea2dc6436208b418b4da2")


