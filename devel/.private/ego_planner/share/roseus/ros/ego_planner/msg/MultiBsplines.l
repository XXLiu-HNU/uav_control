;; Auto-generated. Do not edit!


(when (boundp 'ego_planner::MultiBsplines)
  (if (not (find-package "EGO_PLANNER"))
    (make-package "EGO_PLANNER"))
  (shadow 'MultiBsplines (find-package "EGO_PLANNER")))
(unless (find-package "EGO_PLANNER::MULTIBSPLINES")
  (make-package "EGO_PLANNER::MULTIBSPLINES"))

(in-package "ROS")
;;//! \htmlinclude MultiBsplines.msg.html


(defclass ego_planner::MultiBsplines
  :super ros::object
  :slots (_drone_id_from _traj ))

(defmethod ego_planner::MultiBsplines
  (:init
   (&key
    ((:drone_id_from __drone_id_from) 0)
    ((:traj __traj) ())
    )
   (send-super :init)
   (setq _drone_id_from (round __drone_id_from))
   (setq _traj __traj)
   self)
  (:drone_id_from
   (&optional __drone_id_from)
   (if __drone_id_from (setq _drone_id_from __drone_id_from)) _drone_id_from)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:serialization-length
   ()
   (+
    ;; int32 _drone_id_from
    4
    ;; ego_planner/Bspline[] _traj
    (apply #'+ (send-all _traj :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _drone_id_from
       (write-long _drone_id_from s)
     ;; ego_planner/Bspline[] _traj
     (write-long (length _traj) s)
     (dolist (elem _traj)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _drone_id_from
     (setq _drone_id_from (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ego_planner/Bspline[] _traj
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traj (let (r) (dotimes (i n) (push (instance ego_planner::Bspline :init) r)) r))
     (dolist (elem- _traj)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ego_planner::MultiBsplines :md5sum-) "7f45adafc838893dace82d2af415aae3")
(setf (get ego_planner::MultiBsplines :datatype-) "ego_planner/MultiBsplines")
(setf (get ego_planner::MultiBsplines :definition-)
      "int32 drone_id_from

Bspline[] traj


================================================================================
MSG: ego_planner/Bspline
int32 drone_id

int32 order
int64 traj_id
time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :ego_planner/MultiBsplines "7f45adafc838893dace82d2af415aae3")


