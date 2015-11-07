;;; Copyright (c) 2015, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :shopping-scenario-executive)

(defvar *markers* (make-hash-table :test 'equal))
(defvar *rack-level-dimensions* (make-hash-table :test 'equal))
(defvar *object-dimensions* (make-hash-table :test 'equal))

(defun get-rack-level-dimensions (level)
  (or (gethash level *rack-level-dimensions*)
      (let* ((rack (first (get-racks)))
             (rack-level (get-rack-on-level rack 1))
             (rack-level-dimensions (get-item-dimensions rack-level)))
        (setf (gethash level *rack-level-dimensions*)
              rack-level-dimensions))))

(defun get-item-dimensions-cached (object)
  (or (gethash object *object-dimensions*)
      (setf (gethash object *object-dimensions*)
            (get-item-dimensions object))))

(defun pose->relative-pose (pose-stamped frame-id)
  (cl-tf2:ensure-pose-stamped-transformed
   *tf* pose-stamped frame-id))

(defun pose->rack-relative-pose (pose-stamped)
  (pose->relative-pose pose-stamped "/shopping_rack"))

(defun pose->map-relative-pose (pose-stamped)
  (pose->relative-pose pose-stamped "/map"))

(defun make-level-relative-pose (level x y z
                                 &key (roll 0.0)
                                   (pitch 0.0)
                                   (yaw 0.0))
  (let ((frame-id (concatenate 'string "/rack_level_"
                               (write-to-string level)))
        (yaw (+ (/ pi 2) yaw)))
    (tf:make-pose-stamped
     frame-id 0.0
     (tf:make-3d-vector x y z)
     (tf:euler->quaternion :ax roll :ay pitch :az yaw))))

(defun make-zone-pose (level zone x y
                       &key (orientation (tf:euler->quaternion)))
  (let* ((rack-level-dimensions (get-rack-level-dimensions level))
         (zones-per-level 4)
         (zone-width (/ (elt rack-level-dimensions 1) zones-per-level))
         (zone-height 0.4)
         (c-x (+ (- (* (+ zone 0.5) zone-width)
                    (/ (elt rack-level-dimensions 1) 2)
                    )
                 y))
         (c-y x)
         (c-z (+ (/ zone-height 2)
                 (/ (elt rack-level-dimensions 2) 2))))
    (tf:make-pose-stamped
     (concatenate
      'string
      "/rack_level_" (write-to-string level))
     0.0
     (tf:make-3d-vector c-x c-y c-z)
     orientation)))

(defun display-zones (&key highlight-zones)
  (let* ((rack-level-dimensions (get-rack-level-dimensions 1))
         (levels 4)
         (zones-per-level 4)
         (zone-width (/ (elt rack-level-dimensions 1) zones-per-level))
         (zone-depth (elt rack-level-dimensions 0))
         (zone-height 0.4))
    (labels ((highlight-zone (level zone)
               (find `(,level ,zone) highlight-zones
                     :test
                     (lambda (subject list-item)
                       (and (eql (car subject) (car list-item))
                            (eql (cadr subject) (cadr list-item))))))
             (zone->marker (level zone id)
               (let* ((dimensions `(,zone-width
                                    ,zone-depth
                                    ,zone-height))
                      (zone-alpha (cond ((highlight-zone level zone)
                                         0.6)
                                        (t 0.25)))
                      (color (cond ((evenp (+ level id))
                                    `(0.0 1.0 0.0 ,zone-alpha))
                                   (t `(0.0 1.0 1.0 ,zone-alpha))))
                      (pose (make-zone-pose level zone 0 0)))
                 (roslisp:make-message
                  "visualization_msgs/Marker"
                  (stamp header) (roslisp:ros-time)
                  (frame_id header) "/map"
                  ns "zones"
                  id id
                  type 1
                  action 0
                  (pose) (tf:pose->msg (pose->map-relative-pose pose))
                  (x scale) (elt dimensions 0)
                  (y scale) (elt dimensions 1)
                  (z scale) (elt dimensions 2)
                  (r color) (first color)
                  (g color) (second color)
                  (b color) (third color)
                  (a color) (fourth color)))))
      (let* ((markers
               (map 'vector #'identity
                    (loop for level from 0 below levels
                          append
                          (loop for zone from 0 below zones-per-level
                                collect
                                (zone->marker
                                 level zone
                                 (+ (* level zones-per-level)
                                    zone))))))
             (markers-message
               (roslisp:make-message "visualization_msgs/MarkerArray"
                                     markers markers))
             (pub (roslisp:advertise
                   "/planner_objects"
                   "visualization_msgs/MarkerArray")))
        (roslisp:publish pub markers-message)))))

(defun display-objects (objects &key highlighted-objects)
  (labels ((object->marker-message (object id)
             (let ((dimensions (get-item-dimensions-cached object))
                   (color (cond ((find object highlighted-objects
                                       :test #'string=)
                                 `(0.0 1.0 0.0 1.0))
                                (t `(1.0 1.0 0.0 1.0)))))
               (roslisp:make-message
                "visualization_msgs/Marker"
                (stamp header) (roslisp:ros-time)
                (frame_id header) "/map"
                ns "objects"
                id id
                type 1
                action 0
                (pose) (tf:pose->msg (pose->map-relative-pose
                                      (get-item-pose object)))
                (x scale) (elt dimensions 0)
                (y scale) (elt dimensions 1)
                (z scale) (elt dimensions 2)
                (r color) (first color)
                (g color) (second color)
                (b color) (third color)
                (a color) (fourth color)))))
    (let* ((markers
             (map 'vector #'identity
                  (loop for i from 0 below (length objects)
                        as object = (elt objects i)
                        collect
                        (progn
                          (setf (gethash object *markers*) i)
                          (object->marker-message object i)))))
           (markers-message
             (roslisp:make-message "visualization_msgs/MarkerArray"
                                   markers markers))
           (pub (roslisp:advertise
                 "/planner_objects" "visualization_msgs/MarkerArray")))
      (roslisp:publish pub markers-message))))

(defun place-object-on-rack (object level x y)
  (let ((dimensions (get-item-dimensions object)))
    (set-item-pose
     object
     (make-level-relative-pose level x y (/ (elt dimensions 2) 2)))))

(defun place-object-in-zone (object level zone x y)
  (let* ((zone-pose (make-zone-pose level zone x (- y)))
         (origin (tf:origin zone-pose)))
    (place-object-on-rack object level (tf:x origin) (tf:y origin))))

(defun assess-object-zones (objects)
  (let* ((levels 4)
         (zones-per-level 4)
         (rack-level-dimensions (get-rack-level-dimensions 1))
         (zone-width
           (/ (elt rack-level-dimensions 1)
              zones-per-level))
         (zone-depth (elt rack-level-dimensions 0)))
    (mapcar
     (lambda (object)
       (let* ((pose (pose->rack-relative-pose
                     (get-item-pose object)))
              (pose-elevation (tf:z (tf:origin pose)))
              (level
                (loop for level from 0 to (1- levels)
                      as level-pose = (pose->rack-relative-pose
                                       (make-level-relative-pose
                                        level 0 0 0))
                      as level-elevation = (tf:z (tf:origin
                                                  level-pose))
                      when (> pose-elevation level-elevation)
                        maximize level))
              (zone (find
                     pose (loop for zone from 0 below 4 collect zone)
                     :test
                     (lambda (pose zone)
                       (let* ((zone-pose (pose->rack-relative-pose
                                          (make-zone-pose
                                           0 zone 0 0)))
                              (zone-x (tf:x (tf:origin zone-pose)))
                              (zone-y (tf:y (tf:origin zone-pose)))
                              (pose-x (tf:x (tf:origin pose)))
                              (pose-y (tf:y (tf:origin pose))))
                         (and (> pose-x
                                 (- zone-x (/ zone-width 2)))
                              (< pose-x
                                 (+ zone-x (/ zone-width 2)))
                              (> pose-y
                                 (- zone-y (/ zone-depth 2)))
                              (< pose-y
                                 (+ zone-y (/ zone-depth 2)))))))))
         `(,level ,zone)))
     objects)))

(defun toy-problem-state ()
  (display-zones)
  (let ((item-1 (add-shopping-item "Kelloggs"))
        (item-2 (add-shopping-item "Kelloggs")))
    (place-object-in-zone item-1 0 3 0.0 0.0)
    (place-object-in-zone item-2 2 2 0.0 0.0)
    (display-objects `(,item-1 ,item-2))
    (let ((object-zones (assess-object-zones `(,item-1 ,item-2))))
      (display-zones :highlight-zones object-zones))))

(defun detected-type (object)
  (let ((found (find "JIRAnnotatorObject"
                     (desig-prop-values
                      object 'desig-props::detection)
                     :test (lambda (subject detail)
                             (string= (cadr (assoc 'desig-props::source
                                                   detail))
                                      subject)))))
    (when found
      (convert-object-name
       (cadr (assoc 'desig-props::type found))))))

(defun convert-object-name (name)
  (let ((new-name
          (cond
            ((string= name "pancake-mix") "PancakeMix")
            ((string= name "can") "Corn")
            ((string= name "tomato-sauce") "TomatoSauce")
            ((string= name "jodsalz-salt-container") "SaltDispenser")
            ((string= name "lion-cereals") "Lion")
            ((string= name "cornflakes") "Kelloggs"))))
    (cond (new-name new-name)
          (t "SaltDispenser")))) ;; Default model

(defun get-current-state ()
  (let ((objects (get-shopping-items)))
    (mapcar (lambda (object)
              `(,(first (assess-object-zones `(,object)))
                ,object))
            objects)))

(defun toy-problem-solve ()
  (remove-all-shopping-items)
  (let ((perceived-objects
          (or *perceived-objects*
              (setf *perceived-objects*
                    (top-level
                      (with-process-modules
                        (robosherlock-pm::perceive-object-designator
                         (make-designator 'object nil))))))))
    (format t "Found ~a object(s)~%" (length perceived-objects))
    (let ((all-objects
            (mapcar (lambda (object)
                      (let ((pose (desig-prop-value
                                   (desig-prop-value
                                    object 'desig-props:at)
                                   'desig-props:pose))
                            (item (add-shopping-item
                                   (or (detected-type object)
                                       "Kelloggs"))))
                        (set-item-pose item pose)
                        item))
                    perceived-objects)))
      (let ((object-zones (assess-object-zones all-objects)))
        (display-zones :highlight-zones object-zones))
      (let ((current-state (get-current-state)))
        (modified-a-star
         current-state
         (make-target-state current-state))))))

(defun make-target-state (start-state)
  (let ((index 0))
    (loop for level from 0 below 4
          append
          (loop for zone from 0 below 4
                as it = (prog1
                            (when (< index (length start-state))
                              `((,level ,zone)
                                ,(second (elt start-state index))))
                          (incf index))
                when it
                  collect it))))

(defun make-planning-state (robot-pose arrangement)
  `((:robot-pose ,robot-pose)
    ,@arrangement))

(defun make-transitions (state)
  )

(defun transition-valid? (state transition)
  )

(defun modified-a-star (start-state target-state)
  (let ((closed-set nil)
        (open-set `(,start-state)))
    open-set))
