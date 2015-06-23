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

(defvar *action-client-torso* nil)
(defvar *shopping-item-urdfs* (make-hash-table :test 'equal))

(defparameter *rack-level-positions-row* 4)
(defparameter *rack-level-positions-depth* 3)

;;;
;;; Infrastructure Utilities
;;;

(defun start-scenario-external ()
  (roslisp:ros-info (shopping) "Connecting to ROS")
  (roslisp-utilities:startup-ros)
  (roslisp:ros-info (shopping) "Initializing Environment")
  (prepare-settings)
  (roslisp:ros-info (shopping) "This starts the scenario for autonomous operation once everyting is in place."))

(defun load-shopping-item-urdf (shopping-item)
  (let ((urdf-path (get-item-urdf-path shopping-item)))
    (unless (gethash urdf-path *shopping-item-urdfs*)
      (setf (gethash urdf-path *shopping-item-urdfs*)
            (cl-urdf:parse-urdf (pathname urdf-path))))
    (gethash urdf-path *shopping-item-urdfs*)))

(defun get-hint (hints hint-symbol &optional default)
  (let ((result (cadr (find hint-symbol hints
                            :test (lambda (x y) (eql x (car y)))))))
    (cond (result result)
          (t (when default default)))))

(defun update-hints (hints updates)
  (let ((hints-without-updated
          (remove-if (lambda (hint)
                       (get-hint updates (car hint)))
                     hints)))
    (append hints-without-updated updates)))

(defmacro with-process-modules (&body body)
  "Implicitly runs process modules necessary for operating the PR2 robot. The started (and after finishing the body code also automatically evaporated) process modules are:

 - pr2-manipulation-process-module
 - pr2-navigation-process-module
 - point-head-process-module
 - robosherlock-process-module"
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        robosherlock-process-module:robosherlock-process-module)
     ,@body))

(defmacro with-simulation-process-modules (&body body)
  "Implicitly runs process modules necessary for operating the PR2 robot in a Gazebo simulation. The started (and after finishing the body code also automatically evaporated) process modules are:

 - pr2-manipulation-process-module
 - pr2-navigation-process-module
 - point-head-process-module
 - gazebo-perception-process-module"
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        gazebo-perception-process-module:gazebo-perception-process-module)
     ,@body))

(defun get-robot-pose (&optional (frame-id "/base_link"))
  (cl-tf2:ensure-pose-stamped-transformed
   *tf2*
   (tf:make-pose-stamped
    frame-id
    0.0
    (tf:make-identity-vector)
    (tf:make-identity-rotation))
   "/map" :use-current-ros-time t))

(defun move-arms-up (&key allowed-collision-objects side ignore-collisions)
  (when (or (eql side :left) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :left
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 0.5 1.3)
      (tf:euler->quaternion :ax 0));pi))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects))
  (when (or (eql side :right) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :right
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 -0.5 1.3)
      (tf:euler->quaternion :ax 0))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects)))

(defun init-belief-state ()
  (let* ((urdf-robot
           (cl-urdf:parse-urdf
            (roslisp:get-param "robot_description_lowres")))
         (urdf-rack
           (cl-urdf:parse-urdf
            (roslisp:get-param "shopping_rack_description")))
         (urdf-area
           (cl-urdf:parse-urdf
            (roslisp:get-param "shopping_area_description")))
         (rack-rot-quaternion (tf:euler->quaternion :az -1.57))
         (rack-rot `(,(tf:x rack-rot-quaternion)
                     ,(tf:y rack-rot-quaternion)
                     ,(tf:z rack-rot-quaternion)
                     ,(tf:w rack-rot-quaternion)))
         (rack-trans `(1.275 0.214 0))
         (area-rot-quaternion (tf:euler->quaternion :az -1.57))
         (area-rot `(,(tf:x area-rot-quaternion)
                     ,(tf:y area-rot-quaternion)
                     ,(tf:z area-rot-quaternion)
                     ,(tf:w area-rot-quaternion)))
         (area-trans `(2.720 0.295 0)))
    (force-ll
     (crs:prolog
      `(and (btr:clear-bullet-world)
            (btr:bullet-world ?w)
            (btr:assert (btr:object
                         ?w btr:static-plane floor
                         ((0 0 0) (0 0 0 1))
                         :normal (0 0 1) :constant 0))
            (btr:debug-window ?w)
            (btr:robot ?robot)
            (assert (btr:object
                     ?w btr:urdf ?robot ,(get-robot-pose)
                     :urdf ,urdf-robot))
            (assert (btr:object
                     ?w btr:semantic-map sem-map-rack
                     (,rack-trans ,rack-rot)
                     :urdf ,urdf-rack))
            (assert (btr:object
                     ?w btr:semantic-map sem-map-area
                     (,area-trans ,area-rot)
                     :urdf ,urdf-area)))))))

(defun init-btr-perception ()
  (cram-environment-representation:ignore-bullet-object-during-perception "floor")
  (cram-environment-representation:ignore-bullet-object-during-perception "ground_plane")
  (cram-environment-representation:ignore-bullet-object-during-perception 'cram-pr2-knowledge::pr2)
  (cram-environment-representation:ignore-bullet-object-during-perception (first (get-racks)))
  (cram-environment-representation:ignore-bullet-object-during-perception 'sem-map-rack)
  (cram-environment-representation:ignore-bullet-object-during-perception 'sem-map-area))

(defun prepare-settings ()
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::check-ik-solution)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::validate-designator-solution)
  (init-belief-state)
  (init-btr-perception)
  (moveit:clear-collision-environment)
  (sem-map-coll-env:publish-semantic-map-collision-objects))

(defun move-torso (&optional (position 0.3))
  (let* ((action-client (or *action-client-torso*
                            (actionlib:make-action-client
                             "/torso_controller/position_joint_action"
                             "pr2_controllers_msgs/SingleJointPositionAction")))
         (goal (actionlib:make-action-goal
                   action-client
                 position position)))
    (actionlib:wait-for-server action-client)
    (setf *action-client-torso* action-client)
    (actionlib:send-goal-and-wait action-client goal)))

(defun move-arms-away ()
  "Moves the left, and then the right robot arm up into a safe pose."
  (move-arms-up :side :left)
  (move-arms-up :side :right))

(defun make-target-arrangement (&key hints)
  )

(defun pick-object (object &key stationary)
  (cond (stationary
         (achieve `(cram-plan-library:object-picked ,object)))
        (t
         (achieve `(cram-plan-library:object-in-hand ,object)))))

(defun perceive-a (object &key stationary (move-head t))
  (cpl:with-failure-handling
      ((cram-plan-failures:object-not-found (f)
         (declare (ignore f))
         (ros-warn (longterm) "Object not found. Retrying.")
         (cpl:retry)))
    (cond (stationary
           (let ((at (desig-prop-value object 'desig-props:at)))
             (when move-head
               (achieve `(cram-plan-library:looking-at ,(reference at))))
             (first (perceive-object
                     'cram-plan-library:currently-visible
                     object))))
          (t (cpl:with-failure-handling
                 ((cram-plan-failures:location-not-reached-failure (f)
                    (declare (ignore f))
                    (cpl:retry)))
               (perceive-object 'cram-plan-library:a object))))))

(defun perceive-all (object &key stationary (move-head t))
  (cpl:with-failure-handling
      ((cram-plan-failures:object-not-found (f)
         (declare (ignore f))
         (ros-warn (longterm) "Object not found. Retrying.")
         (cpl:retry)))
    (cond (stationary
           (let ((at (desig-prop-value object 'desig-props:at)))
             (when move-head
               (achieve `(cram-plan-library:looking-at ,(reference at))))
             (first (perceive-object
                     'cram-plan-library:currently-visible
                     object))))
          (t (cpl:with-failure-handling
                 ((cram-plan-failures:location-not-reached-failure (f)
                    (declare (ignore f))
                    (cpl:retry)))
               (perceive-object 'cram-plan-library:all object))))))

(defun make-handles (distance-from-center
                     &key
                       (segments 1)
                       (ax 0.0) (ay 0.0) (az 0.0)
                       (offset-angle 0.0)
                       grasp-type
                       (center-offset
                        (tf:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (/ i segments)))
                              offset-angle)
        as handle-pose = (tf:make-pose
                          (tf:make-3d-vector
                           (+ (* distance-from-center (cos current-angle))
                              (tf:x center-offset))
                           (+ (* distance-from-center (sin current-angle))
                              (tf:y center-offset))
                           (+ 0.0
                              (tf:z center-offset)))
                          (tf:euler->quaternion
                           :ax ax :ay ay :az (+ az current-angle)))
        as handle-object = (make-designator
                            'cram-designators:object
                            (append
                             `((desig-props:type desig-props:handle)
                               (desig-props:at
                                ,(a location `((desig-props:pose
                                                ,handle-pose)))))
                             (when grasp-type
                               `((desig-props:grasp-type ,grasp-type)))))
        collect handle-object))

(defun spawn-model-on-rack-level-index (rack level model urdf x y elevation rotation)
  (spawn-model-on-rack-level
   (get-rack-on-level rack level) model urdf x y elevation rotation))

(defun spawn-model-on-rack-level (racklevel model urdf x y elevation rotation)
  "Spawns the URDF model `urdf' (path) of the object `model' (model name) on the named rack level `racklevel',  at lower-left relative coordinates `x', `y'. `rotation' is the object's rotation, and defaults to a zero-rotation."
  ;; This elevation helps not spawning objects inside of each
  ;; other. Like the shopping item inside of the rack level.
  (let* ((elevation (+ elevation 0.01))
         (pose (get-rack-level-relative-pose
                racklevel x y elevation rotation)))
    (cram-gazebo-utilities:spawn-gazebo-model model pose urdf)))

(defun spawn-shopping-item (item level x y &optional (rotation (tf:euler->quaternion)))
  "Spawns the object `item' on the indexed rack level `level' at lower-left relative coordinates `x', `y'. Optionally, the object's `rotation' can be set. This defaults to a zero-rotation."
  (let ((urdf (get-item-urdf-path item)))
    (spawn-model-on-rack-level-index
     (first (get-racks)) level
     item urdf x y 0 rotation)))

(defun spawn-shopping-item-on-named-level (item level x y elevation &optional (rotation (tf:euler->quaternion)))
  "Spawns the object `item' on the named rack level `level' at lower-left relative coordinates `x', `y'. Optionally, the object's `rotation' can be set. This defaults to a zero-rotation."
  (let ((urdf (get-item-urdf-path item)))
    (spawn-model-on-rack-level level item urdf x y elevation rotation)))

(defun make-empty-object-arrangement ()
  "Creates an empty arrangement map for mapping objects to their positions on a rack."
  (let* ((rack (first (get-racks)))
         (rack-levels (get-rack-levels rack)))
    (make-array `(,(length rack-levels)
                  ,*rack-level-positions-row*
                  ,*rack-level-positions-depth*)
                :initial-element nil)))

(defun make-random-object-arrangement (&key hints)
  "Randomly places all known shopping items in a (m x n) grid such that it can be used to spawn the respective objects on an actual rack."
  ;; TODO(winkler): Add support for the hints system
  (declare (ignore hints))
  (let* ((arrangement (make-empty-object-arrangement))
         (dimensions (array-dimensions arrangement))
         (items (get-shopping-items)))
    (loop while items
          as item = (first items)
          as stackable = (is-stackable item)
          as i = (random (first dimensions))
          as j = (random (second dimensions))
          as k = (random (third dimensions))
          do (cond ((eql (aref arrangement i j k) nil)
                    (setf (aref arrangement i j k) `(,item))
                    (setf items (remove item items)))
                   (t (let ((stored-items (aref arrangement i j k)))
                        (when (and (< (length stored-items)
                                      (1- *rack-level-positions-depth*))
                                   stackable
                                   (string= (get-item-class
                                             (first stored-items))
                                            (get-item-class item)))
                          (push item (aref arrangement i j k))
                          (setf items (remove item items)))))))
    arrangement))

(defun resolve-object-arrangement (arrangement)
  "Using a template object arrangement grid (see `make-arrangement-position-map'), the random object arrangement `arrangement' is mapped to specific coordinates on the current rack."
  (let ((dimensions (array-dimensions arrangement))
        (positions (make-arrangement-position-map)))
    (loop for i from 0 below (first dimensions)
          append
          (loop for j from 0 below (second dimensions)
                append
                (loop for k from 0 below (third dimensions)
                      append
                      (loop for item in (aref arrangement i j k)
                            collect
                            (destructuring-bind
                                (rack-level . (offset-h offset-d))
                                (aref positions i j k)
                              `(,item ,rack-level
                                      ,offset-h ,offset-d))))))))

(defun make-arrangement-position-map ()
  "Creates a grid template (m x n matrix) that describes the layout of a rack, mentioning which position on the grid reflects which rack level (using its unique identifier), and what its offset from the left lower corner would be."
  (let* ((rack (first (get-racks)))
         (map (make-empty-object-arrangement))
         (dimensions (array-dimensions map))
         (items-per-row (second dimensions))
         (items-depth (third dimensions)))
    (loop for i from 0 below (first dimensions)
          as rack-level = (get-rack-on-level rack i)
          as rack-width = (elt (get-item-dimensions rack-level) 1)
          as item-space = (/ rack-width items-per-row)
          do (loop for j from 0 below items-per-row
                   as offset-h = (- (+ (* j item-space)
                                       (* item-space 0.5))
                                    (/ rack-width 2))
                   do (loop for k from 0 below items-depth
                            as offset-d = (+ (* k 0.1) -0.15)
                            do (setf (aref map i j k)
                                     (cons rack-level `(,offset-h
                                                        ,offset-d))))))
    map))

(defun resolve-and-spawn-object-arrangement (arrangement)
  (let ((resolved-arrangement (resolve-object-arrangement arrangement)))
    (dolist (object-position resolved-arrangement)
      (destructuring-bind (items racklevel y-offset x-offset)
          object-position
        (let ((combined-heights
                (loop for i from 0 below (length items)
                      as item = (elt items i)
                      collect
                      (loop for j from 0 below i
                            summing
                            (third (get-item-dimensions item))))))
          (loop for i from 0 below (length items)
                as item = (elt items i)
                do (spawn-shopping-item-on-named-level
                    item racklevel x-offset y-offset
                    (elt combined-heights i)
                    (tf:euler->quaternion :az (/ pi 2)))))))))

(defun spawn-random-object-arrangement (&key hints)
  "Creates a random object arrangement on a shopping rack (considering all known rack levels) and randomly places all known objects on it. Using this arrangement, the URDF models of the objects will be spawned into the current Gazebo scene."
  (resolve-and-spawn-object-arrangement
   (make-random-object-arrangement :hints hints)))

(defun delete-shopping-items-from-gazebo ()
  "Deletes (unspawns) all known shopping items from the current Gazebo scene."
  (let ((items (get-shopping-items)))
    (dolist (item items)
      (cram-gazebo-utilities::delete-gazebo-model item))))

(defun prepare-simulated-scene (&key hints)
  "First deletes all shopping items from the Gazebo scene, and then populates the scenen with a random object arrangement."
  (delete-shopping-items-from-gazebo)
  (let ((hints
          (or (when (eql (get-hint hints :simulation-type) :simple)
                (update-hints
                 hints `((:items-scene-amount 1)
                         (:items-scene-classes "Kelloggs"))))
              hints)))
    (populate-item-database :hints hints)
    (spawn-random-object-arrangement :hints hints)))

(defmacro try-all-objects ((var list) &body body)
  "Tries all objects in `list', binding each of the to the variable `var'. In that environment, `body' is executed. Failures of type `object-not-found', `manipulation-pose-unreachable', `manipulation-failure', and `location-not-reached-failure' are caught implicitly, and the next object is tried."
  `(block try-all-objects
     (dolist (,var ,list)
       (block try-all-objects-safety
         (cpl:with-failure-handling
             (((or cram-plan-failures:object-not-found
                   cram-plan-failures:manipulation-pose-unreachable
                   cram-plan-failures:manipulation-failure
                   cram-plan-failures:location-not-reached-failure) (f)
                (declare (ignore f))
                (return-from try-all-objects-safety)))
           (let ((result (progn ,@body)))
             (return-from try-all-objects result)))))))

(defun get-shopping-objects (&key class-type)
  "Constructs object designators from shopping items known to the underlying knowledge base. Each item will be equipped with a name, semantic handles, and the object shape (all acquired from the knowledge base). Returns a list of object designators."
  (let ((shopping-items (or (and class-type (get-items-by-class-type class-type))
                            (get-shopping-items))))
    (mapcar
     (lambda (item)
       (let ((handles (get-item-semantic-handles item))
             (shape (intern
                     (string-upcase
                      (or
                       (get-item-primitive-shape item)
                       "box"))
                     'desig-props)))
         (make-designator
          'object `((desig-props:name ,item)
                    (desig-props:type ,(get-item-class item))
                    ,@(mapcar
                       (lambda (handle)
                         `(desig-props:handle ,handle))
                       handles)
                    (desig-props:shape ,shape)))))
     shopping-items)))

(defun go-to-pose (pose)
  (with-designators ((goal-location (location `((pose ,pose))))
                     (navigate (action `((type navigation)
                                         (goal ,goal-location)))))
    (perform navigate)))

(defun go-in-front-of-rack (rack)
  (let* ((rack-pose (get-rack-pose rack))
         (pose-in-front-of
           ;; NOTE(winkler): Hacky, until I find out why the other one
           ;; doesn't work. The robot seems to drive somewhere beyond
           ;; the set pose, far to the back of the scene.
           (tf:make-pose-stamped
            "map" 0.0
            (tf:make-3d-vector 0 0 0)
            (tf:euler->quaternion))))
           ;; (tf:pose->pose-stamped
           ;;  (tf:frame-id rack-pose) (tf:stamp rack-pose)
           ;;  (cl-transforms:transform-pose
           ;;   (tf:make-transform
           ;;    (tf:make-3d-vector -2.0 0.0 0)
           ;;    (tf:euler->quaternion))
           ;;   (cl-transforms:transform-pose
           ;;    (tf:make-transform
           ;;     (tf:make-3d-vector 0.0 0.0 0.0)
           ;;     (tf:euler->quaternion :az (/ pi 2)))
           ;;    rack-pose)))))
    (go-to-pose pose-in-front-of)))

(defun rotmat->pose (rotmat)
  (cl-transforms:transform->pose
   (cl-transforms:matrix->transform
    (make-array
     '(4 4) :displaced-to (make-array
                           16 :initial-contents rotmat)))))

(defun reposition-torso-for-rack-level (rack level)
  (let* ((level (get-rack-on-level rack level))
         (elevation (get-rack-level-elevation level)))
    (move-torso (/ elevation 5.0))))

(defun json-prolog-present ()
  (roslisp:wait-for-service "json_prolog/query" 0.25))

(defun check-system-settings (&key hints)
  (when (cond ((eql (roslisp:node-status) :shutdown)
               (roslisp:ros-error
                (check) "The ROS node is not connected.")
               nil)
              (t t))
    (when (case (get-hint hints :world)
            (:reality
             (cond ((cram-uima:uima-present)
                    t)
                   (t (roslisp:ros-error
                       (check) "RoboSherlock service not present.")
                      nil)))
            (:simulation
             (cond ((cram-gazebo-utilities:gazebo-present)
                    t)
                   (t (roslisp:ros-error
                       (check) "Gazebo spawn service not present.")
                      nil))))
      (when (cond ((roslisp:has-param "/robot_description_lowres")
                   t)
                  (t (roslisp:ros-error
                      (check) "Low-Resolution robot model not present on the parameter server.")
                     nil))
        (cond ((json-prolog-present)
               t)
              (t (roslisp:ros-error
                  (check) "JSON-Prolog query service not present.")
                 nil))))))

(defun populate-item-database (&key hints)
  "Populates the item database with a new distribution of item instances, removing all formerly known instances of shopping items. The hints system here understands the two keywords `:items-scene-amount', denoting an integer value of how many items should be inserted into the knowledge base, and `:items-scene-classes', describing which item classes may be used for population. The former defaults to 8 items, the latter to all known shopping item classes."
  (remove-all-shopping-items)
  (let* ((amount (get-hint hints :items-scene-amount 8))
         (valid-classes (get-hint hints :items-scene-classes
                                  (shopping-item-classes))))
    (loop for i from 0 below amount
          as class-idx = (random (length valid-classes))
          as class = (elt valid-classes class-idx)
          do (add-shopping-item class))))
