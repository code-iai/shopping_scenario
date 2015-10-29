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

;;;
;;; Hints Documentation
;;;

;; The hints system allows customization of the scenario before
;; running it, without actually touching any plan code or changing the
;; knowledge base. This is the second-level, quasi static task
;; parameter knowledge. All hint keys are precedet with a colon (`:')
;; and are therefore Lisp keyword symbols.
;; 
;; Available hints (and their expected values) are:
;; 
;;  * `:world':
;;    - `:simulation': Runs the scenario in a Gazebo
;;      simulation. Prepares the simulated scene, and spawns
;;      appropriate object scenes.
;;    - `:reality': Runs the scenatio on a real PR2 robot.
;;
;;  * `:simulation-type':
;;    - `:simple': Only valid in `:simulation' worlds. Triggers
;;      spawning only one `Kelloggs' object in the third rack level,
;;      easily reachable for the PR2.
;;    - `:default': Spawns random object arrangements all over the
;;      whole rack. This is the default setting.
;;
;;  * `:items-scene-amount':
;;    - Number of objects to spawn in the simulated world (and to
;;      assert into the knowledge base). Defaults to 8.
;;
;;  * `:items-scene-classes':
;;    - Which classes to use for spawning random objects in the
;;      simulated world (and to assert into the knowledge
;;      base). Defaults to all known shopping item classes.


;;;
;;; Shortcuts
;;;

(defun start-scenario-external ()
  (roslisp:ros-info (shopping) "Connecting to ROS")
  (roslisp-utilities:startup-ros)
  (roslisp:ros-info (shopping) "Running Shopping Scenario (simulated, simplified)")
  (run-simulated-simple))

(defun run-simulated (&key hints)
  "Shortcut for running the rack arrangement scenario in simulation."
  (run-rack-arrangement-protected
   :hints (update-hints hints `((:world :simulation)
                                (:perceive-scene-rack-level 2)))))

(defun run-simulated-simple (&key hints)
  "Shortcut for running the rack arrangement scenario in simulation in a simplified version."
  (run-simulated :hints (update-hints
                         hints
                         `((:items-scene-classes ("Lion" "Kelloggs"))
                           (:items-scene-amount 4)
                           (:allowed-rack-levels (1 2))))))

(defun run-simulated-problem (&key hints)
  (run-simulated :hints (update-hints hints `((:version :problem)))))

(defun run-reality (&key hints)
  "Shortcut for running the rack arrangement scenario in reality."
  (run-rack-arrangement-protected
   :hints (update-hints hints `((:world :reality)))))

(defun run-rack-arrangement-protected (&key hints)
  "Runs the rack arrangement scenario in an environment protected by `check-system-settings'."
  (when (check-system-settings :hints hints)
    (run-rack-arrangement :hints hints)))


;;;
;;; Top-Level Plans
;;;

(def-top-level-cram-function run-rack-arrangement (&key hints)
  "Main scenario entry point to start arranging objects. The `hints' (if defined) are forwarded to the target arrangement sampler."
  (prepare-settings)
  (let ((world (get-hint hints :world :reality)))
    (ecase world
      (:simulation
       (with-simulation-process-modules
         (prepare-simulated-scene :hints hints)
         (case (get-hint hints :version :normal)
           (:normal (rack-arrangement :hints hints))
           (:handover (rack-arrangement-handover :hints hints))
           (:problem (solve-arrangement-problem :hints hints)))))
      (:reality
       (with-process-modules
         (rack-arrangement :hints hints))))))

(def-cram-function rack-arrangement (&key hints)
  "Performs a rack-tidying up scenario by controlling a PR2 robot that rearranges objects, based on a given target arrangement."
  (let ((rack (first (get-racks))))
    (move-torso)
    (move-arms-away)
    (achieve `(rack-scene-perceived ,rack ,hints))
    (loop for i from 0 to 2 do
      (let* ((objects (get-shopping-objects)))
        (dolist (object objects)
          (achieve `(objects-detected-in-rack ,rack ,object)))
        (dolist (object objects)
          (let ((detected-objects
                  (achieve `(objects-detected-in-rack ,rack ,object))))
            (unless detected-objects
              (cpl:fail 'cram-plan-failures:object-not-found))
            (try-all-objects (detected-object detected-objects)
              (when (desig-prop-value detected-object 'handle)
                (achieve `(object-picked-from-rack ,rack ,detected-object))
                (unless (desig:desig-equal object detected-object)
                  (equate object detected-object))
                (try-forever
                  (multiple-value-bind (rack-level x y)
                      (get-free-position-on-rack rack :hints hints)
                    (let ((elevation (get-rack-level-elevation
                                      (get-rack-on-level rack rack-level))))
                      (move-torso (/ elevation 5.0))
                      (achieve `(object-placed-on-rack
                                 ,object ,(get-rack-on-level rack rack-level)
                                 ,x ,y)))))))))))))

(def-cram-function rack-arrangement-handover (&key hints)
  "Performs a rack-tidying up scenario by controlling a PR2 robot that rearranges objects, based on a given target arrangement."
  (let ((rack (first (get-racks))))
    (move-torso)
    (move-arms-away)
    (achieve `(rack-scene-perceived ,rack ,hints))
      (let* ((objects (get-shopping-objects)))
        (dolist (object objects)
          (achieve `(objects-detected-in-rack ,rack ,object)))
        (let ((object (first objects)))
          (let ((detected-objects
                  (achieve `(objects-detected-in-rack ,rack ,object))))
            (unless detected-objects
              (cpl:fail 'cram-plan-failures:object-not-found))
            (try-all-objects (detected-object detected-objects)
              (when (desig-prop-value detected-object 'handle)
                (achieve `(object-picked-from-rack ,rack ,detected-object))
                (go-in-front-of-rack rack)
                (achieve `(switched-holding-hand ,detected-object))
                )))))))
                ;; (unless (desig:desig-equal object detected-object)
                ;;   (equate object detected-object))
                ;; (try-forever
                ;;   (multiple-value-bind (rack-level x y)
                ;;       (get-free-position-on-rack rack :hints hints)
                ;;     (let ((elevation (get-rack-level-elevation
                ;;                       (get-rack-on-level rack rack-level))))
                ;;       (move-torso (/ elevation 5.0))
                ;;       (achieve `(object-placed-on-rack
                ;;                  ,object ,(get-rack-on-level rack rack-level)
                ;;                  ,x ,y)))))))))))))

(def-cram-function solve-arrangement-problem (&key hints)
  (let* ((problem (assert-planning-problem))
         (target (first problem))
         (sequence (second problem)))
    (let ((rack (first (get-racks))))
      (move-torso)
      (move-arms-away)
      (achieve `(rack-scene-perceived ,rack ,hints))
      (let* ((objects (get-shopping-objects)))
        (dolist (object objects)
          (equate object
                  (first (achieve `(objects-detected-in-rack ,rack ,object)))))
        (labels ((relative-xy (x-index y-index)
                   (let* ((rack-level (get-rack-on-level rack y-index))
                          (rack-width (elt (get-item-dimensions rack-level) 1))
                          (item-space (/ rack-width 4))
                          (offset-h (- (+ (* (- 3 x-index) item-space)
                                          (* item-space 0.5))
                                       (/ rack-width 2))))
                     `(-0.2 ,offset-h)))
                 (pose-on-rack (x-index y-index)
                   (let* ((rack-level (get-rack-on-level rack y-index))
                          (rack-width (elt (get-item-dimensions rack-level) 1))
                          (item-space (/ rack-width 4))
                          (offset-h (- (+ (* (- 3 x-index) item-space)
                                          (* item-space 0.5))
                                       (/ rack-width 2)))
                          (rel-pose (get-rack-level-relative-pose
                                     rack-level
                                     -0.2 offset-h 0.02)))
                     rel-pose))
                 (object-at-rack-position (x-index y-index)
                   (let ((closest-object nil)
                         (smallest-distance 1000.0))
                     (let ((adv (roslisp:advertise "/hhhhhh" "geometry_msgs/PoseStamped")))
                       (loop for object in objects
                             for global-pose = (pose-on-rack x-index y-index)
                             for testing = (roslisp:publish
                                            adv (tf:pose-stamped->msg global-pose))
                             for distance = (tf:v-dist
                                             (tf:origin global-pose)
                                             (tf:origin (reference
                                                         (desig-prop-value
                                                          (current-desig object) 'at))))
                             when (< distance smallest-distance)
                               do (setf smallest-distance distance)
                                  (setf closest-object object)))
                     closest-object)))
          (loop for step in sequence do
            (let ((command (first step))
                  (detail-1 (second step))
                  (detail-2 (third step)))
              (case command
                (:move
                 (let ((x-from (second detail-1))
                       (y-from (first detail-1))
                       (x-to (second detail-2))
                       (y-to (first detail-2)))
                   (let ((obj (object-at-rack-position x-from y-from)))
                     (equate obj (first (achieve `(objects-detected-in-rack
                                                   ,rack ,obj))))
                     (roslisp:ros-info (shopping plans) "Moving from ~a/~a to ~a/~a"
                                       x-from y-from x-to y-to)
                     (achieve `(object-picked-from-rack ,rack ,(current-desig obj)))
                     (let ((elevation (get-rack-level-elevation
                                       (get-rack-on-level rack y-to)))
                           (xy (relative-xy x-to y-to)))
                       (move-torso (/ elevation 5.0))
                       (achieve `(object-placed-on-rack
                                  ,obj ,(get-rack-on-level rack y-to)
                                  ,(first xy) ,(second xy)))))))))))))))
