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

(defun run-simulated (&key hints)
  "Shortcut for running the rack arrangement scenario in simulation."
  (run-rack-arrangement-protected
   :hints (update-hints hints `((:world :simulation)))))

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
         (rack-arrangement :hints hints)))
      (:reality
       (with-process-modules
         (rack-arrangement :hints hints))))))

(def-cram-function rack-arrangement (&key hints)
  "Performs a rack-tidying up scenario by controlling a PR2 robot that rearranges objects, based on a given target arrangement."
  (let ((rack (first (get-racks))))
    (move-torso)
    (move-arms-away)
    ;; First, perceive scene
    (achieve `(rack-scene-perceived ,rack))
    (let ((objects (get-shopping-objects)))
      (dolist (object objects)
        (let ((detected-objects
                (achieve `(objects-detected-in-rack ,rack ,object))))
          (unless detected-objects
            (cpl:fail 'cram-plan-failures:object-not-found))
          (try-all-objects (detected-object detected-objects)
            (achieve `(object-picked-from-rack ,rack ,detected-object))
            (equate object detected-object))
          (try-forever
            (let ((level (get-rack-on-level rack 2))
                  (x -0.15)
                  (y 0.0))
              (achieve `(object-placed-on-rack
                         ,object ,level ,x ,y)))))))))
