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
;;; Plans
;;;

(def-top-level-cram-function arrange-rack-objects (&key hints)
  "Main scenario entry point to start arranging objects. The `hints' (if defined) are forwarded to the target arrangement sampler."
  (with-process-modules
    (let ((target-arrangement (make-target-arrangement :hints hints)))
      ;; TODO(winkler): Arrange objects here.
      )))

(def-top-level-cram-function arrange-rack-objects-simulated (&key hints)
  "Main scenario entry point to start arranging objects. The `hints' (if defined) are forwarded to the target arrangement sampler."
  (with-simulation-process-modules
    (let ((target-arrangement (make-target-arrangement :hints hints)))
      ;; TODO(winkler): Arrange objects here.
      (with-designators ((rack-level (location `((desig-props::on "RackLevel")
                                                 (desig-props::name "RackLevel1_fh28hepgfq"))))
                         (obj (object `((desig-props:name "Corn_uai8735a")
                                        (desig-props:at ,rack-level)
                                        (desig-props::max-handles 1)
                                        ,@(mapcar
                                           (lambda (handle-object)
                                             `(desig-props:handle ,handle-object))
                                           (make-handles
                                            0.04
                                            :segments 2
                                            :ax (/ pi 2)
                                            :center-offset
                                            (tf:make-3d-vector 0.02 0.0 0.07)))))))
        (spawn-shopping-item "Corn_uai8735a" 2 -0.2 0.2)
        (move-torso-up)
        (move-arms-away)
        (pick-object obj)))))
