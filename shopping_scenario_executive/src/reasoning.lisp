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


(defun json-symbol->string (symbol)
  "Converts `symbol' as returned from json-prolog to a lisp-usable string by trimming `|' characters at the beginning and the end."
  (let* ((string-symbol (write-to-string symbol)))
    (subseq string-symbol 2 (- (length string-symbol) 2))))

(defun split-prolog-symbol (prolog-symbol &key (delimiter '\#))
  "Splits the namespace from the symbol of a prolog identifier symbol `prolog-symbol'. The two parts must be delimited by the delimiter `delimiter'. Returns a values list, consisting of the symbol, and the namespace."
  (let ((delimiter-position
          (position delimiter prolog-symbol :test #'string=)))
    (values
     (subseq prolog-symbol (1+ delimiter-position))
     (subseq prolog-symbol 0 delimiter-position))))

(defun add-prolog-namespace (symbol &key (namespace "http://knowrob.org/kb/ias_semantic_map.owl") (delimiter '\#))
  "Concatenates a string that consists of the given `namespace', the `delimiter', and finally the `symbol'."
  (concatenate
   'string
   namespace
   (json-symbol->string (write-to-string delimiter))
   symbol))

(defun get-shopping-items ()
  "Returns all shopping items known in the current semantic environment."
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (with-vars-bound (?item) bdgs
        (split-prolog-symbol (json-symbol->string ?item))))
    (json-prolog:prolog `("shopping_item" ?item)))))

(defun is-stackable (item)
  "Returns whether the shopping item `item' is stackable or not."
  (not (not (json-prolog:prolog
             `("is_stackable" ,(add-prolog-namespace
                                item
                                :namespace "http://knowrob.org/kb/knowrob.owl"))))))

(defun get-racks ()
  "Returns all racks known in the current semantic environment."
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (with-vars-bound (?rack) bdgs
        (split-prolog-symbol (json-symbol->string ?rack))))
    (json-prolog:prolog `("rack" ?rack)))))

(defun get-rack-levels (rack)
  "Returns all rack levels for the given rack `rack'."
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (with-vars-bound (?racklevel) bdgs
        (split-prolog-symbol (json-symbol->string ?racklevel))))
    (json-prolog:prolog `("rack_level" ,(add-prolog-namespace rack) ?racklevel)))))

(defun get-rack-on-level (rack level)
  "Returns the rack level `level' on rack `rack'. `level' is an integer."
  (with-vars-bound (?racklevel)
      (lazy-car
       (json-prolog:prolog
        `("rack_on_level" ,(add-prolog-namespace rack) ,level ?racklevel)))
    (split-prolog-symbol (json-symbol->string ?racklevel))))

(defun location-on-rack-level (rack level)
  "Generates a location designator that describes a three dimensional pose on the two dimensional plane of the given rack level `level' on rack `rack'. `level' is an integer."
  (let ((rack-level (get-rack-on-level rack level)))
    (make-designator 'location
                     `((desig-props::on "RackLevel")
                       (desig-props::name ,(add-prolog-namespace rack-level))))))

(defun get-object-rack-level (rack object)
  (let* ((at (desig-prop-value object 'desig-props::at))
         (pose (reference at)))
    (with-vars-bound (?racklevel)
        (lazy-car
         (json-prolog:prolog
          `("position_on_rack" ,(tf:x (tf:origin pose)) ,(tf:y (tf:origin pose)) ,(tf:z (tf:origin pose))
                               0.3 ,(add-prolog-namespace rack) ?racklevel)))
      (split-prolog-symbol (json-symbol->string ?racklevel)))))

(defun get-rack-level-elevation (racklevel)
  (with-vars-bound (?elevation)
      (lazy-car
       (json-prolog:prolog
        `("rack_level_elevation" ,(add-prolog-namespace racklevel) ?elevation)))
    ?elevation))

(defun get-rack-level-relative-pose (racklevel x y z rotation)
  (with-vars-bound (?result)
      (lazy-car
       (json-prolog:prolog
        `("rack_level_relative_position" ,(add-prolog-namespace racklevel) ,x ,y ?result)))
    (destructuring-bind (x y elevation) ?result
      (tf:make-pose-stamped
       "map" 0.0 (tf:make-3d-vector x y (+ z elevation)) rotation))))

(defun get-item-urdf-path (item)
  (with-vars-bound (?urdfpath)
      (lazy-car
       (json-prolog:prolog
        `("item_urdf_path" ,(add-prolog-namespace
                             item
                             :namespace "http://knowrob.org/kb/knowrob.owl") ?urdfpath)))
    (json-symbol->string ?urdfpath)))
