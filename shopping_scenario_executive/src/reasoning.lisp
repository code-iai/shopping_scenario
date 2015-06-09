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


(defmacro with-first-prolog-vars-bound (vars prolog-query &body body)
  "Evaluates the prolog query `prolog-query' and transforms variables `vars' via `body', returning the result."
  `(with-vars-bound ,vars
       (lazy-car
        (json-prolog:prolog ,prolog-query))
     ,@body))

(defmacro with-prolog-vars-bound (vars prolog-query &body body)
  "Lists all results from the prolog query `prolog-query', each being transformed by `body'. `vars' denotes all variables to make available in `body'."
  `(force-ll
    (lazy-mapcar
     (lambda (bdgs)
       (with-vars-bound ,vars bdgs
         ,@body))
     (json-prolog:prolog ,prolog-query))))

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
  (with-prolog-vars-bound (?item)
      `("shopping_item" ?item)
    (split-prolog-symbol (json-symbol->string ?item))))

(defun is-stackable (item)
  "Returns whether the shopping item `item' is stackable or not."
  (not (not (json-prolog:prolog
             `("is_stackable" ,(add-prolog-namespace item))))))

(defun get-racks ()
  "Returns all racks known in the current semantic environment."
  (with-prolog-vars-bound (?rack)
      `("rack" ?rack)
    (split-prolog-symbol (json-symbol->string ?rack))))

(defun get-rack-levels (rack)
  "Returns all rack levels for the given rack `rack'."
  (with-prolog-vars-bound (?racklevel)
      `("rack_level" ,(add-prolog-namespace rack) ?racklevel)
    (split-prolog-symbol (json-symbol->string ?racklevel))))

(defun get-rack-on-level (rack level)
  "Returns the rack level `level' on rack `rack'. `level' is an integer."
  (with-first-prolog-vars-bound (?racklevel)
      `("rack_on_level" ,(add-prolog-namespace rack) ,level ?racklevel)
    (split-prolog-symbol (json-symbol->string ?racklevel))))

(defun location-on-rack-level (rack level)
  "Generates a location designator that describes a three dimensional pose on the two dimensional plane of the given rack level `level' on rack `rack'. `level' is an integer."
  (let ((rack-level (get-rack-on-level rack level)))
    (make-designator 'location
                     `((desig-props::on "RackLevel")
                       (desig-props::name ,(add-prolog-namespace rack-level))))))

(defun get-object-rack-level (rack object)
  "Returns which level of a rack `rack' an object `object' resides on. Returns the (namespace-less) OWL identifier of the rack level."
  (let* ((at (desig-prop-value object 'desig-props::at))
         (pose (reference at))
         (origin (tf:origin pose)))
    (with-first-prolog-vars-bound (?racklevel)
        `("position_on_rack"
          ,(tf:x origin) ,(tf:y origin) ,(tf:z origin)
          0.3 ,(add-prolog-namespace rack) ?racklevel)
      (split-prolog-symbol (json-symbol->string ?racklevel)))))

(defun get-rack-level-elevation (racklevel)
  "Returns the z-coordinate of the surface of the rack level `racklevel'."
  (with-first-prolog-vars-bound (?elevation)
      `("rack_level_elevation" ,(add-prolog-namespace racklevel) ?elevation)
    ?elevation))

(defun get-rack-level-relative-pose (racklevel x y z rotation)
  "Returns (in absolute map coordinates) a pose stamped that describes the relative pose ((x y z) rotation) on the rack level `racklevel'."
  (with-first-prolog-vars-bound (?result)
      `("rack_level_relative_position"
        ,(add-prolog-namespace racklevel) ,x ,y ?result)
    (destructuring-bind (x y elevation) ?result
      (tf:make-pose-stamped
       "map" 0.0 (tf:make-3d-vector x y (+ z elevation)) rotation))))

(defun get-item-urdf-path (item)
  "Returns the absolute URDF file path for an item `item' (if set in the semantic information supplied to KnowRob)."
  (with-first-prolog-vars-bound (?urdfpath)
      `("item_urdf_path" ,(add-prolog-namespace item)
                         ?urdfpath)
    (json-symbol->string ?urdfpath)))

(defun get-item-dimensions (item)
  "Returns the dimensions (with, depth, height) of an item `item'."
  (with-first-prolog-vars-bound (?width ?depth ?height)
      `("object_dimensions_restricted"
        ,(add-prolog-namespace item)
        ?width ?depth ?height)
    (vector ?width ?depth ?height)))

(defun get-items-by-class-type (class-type)
  "Returns all item instances that are of class type `class-type'."
  (with-prolog-vars-bound (?item)
      `("item_class_type" ,(add-prolog-namespace class-type)
                          ?item)
    (split-prolog-symbol (json-symbol->string ?item))))
