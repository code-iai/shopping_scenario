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
  (let* ((string-symbol (write-to-string symbol)))
    (subseq string-symbol 2 (- (length string-symbol) 2))))

(defun get-shopping-items ()
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (with-vars-bound (?item) bdgs
        (json-symbol->string ?item)))
    (json-prolog:prolog `("shopping_item" ?item)))))

(defun is-stackable (item)
  (not (not (json-prolog:prolog
             `("is_stackable" ,item)))))

(defun get-racks ()
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (with-vars-bound (?rack) bdgs
        (json-symbol->string ?rack)))
    (json-prolog:prolog `("rack" ?rack)))))

(defun get-rack-levels (rack)
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (with-vars-bound (?racklevel) bdgs
        (json-symbol->string ?racklevel)))
    (json-prolog:prolog `("rack_level" ,rack ?racklevel)))))
