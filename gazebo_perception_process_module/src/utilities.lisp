;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :gazebo-perception-pm)

(defclass camera ()
  ((name :initform nil :initarg :name :reader :name)
   (tf-frame :initform nil :initarg :tf-frame :reader :tf-frame)
   (fixed-pose :initform nil :initarg :fixed-pose :reader :fixed-pose)
   (fov :initform nil :initarg :fov :reader :fov)
   (distance-limit :initform nil :initarg :distance-limit :reader :distance-limit)))

(defvar *active-camera* nil)
(defvar *ignored-objects* nil)

(defun set-camera (camera)
  (setf *active-camera* camera)
  (cond (camera
         (roslisp:ros-info (gazebo-pm) "Camera '~a' now active"
                           (slot-value camera 'name)))
        (t (roslisp:ros-info (gazebo-pm) "Camera deactivated"))))

(defun set-tf-camera (tf-frame fov distance-limit)
  (set-camera
   (make-instance 'camera :name tf-frame
                          :tf-frame tf-frame
                          :fov fov
                          :distance-limit distance-limit)))

(defun active-camera ()
  *active-camera*)

(defun camera-pose (camera &key (fixed-frame "map"))
  (cond ((slot-value camera 'tf-frame)
         (tf:transform-pose-stamped
          cram-tf:*transformer*
          :pose (tf:pose->pose-stamped
                 (slot-value camera 'tf-frame) 0.0
                 (tf:make-identity-pose))
          :target-frame fixed-frame))
        ((slot-value camera 'fixed-pose)
         (slot-value camera 'fixed-pose))))

(defun object-names-equal (name-1 name-2)
  "In designators, we want to support symbols as names, too. It first
  converts all names that are symbols to strings and then compares
  them, ignoring case."
  (let ((name-1 (etypecase name-1
                  (symbol (symbol-name name-1))
                  (string name-1)))
        (name-2 (etypecase name-2
                  (symbol (symbol-name name-2))
                  (string name-2))))
    (string-equal name-1 name-2)))

(defun ignore-object (name &optional (ignore t))
  (if ignore
      (unless (is-object-ignored name)
        (push name *ignored-objects*))
      (setf *ignored-objects* (remove name *ignored-objects* :test #'string=))))

(defun is-object-ignored (name)
  (not (not (find name *ignored-objects* :test #'string=))))

(defun object-in-fov (object-pose camera-pose fov distance-limit)
  (labels ((scalar-product (v-1 v-2)
             (+ (* (tf:x v-1) (tf:x v-2))
                (* (tf:y v-1) (tf:y v-2))
                (* (tf:z v-1) (tf:z v-2))))
           (norm (v)
             (sqrt (+ (* (tf:x v) (tf:x v))
                      (* (tf:y v) (tf:y v))
                      (* (tf:z v) (tf:z v))))))
    (let* ((object-origin (tf:origin object-pose))
           (camera-origin (tf:origin camera-pose))
           (camera-orientation
             (tf:quaternion->matrix (tf:orientation camera-pose)))
           (view-direction
             (tf:make-3d-vector (aref camera-orientation 0 0)
                                (- (aref camera-orientation 0 1))
                                (aref camera-orientation 0 2)))
           (object-camera (tf:v- object-origin camera-origin))
           (angle (acos (/ (scalar-product view-direction object-camera)
                           (norm (tf:v* view-direction
                                        (norm object-camera)))))))
      (and (<= angle (/ fov 2))
           (<= (tf:v-dist camera-origin object-origin)
               distance-limit)))))

(defun object-visible (object-pose)
  (let ((camera (active-camera)))
    (cond (camera ;; There is an active camera, so visibility is checked
           (let ((camera-pose (camera-pose camera)))
             (object-in-fov
              object-pose
              camera-pose
              (slot-value camera 'fov)
              (slot-value camera 'distance-limit))))
          (t t))))
