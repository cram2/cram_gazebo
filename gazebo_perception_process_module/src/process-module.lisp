;;; Copyright (c) 2012, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :gazebo-perception-process-module)

; Generate logs - copied from the robosherlock perception module...
(cut:define-hook cram-language::on-prepare-perception-request (designator-request))
(cut:define-hook cram-language::on-finish-perception-request (log-id designators-result))

; The following stuff is copied from cram-task-knowledge since it is not yet available in cram2...
(define-hook objects-perceived (object-template object-designators))

(defparameter *tf-listener* nil)
(defun ensure-tf-listener ()
  (unless *tf-listener*
    (progn
      (setf *tf-listener* (make-instance 'cl-tf:transform-listener))
      (roslisp:wait-duration 1.0)))
  *tf-listener*)

(defun destroy-tf-listener ()
  (setf *tf-listener* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-tf-listener)


(defmethod objects-perceived (object-template object-designators)
  (declare (ignore object-template))
  ;; TODO: get a proper robot camera frame here, and avoid magic numbers.
  (let* ((camera-pose ;;(cl-transforms:make-transform (cl-transforms:make-3d-vector 0 0 8) (cl-transforms:make-quaternion 0 -0.72 0 0.72)))
                      (cl-transforms-stamped:lookup-transform (ensure-tf-listener) "/odom_combined" "/head_tilt_link" :timeout 2.0))
         (camera-fwd (cl-transforms-stamped:make-3d-vector 1 0 0))
         (camera-fwd (cl-transforms-stamped:rotate (cl-transforms-stamped:rotation camera-pose) camera-fwd))
         (camera-up (cl-transforms-stamped:make-3d-vector 0 0 1))
         (camera-up (cl-transforms-stamped:rotate (cl-transforms-stamped:rotation camera-pose) camera-up))
         (camera-pose (cl-transforms-stamped:translation camera-pose))
         (camera-pose (roslisp:make-message "geometry_msgs/Point"
                                            :x (cl-transforms-stamped:x camera-pose)
                                            :y (cl-transforms-stamped:y camera-pose)
                                            :z (cl-transforms-stamped:z camera-pose)))
         (camera-fwd (roslisp:make-message "geometry_msgs/Point"
                                           :x (cl-transforms-stamped:x camera-fwd)
                                           :y (cl-transforms-stamped:y camera-fwd)
                                           :z (cl-transforms-stamped:z camera-fwd)))
         (camera-up (roslisp:make-message "geometry_msgs/Point"
                                          :x (cl-transforms-stamped:x camera-up)
                                          :y (cl-transforms-stamped:y camera-up)
                                          :z (cl-transforms-stamped:z camera-up)))
         (focal-distance 1)
         (width 1)
         (height 1)
         (max-distance 12)
         (threshold 0.2))
    (remove-if #'null
               (mapcar (lambda (obj-desig)
                         (let* ((name (desig-prop-value obj-desig :name)))
                           (roslisp:with-fields (visible)
                                                (roslisp:call-service "/gazebo_visibility_ros/QueryGazeboVisibility" "gazebo_visibility_ros/QueryGazeboVisibility"
                                                                      :name name
                                                                      :camera_pose camera-pose
                                                                      :camera_fwd camera-fwd
                                                                      :camera_up camera-up
                                                                      :focal_distance focal-distance
                                                                      :width width
                                                                      :height height
                                                                      :max_distance max-distance
                                                                      :threshold threshold)
                             (format t "    ~a~%" visible)
                             (if (equal visible 0)
                               nil
                               obj-desig))))
                       object-designators))))


(defgeneric filter-perceived-objects (template-designator perceived-objects))
(defgeneric filter-perceived-objects (object-template perceived-objects)
  (:documentation "Filters all perceived objects according to all registered filters. This method is mainly used by perception process modules that want to validate and filter their results. Also, this function triggers the `object-perceived-event' plan event, updating the belief state.")
  (:method (object-template perceived-objects)
    (let* ((filtered-objects
             (loop for filter-result in (objects-perceived
                                         object-template perceived-objects)
                   append filter-result)))
      filtered-objects)))

; Start of the original module
(defmethod designator-pose ((designator object-designator))
  (object-pose (reference designator)))

(defmethod designator-distance ((designator-1 object-designator)
                                (designator-2 object-designator))
  (cl-transforms:v-dist (cl-transforms:origin (designator-pose designator-1))
                        (cl-transforms:origin (designator-pose designator-2))))

(defgeneric make-new-desig-description (old-desig perceived-object)
  (:documentation "Merges the description of `old-desig' with the
properties of `perceived-object'.")
  (:method ((old-desig object-designator)
            (perceived-object object-designator-data))
    (let ((obj-loc-desig (make-designator
                          :location
                          `((:pose ,(object-pose perceived-object)))))
          (object-name (or (when (desig-prop-value old-desig :name)
                             (desig-prop-value old-desig :name))
                           (object-identifier perceived-object))))
      `((:at ,obj-loc-desig)
        (:name ,object-name)
        ,@(remove-if (lambda (element)
                       (member element '(:at type :name)))
                     (description old-desig) :key #'car)))))

(defun make-handle-designator-sequence (handles)
  "Converts the sequence `handles' (handle-pose handle-radius) into a
sequence of object designators representing handle objects. Each
handle object then consist of a location designator describing its
relative position as well as the handle's radius for grasping
purposes."
  (mapcar (lambda (handle-desc)
            (destructuring-bind (pose radius) handle-desc
              `(handle
                ,(make-designator :object
                                  `((:at ,(make-designator
                                          :location `((:pose ,pose))))
                                    (:radius ,radius)
                                    (:type handle))))))
          handles))

(defun find-object (&key object-name object-type)
  "Finds objects based on either their name `object-name' or their
type `object-type', depending what is given. An invalid combination of
both parameters will result in an empty list. When no parameters are
given, all known objects from the knowledge base are returned."
  (cond (object-name
         (let* ((obj-symbol object-name)
                (model-pose (cram-gazebo-utilities:get-model-pose
                             object-name)))
           (when model-pose
             (list ( make-instance 'gazebo-designator-shape-data
                                  :object-identifier obj-symbol
                                  :pose model-pose)))))
        (object-type
         (loop for model-data in (cram-gazebo-utilities:get-models)
               as name = (car model-data)
               when (and (>= (length name) (length object-type))
                         (string= object-type (subseq name 0 (length object-type))))
                 collect
                 (make-instance 'gazebo-designator-shape-data
                                :object-identifier name
                                :pose (cdr model-data))))
        (t
         (mapcar (lambda (model-data)
                   (destructuring-bind (model-name . model-pose)
                       model-data
                     (make-instance 'gazebo-designator-shape-data
                                    :object-identifier model-name
                                    :pose model-pose)))
                 (cram-gazebo-utilities:get-models)))))

(defun perceived-object->designator (designator perceived-object)
  (make-effective-designator
   designator
   :new-properties (make-new-desig-description
                    designator perceived-object)
   :data-object perceived-object))

(defun find-with-designator (designator)
  (with-desig-props (desig-props::name desig-props::type) designator
    (let* ((at (desig-prop-value designator :at))
           (pose-in-at (desig-prop-value at :pose))
           (filter-function
             (cond ((and at (not pose-in-at))
                    (lambda (object-check)
                      (let* ((sample (reference at))
                             ;; This is a 2d comparison; put the z
                             ;; coordinate from the sample into the
                             ;; pose before validating. Otherwise,
                             ;; gravity will mess up everything.
                             (pose (desig-prop-value
                                    (desig-prop-value
                                     object-check
                                     :at)
                                    :pose))
                             (pose-elevated
                               (cl-transforms:copy-pose
                                pose
                                :origin (cl-transforms:make-3d-vector (cl-transforms:x (cl-transforms:origin pose))
                                                           (cl-transforms:y (cl-transforms:origin pose))
                                                           (cl-transforms:z (cl-transforms:origin sample))))))
                        (not (validate-location-designator-solution at pose-elevated)))))
                   (t #'not))))
      (remove-if
       filter-function
       (mapcar (lambda (perceived-object)
                 (perceived-object->designator
                  designator perceived-object))
               (find-object :object-name desig-props::name
                            :object-type desig-props::type))))))

(def-process-module gazebo-perception-process-module (input)
  (assert (typep input 'action-designator))
  (let* ((object-designator (desig-prop-value input :obj))
	 (log-id (first (cram-language::on-prepare-perception-request object-designator))))
    (ros-info (gazebo perception-process-module) "Searching for object ~a" object-designator)
    (let ((results (cpl:mapcar-clean
                    (lambda (designator)
                      (let ((name (desig-prop-value designator :name)))
                        (when (and name (not (is-object-ignored name)))
                          designator)))
                    (find-with-designator object-designator))))
      (cram-language::on-finish-perception-request log-id results)
      (if (not results) (cpl:fail 'cram-plan-failures:object-not-found :object-desig object-designator) results))))
