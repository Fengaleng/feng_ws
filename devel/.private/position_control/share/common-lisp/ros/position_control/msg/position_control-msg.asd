
(cl:in-package :asdf)

(defsystem "position_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
  ))