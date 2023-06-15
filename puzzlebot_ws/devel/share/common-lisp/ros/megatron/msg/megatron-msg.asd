
(cl:in-package :asdf)

(defsystem "megatron-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "values" :depends-on ("_package_values"))
    (:file "_package_values" :depends-on ("_package"))
  ))