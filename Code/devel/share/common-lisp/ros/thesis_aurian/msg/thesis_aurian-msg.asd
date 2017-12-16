
(cl:in-package :asdf)

(defsystem "thesis_aurian-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "refdoors" :depends-on ("_package_refdoors"))
    (:file "_package_refdoors" :depends-on ("_package"))
  ))