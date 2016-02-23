;;
;; generic include file for dsPIC33F
;;

.ifdef __33FJ128GP706
.include "p33FJ128GP706.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ128GP708
.include "p33FJ128GP708.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ128GP710
.include "p33FJ128GP710.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ128MC506
.include "p33FJ128MC506.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ128MC706
.include "p33FJ128MC706.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ128MC708
.include "p33FJ128MC708.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ128MC710
.include "p33FJ128MC710.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ256GP506
.include "p33FJ256GP506.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ256GP710
.include "p33FJ256GP710.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ256MC710
.include "p33FJ256MC710.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64GP206
.include "p33FJ64GP206.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64GP310
.include "p33FJ64GP310.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64GP706
.include "p33FJ64GP706.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64GP708
.include "p33FJ64GP708.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64GP710
.include "p33FJ64GP710.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64MC506
.include "p33FJ64MC506.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64MC508
.include "p33FJ64MC508.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64MC510
.include "p33FJ64MC510.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64MC706
.include "p33FJ64MC706.inc"
.equ VALID_ID,1
.endif

.ifdef __33FJ64MC710
.include "p33FJ64MC710.inc"
.equ VALID_ID,1
.endif

.ifndef VALID_ID
.error "processor ID not specified in generic include file"
.endif
