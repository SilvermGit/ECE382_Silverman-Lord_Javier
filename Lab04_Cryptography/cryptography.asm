; cryptography.asm
; Runs on any Cortex M
; Basic XOR encryption and decryption functions for cryptography.
; Capt Steven Beyer
; July 23 2020
; Updated by Stan Baek
; June 03 2021
;
;Simplified BSD License (FreeBSD License)
;Copyright (c) 2020, Steven Beyer, All rights reserved.
;
;Redistribution and use in source and binary forms, with or without modification,
;are permitted provided that the following conditions are met:
;
;1. Redistributions of source code must retain the above copyright notice,
;   this list of conditions and the following disclaimer.
;2. Redistributions in binary form must reproduce the above copyright notice,
;   this list of conditions and the following disclaimer in the documentation
;   and/or other materials provided with the distribution.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
;USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;The views and conclusions contained in the software and documentation are
;those of the authors and should not be interpreted as representing official
;policies, either expressed or implied, of the FreeBSD Project.


       .thumb
       .text
       .align 2
       .global Encrypt
       .global Decrypt
       .global XOR_bytes

EOM    .byte  '#'


;------------ Encrypt ------------
; This subroutine takes the address of a plain text message and a key,
;   then encrypts the message by performing an XOR operation.
;   The encrypted result is stored at the specified memory location.
;   The encryption stops when EOM is encountered.
;
; Inputs:
;   R0 - Address of the plain text message
;   R1 - Key for XOR encryption
;   R2 - Address to store the encrypted message
;
; Return:
;   None
Encrypt:   .asmfunc
; ============ Add your code and comments below ===================

    ; Use EOM defined at line 44 for '#'
    PUSH {R0, R1, R2, R4, R5, R6, R7, R8, LR} ; Push important registers to prevent corruption
    MOV R4, R0 ; Make R4 the address of the current message so we can use R0 later
    MOV R5, R1 ; Make R5 the key so we can use R1 later
    MOV R6, R2 ; Make R6 the address of the future encrypted message so we can use R2 later
    LDRB R7, EOM ; Store EOM address in R7 to compare later
Loop1
    LDRB R0, [R4] ; Load the current byte at R4's address into R0 for use in the function
    LDRB R8, [R4] ; Load the current byte at R4's address into R8 for future use
    ; You must use the XOR_bytes function for exclusive or!
    BL XOR_bytes        ; XOR_Bytes
    MOV R1, R5 ; Make sure R1 has its original value after the function is called
    STRB R0, [R6] ; Store the current byte at R0 into the address of R6, the encrypted message
    CMP R8, R7 ; Check to see if the last character stored was the hashtag character
    BEQ Exit1   ; If so, exit program
    ADD R4, #1 ; Else, continue loop by going to the next character
    ADD R6, #1 ; Also go to the next storage spot for the encryption
    B Loop1 ; Continue looping
Exit1
    POP {R0, R1, R2, R4, R5, R6, R7, R8, LR} ; Take out important registers from memory
    BX LR ; Leave program



; =============== End of your code ================================
    .endasmfunc


;------------ Decrypt ------------
; This subroutine takes the address of an encrypted message and a key,
;   then decrypts the message by performing an XOR operation.
;   The decrypted result is stored at the specified memory location.
;   The decryption stops when the EOM is encountered.
;
; Inputs:
;   R0 - Address of the encrypted message
;   R1 - Key for XOR decryption
;   R2 - Address to store the decrypted message
;
; Return:
;   None
Decrypt:    .asmfunc
; ============ Add your code and comments below ===================

    ; Use EOM defined at line 44 for '#'
    PUSH {R0, R1, R2, R4, R5, R6, R7, R8, LR} ; Push important registers to prevent corruption again
    MOV R4, R0 ; Store the value at R0 to R4 so R0 can be used again later. R4 now holds the encrypted memory address
    MOV R5, R1 ; Same thing but for R1
    MOV R6, R2 ; Same thing but for R2, the decrypted memory address
    LDRB R7, EOM ; Load EOM address into R7 for future use
Loop2
    LDRB R0, [R4] ; Load current value at the address of R4 into R0
    ; You must use the XOR_bytes function for exclusive or!
    BL XOR_bytes        ; XOR_Bytes
    STRB R0, [R6] ; Store the decrypted value into the memory dress at R6, the decrypted key
    MOV R1, R5 ; Make sure that R1 has its original value after being corrupted
    LDRB R0, [R6] ; Load the current value at R6's address into R0 into see if it is the hashtag
    CMP R0, R7 ; Sees if R0 was a hashtag
    BEQ Exit2 ; If it is, exit program
    ADD R4, #1 ; If not, keep looping through the encrypted message
    ADD R6, #1 ; Also go to the next space in the decrypted memory
    B Loop2 ; Keep looping
Exit2
    POP {R0, R1, R2, R4, R5, R6, R7, R8, LR} ; Take out registers from memory with their original values
    BX LR ; Leave program


; =============== End of your code ================================
    .endasmfunc
    .end
