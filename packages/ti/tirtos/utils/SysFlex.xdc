/*
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 *  ======== SysFlex ========
 *  Generic implementation of ISystemSupport.
 *
 *  SysFlex allows the user to implement the five functions required for a
 *  system provider. The functions must follow the prototype type defined here and
 *  will be called when the corresponding system call is made.  If the config
 *  parameter is not set no function will be called and in the case of readyFxn
 *  SysFlex will always return that it is ready.  System providers can be called
 *  from any thread type and must not be blocking.
 */
module SysFlex inherits xdc.runtime.ISystemSupport {

    /*!
     *  ======== AbortFxn ========
     *  Function prototype for abort function
     *
     *  This function is called as part of the System_abort processing.
     *  This function should return to let System_abort continue the
     *  abort process.
     *
     *  @param(String)  String passed into the System_abort call
     */
    typedef Void (*AbortFxn)(CString);

    /*!
     *  ======== ExitFxn ========
     *  Function prototype for exit function
     *
     *  This function is called as part of the System_exit processing.
     *  This function should return to let System_exit continue the
     *  exit process.
     *
     *  @param(Int)  Int passed into the System_exit call
     */
    typedef Void (*ExitFxn)(Int);

    /*!
     *  ======== FlushFxn ========
     *  Function prototype for flush function
     *
     *  This function is called as part of the System_flush processing.
     */
    typedef Void (*FlushFxn)();

    /*!
     *  ======== PutchFxn ========
     *  Function prototype for putch function
     *
     *  This function is called when the System module wants to
     *  output a character.
     *
     *  @param(Char)  Character to be outputted
     */
    typedef Void (*PutchFxn)(Char);

    /*!
     *  ======== ReadyFxn ========
     *  Function prototype for ready function
     *
     *  Called at various times within the System module and is used
     *  to help improve performance (e.g. do not do complex string manipulations
     *  if the putchFxn is not ready). If the putchFxn is not ready to be called
     *  (e.g. have not initialized UART yet), the readyFxn should return FALSE.
     *  Once the putchFxn can be called, the readyFxn should return TRUE.
     *
     *  @b(returns)     TRUE is putchFxn is ready to be called
     *                  FALSE is putchFxn is not ready to be called
     */
    typedef Bool (*ReadyFxn)();

    /*!
     *  ======== abortFxn ========
     *  Abort function called by System
     */
    config AbortFxn abortFxn = null;

    /*!
     *  ======== exitFxn ========
     *  Exit function called by System
     */
    config ExitFxn exitFxn = null;

    /*!
     *  ======== flushFxn ========
     *  Flush function called by System
     */
    config FlushFxn flushFxn = null;

    /*!
     *  ======== putchFxn ========
     *  putch function called by System
     */
    config PutchFxn putchFxn = null;

    /*!
     *  ======== readyFxn ========
     *  Ready function called by System
     *
     *  If readyFxn is not configured and is equal to null SysFlex will always
     *  return ready.
     */
    config ReadyFxn readyFxn = null;
}
